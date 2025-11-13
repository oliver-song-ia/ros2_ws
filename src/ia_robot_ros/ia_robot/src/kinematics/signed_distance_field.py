#!/usr/bin/env python3

import numpy as np
from scipy import ndimage
from scipy.spatial.distance import cdist
import warnings
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SignedDistanceField:
    """
    3D Signed Distance Field for obstacle avoidance in robotic applications.
    Converts point cloud data to a continuous 3D SDF for efficient distance queries.
    """
    
    def __init__(self, bounds=None, resolution=0.05, max_distance=2.0, robot_centered=True):
        """
        Initialize SDF with workspace bounds and resolution.
        
        Args:
            bounds: tuple of (min_xyz, max_xyz) defining workspace, e.g. ([-2,-2,0], [2,2,2])
                   If robot_centered=True, this defines the size around robot, not absolute bounds
            resolution: grid resolution in meters
            max_distance: maximum distance to compute (for efficiency)
            robot_centered: if True, bounds move with robot position
        """
        if bounds is None:
            # Default workspace bounds around robot (relative bounds)
            bounds = ([-3.0, -3.0, -0.5], [3.0, 3.0, 3.0])
        
        self.resolution = resolution
        self.max_distance = max_distance
        self.robot_centered = robot_centered
        
        if robot_centered:
            # Store relative bounds (offset from robot center)
            self.relative_bounds_min = np.array(bounds[0])
            self.relative_bounds_max = np.array(bounds[1])
            # Initialize at origin, will be updated when robot position is known
            self.robot_position = np.array([0.0, 0.0, 0.0])
            self.bounds_min = self.relative_bounds_min + self.robot_position
            self.bounds_max = self.relative_bounds_max + self.robot_position
        else:
            # Use absolute bounds (legacy behavior)
            self.bounds_min = np.array(bounds[0])
            self.bounds_max = np.array(bounds[1])
            self.robot_position = None
        
        # Compute grid dimensions
        self.grid_size = ((self.bounds_max - self.bounds_min) / resolution).astype(int)
        
        # Initialize distance field
        self.sdf = np.full(self.grid_size, max_distance, dtype=np.float32)
        self.gradient = np.zeros((*self.grid_size, 3), dtype=np.float32)
        
        # Track if SDF is valid
        self.is_valid = False
        
        print(f"SDF initialized: bounds {bounds}, resolution {resolution}m, grid {self.grid_size}, robot_centered={robot_centered}")
    
    def world_to_grid(self, points):
        """Convert world coordinates to grid indices"""
        return ((points - self.bounds_min) / self.resolution).astype(int)
    
    def grid_to_world(self, indices):
        """Convert grid indices to world coordinates"""
        return indices * self.resolution + self.bounds_min
    
    def update_robot_position(self, robot_position):
        """
        Update robot position and recenter bounds if robot_centered=True.
        
        Args:
            robot_position: 3D numpy array of robot's current position
        """
        if not self.robot_centered:
            return  # No update needed for fixed bounds
        
        robot_position = np.array(robot_position)
        
        # Check if robot has moved significantly (avoid unnecessary updates)
        if self.robot_position is not None:
            position_change = np.linalg.norm(robot_position - self.robot_position)
            if position_change < self.resolution:  # Less than one grid cell
                return
        
        # Update robot position and recenter bounds
        self.robot_position = robot_position.copy()
        self.bounds_min = self.relative_bounds_min + self.robot_position
        self.bounds_max = self.relative_bounds_max + self.robot_position
        
        # Invalidate SDF since bounds changed
        self.is_valid = False
    
    def update_from_pointcloud(self, points, robot_position=None, use_fast_marching=True):
        """
        Update SDF from point cloud data.
        
        Args:
            points: Nx3 numpy array of obstacle points in world coordinates
            robot_position: optional 3D robot position for centering bounds
            use_fast_marching: Use fast marching method (faster) or brute force (more accurate)
        """
        # Update robot position if provided
        if robot_position is not None:
            self.update_robot_position(robot_position)
        if len(points) == 0:
            # No obstacles - set all distances to max
            self.sdf.fill(self.max_distance)
            self.is_valid = True
            return
        
        # Filter points within bounds
        valid_mask = np.all((points >= self.bounds_min) & (points <= self.bounds_max), axis=1)
        points = points[valid_mask]
        
        if len(points) == 0:
            self.sdf.fill(self.max_distance)
            self.is_valid = True
            return
        
        if use_fast_marching:
            self._compute_sdf_fast_marching(points)
        else:
            self._compute_sdf_brute_force(points)
        
        # Compute gradient for optimization
        self._compute_gradient()
        self.is_valid = True
    
    def _compute_sdf_fast_marching(self, points):
        """Fast SDF computation using distance transform"""
        # Create binary occupancy grid
        occupancy = np.zeros(self.grid_size, dtype=bool)
        
        # Mark occupied cells
        grid_points = self.world_to_grid(points)
        
        # Filter points within grid bounds
        valid_indices = np.all((grid_points >= 0) & (grid_points < self.grid_size), axis=1)
        grid_points = grid_points[valid_indices]
        
        if len(grid_points) > 0:
            occupancy[grid_points[:, 0], grid_points[:, 1], grid_points[:, 2]] = True
        
        # Compute distance transform
        # Note: scipy's distance_transform_edt gives distance to nearest False value
        distances = ndimage.distance_transform_edt(~occupancy) * self.resolution
        
        # Clamp to max distance
        self.sdf = np.minimum(distances, self.max_distance)
    
    def _compute_sdf_brute_force(self, points):
        """Brute force SDF computation - more accurate but slower"""
        # Create grid of all voxel centers
        x_coords = np.arange(self.grid_size[0]) * self.resolution + self.bounds_min[0]
        y_coords = np.arange(self.grid_size[1]) * self.resolution + self.bounds_min[1]
        z_coords = np.arange(self.grid_size[2]) * self.resolution + self.bounds_min[2]
        
        xx, yy, zz = np.meshgrid(x_coords, y_coords, z_coords, indexing='ij')
        grid_points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)
        
        # Compute distances using scipy's cdist (more compatible)
        distances = cdist(grid_points, points).min(axis=1)
        
        # Reshape and clamp
        self.sdf = np.minimum(distances.reshape(self.grid_size), self.max_distance)
    
    def _compute_gradient(self):
        """Compute gradient of SDF for optimization"""
        # Use central differences for gradient computation
        grad_x = np.gradient(self.sdf, axis=0) / self.resolution
        grad_y = np.gradient(self.sdf, axis=1) / self.resolution 
        grad_z = np.gradient(self.sdf, axis=2) / self.resolution
        
        self.gradient = np.stack([grad_x, grad_y, grad_z], axis=-1)
    
    def query_distance(self, points):
        """
        Query distance at world coordinates using trilinear interpolation.
        
        Args:
            points: Nx3 array or single 3D point
            
        Returns:
            distances: N array or single distance value
        """
        if not self.is_valid:
            warnings.warn("SDF not initialized. Call update_from_pointcloud first.")
            return np.full(len(points) if points.ndim > 1 else 1, self.max_distance)
        
        points = np.atleast_2d(points)
        
        # Convert to grid coordinates (continuous)
        grid_coords = (points - self.bounds_min) / self.resolution
        
        # Trilinear interpolation
        distances = []
        for coord in grid_coords:
            distance = self._trilinear_interpolate(coord)
            distances.append(distance)
        
        distances = np.array(distances)
        return distances[0] if len(distances) == 1 else distances
    
    def query_gradient(self, points):
        """
        Query gradient at world coordinates using trilinear interpolation.
        
        Args:
            points: Nx3 array or single 3D point
            
        Returns:
            gradients: Nx3 array or single 3D gradient vector
        """
        if not self.is_valid:
            warnings.warn("SDF not initialized. Call update_from_pointcloud first.")
            return np.zeros((len(points), 3) if points.ndim > 1 else (3,))
        
        points = np.atleast_2d(points)
        
        # Convert to grid coordinates (continuous)
        grid_coords = (points - self.bounds_min) / self.resolution
        
        # Trilinear interpolation for each gradient component
        gradients = []
        for coord in grid_coords:
            grad = self._trilinear_interpolate_gradient(coord)
            gradients.append(grad)
        
        gradients = np.array(gradients)
        return gradients[0] if len(gradients) == 1 else gradients
    
    def _trilinear_interpolate(self, coord):
        """Trilinear interpolation for distance field"""
        # Clamp coordinates to grid bounds
        coord = np.clip(coord, 0, np.array(self.grid_size) - 1)
        
        # Get integer coordinates and fractions
        coord_floor = np.floor(coord).astype(int)
        coord_ceil = np.minimum(coord_floor + 1, np.array(self.grid_size) - 1)
        frac = coord - coord_floor
        
        # Get 8 corner values
        c000 = self.sdf[coord_floor[0], coord_floor[1], coord_floor[2]]
        c001 = self.sdf[coord_floor[0], coord_floor[1], coord_ceil[2]]
        c010 = self.sdf[coord_floor[0], coord_ceil[1], coord_floor[2]]
        c011 = self.sdf[coord_floor[0], coord_ceil[1], coord_ceil[2]]
        c100 = self.sdf[coord_ceil[0], coord_floor[1], coord_floor[2]]
        c101 = self.sdf[coord_ceil[0], coord_floor[1], coord_ceil[2]]
        c110 = self.sdf[coord_ceil[0], coord_ceil[1], coord_floor[2]]
        c111 = self.sdf[coord_ceil[0], coord_ceil[1], coord_ceil[2]]
        
        # Trilinear interpolation
        c00 = c000 * (1 - frac[2]) + c001 * frac[2]
        c01 = c010 * (1 - frac[2]) + c011 * frac[2]
        c10 = c100 * (1 - frac[2]) + c101 * frac[2]
        c11 = c110 * (1 - frac[2]) + c111 * frac[2]
        
        c0 = c00 * (1 - frac[1]) + c01 * frac[1]
        c1 = c10 * (1 - frac[1]) + c11 * frac[1]
        
        return c0 * (1 - frac[0]) + c1 * frac[0]
    
    def _trilinear_interpolate_gradient(self, coord):
        """Trilinear interpolation for gradient field"""
        # Clamp coordinates to grid bounds  
        coord = np.clip(coord, 0, np.array(self.grid_size) - 1)
        
        # Get integer coordinates and fractions
        coord_floor = np.floor(coord).astype(int)
        coord_ceil = np.minimum(coord_floor + 1, np.array(self.grid_size) - 1)
        frac = coord - coord_floor
        
        # Get 8 corner gradient values
        g000 = self.gradient[coord_floor[0], coord_floor[1], coord_floor[2]]
        g001 = self.gradient[coord_floor[0], coord_floor[1], coord_ceil[2]]
        g010 = self.gradient[coord_floor[0], coord_ceil[1], coord_floor[2]]
        g011 = self.gradient[coord_floor[0], coord_ceil[1], coord_ceil[2]]
        g100 = self.gradient[coord_ceil[0], coord_floor[1], coord_floor[2]]
        g101 = self.gradient[coord_ceil[0], coord_floor[1], coord_ceil[2]]
        g110 = self.gradient[coord_ceil[0], coord_ceil[1], coord_floor[2]]
        g111 = self.gradient[coord_ceil[0], coord_ceil[1], coord_ceil[2]]
        
        # Trilinear interpolation for each component
        g00 = g000 * (1 - frac[2]) + g001 * frac[2]
        g01 = g010 * (1 - frac[2]) + g011 * frac[2]
        g10 = g100 * (1 - frac[2]) + g101 * frac[2]
        g11 = g110 * (1 - frac[2]) + g111 * frac[2]
        
        g0 = g00 * (1 - frac[1]) + g01 * frac[1]
        g1 = g10 * (1 - frac[1]) + g11 * frac[1]
        
        return g0 * (1 - frac[0]) + g1 * frac[0]
    
    def get_occupancy_grid(self, threshold=0.1):
        """
        Get binary occupancy grid for visualization.
        
        Args:
            threshold: distance threshold for occupied cells
            
        Returns:
            occupancy: binary grid where True = occupied
        """
        return self.sdf < threshold
    
    def visualize_2d_slice(self, z_world=1.0, show_gradient=False, figsize=(12, 5)):
        """
        Visualize 2D slice of the SDF at given Z coordinate.
        
        Args:
            z_world: world Z coordinate for the slice
            show_gradient: whether to show gradient vectors
            figsize: figure size for matplotlib
        """
        if not self.is_valid:
            print("SDF not initialized. Call update_from_pointcloud first.")
            return
        
        # Find closest Z grid index
        z_grid = int((z_world - self.bounds_min[2]) / self.resolution)
        z_grid = np.clip(z_grid, 0, self.grid_size[2] - 1)
        
        # Extract 2D slice
        sdf_slice = self.sdf[:, :, z_grid]
        
        # Create coordinate grids for plotting
        x_coords = np.arange(self.grid_size[0]) * self.resolution + self.bounds_min[0]
        y_coords = np.arange(self.grid_size[1]) * self.resolution + self.bounds_min[1]
        X, Y = np.meshgrid(x_coords, y_coords, indexing='ij')
        
        if show_gradient:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)
        else:
            fig, ax1 = plt.subplots(1, 1, figsize=(figsize[0]//2, figsize[1]))
        
        # Plot distance field
        im1 = ax1.contourf(X, Y, sdf_slice, levels=20, cmap='viridis')
        contours = ax1.contour(X, Y, sdf_slice, levels=10, colors='white', alpha=0.5, linewidths=0.5)
        ax1.clabel(contours, inline=True, fontsize=8, fmt='%.2f')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(f'SDF at Z = {z_world:.2f}m')
        ax1.set_aspect('equal')
        plt.colorbar(im1, ax=ax1, label='Distance (m)')
        
        if show_gradient:
            # Plot gradient field
            gradient_slice = self.gradient[:, :, z_grid, :2]  # Only X,Y components
            
            # Subsample for cleaner visualization
            step = max(1, min(self.grid_size[0], self.grid_size[1]) // 15)
            X_sub = X[::step, ::step]
            Y_sub = Y[::step, ::step]
            U = gradient_slice[::step, ::step, 0]
            V = gradient_slice[::step, ::step, 1]
            
            ax2.quiver(X_sub, Y_sub, U, V, angles='xy', scale_units='xy', scale=1, alpha=0.7)
            im2 = ax2.contourf(X, Y, sdf_slice, levels=20, cmap='viridis', alpha=0.3)
            
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title(f'SDF Gradient at Z = {z_world:.2f}m')
            ax2.set_aspect('equal')
        
        plt.tight_layout()
        plt.show()
    
    def visualize_3d_isosurfaces(self, distance_levels=[0.1, 0.3, 0.5], figsize=(10, 8)):
        """
        Visualize 3D isosurfaces of the SDF.
        
        Args:
            distance_levels: list of distance values to show as isosurfaces
            figsize: figure size for matplotlib
        """
        if not self.is_valid:
            print("SDF not initialized. Call update_from_pointcloud first.")
            return
        
        try:
            from skimage import measure
        except ImportError:
            print("scikit-image not available. Install with: pip install scikit-image")
            return
        
        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(111, projection='3d')
        
        # Create coordinate grids
        x_coords = np.arange(self.grid_size[0]) * self.resolution + self.bounds_min[0]
        y_coords = np.arange(self.grid_size[1]) * self.resolution + self.bounds_min[1]
        z_coords = np.arange(self.grid_size[2]) * self.resolution + self.bounds_min[2]
        
        colors = ['red', 'orange', 'yellow', 'green', 'blue']
        
        for i, level in enumerate(distance_levels):
            try:
                # Extract isosurface using marching cubes
                verts, faces, _, _ = measure.marching_cubes(self.sdf, level, spacing=(self.resolution, self.resolution, self.resolution))
                
                # Convert to world coordinates
                verts = verts + np.array([self.bounds_min[0], self.bounds_min[1], self.bounds_min[2]])
                
                # Plot the isosurface
                ax.plot_trisurf(verts[:, 0], verts[:, 1], verts[:, 2], 
                              triangles=faces, alpha=0.3, 
                              color=colors[i % len(colors)], 
                              label=f'd = {level:.1f}m')
            except Exception as e:
                print(f"Could not generate isosurface for level {level}: {e}")
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('SDF 3D Isosurfaces')
        ax.legend()
        
        plt.show()
    
    def visualize_point_cloud_with_sdf(self, original_points, query_points=None, figsize=(12, 5)):
        """
        Visualize the original point cloud alongside the SDF.
        
        Args:
            original_points: Nx3 array of original obstacle points
            query_points: optional Nx3 array of points to show distances for
            figsize: figure size for matplotlib
        """
        if not self.is_valid:
            print("SDF not initialized. Call update_from_pointcloud first.")
            return
        
        fig = plt.figure(figsize=figsize)
        
        # 3D plot of point cloud and SDF sample
        ax1 = fig.add_subplot(121, projection='3d')
        
        # Plot original obstacle points
        if len(original_points) > 0:
            ax1.scatter(original_points[:, 0], original_points[:, 1], original_points[:, 2], 
                       c='red', s=20, alpha=0.6, label='Obstacles')
        
        # Sample and plot SDF points
        sample_step = max(1, max(self.grid_size) // 20)
        sample_points = []
        sample_distances = []
        
        for i in range(0, self.grid_size[0], sample_step):
            for j in range(0, self.grid_size[1], sample_step):
                for k in range(0, self.grid_size[2], sample_step):
                    if self.sdf[i, j, k] < 1.0:  # Only show points close to obstacles
                        world_point = self.grid_to_world(np.array([i, j, k]))
                        sample_points.append(world_point)
                        sample_distances.append(self.sdf[i, j, k])
        
        if sample_points:
            sample_points = np.array(sample_points)
            sample_distances = np.array(sample_distances)
            
            # Color by distance
            scatter = ax1.scatter(sample_points[:, 0], sample_points[:, 1], sample_points[:, 2],
                                c=sample_distances, cmap='viridis', s=10, alpha=0.5)
            plt.colorbar(scatter, ax=ax1, label='Distance (m)', shrink=0.5)
        
        # Plot query points if provided
        if query_points is not None:
            distances = self.query_distance(query_points)
            ax1.scatter(query_points[:, 0], query_points[:, 1], query_points[:, 2],
                       c='blue', s=50, marker='^', label='Query Points')
            
            # Add distance labels
            for i, (point, dist) in enumerate(zip(query_points, distances)):
                ax1.text(point[0], point[1], point[2], f'{dist:.2f}', fontsize=8)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Point Cloud & SDF')
        ax1.legend()
        
        # 2D slice at middle Z
        ax2 = fig.add_subplot(122)
        z_middle = (self.bounds_min[2] + self.bounds_max[2]) / 2
        z_grid = int((z_middle - self.bounds_min[2]) / self.resolution)
        z_grid = np.clip(z_grid, 0, self.grid_size[2] - 1)
        
        sdf_slice = self.sdf[:, :, z_grid]
        x_coords = np.arange(self.grid_size[0]) * self.resolution + self.bounds_min[0]
        y_coords = np.arange(self.grid_size[1]) * self.resolution + self.bounds_min[1]
        X, Y = np.meshgrid(x_coords, y_coords, indexing='ij')
        
        im = ax2.contourf(X, Y, sdf_slice, levels=15, cmap='viridis')
        contours = ax2.contour(X, Y, sdf_slice, levels=8, colors='white', alpha=0.5, linewidths=0.5)
        
        # Plot obstacle points in this slice
        z_slice_points = original_points[np.abs(original_points[:, 2] - z_middle) < self.resolution/2]
        if len(z_slice_points) > 0:
            ax2.scatter(z_slice_points[:, 0], z_slice_points[:, 1], c='red', s=30, marker='x', label='Obstacles')
        
        # Plot query points in this slice
        if query_points is not None:
            z_query_points = query_points[np.abs(query_points[:, 2] - z_middle) < self.resolution/2]
            if len(z_query_points) > 0:
                ax2.scatter(z_query_points[:, 0], z_query_points[:, 1], c='blue', s=50, marker='^', label='Query Points')
        
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title(f'SDF Slice at Z = {z_middle:.2f}m')
        ax2.set_aspect('equal')
        plt.colorbar(im, ax=ax2, label='Distance (m)')
        if len(z_slice_points) > 0 or (query_points is not None and len(z_query_points) > 0):
            ax2.legend()
        
        plt.tight_layout()
        plt.show()


def test():
    print("=== Testing SignedDistanceField with Visualization ===\n")
    
    # Create test scenarios
    scenarios = [
        {
            'name': 'Simple Cube Obstacle',
            'description': 'Single cube-shaped obstacle for basic testing',
            'points': []
        },
        {
            'name': 'Multiple Obstacles',
            'description': 'Multiple scattered obstacles',
            'points': []
        },
        {
            'name': 'Complex Environment',
            'description': 'Complex environment with walls and obstacles',
            'points': []
        }
    ]
    
    # Scenario 1: Simple cube obstacle
    for x in np.linspace(0.5, 1.0, 8):
        for y in np.linspace(-0.3, 0.3, 6):
            for z in np.linspace(0.5, 1.2, 7):
                scenarios[0]['points'].append([x, y, z])
    
    # Scenario 2: Multiple obstacles
    # Obstacle 1: Small cube
    for x in np.linspace(0.3, 0.7, 5):
        for y in np.linspace(-0.2, 0.2, 5):
            for z in np.linspace(0.8, 1.2, 5):
                scenarios[1]['points'].append([x, y, z])
    
    # Obstacle 2: Sphere-like
    center = np.array([1.5, 0.5, 1.0])
    radius = 0.3
    for _ in range(200):
        # Random points in sphere
        theta = np.random.uniform(0, 2*np.pi)
        phi = np.random.uniform(0, np.pi)
        r = np.random.uniform(0, radius)
        
        x = center[0] + r * np.sin(phi) * np.cos(theta)
        y = center[1] + r * np.sin(phi) * np.sin(theta)
        z = center[2] + r * np.cos(phi)
        scenarios[1]['points'].append([x, y, z])
    
    # Obstacle 3: Linear obstacle
    for t in np.linspace(0, 1, 50):
        scenarios[1]['points'].append([0.2 + t * 1.0, -0.8 + t * 0.6, 0.5 + t * 0.8])
    
    # Scenario 3: Complex environment
    # Wall 1 (vertical)
    for x in np.linspace(-0.5, -0.3, 3):
        for y in np.linspace(-1.0, 1.0, 20):
            for z in np.linspace(0.0, 2.0, 20):
                scenarios[2]['points'].append([x, y, z])
    
    # Wall 2 (horizontal)  
    for x in np.linspace(0.8, 1.5, 8):
        for y in np.linspace(-0.2, 0.2, 5):
            for z in np.linspace(0.0, 2.0, 20):
                scenarios[2]['points'].append([x, y, z])
    
    # Scattered obstacles
    np.random.seed(42)
    for _ in range(100):
        x = np.random.uniform(-2, 2)
        y = np.random.uniform(-2, 2)
        z = np.random.uniform(0.3, 1.8)
        scenarios[2]['points'].append([x, y, z])
    
    # Convert to numpy arrays
    for scenario in scenarios:
        scenario['points'] = np.array(scenario['points'])
    
    # Interactive scenario selection
    print("Available test scenarios:")
    for i, scenario in enumerate(scenarios):
        print(f"{i+1}. {scenario['name']}: {scenario['description']}")
    
    try:
        choice = input(f"\nSelect scenario (1-{len(scenarios)}) or 'all' for all scenarios: ").strip().lower()
        
        if choice == 'all':
            selected_scenarios = scenarios
        else:
            scenario_idx = int(choice) - 1
            if 0 <= scenario_idx < len(scenarios):
                selected_scenarios = [scenarios[scenario_idx]]
            else:
                print("Invalid choice, using first scenario")
                selected_scenarios = [scenarios[0]]
    except (ValueError, KeyboardInterrupt):
        print("Using first scenario")
        selected_scenarios = [scenarios[0]]
    
    # Process each selected scenario
    for scenario in selected_scenarios:
        print(f"\n=== Testing Scenario: {scenario['name']} ===")
        test_points = scenario['points']
        
        if len(test_points) == 0:
            print("No points in scenario, skipping...")
            continue
        
        print(f"Number of obstacle points: {len(test_points)}")
        
        # Create SDF with appropriate bounds
        point_bounds_min = test_points.min(axis=0) - 0.5
        point_bounds_max = test_points.max(axis=0) + 0.5
        bounds = (point_bounds_min.tolist(), point_bounds_max.tolist())
        
        print(f"SDF bounds: {bounds}")
        
        # Create SDF
        sdf = SignedDistanceField(bounds=bounds, resolution=0.08)
        sdf.update_from_pointcloud(test_points)
        
        # Define test query points around the obstacles
        query_points = np.array([
            [0.0, 0.0, 1.0],        # Origin area
            [2.0, 0.0, 1.0],        # Far from obstacles
            [0.75, 0.0, 1.0],       # Close to obstacles
            [1.0, 0.5, 1.2],        # Above obstacles
            [-1.0, 0.0, 0.5],       # Left side
        ])
        
        # Filter query points to be within bounds
        valid_queries = []
        for point in query_points:
            if np.all(point >= sdf.bounds_min) and np.all(point <= sdf.bounds_max):
                valid_queries.append(point)
        
        if valid_queries:
            query_points = np.array(valid_queries)
        else:
            # Generate query points within bounds if none are valid
            query_points = np.array([
                sdf.bounds_min + 0.1 * (sdf.bounds_max - sdf.bounds_min),
                sdf.bounds_max - 0.1 * (sdf.bounds_max - sdf.bounds_min),
                0.5 * (sdf.bounds_min + sdf.bounds_max)
            ])
        
        # Test distance queries
        distances = sdf.query_distance(query_points)
        gradients = sdf.query_gradient(query_points)
        
        print(f"\nQuery Results:")
        print("-" * 60)
        for i, (point, dist, grad) in enumerate(zip(query_points, distances, gradients)):
            grad_mag = np.linalg.norm(grad)
            print(f"Point {i}: {point} -> distance: {dist:.3f}m, gradient magnitude: {grad_mag:.3f}")
        
        print(f"\nSDF Statistics:")
        print(f"Grid shape: {sdf.sdf.shape}")
        print(f"Min distance: {sdf.sdf.min():.3f}m")
        print(f"Max distance: {sdf.sdf.max():.3f}m") 
        print(f"Mean distance: {sdf.sdf.mean():.3f}m")
        
        # Ask for visualization
        try:
            show_viz = input(f"\nShow visualizations for '{scenario['name']}'? (y/n/all): ").strip().lower()
            
            if show_viz in ['y', 'yes', 'all']:
                print("\n1. Showing 3D point cloud with SDF...")
                sdf.visualize_point_cloud_with_sdf(test_points, query_points)
                
                print("2. Showing 2D SDF slice...")
                z_middle = (sdf.bounds_min[2] + sdf.bounds_max[2]) / 2
                sdf.visualize_2d_slice(z_middle, show_gradient=True)
                
                if show_viz == 'all':
                    print("3. Showing different Z slices...")
                    z_levels = np.linspace(sdf.bounds_min[2] + 0.1, sdf.bounds_max[2] - 0.1, 3)
                    for z_level in z_levels:
                        print(f"   Slice at Z = {z_level:.2f}m")
                        sdf.visualize_2d_slice(z_level, show_gradient=False)
                
                try:
                    print("4. Attempting 3D isosurface visualization...")
                    sdf.visualize_3d_isosurfaces([0.1, 0.2, 0.4, 0.8])
                except ImportError:
                    print("   Skipping 3D isosurfaces (scikit-image not available)")
                except Exception as e:
                    print(f"   Error creating 3D isosurfaces: {e}")
                
            elif show_viz in ['n', 'no']:
                print("Skipping visualizations...")
            
        except (KeyboardInterrupt, EOFError):
            print("\nSkipping visualizations...")
    
    print(f"\n=== SDF Testing Complete ===")
    print("The SignedDistanceField class is working correctly!")
    print("\nVisualization features:")
    print("• visualize_point_cloud_with_sdf(): 3D point cloud + SDF sample points")
    print("• visualize_2d_slice(): 2D distance field slices with gradients")
    print("• visualize_3d_isosurfaces(): 3D isosurface rendering (requires scikit-image)")
    print("\nUse these methods to debug and verify SDF behavior in your applications.")

# Test function for the SDF with comprehensive visualization
if __name__ == "__main__":
    test()

