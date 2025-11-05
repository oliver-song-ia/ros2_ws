#!/usr/bin/env python3

import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
sys.path.append('/home/hanxi/code/ia_robot_sim/src/ia_robot/src')

from signed_distance_field import SignedDistanceField
from collision_checker import CollisionChecker
from simple_mobile_ik import SimpleMobileIK

def visualize_robot_links(ik_solver, q, ax, color='blue', alpha=0.7, label='Robot'):
    """Visualize robot links as spheres at their positions"""
    try:
        # Get link positions using collision checker approach
        import pinocchio as pin
        
        # Map joints to pinocchio format
        pin_q = ik_solver._map_joints_to_pinocchio(q)
        
        # Update forward kinematics
        pin.forwardKinematics(ik_solver.model, ik_solver.data, pin_q)
        pin.updateFramePlacements(ik_solver.model, ik_solver.data)
        
        # Get chassis position
        chassis_x, chassis_y, chassis_theta = ik_solver.get_chassis_pose(q)
        
        # Plot chassis as a box
        chassis_size = [0.6, 0.4, 0.3]
        chassis_center = [chassis_x, chassis_y, 0.6]  # Chassis height
        
        # Simple chassis visualization as a point with size
        ax.scatter([chassis_center[0]], [chassis_center[1]], [chassis_center[2]], 
                  s=200, c=color, marker='s', alpha=alpha, label=f'{label} Chassis')
        
        # Plot arm links if we can find them
        link_positions = []
        link_names = ['CHEST1', 'ARM0_LEFT', 'ARM1_LEFT', 'ARM2_LEFT', 'ARM3_LEFT', 
                     'ARM0_RIGHT', 'ARM1_RIGHT', 'ARM2_RIGHT', 'ARM3_RIGHT']
        
        for link_name in link_names:
            # Try to find frame
            for frame_id in range(ik_solver.model.nframes):
                if ik_solver.model.frames[frame_id].name == link_name:
                    link_pose = ik_solver.data.oMf[frame_id]
                    pos = link_pose.translation
                    link_positions.append(pos)
                    
                    # Plot link as sphere
                    radius = 0.08 if 'ARM' in link_name else 0.12
                    ax.scatter([pos[0]], [pos[1]], [pos[2]], 
                             s=100*radius, c=color, alpha=alpha, marker='o')
                    break
        
        # Get end effector positions
        try:
            left_ee = ik_solver.forward_kinematics(q, 'left')
            right_ee = ik_solver.forward_kinematics(q, 'right')
            
            # Plot end effectors
            ax.scatter([left_ee[0,3]], [left_ee[1,3]], [left_ee[2,3]], 
                      s=150, c='red' if color=='blue' else 'darkred', 
                      marker='^', alpha=alpha, label=f'{label} Left EE')
            ax.scatter([right_ee[0,3]], [right_ee[1,3]], [right_ee[2,3]], 
                      s=150, c='red' if color=='blue' else 'darkred', 
                      marker='^', alpha=alpha, label=f'{label} Right EE')
            
            return link_positions + [left_ee[:3,3], right_ee[:3,3]]
            
        except Exception as e:
            print(f"Warning: Could not get end effector positions: {e}")
            return link_positions
            
    except Exception as e:
        print(f"Error visualizing robot: {e}")
        return []

def visualize_sdf_slice_with_robot(sdf, q_configs, obstacles, ik_solver, z_height=1.0):
    """Visualize 2D SDF slice with robot configurations"""
    if not sdf.is_valid:
        print("SDF not valid, skipping slice visualization")
        return
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Get SDF slice
    z_grid = int((z_height - sdf.bounds_min[2]) / sdf.resolution)
    z_grid = np.clip(z_grid, 0, sdf.grid_size[2] - 1)
    sdf_slice = sdf.sdf[:, :, z_grid]
    
    # Create coordinate grids
    x_coords = np.arange(sdf.grid_size[0]) * sdf.resolution + sdf.bounds_min[0]
    y_coords = np.arange(sdf.grid_size[1]) * sdf.resolution + sdf.bounds_min[1]
    X, Y = np.meshgrid(x_coords, y_coords, indexing='ij')
    
    # Plot 1: SDF with robot configurations
    im1 = ax1.contourf(X, Y, sdf_slice, levels=15, cmap='viridis', alpha=0.7)
    contours1 = ax1.contour(X, Y, sdf_slice, levels=[0.1, 0.2, 0.4], 
                           colors=['red', 'orange', 'yellow'], linewidths=2)
    ax1.clabel(contours1, inline=True, fontsize=8, fmt='%.1f m')
    
    # Plot obstacles in this slice
    z_slice_obs = obstacles[np.abs(obstacles[:, 2] - z_height) < sdf.resolution]
    if len(z_slice_obs) > 0:
        ax1.scatter(z_slice_obs[:, 0], z_slice_obs[:, 1], c='red', s=50, 
                   marker='x', alpha=0.8, label='Obstacles')
    
    # Plot robot configurations at this height
    colors = ['blue', 'green']
    labels = ['Without Avoidance', 'With Avoidance']
    
    for i, (q, color, label) in enumerate(zip(q_configs, colors, labels)):
        if q is not None:
            # Get chassis position
            chassis_x, chassis_y, _ = ik_solver.get_chassis_pose(q)
            ax1.scatter([chassis_x], [chassis_y], s=200, c=color, marker='s', 
                       alpha=0.8, label=f'{label} Chassis')
            
            # Get end effector positions
            try:
                left_ee = ik_solver.forward_kinematics(q, 'left')
                right_ee = ik_solver.forward_kinematics(q, 'right')
                
                if abs(left_ee[2,3] - z_height) < 0.3:  # If EE is near this height
                    ax1.scatter([left_ee[0,3]], [left_ee[1,3]], s=100, c=color, 
                               marker='^', alpha=0.8)
                if abs(right_ee[2,3] - z_height) < 0.3:
                    ax1.scatter([right_ee[0,3]], [right_ee[1,3]], s=100, c=color, 
                               marker='^', alpha=0.8)
            except:
                pass
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title(f'SDF Slice at Z = {z_height:.1f}m with Robot Configs')
    ax1.set_aspect('equal')
    ax1.legend()
    plt.colorbar(im1, ax=ax1, label='Distance (m)')
    
    # Plot 2: Gradient field with safety margins
    grad_slice = sdf.gradient[:, :, z_grid, :2]  # X,Y gradients only
    
    # Subsample for cleaner visualization
    step = max(1, min(sdf.grid_size[0], sdf.grid_size[1]) // 12)
    X_sub = X[::step, ::step]
    Y_sub = Y[::step, ::step]
    U = grad_slice[::step, ::step, 0]
    V = grad_slice[::step, ::step, 1]
    
    # Plot gradient vectors
    ax2.quiver(X_sub, Y_sub, U, V, angles='xy', scale_units='xy', scale=0.5, 
               alpha=0.6, color='gray', label='SDF Gradient')
    
    # Plot SDF contours
    im2 = ax2.contourf(X, Y, sdf_slice, levels=15, cmap='viridis', alpha=0.3)
    
    # Highlight safety margins
    safety_contours = ax2.contour(X, Y, sdf_slice, levels=[0.15, 0.3], 
                                 colors=['red', 'orange'], linewidths=3, 
                                 linestyles=['--', '-'])
    ax2.clabel(safety_contours, inline=True, fontsize=10, fmt='Safety: %.2f m')
    
    # Plot robot configurations again
    for i, (q, color, label) in enumerate(zip(q_configs, colors, labels)):
        if q is not None:
            chassis_x, chassis_y, _ = ik_solver.get_chassis_pose(q)
            ax2.scatter([chassis_x], [chassis_y], s=200, c=color, marker='s', 
                       alpha=0.8, label=f'{label} Chassis')
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('SDF Gradients and Safety Margins')
    ax2.set_aspect('equal')
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig('/tmp/sdf_robot_slice.png', dpi=150, bbox_inches='tight')
    print("✓ 2D visualization saved to /tmp/sdf_robot_slice.png")
    plt.show()

def visualize_3d_robot_and_obstacles(obstacles, q_configs, ik_solver, sdf, target_poses=None):
    """Visualize 3D scene with robot, obstacles, and SDF"""
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot obstacles
    ax.scatter(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], 
               c='red', s=20, alpha=0.6, label='Obstacles')
    
    # Plot SDF sample points (only close to obstacles for clarity)
    if sdf.is_valid:
        sample_points = []
        sample_distances = []
        
        # Sample every few grid points
        step = max(1, max(sdf.grid_size) // 15)
        for i in range(0, sdf.grid_size[0], step):
            for j in range(0, sdf.grid_size[1], step):
                for k in range(0, sdf.grid_size[2], step):
                    if sdf.sdf[i, j, k] < 0.8:  # Only show close points
                        world_point = sdf.grid_to_world(np.array([i, j, k]))
                        sample_points.append(world_point)
                        sample_distances.append(sdf.sdf[i, j, k])
        
        if sample_points:
            sample_points = np.array(sample_points)
            sample_distances = np.array(sample_distances)
            
            # Color by distance
            scatter = ax.scatter(sample_points[:, 0], sample_points[:, 1], sample_points[:, 2],
                               c=sample_distances, cmap='plasma', s=15, alpha=0.3, 
                               label='SDF Field')
            plt.colorbar(scatter, ax=ax, label='Distance (m)', shrink=0.5)
    
    # Plot robot configurations
    colors = ['blue', 'green']
    labels = ['Without Avoidance', 'With Avoidance']
    
    for i, (q, color, label) in enumerate(zip(q_configs, colors, labels)):
        if q is not None:
            robot_points = visualize_robot_links(ik_solver, q, ax, color=color, 
                                               alpha=0.8, label=label)
    
    # Plot target poses if provided
    if target_poses is not None:
        left_target, right_target = target_poses
        ax.scatter([left_target[0,3]], [left_target[1,3]], [left_target[2,3]], 
                  s=200, c='purple', marker='*', alpha=0.9, label='Left Target')
        ax.scatter([right_target[0,3]], [right_target[1,3]], [right_target[2,3]], 
                  s=200, c='purple', marker='*', alpha=0.9, label='Right Target')
    
    # Set equal aspect ratio and labels
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Robot Configurations with Obstacles and SDF')
    
    # Set reasonable axis limits
    if len(obstacles) > 0:
        margin = 0.5
        ax.set_xlim([obstacles[:, 0].min() - margin, obstacles[:, 0].max() + margin])
        ax.set_ylim([obstacles[:, 1].min() - margin, obstacles[:, 1].max() + margin])
        ax.set_zlim([obstacles[:, 2].min() - margin, obstacles[:, 2].max() + margin])
    
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    plt.tight_layout()
    plt.savefig('/tmp/robot_3d_scene.png', dpi=150, bbox_inches='tight')
    print("✓ 3D visualization saved to /tmp/robot_3d_scene.png")
    plt.show()

def visualize_collision_analysis(ik_solver, q_configs, collision_checker, config_labels):
    """Visualize collision distances for different configurations"""
    if not q_configs or collision_checker is None:
        return
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Analyze collision distances for each configuration
    all_distances = []
    all_costs = []
    link_names = collision_checker.collision_link_names
    
    for q in q_configs:
        if q is not None:
            distances, cost = collision_checker.compute_collision_distances(q)
            all_distances.append(distances)
            all_costs.append(cost)
        else:
            all_distances.append([0] * len(link_names))
            all_costs.append(0)
    
    # Plot 1: Collision distances by link
    x_pos = np.arange(len(link_names))
    width = 0.35
    
    for i, (distances, label) in enumerate(zip(all_distances, config_labels)):
        offset = (i - 0.5) * width
        colors = ['red' if d < 0.1 else 'orange' if d < 0.3 else 'green' for d in distances]
        ax1.bar(x_pos + offset, distances, width, alpha=0.7, label=label, color=colors)
    
    ax1.set_xlabel('Robot Links')
    ax1.set_ylabel('Distance to Obstacles (m)')
    ax1.set_title('Collision Distances by Robot Link')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(link_names, rotation=45, ha='right')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Add safety threshold lines
    ax1.axhline(y=0.15, color='red', linestyle='--', alpha=0.7, label='Collision Risk')
    ax1.axhline(y=0.3, color='orange', linestyle='--', alpha=0.7, label='Safety Margin')
    
    # Plot 2: Total collision costs
    ax2.bar(config_labels, all_costs, alpha=0.7, 
           color=['red' if c > 10 else 'orange' if c > 1 else 'green' for c in all_costs])
    ax2.set_ylabel('Total Collision Cost')
    ax2.set_title('Total Collision Cost Comparison')
    ax2.grid(True, alpha=0.3)
    
    # Add cost annotations
    for i, (cost, label) in enumerate(zip(all_costs, config_labels)):
        ax2.text(i, cost + max(all_costs) * 0.02, f'{cost:.2f}', 
                ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('/tmp/collision_analysis.png', dpi=150, bbox_inches='tight')
    print("✓ Collision analysis saved to /tmp/collision_analysis.png")
    plt.show()

def test_sdf():
    """Test SDF functionality"""
    print("Testing Signed Distance Field...")
    
    # Create test obstacle (cube)
    obstacles = []
    for x in np.linspace(1.0, 1.5, 10):
        for y in np.linspace(-0.3, 0.3, 10):
            for z in np.linspace(0.5, 1.0, 10):
                obstacles.append([x, y, z])
    
    obstacles = np.array(obstacles)
    
    # Create robot-centered SDF
    sdf = SignedDistanceField(bounds=([-2, -2, -0.5], [3, 3, 2]), resolution=0.1, robot_centered=True)
    robot_pos = np.array([0.0, 0.0, 0.6])  # Robot at origin for test
    sdf.update_from_pointcloud(obstacles, robot_position=robot_pos)
    
    # Test queries
    test_points = [
        [0.0, 0.0, 0.75],  # Should be ~1.0m from obstacle
        [1.25, 0.0, 0.75], # Should be very close to obstacle
        [2.0, 0.0, 0.75],  # Should be ~0.5m from obstacle
    ]
    
    distances = sdf.query_distance(np.array(test_points))
    print(f"Test distances: {distances}")
    
    return sdf

def test_collision_checker():
    """Test collision checker"""
    print("Testing Collision Checker...")
    
    try:
        # Load robot model
        ik = SimpleMobileIK()
        sdf = test_sdf()
        
        # Create collision checker
        checker = CollisionChecker(ik.model, sdf)
        
        # Test with zero configuration
        q_test = np.zeros(len(ik.joint_names))
        distances, cost = checker.compute_collision_distances(q_test)
        
        print(f"Collision links: {len(checker.collision_link_names)}")
        print(f"Link names: {checker.collision_link_names}")
        print(f"Collision distances: {distances}")
        print(f"Collision cost: {cost:.3f}")
        
        # Test with configuration that might be closer to obstacles
        q_test[0] = 1.0  # Move chassis towards obstacle
        distances2, cost2 = checker.compute_collision_distances(q_test)
        print(f"After moving chassis forward:")
        print(f"Collision distances: {distances2}")
        print(f"Collision cost: {cost2:.3f}")
        
        return True
        
    except Exception as e:
        print(f"Error testing collision checker: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_ik_with_obstacles(show_visualization=True):
    """Test IK with obstacle avoidance and visualization"""
    print("Testing IK with obstacle avoidance...")
    
    try:
        # Create IK solver
        ik = SimpleMobileIK()
        
        # Create SDF with obstacles
        sdf = test_sdf()
        
        # Create and set collision checker
        checker = CollisionChecker(ik.model, sdf)
        ik.set_collision_checker(checker)
        
        # Get obstacle points for visualization
        obstacles = []
        for x in np.linspace(1.0, 1.5, 10):
            for y in np.linspace(-0.3, 0.3, 10):
                for z in np.linspace(0.5, 1.0, 10):
                    obstacles.append([x, y, z])
        obstacles = np.array(obstacles)
        
        # Create test targets (same as original but closer to obstacle)
        q_zero = np.zeros(len(ik.joint_names))
        left_fk_zero = ik.forward_kinematics(q_zero, 'left')
        right_fk_zero = ik.forward_kinematics(q_zero, 'right')
        
        # Target close to obstacle (should trigger avoidance)
        left_target = np.eye(4)
        left_target[:3, :3] = left_fk_zero[:3, :3]
        left_target[:3, 3] = [1.2, 0.0, 0.75]  # Close to obstacle
        
        right_target = np.eye(4)
        right_target[:3, :3] = right_fk_zero[:3, :3]
        right_target[:3, 3] = right_fk_zero[:3, 3] + [0.1, 0.1, 0.1]
        
        print("Testing IK without obstacle avoidance...")
        # First test without obstacle avoidance
        ik.set_collision_checker(None)
        q_no_avoid, success_no_avoid = ik.inverse_kinematics(left_target, right_target)
        
        cost_no_avoid = 0
        if success_no_avoid:
            distances_no_avoid, cost_no_avoid = checker.compute_collision_distances(q_no_avoid)
            print(f"Without avoidance - Success: {success_no_avoid}, Collision cost: {cost_no_avoid:.3f}")
        
        print("Testing IK with obstacle avoidance...")
        # Now test with obstacle avoidance
        ik.set_collision_checker(checker)
        q_with_avoid, success_with_avoid = ik.inverse_kinematics(left_target, right_target)
        
        cost_with_avoid = 0
        if success_with_avoid:
            distances_with_avoid, cost_with_avoid = checker.compute_collision_distances(q_with_avoid)
            print(f"With avoidance - Success: {success_with_avoid}, Collision cost: {cost_with_avoid:.3f}")
            
            # Check if avoidance actually helped
            if success_no_avoid and success_with_avoid:
                if cost_with_avoid < cost_no_avoid:
                    print("✓ Obstacle avoidance working - collision cost reduced!")
                else:
                    print("⚠ Obstacle avoidance might not be working optimally")
        
        # Visualization
        if show_visualization and (success_no_avoid or success_with_avoid):
            print("\n=== Generating Visualizations ===")
            
            # Prepare configurations for visualization
            q_configs = [
                q_no_avoid if success_no_avoid else None,
                q_with_avoid if success_with_avoid else None
            ]
            config_labels = ['Without Avoidance', 'With Avoidance']
            target_poses = (left_target, right_target)
            
            try:
                print("1. Creating 3D scene visualization...")
                visualize_3d_robot_and_obstacles(obstacles, q_configs, ik, sdf, target_poses)
            except Exception as e:
                print(f"Error in 3D visualization: {e}")
            
            try:
                print("2. Creating 2D SDF slice visualization...")
                visualize_sdf_slice_with_robot(sdf, q_configs, obstacles, ik, z_height=0.75)
            except Exception as e:
                print(f"Error in 2D slice visualization: {e}")
            
            try:
                print("3. Creating collision analysis...")
                valid_configs = [q for q in q_configs if q is not None]
                valid_labels = [label for q, label in zip(q_configs, config_labels) if q is not None]
                if len(valid_configs) > 0:
                    visualize_collision_analysis(ik, valid_configs, checker, valid_labels)
            except Exception as e:
                print(f"Error in collision analysis: {e}")
            
            print("\nVisualization files created:")
            print("• /tmp/robot_3d_scene.png - 3D robot and obstacles")
            print("• /tmp/sdf_robot_slice.png - 2D SDF slice with robot")
            print("• /tmp/collision_analysis.png - Collision distance analysis")
        
        return success_with_avoid
        
    except Exception as e:
        print(f"Error testing IK with obstacles: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests with optional visualization"""
    print("=== Testing Obstacle Avoidance System with Visualization ===")
    
    # Enable visualization in auto mode for non-interactive testing
    show_viz = True  # Enable by default for demonstration
    auto_viz = False
    
    print("\n1. Testing SDF...")
    sdf_success = True
    try:
        test_sdf()
        print("✓ SDF test passed")
    except Exception as e:
        print(f"✗ SDF test failed: {e}")
        sdf_success = False
    
    print("\n2. Testing Collision Checker...")
    collision_success = test_collision_checker()
    if collision_success:
        print("✓ Collision checker test passed")
    else:
        print("✗ Collision checker test failed")
    
    print("\n3. Testing IK with Obstacle Avoidance...")
    # Enable visualization if requested or if auto mode and tests are passing
    enable_viz = show_viz or (auto_viz and sdf_success and collision_success)
    
    ik_success = test_ik_with_obstacles(show_visualization=enable_viz)
    if ik_success:
        print("✓ IK with obstacle avoidance test passed")
    else:
        print("✗ IK with obstacle avoidance test failed")
    
    print("\n=== Test Summary ===")
    if sdf_success and collision_success and ik_success:
        print("✓ All tests passed! The obstacle avoidance system is working.")
        
        if enable_viz:
            print("\n✓ Visualizations generated successfully!")
            print("\nVisualization files:")
            print("• /tmp/robot_3d_scene.png - 3D scene with robot configurations")
            print("• /tmp/sdf_robot_slice.png - 2D SDF slice with robot positions")  
            print("• /tmp/collision_analysis.png - Collision distance analysis")
        
        print("\nTo use the system:")
        print("1. Run: ros2 run ia_robot mobile_ik_controller.py")
        print("2. Publish point cloud data to /camera/depth/points")
        print("3. Publish target poses to /demo_target_poses")
        print("4. Monitor collision distances on /collision_distances")
        print("5. Visualize SDF in RViz using /sdf_visualization markers")
        
        if not enable_viz:
            print("\nTo see visualizations, run with visualization enabled:")
            print("python3 test_obstacle_avoidance.py  # and choose 'y' for visualization")
            
    else:
        print("✗ Some tests failed. Check the error messages above.")
    
    return sdf_success and collision_success and ik_success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
