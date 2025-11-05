#!/usr/bin/env python3

import numpy as np
import pinocchio as pin
from signed_distance_field import SignedDistanceField
from obstacle_avoidance_config import ObstacleAvoidanceConfig

class CollisionChecker:
    """
    Collision checker for mobile robot using Signed Distance Field.
    Computes distances between robot links and obstacles for optimization.
    """
    
    def __init__(self, pinocchio_model, sdf, collision_links=None):
        """
        Initialize collision checker.
        
        Args:
            pinocchio_model: Pinocchio model of the robot
            sdf: SignedDistanceField instance
            collision_links: List of link names to check for collisions
        """
        self.model = pinocchio_model
        self.data = pinocchio_model.createData()
        self.sdf = sdf
        
        # Define links to check for collision (important robot parts)
        if collision_links is None:
            # Default collision links - mainly the arms and chassis
            self.collision_links = [
                'chassis_link',  # Main body
                'CHEST1_link',   # Torso
                'ARM0_LEFT_link', 'ARM1_LEFT_link', 'ARM2_LEFT_link', 'ARM3_LEFT_link',  # Left arm
                'ARM0_RIGHT_link', 'ARM1_RIGHT_link', 'ARM2_RIGHT_link', 'ARM3_RIGHT_link',  # Right arm
            ]
        else:
            self.collision_links = collision_links
        
        # Find frame IDs for collision links
        self.collision_frame_ids = []
        self.collision_link_names = []
        
        for link_name in self.collision_links:
            # Try both with and without "_link" suffix
            # TODO: necessary to check the variants?
            for name_variant in [link_name, link_name.replace('_link', ''), link_name + '_link']:
                if self._find_frame_id(name_variant) is not None:
                    frame_id = self._find_frame_id(name_variant)
                    self.collision_frame_ids.append(frame_id)
                    self.collision_link_names.append(name_variant)
                    break
        
        print(f"Collision checker initialized with {len(self.collision_frame_ids)} links: {self.collision_link_names}")
        
        # Link geometry approximation (sphere/cylinder representations)
        self.link_geometry = self._setup_link_geometry()
    
    def _find_frame_id(self, frame_name):
        """Find frame ID by name"""
        for frame_id in range(self.model.nframes):
            if self.model.frames[frame_id].name == frame_name:
                return frame_id
        return None
    
    def _setup_link_geometry(self):
        """
        Setup simplified geometry for each collision link.
        Using sphere/cylinder approximations for efficiency.
        """
        geometry = {}
        
        # Default geometry parameters
        default_params = {'type': 'sphere', 'radius': 0.1}
        
        # Use geometry from configuration
        link_specific_geometry = ObstacleAvoidanceConfig.LINK_GEOMETRY
        
        for link_name in self.collision_link_names:
            # Clean link name (remove _link suffix if present)
            clean_name = link_name.replace('_link', '')
            if clean_name in link_specific_geometry:
                geometry[link_name] = link_specific_geometry[clean_name]
            else:
                geometry[link_name] = default_params.copy()
        
        return geometry
    
    def compute_collision_distances(self, q, margin=None):
        """
        Compute minimum distances from robot links to obstacles.
        
        Args:
            q: joint configuration
            margin: safety margin to add to distances (uses config default if None)
            
        Returns:
            distances: list of minimum distances for each collision link
            total_collision_cost: scalar cost for optimization
        """
        if not self.sdf.is_valid:
            # No SDF available, return large distances (no collision cost)
            return [10.0] * len(self.collision_frame_ids), 0.0
        
        if margin is None:
            margin = ObstacleAvoidanceConfig.COLLISION_MARGIN
        
        # Map joints to pinocchio format
        pin_q = self._map_joints_to_pinocchio(q)
        
        # Update forward kinematics
        pin.forwardKinematics(self.model, self.data, pin_q)
        pin.updateFramePlacements(self.model, self.data)
        
        distances = []
        total_cost = 0.0
        collision_cost_fn = ObstacleAvoidanceConfig.get_collision_cost_function()
        
        for i, (frame_id, link_name) in enumerate(zip(self.collision_frame_ids, self.collision_link_names)):
            # Get link pose
            link_pose = self.data.oMf[frame_id]
            link_position = link_pose.translation
            
            # Compute distance based on link geometry
            min_distance = self._compute_link_sdf_distance(link_position, link_name)
            
            # Add margin
            distance_with_margin = min_distance - margin
            distances.append(distance_with_margin)
            
            # Add to collision cost using configured cost function
            cost = collision_cost_fn(distance_with_margin)
            total_cost += cost
        
        return distances, total_cost
    
    def _compute_link_sdf_distance(self, link_position, link_name):
        """
        Compute minimum distance from a link to obstacles using SDF.
        Takes into account link geometry (not just center point).
        """
        geometry = self.link_geometry[link_name]
        
        if geometry['type'] == 'sphere':
            # For sphere, query distance at center and subtract radius
            center_distance = self.sdf.query_distance(link_position)
            return center_distance - geometry['radius']
        
        elif geometry['type'] == 'cylinder':
            # For cylinder, sample points along the axis
            radius = geometry['radius']
            height = geometry['height']
            
            # Sample points along cylinder axis (assume z-axis)
            num_samples = 5
            sample_points = []
            for i in range(num_samples):
                z_offset = (i / (num_samples - 1) - 0.5) * height
                sample_point = link_position + np.array([0, 0, z_offset])
                sample_points.append(sample_point)
            
            # Query distances and take minimum
            sample_distances = self.sdf.query_distance(np.array(sample_points))
            min_distance = np.min(sample_distances)
            return min_distance - radius
        
        elif geometry['type'] == 'box':
            # For box, sample points at corners/faces
            size = geometry['size']  # [width, depth, height]
            
            # Sample points at box corners
            sample_points = []
            for dx in [-size[0]/2, 0, size[0]/2]:
                for dy in [-size[1]/2, 0, size[1]/2]:
                    for dz in [-size[2]/2, 0, size[2]/2]:
                        sample_point = link_position + np.array([dx, dy, dz])
                        sample_points.append(sample_point)
            
            # Query distances and take minimum
            sample_distances = self.sdf.query_distance(np.array(sample_points))
            return np.min(sample_distances)
        
        else:
            # Default: treat as point
            return self.sdf.query_distance(link_position)
    
    def _map_joints_to_pinocchio(self, q):
        """Map our joint configuration to Pinocchio's joint order - matches SimpleMobileIK"""
        # Create configuration vector for Pinocchio
        pin_q = pin.neutral(self.model)
        
        # Use the same joint names as SimpleMobileIK
        joint_names = [
            'chassis_x_joint', 'chassis_y_joint', 'chassis_rotation_joint',
            'CHEST1', 'ARM0_LEFT', 'ARM1_LEFT', 'ARM2_LEFT', 'ARM3_LEFT',
            'ARM0_RIGHT', 'ARM1_RIGHT', 'ARM2_RIGHT', 'ARM3_RIGHT',
            'Leg_front_left_3', 'Leg_front_left_2', 'Leg_front_left_1',
            'Leg_front_right_3', 'Leg_front_right_2', 'Leg_front_right_1',
            'Leg_back_left_2', 'Leg_back_left_1', 'Leg_back_right_2', 'Leg_back_right_1'
        ]
        
        # Map our joints to Pinocchio joints with correct dimensionality
        for i, our_joint_name in enumerate(joint_names):
            if i < len(q) and self.model.existJointName(our_joint_name):
                pin_joint_id = self.model.getJointId(our_joint_name)
                if pin_joint_id > 0:  # Skip universe joint (id=0)
                    joint_model = self.model.joints[pin_joint_id]
                    idx_q = joint_model.idx_q
                    
                    if joint_model.nq == 1:  # Simple joints (prismatic, revolute)
                        pin_q[idx_q] = q[i]
                    elif joint_model.nq == 2:  # Unbounded revolute joints (use complex representation)
                        # For unbounded revolute joints, use cos/sin representation
                        angle = q[i]
                        pin_q[idx_q] = np.cos(angle)     # Real part
                        pin_q[idx_q + 1] = np.sin(angle) # Imaginary part
        
        return pin_q
    
    def get_collision_gradient(self, q, epsilon=1e-6):
        """
        Compute gradient of collision cost with respect to joint angles.
        Uses finite differences for now.
        
        Args:
            q: joint configuration
            epsilon: finite difference step size
            
        Returns:
            gradient: gradient vector of same size as q
        """
        _, base_cost = self.compute_collision_distances(q)
        gradient = np.zeros_like(q)
        
        for i in range(len(q)):
            # Forward finite difference
            q_plus = q.copy()
            q_plus[i] += epsilon
            _, cost_plus = self.compute_collision_distances(q_plus)
            
            # Compute gradient
            gradient[i] = (cost_plus - base_cost) / epsilon
        
        return gradient


# Test function
if __name__ == "__main__":
    import os
    from signed_distance_field import SignedDistanceField
    
    # Test collision checker
    print("Testing collision checker...")
    
    # Load URDF for testing
    urdf_path = "/home/hanxi/code/ia_robot_sim/src/ia_robot/urdf/ia_robot_ik.urdf"
    if os.path.exists(urdf_path):
        model = pin.buildModelFromUrdf(urdf_path)
        
        # Create test SDF
        sdf = SignedDistanceField(resolution=0.1)
        test_points = np.array([[1.0, 0.0, 1.0], [1.0, 0.1, 1.0], [1.0, -0.1, 1.0]])  # Simple obstacle
        sdf.update_from_pointcloud(test_points)
        
        # Create collision checker
        checker = CollisionChecker(model, sdf)
        
        # Test with zero configuration
        q_test = np.zeros(22)  # Assuming 22 joints
        distances, cost = checker.compute_collision_distances(q_test)
        
        print(f"Test collision distances: {distances}")
        print(f"Test collision cost: {cost}")
        print("Collision checker test completed.")
    else:
        print(f"URDF not found at {urdf_path}")
