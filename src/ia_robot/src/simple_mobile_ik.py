#!/usr/bin/env python3

import numpy as np
import scipy.optimize as opt
from scipy.spatial.transform import Rotation
import logging
import pinocchio as pin
import os
import matplotlib.pyplot as plt
from collision_checker import CollisionChecker
from obstacle_avoidance_config import ObstacleAvoidanceConfig

from ament_index_python.packages import get_package_share_directory

# Mobile IK solver using URDF-based forward kinematics
class SimpleMobileIK:
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # Load URDF model using Pinocchio
        urdf_pkg_share = get_package_share_directory('ia_robot_urdf')
        urdf_path = os.path.join(urdf_pkg_share, 'urdf', 'ia_robot_ik.absolute.urdf')
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
            
        self.model = pin.buildModelFromUrdf(urdf_path)
        
        # Robot configuration (approximate based on URDF)
        # Joint limits (radians for continuous joints, meters for prismatic)
        self.joint_limits = {
            # Chassis joints
            'chassis_x_joint': (-20.0, 20.0),
            'chassis_y_joint': (-20.0, 20.0), 
            'chassis_rotation_joint': (-np.pi, 2 * np.pi),
            # Torso
            'CHEST1': (0, np.pi/2),
            # Left arm
            'ARM0_LEFT': (-0.05, 0.2),   # prismatic
            'ARM1_LEFT': (0, np.pi),    # Limited to 0 to π
            'ARM2_LEFT': (-np.pi, 0),   # Limited to -π to 0
            'ARM3_LEFT': (-np.pi, np.pi),
            # Right arm
            'ARM0_RIGHT': (-0.05, 0.2),  # prismatic
            'ARM1_RIGHT': (0, np.pi),   # Limited to 0 to π
            'ARM2_RIGHT': (-np.pi, 0),  # Limited to -π to 0
            'ARM3_RIGHT': (-np.pi, np.pi),
            # Legs (keeping them at fixed positions for stability)
            'Leg_front_left_3': (0.0, 0.0),    # Fixed
            'Leg_front_left_2': (0.0, 0.0),    # Fixed
            'Leg_front_left_1': (0.0, 0.0),    # Fixed
            'Leg_front_right_3': (0.0, 0.0),   # Fixed
            'Leg_front_right_2': (0.0, 0.0),   # Fixed
            'Leg_front_right_1': (0.0, 0.0),   # Fixed
            'Leg_back_left_2': (0.0, 0.0),     # Fixed
            'Leg_back_left_1': (0.0, 0.0),     # Fixed
            'Leg_back_right_2': (0.0, 0.0),    # Fixed
            'Leg_back_right_1': (0.0, 0.0),    # Fixed
        }
        
        # Joint order (matching expected output)
        self.joint_names = list(self.joint_limits.keys())
        self.n_joints = len(self.joint_names)
        
        # Current joint configuration
        self.current_q = np.zeros(self.n_joints)
        
        # Create frame IDs for end effectors
        self.left_ee_frame_id = None
        self.right_ee_frame_id = None
        
        # Find or create end effector frames
        self._setup_end_effector_frames()
        
        # Create data after adding frames
        self.data = self.model.createData()
        
        # Collision checker (will be set by controller)
        self.collision_checker = None
        
        # Store last cost breakdown for visualization
        self.last_cost_breakdown = {}
        
        # Debug: Test if frame modification worked
        test_q = np.zeros(self.n_joints)
        test_fk = self.forward_kinematics(test_q, 'left')
        self.logger.info(f"Frame test - FK rotation [0,0]: {test_fk[:3, :3][0, 0]:.3f}")
        self.logger.info(f"Expected: -1.0 if reversal applied, other value if not")
        
    def _setup_end_effector_frames(self):
        """Setup end effector frames for left and right arms"""
        # Add end effector frames to ARM2_LEFT and ARM2_RIGHT with -0.15m X offset
        left_ee_offset = pin.SE3(np.eye(3), np.array([-0.15, 0, 0.05]))
        right_ee_offset = pin.SE3(np.eye(3), np.array([-0.15, 0, -0.05]))

        # Find ARM2_LEFT and ARM2_RIGHT frame IDs
        arm2_left_id = None
        arm2_right_id = None
        
        for frame_id in range(self.model.nframes):
            frame_name = self.model.frames[frame_id].name
            if frame_name == "ARM2_LEFT":
                arm2_left_id = frame_id
            elif frame_name == "ARM2_RIGHT":
                arm2_right_id = frame_id
        
        if arm2_left_id is not None:
            # Create left end effector frame
            left_ee_frame = pin.Frame("left_ee", 
                                    self.model.frames[arm2_left_id].parentJoint,
                                    self.model.frames[arm2_left_id].placement * left_ee_offset,
                                    pin.FrameType.OP_FRAME)
            self.left_ee_frame_id = self.model.addFrame(left_ee_frame)
        
        if arm2_right_id is not None:
            # Create right end effector frame
            right_ee_frame = pin.Frame("right_ee",
                                     self.model.frames[arm2_right_id].parentJoint, 
                                     self.model.frames[arm2_right_id].placement * right_ee_offset,
                                     pin.FrameType.OP_FRAME)
            self.right_ee_frame_id = self.model.addFrame(right_ee_frame)
        
        self.logger.info(f"Created EE frames - Left: {self.left_ee_frame_id}, Right: {self.right_ee_frame_id}")
        
        # Note: Will test FK after data is created
        
    def _map_joints_to_pinocchio(self, q):
        """Map our joint configuration to Pinocchio's joint order"""
        # Create configuration vector for Pinocchio
        pin_q = pin.neutral(self.model)
        
        # Map our joints to Pinocchio joints with correct dimensionality
        for i, our_joint_name in enumerate(self.joint_names):
            if self.model.existJointName(our_joint_name):
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
        
    def forward_kinematics(self, q, arm='left'):
        """Compute forward kinematics using URDF and Pinocchio"""
        # Map our joint configuration to Pinocchio format
        pin_q = self._map_joints_to_pinocchio(q)
        
        # Compute forward kinematics
        pin.forwardKinematics(self.model, self.data, pin_q)
        pin.updateFramePlacements(self.model, self.data)
        
        # Get end effector transformation
        if arm == 'left':
            if self.left_ee_frame_id is None:
                raise ValueError("Left end effector frame not created")
            ee_placement = self.data.oMf[self.left_ee_frame_id]
        elif arm == 'right':
            if self.right_ee_frame_id is None:
                raise ValueError("Right end effector frame not created")
            ee_placement = self.data.oMf[self.right_ee_frame_id]
        else:
            raise ValueError(f"Unknown arm: {arm}")
        
        # Convert to 4x4 transformation matrix
        T_world_ee = np.eye(4)
        T_world_ee[:3, :3] = ee_placement.rotation
        T_world_ee[:3, 3] = ee_placement.translation
        
        return T_world_ee
        
    def inverse_kinematics(self, left_target, right_target, q_init=None, mode="rest"):
        """Solve inverse kinematics for dual-arm mobile robot
        
        Args:
            left_target: Target pose for left end-effector (4x4 matrix)
            right_target: Target pose for right end-effector (4x4 matrix)
            q_init: Initial joint configuration (optional)
            mode: Current robot mode ("rest", "approach", "assist", "retreat")
        """
        
        if q_init is None:
            q_init = self.current_q.copy()
            
        def objective(q):
            """Objective function to minimize"""
            # Forward kinematics for both arms
            left_fk = self.forward_kinematics(q, 'left')
            right_fk = self.forward_kinematics(q, 'right')
            
            # Debug: print first iteration to see if FK changed
            if not hasattr(self, '_debug_printed'):
                self.logger.info(f"IK using FK - Left: {left_fk[:3, :3][0, 0]:.3f}, Right: {right_fk[:3, :3][0, 0]:.3f}")
                self._debug_printed = True
            
            # Position errors
            left_pos_error = np.linalg.norm(left_fk[:3, 3] - left_target[:3, 3])
            right_pos_error = np.linalg.norm(right_fk[:3, 3] - right_target[:3, 3])

            # symmetry encouragement
            # minimize the difference in ARM0_LEFT and ARM0_RIGHT (prismatic joints)
            arm0_left_idx = self.joint_names.index('ARM0_LEFT')
            arm0_right_idx = self.joint_names.index('ARM0_RIGHT')
            symmetry_error = np.abs(q[arm0_left_idx] - q[arm0_right_idx]) * ObstacleAvoidanceConfig.SYMMETRY_WEIGHT
            
            # Orientation errors (using rotation matrices)
            def rotation_error_old(R_current, R_target):
                # Compute relative rotation matrix
                R_error = R_target.T @ R_current
                # Extract rotation angle from rotation matrix
                trace_R = np.trace(R_error)
                # Clamp trace to valid range for arccos
                trace_R = np.clip(trace_R, -1.0, 3.0)
                angle_error = np.arccos((trace_R - 1) / 2)
                return angle_error
            # Calculate rotation error only for x-axis (ignore y and z axis rotations)
            def rotation_error(R_current, R_target):
                # Extract x-axis vectors from rotation matrices
                x_current = R_current[:, 0]  # First column is x-axis
                x_target = R_target[:, 0]    # First column is x-axis
                
                # Calculate angle between x-axis vectors
                dot_product = np.dot(x_current, x_target)
                # Clamp to valid range for arccos
                dot_product = np.clip(dot_product, -1.0, 1.0)
                angle_error = np.arccos(dot_product)
                return angle_error
            
            left_rot_error = rotation_error(left_fk[:3, :3], left_target[:3, :3])
            right_rot_error = rotation_error(right_fk[:3, :3], right_target[:3, :3])
            
            # Regularization (prefer smaller joint movements)
            regularization = ObstacleAvoidanceConfig.REGULARIZATION_WEIGHT * np.linalg.norm(q - q_init)
            
            # Chassis movement penalty - heavily penalize chassis movement to prefer upper body motion
            # Penalize both absolute movement and changes from initial configuration
            chassis_x_change = q[0] - q_init[0]
            chassis_y_change = q[1] - q_init[1]
            chassis_theta_change = q[2] - q_init[2]
            
            # Quadratic penalties for chassis changes from initial position
            chassis_xy_penalty = ObstacleAvoidanceConfig.CHASSIS_XY_PENALTY * (chassis_x_change**2 + chassis_y_change**2)
            chassis_z_penalty = ObstacleAvoidanceConfig.CHASSIS_Z_PENALTY * chassis_theta_change**2

            # reward instead of penalty while moving
            if mode in ('approach', 'retreat'):
                chassis_xy_penalty *= -1
                chassis_z_penalty *= -1
            
            # CHEST1 angle encouragement - reward larger CHEST1 angles
            # Use different reward based on mode (penalize extension when approaching human)
            chest1_angle = q[self.joint_names.index('CHEST1')]
            if mode in ('approach', 'rest', 'retreat'):
                chest1_reward = ObstacleAvoidanceConfig.CHEST_PENALTY_APPROACHING * chest1_angle # max(0, (chest1_angle - (np.pi/8)))
            elif mode == "assist":
                chest1_reward = ObstacleAvoidanceConfig.CHEST_PENALTY * chest1_angle
            else:
                chest1_reward = 0.0
            
            # ARM1 angle encouragement - reward ARM1_LEFT and ARM1_RIGHT to be more extended (closer to π/2)
            arm1_left_angle = q[self.joint_names.index('ARM1_LEFT')]
            arm1_right_angle = q[self.joint_names.index('ARM1_RIGHT')]
            arm1_reward = ObstacleAvoidanceConfig.ARM1_PENALTY * ((arm1_left_angle - (np.pi/2))**2 + (arm1_right_angle - (np.pi/2))**2)

            # Obstacle avoidance cost
            obstacle_cost = 0.0
            # if self.collision_checker is not None:
            #     try:
            #         _, collision_cost = self.collision_checker.compute_collision_distances(q)
            #         obstacle_cost = ObstacleAvoidanceConfig.OBSTACLE_AVOIDANCE_WEIGHT * collision_cost
            #     except Exception as e:
            #         self.logger.warning(f"Error computing collision cost: {e}")
            
            # Calculate weighted costs
            position_cost = ObstacleAvoidanceConfig.POSITION_WEIGHT * (left_pos_error + right_pos_error)
            orientation_cost = ObstacleAvoidanceConfig.ORIENTATION_WEIGHT * (left_rot_error + right_rot_error)
            
            # Store cost breakdown for visualization (only for final result)
            self.last_cost_breakdown = {
                'Position Error': position_cost,
                'Orientation Error': orientation_cost,
                'Regularization': regularization,
                'Chassis XY Penalty': chassis_xy_penalty,
                'Chassis Z Penalty': chassis_z_penalty,
                'Chest Reward': -chest1_reward,  # Negative because it's a reward
                'Obstacle Avoidance': obstacle_cost,
                'Symmetry Error': symmetry_error
            }
            
            total_cost = (position_cost + orientation_cost + regularization + 
                         chassis_xy_penalty + chassis_z_penalty + chest1_reward + obstacle_cost + symmetry_error + arm1_reward)
            
            return total_cost
            
        # Joint limits
        # bounds = [self.joint_limits[name] for name in self.joint_names]

        # Joint limits - restrict based on both absolute limits and max step size
        bounds = []
        for i, name in enumerate(self.joint_names):
            abs_lower, abs_upper = self.joint_limits[name]
            
            # Apply different step limits for chassis vs other joints
            if name in ['chassis_x_joint', 'chassis_y_joint', 'chassis_rotation_joint']:
                max_step = ObstacleAvoidanceConfig.MAX_CHASSIS_STEP
            else:
                max_step = ObstacleAvoidanceConfig.MAX_JOINT_STEP
            
            # Limit the change from initial configuration
            step_lower = max(abs_lower, q_init[i] - max_step)
            step_upper = min(abs_upper, q_init[i] + max_step)
            
            bounds.append((step_lower, step_upper))
        
        # Position error constraint - reject solutions with position error > MAX_POSITION_ERROR
        def position_constraint(q):
            """Constraint to keep position error below configured threshold"""
            left_fk = self.forward_kinematics(q, 'left')
            right_fk = self.forward_kinematics(q, 'right')
            
            left_pos_error = np.linalg.norm(left_fk[:3, 3] - left_target[:3, 3])
            right_pos_error = np.linalg.norm(right_fk[:3, 3] - right_target[:3, 3])
            
            total_pos_error = left_pos_error + right_pos_error
            # Constraint should be >= 0, so we return (max_allowed - actual_error)
            return ObstacleAvoidanceConfig.MAX_POSITION_ERROR - total_pos_error
        
        # CHEST1 angle constraint
        def chest1_constraint(q):
            """Constraint to keep CHEST1 angle below CHEST_MAX_ANGLE (default = 0.7) radians"""
            chest1_idx = self.joint_names.index('CHEST1')
            # Constraint should be >= 0, so we return (max_allowed - actual_angle)
            return ObstacleAvoidanceConfig.CHEST_MAX_ANGLE - q[chest1_idx]
        
        # Define constraint dictionaries
        constraints = [
            # {'type': 'ineq', 'fun': position_constraint},
            {'type': 'ineq', 'fun': chest1_constraint}
        ]
        
        try:
            # Solve optimization problem with position constraint
            result = opt.minimize(
                objective, 
                q_init,
                method='SLSQP',
                bounds=bounds,
                constraints=constraints,
                options={'maxiter': 200, 'ftol': 1e-4, 'disp': False}
            )
            
            if result.success:
                # Check if any joints hit step limits
                joint_changes = np.abs(result.x - q_init)
                max_change = np.max(joint_changes)
                max_change_idx = np.argmax(joint_changes)
                
                if max_change > ObstacleAvoidanceConfig.MAX_JOINT_STEP * 0.95:
                    self.logger.debug(f"Joint step limit approached: {self.joint_names[max_change_idx]}: {joint_changes[max_change_idx]:.3f}")
                
                self.current_q = result.x.copy()
                # print(f"pos err, obs err, other err, {self.last_cost_breakdown['Position Error']:.4f} {self.last_cost_breakdown['Obstacle Avoidance']:.4f} {sum(v for k,v in self.last_cost_breakdown.items() if k not in ['Position Error', 'Obstacle Avoidance']):.4f}")

                # print(f"pos err: \t {self.last_cost_breakdown['Position Error']:.4f},\n"
                #       f"orient err: \t {self.last_cost_breakdown['Orientation Error']:.4f},\n"
                #       f"regularization:\t {self.last_cost_breakdown['Regularization']:.4f},\n"
                #       f"chassis xy pen:\t {self.last_cost_breakdown['Chassis XY Penalty']:.4f},\n"
                #       f"chassis z pen:\t {self.last_cost_breakdown['Chassis Z Penalty']:.4f},\n"
                #       f"chest reward:\t {self.last_cost_breakdown['Chest Reward']:.4f},\n"
                # )
                return result.x, True
            else:
                self.logger.warning(f"IK optimization failed: {result.message}")
                return q_init, False
                
        except Exception as e:
            self.logger.error(f"IK solver error: {e}")
            return q_init, False
            
    def get_chassis_pose(self, q):
        """Extract chassis pose from joint configuration"""
        joint_indices = {name: i for i, name in enumerate(self.joint_names)}
        return (q[joint_indices['chassis_x_joint']], 
                q[joint_indices['chassis_y_joint']], 
                q[joint_indices['chassis_rotation_joint']])  # x, y, theta
        
    def set_chassis_pose(self, q, x, y, theta):
        """Set chassis pose in joint configuration"""
        joint_indices = {name: i for i, name in enumerate(self.joint_names)}
        q[joint_indices['chassis_x_joint']] = x
        q[joint_indices['chassis_y_joint']] = y  
        q[joint_indices['chassis_rotation_joint']] = theta
        
    def get_joint_names(self):
        """Get list of joint names"""
        return self.joint_names.copy()
    
    def set_collision_checker(self, collision_checker):
        """Set collision checker for obstacle avoidance"""
        self.collision_checker = collision_checker
    
    def plot_cost_breakdown(self, title="IK Cost Breakdown", save_path=None):
        """Plot bar chart of cost components from the last IK solution"""
        if not self.last_cost_breakdown:
            self.logger.warning("No cost breakdown available. Run IK first.")
            return
        
        # Prepare data for plotting
        cost_names = list(self.last_cost_breakdown.keys())
        cost_values = list(self.last_cost_breakdown.values())
        
        # Create colors - red for penalties, green for rewards, blue for errors
        colors = []
        for name in cost_names:
            if 'Penalty' in name or 'Avoidance' in name:
                colors.append('red')
            elif 'Reward' in name:
                colors.append('green')
            elif 'Error' in name:
                colors.append('orange')
            else:
                colors.append('blue')
        
        # Create the plot
        plt.figure(figsize=(12, 8))
        bars = plt.bar(cost_names, cost_values, color=colors, alpha=0.7, edgecolor='black')
        
        # Add value labels on bars
        for bar, value in zip(bars, cost_values):
            height = bar.get_height()
            plt.text(bar.get_x() + bar.get_width()/2., height + (0.01 * max(abs(v) for v in cost_values)),
                    f'{value:.4f}', ha='center', va='bottom', fontsize=10)
        
        plt.title(title, fontsize=16, fontweight='bold')
        plt.xlabel('Cost Components', fontsize=12)
        plt.ylabel('Cost Value', fontsize=12)
        plt.xticks(rotation=45, ha='right')
        plt.grid(axis='y', alpha=0.3)
        
        # Add total cost as text
        total_cost = sum(cost_values)
        plt.text(0.02, 0.98, f'Total Cost: {total_cost:.4f}', 
                transform=plt.gca().transAxes, fontsize=12, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            self.logger.info(f"Cost breakdown plot saved to: {save_path}")
        
        plt.show()
        
        # Also print the breakdown
        print("\nCost Breakdown:")
        print("-" * 40)
        for name, value in self.last_cost_breakdown.items():
            print(f"{name:<20}: {value:>10.6f}")
        print("-" * 40)
        print(f"{'Total Cost':<20}: {total_cost:>10.6f}")
        print("-" * 40)


if __name__ == "__main__":
    # Test the simple IK solver
    ik = SimpleMobileIK()
    
    # Get natural end effector orientations from zero configuration
    q_zero = np.zeros(22)
    left_fk_zero = ik.forward_kinematics(q_zero, 'left')
    right_fk_zero = ik.forward_kinematics(q_zero, 'right')
    
    # Define test targets using natural orientations and positions close to zero config
    left_target = np.eye(4)
    left_target[:3, :3] = left_fk_zero[:3, :3]  # Use natural orientation
    left_target[:3, 3] = left_fk_zero[:3, 3] + [0.2, 0.1, 0.1]  # Small offset from natural position
    
    right_target = np.eye(4) 
    right_target[:3, :3] = right_fk_zero[:3, :3]  # Use natural orientation
    right_target[:3, 3] = right_fk_zero[:3, 3] + [0.2, -0.1, 0.1]  # Small offset from natural position
    
    print("Testing Simple Mobile IK...")
    
    # Solve IK
    q_solution, success = ik.inverse_kinematics(left_target, right_target)
    
    if success:
        print(f"IK solved successfully!")
        print(f"Joint configuration: {q_solution}")
        
        # Verify solution
        left_fk = ik.forward_kinematics(q_solution, 'left')
        right_fk = ik.forward_kinematics(q_solution, 'right')
        
        # Position errors
        left_pos_error = np.linalg.norm(left_fk[:3, 3] - left_target[:3, 3])
        right_pos_error = np.linalg.norm(right_fk[:3, 3] - right_target[:3, 3])
        
        # Orientation errors
        def rotation_error_degrees(R_current, R_target):
            R_error = R_target.T @ R_current
            trace_R = np.trace(R_error)
            trace_R = np.clip(trace_R, -1.0, 3.0)
            angle_error = np.arccos((trace_R - 1) / 2)
            return np.degrees(angle_error)
        
        left_rot_error = rotation_error_degrees(left_fk[:3, :3], left_target[:3, :3])
        right_rot_error = rotation_error_degrees(right_fk[:3, :3], right_target[:3, :3])
        
        print(f"Left arm position error: {left_pos_error:.4f} m")
        print(f"Left arm orientation error: {left_rot_error:.2f} degrees")
        print(f"Right arm position error: {right_pos_error:.4f} m")
        print(f"Right arm orientation error: {right_rot_error:.2f} degrees")
        
        chassis_pose = ik.get_chassis_pose(q_solution)
        print(f"Chassis pose: x={chassis_pose[0]:.3f}, y={chassis_pose[1]:.3f}, θ={chassis_pose[2]:.3f}")
        
        # Visualize cost breakdown
        print("\nGenerating cost breakdown visualization...")
        ik.plot_cost_breakdown(title="IK Cost Breakdown - Test Solution")
        
    else:
        print("IK failed to converge")