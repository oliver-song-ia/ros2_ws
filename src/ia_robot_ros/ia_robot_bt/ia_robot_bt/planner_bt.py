#!/usr/bin/env python3
"""
Robot Planner Behavior Tree
Converts the manual state machine from robot_planner_manual_sm.py into a behavior tree structure.

Modes: rest, approach, wait_for_arm_raise, assist, retreat
State transitions can be triggered via /set_mode topic or automatically based on conditions.
"""

import py_trees
from py_trees.composites import Sequence, Selector, Parallel
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from std_msgs.msg import String
import numpy as np
import rclpy
from rclpy.action import ActionClient
from ia_robot_interfaces.action import MoveToStandby
from ia_robot_interfaces.srv import SetJointLock


class PlannerState(py_trees.behaviour.Behaviour):
    """
    Base class for planner state behaviors.
    Handles pose generation, subscription to pose detection, and publishing targets.
    """
    
    # Class-level constants for blackboard keys
    BLACKBOARD_KEYS = [
        'planner_mode',
        'planner_mode_requested',
        'planner_joints',
        'planner_human_pos',
        'planner_human_ori',
        'planner_current_poses',
        'planner_rest_target',
        'planner_approach_target',
        'planner_last_assist_pos',
        'planner_last_assist_ori',
        'planner_last_assist_height',
    ]
    
    def __init__(self, name, node, blackboard):
        super().__init__(name)
        self.node = node
        self.blackboard = blackboard
        
        # Initialize ROS interfaces once per node
        self._init_ros_interfaces()
        
        # Register blackboard keys
        self._register_blackboard_keys()
    
    def _init_ros_interfaces(self):
        """Initialize publishers, subscribers, and clients (created once per node)"""
        if not hasattr(self.node, '_planner_target_pub'):
            self.node._planner_target_pub = self.node.create_publisher(
                PoseArray, '/target_eef_poses', 10)
            self.node._planner_mode_pub = self.node.create_publisher(
                String, '/mode', 10)
        
        if not hasattr(self.node, '_planner_pose_sub'):
            self.node._planner_pose_sub = self.node.create_subscription(
                MarkerArray, '/pose_detection', self._pose_callback, 10)
        
        if not hasattr(self.node, '_planner_current_eef_sub'):
            self.node._planner_current_eef_sub = self.node.create_subscription(
                PoseArray, '/current_poses', self._current_eef_callback, 10)
        
        if not hasattr(self.node, '_planner_mode_cmd_sub'):
            self.node._planner_mode_cmd_sub = self.node.create_subscription(
                String, '/set_mode', self._mode_command_callback, 10)
        
        if not hasattr(self.node, '_standby_action_client'):
            self.node._standby_action_client = ActionClient(
                self.node, MoveToStandby, 'move_to_standby')
        
        if not hasattr(self.node, '_joint_lock_client'):
            self.node._joint_lock_client = self.node.create_client(
                SetJointLock, 'set_joint_lock')
    
    def _register_blackboard_keys(self):
        """Register all blackboard keys with WRITE access"""
        for key in self.BLACKBOARD_KEYS:
            try:
                self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
            except KeyError:
                pass  # Already registered
    
    def _pose_callback(self, msg):
        """Store latest pose detection"""
        joints = self.extract_joint_positions(msg)
        if joints is not None:
            self.blackboard.set('planner_joints', joints)
            
            # Calculate human state
            human_pos_xy = joints[8, :2]  # Torso XY
            r_shoulder, l_shoulder = joints[2], joints[3]
            human_ori = self.calculate_human_orientation(r_shoulder, l_shoulder)
            
            self.blackboard.set('planner_human_pos', human_pos_xy)
            self.blackboard.set('planner_human_ori', human_ori)
    
    def _current_eef_callback(self, msg):
        """Store current EEF poses"""
        if len(msg.poses) >= 2:
            self.blackboard.set('planner_current_poses', [msg.poses[0], msg.poses[1]])
    
    def _mode_command_callback(self, msg):
        """Handle mode command from /set_mode topic"""
        requested_mode = msg.data.lower().strip()
        valid_modes = ['rest', 'approach', 'wait_for_arm_raise', 'insert', 'wait_for_rest', 'assist', 'wait_for_retreat', 'retreat']
        
        if requested_mode in valid_modes:
            self.blackboard.set('planner_mode_requested', requested_mode)
            self.node.get_logger().info(f'[Planner BT] Mode change requested: {requested_mode}')
        else:
            self.node.get_logger().warn(
                f'[Planner BT] Invalid mode requested: "{requested_mode}". '
                f'Valid modes: {valid_modes}'
            )
    
    def extract_joint_positions(self, marker_array):
        """
        Extract joint positions from MarkerArray
        joint names: 'Head', 'Neck', 'R_Shoulder', 'L_Shoulder', 'R_Elbow', 'L_Elbow',
            'R_Hand', 'L_Hand', 'Torso'
        """
        joint_positions = np.zeros((9, 3))
        found_joints = 0
        
        for marker in marker_array.markers:
            if marker.ns == 'skeleton_joints' and marker.type == 2:
                joint_idx = marker.id
                if 0 <= joint_idx < 9:
                    joint_positions[joint_idx] = [
                        marker.pose.position.x,
                        marker.pose.position.y,
                        marker.pose.position.z
                    ]
                    found_joints += 1
        
        if found_joints < 9:
            return None
        return joint_positions
    
    def calculate_human_orientation(self, r_shoulder, l_shoulder):
        """Calculate human facing direction"""
        shoulder_vec = l_shoulder - r_shoulder
        forward_dir = np.cross(shoulder_vec, np.array([0.0, 0.0, 1.0]))
        forward_dir_xy = forward_dir[:2]
        forward_norm = np.linalg.norm(forward_dir_xy)
        
        if forward_norm > 1e-6:
            forward_dir_xy = forward_dir_xy / forward_norm
            return np.arctan2(forward_dir_xy[1], forward_dir_xy[0])
        return 0.0
    
    def create_pose(self, position, orientation, local_roll=0.0):
        """Create Pose from position, orientation (yaw), and local roll"""
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        
        # First create quaternion for yaw rotation (around global Z)
        half_yaw = orientation * 0.5
        q_yaw = np.array([0.0, 0.0, np.sin(half_yaw), np.cos(half_yaw)])
        
        # Then create quaternion for local roll (around local X)
        half_roll = local_roll * 0.5
        q_roll = np.array([np.sin(half_roll), 0.0, 0.0, np.cos(half_roll)])
        
        # Combine: first apply yaw, then apply roll in the rotated frame
        x1, y1, z1, w1 = q_yaw
        x2, y2, z2, w2 = q_roll
        
        pose.orientation = Quaternion(
            x=float(w1*x2 + x1*w2 + y1*z2 - z1*y2),
            y=float(w1*y2 - x1*z2 + y1*w2 + z1*x2),
            z=float(w1*z2 + x1*y2 - y1*x2 + z1*w2),
            w=float(w1*w2 - x1*x2 - y1*y2 - z1*z2)
        )
        return pose
    
    def create_front_position_poses(self, human_pos_xy, human_ori, distance, height):
        """Create poses at fixed distance in front of human"""
        target_xy = human_pos_xy + distance * np.array([np.cos(human_ori), np.sin(human_ori)])
        
        arm_spacing = 0.45
        lateral_vec = np.array([-np.sin(human_ori), np.cos(human_ori)])
        
        left_pos = np.array([
            target_xy[0] - lateral_vec[0] * arm_spacing/2,
            target_xy[1] - lateral_vec[1] * arm_spacing/2,
            height
        ])
        right_pos = np.array([
            target_xy[0] + lateral_vec[0] * arm_spacing/2,
            target_xy[1] + lateral_vec[1] * arm_spacing/2,
            height
        ])
        
        target_ori = human_ori + np.pi
        
        left_pose = self.create_pose(left_pos, target_ori)
        right_pose = self.create_pose(right_pos, target_ori)
        
        return [left_pose, right_pose]
    
    def create_shoulder_based_poses(self, joints, offset_forward, offset_down=0.1):
        """Create poses based on shoulder positions (10cm below)"""
        r_shoulder, l_shoulder = joints[2], joints[3]
        
        shoulder_vec = l_shoulder - r_shoulder
        forward_dir = np.cross(shoulder_vec, np.array([0.0, 0.0, 1.0]))
        forward_norm = np.linalg.norm(forward_dir[:2])
        
        if forward_norm > 1e-6:
            direction = forward_dir / np.linalg.norm(forward_dir)
        else:
            direction = np.array([1.0, 0.0, 0.0])
        
        left_center = r_shoulder.copy()  # Robot left = human right
        left_center[2] -= offset_down
        
        right_center = l_shoulder.copy()  # Robot right = human left
        right_center[2] -= offset_down
        
        # Adjust if spacing is too narrow
        min_arm_spacing = 0.4
        current_spacing = np.linalg.norm(right_center - left_center)
        if current_spacing < min_arm_spacing:
            adjust_amount = (min_arm_spacing - current_spacing) / 2.0
            lateral_dir = np.array([-direction[1], direction[0], 0.0])
            lateral_dir = lateral_dir / np.linalg.norm(lateral_dir)
            left_center -= lateral_dir * adjust_amount
            right_center += lateral_dir * adjust_amount
        
        left_center += direction * offset_forward
        right_center += direction * offset_forward
        
        target_ori = np.arctan2(-direction[1], -direction[0])
        
        left_pose = self.create_pose(left_center, target_ori)
        right_pose = self.create_pose(right_center, target_ori)
        
        return [left_pose, right_pose]
    
    def call_joint_lock(self, lock: bool):
        """
        Call joint lock service to lock/unlock robot joints.
        
        Args:
            lock (bool): True to lock joints, False to unlock
        
        Returns:
            bool: True if service call succeeded, False otherwise
        """
        if not self.node._joint_lock_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Joint lock service not available')
            return False
        
        request = SetJointLock.Request()
        request.lock = lock
        
        future = self.node._joint_lock_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'Joint lock: {response.message}')
                return True
            else:
                self.node.get_logger().error(f'Joint lock failed: {response.message}')
                return False
        else:
            self.node.get_logger().error('Joint lock service call timed out')
            return False
    
    def call_move_to_standby(self):
        """
        Call move to standby action (non-blocking).
        
        Returns:
            bool: True if goal was sent, False otherwise
        """
        if not self.node._standby_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('Move to standby action server not available')
            return False
        
        goal_msg = MoveToStandby.Goal()
        
        self.node.get_logger().info('Sending move to standby goal...')
        send_goal_future = self.node._standby_action_client.send_goal_async(goal_msg)
        
        # Note: In BT, we send the goal and return. The action executes asynchronously.
        # For full feedback handling, you would need to implement callbacks.
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=5.0)
        
        if send_goal_future.done():
            goal_handle = send_goal_future.result()
            if goal_handle.accepted:
                self.node.get_logger().info('Move to standby goal accepted')
                return True
            else:
                self.node.get_logger().error('Move to standby goal rejected')
                return False
        else:
            self.node.get_logger().error('Failed to send move to standby goal')
            return False
    
    def publish_target_poses(self, poses):
        """Publish target EEF poses"""
        if poses is None:
            return
        
        pose_array = PoseArray()
        pose_array.header.stamp = self.node.get_clock().now().to_msg()
        pose_array.header.frame_id = "odom"
        pose_array.poses = poses
        
        self.node._planner_target_pub.publish(pose_array)
    
    def publish_mode(self, mode):
        """Publish current mode"""
        mode_msg = String()
        mode_msg.data = mode
        self.node._planner_mode_pub.publish(mode_msg)
    
    def enter_mode(self, mode_name):
        """Common mode entry logic"""
        self.node.get_logger().info(f'[Planner BT] Entering {mode_name.upper()} mode')
        self.blackboard.set('planner_mode', mode_name)
        self.feedback_message = f"In {mode_name.upper()} mode"
    
    def check_mode_transition(self, current_mode):
        """Check if a mode transition has been requested"""
        mode_requested = self.blackboard.get('planner_mode_requested')
        if mode_requested and mode_requested != current_mode:
            self.node.get_logger().info(f'[Planner BT] Mode change from {current_mode.upper()} to {mode_requested.upper()}')
            return py_trees.common.Status.SUCCESS
        return None
    
    def calculate_pose_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        return np.linalg.norm(np.array([
            pose1.position.x - pose2.position.x,
            pose1.position.y - pose2.position.y,
            pose1.position.z - pose2.position.z
        ]))
    
    def check_target_reached(self, current_poses, target_poses, threshold):
        """Check if both end-effectors have reached their targets"""
        if current_poses is None or target_poses is None:
            return False
        
        left_dist = self.calculate_pose_distance(current_poses[0], target_poses[0])
        right_dist = self.calculate_pose_distance(current_poses[1], target_poses[1])
        
        return left_dist < threshold and right_dist < threshold, left_dist, right_dist
    
    def manage_joint_lock(self, lock, context=""):
        """
        Manage joint locking with logging.
        
        Args:
            lock (bool): True to lock, False to unlock
            context (str): Context message for logging
        
        Returns:
            bool: True if successful, False otherwise
        """
        action = "Locking" if lock else "Unlocking"
        self.node.get_logger().info(f'[Planner BT] {action} joints {context}')
        
        if self.call_joint_lock(lock):
            return True
        else:
            self.node.get_logger().warn(f'[Planner BT] Failed to {action.lower()} joints')
            return False


class RestModeAction(PlannerState):
    """Rest mode: Robot at rest position"""
    
    def __init__(self, name, node, blackboard):
        super().__init__(name, node, blackboard)
    
    def initialise(self):
        """Initialize rest mode"""
        self.enter_mode('rest')
    
    def update(self):
        """Execute rest mode"""
        # Get or set rest target
        rest_target = self.blackboard.get('planner_rest_target')
        if rest_target is None:
            current_poses = self.blackboard.get('planner_current_poses')
            if current_poses is not None:
                self.blackboard.set('planner_rest_target', current_poses)
                self.node.get_logger().info('[Planner BT] Set rest pose from current position')
                rest_target = current_poses
            else:
                # Wait for current poses
                return py_trees.common.Status.RUNNING
        else:
            # Rest target already exists, automatically transition to approach mode
            self.node.get_logger().info('[Planner BT] Rest target set - transitioning to APPROACH mode')
            self.blackboard.set('planner_mode_requested', 'approach')
            return py_trees.common.Status.SUCCESS
        
        # Publish rest target and mode
        self.publish_target_poses(rest_target)
        self.publish_mode('rest')
        
        # Check for mode change request
        transition = self.check_mode_transition('rest')
        if transition:
            return transition
        
        return py_trees.common.Status.RUNNING


class ApproachModeAction(PlannerState):
    """Approach mode: Move to position in front of human"""
    
    def __init__(self, name, node, blackboard, approach_position_threshold=0.1, target_reached_threshold=0.1):
        super().__init__(name, node, blackboard)
        self.approach_position_threshold = approach_position_threshold
        self.target_reached_threshold = target_reached_threshold
    
    def initialise(self):
        """Initialize approach mode"""
        self.enter_mode('approach')
    
    def update(self):
        """Execute approach mode"""
        joints = self.blackboard.get('planner_joints')
        human_pos = self.blackboard.get('planner_human_pos')
        human_ori = self.blackboard.get('planner_human_ori')
        rest_target = self.blackboard.get('planner_rest_target')
        current_poses = self.blackboard.get('planner_current_poses')
        
        if joints is None or human_pos is None or human_ori is None or rest_target is None:
            return py_trees.common.Status.RUNNING
        
        # Calculate approach target
        rest_height = (rest_target[0].position.z + rest_target[1].position.z) / 2.0
        _approach_target = self.create_front_position_poses(human_pos, human_ori, distance=0.8, height=rest_height)
        
        # Filter jitter
        approach_target = self.blackboard.get('planner_approach_target')
        if approach_target is None:
            self.blackboard.set('planner_approach_target', _approach_target)
            approach_target = _approach_target
        else:
            # Check if change is significant
            left_dist = self.calculate_pose_distance(_approach_target[0], approach_target[0])
            right_dist = self.calculate_pose_distance(_approach_target[1], approach_target[1])
            
            if left_dist > self.approach_position_threshold or right_dist > self.approach_position_threshold:
                self.blackboard.set('planner_approach_target', _approach_target)
                approach_target = _approach_target
        
        # Publish approach target and mode
        self.publish_target_poses(approach_target)
        self.publish_mode('approach')
        
        # Check if current poses are close enough to approach target
        reached, left_dist, right_dist = self.check_target_reached(current_poses, approach_target, self.target_reached_threshold)
        if reached:
            self.node.get_logger().info(
                f'[Planner BT] Approach target reached (L: {left_dist:.3f}m, R: {right_dist:.3f}m) - '
                f'transitioning to WAIT_FOR_ARM_RAISE mode'
            )
            self.blackboard.set('planner_mode_requested', 'wait_for_arm_raise')
            return py_trees.common.Status.SUCCESS
        
        # Check for manual mode change request
        transition = self.check_mode_transition('approach')
        if transition:
            return transition
        
        return py_trees.common.Status.RUNNING


class WaitForArmRaiseModeAction(PlannerState):
    """Wait for arm raise mode: Instruct user to raise arm and wait for detection"""
    
    def __init__(self, name, node, blackboard, arm_raise_threshold=0.7):
        super().__init__(name, node, blackboard)
        self.arm_raise_threshold = arm_raise_threshold
        self.instruction_logged = False
    
    def initialise(self):
        """Initialize wait for arm raise mode"""
        self.enter_mode('wait_for_arm_raise')
        self.instruction_logged = False
    
    def update(self):
        """Execute wait for arm raise mode"""
        # Log instruction once per entry into this mode
        if not self.instruction_logged:
            self.send_instruction()
        
        joints = self.blackboard.get('planner_joints')
        
        if joints is None:
            self.node.get_logger().info('[Planner BT] Waiting for joint data...')
            return py_trees.common.Status.RUNNING
        
        # Check the x difference between two hands to determine raised arm
        left_hand = joints[7]
        right_hand = joints[6]
        
        if abs(left_hand[0] - right_hand[0]) > self.arm_raise_threshold:
            self.node.get_logger().info(
                f'[Planner BT] Arm raised detected (R: {right_hand[0]:.2f}m, L: {left_hand[0]:.2f}m) - '
                f'transitioning to INSERT mode'
            )
            # Automatically transition to insert mode
            self.blackboard.set('planner_mode_requested', 'insert')
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f'[Planner BT] Waiting for arm raise... {abs(left_hand[0] - right_hand[0])}')
        
        # Keep approach target published while waiting
        approach_target = self.blackboard.get('planner_approach_target')
        if approach_target is not None:
            self.publish_target_poses(approach_target)
        
        self.publish_mode('wait_for_arm_raise')
        
        # Check for manual mode change request
        transition = self.check_mode_transition('wait_for_arm_raise')
        if transition:
            return transition
        
        return py_trees.common.Status.RUNNING
    
    def send_instruction(self):
        """Send instruction to user (could be extended to TTS or UI)"""
        self.node.get_logger().info('[Planner BT] !!!!!!!!!!!!!!!!!!!! Please raise your arm to continue.')
        self.instruction_logged = True


class InsertModeAction(PlannerState):
    """Insert mode: Move to shoulder-based position for insertion"""
    
    def __init__(self, name, node, blackboard, target_reached_threshold=0.05, offset_forward=0.1):
        super().__init__(name, node, blackboard)
        self.target_reached_threshold = target_reached_threshold
        self.offset_forward = offset_forward # Move forward by 10cm for insertion
        self.offset_down = 0.1  # Lower target by 20cm for insertion
    
    def initialise(self):
        """Initialize insert mode"""
        self.enter_mode('insert')
    
    def update(self):
        """Execute insert mode"""
        joints = self.blackboard.get('planner_joints')
        current_poses = self.blackboard.get('planner_current_poses')
        
        if joints is None:
            return py_trees.common.Status.RUNNING
        
        # Generate shoulder-based target poses
        insert_target = self.create_shoulder_based_poses(joints, offset_forward=self.offset_forward, offset_down=self.offset_down)

        # Publish insert target and mode
        self.publish_target_poses(insert_target)
        self.publish_mode('insert')
        
        # Check if current poses are close enough to insert target
        reached, left_dist, right_dist = self.check_target_reached(current_poses, insert_target, self.target_reached_threshold)
        if reached:
            self.node.get_logger().info(
                f'[Planner BT] Insert target reached (L: {left_dist:.3f}m, R: {right_dist:.3f}m) - '
                f'transitioning to WAIT_FOR_REST mode'
            )
            self.blackboard.set('planner_mode_requested', 'wait_for_rest')
            return py_trees.common.Status.SUCCESS
        
        # Check for manual mode change request
        transition = self.check_mode_transition('insert')
        if transition:
            return transition
        
        return py_trees.common.Status.RUNNING


class WaitForRestModeAction(PlannerState):
    """Wait for rest mode: Instruct user to rest arms on robot and wait for detection"""
    
    def __init__(self, name, node, blackboard, arm_lower_threshold=0.15):
        super().__init__(name, node, blackboard)
        self.arm_lower_threshold = arm_lower_threshold
        self.instruction_logged = False
        self.initial_hand_height = None
        self.joints_locked = False
    
    def initialise(self):
        """Initialize wait for rest mode"""
        self.enter_mode('wait_for_rest')
        self.instruction_logged = False
        self.initial_hand_height = None
        self.joints_locked = False
    
    def update(self):
        """Execute wait for rest mode"""
        # Log instruction once per entry into this mode
        if not self.instruction_logged:
            self.send_instruction()

            # Lock joints immediately to hold current position
            self.joints_locked = self.manage_joint_lock(lock=True, context="to hold current position")
        
        joints = self.blackboard.get('planner_joints')
        
        if joints is None:
            self.node.get_logger().info('[Planner BT] Waiting for joint data...')
            return py_trees.common.Status.RUNNING
        
        # Get hand positions
        left_hand = joints[7]
        right_hand = joints[6]
        
        # Record initial hand height on first valid reading
        if self.initial_hand_height is None:
            # Average Z height of both hands
            self.initial_hand_height = (left_hand[2] + right_hand[2]) / 2.0
            self.node.get_logger().info(f'[Planner BT] Initial hand height recorded: {self.initial_hand_height:.3f}m')
        
        # Calculate current average hand height
        current_hand_height = (left_hand[2] + right_hand[2]) / 2.0
        height_drop = self.initial_hand_height - current_hand_height
        
        # Check if hands have lowered significantly (user resting on robot)
        if height_drop > self.arm_lower_threshold:
            self.node.get_logger().info(
                f'[Planner BT] Arms lowered detected (drop: {height_drop:.3f}m) - '
                f'transitioning to ASSIST mode'
            )

            # Unlock joints before transitioning
            self.manage_joint_lock(lock=False, context="before transitioning to ASSIST mode")
            self.joints_locked = False

            # Automatically transition to assist mode
            self.blackboard.set('planner_mode_requested', 'assist')
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(
                f'[Planner BT] Waiting for arms to rest... (current drop: {height_drop:.3f}m, '
                f'threshold: {self.arm_lower_threshold:.3f}m)'
            )
        
        # Keep insert target published while waiting
        # Generate shoulder-based target poses for current position
        insert_target = self.create_shoulder_based_poses(joints, offset_forward=0.1, offset_down=0.2)
        if insert_target is not None:
            self.publish_target_poses(insert_target)
        
        self.publish_mode('wait_for_rest')
        
        # Check for manual mode change request
        transition = self.check_mode_transition('wait_for_rest')
        if transition:
            # Unlock joints before transitioning
            self.manage_joint_lock(lock=False, context="before manual mode change")
            self.joints_locked = False
            return transition
        
        return py_trees.common.Status.RUNNING
    
    def send_instruction(self):
        """Send instruction to user (could be extended to TTS or UI)"""
        self.node.get_logger().info('[Planner BT] !!!!!!!!!!!!!!!!!!!! Please rest your arms on the robot.')
        self.instruction_logged = True


class AssistModeAction(PlannerState):
    """Assist mode: Follow shoulder targets"""
    
    def __init__(self, name, node, blackboard, standing_threshold=1.05, height_change_threshold=0.02, stability_duration=2.0):
        super().__init__(name, node, blackboard)
        self.standing_threshold = standing_threshold  # Height threshold in meters to detect standing
        self.height_change_threshold = height_change_threshold  # Maximum height change to consider stable (in meters)
        self.stability_duration = stability_duration  # Duration of stability required (in seconds)
        self.initial_torso_height = None
        self.previous_torso_height = None
        self.stable_start_time = None
    
    def initialise(self):
        """Initialize assist mode"""
        self.enter_mode('assist')
        self.initial_torso_height = None
        self.previous_torso_height = None
        self.stable_start_time = None
    
    def is_human_standing(self, joints):
        """
        Check if human is standing based on torso height.
        
        Two criteria:
        1. Torso height is above the standing threshold
        2. Torso height has been stable (changing less than threshold) for continuous 2 seconds
        
        Args:
            joints: Joint positions array (9x3), where joints[8] is the torso
        
        Returns:
            bool: True if torso height is above threshold AND has been stable for 2 seconds
        """
        if joints is None:
            return False
        
        torso_height = joints[8, 2]  # Z coordinate of torso
        current_time = self.node.get_clock().now()
        
        # Record initial height on first valid reading
        if self.initial_torso_height is None:
            self.initial_torso_height = torso_height
            self.previous_torso_height = torso_height
            self.node.get_logger().info(f'[Planner BT] Initial torso height recorded: {self.initial_torso_height:.3f}m')
        
        # Check if height is above standing threshold
        is_above_threshold = torso_height > self.standing_threshold
        
        # Check height stability
        if self.previous_torso_height is not None:
            height_change = abs(torso_height - self.previous_torso_height)
            
            if height_change < self.height_change_threshold:
                # Height is stable
                if self.stable_start_time is None:
                    # Start tracking stability
                    self.stable_start_time = current_time
                    self.node.get_logger().info(
                        f'[Planner BT] Height stable - starting stability timer (change: {height_change:.4f}m)'
                    )
                else:
                    # Check if stable duration has been reached
                    stable_duration = (current_time - self.stable_start_time).nanoseconds / 1e9
                    self.node.get_logger().info(
                        f'[Planner BT] Height: {torso_height:.3f}m, stable for {stable_duration:.2f}s '
                        f'(change: {height_change:.4f}m)'
                    )
                    
                    if stable_duration >= self.stability_duration:
                        if is_above_threshold:
                            self.node.get_logger().info(
                                f'[Planner BT] Standing detected: height={torso_height:.3f}m > {self.standing_threshold:.3f}m '
                                f'AND stable for {stable_duration:.2f}s'
                            )
                            return True
                        else:
                            self.node.get_logger().info(
                                f'[Planner BT] Height stable but below threshold ({torso_height:.3f}m < {self.standing_threshold:.3f}m)'
                            )
            else:
                # Height changed significantly - reset stability timer
                if self.stable_start_time is not None:
                    self.node.get_logger().info(
                        f'[Planner BT] Height unstable - resetting timer (change: {height_change:.4f}m > {self.height_change_threshold:.4f}m)'
                    )
                self.stable_start_time = None
        
        # Update previous height for next iteration
        self.previous_torso_height = torso_height
        
        return False
    
    def update(self):
        """Execute assist mode"""
        joints = self.blackboard.get('planner_joints')
        human_pos = self.blackboard.get('planner_human_pos')
        human_ori = self.blackboard.get('planner_human_ori')
        
        if joints is None or human_pos is None or human_ori is None:
            return py_trees.common.Status.RUNNING
        
        # Check if human is standing - if so, transition to wait_for_retreat mode
        if self.is_human_standing(joints):
            torso_height = joints[8, 2]
            self.node.get_logger().info(
                f'[Planner BT] Human standing detected (torso height: {torso_height:.3f}m, '
                f'threshold: {self.standing_threshold:.3f}m) - transitioning to WAIT_FOR_RETREAT mode'
            )
            self.blackboard.set('planner_mode_requested', 'wait_for_retreat')
            return py_trees.common.Status.SUCCESS
        
        # Store assist frame data for retreat reference
        self.blackboard.set('planner_last_assist_pos', human_pos.copy())
        self.blackboard.set('planner_last_assist_ori', human_ori)
        self.blackboard.set('planner_last_assist_height', joints[8, 2])  # Torso height
        
        # Generate shoulder-based targets
        assist_target = self.create_shoulder_based_poses(joints, offset_forward=0.1)
        
        # Publish assist target and mode
        self.publish_target_poses(assist_target)
        self.publish_mode('assist')
        
        # Check for manual mode change request
        transition = self.check_mode_transition('assist')
        if transition:
            return transition
        
        return py_trees.common.Status.RUNNING


class WaitForRetreatModeAction(PlannerState):
    """Wait for retreat mode: Instruct user to step back and wait for detection"""
    
    def __init__(self, name, node, blackboard, retreat_distance_threshold=0.2):
        super().__init__(name, node, blackboard)
        self.retreat_distance_threshold = retreat_distance_threshold
        self.instruction_logged = False
        self.initial_torso_position = None
        self.joints_locked = False
    
    def initialise(self):
        """Initialize wait for retreat mode"""
        self.enter_mode('wait_for_retreat')
        self.instruction_logged = False
        self.initial_torso_position = None
        self.joints_locked = False
    
    def update(self):
        """Execute wait for retreat mode"""
        # Log instruction once per entry into this mode
        if not self.instruction_logged:
            self.send_instruction()
        
        joints = self.blackboard.get('planner_joints')
        
        if joints is None:
            self.node.get_logger().info('[Planner BT] Waiting for joint data...')
            return py_trees.common.Status.RUNNING
        
        # Get torso position
        torso_pos = joints[8]  # [x, y, z]
        
        # Record initial torso position on first valid reading
        if self.initial_torso_position is None:
            self.initial_torso_position = torso_pos.copy()
            self.node.get_logger().info(
                f'[Planner BT] Initial torso position recorded: '
                f'x={self.initial_torso_position[0]:.3f}m, y={self.initial_torso_position[1]:.3f}m, '
                f'z={self.initial_torso_position[2]:.3f}m'
            )
            
            # Lock joints immediately to hold current position
            self.joints_locked = self.manage_joint_lock(lock=True, context="to hold current position")
        
        # Calculate horizontal distance from initial position (XY plane)
        horizontal_distance = np.linalg.norm(torso_pos[:2] - self.initial_torso_position[:2])
        
        # Check if user has retreated significantly
        if horizontal_distance > self.retreat_distance_threshold:
            self.node.get_logger().info(
                f'[Planner BT] User retreat detected (distance: {horizontal_distance:.3f}m) - '
                f'transitioning to RETREAT mode'
            )
            
            # Unlock joints before transitioning to retreat mode
            self.manage_joint_lock(lock=False, context="for retreat movement")
            self.joints_locked = False
            
            # Automatically transition to retreat mode
            self.blackboard.set('planner_mode_requested', 'retreat')
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(
                f'[Planner BT] Waiting for user to retreat... (current distance: {horizontal_distance:.3f}m, '
                f'threshold: {self.retreat_distance_threshold:.3f}m)'
            )
        
        self.publish_mode('wait_for_retreat')
        
        # Check for manual mode change request
        transition = self.check_mode_transition('wait_for_retreat')
        if transition:
            # Unlock joints if changing modes manually
            self.manage_joint_lock(lock=False, context="due to manual mode change")
            self.joints_locked = False
            return transition
        
        return py_trees.common.Status.RUNNING
    
    def send_instruction(self):
        """Send instruction to user (could be extended to TTS or UI)"""
        self.node.get_logger().info('[Planner BT] !!!!!!!!!!!!!!!!!!!! Please step back from the robot.')
        self.instruction_logged = True


class RetreatModeAction(PlannerState):
    """Retreat mode: Return to position behind last assist location"""
    
    def __init__(self, name, node, blackboard):
        super().__init__(name, node, blackboard)
        self.standby_called = False
    
    def initialise(self):
        """Initialize retreat mode"""
        self.enter_mode('retreat')
        self.standby_called = False
    
    def update(self):
        """Execute retreat mode"""
      
        # Lower body
        # Use rest target if available, otherwise calculate fallback position
        planner_rest_target = self.blackboard.get('planner_rest_target')
        if planner_rest_target is None:
            last_assist_pos = self.blackboard.get('planner_last_assist_pos')
            last_assist_ori = self.blackboard.get('planner_last_assist_ori')
            last_assist_height = self.blackboard.get('planner_last_assist_height')
            
            # Fallback to current human position if no assist data
            if last_assist_pos is None or last_assist_height is None:
                human_pos = self.blackboard.get('planner_human_pos')
                human_ori = self.blackboard.get('planner_human_ori')
                joints = self.blackboard.get('planner_joints')
                
                if human_pos is None or human_ori is None or joints is None:
                    return py_trees.common.Status.RUNNING
                
                planner_rest_target = self.create_front_position_poses(human_pos, human_ori, distance=2.0, height=1.0)
            else:
                planner_rest_target = self.create_front_position_poses(
                    last_assist_pos, last_assist_ori, distance=2.1, height=last_assist_height
                )
        
        # Publish retreat target and mode
        self.publish_target_poses(planner_rest_target)
        self.publish_mode('retreat')

        # Upper body
        # First, call move to standby if not already done
        if not self.standby_called:
            self.node.get_logger().info('[Planner BT] Moving to standby before retreat')
            if self.call_move_to_standby():
                self.standby_called = True
            else:
                self.node.get_logger().warn('[Planner BT] Failed to call move to standby, continuing anyway')
                self.standby_called = True  # Don't keep retrying
        
        # Check for mode change request
        transition = self.check_mode_transition('retreat')
        if transition:
            return transition
        
        return py_trees.common.Status.RUNNING


class ModeSelector(py_trees.behaviour.Behaviour):
    """
    Route to appropriate mode based on planner_mode_requested.
    This is a decorator/condition that determines which mode to execute.
    """
    
    def __init__(self, name, blackboard, target_mode):
        super().__init__(name)
        self.blackboard = blackboard
        self.target_mode = target_mode
    
    def update(self):
        """Check if this is the requested mode"""
        mode_requested = self.blackboard.get('planner_mode_requested')
        
        if mode_requested == self.target_mode:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


def create_planner_subtree(node, blackboard):
    """
    Create the planner behavior subtree.
    
    Structure:
        Planner Root (Selector with memory - remembers which mode is active)
        ├── Rest Mode (Sequence)
        │   ├── Is Rest Mode Requested?
        │   └── Execute Rest Mode
        ├── Approach Mode (Sequence)
        │   ├── Is Approach Mode Requested?
        │   └── Execute Approach Mode
        ├── Wait For Arm Raise Mode (Sequence)
        │   ├── Is Wait For Arm Raise Mode Requested?
        │   └── Execute Wait For Arm Raise Mode
        │       (Instruct user to raise arm and wait for detection)
        ├── Insert Mode (Sequence)
        │   ├── Is Insert Mode Requested?
        │   └── Execute Insert Mode
        │       (Move to shoulder-based position)
        ├── Wait For Rest Mode (Sequence)
        │   ├── Is Wait For Rest Mode Requested?
        │   └── Execute Wait For Rest Mode
        │       (Instruct user to rest arms and wait for detection)
        ├── Assist Mode (Sequence)
        │   ├── Is Assist Mode Requested?
        │   └── Execute Assist Mode
        ├── Wait For Retreat Mode (Sequence)
        │   ├── Is Wait For Retreat Mode Requested?
        │   └── Execute Wait For Retreat Mode
        │       (Instruct user to step back and wait for detection)
        └── Retreat Mode (Sequence)
            ├── Is Retreat Mode Requested?
            └── Execute Retreat Mode
    
    The Selector will:
    1. Try each mode sequence in order
    2. Each sequence checks if its mode is requested
    3. If yes, execute that mode (which returns RUNNING continuously)
    4. Mode changes are detected when the current mode action returns SUCCESS
    5. Selector then re-evaluates and switches to new mode
    """
    
    # Register and initialize blackboard keys
    for key in PlannerState.BLACKBOARD_KEYS:
        try:
            blackboard.register_key(key, access=py_trees.common.Access.WRITE)
        except KeyError:
            pass  # Already registered
    
    # Initialize blackboard values
    blackboard.set('planner_mode_requested', 'rest')
    blackboard.set('planner_mode', 'rest')
    for key in PlannerState.BLACKBOARD_KEYS[2:]:  # Skip 'planner_mode', 'planner_mode_requested'
        blackboard.set(key, None)
    
    # Helper function to create mode sequence
    def create_mode_sequence(mode_name, action_class, **action_kwargs):
        """Create a mode sequence with selector and action"""
        return Sequence(
            name=f"{mode_name.replace('_', ' ').title()} Mode Sequence",
            memory=True,
            children=[
                ModeSelector(f"Is {mode_name.replace('_', ' ').title()} Requested?", blackboard, mode_name),
                action_class(f"Execute {mode_name.replace('_', ' ').title()} Mode", node, blackboard, **action_kwargs)
            ]
        )
    
    # Create mode sequences
    mode_sequences = [
        create_mode_sequence('rest', RestModeAction),
        create_mode_sequence('approach', ApproachModeAction),
        create_mode_sequence('wait_for_arm_raise', WaitForArmRaiseModeAction),
        create_mode_sequence('insert', InsertModeAction),
        create_mode_sequence('wait_for_rest', WaitForRestModeAction),
        create_mode_sequence('assist', AssistModeAction),
        create_mode_sequence('wait_for_retreat', WaitForRetreatModeAction),
        create_mode_sequence('retreat', RetreatModeAction)
    ]
    
    # Create selector to choose mode
    planner_selector = Selector(
        name="Planner Mode Selector",
        memory=True,  # Remember which mode is active
        children=mode_sequences
    )
    
    # Wrap in Repeat decorator to keep it running forever
    # Otherwise, once a mode action (selector) returns SUCCESS, the planner would stop
    # The planner should never terminate - it runs continuously during manipulation
    planner_loop = py_trees.decorators.Repeat(
        name="Planner Loop (Always Running)",
        child=planner_selector,
        num_success=-1  # Repeat forever on success
    )
    
    return planner_loop
