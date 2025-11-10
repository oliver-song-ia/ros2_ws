#!/usr/bin/env python3
"""
Robot Planner with Manual State Machine Control
- Manages robot behavior through 4 modes: rest, approach, assist, retreat
- Publishes /mode (String) and /target_eef_poses (PoseArray)
- State transitions controlled via /set_mode topic (String)

Modes:
- rest: Robot at 2m in front, initialization
- approach: Move to 0.4m front, height at shoulder level
- assist: Follow shoulder targets
- retreat: Return to 2.1m front

Commands:
- Send "rest", "approach", "assist", or "retreat" to /set_mode topic
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from std_msgs.msg import String
import numpy as np
from collections import deque

class RobotPlannerNew(Node):
    def __init__(self):
        super().__init__('robot_planner_new')

        # State machine
        self.mode = 'rest'  # rest, approach, assist, retreat
        self.valid_modes = ['rest', 'approach', 'assist', 'retreat']
        self.last_assist_position = None  # Human position at last assist frame
        self.last_assist_orientation = None  # Human orientation at last assist frame
        self.last_assist_height = None  # Height at last assist frame

        # Rest mode fixed target
        self.current_poses = None
        self.rest_target_poses = None  # Fixed target poses in rest mode
        self.approach_target_poses = None

        # Jitter filtering threshold (meters)
        self.approach_position_threshold = 0.1  # 10cm threshold for approach mode

        # Publishers
        self.target_eef_publisher = self.create_publisher(
            PoseArray, '/target_eef_poses', 10)
        self.mode_publisher = self.create_publisher(
            String, '/mode', 10)

        # Subscribers
        self.pose_subscriber = self.create_subscription(
            MarkerArray, '/pose_detection', self.pose_callback, 10)
        self.mode_command_subscriber = self.create_subscription(
            String, '/set_mode', self.mode_command_callback, 10)
        self.current_eef_subscriber = self.create_subscription(
            PoseArray, '/current_poses', self.current_eef_callback, 10)

        self.get_logger().info('Robot Planner with Manual State Machine initialized')
        self.get_logger().info(f'Initial mode: {self.mode}')
        self.get_logger().info('Send commands to /set_mode topic: "rest", "approach", "assist", or "retreat"')

    def current_eef_callback(self, msg):
        """Receive current EEF poses"""
        if len(msg.poses) >= 2:
            self.current_poses = [msg.poses[0], msg.poses[1]]
        else:
            self.get_logger().warning(f'Received current_eef_poses with {len(msg.poses)} poses, expected 2')

    def mode_command_callback(self, msg):
        """Handle mode change commands from /set_mode topic"""
        requested_mode = msg.data.lower().strip()
        
        if requested_mode in self.valid_modes:
            old_mode = self.mode
            self.mode = requested_mode
            
            # Clear target
            if old_mode == 'retreat' and self.mode != 'retreat':
                self.rest_target_poses = None
            
            if old_mode == 'approach' and self.mode != 'approach':
                self.approach_target_poses = None
            
            self.get_logger().info(f'Mode changed: {old_mode} -> {self.mode}')
            self.publish_mode()
        else:
            self.get_logger().warn(f'Invalid mode requested: "{requested_mode}". Valid modes: {self.valid_modes}')

    def pose_callback(self, msg):
        """Process incoming pose and generate targets based on current mode"""
        try:
            joints = self.extract_joint_positions(msg)
            if joints is None:
                return

            # Get human state
            human_pos_xy = joints[8, :2]  # Torso XY

            # Calculate human orientation
            r_shoulder, l_shoulder = joints[2], joints[3]
            human_ori = self.calculate_human_orientation(r_shoulder, l_shoulder)

            # Store assist frame data when in assist mode (for retreat reference)
            if self.mode == 'assist':
                self.last_assist_position = human_pos_xy.copy()
                self.last_assist_orientation = human_ori
                self.last_assist_height = joints[8, 2]  # Torso height

            # Generate and publish target based on current mode
            target_poses = self.generate_target_poses(human_pos_xy, human_ori, joints)
            self.publish_target_eef_poses(target_poses)

            # Publish current mode
            self.publish_mode()

        except Exception as e:
            import traceback
            traceback.print_exc()
            self.get_logger().error(f'Error in pose callback: {e}')

    def generate_target_poses(self, human_pos_xy, human_ori, joints):
        """
        Generate target EEF poses based on current mode

        Returns:
            [left_pose, right_pose]: list of 2 Pose objects
        """
        if self.mode == 'rest':
            # Use fixed rest target position
            if self.rest_target_poses is not None:
                return self.rest_target_poses
            elif self.current_poses is not None:
                self.rest_target_poses = self.current_poses
                self.get_logger().info("Set rest pose")
                return self.rest_target_poses
            else:
                return None
                # # Fallback: set rest pose based on detected human pose
                # self.get_logger().warn('Rest target not set')
                # self.rest_target_poses = self.create_front_position_poses(human_pos_xy, human_ori, distance=1.5, height=1.0)
                # return self.rest_target_poses

        elif self.mode == 'approach':
            # 0.4m in front, height at rest pose
            # Calculate shoulder height (average of left and right shoulders)
            r_shoulder, l_shoulder = joints[2], joints[3]
            # shoulder_height = (r_shoulder[2] + l_shoulder[2]) / 2.0 - 0.1  # 10cm below shoulders
            rest_height = (self.rest_target_poses[0].position.z + self.rest_target_poses[1].position.z) / 2.0
            _approach_target_poses = self.create_front_position_poses(human_pos_xy, human_ori, distance=0.8, height=rest_height) # was shoulder_height

            # Filter jitter: only update if change is significant
            if self.approach_target_poses is None:
                self.approach_target_poses = _approach_target_poses
            else:
                # Check if new poses are close enough to current ones
                left_dist = np.linalg.norm(np.array([
                    _approach_target_poses[0].position.x - self.approach_target_poses[0].position.x,
                    _approach_target_poses[0].position.y - self.approach_target_poses[0].position.y,
                    _approach_target_poses[0].position.z - self.approach_target_poses[0].position.z
                ]))
                right_dist = np.linalg.norm(np.array([
                    _approach_target_poses[1].position.x - self.approach_target_poses[1].position.x,
                    _approach_target_poses[1].position.y - self.approach_target_poses[1].position.y,
                    _approach_target_poses[1].position.z - self.approach_target_poses[1].position.z
                ]))
                
                # Update only if either arm moved significantly
                if left_dist > self.approach_position_threshold or right_dist > self.approach_position_threshold:
                    self.approach_target_poses = _approach_target_poses

            return self.approach_target_poses

        elif self.mode == 'assist':
            # Shoulder-based targets
            return self.create_shoulder_based_poses(joints, distance=0.1)

        elif self.mode == 'retreat':
            # Return to last assist position + 2.1m, using last assist height
            if self.last_assist_position is not None and self.last_assist_height is not None:
                return self.create_front_position_poses(
                    self.last_assist_position, self.last_assist_orientation,
                    distance=2.1, height=self.last_assist_height)
            else:
                # Fallback
                return self.create_front_position_poses(human_pos_xy, human_ori, distance=2.0, height=1.0)

    def create_front_position_poses(self, human_pos_xy, human_ori, distance, height):
        """Create poses at fixed distance in front of human"""
        # Target position
        target_xy = human_pos_xy + distance * np.array([np.cos(human_ori), np.sin(human_ori)])

        # Arm spacing: 45cm for rest/approach/retreat modes
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

        # Orientation: facing human (opposite direction)
        target_ori = human_ori + np.pi

        # No roll rotation
        left_pose = self.create_pose(left_pos, target_ori)
        right_pose = self.create_pose(right_pos, target_ori)

        return [left_pose, right_pose]

    def create_shoulder_based_poses(self, joints, distance):
        """Create poses based on shoulder positions (10cm below)"""
        r_shoulder, l_shoulder = joints[2], joints[3]

        # Calculate direction
        shoulder_vec = l_shoulder - r_shoulder
        forward_dir = np.cross(shoulder_vec, np.array([0.0, 0.0, 1.0]))
        forward_norm = np.linalg.norm(forward_dir[:2])

        if forward_norm > 1e-6:
            direction = forward_dir / np.linalg.norm(forward_dir)
        else:
            direction = np.array([1.0, 0.0, 0.0])

        # Arm parameters
        arm_length = 0.4
        offset_down = 0.1

        # Centers: 10cm below shoulders
        left_center = r_shoulder.copy()  # Robot left = human right
        left_center[2] -= offset_down

        right_center = l_shoulder.copy()  # Robot right = human left
        right_center[2] -= offset_down

        # if arm_spacing is too narrow, adjust to minimum
        min_arm_spacing = 0.4 # m
        current_spacing = np.linalg.norm(right_center - left_center)
        if current_spacing < min_arm_spacing:
            adjust_amount = (min_arm_spacing - current_spacing) / 2.0
            lateral_dir = np.array([-direction[1], direction[0], 0.0])
            lateral_dir = lateral_dir / np.linalg.norm(lateral_dir)
            left_center -= lateral_dir * adjust_amount
            right_center += lateral_dir * adjust_amount

        # Move points towards human by 'distance'
        left_center += direction * distance
        right_center += direction * distance

        # Orientation: towards human (opposite of direction)
        target_ori = np.arctan2(-direction[1], -direction[0])

        # No roll rotation
        left_pose = self.create_pose(left_center, target_ori)
        right_pose = self.create_pose(right_center, target_ori)

        return [left_pose, right_pose]

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
        """
        Create Pose from position, orientation (yaw), and local roll
        local_roll: rotation around the local forward axis (x-axis in end-effector frame)
        """
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])

        # First create quaternion for yaw rotation (around global Z)
        half_yaw = orientation * 0.5
        q_yaw = np.array([0.0, 0.0, np.sin(half_yaw), np.cos(half_yaw)])

        # Then create quaternion for local roll (around local X, which is the forward direction)
        half_roll = local_roll * 0.5
        q_roll = np.array([np.sin(half_roll), 0.0, 0.0, np.cos(half_roll)])

        # Combine: first apply yaw, then apply roll in the rotated frame
        # Quaternion multiplication: q_yaw * q_roll
        x1, y1, z1, w1 = q_yaw
        x2, y2, z2, w2 = q_roll

        pose.orientation = Quaternion(
            x=float(w1*x2 + x1*w2 + y1*z2 - z1*y2),
            y=float(w1*y2 - x1*z2 + y1*w2 + z1*x2),
            z=float(w1*z2 + x1*y2 - y1*x2 + z1*w2),
            w=float(w1*w2 - x1*x2 - y1*y2 - z1*z2)
        )
        return pose

    def extract_joint_positions(self, marker_array):
        """Extract joint positions from MarkerArray"""
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

    def publish_target_eef_poses(self, poses):
        """Publish target EEF poses"""
        if poses is None:
            return
        
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "odom"
        pose_array.poses = poses

        self.target_eef_publisher.publish(pose_array)

    def publish_mode(self):
        """Publish current mode"""
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_publisher.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        planner = RobotPlannerNew()

        print("\n" + "="*60)
        print("Robot Planner with Manual State Machine Started")
        print("="*60)
        print("Topics:")
        print("  - Subscribing to: /pose_detection (MarkerArray)")
        print("  - Subscribing to: /set_mode (String) - for mode commands")
        print("  - Publishing to: /target_eef_poses (PoseArray)")
        print("  - Publishing to: /mode (String)")
        print("")
        print("Manual State Control:")
        print("  Send mode commands to /set_mode topic:")
        print("    - 'rest': Robot at 2m in front")
        print("    - 'approach': Move to 0.4m, shoulder height")
        print("    - 'assist': Follow shoulder targets")
        print("    - 'retreat': Return to 2.1m from last assist position")
        print("")
        print("Example commands:")
        print("  ros2 topic pub /set_mode std_msgs/msg/String \"data: 'approach'\" --once")
        print("  ros2 topic pub /set_mode std_msgs/msg/String \"data: 'assist'\" --once")
        print("  ros2 topic pub /set_mode std_msgs/msg/String \"data: 'retreat'\" --once")
        print("  ros2 topic pub /set_mode std_msgs/msg/String \"data: 'rest'\" --once")
        print("="*60)
        print("Press Ctrl+C to stop...\n")

        rclpy.spin(planner)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'planner' in locals():
            planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
