#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
import numpy as np
import os
import threading
import sys


class MocapEnterPublisher(Node):
    def __init__(self, csv_file=None):
        super().__init__('mocap_enter_publisher')

        # Create publishers
        self.arm_publisher = self.create_publisher(PoseArray, '/demo_target_poses', 10)
        self.skeleton_marker_publisher = self.create_publisher(MarkerArray, '/skeleton_markers', 10)

        # Default to the inference_trajectory_gt.csv file
        if csv_file is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            csv_file = os.path.join(script_dir, 'inference_trajectory_gt.csv')

        # Load data
        self.df = pd.read_csv(csv_file)

        # Get first trajectory (traj_id = 0)
        all_traj_ids = self.df['traj_id'].unique()
        first_traj_id = all_traj_ids[0]
        self.traj_data = self.df[self.df['traj_id'] == first_traj_id].copy()

        if self.traj_data.empty:
            self.get_logger().error(f'No data found for trajectory {first_traj_id}')
            return

        self.get_logger().info(f'Loaded {len(self.traj_data)} frames for trajectory {first_traj_id}')

        # Position offsets
        self.x_offset = -0.8  # X-axis offset (1.5m to the left, negative x direction)
        self.y_offset = 1.5   # Y-axis offset (1.5m in positive y direction)

        # Reset frame index for easy iteration
        self.traj_data.reset_index(drop=True, inplace=True)
        self.current_frame = 0

        # Define skeleton joint names
        self.joint_names = [
            'Head', 'Neck', 'R_Shoulder', 'L_Shoulder',
            'R_Elbow', 'L_Elbow', 'R_Hand', 'L_Hand', 'Torso',
            'R_Hip', 'L_Hip', 'R_Knee', 'L_Knee', 'R_Foot', 'L_Foot'
        ]

        # Define skeleton connections
        self.connections = [
            ('Torso', 'Neck'), ('Neck', 'Head'),
            ('Neck', 'R_Shoulder'), ('Neck', 'L_Shoulder'),
            ('R_Shoulder', 'R_Elbow'), ('R_Elbow', 'R_Hand'),
            ('L_Shoulder', 'L_Elbow'), ('L_Elbow', 'L_Hand'),
            ('Torso', 'R_Hip'), ('Torso', 'L_Hip'),
            ('R_Hip', 'R_Knee'), ('R_Knee', 'R_Foot'),
            ('L_Hip', 'L_Knee'), ('L_Knee', 'L_Foot'),
            ('R_Hip', 'L_Hip')
        ]

        # Flag for running
        self.running = True

    def publish_next_frame(self):
        """Publish the next frame of data"""
        if self.current_frame >= len(self.traj_data):
            # Loop back to beginning
            self.current_frame = 0
            self.get_logger().info('Looping trajectory data')

        row = self.traj_data.iloc[self.current_frame]
        timestamp = self.get_clock().now().to_msg()

        # Publish arm poses
        self.publish_arm_poses(row, timestamp)

        # Publish skeleton markers
        self.publish_skeleton(row, timestamp)

        self.get_logger().info(f'Published frame {self.current_frame + 1}/{len(self.traj_data)}')
        self.current_frame += 1

    def publish_arm_poses(self, row, timestamp):
        # Get raw positions from CSV (convert mm to m)
        left1_pos = np.array([row['Left_L1_x']/1000.0, row['Left_L1_y']/1000.0, row['Left_L1_z']/1000.0])
        left2_pos = np.array([row['Left_L2_x']/1000.0, row['Left_L2_y']/1000.0, row['Left_L2_z']/1000.0])
        right1_pos = np.array([row['Right_R1_x']/1000.0, row['Right_R1_y']/1000.0, row['Right_R1_z']/1000.0])
        right2_pos = np.array([row['Right_R2_x']/1000.0, row['Right_R2_y']/1000.0, row['Right_R2_z']/1000.0])

        # Apply rotation around X-axis by -90 degrees: (x, y, z) -> (x, -z, y)
        left1_pos = self.rotate_x_minus_90(left1_pos)
        left2_pos = self.rotate_x_minus_90(left2_pos)
        right1_pos = self.rotate_x_minus_90(right1_pos)
        right2_pos = self.rotate_x_minus_90(right2_pos)

        # Apply position offsets
        left1_pos[0] += self.x_offset
        left1_pos[1] += self.y_offset
        left2_pos[0] += self.x_offset
        left2_pos[1] += self.y_offset
        right1_pos[0] += self.x_offset
        right1_pos[1] += self.y_offset
        right2_pos[0] += self.x_offset
        right2_pos[1] += self.y_offset

        # Calculate arm centers
        left_center = (left1_pos + left2_pos) / 2.0
        right_center = (right1_pos + right2_pos) / 2.0

        # Calculate orientations (direction from point 1 to point 2)
        left_direction = left2_pos - left1_pos
        right_direction = right2_pos - right1_pos

        # Create poses
        left_arm_pose = Pose()
        left_arm_pose.position = Point(x=float(left_center[0]),
                                      y=float(left_center[1]),
                                      z=float(left_center[2]))
        left_arm_pose.orientation = self.vector_to_quaternion(left_direction)

        right_arm_pose = Pose()
        right_arm_pose.position = Point(x=float(right_center[0]),
                                       y=float(right_center[1]),
                                       z=float(right_center[2]))
        right_arm_pose.orientation = self.vector_to_quaternion(right_direction)

        # Create and publish PoseArray
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = "map"
        pose_array.poses = [left_arm_pose, right_arm_pose]

        self.arm_publisher.publish(pose_array)

    def publish_skeleton(self, row, timestamp):
        # Extract joint positions
        joint_poses = {}
        for joint_name in self.joint_names:
            pos = np.array([row[f'{joint_name}_gt_x'] / 1000.0,
                           row[f'{joint_name}_gt_y'] / 1000.0,
                           row[f'{joint_name}_gt_z'] / 1000.0])
            pos = self.rotate_x_minus_90(pos)
            # Apply position offsets
            pos[0] += self.x_offset
            pos[1] += self.y_offset
            joint_poses[joint_name] = pos

        # Create marker array
        marker_array = MarkerArray()

        # Create sphere markers for joints
        for i, (joint_name, pos) in enumerate(joint_poses.items()):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.ns = "skeleton_joints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05

            # Color based on body part
            if 'Head' in joint_name or 'Neck' in joint_name:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
            elif 'L' in joint_name:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            elif 'R' in joint_name:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Create line markers for connections
        for i, (parent, child) in enumerate(self.connections):
            if parent in joint_poses and child in joint_poses:
                line = Marker()
                line.header.frame_id = "map"
                line.header.stamp = timestamp
                line.ns = "skeleton_bones"
                line.id = i
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                parent_pos = joint_poses[parent]
                child_pos = joint_poses[child]
                line.points = [
                    Point(x=float(parent_pos[0]), y=float(parent_pos[1]), z=float(parent_pos[2])),
                    Point(x=float(child_pos[0]), y=float(child_pos[1]), z=float(child_pos[2]))
                ]
                line.scale.x = 0.02

                # Color based on body part
                if 'L' in parent or 'L' in child:
                    line.color.r, line.color.g, line.color.b = 1.0, 0.0, 0.0
                elif 'R' in parent or 'R' in child:
                    line.color.r, line.color.g, line.color.b = 0.0, 0.0, 1.0
                else:
                    line.color.r, line.color.g, line.color.b = 0.0, 1.0, 0.0
                line.color.a = 0.8
                marker_array.markers.append(line)

        self.skeleton_marker_publisher.publish(marker_array)

    def rotate_x_minus_90(self, pos):
        """Rotate point around X-axis by -90 degrees: (x, y, z) -> (x, -z, y)"""
        x, y, z = pos
        return np.array([x, -z, y])

    def vector_to_quaternion(self, direction_vector):
        """Convert a direction vector to a quaternion"""
        direction = direction_vector / np.linalg.norm(direction_vector)
        forward = np.array([1.0, 0.0, 0.0])

        axis = np.cross(forward, direction)
        axis_length = np.linalg.norm(axis)

        if axis_length < 1e-6:
            if np.dot(forward, direction) > 0:
                return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            else:
                return Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

        axis = axis / axis_length
        angle = np.arccos(np.clip(np.dot(forward, direction), -1.0, 1.0))

        half_angle = angle / 2.0
        sin_half = np.sin(half_angle)
        cos_half = np.cos(half_angle)

        return Quaternion(
            x=float(axis[0] * sin_half),
            y=float(axis[1] * sin_half),
            z=float(axis[2] * sin_half),
            w=float(cos_half)
        )


def input_thread(publisher):
    """Thread to handle keyboard input"""
    print("\nPress Enter to publish next frame, or type 'q' and Enter to quit...")
    while publisher.running:
        try:
            user_input = input()
            if user_input.lower() == 'q':
                publisher.running = False
                break
            # Empty input (just Enter key) publishes next frame
            publisher.publish_next_frame()
        except EOFError:
            break
        except KeyboardInterrupt:
            publisher.running = False
            break


def main(args=None):
    rclpy.init(args=args)

    publisher = MocapEnterPublisher()

    if publisher.traj_data.empty:
        return

    print("Publishing from inference_trajectory_gt.csv (first trajectory)")
    print("Topics:")
    print("  - /demo_target_poses: Arm poses")
    print("  - /skeleton_markers: Skeleton visualization")
    print(f"\nTotal frames: {len(publisher.traj_data)}")

    # Start input thread
    input_t = threading.Thread(target=input_thread, args=(publisher,), daemon=True)
    input_t.start()

    try:
        while rclpy.ok() and publisher.running:
            rclpy.spin_once(publisher, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        publisher.running = False
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
