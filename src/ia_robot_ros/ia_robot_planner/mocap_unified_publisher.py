#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
import pandas as pd
import numpy as np
import os


class MocapUnifiedPublisher(Node):
    def __init__(self, csv_file=None, frame_rate=10.0):
        super().__init__('mocap_unified_publisher')

        # Create publisher for demo_target_poses
        self.arm_publisher = self.create_publisher(PoseArray, '/demo_target_poses', 10)

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

        # Calculate publish interval based on frame rate
        self.publish_interval = 1.0 / frame_rate

        # Reset frame index for easy iteration
        self.traj_data.reset_index(drop=True, inplace=True)
        self.current_frame = 0

        # Create timer for publishing
        self.timer = self.create_timer(self.publish_interval, self.publish_data)

    def publish_data(self):
        if self.current_frame >= len(self.traj_data):
            # Loop back to beginning
            self.current_frame = 0
            self.get_logger().info('Looping trajectory data')

        row = self.traj_data.iloc[self.current_frame]

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
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        pose_array.poses = [left_arm_pose, right_arm_pose]

        self.arm_publisher.publish(pose_array)

        self.current_frame += 1

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


def main(args=None):
    rclpy.init(args=args)

    publisher = MocapUnifiedPublisher()

    if publisher.traj_data.empty:
        return

    print("Publishing demo_target_poses from inference_trajectory_gt.csv (first trajectory)")
    print("Press Ctrl+C to stop...")

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()