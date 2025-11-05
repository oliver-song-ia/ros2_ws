#handle anywhere demo

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from pose_estimation import *
from geometry_msgs.msg import PoseStamped
# import tf_transformations  # For quaternion handling if needed



class RGBDOnceListener(Node):
    def __init__(self):
        super().__init__('rgbd_once_listener')
        self.bridge = CvBridge()

        self.rgb_image = None
        self.depth_image = None
        self.got_rgb = False
        self.got_depth = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.media_pipe_model = construct_meida_pipe()

        # Subscriptions
        self.rgb_sub = self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Timer to periodically check if data is ready
        self.timer = self.create_timer(0.5, self.check_ready)
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def rgb_callback(self, msg):
        if not self.got_rgb:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.got_rgb = True
            self.get_logger().info('RGB image received.')

    def depth_callback(self, msg):
        if not self.got_depth:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.got_depth = True
            self.get_logger().info('Depth image received.')

    def check_ready(self):
        if self.got_rgb and self.got_depth:
            self.timer.cancel()
            self.process_and_print()

    def process_and_print(self):
        print("RGB Image shape:", self.rgb_image.shape, "dtype:", self.rgb_image.dtype)
        print("Depth Image shape:", self.depth_image.shape, "dtype:", self.depth_image.dtype)

        # Lookup transform from dummy_camera to map
        transform: TransformStamped = self.tf_buffer.lookup_transform(
            'map', 'dummy_camera', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))

        t = transform.transform.translation
        q = transform.transform.rotation

        print("\nTransform from dummy_camera to map:")
        print("Translation: x={:.3f}, y={:.3f}, z={:.3f}".format(t.x, t.y, t.z))
        print("Rotation (quaternion): x={:.4f}, y={:.4f}, z={:.4f}, w={:.4f}".format(q.x, q.y, q.z, q.w))

        r = R.from_quat([q.x, q.y, q.z, q.w])
        rpy_rad = r.as_euler('xyz', degrees=False)
        rpy_deg = r.as_euler('xyz', degrees=True)

        print("Rotation (roll, pitch, yaw) in radians: {:.4f}, {:.4f}, {:.4f}".format(*rpy_rad))
        print("Rotation (roll, pitch, yaw) in degrees: {:.2f}°, {:.2f}°, {:.2f}°".format(*rpy_deg))



        # Build T_map_dummy
        T_map_dummy = np.eye(4)
        T_map_dummy[:3, :3] = r.as_matrix()
        T_map_dummy[:3, 3] = [t.x, t.y, t.z]
        print("Transformation matrix T_map_dummy:\n", T_map_dummy)

        # Convert image
        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        self.depth_image = depth_inpainting(self.depth_image)

        # cv2.imwrite(f'/home/ia/tmp/color.png', self.rgb_image.astype(np.uint8))
        # cv2.imwrite(f'/home/ia/tmp/depth.png', (self.depth_image.astype(np.float32)*1000).astype(np.uint16))  # Save depth in mm
        # print(f'Saved merged images as /tmp/merged_rgb_{self.id}.png and /tmp/merged_depth_{self.id}.png')

        # Get robot pose in dummy_camera frame
        results = compute_handle_bar_by_rgbd_media_pipe(
            self.media_pipe_model,
            rgb_image=self.rgb_image,
            depth_image=self.depth_image,
            vis=True,
            frame=0
        )

        robot_position = results['robot_position']
        robot_orientation = results['robot_orientation']  # quaternion
        

        # Build T_dummy_robot
        r_robot = R.from_quat(robot_orientation)
        rpy = R.from_quat(robot_orientation).as_euler('xyz', degrees=True)
        print(f"robot orientation in camera dummy Roll: {rpy[0]:.2f}°, Pitch: {rpy[1]:.2f}°, Yaw: {rpy[2]:.2f}°")
        T_dummy_robot = np.eye(4)
        T_dummy_robot[:3, :3] = r_robot.as_matrix()
        T_dummy_robot[:3, 3] = robot_position

        # Camera frame (OpenCV) → robot base frame (ROS)
        T_cam_to_base = np.eye(4)
        T_cam_to_base[:3, :3] = R.from_euler('zyx', [-90, 0, -90], degrees=True).as_matrix()

        # Final transform: map ← dummy_camera ← robot ← camera
        T_map_robot = T_map_dummy @ T_dummy_robot

        # Extract position
        robot_position_map = T_map_robot[:3, 3]

        # Extract rotation matrix and orthogonalize it
        R_map_robot = T_map_robot[:3, :3]
        U, _, Vt = np.linalg.svd(R_map_robot)
        R_corrected = U @ Vt
        if np.linalg.det(R_corrected) < 0:
            R_corrected *= -1
        R_corrected = T_map_robot[:3, :3]
        robot_orientation_map = R.from_matrix(R_corrected).as_quat()
        # Print Roll, Pitch, Yaw (in degrees)
        rpy = R.from_quat(robot_orientation_map).as_euler('xyz', degrees=True)
        print(f"Roll: {rpy[0]:.2f}°, Pitch: {rpy[1]:.2f}°, Yaw: {rpy[2]:.2f}°")

        # Create and publish PoseStamped
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.position.x = robot_position_map[0]
        goal_msg.pose.position.y = robot_position_map[1]
        goal_msg.pose.position.z = 0.0

        goal_msg.pose.orientation.x = robot_orientation_map[0]
        goal_msg.pose.orientation.y = robot_orientation_map[1]
        goal_msg.pose.orientation.z = robot_orientation_map[2]
        goal_msg.pose.orientation.w = robot_orientation_map[3]

        self.publisher_.publish(goal_msg)
        self.get_logger().info('Published transformed goal pose to /goal_pose')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RGBDOnceListener()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
