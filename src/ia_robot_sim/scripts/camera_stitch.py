#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
import math


class RGBDProjectMerger(Node):
    def __init__(self):
        super().__init__('rgbd_project_merger')
        self.bridge = CvBridge()

        # Cached data
        self.rgb1 = None
        self.rgb2 = None
        self.depth1 = None
        self.depth2 = None
        self.cam_info = None

        # Subscriptions
        self.sub_rgb1 = self.create_subscription(Image, '/rgb1', self.rgb1_callback, 10)
        self.sub_rgb2 = self.create_subscription(Image, '/rgb2', self.rgb2_callback, 10)
        self.sub_depth1 = self.create_subscription(Image, '/depth1', self.depth1_callback, 10)
        self.sub_depth2 = self.create_subscription(Image, '/depth2', self.depth2_callback, 10)
        self.sub_cam_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.caminfo_callback, 10)

        # Publishers
        self.pub_rgb = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.pub_depth_color = self.create_publisher(Image, '/camera/depth/colorized', 10)

        self.id = 0

    def caminfo_callback(self, msg):
        self.cam_info = msg

    def rgb1_callback(self, msg):
        self.rgb1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_merge(msg.header)

    def rgb2_callback(self, msg):
        self.rgb2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.try_merge(msg.header)

    def depth1_callback(self, msg):
        self.depth1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.try_merge(msg.header)

    def depth2_callback(self, msg):
        self.depth2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.try_merge(msg.header)

    def get_rotation_matrix(self, pitch_deg):
        """Returns 3x3 rotation matrix for pitch (rotation about X axis)"""
        pitch_rad = math.radians(pitch_deg)
        return np.array([
            [1, 0, 0],
            [0, math.cos(pitch_rad), -math.sin(pitch_rad)],
            [0, math.sin(pitch_rad),  math.cos(pitch_rad)]
        ])

    def warp_image(self, image, K, R, shape, interpolation=cv2.INTER_LINEAR):
        """Warp an image using homography derived from rotation matrix R"""
        H = K @ R @ np.linalg.inv(K)
        return cv2.warpPerspective(image, H, shape, flags=interpolation)

    def warp_depth(self, depth, K, R, shape):
        """Warp a depth image, correcting depth values for perspective transformation"""
        h, w = depth.shape
        # Create pixel grid
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        pixels = np.stack([u, v, np.ones_like(u)], axis=-1).reshape(-1, 3)

        # Unproject to 3D points
        depth_flat = depth.flatten()
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        X = (pixels[:, 0] - cx) * depth_flat / fx
        Y = (pixels[:, 1] - cy) * depth_flat / fy
        Z = depth_flat
        points_3d = np.stack([X, Y, Z], axis=-1)

        # Apply rotation
        points_3d_rotated = points_3d @ R.T

        # Reproject to 2D
        Z_rotated = points_3d_rotated[:, 2]
        valid = Z_rotated > 0
        pixels_warped = np.zeros_like(pixels)
        pixels_warped[valid, 0] = fx * points_3d_rotated[valid, 0] / Z_rotated[valid] + cx
        pixels_warped[valid, 1] = fy * points_3d_rotated[valid, 1] / Z_rotated[valid] + cy
        pixels_warped[valid, 2] = 1

        # Create output depth image
        depth_warped = np.zeros((h, w), dtype=np.float32)
        u_warped = np.clip(pixels_warped[:, 0], 0, w-1).astype(int)
        v_warped = np.clip(pixels_warped[:, 1], 0, h-1).astype(int)
        depth_warped[v_warped[valid], u_warped[valid]] = Z_rotated[valid]

        return depth_warped

    def try_merge(self, header):
        if not all([self.rgb1 is not None, self.rgb2 is not None,
                    self.depth1 is not None, self.depth2 is not None,
                    self.cam_info is not None]):
            return

        h, w = self.cam_info.height, self.cam_info.width
        K = np.array(self.cam_info.k).reshape(3, 3)

        # Rotation matrices
        R1 = self.get_rotation_matrix(-30.0)
        R2 = self.get_rotation_matrix(30.0)

        # === Warp RGB ===
        rgb1_warped = self.warp_image(self.rgb1, K, R1, (w, h), interpolation=cv2.INTER_LINEAR)
        rgb2_warped = self.warp_image(self.rgb2, K, R2, (w, h), interpolation=cv2.INTER_LINEAR)

        mask1_rgb = (rgb1_warped > 0).any(axis=2)
        merged_rgb = np.where(mask1_rgb[..., None], rgb1_warped, rgb2_warped)

        # === Warp Depth ===
        depth1_warped = self.warp_depth(self.depth1, K, R1, (w, h))
        depth2_warped = self.warp_depth(self.depth2, K, R2, (w, h))

        mask1_depth = (depth1_warped > 0)
        mask2_depth = (depth2_warped > 0)

        # Combine with min-depth priority
        merged_depth = np.where(mask1_depth & mask2_depth,
                                np.minimum(depth1_warped, depth2_warped),
                                np.where(mask1_depth, depth1_warped, depth2_warped))

        # === Publish RGB ===
        rgb_msg = self.bridge.cv2_to_imgmsg(merged_rgb.astype(np.uint8), 'bgr8')
        rgb_msg.header = header
        self.pub_rgb.publish(rgb_msg)

        # === Publish Depth ===
        depth_msg = self.bridge.cv2_to_imgmsg(merged_depth.astype(np.float32), encoding='32FC1')
        depth_msg.header = header
        self.pub_depth.publish(depth_msg)

        # === Publish Colorized Depth ===
        depth_norm = np.clip((merged_depth - 0.5) / (5.0 - 0.5), 0.0, 1.0)  # Normalize between 0.5m and 5.0m
        depth_8u = (depth_norm * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)

        depth_color_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding='bgr8')
        depth_color_msg.header = header
        self.pub_depth_color.publish(depth_color_msg)

        # Save the merged images to disk
        self.id = self.id + 1
        


def main(args=None):
    rclpy.init(args=args)
    node = RGBDProjectMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()