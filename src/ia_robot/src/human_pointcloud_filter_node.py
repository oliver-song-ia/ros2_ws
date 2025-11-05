#!/usr/bin/env python3
"""
Human Point Cloud Filter Node

Subscribes to:
  - Two organized point clouds (cam0 and cam1, with H×W structure preserved)
  - One concatenated mask image containing both camera masks vertically stacked

Publishes:
  - Merged filtered point cloud containing only human points (standard PointCloud2)
  - Merged filtered point cloud containing only non-human points (standard PointCloud2)

This node uses message_filters to synchronize point clouds and the dual mask,
then splits the mask and applies to corresponding point clouds.
"""

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
import message_filters
from typing import Optional, Tuple
import cv2
import struct
import time
import tf2_ros
from tf2_ros import TransformException
import tf2_py as tf2
from rclpy.duration import Duration


class HumanPointCloudFilter(Node):
    def __init__(self):
        super().__init__('human_pointcloud_filter')
        
        # Declare parameters
        self.declare_parameter('pointcloud_cam0_topic', '/points_dep0')
        self.declare_parameter('pointcloud_cam1_topic', '/points_dep1')
        self.declare_parameter('mask_dual_topic', '/human_masks_dual')
        self.declare_parameter('filtered_human_topic', '/pointcloud_human_only')
        self.declare_parameter('filtered_environment_topic', '/pointcloud_environment_only')
        self.declare_parameter('target_frame', 'odom')  # Target frame for transformations
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.5)  # Time synchronization tolerance in seconds
        self.declare_parameter('mask_threshold', 127)  # Binary threshold (0-255)
        self.declare_parameter('visualize_mask', False)  # Debug visualization
        self.declare_parameter('max_human_distance', 1.0)  # Maximum distance (meters) for human points
        
        # Get parameters
        pc0_topic = self.get_parameter('pointcloud_cam0_topic').value
        pc1_topic = self.get_parameter('pointcloud_cam1_topic').value
        mask_dual_topic = self.get_parameter('mask_dual_topic').value
        human_topic = self.get_parameter('filtered_human_topic').value
        env_topic = self.get_parameter('filtered_environment_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        queue_size = self.get_parameter('queue_size').value
        self.slop = self.get_parameter('slop').value
        self.mask_threshold = self.get_parameter('mask_threshold').value
        self.visualize = self.get_parameter('visualize_mask').value
        self.max_human_distance = self.get_parameter('max_human_distance').value
        
        # Initialize
        self.bridge = CvBridge()
        self.sync_count = 0
        self.drop_count = 0
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size
        )
        
        # Subscribers with message_filters for synchronization (3 topics now!)
        self.pc0_sub = message_filters.Subscriber(
            self, PointCloud2, pc0_topic, qos_profile=qos_profile
        )
        self.pc1_sub = message_filters.Subscriber(
            self, PointCloud2, pc1_topic, qos_profile=qos_profile
        )
        self.mask_dual_sub = message_filters.Subscriber(
            self, Image, mask_dual_topic, qos_profile=qos_profile
        )
        
        # Approximate time synchronizer for 3 topics (reduced from 4!)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pc0_sub, self.pc1_sub, self.mask_dual_sub],
            queue_size=queue_size,
            slop=self.slop
        )
        self.ts.registerCallback(self.synchronized_callback)
        
        # Publishers
        self.human_pub = self.create_publisher(
            PointCloud2, human_topic, qos_profile
        )
        self.env_pub = self.create_publisher(
            PointCloud2, env_topic, qos_profile
        )
        
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.print_statistics)
        
        self.get_logger().info(f"Human Point Cloud Filter initialized")
        self.get_logger().info(f"  Subscribing to PC cam0: {pc0_topic}")
        self.get_logger().info(f"  Subscribing to PC cam1: {pc1_topic}")
        self.get_logger().info(f"  Subscribing to dual mask: {mask_dual_topic}")
        self.get_logger().info(f"  Publishing human points to: {human_topic}")
        self.get_logger().info(f"  Publishing environment points to: {env_topic}")
        self.get_logger().info(f"  Target frame: {self.target_frame}")
        self.get_logger().info(f"  Synchronization slop: {self.slop}s")
        self.get_logger().info(f"  Max human distance: {self.max_human_distance}m")
    
    def synchronized_callback(self, pc0_msg: PointCloud2, pc1_msg: PointCloud2,
                              mask_dual_msg: Image):
        """
        Called when all 3 topics are synchronized.
        """
        try:
            t_start = time.perf_counter()
            self.sync_count += 1
            
            # Convert dual mask to numpy array
            t0 = time.perf_counter()
            mask_dual = self.bridge.imgmsg_to_cv2(mask_dual_msg, desired_encoding='mono8')
            t1 = time.perf_counter()
            
            # Parse organized point clouds
            pc0_array = self.parse_organized_pointcloud2(pc0_msg)  # (H0, W0, 3)
            pc1_array = self.parse_organized_pointcloud2(pc1_msg)  # (H1, W1, 3)
            t2 = time.perf_counter()
            
            if pc0_array is None or pc1_array is None:
                self.get_logger().error("Failed to parse point clouds")
                self.drop_count += 1
                return
            
            # Split dual mask into cam0 and cam1 masks
            mask0, mask1 = self.split_dual_mask(mask_dual, pc0_array.shape[:2], pc1_array.shape[:2])
            t3 = time.perf_counter()
            
            if mask0 is None or mask1 is None:
                self.get_logger().error("Failed to split dual mask")
                self.drop_count += 1
                return
            
            # Convert to binary masks
            mask0_binary = (mask0 > self.mask_threshold).astype(bool)
            mask1_binary = (mask1 > self.mask_threshold).astype(bool)
            t4 = time.perf_counter()
            
            # Visualize if enabled
            if self.visualize:
                self.visualize_masks(mask0_binary, mask1_binary)
            
            # Filter point clouds with masks
            human_points0, env_points0 = self.filter_pointcloud_with_mask(pc0_array, mask0_binary)
            human_points1, env_points1 = self.filter_pointcloud_with_mask(pc1_array, mask1_binary)
            t5 = time.perf_counter()
            
            # Transform point clouds to target frame
            source_frame0 = pc0_msg.header.frame_id
            source_frame1 = pc1_msg.header.frame_id
            timestamp = pc0_msg.header.stamp
            
            # Transform cam0 points to target frame
            if source_frame0 != self.target_frame:
                human_points0 = self.transform_points(human_points0, source_frame0, self.target_frame, timestamp)
                env_points0 = self.transform_points(env_points0, source_frame0, self.target_frame, timestamp)
            
            # Transform cam1 points to target frame
            if source_frame1 != self.target_frame:
                human_points1 = self.transform_points(human_points1, source_frame1, self.target_frame, timestamp)
                env_points1 = self.transform_points(env_points1, source_frame1, self.target_frame, timestamp)
            
            t6 = time.perf_counter()
            
            # Merge points from both cameras (now in the same frame)
            human_points = np.vstack([human_points0, human_points1]) if human_points0.shape[0] > 0 and human_points1.shape[0] > 0 else \
                          human_points0 if human_points0.shape[0] > 0 else human_points1
            
            env_points = np.vstack([env_points0, env_points1]) if env_points0.shape[0] > 0 and env_points1.shape[0] > 0 else \
                        env_points0 if env_points0.shape[0] > 0 else env_points1
            t7 = time.perf_counter()
            
            # Create and publish messages (use target frame)
            if human_points.shape[0] > 0:
                human_pc_msg = self.create_pointcloud2_msg(
                    human_points, timestamp, self.target_frame
                )
                self.human_pub.publish(human_pc_msg)
            
            if env_points.shape[0] > 0:
                env_pc_msg = self.create_pointcloud2_msg(
                    env_points, timestamp, self.target_frame
                )
                self.env_pub.publish(env_pc_msg)
            t8 = time.perf_counter()
            
            t8 = time.perf_counter()
            
            # Calculate and log timing
            t_total = t8 - t_start
            timing_info = (
                f"Timing (ms): "
                f"mask_convert={1000*(t1-t0):.2f}, "
                f"parse_pc={1000*(t2-t1):.2f}, "
                f"split_mask={1000*(t3-t2):.2f}, "
                f"binarize={1000*(t4-t3):.2f}, "
                f"filter_pc={1000*(t5-t4):.2f}, "
                f"transform={1000*(t6-t5):.2f}, "
                f"merge={1000*(t7-t6):.2f}, "
                f"publish={1000*(t8-t7):.2f}, "
                f"total={1000*t_total:.2f}"
            )
            
            # Log periodically
            if self.sync_count % 50 == 0:
                self.get_logger().info(
                    f"Filtered: {human_points.shape[0]} human pts, "
                    f"{env_points.shape[0]} env pts "
                    f"(cam0: {human_points0.shape[0]}+{env_points0.shape[0]}, "
                    f"cam1: {human_points1.shape[0]}+{env_points1.shape[0]})"
                )
                self.get_logger().info(timing_info)
                
        except Exception as e:
            self.drop_count += 1
            self.get_logger().error(f"Error in synchronized callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def split_dual_mask(self, mask_dual: np.ndarray, 
                        pc0_shape: Tuple[int, int], 
                        pc1_shape: Tuple[int, int]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Split concatenated dual mask into two separate masks and scale to match pointcloud dimensions.
        The dual mask is vertically stacked: [cam1_mask | cam0_mask]
        The mask resolution may differ from pointcloud (e.g., mask 400x640, PC 100x160) but maintains aspect ratio.
        
        Args:
            mask_dual: (H_total, W) concatenated mask (may be higher resolution than pointcloud)
            pc0_shape: (H0, W0) shape of cam0 point cloud
            pc1_shape: (H1, W1) shape of cam1 point cloud
        
        Returns:
            mask0: (H0, W0) mask for cam0, scaled to match pointcloud dimensions
            mask1: (H1, W1) mask for cam1, scaled to match pointcloud dimensions
        """
        try:
            h0, w0 = pc0_shape
            h1, w1 = pc1_shape
            h_total, w_total = mask_dual.shape
            
            # The dual mask should be cam1 (top) + cam0 (bottom)
            # Both cameras' masks are concatenated vertically in the dual mask
            # The masks may be at higher resolution than the pointclouds but maintain aspect ratio
            
            # Calculate the expected aspect ratio for each camera from pointcloud dimensions
            # Assuming the mask maintains the same aspect ratio as the pointcloud
            aspect_ratio_0 = w0 / h0
            aspect_ratio_1 = w1 / h1
            
            # Estimate split point (proportional to pointcloud heights)
            # The split should respect the original camera proportions
            total_h_pc = h0 + h1
            split_ratio = h1 / total_h_pc
            split_h = int(h_total * split_ratio)
            
            # Split the dual mask into two parts
            mask1_raw = mask_dual[:split_h, :]  # Top part for cam1
            mask0_raw = mask_dual[split_h:, :]  # Bottom part for cam0
            
            # Resize masks to match pointcloud dimensions
            # Use INTER_NEAREST to preserve mask values (no interpolation artifacts)
            mask0 = cv2.resize(mask0_raw, (w0, h0), interpolation=cv2.INTER_NEAREST)
            mask1 = cv2.resize(mask1_raw, (w1, h1), interpolation=cv2.INTER_NEAREST)
            
            # Log dimension info periodically for debugging
            if self.sync_count % 30 == 0:
                self.get_logger().info(
                    f"Mask scaling: dual_mask {h_total}x{w_total} -> "
                    f"cam0 {mask0_raw.shape[0]}x{mask0_raw.shape[1]} -> {h0}x{w0}, "
                    f"cam1 {mask1_raw.shape[0]}x{mask1_raw.shape[1]} -> {h1}x{w1}"
                )
            
            return mask0, mask1
            
        except Exception as e:
            self.get_logger().error(f"Failed to split dual mask: {e}")
            return None, None
    
    def transform_points(self, points: np.ndarray, source_frame: str, target_frame: str, timestamp) -> np.ndarray:
        """
        Transform points from source frame to target frame using TF2.
        
        Args:
            points: (N, 3) array of [x, y, z] points in source frame
            source_frame: Source frame ID
            target_frame: Target frame ID
            timestamp: ROS timestamp for the transformation
        
        Returns:
            (N, 3) array of transformed points in target frame
        """
        if points.shape[0] == 0:
            return points
        
        try:
            # Look up the transform - use latest available if exact timestamp not available
            if self.tf_buffer.can_transform(
                target_frame,
                source_frame,
                timestamp,
                timeout=Duration(seconds=self.slop)
            ):
                # Use the exact timestamp if available
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    timestamp
                )
            else:
                # Fall back to latest available transform to avoid huge errors
                self.get_logger().debug(
                    f"Exact transform not available, using latest transform from {source_frame} to {target_frame}"
                )
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()  # Time(0) means latest available
                )
            
            # Extract translation and rotation
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # Convert quaternion to rotation matrix
            # Quaternion: [x, y, z, w]
            qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
            
            # Rotation matrix from quaternion
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
                [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
                [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
            ])
            
            # Translation vector
            t = np.array([trans.x, trans.y, trans.z])
            
            # Transform points: p_target = R * p_source + t
            transformed_points = (R @ points.T).T + t
            
            return transformed_points.astype(np.float32)
            
        except TransformException as e:
            self.get_logger().warn(
                f"Could not transform from {source_frame} to {target_frame}: {e}"
            )
            # Return original points if transformation fails
            return points
    
    def parse_organized_pointcloud2(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """
        Parse organized PointCloud2 (height > 1) to numpy array.
        
        Returns:
            (H, W, 3) numpy array of [x, y, z] points, or None if parsing fails
        """
        try:
            height = msg.height
            width = msg.width
            
            if height <= 1:
                self.get_logger().error(f"Point cloud is not organized (height={height})")
                return None
            
            # Find xyz field offsets
            x_offset = y_offset = z_offset = None
            for field in msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error("Missing x, y, or z fields in point cloud")
                return None
            
            # Parse point cloud data
            points = np.zeros((height, width, 3), dtype=np.float32)
            point_step = msg.point_step
            row_step = msg.row_step
            
            for i in range(height):
                for j in range(width):
                    idx = i * row_step + j * point_step
                    x = struct.unpack_from('f', msg.data, idx + x_offset)[0]
                    y = struct.unpack_from('f', msg.data, idx + y_offset)[0]
                    z = struct.unpack_from('f', msg.data, idx + z_offset)[0]
                    points[i, j] = [x, y, z]
            
            return points
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse organized point cloud: {e}")
            return None
    
    def filter_pointcloud_with_mask(self, pc: np.ndarray, mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Filter point cloud using binary mask and distance threshold.
        
        Args:
            pc: (H, W, 3) point cloud array
            mask: (H, W) binary mask (True = human, False = environment)
        
        Returns:
            human_points: (N, 3) array of human points (within max_human_distance)
            env_points: (M, 3) array of environment points (including distant human points)
        """
        # Filter out NaN/invalid points
        valid_mask = ~np.isnan(pc).any(axis=2)
        
        # Calculate distance from origin (camera) for each point
        distances = np.linalg.norm(pc, axis=2)  # (H, W)
        dist_mask = valid_mask & mask
        mean_distance = np.median(distances[dist_mask]) if np.any(valid_mask) else 0.0
        self.get_logger().info(f"Mean distance of masked points: {mean_distance:.2f}m")
        
        # Distance filter: points within max_human_distance
        distance_mask = abs(distances - mean_distance) <= self.max_human_distance if self.max_human_distance > 0 else np.ones_like(distances, dtype=bool)
        
        # Create human and environment masks
        # Human points must be: in mask AND valid AND within distance threshold
        human_mask = mask & valid_mask & distance_mask
        
        # Environment points are: (not in mask OR beyond distance threshold) AND valid
        env_mask = (~mask | ~distance_mask) & valid_mask
        
        # Extract points
        human_points = pc[human_mask]
        env_points = pc[env_mask]
        
        return human_points, env_points
    
    def create_pointcloud2_msg(self, points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
        """
        Convert numpy array of points to PointCloud2 message.
        
        Args:
            points: (N, 3) array of [x, y, z] points
            stamp: ROS timestamp
            frame_id: Frame ID for the point cloud
        """
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        # Convert to structured array
        points_arr = points.astype(np.float32)
        cloud_arr = np.core.records.fromarrays(
            points_arr.T, names='x,y,z', formats='f4,f4,f4'
        )
        cloud_arr = np.atleast_2d(cloud_arr)
        
        msg.height = 1  # Unorganized point cloud
        msg.width = cloud_arr.shape[1]
        msg.is_bigendian = False
        msg.point_step = cloud_arr.dtype.itemsize
        msg.row_step = msg.point_step * cloud_arr.shape[1]
        msg.is_dense = True  # We've filtered out NaN
        msg.data = cloud_arr.tobytes()
        
        # Define fields
        type_mappings = [
            (PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')),
            (PointField.INT16, np.dtype('int16')), (PointField.UINT16, np.dtype('uint16')),
            (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
            (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))
        ]
        nptype_to_pftype = {nptype: pftype for pftype, nptype in type_mappings}
        
        msg.fields = []
        for name in cloud_arr.dtype.names:
            np_type, offset = cloud_arr.dtype.fields[name]
            pf = PointField()
            pf.name = name
            if np_type.subdtype:
                item_type, shape = np_type.subdtype
                pf.count = int(np.prod(shape))
                np_type = item_type
            else:
                pf.count = 1
            pf.datatype = nptype_to_pftype[np_type]
            pf.offset = offset
            msg.fields.append(pf)
        
        return msg
    
    def visualize_masks(self, mask0: np.ndarray, mask1: np.ndarray):
        """Debug visualization of both masks (optional)"""
        try:
            # Convert masks to color images for visualization
            mask0_vis = (mask0.astype(np.uint8) * 255)
            mask1_vis = (mask1.astype(np.uint8) * 255)
            
            mask0_color = cv2.applyColorMap(mask0_vis, cv2.COLORMAP_JET)
            mask1_color = cv2.applyColorMap(mask1_vis, cv2.COLORMAP_JET)
            
            # Stack side by side
            combined = np.hstack([mask0_color, mask1_color])
            cv2.imshow('Human Masks (Cam0 | Cam1)', combined)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Visualization error: {e}")
    
    def print_statistics(self):
        """Print periodic statistics"""
        if self.sync_count > 0:
            success_rate = (self.sync_count / (self.sync_count + self.drop_count)) * 100
            self.get_logger().info(
                f"Statistics: {self.sync_count} synced, {self.drop_count} dropped "
                f"({success_rate:.1f}% success rate)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = HumanPointCloudFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
