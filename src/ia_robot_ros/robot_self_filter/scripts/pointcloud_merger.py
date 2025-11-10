#!/usr/bin/env python3
"""
Pointcloud Merger Node
Subscribes to two pointcloud topics and publishes a combined pointcloud
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
import struct
import array

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')
        
        # Declare parameters
        self.declare_parameter('input_topic_0', '/points_dep0_filtered')
        self.declare_parameter('input_topic_1', '/points_dep1_filtered')
        self.declare_parameter('output_topic', '/lidar_filter_combined')
        # self.declare_parameter('use_sim_time', False)
        self.declare_parameter('use_approximate_sync', True)
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('slop', 0.1)  # Time tolerance for approximate sync in seconds
        
        # Get parameters
        input_topic_0 = self.get_parameter('input_topic_0').value
        input_topic_1 = self.get_parameter('input_topic_1').value
        output_topic = self.get_parameter('output_topic').value
        use_approximate_sync = self.get_parameter('use_approximate_sync').value
        queue_size = self.get_parameter('queue_size').value
        slop = self.get_parameter('slop').value
        
        self.get_logger().info(f'Starting PointCloud Merger')
        self.get_logger().info(f'  Input topic 0: {input_topic_0}')
        self.get_logger().info(f'  Input topic 1: {input_topic_1}')
        self.get_logger().info(f'  Output topic: {output_topic}')
        self.get_logger().info(f'  Use approximate sync: {use_approximate_sync}')
        self.get_logger().info(f'  Queue size: {queue_size}')
        self.get_logger().info(f'  Slop: {slop}s')
        
        # Create publisher
        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        
        # Create subscribers using message_filters for synchronization
        self.sub_0 = Subscriber(self, PointCloud2, input_topic_0)
        self.sub_1 = Subscriber(self, PointCloud2, input_topic_1)
        
        # Create synchronizer
        if use_approximate_sync:
            self.sync = ApproximateTimeSynchronizer(
                [self.sub_0, self.sub_1],
                queue_size=queue_size,
                slop=slop
            )
        else:
            from message_filters import TimeSynchronizer
            self.sync = TimeSynchronizer(
                [self.sub_0, self.sub_1],
                queue_size=queue_size
            )
        
        self.sync.registerCallback(self.callback)
        self.get_logger().info('PointCloud Merger initialized successfully')
        
    def callback(self, cloud_0, cloud_1):
        """Callback to merge two synchronized pointclouds"""
        try:
            # Create merged pointcloud
            merged_cloud = self.merge_pointclouds(cloud_0, cloud_1)
            
            # Publish merged cloud
            self.publisher.publish(merged_cloud)
            
            self.get_logger().debug(
                f'Merged clouds: {len(cloud_0.data)} + {len(cloud_1.data)} = {len(merged_cloud.data)} bytes'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error merging pointclouds: {str(e)}')
    
    def merge_pointclouds(self, cloud_0, cloud_1):
        """Merge two PointCloud2 messages into one"""
        # Use the first cloud as template
        merged = PointCloud2()
        merged.header = cloud_0.header  # Use timestamp from first cloud
        merged.height = 1  # Unordered point cloud
        merged.width = cloud_0.width + cloud_1.width
        merged.fields = cloud_0.fields
        merged.is_bigendian = cloud_0.is_bigendian
        merged.point_step = cloud_0.point_step
        merged.row_step = merged.point_step * merged.width
        merged.is_dense = cloud_0.is_dense and cloud_1.is_dense
        
        # Combine the data
        merged.data = cloud_0.data + cloud_1.data
        
        return merged

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
