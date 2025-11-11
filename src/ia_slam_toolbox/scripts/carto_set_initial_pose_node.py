#!/usr/bin/env python3
"""
ROS2版本的Cartographer初始姿态设置节点
基于Python实现，更简单易用
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
import tf_transformations
import threading

class CartoSetInitialPoseNode(Node):
    def __init__(self):
        super().__init__('carto_set_initial_pose_node')
        
        # 初始化参数
        self.trajectory_id = 1
        
        # 创建订阅者
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.init_pose_callback,
            QoSProfile(depth=100))
        
        # 创建服务客户端
        self.finish_trajectory_client = self.create_client(
            FinishTrajectory, 'finish_trajectory')
        self.start_trajectory_client = self.create_client(
            StartTrajectory, 'start_trajectory')
        
        # 等待服务可用
        self.finish_trajectory_client.wait_for_service()
        self.start_trajectory_client.wait_for_service()
        
        self.get_logger().info('Cartographer initial pose node initialized')

    def init_pose_callback(self, pose_data):
        """处理初始姿态消息"""
        pose_x = pose_data.pose.pose.position.x
        pose_y = pose_data.pose.pose.position.y
        theta_yaw = tf_transformations.euler_from_quaternion([
            pose_data.pose.pose.orientation.x,
            pose_data.pose.pose.orientation.y,
            pose_data.pose.pose.orientation.z,
            pose_data.pose.pose.orientation.w
        ])[2]  # 获取yaw角

        self.get_logger().info(f'Received initial pose: x={pose_x:.3f}, y={pose_y:.3f}, yaw={theta_yaw:.3f}')

        # 结束当前轨迹
        self.finish_current_trajectory()
        
        # 启动新轨迹
        self.start_new_trajectory(pose_data)

    def finish_current_trajectory(self):
        """结束当前轨迹"""
        request = FinishTrajectory.Request()
        request.trajectory_id = self.trajectory_id

        future = self.finish_trajectory_client.call_async(request)
        
        # 等待服务响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Successfully finished trajectory {self.trajectory_id}')
        else:
            self.get_logger().error(f'Failed to finish trajectory {self.trajectory_id}')

    def start_new_trajectory(self, pose_data):
        """启动新轨迹并设置初始姿态"""
        request = StartTrajectory.Request()
        
        # 设置配置参数
        request.configuration_directory = "/home/ia/fangfudong/ros2_ws/src/ia_cartographer/config"
        request.configuration_basename = "ia_location.lua"
        request.use_initial_pose = True
        request.initial_pose = pose_data.pose.pose
        request.relative_to_trajectory_id = 0

        future = self.start_trajectory_client.call_async(request)
        
        # 等待服务响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Successfully started new trajectory')
            self.trajectory_id += 1
        else:
            self.get_logger().error('Failed to start new trajectory')

def main(args=None):
    rclpy.init(args=args)
    
    node = CartoSetInitialPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()