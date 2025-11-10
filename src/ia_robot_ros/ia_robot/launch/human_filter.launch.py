#!/usr/bin/env python3
"""
Launch file for human point cloud filtering system.

This launches the human point cloud filter node that:
- Subscribes to two organized point clouds (cam0, cam1)
- Subscribes to one concatenated dual mask from STCN tracker
- Publishes filtered human and environment point clouds

Prerequisites:
- Simulator publishes organized point clouds to /points_dep0 and /points_dep1
- STCN tracker (realtime_stcn_tracking.py) publishes concatenated mask to /human_masks_dual
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'pointcloud_cam0_topic',
            default_value='/points_dep0',
            description='Topic for cam0 organized point cloud input'
        ),
        DeclareLaunchArgument(
            'pointcloud_cam1_topic',
            default_value='/points_dep1',
            description='Topic for cam1 organized point cloud input'
        ),
        DeclareLaunchArgument(
            'mask_dual_topic',
            default_value='/human_masks_dual',
            description='Topic for concatenated dual mask (cam1|cam0)'
        ),
        DeclareLaunchArgument(
            'filtered_human_topic',
            default_value='/pointcloud_human_only',
            description='Topic for merged filtered human points'
        ),
        DeclareLaunchArgument(
            'filtered_environment_topic',
            default_value='/pointcloud_environment_only',
            description='Topic for merged filtered environment points'
        ),
        DeclareLaunchArgument(
            'synchronization_slop',
            default_value='0.1',
            description='Time synchronization tolerance in seconds'
        ),
        
        # Human Point Cloud Filter Node
        Node(
            package='ia_robot',  # You may need to adjust this
            executable='human_pointcloud_filter_node.py',
            name='human_pointcloud_filter',
            output='screen',
            parameters=[{
                'pointcloud_cam0_topic': LaunchConfiguration('pointcloud_cam0_topic'),
                'pointcloud_cam1_topic': LaunchConfiguration('pointcloud_cam1_topic'),
                'mask_dual_topic': LaunchConfiguration('mask_dual_topic'),
                'filtered_human_topic': LaunchConfiguration('filtered_human_topic'),
                'filtered_environment_topic': LaunchConfiguration('filtered_environment_topic'),
                'slop': LaunchConfiguration('synchronization_slop'),
                'queue_size': 10,
                'mask_threshold': 127,
                'visualize_mask': False,
            }]
        ),
    ])
