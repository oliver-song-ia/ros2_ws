#!/usr/bin/env python3
"""
Launch file for the perception pipeline including human tracking and pose detection.

This launches:
1. realtime_stcn_tracking.py - Human segmentation from dual camera images
2. human_filter.launch.py - Point cloud filtering based on human masks
3. pose_detector.py - Human pose detection from filtered point cloud

Prerequisites:
- Genesis simulator or Isaac Sim must be running and publishing:
  - /rgb_cam0 (sensor_msgs/Image)
  - /rgb_cam1 (sensor_msgs/Image)
  - /points_dep0 (sensor_msgs/PointCloud2)
  - /points_dep1 (sensor_msgs/PointCloud2)
- STCN models must be available in ia_perception_human_tracking package

Usage:
    ros2 launch ia_robot perception.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # Get workspace root by navigating up from this file
    # This file is in: src/ia_robot/launch/perception.launch.py
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(launch_file_dir)))
    workspace_root = os.path.dirname(os.path.dirname(workspace_root))
    
    # Paths to the Python scripts
    stcn_tracking_script = os.path.join(
        workspace_root,
        'src',
        'ia_perception_human_tracking',
        'realtime_stcn_tracking.py'
    )
    
    pose_detector_script = os.path.join(
        workspace_root,
        'src',
        'ia_robot_spike',
        'pose_detector.py'
    )
    
    # Get package share directory for ia_robot
    ia_robot_share = FindPackageShare('ia_robot')

    return LaunchDescription([
        # Launch arguments for topic remapping
        DeclareLaunchArgument(
            'rgb_cam0_topic',
            default_value='/rgb_cam0',
            description='RGB camera 0 topic'
        ),
        DeclareLaunchArgument(
            'rgb_cam1_topic',
            default_value='/rgb_cam1',
            description='RGB camera 1 topic'
        ),
        DeclareLaunchArgument(
            'points_dep0_topic',
            default_value='/points_dep0',
            description='Depth point cloud camera 0 topic'
        ),
        DeclareLaunchArgument(
            'points_dep1_topic',
            default_value='/points_dep1',
            description='Depth point cloud camera 1 topic'
        ),
        DeclareLaunchArgument(
            'human_mask_dual_topic',
            default_value='/human_masks_dual',
            description='Dual human mask output topic'
        ),
        DeclareLaunchArgument(
            'pointcloud_human_only_filtered_topic',
            default_value='/pointcloud_human_only_filter',
            description='Filtered human-only point cloud topic (after self-filter)'
        ),
        DeclareLaunchArgument(
            'pose_detection_topic',
            default_value='/pose_detection',
            description='Human pose detection markers topic'
        ),

        # 1. STCN Human Tracking - Launch as Python process
        ExecuteProcess(
            cmd=['python3', stcn_tracking_script],
            name='stcn_human_tracking',
            output='screen',
            shell=False
        ),

        # 2. Human Filter Launch (includes point cloud filtering)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    ia_robot_share,
                    'launch',
                    'human_filter.launch.py'
                ])
            ]),
            launch_arguments={
                'pointcloud_cam0_topic': LaunchConfiguration('points_dep0_topic'),
                'pointcloud_cam1_topic': LaunchConfiguration('points_dep1_topic'),
                'mask_dual_topic': LaunchConfiguration('human_mask_dual_topic'),
            }.items()
        ),

        # 3. Pose Detector - Launch as Python process
        ExecuteProcess(
            cmd=['python3', pose_detector_script],
            name='pose_detector',
            output='screen',
            shell=False
        ),
    ])
