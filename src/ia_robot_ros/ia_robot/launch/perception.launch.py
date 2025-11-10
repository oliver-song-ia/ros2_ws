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
    ros2 launch ia_robot perception.launch.py python_interpreter:=/usr/bin/python3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def launch_setup(context, *args, **kwargs):
    """Setup function to evaluate launch configuration and return launch actions."""
    # Get workspace root by navigating up from this file
    # This file is in: src/ia_robot/launch/perception.launch.py
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(launch_file_dir)))
    workspace_root = os.path.dirname(os.path.dirname(workspace_root)) # ros2_ws root

    # Paths to the Python scripts
    stcn_tracking_script = os.path.join(
        workspace_root,
        'src',
        'ia_robot_ros',
        'ia_robot_perception',
        'human_tracking',
        'realtime_stcn_tracking.py'
    )

    pose_detector_script = os.path.join(
        workspace_root,
        'src',
        'ia_robot_ros',
        'ia_robot_perception',
        'spike',
        'pose_detector.py'
    )

    # Get package share directory for ia_robot
    ia_robot_share = FindPackageShare('ia_robot')

    # Get Python interpreter from launch configuration
    python_interp = LaunchConfiguration('python_interpreter').perform(context)

    # Expand user path (handles ~)
    python_path = os.path.expanduser(python_interp)

    # Check if the Python path exists, otherwise fall back to 'python'
    if os.path.exists(python_path):
        python_cmd = python_path
    else:
        python_cmd = 'python'

    return [
        # 1. STCN Human Tracking - Launch as Python process
        ExecuteProcess(
            cmd=[python_cmd, stcn_tracking_script],
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
            cmd=[python_cmd, pose_detector_script],
            name='pose_detector',
            output='screen',
            shell=False
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Launch argument for Python interpreter
        DeclareLaunchArgument(
            'python_interpreter',
            default_value='~/miniforge3/envs/genesis/bin/python',
            description='Path to Python interpreter. Falls back to "python" if path does not exist.'
        ),

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

        # Use OpaqueFunction to handle conditional logic for python interpreter
        OpaqueFunction(function=launch_setup)
    ])
