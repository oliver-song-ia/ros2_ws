#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    ia_robot_urdf_pkg_share = get_package_share_directory('ia_robot_urdf')
    ia_robot_pkg_share = get_package_share_directory('ia_robot')
    robot_self_filter_pkg_share = get_package_share_directory('robot_self_filter')

    # File paths
    urdf_file = os.path.join(ia_robot_urdf_pkg_share, 'urdf', 'ia_robot.urdf')
    rviz_config_file = os.path.join(ia_robot_pkg_share, 'config', 'nav2.rviz')
    self_filter_launch_file = os.path.join(robot_self_filter_pkg_share, 'launch', 'self_filter.launch.py')
    lidar_self_filter_launch_file = os.path.join(robot_self_filter_pkg_share, 'launch', 'lidar_self_filter.launch.py')
    # dual_lidar_self_filter_launch_file = os.path.join(robot_self_filter_pkg_share, 'launch', 'dual_lidar_self_filter.launch.py')
    ctrl_launch_file = os.path.join(ia_robot_pkg_share, 'launch', 'ctrl_ia_robot.launch.py')

    return LaunchDescription([
        # Declare arguments (like <arg>)
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Model argument (not used yet)'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Include ia_robot controller launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ctrl_launch_file),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),

        # Set robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open(urdf_file).read(),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # Include robot self filter launch files with different configurations
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_self_filter_launch_file),
            launch_arguments={
                'target_frame': 'lidar_link',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'in_pointcloud_topic': '/lidar',
                'out_pointcloud_topic': '/lidar_filter',
                'default_sphere_padding': '0.05'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_self_filter_launch_file),
            launch_arguments={
                'target_frame': 'odom',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'in_pointcloud_topic': '/pointcloud_human_only',
                'out_pointcloud_topic': '/pointcloud_human_only_filter',
                'default_sphere_padding': '0.02'
            }.items()
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(dual_lidar_self_filter_launch_file),
        #     launch_arguments={
        #         'target_frame': 'odom',
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #         'in_pointcloud_topic_0': '/points_dep0',
        #         'in_pointcloud_topic_1': '/points_dep1',
        #         'out_pointcloud_topic': '/lidar_filter_combined'
        #     }.items()
        # ),

        # Pointcloud to LaserScan converter
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', '/lidar_filter'),
                ('scan', '/scan')
            ],
            parameters=[{
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.0174533,
                'range_min': 0.3,
                'range_max': 25.0,
                'min_height': 0.1,
                'max_height': 2.0,
                'scan_time': 0.1,
                'target_frame': 'base_footprint',
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
