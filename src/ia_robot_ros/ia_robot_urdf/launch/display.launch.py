#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('ia_robot_urdf')

    # File paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'ia_robot_ik.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'urdf.rviz')

    return LaunchDescription([
        # Declare arguments (like <arg>)
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Model argument (not used yet)'
        ),

        # Set robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])
