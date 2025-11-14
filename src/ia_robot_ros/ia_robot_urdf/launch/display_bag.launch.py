#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('ia_robot_urdf')

    # File paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'ia_robot_ik.absolute.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'urdf.rviz')

    return LaunchDescription([
        # Set robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 注意：不启动 joint_state_publisher
        # bag播放会提供 /joint_states 或 /ik/joint_states

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])
