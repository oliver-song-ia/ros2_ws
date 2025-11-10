#!/usr/bin/env python3
"""
Launch file for the joint states remapper node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the script
    package_dir = get_package_share_directory('ia_robot')
    script_path = os.path.join(package_dir, 'src', 'joint_states_remapper.py')

    # Create the remapper node
    remapper_node = Node(
        package='ia_robot',
        executable='joint_states_remapper.py',
        name='joint_states_remapper',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        remapper_node
    ])