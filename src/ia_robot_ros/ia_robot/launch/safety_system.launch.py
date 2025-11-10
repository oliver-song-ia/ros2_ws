#!/usr/bin/env python3
"""
Safety System Launch File
Launches ultrasonic emergency stop node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch parameters
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_safety_threshold_cmd = DeclareLaunchArgument(
        'safety_threshold',
        default_value='0.15',
        description='Safety distance threshold in meters'
    )
    
    declare_hysteresis_threshold_cmd = DeclareLaunchArgument(
        'hysteresis_threshold',
        default_value='0.20',
        description='Hysteresis threshold to prevent oscillation'
    )
    
    declare_enable_emergency_stop_cmd = DeclareLaunchArgument(
        'enable_emergency_stop',
        default_value='True',
        description='Enable emergency stop functionality'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    safety_threshold = LaunchConfiguration('safety_threshold')
    hysteresis_threshold = LaunchConfiguration('hysteresis_threshold')
    enable_emergency_stop = LaunchConfiguration('enable_emergency_stop')
    
    # Ultrasonic emergency stop node
    ultrasonic_emergency_stop_node = Node(
        package='ia_robot',
        executable='ultrasonic_emergency_stop.py',
        name='ultrasonic_emergency_stop',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'safety_threshold': safety_threshold,
            'hysteresis_threshold': hysteresis_threshold,
            'enable_emergency_stop': enable_emergency_stop,
            # Eight corner sensors - 2 per corner (forward-facing and side-facing)
            'ultrasonic_front_left_forward_topic': '/ultrasonic/front_left_forward',
            'ultrasonic_front_left_side_topic': '/ultrasonic/front_left_side',
            'ultrasonic_front_right_forward_topic': '/ultrasonic/front_right_forward',
            'ultrasonic_front_right_side_topic': '/ultrasonic/front_right_side',
            'ultrasonic_rear_left_forward_topic': '/ultrasonic/rear_left_forward',
            'ultrasonic_rear_left_side_topic': '/ultrasonic/rear_left_side',
            'ultrasonic_rear_right_forward_topic': '/ultrasonic/rear_right_forward',
            'ultrasonic_rear_right_side_topic': '/ultrasonic/rear_right_side',
            'emergency_cmd_vel_topic': '/cmd_vel_emergency',
            'emergency_status_topic': '/emergency_stop_status',
            'publish_rate': 20.0,  # 20Hz to ensure fast response
            'sensor_timeout': 1.0,
        }],
        remappings=[
            # Topic remappings can be added here
        ]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch parameters
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_safety_threshold_cmd)
    ld.add_action(declare_hysteresis_threshold_cmd)
    ld.add_action(declare_enable_emergency_stop_cmd)
    
    # Add nodes
    ld.add_action(ultrasonic_emergency_stop_node)
    
    return ld
