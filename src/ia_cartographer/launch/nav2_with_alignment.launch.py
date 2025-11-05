#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('ia_cartographer')
    
    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    alignment_points_file = LaunchConfiguration('alignment_points_file')
    behavior_tree_file = LaunchConfiguration('behavior_tree_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('ia_robot_sim'), 'maps', 'ia_map.yaml'),
        description='Full path to the ROS2 map yaml file to use')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('ia_robot_sim'), 'config', 'nav2_params_real.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_alignment_points_file_cmd = DeclareLaunchArgument(
        'alignment_points_file',
        default_value=os.path.join(get_package_share_directory('ia_robot_sim'), 'maps', 'map_0722_empty.json'),
        description='Full path to the alignment points JSON file')

    declare_behavior_tree_file_cmd = DeclareLaunchArgument(
        'behavior_tree_file',
        default_value=os.path.join(get_package_share_directory('ia_robot_sim'), 'behavior_trees', 'navigate_backup_first.xml'),
        description='Full path to behavior tree xml file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM or localization')

    # Create our own yaml files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'alignment_points_file': alignment_points_file,
        'default_nav_to_pose_bt_xml': behavior_tree_file,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Include the regular nav2 bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'slam': slam,
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_alignment_points_file_cmd)
    ld.add_action(declare_behavior_tree_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_cmd)


    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_launch)

    return ld