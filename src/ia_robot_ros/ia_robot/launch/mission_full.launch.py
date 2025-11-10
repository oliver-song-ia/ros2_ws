#!/usr/bin/env python3
"""
Full mission launch file
Launches all components needed for navigation + manipulation mission
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM'
    )
    
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto-start mission when human pose detected'
    )
    
    declare_nav_timeout = DeclareLaunchArgument(
        'navigation_timeout',
        default_value='120.0',
        description='Navigation timeout in seconds'
    )
    
    declare_manip_timeout = DeclareLaunchArgument(
        'manipulation_timeout',
        default_value='60.0',
        description='Manipulation timeout in seconds'
    )
    
    declare_launch_nav = DeclareLaunchArgument(
        'launch_nav',
        default_value='true',
        description='Launch navigation stack'
    )
    
    declare_launch_genesis = DeclareLaunchArgument(
        'launch_genesis',
        default_value='false',
        description='Launch Genesis simulation (usually started separately)'
    )

    declare_python_interpreter = DeclareLaunchArgument(
        'python_interpreter',
        default_value='~/miniforge3/envs/genesis/bin/python',
        description='Path to Python interpreter for spawning manipulation processes'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    auto_start = LaunchConfiguration('auto_start')
    nav_timeout = LaunchConfiguration('navigation_timeout')
    manip_timeout = LaunchConfiguration('manipulation_timeout')
    launch_nav = LaunchConfiguration('launch_nav')
    launch_genesis = LaunchConfiguration('launch_genesis')
    python_interpreter = LaunchConfiguration('python_interpreter')
    
    # Find package paths
    ia_robot_dir = FindPackageShare('ia_robot')
    gs_ros2_control_demos_dir = FindPackageShare('gs_ros2_control_demos')
    
    # Mission BT Node
    mission_bt_node = Node(
        package='ia_robot_bt',
        executable='mission_bt_node',
        name='mission_bt_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'manipulation_timeout': manip_timeout,
            'python_interpreter': python_interpreter,
        }],
        remappings=[
            ('/goal_pose', '/goal_pose'),
            ('/mission_trigger', '/mission_trigger'),
        ]
    )
    
    # Include Navigation Launch (Nav2 + SLAM)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ia_robot_dir,
                'launch',
                'nav2_with_alignment.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,
        }.items(),
        condition=IfCondition(launch_nav)
    )
    
    # Include gs_util launch (TF, RViz, etc.)
    gs_util_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ia_robot_dir,
                'launch',
                'gs_util.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(launch_nav)
    )
    
    # Include perception launch (human tracking and pose detection)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ia_robot_dir,
                'launch',
                'perception.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'python_interpreter': python_interpreter,
        }.items()
    )
    
    
    
    # Optional: Genesis simulation launch
    # Note: Usually started separately with:
    # colcon build --packages-select gs_ros ia_robot ia_robot_urdf gs_ros2_control_demos swerve_drive_controller && python src/genesis_ros/test_import.py
    
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_slam,
        declare_auto_start,
        declare_nav_timeout,
        declare_manip_timeout,
        declare_launch_nav,
        declare_launch_genesis,
        declare_python_interpreter,

        # Launch nodes and includes
        mission_bt_node,
        gs_util_launch,
        nav2_launch,
        perception_launch,
    ])
