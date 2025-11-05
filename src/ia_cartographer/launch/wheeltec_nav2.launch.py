"""
  Cartographer ROS2 导航系统启动文件
  基于您的定位代码集成导航栈
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import SetRemap

def generate_launch_description():
    # 配置变量
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('ia_cartographer')
    
    # Cartographer配置
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
        turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', 
                                                 default='ia_location.lua')
    
    # 导航参数
    resolution = LaunchConfiguration('resolution', default='0.03')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    publish_occupancy_grid = LaunchConfiguration('publish_occupancy_grid', default='true')
    
    # 导航栈参数
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        turtlebot3_cartographer_prefix, 'config', 'nav2_params.yaml'))
    
    # 地图和路径
    rviz_config_dir = os.path.join(turtlebot3_cartographer_prefix, 'rviz', 'slam.rviz')
    pbstream_path = LaunchConfiguration('pbstream_path', default="/home/ia/fangfudong/ros2_ws/src/ia_cartographer/map/ia_map.pbstream")
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='配置文件路径'),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Cartographer配置文件名'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间'),
        
        DeclareLaunchArgument(
            'publish_occupancy_grid',
            default_value='true',
            description='是否发布占用栅格地图'),
        
        # Cartographer节点（定位）
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', pbstream_path,
                       '-load_frozen_state', 'true'],
            remappings=[('imu', '/imu'),
                        ('scan', '/scan')]),
        
        # 占用栅格节点
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'resolution': resolution},
                {'publish_period_sec': publish_period_sec}
            ],
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period_sec
            ],
            condition=IfCondition(publish_occupancy_grid)),
        
        # 导航栈节点
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                       {'autostart': True},
                       {'node_names': ['controller_server',
                                       'planner_server',
                                       'recoveries_server',
                                       'bt_navigator']}]),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])