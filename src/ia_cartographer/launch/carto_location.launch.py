import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument 
from launch_ros.actions import Node 
from launch.substitutions import LaunchConfiguration 
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch.substitutions import ThisLaunchFileDir 
 
 
def generate_launch_description(): 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    turtlebot3_cartographer_prefix = get_package_share_directory('ia_cartographer') 
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join( 
                                                    turtlebot3_cartographer_prefix, 'config')) 
    configuration_basename = LaunchConfiguration('configuration_basename', 
                                                 default='ia_location.lua') 
 
    resolution = LaunchConfiguration('resolution', default='0.03') 
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0') 
    publish_occupancy_grid = LaunchConfiguration('publish_occupancy_grid', default='false')  # 添加这行
 
    rviz_config_dir = os.path.join(turtlebot3_cartographer_prefix, 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'ia_cartographer.rviz')
    pbstream_path = "/home/ia/fangfudong/ros2_ws/src/ia_cartographer/maps/ia_map.pbstream" 
    
    return LaunchDescription([ 
        DeclareLaunchArgument( 
            'cartographer_config_dir', 
            default_value=cartographer_config_dir, 
            description='Full path to config file to load'), 
        DeclareLaunchArgument( 
            'configuration_basename', 
            default_value=configuration_basename, 
            description='Name of lua file for cartographer'), 
        DeclareLaunchArgument( 
            'use_sim_time', 
            default_value='false', 
            description='Use simulation (Gazebo) clock if true'), 
        DeclareLaunchArgument( 
            'publish_occupancy_grid', 
            default_value='false', 
            description='Enable/disable occupancy grid publishing'),  # 添加这行
 
        Node( 
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node', 
            output='screen', 
            parameters=[{'use_sim_time': use_sim_time}],  # 移除错误的publish_occupancy_grid参数
            arguments=['-configuration_directory', cartographer_config_dir, 
                       '-configuration_basename', configuration_basename, 
                       '-load_state_filename', pbstream_path, 
                        '-pure_localization_mode', 'true',
                       '-load_frozen_state', 'true'], 
            remappings=[('imu', '/imu'), 
                       ('scan', '/scan')]), 
        
        DeclareLaunchArgument( 
            'resolution', 
            default_value=resolution, 
            description='Resolution of a grid cell in the published occupancy grid'), 
 
        DeclareLaunchArgument( 
            'publish_period_sec', 
            default_value=publish_period_sec, 
            description='OccupancyGrid publishing period'), 
 
        # 直接添加occupancy_grid_node，而不是引用不存在的文件
        # Node(
        #     package='cartographer_ros',
        #     executable='cartographer_occupancy_grid_node',
        #     name='cartographer_occupancy_grid_node',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': use_sim_time},
        #         {'resolution': resolution}
        #     ],
        #     arguments=[
        #         # '-publish_occupancy_grid', publish_occupancy_grid,  # 正确传递参数
        #         '-resolution', resolution,
        #         '-publish_period_sec', publish_period_sec
        #     ]
        # ),
        # ros2 run ia_cartographer carto_set_initial_pose_node
        Node(
            package='ia_cartographer',
            executable='carto_set_initial_pose_node',
            name='carto_set_initial_pose_node',
            output='screen',
        ),
 
        Node( 
            package='rviz2', 
            executable='rviz2', 
            name='rviz2', 
            arguments=['-d', rviz_config_file], 
            parameters=[{'use_sim_time': use_sim_time}], 
            output='screen'), 

        
    ])