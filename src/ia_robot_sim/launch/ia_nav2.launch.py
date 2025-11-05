import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
        
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2_launcher = os.path.join(nav2_dir, 'launch')

    ia_robot_dir = get_package_share_directory('ia_robot_sim')
    map_file = LaunchConfiguration('maps', default=os.path.join(
        ia_robot_dir, 'maps', 'map_0722.yaml'))


    param_dir = os.path.join(ia_robot_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'nav2_params_shim.yaml'))


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launcher, '/localization_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file,
                'autostart': autostart}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launcher, '/navigation_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file,
                'autostart': autostart}.items(),
        ),

    ])
