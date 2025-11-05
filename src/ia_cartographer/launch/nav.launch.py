import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam = LaunchConfiguration('slam', default='false')  # 添加slam参数定义
        
    wheeltec_nav_dir = get_package_share_directory('ia_cartographer')
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, 'launch')

    map_dir = os.path.join(wheeltec_nav_dir, 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'ia_map.yaml'))

    param_dir = os.path.join(wheeltec_nav_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'param_mini_omni.yaml'))
    
    declare_behavior_tree_file_cmd = DeclareLaunchArgument(
        'behavior_tree_file',
        default_value=os.path.join(get_package_share_directory('ia_robot_sim'), 'behavior_trees', 'navigate_backup_first.xml'),
        description='Full path to behavior tree xml file')
    # rviz
    rviz_config_dir = os.path.join(wheeltec_nav_dir, 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'ia_cartographer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # 首先声明参数
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Whether run a SLAM'),
            
        # RViz
        # rviz_node,
        
        # Waypoint follower
        Node(
            name='waypoint_follower',
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            parameters=[param_file, {'use_sim_time': use_sim_time}]
        ),
        
        # 启动bringup，它会处理map_server和其他导航节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(wheeltec_nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file,
                'slam': slam  # 使用定义的slam参数
            }.items(),
        )
    ])
