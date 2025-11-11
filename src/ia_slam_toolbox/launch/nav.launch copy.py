from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数（保留原地图路径和仿真时间参数，新增导航配置参数）
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('ia_cartographer'), 'map', 'ia_map.yaml'),
        description='Path to the map YAML file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # 新增：导航参数文件路径（包含全局/局部代价地图、避障配置等）
    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ia_cartographer'),  # 替换为你的功能包名
            'config',
            'param_mini_akm.yaml'
        ]),
        description='Path to Nav2 parameters file'
    )

    # 1. 地图服务器（全局地图加载）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # 2. 规划器服务器（全局路径规划）
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[LaunchConfiguration('nav_params_file')],
        output='screen'
    )

    # 3. 控制器服务器（局部路径跟踪与避障控制）
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[LaunchConfiguration('nav_params_file')],
        output='screen'
    )

    # 4. 行为服务器（恢复行为，如避障失败时的重试逻辑）
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[LaunchConfiguration('nav_params_file')],
        output='screen'
    )

    # 5. 行为树导航器（高层导航逻辑调度）
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[LaunchConfiguration('nav_params_file')],
        output='screen'
    )

    # 6. 生命周期管理器（统一管理所有导航节点的启动/关闭）
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time_arg,
        declare_nav_params_arg,  # 新增：导航参数声明
        map_server_node,
        # 新增：导航核心组件（替换原move_base，符合Nav2架构）
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node  # 新增：生命周期管理
    ])