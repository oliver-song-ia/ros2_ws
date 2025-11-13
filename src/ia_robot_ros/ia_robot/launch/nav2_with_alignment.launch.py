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
    # 获取 nav2_bringup 和 ia_robot 包的共享目录
    bringup_dir = get_package_share_directory('nav2_bringup')
    ia_robot_dir = get_package_share_directory('ia_robot')
    
    # 获取配置文件路径
    twist_mux_config = os.path.join(ia_robot_dir, 'config', 'twist_mux.yaml')

    # 创建启动配置变量
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    alignment_points_file = LaunchConfiguration('alignment_points_file')
    behavior_tree_file = LaunchConfiguration('behavior_tree_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam = LaunchConfiguration('slam')
    enable_emergency_stop = LaunchConfiguration('enable_emergency_stop')

    # 声明启动参数
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('ia_robot'), 'maps', 'ia_map.yaml'),
        description='ROS2 地图文件的完整路径')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('ia_robot'), 'config', 'nav2_params_real.yaml'),
        description='所有启动节点使用的 ROS2 参数文件的完整路径')

    declare_alignment_points_file_cmd = DeclareLaunchArgument(
        'alignment_points_file',
        default_value=os.path.join(get_package_share_directory('ia_robot'), 'maps', 'map_0722_empty.json'),
        description='对齐点 JSON 文件的完整路径')

    declare_behavior_tree_file_cmd = DeclareLaunchArgument(
        'behavior_tree_file',
        default_value=os.path.join(get_package_share_directory('ia_robot'), 'behavior_trees', 'navigate_backup_first.xml'),
        description='行为树 XML 文件的完整路径')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='是否使用仿真（Gazebo）时钟，如果为 True 则使用仿真时间')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='是否自动启动 nav2 栈')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='是否运行 SLAM 或定位')
    
    declare_enable_emergency_stop_cmd = DeclareLaunchArgument(
        'enable_emergency_stop',
        default_value='True',
        description='是否启用超声波紧急停止安全系统')

    # 创建包含替换项的 yaml 文件
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'alignment_points_file': alignment_points_file,  # 修正了这里缺少逗号的错误
        'default_nav_to_pose_bt_xml': behavior_tree_file,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # 包含常规的 nav2 bringup 启动文件
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
    
    # 包含安全系统（超声波紧急停止 + twist_mux）
    safety_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ia_robot_dir, 'launch', 'safety_system.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'safety_threshold': '0.15',
            'hysteresis_threshold': '0.20',
            'enable_emergency_stop': enable_emergency_stop,
        }.items(),
        condition=IfCondition(enable_emergency_stop)  # 仅当启用紧急停止时才包含此启动文件
    )
    
    # Twist Mux 节点 - 始终运行以管理速度命令优先级
    # 这确保了导航命令 (/cmd_vel_nav) 和最终机器人命令 (/cmd_vel) 的清晰分离
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # twist_mux 将根据配置文件处理主题映射
            # 主输出是 /cmd_vel 到机器人
            ('cmd_vel_out', 'cmd_vel'),
        ]
    )

    rviz_config_file = os.path.join(ia_robot_dir, 'rviz', 'nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    # 创建启动描述并填充
    ld = LaunchDescription()

    # 声明启动选项
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_alignment_points_file_cmd)
    ld.add_action(declare_behavior_tree_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_enable_emergency_stop_cmd)
    ld.add_action(rviz_node)
    # 添加启动所有导航节点的动作
    ld.add_action(bringup_launch)
    
    # # 添加 twist_mux（始终运行以管理速度命令）
    # ld.add_action(twist_mux_node)
    
    # # 添加安全系统（条件运行 - 仅在启用紧急停止时运行）
    # ld.add_action(safety_system_launch)

    return ld
