# 导入必要模块：文件操作、ROS2 launch核心组件、参数重写工具等
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml  # 用于动态重写参数文件
from launch_ros.actions import Node


def generate_launch_description():
    """生成导航栈启动描述，整合定位、路径规划、控制等导航核心功能"""
    
    # 获取功能包资源目录：用于定位launch文件、配置文件等资源
    bringup_dir = get_package_share_directory('wheeltec_robot_rrt')  # 主功能包目录
    launch_dir = os.path.join(bringup_dir, 'launch', 'navigation')    # 导航相关launch文件目录

    # 声明启动配置变量（可通过命令行参数覆盖）
    namespace = LaunchConfiguration('namespace')               # 命名空间（用于多机器人）
    use_namespace = LaunchConfiguration('use_namespace')       # 是否启用命名空间
    slam = LaunchConfiguration('slam')                         # 是否启用SLAM建图模式
    map_yaml_file = LaunchConfiguration('map')                 # 地图文件路径（定位模式使用）
    use_sim_time = LaunchConfiguration('use_sim_time')         # 是否使用仿真时间（Gazebo等模拟器）
    params_file = LaunchConfiguration('params_file')           # 导航参数配置文件路径
    autostart = LaunchConfiguration('autostart')               # 是否自动启动导航栈
    use_composition = LaunchConfiguration('use_composition')   # 是否使用组件化启动（节点容器）
    use_respawn = LaunchConfiguration('use_respawn')           # 节点崩溃时是否自动重启（非组件化模式）

    # 话题重映射：将全局tf话题映射到当前命名空间（避免多机器人冲突）
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
                  
    # 参数替换规则：动态将启动配置变量注入到参数文件中
    param_substitutions = {
        'use_sim_time': use_sim_time,      # 替换参数文件中的use_sim_time值
        'yaml_filename': map_yaml_file}    # 替换参数文件中的地图路径值
        
    # 重写参数文件：根据上述替换规则生成最终参数配置
    configured_params = RewrittenYaml(
        source_file=params_file,           # 原始参数文件
        root_key=namespace,                # 按命名空间划分参数（多机器人隔离）
        param_rewrites=param_substitutions,# 应用参数替换
        convert_types=True)                # 自动转换参数类型（字符串->布尔/数值等）
            
    # 设置环境变量：启用ROS2日志缓冲流（优化日志输出性能）
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 声明启动参数（可通过命令行--ros-args设置）
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')  # 导航栈顶层命名空间

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')  # 是否启用命名空间隔离

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')  # true：启动SLAM建图；false：启动定位模式

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')  # 定位模式下加载的地图文件路径

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')  # 是否使用模拟器时间

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')  # 导航参数文件默认路径

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')  # 是否自动激活导航栈

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')  # 是否使用组件化启动（节点整合到容器中）

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')  # 非组件化模式下节点崩溃重启开关


    # 导航栈核心动作组：组织命名空间、节点容器、定位和导航模块
    bringup_cmd_group = GroupAction([
        # 命名空间推送：若启用命名空间，则所有子节点使用该命名空间
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
            
        # 导航组件容器：当使用组件化启动时，所有导航节点作为组件加载到该容器中
        Node(
            condition=IfCondition(use_composition),  # 仅在组件化模式下启动
            name='nav2_container',                   # 容器名称
            package='rclcpp_components',             # 组件容器包
            executable='component_container_isolated',# 隔离式组件容器（避免符号冲突）
            parameters=[configured_params, {'autostart': autostart}],  # 容器参数（含导航参数和自动启动开关）
            remappings=remappings,                   # 应用话题重映射
            ),

        # 定位模块：仅在非SLAM模式下启动（SLAM模式已包含位姿估计）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),  # slam=false时启动
            launch_arguments={                      # 传递给定位模块的参数
                'namespace': namespace,
                'map': map_yaml_file,               # 地图文件路径
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'}.items()),  # 组件化时的容器名称


        # 导航模块：包含路径规划、控制器、恢复行为等核心导航功能
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_navigation_launch.py')),
            launch_arguments={                      # 传递给导航模块的参数
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'}.items()),
    ])

    # 组装启动描述
    ld = LaunchDescription()

    # 设置环境变量（日志缓冲）
    ld.add_action(stdout_linebuf_envvar)

    # 添加启动参数声明（顺序不影响，仅声明参数）
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    
    # 添加导航栈核心动作组（按添加顺序启动）
    ld.add_action(bringup_cmd_group)

    return ld
