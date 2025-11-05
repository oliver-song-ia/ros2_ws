# 导入系统模块：用于文件路径处理
import os
# 导入ROS2 Launch核心模块：定义launch描述结构
from launch import LaunchDescription
# 导入Launch配置工具：处理参数配置与替换
from launch.substitutions import LaunchConfiguration
# 导入Launch动作类：用于包含其他launch文件、执行进程、注册事件等
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
# 导入事件处理器：处理进程退出等事件
from launch.event_handlers import OnProcessExit
# 导入Launch描述源：指定Python格式的launch文件
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 导入ROS2包路径工具：获取功能包的share目录
from ament_index_python import get_package_share_directory
# 导入条件判断工具：基于条件决定是否执行某些动作
from launch.conditions import IfCondition
# 导入ROS2节点工具：定义要启动的ROS节点
from launch_ros.actions import Node
# 导入进程执行工具：执行外部进程（此处冗余导入，可考虑清理）
from launch.actions import ExecuteProcess


def generate_launch_description():
    """生成ROS2 Launch描述的主函数，定义系统启动所需的所有组件与参数"""
    
    # 定义"使用仿真时间"参数，默认值为False（实际机器人模式）
    # 可通过命令行参数覆盖，用于切换仿真/真实环境时间源
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    # 将"use_sim_time"声明为launch参数，允许外部传入该参数值
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    
    # 获取功能包"wheeltec_robot_rrt"的share目录路径，用于定位内部资源
    wheeltec_rrt_dir = get_package_share_directory('wheeltec_robot_rrt')
    # 获取SLAM功能包"wheeltec_slam_toolbox"的share目录路径
    wheeltec_slam_dir = get_package_share_directory('wheeltec_slam_toolbox')
    
    # 包含Nav2导航系统的launch文件（导航栈启动配置）
    nav2_include = IncludeLaunchDescription(
        # 指定Nav2启动文件路径：nav2_bringup_launch.py
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_rrt_dir, 'launch', 'navigation', 'nav2_bringup_launch.py')),
        # 传递给Nav2的启动参数
        launch_arguments={
            'slam': "True",  # 启用SLAM模式（实时建图）
            'map': os.path.join(wheeltec_rrt_dir, 'map_data', 'image_map.yaml'),  # 地图文件路径（SLAM模式下可能未使用）
            'use_sim_time': use_sim_time,  # 传递仿真时间参数
            'params_file': os.path.join(wheeltec_rrt_dir, 'config', 'nav2_params.yaml'),  # Nav2参数配置文件
            'default_bt_xml_filename': os.path.join(wheeltec_rrt_dir, 'behaviour_trees', 'navigate_w_replanning_time.xml'),  # 行为树XML文件
            'autostart': 'True'  # 是否自动启动导航系统
        }.items()  # 将字典转换为键值对列表
    )

    # 包含RRT探索算法的launch文件（启动路径规划探索模块）
    rrt_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_rrt_dir, 'launch', 'rrt_exploration', 'rrt_exploration.launch.py')
        ),
    )
    
    # 包含SLAM工具的launch文件（启动在线同步建图节点）
    wheeltec_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_slam_dir, 'launch', 'online_sync.launch.py')
        ),
    )
    
    # 包含动作服务器的launch文件（启动RRT探索所需的动作服务）
    action_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_rrt_dir, 'launch', 'rrt_exploration', 'action_servers.launch.py')
        )
    )

    # 定义"robot_picker"节点（机器人选择器，可能用于多机协调或任务分配）
    robot_picker = Node(
        package='wheeltec_robot_rrt',  # 节点所属功能包
        executable='robot_picker',     # 节点可执行文件名
        name='robot_picker',           # 节点名称（ROS图中显示的名称）
        # prefix=['xterm -e gdb -ex run --args'],  # 调试前缀（当前注释掉，未启用）
        # 节点参数：指定行为树XML文件路径
        parameters=[{'bt_xml_filename': os.path.join(
            get_package_share_directory("wheeltec_robot_rrt"), 
            'behaviour_trees', 
            'robot_picker_behaviour_tree.xml'
        )}]
    )

    # 定义"robot_pose_publisher"节点（机器人位姿发布器，发布当前位姿信息）
    robot_pose_publisher = Node(
        package='wheeltec_robot_rrt',  # 节点所属功能包
        executable='robot_pose_publisher',  # 节点可执行文件名
        name='robot_pose_publisher',   # 节点名称
    )
    
    # 构建并返回Launch描述对象，包含所有待启动的组件
    return LaunchDescription([
        # wheeltec_slam,     # 注释掉的SLAM节点（当前未启用）
        nav2_include,      # 注释掉的Nav2导航节点（当前未启用）
        rrt_exploration,        # RRT探索算法节点
        action_servers,         # 动作服务器节点
        robot_picker,           # 机器人选择器节点
        robot_pose_publisher,   # 位姿发布器节点
        use_sim_time_arg        # 仿真时间参数声明
    ])
