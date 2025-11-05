# 系统模块：文件路径处理
import os
import pathlib

# ROS2 核心工具：包路径获取、launch描述生成、节点定义等
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# XACRO处理器：将XACRO模型文件转换为URDF
import xacro


def generate_launch_description():
    """生成Gazebo仿真控制所需的完整Launch描述"""
    
    # -------------------------- 1. 路径与资源定位 --------------------------
    # 获取当前功能包(ia_robot_sim)的share目录
    package_share_dir = get_package_share_directory('ia_robot_sim')
    
    # 定义XACRO模型文件路径与URDF输出路径
    xacro_file = os.path.join(package_share_dir, 'description', 'robot.urdf.xacro')
    urdf_path = os.path.join(package_share_dir, 'description', 'robot.urdf')
    
    # 控制器配置文件路径
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ia_robot_sim"),
        "config",
        "swerve_ctrl_parameter.yaml",  # 确保此文件已定义控制器类型
    ])


    # -------------------------- 2. 机器人模型处理 --------------------------
    # 处理XACRO文件（启用仿真模式）并生成URDF
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    
    # 将URDF内容写入文件（可选，用于调试）
    with open(urdf_path, 'w') as f:
        f.write(robot_desc)
    
    # 定义机器人描述参数（供节点使用）
    params = {'robot_description': robot_desc, 'use_sim_time': True}  # 同步仿真时间


    # -------------------------- 3. 核心节点创建 --------------------------
    # 3.1 仿真环境相关节点
    # Gazebo空世界启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": "empty.world",
            "verbose": "false"  # 关闭冗余日志
        }.items(),
    )
    
    # 机器人模型Spawn到Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "ia_robot"],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 3.2 状态发布相关节点
    # 机器人状态发布（TF变换与模型描述）
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[params],
    )
    
    # 3.3 控制相关节点（ros2_control核心）
    # 控制节点（ros2_control_node）
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output="both",
        emulate_tty=True  # 启用TTY日志颜色
    )
    
    # 控制器延迟加载器
    delayed_controller_loader = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        # 关节状态广播器（直接激活）
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "joint_state_broadcaster",
                                "--controller-manager", "/controller_manager",
                                "--activate",  # 直接激活控制器
                            ],
                            name="joint_state_spawner",
                        ),
                        # 底盘控制器（直接激活）
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "swerve_drive_controller",
                                "--controller-manager", "/controller_manager",
                                "--activate",
                            ],
                            name="swerve_controller_spawner",
                        )
                    ]
                )
            ]
        )
    )


    # -------------------------- 4. 组装启动描述 --------------------------
    return LaunchDescription([
        # 仿真环境
        gazebo,
        # 模型与状态
        spawn_entity,
        robot_state_pub_node,
        # 控制器与广播器
        control_node,
        delayed_controller_loader
    ])
