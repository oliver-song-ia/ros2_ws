import os
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


def generate_launch_description():
    # 定义EKF节点的配置文件路径：常规模式和cartographer建图模式
    ekf_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'ekf.yaml')
    ekf_carto_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'ekf_carto.yaml')

    # 声明是否启用cartographer建图模式的启动参数（默认关闭）
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='false',
                                          description='是否启用cartographer建图模式下的EKF配置')
             
    return LaunchDescription([
        carto_slam_dec,  # 添加启动参数声明
        
        # 条件节点1：当启用cartographer建图时启动（使用cartographer专用EKF配置）
        Node(
            condition=IfCondition(carto_slam),
            package='robot_localization',
            executable='ekf_node',
            name='carto_ekf_filter_node',  # cartographer模式下的EKF节点名称
            parameters=[ekf_carto_config],  # 加载cartographer模式的EKF配置
            remappings=[('/odometry/filtered','odom_combined')]  # 重映射输出的里程计话题
        ),

        # 条件节点2：当未启用cartographer建图时启动（使用常规EKF配置）
        Node(
            condition=UnlessCondition(carto_slam),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',  # 常规模式下的EKF节点名称
            parameters=[ekf_config],  # 加载常规EKF配置
            remappings=[('/odometry/filtered','odom_combined')]  # 重映射输出的里程计话题
        ),
    ])
