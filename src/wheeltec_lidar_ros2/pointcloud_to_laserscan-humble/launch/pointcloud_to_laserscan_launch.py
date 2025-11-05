# 用于启动pointcloud_to_laserscan节点，将3D点云数据转换为2D激光雷达扫描数据的启动文件
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 声明"scanner"启动参数，用于设置命名空间（默认值为"scanner"）
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar_frame'
            ]
        ),

        # 启动pointcloud_to_laserscan节点，实现点云到激光扫描数据的转换
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # 话题重映射：输入点云话题(cloud_in)映射到/point_cloud_raw，输出激光扫描话题(scan)映射到/scan
            remappings=[('cloud_in','/point_cloud_raw'),
                        ('scan','/scan')],
            # 点云转激光扫描的核心参数配置
            parameters=[{
                'target_frame': '',           # 激光扫描数据的目标坐标系
                'transform_tolerance': 0.01,       # 坐标系转换的容忍时间(秒)
                'min_height': -0.1,                # 点云高度过滤下限(米)
                'max_height': 1.5,                 # 点云高度过滤上限(米)
                'angle_min': -3.14159,  # -M_PI/2  # 扫描角度下限(弧度，-180°)
                'angle_max': 3.14159,  # M_PI/2   # 扫描角度上限(弧度，180°)
                'angle_increment': 0.007,  # M_PI/360.0  # 角度分辨率(弧度，约0.5°)
                'scan_time': 0.1,                  # 扫描周期(秒)
                'range_min': 0.8,                  # 最小检测距离(米)
                'range_max': 20.0,                # 最大检测距离(米)
                'use_inf': True,                   # 是否使用无穷远值表示超出最大距离的点
                'inf_epsilon': 1.0               # 无穷远值的判定阈值
            }],

            name='pointcloud_to_laserscan'        # 节点名称
        )
    ])
