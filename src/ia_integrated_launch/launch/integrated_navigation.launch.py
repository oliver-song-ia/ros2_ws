from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 启动SLAM工具箱
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('wheeltec_slam_toolbox'),
                '/launch/online_async_launch.py'
            ])
        ),
        
        # 启动Swerve控制器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ia_robot'),
                '/launch/ctrl_swerve_drive.launch.py'
            ])
        ),
        
        # 启动导航系统
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ia_robot'),
                '/launch/nav2_with_alignment.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'slam': 'True'
            }.items()
        ),
        
        # 启动自主探索建图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('explore_lite'),
                '/launch/explore.launch.py'
            ])
        )
    ])
