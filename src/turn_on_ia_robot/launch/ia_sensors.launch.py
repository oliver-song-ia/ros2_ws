import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.conditions import IfCondition

#def launch(launch_descriptor, argv):
def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_ia_robot')
    ia_robot_dir = get_package_share_directory('ia_robot')
    imu_launch_dir = get_package_share_directory('wit_ros2_imu')
    imu_config_dir = os.path.join(imu_launch_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    launch_dir = os.path.join(bringup_dir, 'launch')
    # 底盘通讯节点启动
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_ia_robot.launch.py')),
    )
    # 雷达节点启动
    lidar_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ia_lidar.launch.py')),
    )
    # imu节点启动
    ahrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('fdilink_ahrs'),  # 获取 fdilink_ahrs 包的路径
            '/launch/ahrs_driver.launch.py'  # ahrs_driver.launch.py 相对于包的路径
        ])
    )
    # 旧版imu节点启动
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_config_dir,  'rviz_and_imu.launch.py')),)

    # 底盘控制计算
    swerve_ctrl = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ia_robot_dir, 'launch', 'ctrl_swerve_drive.launch.py')),
    )
    # rviz节点启动
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',  # 将rviz2输出打印到终端
        # 如果需要加载特定配置文件，可以添加以下参数
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'laser_2d.rviz')]
    )
    return LaunchDescription([
        wheeltec_robot,  #base_control
        lidar_ros,      # lidar
        ahrs_launch,  #IMU
        # imu_launch,     #IMU
        swerve_ctrl,    
        # rviz_node,    #rviz

        ]
    )


