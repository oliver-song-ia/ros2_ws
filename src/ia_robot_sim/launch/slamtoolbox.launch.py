from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        )
    )

    # Navigation2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        )
    )



    # PointCloud to LaserScan using ExecuteProcess (NO Node)
    pointcloud_to_laserscan_proc = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            '--ros-args',
            '-r', 'cloud_in:=/point_cloud',
            '-r', 'scan:=/scan',
            '-p', 'angle_min:=-3.1415926',
            '-p', 'angle_max:=3.1415926',
            '-p', 'angle_increment:=0.0174533',
            '-p', 'range_min:=1.1',
            '-p', 'range_max:=10.0',
            '-p', 'min_height:=-0.24',
            '-p', 'max_height:=1.0',
            '-p', 'scan_time:=0.1',
            '-p', 'target_frame:=lidar_frame'
        ],
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox_launch,
        nav2_launch,

        pointcloud_to_laserscan_proc
    ])
