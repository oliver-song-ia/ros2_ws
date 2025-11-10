import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # LiDAR driver configuration
    Lslidar_dir = get_package_share_directory('lslidar_driver')
    Lslidar_launch_dir = os.path.join(Lslidar_dir, 'launch')

    # Pointcloud to laserscan converter configuration
    pointcloud_to_laserscan_dir = get_package_share_directory('pointcloud_to_laserscan')
    pointcloud_to_laserscan_launch_dir = os.path.join(pointcloud_to_laserscan_dir, 'launch')

    # LSlidar C16 driver launch
    LSC16 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Lslidar_launch_dir, 'lslidar_cx_launch.py')),)

    # Convert 3D point cloud to 2D laser scan
    point_to_scan = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pointcloud_to_laserscan_launch_dir, 'pointcloud_to_laserscan_launch.py')),)

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(LSC16)
    ld.add_action(point_to_scan)
    return ld

