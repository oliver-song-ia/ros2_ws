import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():
    # 加载 ia_cartographer 包的 launch 文件
    bringup_dir = get_package_share_directory('ia_cartographer')
    base_dir = get_package_share_directory('turn_on_wheeltec_robot')
    base_bringup_dir = os.path.join(base_dir, 'launch')
    cartographer_launch_dir = os.path.join(bringup_dir, 'launch')




    # 启动底盘+IMU+Lidar
    robot_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(base_bringup_dir, 'wheeltec_sensors.launch.py')),
    )

    # location 节点
    location_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cartographer_launch_dir, 'carto_location.launch.py')),
    )

    # navigation 节点
    navigation_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cartographer_launch_dir, 'nav.launch.py')),
    )




    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',  # 将rviz2输出打印到终端
    #     # 如果需要加载特定配置文件，可以添加以下参数
    #     arguments=['-d', os.path.join(bringup_dir, 'rviz', 'ia_cartographer.rviz')]
    # )
#     return LaunchDescription([
#         robot_bringup,
#         location_bringup,
#         navigation_bringup,
#         # rviz_node
#         ]
#     )


