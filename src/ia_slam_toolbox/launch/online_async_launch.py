from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    slam_dir = get_package_share_directory('wheeltec_slam_toolbox')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_sensors.launch.py')),
    )
    
    # wheeltec_lidar = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    # )

    # 启动rviz2节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',  # 将rviz2输出打印到终端
        # 如果需要加载特定配置文件，可以添加以下参数
        arguments=['-d', os.path.join(slam_dir, 'rviz', 'slam.rviz')]
    )
    
    return LaunchDescription([
        wheeltec_robot,
        # wheeltec_lidar,
        rviz_node,
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("wheeltec_slam_toolbox") + '/config/mapper_params_online_async.yaml'
        	],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[('odom','odom_combined')]
        )
    ])
