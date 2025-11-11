import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='ia_cartographer').find('ia_cartographer')
    
    # 建图模式配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.03')  # 地图分辨率
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')  # 发布周期
    
    configuration_directory = LaunchConfiguration('configuration_directory',
                                                 default=os.path.join(pkg_share, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', 
                                              default='ia_lds_2d.lua')  # 使用建图配置文件

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'purelocation_or_slam': False}],
        arguments=['-configuration_directory', configuration_directory,
                  '-configuration_basename', configuration_basename,
                  '-resolution', resolution],
        remappings=[
            ('scan', '/scan'),    # 激光雷达话题
            ('imu', '/imu')       # IMU话题
        ])

    # 地图发布节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'purelocation_or_slam': False},
                    {'pure_localization_mode': False}],
        arguments=['-resolution', resolution,
                  '-publish_period_sec', publish_period_sec])
    rviz_config_dir = os.path.join(pkg_share, 'rviz', 'ia_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    

    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)

    return ld
