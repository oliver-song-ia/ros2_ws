import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def generate_launch_description():
    # 获取功能包路径和启动文件目录，用于定位后续的配置文件和子启动文件
    bringup_dir = get_package_share_directory('turn_on_ia_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
        
    # 定义EKF（扩展卡尔曼滤波）和IMU的配置文件路径
#     ekf_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'ekf.yaml')
#     ekf_carto_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'ekf_carto.yaml')
    imu_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'imu.yaml')

    
    # 声明Cartographer SLAM启动参数（默认关闭），用于控制是否启用Cartographer建图
    # carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='false')

    # 包含机器人基础TCP通信启动文件，禁用内置IMU（当前配置为TCP控制模式）
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_tcp.launch.py')),
            launch_arguments={'use_wheeltec_imu':'false'}.items(),
    )
    
    #TODO-imu需要重新集成 需调试 slam导致 odom_combined 连不上 必须处理
    
    # 启动EKF节点，根据carto_slam参数选择不同的EKF配置（普通模式/Cartographer模式）
    # robot_ekf = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ia_ekf.launch.py')),
    #         launch_arguments={'carto_slam':carto_slam}.items(),            
    # )

    
    # 静态TF变换：base_footprint -> base_link（机器人基坐标系到机器人本体坐标系的变换）
    # base_to_link = launch_ros.actions.Node(
    #         package='tf2_ros', 
    #         executable='static_transform_publisher', 
    #         name='base_to_link',
    #         arguments=['0', '0', '-0.5','0', '0','0','base_footprint','base_link'],
    # )
    
    # 静态TF变换：base_footprint -> gyro_link（机器人基坐标系到陀螺仪坐标系的变换）
    # base_to_gyro = launch_ros.actions.Node(
    #         package='tf2_ros', 
    #         executable='static_transform_publisher', 
    #         name='base_to_gyro',
    #     #     arguments=['-0.28724', '0.15', '0.1','0', '0','0','base_footprint','gyro_link'],
    #         arguments=['-0.28724', '0.15', '0.28','0', '0','0','base_footprint','gyro_link'],
    # )

    #TODO此处为雷达安装位置 后续视觉需要重新修改
    
    base_to_imu = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_imu',
            arguments=['0', '0', '0','0', '0','0','base_link','imu_link'],
    )



    # laser
    base_to_laser = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['-0.15', '0', '0','0', '3.1415','0','base_link','laser'],  # z:-0.35 以imu坐标为原点，imu的z轴
    )      

    # 使用slamtool建图时 需要指定odom_combined和map的关系
    # 静态TF变换：map -> odom_combined（地图坐标系到融合里程计坐标系的变换，用于SLAM建图时的坐标关联）
    # map_to_odom_combined = launch_ros.actions.Node(
    #         package='tf2_ros', 
    #         executable='static_transform_publisher', 
    #         name='base_to_odom_combined',
    #         arguments=['0', '0', '0','0', '0','0','map','odom_combined'],
    # )

    # IMU滤波节点：使用Madgwick算法对IMU数据进行滤波处理
#     imu_filter_node =  launch_ros.actions.Node(
#         package='imu_filter_madgwick',
#         executable='imu_filter_madgwick_node',
#         parameters=[imu_config]
#     )
    
              
    # 关节状态发布器节点：发布机器人关节状态信息（用于URDF模型可视化）
    joint_state_publisher_node = launch_ros.actions.Node(
            package='joint_state_publisher', 
            executable='joint_state_publisher', 
            name='joint_state_publisher',
    )


    # 包含小型机器人模型描述启动文件（加载mini_mec型号的URDF/Xacro模型）
    minibot_type = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description_minibot.launch.py')),
            launch_arguments={'mini_mec': 'true'}.items(),
    )

   
    

    # 启动描述对象：组装所有启动组件
    ld = LaunchDescription()

    # 添加机器人模型描述（启用小型机器人模型）
#     ld.add_action(minibot_type)
    # 添加Cartographer SLAM启动参数声明
    ld.add_action(carto_slam_dec)
    # 添加机器人基础TCP通信组件
    ld.add_action(wheeltec_robot)
    # 添加静态TF变换节点
#     ld.add_action(base_to_link)
#     ld.add_action(base_to_gyro)
    ld.add_action(base_to_laser)
#     ld.add_action(map_to_odom_combined)
    # 添加关节状态发布器
    ld.add_action(joint_state_publisher_node)

    ld.add_action(base_to_imu)
    # 添加IMU滤波节点
#     ld.add_action(imu_filter_node)    
    # 添加EKF节点
#     ld.add_action(robot_ekf)
    #TODO 雷达会打开rviz    
    
    # 返回最终组装的启动描述
    return ld
