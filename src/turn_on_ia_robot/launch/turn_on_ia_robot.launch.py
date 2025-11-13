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
    ekf_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'ekf.yaml')
#     ekf_carto_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'ekf_carto.yaml')
    imu_config = Path(get_package_share_directory('turn_on_ia_robot'), 'config', 'imu.yaml')

    
    # 声明Cartographer SLAM启动参数（默认关闭），用于控制是否启用Cartographer建图
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='false')

    # 包含机器人基础TCP通信启动文件，禁用内置IMU（当前配置为TCP控制模式）
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_tcp.launch.py')),
            launch_arguments={'use_wheeltec_imu':'false'}.items(),
    )
    
    #TODO-imu需要重新集成 需调试 slam导致 odom_combined 连不上 必须处理
    
    # 启动EKF节点，根据carto_slam参数选择不同的EKF配置（普通模式/Cartographer模式）
    robot_ekf = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ia_ekf.launch.py')),
            launch_arguments={'carto_slam':carto_slam}.items(),            
    )

    


    #TODO此处为雷达安装位置 后续视觉需要重新修改
    
    base_to_imu = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_imu',
            arguments=['0', '0', '0','0', '0','0','base_footprint','imu_link'],
    )

    # laser
    base_to_laser = launch_ros.actions.Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            name='base_to_laser',
            arguments=['-0.05', '0', '0','-1.570796', '0','0','base_footprint','laser'],
    )      



    # IMU滤波节点：使用Madgwick算法对IMU数据进行滤波处理
    imu_filter_node =  launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config]
    )
    
              
    # 加载机器人模型
    # 加载ia_robot.urdf
    ia_robot_urdf = os.path.join(get_package_share_directory('ia_robot_urdf'), 'urdf', 'ia_robot_model.urdf')
    robot_state_publisher_node = launch_ros.actions.Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            name='robot_state_publisher',
            arguments=[ia_robot_urdf],
        # remap joint_states to /ia_robot/joint_states
        # remappings=[('joint_states', '/ia_robot/joint_states')]
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
#     ld.add_action(joint_state_publisher_node)
    # 添加机器人状态发布器节点
    ld.add_action(robot_state_publisher_node)
    ld.add_action(base_to_imu)
    # 添加IMU滤波节点
    # ld.add_action(imu_filter_node)    
    # 添加EKF节点
#     ld.add_action(robot_ekf)
    #TODO 雷达会打开rviz    
    
    # 返回最终组装的启动描述
    return ld
