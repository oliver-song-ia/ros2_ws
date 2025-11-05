

import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


#def launch(launch_descriptor, argv):
def generate_launch_description():
    # 声明启动参数: 是否使用wheeltec内置IMU
    # 默认值为'false'，通过外部传入参数可控制IMU启用状态
    use_wheeltec_imu_declare = DeclareLaunchArgument(
        'use_wheeltec_imu',
        default_value='false',  
        description='If true, use wheeltec_imu'
    )
    # 将声明的启动参数转换为可在条件判断中使用的LaunchConfiguration对象
    declare_use_imu = LaunchConfiguration('use_wheeltec_imu')

    

    # 提取通用参数配置，这些参数将传递给机器人节点
    # 包含串口配置、坐标系名称、速度指令话题、里程计缩放系数等
    common_params = {
        'usart_port_name': '/dev/wheeltec_controller',  # 串口设备名称（TCP模式下可能未实际使用）
        'serial_baud_rate': 115200,                     # 串口波特率（TCP模式下可能未实际使用）
        'robot_frame_id': 'base_footprint',             # 机器人基坐标系ID
        'odom_frame_id': 'odom_combined',               # 融合里程计坐标系ID
        'cmd_vel': 'cmd_vel',                           # 速度控制指令话题名称
        'akm_cmd_vel': 'none',                          # AKM底盘速度指令话题（未使用）
        'product_number': 0,                            # 产品型号编号
        'odom_x_scale': 1.0,                            # X方向里程计缩放系数
        'odom_y_scale': 1.0,                            # Y方向里程计缩放系数
        'odom_z_scale_positive': 1.0,                   # Z轴正方向角速度缩放系数
        'odom_z_scale_negative': 1.0                    # Z轴负方向角速度缩放系数
    }
    
    # 话题重映射配置: 将IMU原始数据话题重映射为imu/data_board
    remappings=[('imu/data_raw', 'imu/data_board')]

    # 定义不使用wheeltec IMU时启动的节点 (条件: use_wheeltec_imu为false)
    # 包含调试前缀(xterm + gdb)，用于运行时调试
    turn_on_robot_use_imu = Node(
        condition=UnlessCondition(declare_use_imu),  # 条件判断: 当不使用IMU时执行
        #prefix=['xterm -e gdb -ex run --args'],      # 调试前缀: 启动xterm终端并通过gdb调试
        package='turn_on_wheeltec_robot',            # 节点所属功能包
        executable='wheeltec_robot_node1',           # 节点可执行文件名称
        output='screen',                              # 日志输出到屏幕
        parameters=[common_params],                   # 传入通用参数配置

    )
    
    # 定义使用wheeltec IMU时启动的节点组 (条件: use_wheeltec_imu为true)
    # 包含机器人主节点和IMU驱动启动文件
    turn_on_robot = GroupAction(
        condition=IfCondition(declare_use_imu),       # 条件判断: 当使用IMU时执行
        actions=[
            # 机器人主节点配置，与不使用IMU时相比增加了话题重映射
            Node(
                package='turn_on_wheeltec_robot', 
                executable='wheeltec_robot_node1', 
                output='screen',
                parameters=[common_params],
                remappings=remappings,),  # 应用IMU话题重映射
            
            # 包含yesense IMU驱动的启动文件
            # 用于加载外部IMU传感器的驱动节点
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('yesense_std_ros2'), 
                                'launch', 'yesense_auto.launch.py'))),    
        ]
    )




    # 创建启动描述对象，用于组装所有启动组件
    ld = LaunchDescription()
    # 添加启动参数声明到启动描述
    ld.add_action(use_wheeltec_imu_declare)  
    # 添加"不使用IMU"时的机器人节点到启动描述
    ld.add_action(turn_on_robot_use_imu)
    # 添加"使用IMU"时的节点组到启动描述
    ld.add_action(turn_on_robot)

    # 返回组装完成的启动描述
    return ld

