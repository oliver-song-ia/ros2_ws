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
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    swerve_ctrl_dir = get_package_share_directory('ia_robot_sim')
    ia_robot_dir = get_package_share_directory('ia_robot')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    swerve_ctrl_launch_dir = os.path.join(swerve_ctrl_dir, 'launch')
    launch_dir = os.path.join(bringup_dir, 'launch')
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    lidar_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )
    # 添加 fdilink_ahrs 驱动的 launch 文件
    ahrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('fdilink_ahrs'),  # 获取 fdilink_ahrs 包的路径
            '/launch/ahrs_driver.launch.py'  # ahrs_driver.launch.py 相对于包的路径
        ])
    )

# # ros2 launch ia_robot_sim swerve_ctrl.launch.py
    swerve_ctrl = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(swerve_ctrl_launch_dir, 'swerve_ctrl.launch.py')),
    )

    # Include safety system (ultrasonic emergency stop + twist_mux)
    safety_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ia_robot_dir, 'launch', 'safety_system.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'safety_threshold': '0.15',
            'hysteresis_threshold': '0.2',
            'enable_emergency_stop': 'True',
        }.items(),
        condition=IfCondition('True')
    )
    
    # Twist Mux node - Always runs to manage velocity command priorities
    # This ensures clean separation between navigation commands (/cmd_vel_nav)
    # and the final robot commands (/cmd_vel)
    twist_mux_config = os.path.join(ia_robot_dir, 'config', 'twist_mux.yaml')

    twist_mux_node = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # twist_mux will handle topic mapping based on config file
            # Main output is /cmd_vel to the robot
            ('cmd_vel_out', 'cmd_vel'),
        ]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',  # 将rviz2输出打印到终端
        # 如果需要加载特定配置文件，可以添加以下参数
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'laser_2d.rviz')]
    )

    # Joint States Remapper - 将 /joint_states 转换为 /robot/joint_states
    # 映射关系:
    # Leg_front_left_2  -> lf_steer_joint  (转向)
    # Leg_front_right_2 -> rf_steer_joint  (转向)
    # Leg_back_left_2   -> lb_steer_joint  (转向)
    # Leg_back_right_2  -> rb_steer_joint  (转向)
    # Leg_front_left_1  -> lf_wheel_joint  (轮速)
    # Leg_front_right_1 -> rf_wheel_joint  (轮速)
    # Leg_back_left_1   -> lb_wheel_joint  (轮速)
    # Leg_back_right_1  -> rb_wheel_joint  (轮速)
    joint_states_remapper_node = launch_ros.actions.Node(
        package='ia_robot_sim',
        executable='joint_states_remapper.py',
        name='joint_states_remapper',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        wheeltec_robot,
        lidar_ros,
        ahrs_launch,
        joint_states_remapper_node,  # 添加 joint states 转换节点
        swerve_ctrl,
        safety_system_launch,
        # twist_mux_node,
        # rviz_node
        ]
    )


