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
    ia_robot_dir = get_package_share_directory('ia_robot')
    ia_robot_urdf_dir = get_package_share_directory('ia_robot_urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    swerve_ctrl_launch_dir = os.path.join(ia_robot_dir, 'launch')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Robot URDF file path
    # Use ia_robot.urdf for real robot (no virtual IK joints that conflict with odom)
    # ia_robot_ik.urdf is only for MoveIt simulation with mobile manipulation
    urdf_file = os.path.join(ia_robot_urdf_dir, 'urdf', 'ia_robot_model.urdf')
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

    swerve_ctrl = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(swerve_ctrl_launch_dir, 'ctrl_swerve_drive.launch.py')),
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

    # Robot State Publisher - publishes the robot model from URDF
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': use_sim_time
        }]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',  # 将rviz2输出打印到终端
        # 如果需要加载特定配置文件，可以添加以下参数
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'laser_2d.rviz')]
    )

    return LaunchDescription([
        wheeltec_robot,
        lidar_ros,
        ahrs_launch,
        swerve_ctrl,
        robot_state_publisher_node,  # Publish robot model from URDF
        # safety_system_launch, # TODO: modify nav2 output topic to make safety system working
        # twist_mux_node,
        # rviz_node
        ]
    )


