# self_filter.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_name_arg = DeclareLaunchArgument(
        'description_name',
        default_value='/robot_description'
    )
    zero_for_removed_points_arg = DeclareLaunchArgument(
        'zero_for_removed_points',
        default_value='true'
    )
    lidar_sensor_type_arg = DeclareLaunchArgument(
        'lidar_sensor_type',
        default_value='0'
    )
    in_pointcloud_topic_arg = DeclareLaunchArgument(
        'in_pointcloud_topic',
        default_value='/lidar'
    )
    out_pointcloud_topic_arg = DeclareLaunchArgument(
        'out_pointcloud_topic',
        default_value='/lidar_filter'
    )
    # Use proper ROS 2 substitutions instead of deprecated $(find ...) which is not expanded here
    # If the URDF is a xacro, you can uncomment the Command(...) form below.
    # robot_description_default = PathJoinSubstitution([
    #     FindPackageShare('ia_robot_urdf'), 'urdf', 'ia_robot_ik.urdf'
    # ])
    robot_description_default = Command(['xacro ', PathJoinSubstitution([FindPackageShare('ia_robot_urdf'), 'urdf', 'ia_robot_ik.urdf'])])
    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_default,
        description='Path to the robot URDF (or xacro if Command substitution is used)'
    )

    filter_config_default = PathJoinSubstitution([
        FindPackageShare('robot_self_filter'), 'config', 'robot_self_filter.yaml'
    ])
    filter_config_arg = DeclareLaunchArgument(
        'filter_config',
        default_value=filter_config_default,
        description='Path to self filter parameter YAML file'
    )
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', # Keep default as true for standalone use
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Declare use_custom_pointcloud_msg argument
    use_custom_pointcloud_msg_arg = DeclareLaunchArgument(
        'use_custom_pointcloud_msg',
        default_value='false',
        description='Use gs_ros_interfaces::PointCloud2WithIndex instead of sensor_msgs::PointCloud2'
    )

    # Ground filtering arguments
    enable_ground_filtering_arg = DeclareLaunchArgument(
        'enable_ground_filtering',
        default_value='false',
        description='Enable ground point filtering based on z-coordinate'
    )
    
    ground_z_threshold_arg = DeclareLaunchArgument(
        'ground_z_threshold',
        default_value='0.03',
        description='Points below this z value are considered ground and filtered out'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='lidar_link',
        description='Frame to transform points to'
    )
    
    default_sphere_padding_arg = DeclareLaunchArgument(
        'default_sphere_padding',
        default_value='0.02',
        description='Default padding for sphere collision shapes'
    )

    # Create a log action to print the config
    log_config = LogInfo(msg=LaunchConfiguration('filter_config'))

    self_filter_node = Node(
        package='robot_self_filter',
        executable='self_filter',
        name='self_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('filter_config'),  # loads the YAML file
            {
                'lidar_sensor_type': LaunchConfiguration('lidar_sensor_type'),
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'),
                    value_type=str
                ),
                'zero_for_removed_points': LaunchConfiguration('zero_for_removed_points'),
                'use_sim_time': LaunchConfiguration('use_sim_time'), # Use the launch argument
                'use_custom_pointcloud_msg': LaunchConfiguration('use_custom_pointcloud_msg'),
                'in_pointcloud_topic': LaunchConfiguration('in_pointcloud_topic'),
                'enable_ground_filtering': LaunchConfiguration('enable_ground_filtering'),
                'ground_z_threshold': LaunchConfiguration('ground_z_threshold'),
                'target_frame': LaunchConfiguration('target_frame'),
                'default_sphere_padding': LaunchConfiguration('default_sphere_padding')
            }
        ],
        remappings=[
            ('/robot_description', LaunchConfiguration('description_name')),
            # ('/cloud_in', LaunchConfiguration('in_pointcloud_topic')),
            ('/cloud_out', LaunchConfiguration('out_pointcloud_topic')),
            ('/cloud_debug', '/points_dep0_filter_debug') # Remap debug topic for visualization
        ],
        # arguments=[
        #     '--ros-args',
        #     '--log-level', 'self_filter:=debug',
        #     '--log-level', 'rcl:=info',
        #     '--log-level', 'rclcpp:=info',
        #     '--log-level', 'rmw_fastrtps_cpp:=info'
        # ]
    )

    return LaunchDescription([
        description_name_arg,
        zero_for_removed_points_arg,
        lidar_sensor_type_arg,
        in_pointcloud_topic_arg,
        out_pointcloud_topic_arg,
        robot_description_arg,
        filter_config_arg,
        use_sim_time_arg, # Add to launch description
        use_custom_pointcloud_msg_arg, # Add custom pointcloud message argument
        enable_ground_filtering_arg, # Add ground filtering arguments
        ground_z_threshold_arg,
        target_frame_arg,
        default_sphere_padding_arg,
        log_config,
        self_filter_node
    ])
