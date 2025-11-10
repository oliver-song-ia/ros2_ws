# dual_lidar_self_filter.launch.py
# Subscribes to two pointcloud topics, filters each, and combines them into one output
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
    
    # Input topics for the two lidars
    in_pointcloud_topic_0_arg = DeclareLaunchArgument(
        'in_pointcloud_topic_0',
        default_value='/points_dep0'
    )
    in_pointcloud_topic_1_arg = DeclareLaunchArgument(
        'in_pointcloud_topic_1',
        default_value='/points_dep1'
    )
    
    # Combined output topic
    out_pointcloud_topic_arg = DeclareLaunchArgument(
        'out_pointcloud_topic',
        default_value='/lidar_filter_combined'
    )
    
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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
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
        default_value='odom',
        description='Frame to transform points to'
    )

    # Create a log action to print the config
    log_config = LogInfo(msg=LaunchConfiguration('filter_config'))

    # First self filter node for points_dep0
    self_filter_node_0 = Node(
        package='robot_self_filter',
        executable='self_filter',
        name='self_filter_0',
        output='screen',
        parameters=[
            LaunchConfiguration('filter_config'),
            {
                'lidar_sensor_type': LaunchConfiguration('lidar_sensor_type'),
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'),
                    value_type=str
                ),
                'zero_for_removed_points': LaunchConfiguration('zero_for_removed_points'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_custom_pointcloud_msg': LaunchConfiguration('use_custom_pointcloud_msg'),
                'in_pointcloud_topic': LaunchConfiguration('in_pointcloud_topic_0'),
                'enable_ground_filtering': LaunchConfiguration('enable_ground_filtering'),
                'ground_z_threshold': LaunchConfiguration('ground_z_threshold'),
                'target_frame': LaunchConfiguration('target_frame')
            }
        ],
        remappings=[
            ('/robot_description', LaunchConfiguration('description_name')),
            ('/cloud_out', '/points_dep0_filtered'),
            ('/cloud_debug', '/points_dep0_filter_debug')
        ]
    )

    # Second self filter node for points_dep1
    self_filter_node_1 = Node(
        package='robot_self_filter',
        executable='self_filter',
        name='self_filter_1',
        output='screen',
        parameters=[
            LaunchConfiguration('filter_config'),
            {
                'lidar_sensor_type': LaunchConfiguration('lidar_sensor_type'),
                'robot_description': ParameterValue(
                    LaunchConfiguration('robot_description'),
                    value_type=str
                ),
                'zero_for_removed_points': LaunchConfiguration('zero_for_removed_points'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_custom_pointcloud_msg': LaunchConfiguration('use_custom_pointcloud_msg'),
                'in_pointcloud_topic': LaunchConfiguration('in_pointcloud_topic_1'),
                'enable_ground_filtering': LaunchConfiguration('enable_ground_filtering'),
                'ground_z_threshold': LaunchConfiguration('ground_z_threshold'),
                'target_frame': LaunchConfiguration('target_frame')
            }
        ],
        remappings=[
            ('/robot_description', LaunchConfiguration('description_name')),
            ('/cloud_out', '/points_dep1_filtered'),
            ('/cloud_debug', '/points_dep1_filter_debug')
        ]
    )

    # Pointcloud merge node to combine both filtered clouds
    merge_node = Node(
        package='robot_self_filter',
        executable='pointcloud_merger.py',
        name='pointcloud_merger',
        output='screen',
        parameters=[
            {
                'input_topic_0': '/points_dep0_filtered',
                'input_topic_1': '/points_dep1_filtered',
                'output_topic': LaunchConfiguration('out_pointcloud_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_approximate_sync': True,
                'queue_size': 10,
                'slop': 0.1  # 100ms tolerance for syncing
            }
        ]
    )

    return LaunchDescription([
        description_name_arg,
        zero_for_removed_points_arg,
        lidar_sensor_type_arg,
        in_pointcloud_topic_0_arg,
        in_pointcloud_topic_1_arg,
        out_pointcloud_topic_arg,
        robot_description_arg,
        filter_config_arg,
        use_sim_time_arg,
        use_custom_pointcloud_msg_arg,
        enable_ground_filtering_arg,
        ground_z_threshold_arg,
        target_frame_arg,
        log_config,
        self_filter_node_0,
        self_filter_node_1,
        merge_node
    ])
