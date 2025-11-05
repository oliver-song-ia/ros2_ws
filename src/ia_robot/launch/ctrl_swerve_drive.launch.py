# https://github.com/hijimasa/isaac-ros2-control-sample

import os
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    # isaac_diffbot_description_path = os.path.join(
    #     get_package_share_directory('ia_robot_sim'))

    # xacro_file = os.path.join(isaac_diffbot_description_path,
    #                           'description',
    #                           'robot.urdf.xacro')
    # # urdf_path = os.path.join(isaac_diffbot_description_path, 'description', 'robot.urdf')

    # doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'false'})
    # robot_desc = doc.toprettyxml(indent='  ')
    # f = open(urdf_path, 'w')
    # f.write(robot_desc)
    # f.close()
    # relative_urdf_path = pathlib.Path(urdf_path).relative_to(os.getcwd())

    # Launch Arguments
    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # namespace = LaunchConfiguration('namespace', default="robot")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ia_robot'),
                 'urdf', 'ia_robot.absolute.urdf']
            ),
        ]
    )

    params = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ia_robot"),
            "config",
            "swerve_drive_controller.yaml",
        ]
    )

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params],
    # )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output="both",
    )


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_drive_controller", "--controller-manager", "/controller_manager"],
    )

    

    nodes = [
        # node_robot_state_publisher,
        control_node,
        robot_controller_spawner
    ]

    return LaunchDescription(nodes)