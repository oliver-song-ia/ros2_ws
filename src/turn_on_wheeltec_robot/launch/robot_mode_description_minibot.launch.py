import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_robot_node(robot_urdf,child):
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{child}',
        arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'), 'urdf', robot_urdf)],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )
    
def generate_launch_description():

    mini_mec = LaunchConfiguration('mini_mec', default='false')
    #TODO增加ia      
    ia_robot = GroupAction(
        condition=IfCondition(mini_mec),
        actions=[
        generate_robot_node('ia_robot.urdf','ia_robot'),
        # generate_static_transform_publisher_node(['-0.09', '0.0000', '0.0'], ['0', '0', '0'], 'base_footprint', 'laser'),
        # generate_static_transform_publisher_node(['0.10709', '0.00032', '0.0762'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 
    

    
    # Create the launch description and populate
    ld = LaunchDescription()


      
    ld.add_action(ia_robot)



    return ld
