from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for Yolo registration node.
    """
    # Declare launch arguments
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes",
    )
    
    # Node declaration with namespace using LaunchConfiguration
    yolo_register_node = Node(
        package="yolo_ros",
        executable="yolo_register_node",
        name="yolo_register_node",
        namespace=LaunchConfiguration("namespace"),  # Use LaunchConfiguration here
        output="screen",
    )

    # Return LaunchDescription with the declared argument and node
    return LaunchDescription([
        namespace_cmd,  # Add the declaration of the argument
        yolo_register_node,  # Add the node to be launched
    ])