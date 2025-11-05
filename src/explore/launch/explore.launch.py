import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成ROS2启动描述，用于配置并启动explore节点"""
    # 创建LaunchDescription对象，用于管理启动过程中的所有动作和参数
    ld = LaunchDescription()
    
    # 构建配置文件路径：获取explore_lite包的共享目录下的params.yaml配置文件
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )
    
    # 定义启动配置变量，用于接收外部传入的参数（如使用仿真时间、命名空间）
    use_sim_time = LaunchConfiguration("use_sim_time")  # 仿真时间标志位
    namespace = LaunchConfiguration("namespace")        # 节点命名空间

    # 声明"use_sim_time"启动参数：控制是否使用仿真时间，默认值为false
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation/Gazebo clock"
    )
    
    # 声明"namespace"启动参数：为explore节点指定命名空间，默认为空
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # 创建explore节点配置：指定节点所属包、可执行文件、命名空间、参数等
    node = Node(
        package="explore_lite",          # 节点所属的ROS包
        name="explore_node",             # 节点名称
        namespace=namespace,             # 节点命名空间（由启动参数指定）
        executable="explore",            # 节点可执行文件名
        parameters=[config, {"use_sim_time": use_sim_time}],  # 节点参数：配置文件+仿真时间标志
        output="screen",                 # 节点输出重定向到终端
        remappings=remappings,           # 话题重映射（处理tf坐标变换的命名空间问题）
    )
    
    # 将启动参数声明和节点配置添加到LaunchDescription中
    ld.add_action(declare_use_sim_time_argument)  # 添加仿真时间参数声明
    ld.add_action(declare_namespace_argument)     # 添加命名空间参数声明
    ld.add_action(node)                           # 添加explore节点
    
    return ld  # 返回构建完成的启动描述
