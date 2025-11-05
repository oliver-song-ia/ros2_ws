
启动底盘
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

查看tf树
ros2 run rqt_tf_tree rqt_tf_tree --force-discover

话题列表
ros2 topic list

/diagnostics
/imu/data
/imu/data_raw
/joint_states
/odom
/odom_combined
/parameter_events
/robot_description
/rosout
/set_pose
/tf
/tf_static

启动激光雷达
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py 

ros2 node list 

节点列表
/base_to_camera_link
/base_to_laser
/c16/lslidar_driver_node
/c16/rviz2
/c16/transform_listener_impl_aaaad1a19f60
/launch_ros_131742
/pointcloud_to_laserscan
/transform_listener_impl_aaaaf366fa20

ros2 topic list 

话题列表
/c16/lslidar_driver_node/transition_event
/clicked_point
/goal_pose
/initialpose
/parameter_events
/point_cloud_raw : 10HZ
/rosout
/scan :10HZ
/scan_raw : 10HZ
/tf
/tf_static

ros2 node info /c16/lslidar_driver_node
/c16/lslidar_driver_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /point_cloud_raw: sensor_msgs/msg/PointCloud2
    /rosout: rcl_interfaces/msg/Log
    /scan_raw: sensor_msgs/msg/LaserScan
  Service Servers:
    /c16/lslidar_driver_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /c16/lslidar_driver_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /c16/lslidar_driver_node/get_parameters: rcl_interfaces/srv/GetParameters
    /c16/lslidar_driver_node/list_parameters: rcl_interfaces/srv/ListParameters
    /c16/lslidar_driver_node/set_parameters: rcl_interfaces/srv/SetParameters
    /c16/lslidar_driver_node/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

ros2 node info /pointcloud_to_laserscan 
/pointcloud_to_laserscan
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /scan: sensor_msgs/msg/LaserScan
  Service Servers:
    /pointcloud_to_laserscan/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /pointcloud_to_laserscan/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /pointcloud_to_laserscan/get_parameters: rcl_interfaces/srv/GetParameters
    /pointcloud_to_laserscan/list_parameters: rcl_interfaces/srv/ListParameters
    /pointcloud_to_laserscan/set_parameters: rcl_interfaces/srv/SetParameters
    /pointcloud_to_laserscan/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:

说明：/scan是点云转激光雷达的数据；/scan_raw是激光雷达原始数据；/point_cloud_raw是原始的激光雷达数据

问题：发现在rviz2中显示不了激光雷达的扫描数据/scan,点云数据显示正常，/scan_raw显示出来数据当时和点云数据相差很大。



 ros2 launch wheeltec_slam_toolbox online_async_launch.py




ros2 topic info /scan --verbose
Type: sensor_msgs/msg/LaserScan

Publisher count: 1

Node name: pointcloud_to_laserscan
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: PUBLISHER
GID: 01.0f.94.c3.27.be.78.58.00.00.00.00.00.00.12.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 2

Node name: rviz2
Node namespace: /c16
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: SUBSCRIPTION
GID: 01.0f.94.c3.1d.be.f0.01.00.00.00.00.00.00.24.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: slam_toolbox
Node namespace: /
Topic type: sensor_msgs/msg/LaserScan
Endpoint type: SUBSCRIPTION
GID: 01.0f.94.c3.2c.be.d2.6f.00.00.01.00.00.00.22.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

slam_toolbox建图算法测试
可能原因：
1. 里程计数据位置姿态没有进行积分运算，导致建图时位置姿态错误。
2. 