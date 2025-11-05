ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/point_cloud \
  -r scan:=/scan \
  -p angle_min:=-3.1415926 \
  -p angle_max:=3.1415926 \
  -p angle_increment:=0.0174533 \
  -p range_min:=0.1 \
  -p range_max:=10.0 \
  -p scan_time:=0.1 \
  -p target_frame:=lidar_frame





    ros2 launch rtabmap_launch rtabmap.launch.py  rgb_topic:=/rgb2 depth_topic:=/depth2 camera_info_topic:=/camera_info frame_id:=base_link odom_topic:=/odom approx_sync:=true