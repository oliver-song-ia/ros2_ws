cmake_minimum_required(VERSION 3.8)
project(ia_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Cartographer初始姿态设置节点
add_executable(carto_set_initial_pose_node src/carto_set_init_pose_node.cpp)
target_link_libraries(carto_set_initial_pose_node
  ${cartographer_ros_msgs_LIBRARIES}
  rclcpp::rclcpp
  tf2::tf2
  # geometry_msgs
)
ament_target_dependencies(carto_set_initial_pose_node
  rclcpp
  cartographer_ros_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  carto_set_initial_pose_node
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# install(
#   DIRECTORY config description launch scripts maps behavior_trees rviz
#   DESTINATION share/${PROJECT_NAME}
# )
install(
  DIRECTORY config launch rviz maps params scripts
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
