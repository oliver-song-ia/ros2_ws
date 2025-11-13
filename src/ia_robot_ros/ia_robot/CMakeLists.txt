cmake_minimum_required(VERSION 3.5)

project(ia_robot)

find_package(ament_cmake REQUIRED)

# Install launch files and config
install(
  DIRECTORY config launch maps behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install all Python scripts from src directory
# CONFIGURE_DEPENDS ensures CMake checks for file changes during build
# GLOB_RECURSE searches recursively through subdirectories
file(GLOB_RECURSE PYTHON_SCRIPTS CONFIGURE_DEPENDS "src/**/*.py" "src/*.py")
install(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()