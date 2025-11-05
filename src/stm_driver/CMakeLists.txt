cmake_minimum_required(VERSION 3.8)
project(stm_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stm_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(node src/node.cpp ${IA_SRC_FILES})

include_directories(include)

#定义头文件
set(IA_HEAD_FILES 
  include/stm_driver/dataparse.hpp
  include/stm_driver/tcp.hpp
  include/stm_driver/stm_driver.hpp
  include/stm_driver/udp.hpp
)

#定义资源文件
set(IA_SRC_FILES 
  src/dataparse.cpp 
  src/tcp.cpp
  src/udp.cpp
)

add_library(${PROJECT_NAME} src/stm_driver.cpp ${IA_SRC_FILES} ${IA_HEAD_FILES})
#target_include_directories(${PROJECT_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

## Install executable
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_14)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  stm_msg
  geometry_msgs
)  
ament_export_include_directories(include)
#导出当前的
ament_export_libraries(${PROJECT_NAME})

#必须增导出的文件
install(
        DIRECTORY include/stm_driver
        DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
