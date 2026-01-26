cmake_minimum_required(VERSION 3.5)
project(tf_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(tf_manager_node src/tf_manager_node.cpp)
ament_target_dependencies(tf_manager_node rclcpp tf2_ros)

install(TARGETS
  tf_manager_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
