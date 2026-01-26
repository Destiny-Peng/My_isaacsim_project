cmake_minimum_required(VERSION 3.5)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp sensor_msgs)

install(TARGETS
  perception_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
