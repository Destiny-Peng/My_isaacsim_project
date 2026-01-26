cmake_minimum_required(VERSION 3.5)
project(manipulation_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(manipulation_interface_node src/manipulation_interface_node.cpp)
ament_target_dependencies(manipulation_interface_node rclcpp std_msgs)

install(TARGETS
  manipulation_interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
