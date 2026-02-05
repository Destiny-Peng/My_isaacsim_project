cmake_minimum_required(VERSION 3.5)
project(manipulation_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_package()

set(CMAKE_CXX_STANDARD 17)

add_executable(manipulation_interface_node src/manipulation_interface_node.cpp)
ament_target_dependencies(manipulation_interface_node rclcpp std_msgs)

install(TARGETS
  manipulation_interface_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python bridge script and launch files so `ros2 launch manipulation_interface ...` works
# install(PROGRAMS
#   src/arm_trajectory_bridge.py
#   DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
