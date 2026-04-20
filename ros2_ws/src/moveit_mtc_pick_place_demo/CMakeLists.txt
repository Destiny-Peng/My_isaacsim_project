cmake_minimum_required(VERSION 3.22)
project(moveit_mtc_pick_place_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(mtc_pick_place_demo src/mtc_pick_place_demo.cpp)
ament_target_dependencies(
  mtc_pick_place_demo
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  sensor_msgs
  tf2_geometry_msgs
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_msgs
)

target_compile_features(mtc_pick_place_demo PUBLIC cxx_std_17)

install(TARGETS mtc_pick_place_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
