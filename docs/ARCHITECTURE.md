Architecture overview

- Simulator: NVIDIA Isaac Sim with ROS2 bridge
- ROS2 packages under `ros2_ws/src`:
  - `navigation_interface`: Nav2 coordination and mission-level goals
  - `manipulation_interface`: MoveIt2 interaction & grasp action server
  - `perception`: image + pointcloud processing (OpenCV/PCL)
  - `tf_manager`: canonical TF frames management and utilities
  - `robot_bringup`: launch files to start sim + bridges
  - `robot_description`: URDF/XACRO for robot model

TF frames to maintain: map -> odom -> base_link -> arm_base -> camera_link

Next dev tasks:
- Add URDF and robot_state_publisher
- Implement TF static transforms and odometry relay
- Design Action interface connecting Nav2 arrival -> manipulation grab
