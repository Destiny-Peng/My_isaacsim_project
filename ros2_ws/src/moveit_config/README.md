MoveIt2 Integration Guide

Goal: Integrate MoveIt2 with the composite mobile base + Franka arm so MoveIt2 plans for the arm (keeping base fixed) and hands trajectories to the arm controller; the mobile base is controlled separately via Isaac Sim cmd_vel.

1) High-level architecture
- `robot_description` (URDF) kept as canonical robot model for MoveIt2 and `robot_state_publisher`.
- MoveIt2 `move_group`:
  - Uses the same URDF and an SRDF that declares a `manipulator` planning group (arm joints only).
  - Planning pipeline computes a `trajectory_msgs/JointTrajectory` for the arm.
  - Execution plugin / controllers: configure a controller that accepts `trajectory_msgs/JointTrajectory` on `/arm_controller/command`.
- `manipulation_interface`:
  - Provides a MoveIt client (optional) and an `arm_trajectory_bridge` node which subscribes to `JointTrajectory` messages and forwards them to the controller command topic.
- Isaac Sim handles the mobile base velocity commands (`cmd_vel`) and publishes `joint_states` for wheels/arm. MoveIt2 only plans for the arm joints.

2) Required MoveIt2 artifacts (generate with MoveIt Setup Assistant for ROS2):
- `robot.srdf` (define `manipulator` planning group referring to the arm links/joints)
- `config/controllers.yaml` (controller to receive trajectories)
- `config/ros_controllers.yaml` (ros2_control controller definitions if using `ros2_control`)
- `config/joints.yaml` (joint limits, optionally)

3) Controller interface
- For simplicity, configure MoveIt2 to use a controller that sends `trajectory_msgs/JointTrajectory` to `/arm_controller/command`.
- Implement `arm_trajectory_bridge` (see `manipulation_interface`) to accept such trajectories and publish to actuators or simulated controllers.

4) Steps to set up
- Run MoveIt2 Setup Assistant with `combined_car_franka.urdf`.
- Define the planning group containing `panda_panda_link1..7` joints.
- Generate the MoveIt2 package, copy it into `ros2_ws/src/moveit_config`.
- Build workspace and launch `move_group` with controller manager running.

5) Notes
- Because the mobile base is simulated/controlled in Isaac Sim, do not include base joints in MoveIt planning groups.
- Ensure `joint_state_publisher` receives the arm joint states from Isaac Sim or from a bridge.

