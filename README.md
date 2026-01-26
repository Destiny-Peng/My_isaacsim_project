isaac_sim_fullstack

Project skeleton for Isaac Sim + ROS2 full-stack robot (mobile base + manipulator).

Structure:
- ros2_ws/src: ROS2 packages (navigation_interface, manipulation_interface, perception, tf_manager, robot_bringup, robot_description)
- sim: Isaac Sim scenes and launch scripts
- configs: Nav2 / MoveIt / sensor config templates
- docs: Design notes and architecture

Next steps:
- Implement URDF/xacro in `robot_description`
- Add MoveIt2 configuration for the arm in `manipulation_interface`
- Add Nav2 params and maps in `navigation_interface`
- Implement TF manager and perception pipelines
