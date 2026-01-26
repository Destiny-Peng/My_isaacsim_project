robot_description
=================

This package provides a minimal composite robot URDF/xacro to represent a mobile base + 6-DOF manipulator for development with Isaac Sim and ROS2.

Usage
-----
- Launch the robot description to publish `robot_state_publisher` and a `joint_state_publisher`:

```bash
source /opt/ros/humble/setup.bash
cd ~/project/isaac_test/isaac_sim_fullstack/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_description robot_description_launch.py
```

Integration with Isaac Sim
-------------------------
We recommend spawning an existing Isaac Sim robot USD (mobile base + arm) inside Isaac Sim, and then aligning the frames used in this URDF to the USD model by:

- Ensuring `base_link` and `arm_base` frames coincide with the USD robot's base and arm root frames (use static transforms in ROS2 if needed).
- Using the ROS-ROS2 bridge in Isaac Sim to forward camera topics; the `camera_link` frame here should be rigidly attached to the corresponding camera sensor in the USD.

This package intentionally uses a minimal manipulator description so you can swap-in a full URDF exported from the USD asset or vendor model later.
