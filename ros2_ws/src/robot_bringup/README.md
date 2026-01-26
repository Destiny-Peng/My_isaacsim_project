Quick bringup for ROS2 <-> Isaac Sim integration

1) Build workspace

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

2) Start Isaac Sim with ROS2 bridge enabled (in Isaac Sim UI or via your usual startup script).

3) Launch bringup (this starts the `robot_description` launch which runs `robot_state_publisher`):

```bash
ros2 launch robot_bringup bringup.launch.py
```

4) Verify topics and TFs:

```bash
ros2 topic list | grep joint_states
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames  # requires graphviz
```

If you need the launch to publish an explicit static transform between `map` and `World` or to start extra nodes, I can add them to this bringup launch.