Quick test: MoveIt -> arm_trajectory_bridge -> controller

1) Build the workspace and source:

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

2) Start robot bringup (robot_state_publisher etc.) and ensure Isaac Sim is publishing `/joint_states`.

3) Start MoveIt `move_group` (use your existing demo or the example in `moveit_resources/panda_moveit_config`).

4) Start the bridge (this will run the Python bridge script):

```bash
ros2 launch manipulation_interface bridge.launch.py input_topic:=/arm_trajectory output_topic:=/arm_controller/command
```

5) From MoveIt (RViz MotionPlanning panel) plan and Execute for the `manipulator` group. MoveIt will publish the planned `JointTrajectory` to whichever execution topic is configured — configure it to publish to `/arm_trajectory` (or remap in the MoveIt launch).

6) Alternatively test manually by publishing a simple `trajectory_msgs/JointTrajectory` to `/arm_trajectory` and observe it forwarded to `/arm_controller/command`:

```bash
ros2 topic pub /arm_trajectory trajectory_msgs/JointTrajectory "{ joint_names: ['panda_joint1','panda_joint2'], points: [{positions: [0.0, 0.0], time_from_start: {sec: 1}}] }"
```

7) Observe controller input topic or simulator subscriptions to confirm receipt.

Notes:
- For a more integrated setup replace the bridge with a proper `ros2_control` controller and configure `moveit_controllers.yaml` accordingly.
- If MoveIt is using an action-based controller (FollowJointTrajectory), ensure the bridge or controller manager exposes the corresponding action interface or use MoveIt's controller manager adapters.
