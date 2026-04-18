# Isaac Sim Launch (combined_car_franka)

This directory contains the no-GUI/GUI Python launcher:

- run_combined_car_franka_headless.py
- run_combined_car_franka_headless_guide.md (学习向讲解文档)

如果你希望先理解脚本结构和每个参数的作用，再上手调试，优先看：

- [run_combined_car_franka_headless_guide.md](run_combined_car_franka_headless_guide.md)

The script can:

- load combined_car_franka USD,
- run Isaac Sim in headless or GUI mode,
- enable a pure Python ROS2 bridge for:
	- /joint_commands (subscribe)
	- /joint_states (publish)
	- /clock (publish)
- optionally create/update demo cube in Isaac Sim for MTC collision alignment.

## 1) Start Simulation

Run from the workspace root or use absolute paths.

### Headless (default)

```bash
/home/jacy/anaconda3/envs/env_issaclab/bin/python \
/home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
--steps 20000 \
--render-interval 5000 \
--setup-ros2-python-bridge
```

### GUI mode

```bash
/home/jacy/anaconda3/envs/env_issaclab/bin/python \
/home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
--gui \
--launcher-backend isaacsim \
--steps 20000 \
--render-interval 5000 \
--clock-mode wall-anchored \
--setup-ros2-python-bridge
```

Notes:

- Without --setup-ros2-python-bridge, the pure Python ROS topics are not created.
- --headless and --gui are mutually exclusive.
- For GUI + Python ROS2 bridge, use `--launcher-backend isaacsim`.

## 2) ROS2 Discovery Setup (another terminal)

Use the same domain and middleware as your running environment.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 daemon stop || true
ros2 daemon start
sleep 2
```

## 3) Verify Topics Exist

```bash
ros2 topic list
```

Expected to include:

- /joint_commands
- /joint_states
- /clock

## 4) Verify Data Is Actually Flowing

### Check one clock message

```bash
ros2 topic echo /clock --once
```

### Check one joint state message

```bash
ros2 topic echo /joint_states --once
```

You should see 13 joints including wheel and panda joints.

## 5) Verify Command Response (Closed Loop)

Send one command and check if joint state reflects it.

```bash
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: ['panda_joint1'], position: [0.25]}"
sleep 1
ros2 topic echo /joint_states --once
```

Expected:

- panda_joint1 position in /joint_states is close to 0.25.

## 6) Useful Debug Flags

- --check-rclpy: validate rclpy import in current runtime.
- --inspect-graph: list graph-like and ros-like prims in loaded stage.
- --ros2-env-mode internal|system|none: control ROS2 env injection strategy.
- --launcher-backend auto|isaaclab|isaacsim: choose app launcher backend.
- --clock-mode wall-anchored|sim: timestamp mode for /clock and /joint_states.
- --sync-mtc-demo-object: create/update `/World/demo_cube` in simulation.
- --demo-object-size: cube size xyz, default `0.04,0.04,0.12`.
- --demo-object-pick-position: cube center xyz, default `0.55,0.0,0.20`.
- --demo-object-color: cube rgb, default `0.9,0.3,0.1`.

## 7) Troubleshooting (GUI + bridge)

If GUI can open USD without bridge, but fails when bridge is enabled:

1. Prefer Python bridge mode first:

```bash
--setup-ros2-python-bridge --launcher-backend isaacsim
```

2. Do not enable OmniGraph bridge unless needed:

```bash
# Avoid adding --setup-ros2-joint-bridge while validating Python bridge
```

3. Verify rclpy import in runtime:

```bash
--check-rclpy
```

Expected log includes `rclpy import: OK` before bridge starts.

If MoveIt prints `Detected jump back in time`, prefer:

```bash
--clock-mode wall-anchored
```

and start simulation before launching MoveIt.

## 8) MoveIt2 integration (recommended sequence)

When testing with `isaac_sim_moveit.launch.py`, avoid running multiple launch instances at the same time.

1. Check existing ROS nodes first:

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

If old MoveIt/controller nodes still exist, stop previous launch terminals first.

2. Start simulation bridge:

```bash
/home/jacy/anaconda3/envs/env_issaclab/bin/python \
/home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
--headless \
--launcher-backend isaacsim \
--setup-ros2-python-bridge \
--sync-mtc-demo-object \
--clock-mode wall-anchored \
--steps 20000 \
--render-interval 2000
```

3. Start MoveIt launch in another terminal:

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_robot_config isaac_sim_moveit.launch.py \
	ros2_control_hardware_type:=isaac \
	use_sim_time:=true \
	use_sim_time_for_control:=false \
	spawn_aux_controllers:=false \
	controller_spawn_delay:=5.0 \
	controller_manager_timeout:=60.0
```

4. Verify arm controller state:

```bash
source /opt/ros/humble/setup.bash
source /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/install/setup.bash
ros2 control list_controllers -c /controller_manager --verbose
```

Expected: `arm_controller` should be `active` and claim `panda_joint1..7/position`.

Notes:

- `Group 'hand' is not a chain` and `/recognize_objects not available` are common in this setup and are not the core arm-control path.
- If RViz exits unexpectedly, first ensure there is only one active MoveIt launch instance.
- For MTC pick and place, keep IsaacSim cube and MoveIt CollisionObject aligned (same pose/size).

Example:

```bash
/home/jacy/anaconda3/envs/env_issaclab/bin/python \
/home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
--setup-ros2-python-bridge \
--check-rclpy \
--inspect-graph
```
