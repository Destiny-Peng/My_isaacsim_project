# Sim Launch Package

## 1. 包概述

`sim/launch` 负责启动 Isaac Sim 场景并提供 ROS2 Python bridge。

组成部分：

- `run_combined_car_franka_headless.py`: 主启动脚本（支持 GUI/headless、bridge、demo cube 同步）。
- `config/sim_default.yaml`: 默认参数文件。
- `run_combined_car_franka_headless_guide.md`: 脚本内部结构说明。

## 2. 如何启动测试

推荐方式（参数文件）：

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/launch/run_combined_car_franka_headless.py \
  --param-file sim/launch/config/sim_default.yaml
```

如果你手动开 Isaac Sim GUI，也可直接加载：

- `ros2_ws/src/robot_description/urdf/combined_car_franka/combined_car_franka.usd`

并确保 bridge 话题对应：

- `/joint_commands`
- `/joint_states`
- `/clock`

## 3. 参数说明

脚本已支持：

- `--param-file <yaml>`: 从 YAML 读取默认参数。
- CLI 参数优先级高于 YAML（可临时覆盖）。

默认参数文件：`config/sim_default.yaml`，建议重点关注：

- `simulation.gui` / `simulation.headless`
- `simulation.setup_ros2_python_bridge`
- `simulation.sync_mtc_demo_object`
- `smoothing.default_max_joint_velocity`
- `smoothing.max_joint_step_rad`

示例：仅临时覆盖步数

```bash
python3 run_combined_car_franka_headless.py \
  --param-file config/sim_default.yaml \
  --steps 40000
```
