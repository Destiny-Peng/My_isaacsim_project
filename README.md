# Isaac Sim Fullstack

## 1. 包概述

本仓库用于验证和维护 `Isaac Sim + ROS2 + MoveIt2 + MTC` 的联动链路，目标是让 `mobile_base_with_franka` 在“仿真执行”和“规划控制”两侧稳定协同。

主要组成：

- `sim/`: Isaac Sim 场景启动、ROS2 Python bridge、联调参数配置。
- `ros2_ws/`: ROS2 工作区，包含 robot_description、MoveIt 配置、MTC demo 与上游 MTC 源码。
- `docs/`: 架构文档、任务进度、交接记录。

## 2. 如何启动测试

推荐顺序：先启动仿真，再启动 MoveIt/MTC。

步骤 1：启动 Isaac Sim（推荐使用参数文件）

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/launch/run_combined_car_franka_headless.py \
  --param-file sim/launch/config/sim_default.yaml
```

步骤 2：启动 MoveIt 基础联动测试（只测 MoveIt 与控制链）

```bash
python3 ros2_ws/scripts/run_moveit_isaac_test.py \
  --param-file ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml
```

步骤 3：启动 MTC 任务测试（可选）

```bash
python3 ros2_ws/scripts/run_mtc_demo.py \
  --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml
```

## 3. 参数说明

你可以用“参数文件 + 短命令”方式运行，避免反复输入长参数。

- 仿真参数文件：`sim/launch/config/sim_default.yaml`
  - 管理 GUI/headless、bridge 开关、平滑参数、demo cube 对齐参数。
- MoveIt 启动参数文件：`ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
  - 管理控制器时序、RViz 开关、aux 控制器是否启动。
- MTC 启动参数文件：`ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`
  - 管理 MTC 启动延迟、抓放位姿覆盖参数。

文档导航：

- `sim/launch/README.md`
- `ros2_ws/README.md`
- `ros2_ws/src/moveit_robot_config/README.md`
- `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
- `docs/ARCHITECTURE.md`
- `docs/TASK_RECORD.md`

## 4. Franka 阻抗控制示例（Standalone）

新增脚本：`sim/force_control/franka_force_control_standalone.py`

用途：

- 在 Isaac Sim 5.1.0 中直接对 Franka 前 7 个关节执行关节空间阻抗控制。
- 控制律：`tau = Kp * (q_ref - q) + Kd * (0 - dq)`，并对力矩做限幅。
- 启动后默认将当前姿态作为阻抗平衡点（也可通过参数给定目标位姿）。

关键参数：

- `--headless`：无界面运行（默认 GUI）。
- `--max-steps`：最大控制步数；`0` 表示一直运行到窗口关闭或 Ctrl+C。
- `--kp`：7 维关节刚度。
- `--damping-ratio`：阻尼比，按 `kd = 2*zeta*sqrt(kp)` 自动计算。
- `--torque-limit`：7 维关节力矩限幅。
- `--q-target`：7 维关节目标位置（单位 rad）。

示例命令：

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/force_control/franka_force_control_standalone.py \
  --headless \
  --max-steps 300 \
  --kp 120,120,110,100,80,60,40 \
  --damping-ratio 1.0 \
  --torque-limit 80,80,80,60,40,30,20 \
  --q-target 0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
```

兼容性说明：

- 继续保留原脚本路径 `sim/force_control/franka_force_control_standalone.py`，无需修改已有调用入口。
