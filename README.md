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

