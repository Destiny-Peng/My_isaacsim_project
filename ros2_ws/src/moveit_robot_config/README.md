# moveit_robot_config

## 1. 包概述

`moveit_robot_config` 是通过 MoveIt Setup Assistant 生成并在本项目中二次调整的配置包，用于验证 MoveIt 与 Isaac Sim 的联动控制链。

组成部分：

- `config/`: SRDF、运动学、控制器、joint limits、RViz 配置。
- `launch/`: MoveIt 启动编排（核心为 `isaac_sim_moveit.launch.py`）。
- `MOVEIT_ISAAC_INTEGRATION_CHANGELOG.md`: 联动稳定化改造记录。

## 2. 如何启动测试

这是“先仿真、后 MoveIt”的基础联动测试。

步骤 1：启动 Isaac Sim（推荐参数文件方式）

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/launch/run_combined_car_franka_headless.py \
  --param-file sim/launch/config/sim_default.yaml
```

步骤 2：启动 MoveIt 联动测试

```bash
python3 ros2_ws/scripts/run_moveit_isaac_test.py \
  --param-file ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml
```

步骤 3：检查控制器状态（含夹爪）

```bash
source /opt/ros/humble/setup.bash
source /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/install/setup.bash
ros2 control list_controllers -c /controller_manager --verbose
```

预期：`arm_controller`、`hand_controller`、`joint_state_broadcaster` 为 `active`。

## 3. 参数说明

默认参数文件：`config/isaac_sim_moveit_defaults.yaml`。

主要参数：

- `ros2_control_hardware_type`: 默认 `isaac`
- `use_sim_time`: MoveIt 是否使用仿真时钟
- `use_sim_time_for_control`: ros2_control 是否使用仿真时钟（当前默认建议 `true`）
- `spawn_aux_controllers`: 是否启动 `hand_controller` 与 `joint_state_broadcaster`
- `controller_spawn_delay`: 控制器拉起延迟
- `controller_manager_timeout`: 控制器服务等待超时
- `launch_rviz`: 是否启动 MoveIt RViz

建议：日常测试保持 `spawn_aux_controllers: true`，避免夹爪未激活导致“手爪不响应”。
