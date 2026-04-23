# Sim Launch Package

## 1. 包概述

`sim/launch` 负责启动 Isaac Sim 场景并提供 ROS2 Python bridge。

组成部分：

- `run_combined_car_franka_headless.py`: 主启动脚本（支持 GUI/headless、bridge、现有 prim 位置同步）。
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

- `simulation.headless`（`true` 为无界面，`false` 为 GUI）
- `simulation.setup_ros2_python_bridge`
- `simulation.clock_mode`（建议 `sim`，发布真实仿真时钟）
- `simulation.sync_mtc_demo_object`
- `demo_object.demo_object_prim`（默认 `/Cube`）
- `demo_object.support_surface_prim`（默认 `/Plane`）
- `demo_object.support_surface_center`（建议与 MTC 的 `support_surface_xyz` 保持一致）

说明：

- `demo_object_prim`、`demo_object_pick_position`、`support_surface_prim`、`support_surface_center` 现在只通过 YAML 管理，已移除对应 `--` CLI 参数，减少重复配置入口。

注意（2026-04-22 行为调整）：

- 脚本不再在运行时创建 ActionGraph。
- 脚本不再在运行时创建/覆盖 cube 与 table 物理体。
- `sync_mtc_demo_object` 与 `sync_demo_support_surface` 仅更新已有 prim 的位置。
- 原先用于运行时创建/调物理属性的一组参数已移除（size/color/dynamic/mass/friction/damping/contact offset/solver iterations）。
- 历史参数 `simulation.gui` 已废弃；若旧 YAML 仍包含该字段，脚本会做兼容映射并给出告警，建议改为 `simulation.headless`。
- `/clock` 发布改为 ROS2 标准 clock QoS，减少 `ros2 topic echo /clock --once` 场景下的丢包提示。
- 当前脚本继续清理了未使用的辅助函数，保留的代码只对应 bridge、reset 和现有 prim 同步路径。

运行时服务（由 Python bridge 提供）：

- `/isaac/reset_demo_cube`：重置 demo cube 完整 pose（位置 + 姿态），并清零动态刚体线速度/角速度
- `/isaac/reset_robot_initial_pose`：只重置机械臂到 `initial_positions.yaml`
- `/isaac/reset_demo_scene`：同时重置 cube + 机械臂

抓取指标评估（新增）：

- `grasp_metric_evaluator.py` 提供 `GraspMetricEvaluator`。
- 它使用 Replicator `pose` / `contact` annotator 记录对象和夹爪的时序数据，不依赖 `pxr.UsdGeom` 手工算位姿。
- 已接入 `run_combined_car_franka_headless.py` 主循环，可在一次仿真中自动完成记录和评估。
- 默认通过 `grasp_eval.*` 配置段控制。
- 输出：
  - 样本时序 CSV（`sim_time_s`、`object_xyz`、`object_vxyz`、接触状态、gripper_closed）
  - 指标 JSON（`success`、`stability_z_std`、`slippage_detected` 等）
- `to_dataframe()` 会优先返回 `pandas.DataFrame`；如果环境里没有 pandas，则回退为 NumPy 结构化数组。

开启录制与评估示例：

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/launch/run_combined_car_franka_headless.py \
  --param-file sim/launch/config/sim_default.yaml \
  --enable-grasp-metric-eval \
  --grasp-eval-target-z 0.62 \
  --grasp-eval-output-prefix sim/outputs/grasp_eval_trial1
```

可选关键参数：

- `--grasp-eval-object-prim`：被抓取物体 prim
- `--grasp-eval-gripper-prim`：夹爪 prim
- `--grasp-eval-table-prim`：支撑面 prim
- `--grasp-eval-gripper-joint-name`：用于判断夹爪闭合的关节名
- `--grasp-eval-gripper-close-threshold`：闭合阈值
- `--grasp-eval-slip-threshold`：滑移检测阈值
- `--grasp-eval-output-csv` / `--grasp-eval-output-json`：显式输出路径

命令示例：

```bash
source /opt/ros/humble/setup.bash
source /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/install/setup.bash

ros2 service call /isaac/reset_demo_cube std_srvs/srv/Trigger '{}'
ros2 service call /isaac/reset_robot_initial_pose std_srvs/srv/Trigger '{}'
ros2 service call /isaac/reset_demo_scene std_srvs/srv/Trigger '{}'
```

坐标与尺寸语义（与 MTC 对齐）：

- `demo_object_pick_position` 表示 cube 的中心点坐标。
- `support_surface_center` 默认为固定值（`0.55,0.0,0.49`），不再跟随 cube 自动重算，避免 IsaacSim 与 MTC 桌面坐标漂移。

示例：仅临时覆盖步数

```bash
python3 run_combined_car_franka_headless.py \
  --param-file config/sim_default.yaml \
  --steps 40000
```
