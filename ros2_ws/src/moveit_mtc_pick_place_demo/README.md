# moveit_mtc_pick_place_demo

## 1. 包概述

`moveit_mtc_pick_place_demo` 是本仓库的 MTC 任务示例包，用于验证 `mobile_base_with_franka` 的抓放任务流程与可视化。

组成部分：

- `src/mtc_pick_place_demo.cpp`: 任务 stage 定义与执行。
- `launch/mtc_pick_place_demo.launch.py`: MTC + MoveIt 启动编排。
- `config/pick_place_params.yaml`: 任务参数（抓取目标、偏置、距离）。
- `config/mtc_pick_place.rviz`: MTC 面板配置。

## 2. 如何启动测试

步骤 1：先启动仿真

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/launch/run_combined_car_franka_headless.py \
  --param-file sim/launch/config/sim_default.yaml
```

步骤 2：启动 MTC demo（参数文件方式）

```bash
python3 ros2_ws/scripts/run_mtc_demo.py \
  --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml
```

步骤 3：若只想无界面运行，可在 YAML 中将 `launch_mtc_rviz: false`。

步骤 4：通过服务触发重试/重置（无需重启进程）

```bash
source /opt/ros/humble/setup.bash
source /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/install/setup.bash

# 推荐使用节点私有命名空间形式
ros2 service call /mtc_pick_place_demo/run_task std_srvs/srv/Trigger '{}'
ros2 service call /mtc_pick_place_demo/reset_task_state std_srvs/srv/Trigger '{}'
ros2 service call /mtc_pick_place_demo/reset_and_run std_srvs/srv/Trigger '{}'
```

服务语义：

- `/mtc_pick_place_demo/run_task`：触发一次规划/执行流程；如果节点已经缓存了上一次成功的 solution，则会直接复用该 solution 执行，不会重新规划。
- `/mtc_pick_place_demo/reset_task_state`：清理 task 状态与规划场景中的 demo 物体附着/残留物体。
- `/mtc_pick_place_demo/reset_and_run`：先 reset，再触发新一轮 run。
- reset 后会复用同一个 task 实例进行下一轮执行；只有调用 reset 才会清空缓存的 solution。

运行日志关键标记：

- 首次初始化 pipeline：`MTC task pipeline initialized once and will be reused across service reruns`
- 服务重跑复用：`Reusing existing MTC task pipeline (cached task object)`

稳定性修复（2026-04-22）：

- 修复了 `Node '/mtc_pick_place_demo' has already been added to an executor.` 崩溃。
- 根因是等待 joint state 时对已在 executor 中的节点再次 `spin_some`。
- 当前行为改为仅等待订阅回调更新 joint state，不再对本节点重复 spin。
- 移除了历史兜底参数 `keep_alive_after_execute_failure`（不再需要）。
- 移除了执行前 `waitForExecutionActionServers` 阶段，执行链路改为直接 `task.execute()`；若 action server 不可用会直接在 execute 阶段报错，便于定位真实故障环节。

动态起终点扩展（2026-04-22）：

- 新增可选外部位姿输入 topic（`geometry_msgs/msg/PoseStamped`）：
  - `pick_pose_topic`（默认 `/mtc_pick_place_demo/pick_pose`）
  - `place_pose_topic`（默认 `/mtc_pick_place_demo/place_pose`）
- 使能开关：
  - `use_external_pick_pose`
  - `use_external_place_pose`
- 行为说明：
  - 每次 run 前读取最新 pick/place 目标；如目标变化会自动清空 cached solution 并重建 pipeline，再进行规划执行。
  - 若外部 topic 未发布，则回退到 YAML 中的 `pick_pose_xyz` / `place_pose_xyz`。
  - topic 的 `frame_id` 需为空或等于 `world_frame`，否则该条消息会被忽略。

兼容端点：同一功能也可通过全局端点调用（`/run_task`、`/reset_task_state`、`/reset_and_run`）。

## 3. 参数说明

分两类参数文件：

- 启动参数：`config/mtc_launch_defaults.yaml`
  - 控制 `mtc_start_delay`、`spawn_aux_controllers`、`launch_mtc_rviz`、`use_sim_time/use_sim_time_for_control`、pick/place 标量覆盖。
- 任务参数：`config/pick_place_params.yaml`
  - 控制对象尺寸、抓取偏置、路径采样与执行复用参数：
    - `grasp_z_offset`
    - `grasp_rpy`
    - `grasp_angle_delta`
    - `grasp_max_ik_solutions`
    - `place_max_ik_solutions`
    - `grasp_ik_ignore_collisions`
    - `goal_joint_tolerance`
    - `move_to_pick_timeout_sec`
    - `move_to_place_timeout_sec`
    - `lift_min_distance`
    - `place_lower_min_distance`
    - `place_lower_distance`
    - `cartesian_step_size`
    - `pick_pose_topic`
    - `place_pose_topic`
    - `use_external_pick_pose`
    - `use_external_place_pose`

  速度均匀化建议参数（当前默认）：

  - `plan_velocity_scaling: 0.06`
  - `plan_acceleration_scaling: 0.03`
  - `approach_min_distance: 0.02`
  - `lift_min_distance: 0.03`
  - `place_lower_min_distance: 0.03`
  - `cartesian_step_size: 0.003`

  已清理无效参数：

  - `keep_alive_after_execute_failure`
  - `auto_run_on_start`
  - `place_retreat_distance`

建议：

- 保持 `spawn_aux_controllers: true`，确保夹爪控制链可用。
- 保持 IsaacSim demo cube 与 MTC object 参数一致，减少场景不一致导致的失败。
