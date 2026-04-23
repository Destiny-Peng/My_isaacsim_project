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
- RViz 配置只保留一个 `Motion Planning Tasks` 实例，避免同名面板/显示项重复呈现为两个 task。

稳定性修复（2026-04-22）：

- 修复了 `Node '/mtc_pick_place_demo' has already been added to an executor.` 崩溃。
- 根因是等待 joint state 时对已在 executor 中的节点再次 `spin_some`。
- 当前行为改为仅等待订阅回调更新 joint state，不再对本节点重复 spin。
- 移除了历史兜底参数 `keep_alive_after_execute_failure`（不再需要）。
- 移除了执行前 `waitForExecutionActionServers` 阶段，执行链路改为直接 `task.execute()`；若 action server 不可用会直接在 execute 阶段报错，便于定位真实故障环节。
- 清理了当前不再使用的辅助代码路径，保留实际消费的 reset / sync / bridge 逻辑。

动态起终点扩展（2026-04-22）：

- 新增可选外部位姿输入 topic（`geometry_msgs/msg/PoseStamped`）：
  - `pick_pose_topic`（默认 `/mtc_pick_place_demo/pick_pose`）
  - `place_pose_topic`（默认 `/mtc_pick_place_demo/place_pose`）
- 使能开关：
  - `use_external_pick_pose`
  - `use_external_place_pose`
- 行为说明：
  - 每次 run 前读取最新 pick/place 目标；如目标变化会清空 cached solution 并重新 plan，但不重建 pipeline。
  - 若外部 topic 未发布，则回退到 YAML 中的 `pick_pose_xyz` / `place_pose_xyz`。
  - topic 的 `frame_id` 需为空或等于 `world_frame`，否则该条消息会被忽略。
  - 新消息是一次性消费语义：消费后会清空 pending 标记，不会因旧消息重复触发重规划。

放置阶段碰撞时机修复（2026-04-22）：

- `forbid collision (hand,object)` 调整到 `detach object` 之后、`retreat after place` 之前，避免放置后回撤路径仍允许手与 cube 相交。

桌面与 Isaac Plane 对齐（2026-04-22）：

- MTC 默认 `support_surface_xyz`/`support_surface_size_xyz` 对齐到 Isaac 侧常用值（`[0.55, 0.0, 0.49]` / `[0.8, 0.8, 0.10]`）。
- 同时移除了旧的标量兼容参数入口（如 `support_surface_x`、`support_surface_size_x` 等），避免与向量参数双通道冲突。

兼容端点：同一功能也可通过全局端点调用（`/run_task`、`/reset_task_state`、`/reset_and_run`）。

## 3. 参数说明

参数文件职责已经拆分：

- 启动参数：`config/mtc_launch_defaults.yaml`
  - 只负责 launch 级行为：`use_sim_time`、`use_sim_time_for_control`、`controller_spawn_delay`、`controller_manager_timeout`、`mtc_start_delay`、`spawn_aux_controllers`、`launch_mtc_rviz`
  - 不再承载 pick/place/object_size/execute 等任务参数。
- 任务参数：`config/pick_place_params.yaml`
  - 只负责 MTC 任务本身：对象/支撑面几何、抓取偏置、IK/路径参数、执行开关、外部位姿输入。
  - 当前包含：
    - `pick_pose_xyz`
    - `place_pose_xyz`
    - `pick_pose_topic`
    - `place_pose_topic`
    - `use_external_pick_pose`
    - `use_external_place_pose`
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

外部位姿行为说明：

- `pick_pose_topic` / `place_pose_topic` 收到新 `PoseStamped` 后，下一次 `run` 会消费该新值。
- 消费后对应“待处理更新”标记会被清空，因此不会因为旧消息一直反复触发重新规划。
- 若 topic 长时间不再发新消息，节点会继续保持当前已生效的 pick/place 目标，不会退回成默认值。

建议：

- 保持 `spawn_aux_controllers: true`，确保夹爪控制链可用。
- 保持 IsaacSim demo cube 与 MTC object 参数一致，减少场景不一致导致的失败。
