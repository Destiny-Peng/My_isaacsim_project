# Architecture Overview

## 1. 系统目标

本项目用于验证 Isaac Sim 与 MoveIt2/MTC 的抓放闭环：

Isaac Sim 负责物理执行与传感状态，MoveIt/MTC 负责规划与任务编排。

## 2. 主链路（当前有效）

`Isaac Sim (timeline sim time + physics) -> ROS2 bridge (/clock /joint_states /joint_commands) -> MoveIt2/MTC planning+execute -> ros2_control -> Isaac Sim articulation`

关键原则：

- `/clock` 必须发布真实仿真时间（sim time），不能使用 wall-clock 锚定时间。
- MoveIt、MTC、TF、ros2_control 统一启用 `use_sim_time=true`。
- 规划场景与仿真场景同时维护同一套抓取物体与支撑面参数。

## 3. 组件与职责

### 3.1 仿真层

- `sim/launch/run_combined_car_franka_headless.py`
  - 启动 Isaac Sim（GUI/headless）。
  - 发布 `/joint_states`、`/clock`，订阅 `/joint_commands`。
  - 创建/更新 demo cube 与 support surface 的物理属性（质量、摩擦、阻尼、CCD、接触偏置、求解迭代）。
  - 提供 reset 服务：
    - `/isaac/reset_demo_cube`
    - `/isaac/reset_robot_initial_pose`
    - `/isaac/reset_demo_scene`

- `sim/launch/config/sim_default.yaml`
  - 仿真默认参数源。
  - 当前默认：`clock_mode: sim`。

### 3.2 MoveIt 配置层

- `ros2_ws/src/moveit_robot_config`
  - 提供 `isaac_sim_moveit.launch.py`。
  - 启动 `move_group`、`robot_state_publisher`、`static_transform_publisher`、`ros2_control_node` 与 controller spawners。
  - 默认开启 `use_sim_time` 与 `use_sim_time_for_control`。

### 3.3 MTC 任务层

- `ros2_ws/src/moveit_mtc_pick_place_demo`
  - 任务节点 `mtc_pick_place_demo`。
  - `Task` 和 `Solution` 成员缓存复用：
    - 首次构建 pipeline 并 plan。
    - 后续 run 可复用 cached solution execute。
    - reset 清空缓存并重置任务状态。
  - 服务接口：
    - `/mtc_pick_place_demo/run_task`（兼容 `/run_task`）
    - `/mtc_pick_place_demo/reset_task_state`（兼容 `/reset_task_state`）
    - `/mtc_pick_place_demo/reset_and_run`（兼容 `/reset_and_run`）

## 4. 碰撞与物理分工

- MoveIt/MTC：规划期碰撞检查（PlanningScene）。
- Isaac Sim：执行期接触、摩擦、刚体动力学。

抓取稳定性的必要条件：

- demo cube 必须是动态刚体（dynamic rigid body）。
- cube 与 table 必须是 collider，并绑定 physics material。
- cube 推荐启用 CCD、阻尼、合理接触偏置与求解迭代。

## 5. 运行与调试入口

- 仿真启动：`sim/launch/run_combined_car_franka_headless.py`
- MoveIt 启动：`ros2_ws/scripts/run_moveit_isaac_test.py`
- MTC 启动：`ros2_ws/scripts/run_mtc_demo.py`

推荐顺序：

1. 启动仿真（先有 `/clock` 与 `/joint_states`）。
2. 启动 MoveIt。
3. 启动 MTC 并触发服务执行。

## 6. 文档索引

- 项目总览：`README.md`
- 环境与强制规则：`docs/ENV_PROJECT_KNOWLEDGE.md`
- 调试记录：`docs/TASK_RECORD.md`
- 仿真说明：`sim/launch/README.md`
- MoveIt 说明：`ros2_ws/src/moveit_robot_config/README.md`
- MTC 说明：`ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
