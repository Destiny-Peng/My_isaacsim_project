# Architecture Overview

## 1. 当前系统边界

- Simulator: NVIDIA Isaac Sim (`sim/launch/run_combined_car_franka_headless.py`)
- Planner: MoveIt2 + MTC
- Middleware: ROS2 Humble

当前主链路聚焦机械臂抓放联动：

`IsaacSim (joint states/clock) -> MoveIt/MTC plan -> ros2_control command -> IsaacSim`

## 2. 关键组件

- `ros2_ws/src/robot_description`
  - 机器人模型与 USD/URDF 资源源头。

- `ros2_ws/src/moveit_robot_config`
  - MoveIt2 配置与 launch。
  - 提供 `isaac_sim_moveit.launch.py`（含 controller 启停稳定化参数）。

- `ros2_ws/src/moveit_mtc_pick_place_demo`
  - MTC 教程风格 pick and place 示例。
  - 包含 stage 流程、参数覆盖和 RViz MTC 可视化。

- `ros2_ws/src/moveit_task_constructor`
  - 官方 MTC 源码依赖和示例参考。

- `sim/launch`
  - Isaac Sim 启动、桥接和诊断脚本。
  - 已支持在仿真中同步 demo cube（对齐 MTC CollisionObject）。

## 3. 碰撞职责分工

- MoveIt/MTC: 规划期碰撞检查（PlanningScene）。
- Isaac Sim: 执行期物理碰撞与接触响应。

为了确保规划与执行一致，需要双场景对齐：

1. MTC 添加 CollisionObject。
2. Isaac Sim 场景创建同 pose/size 的真实物体。

## 4. 已知稳定策略

- `use_sim_time_for_control:=false` 避免控制栈时钟抖动。
- spawner 延迟与超时参数降低 `/controller_manager` 竞态。
- MTC launch 显式注入 `robot_description*`，规避 SRDF 空加载。

## 5. 文档索引（交接入口）

- 项目 README: `../README.md`
- ROS2 工作区 README: `../ros2_ws/README.md`
- 仿真与桥接 README: `../sim/launch/README.md`
- MTC demo README: `../ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
- 任务记录: `TASK_RECORD.md`
