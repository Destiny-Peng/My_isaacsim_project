# ROS2 Workspace (`ros2_ws`)

本工作区当前主要聚焦 `mobile_base_with_franka` 的 MoveIt2 + MTC + IsaacSim 联动。

## 1. 包与职责

- `src/robot_description`
  - 机器人 URDF/USD 资源与模型文件。
  - 用于 MoveIt 配置、Isaac Sim 场景引用、以及控制链统一模型源。

- `src/moveit_robot_config`
  - MoveIt 配置包（SRDF、controllers、launch）。
  - 关键入口：`launch/isaac_sim_moveit.launch.py`。
  - 已包含 controller 时序稳定化参数（delay/timeout/use_sim_time_for_control）。

- `src/moveit_mtc_pick_place_demo`
  - 本仓库的 MTC pick and place 示例实现。
  - 关键入口：`launch/mtc_pick_place_demo.launch.py`。
  - 已支持：MTC RViz 面板、pick/place 参数覆盖、语义参数显式注入。

- `src/moveit_task_constructor`
  - 官方 MTC 源码（core/demo/visualization/capabilities/msgs）。
  - 作为依赖与参考实现，便于二次开发和排障。

## 2. 推荐构建命令

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
source /opt/ros/humble/setup.bash
colcon build \
  --base-paths /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/src \
  --build-base /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/build \
  --install-base /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/install
source install/setup.bash
```

## 3. 联动运行入口

1. 先启动 IsaacSim bridge（见 `sim/launch/README.md`）。
2. 再启动 MTC demo：

```bash
ros2 launch moveit_mtc_pick_place_demo mtc_pick_place_demo.launch.py
```

## 4. 交接导航

- 项目总览：`../README.md`
- 架构文档：`../docs/ARCHITECTURE.md`
- 任务记录：`../docs/TASK_RECORD.md`
- MTC demo 说明：`src/moveit_mtc_pick_place_demo/README.md`
