# robot_description

## 1. 包概述

`robot_description` 提供机器人模型资源，是 MoveIt、robot_state_publisher 与 Isaac Sim 场景对齐的模型基线。

组成部分：

- `urdf/`: 机器人 URDF/xacro。
- `meshes/` 等资源目录：可视化与碰撞模型依赖。
- 启动文件：用于发布 TF 与机器人状态。

## 2. 如何启动测试

若只验证模型发布链路，可单独启动：

```bash
source /opt/ros/humble/setup.bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_description robot_description_launch.py
```

在本项目的联动路径中，通常由 MoveIt 启动文件间接加载，无需单独手工启动。

## 3. 参数说明

本包本身不维护独立的“长参数启动链”，主要参数由上层包管理：

- MoveIt 联动参数：`../moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- 仿真侧参数：`../../../sim/launch/config/sim_default.yaml`

如果修改模型坐标语义，请同步检查：

- MoveIt 的 `robot_description` / SRDF 对应关系。
- Isaac Sim USD 中 base 与 arm 根节点姿态。
