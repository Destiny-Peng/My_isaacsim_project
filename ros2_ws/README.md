# ROS2 Workspace

## 1. 包概述

`ros2_ws` 是本项目的 ROS2 运行与开发主工作区，负责承接仿真桥接后的控制、规划和任务编排。

主要组成：

- `src/robot_description`: 机器人模型与资源。
- `src/moveit_robot_config`: MoveIt Setup Assistant 生成并扩展的配置包。
- `src/moveit_mtc_pick_place_demo`: 基于 MTC 的 pick&place 测试包。
- `src/moveit_task_constructor`: 上游 MTC 源码（作为依赖与参考）。

## 2. 如何启动测试

先构建工作区：

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

推荐测试顺序：

1. 先启动仿真（在 `sim/launch` 执行，详见该目录 README）。
2. 仅验证 MoveIt 与 Isaac 联动：

```bash
python3 /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/scripts/run_moveit_isaac_test.py \
  --param-file /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml
```

3. 验证 MTC demo：

```bash
python3 /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/scripts/run_mtc_demo.py \
  --param-file /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml
```

## 3. 参数说明

为避免长命令误配，已提供默认 YAML：

- MoveIt 启动默认参数：`src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- MTC 启动默认参数：`src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`

两个统一入口脚本：

- `scripts/run_moveit_isaac_test.py --param-file <yaml>`
- `scripts/run_mtc_demo.py --param-file <yaml>`

`--dry-run` 可用于先打印即将执行的 `ros2 launch` 命令。
