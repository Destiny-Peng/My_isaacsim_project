**Isaac Sim Fullstack — ROS2 Workspace 概览**

项目根说明：本 ROS2 workspace 包含针对 Isaac Sim + ROS2 集成的若干包（机械臂规划、桥接、描述、TF 管理、感知与导航占位）。本文件简要说明每个 `pkg` 的职责与常用启动/测试命令，帮助你快速定位与验证控制流程。

**Packages**
- **robot_description**: 机器人模型（URDF/xacro）。
  - 描述：包含合并版与 arm-only 的 URDF，用于 `robot_state_publisher`、MoveIt2 和其他需要 robot_description 的工具。
  - 关键路径：[ros2_ws/src/robot_description](ros2_ws/src/robot_description)
  - 快速启动（加载 arm-only URDF）：
    ```bash
    ros2 launch robot_description robot_description_launch.py robot_file:=urdf/arm_only_franka.urdf
    ```

- **robot_bringup**: 机器人启动集合（包含 `robot_state_publisher` 等）。
  - 描述：汇总 robot_description、tf 发布、joint_state 发布等，作为整体 bringup 的入口。
  - 关键路径：[ros2_ws/src/robot_bringup](ros2_ws/src/robot_bringup)
  - 示例：包含 `bringup.launch.py`（可扩展以加载 arm-only 或 combined URDF）。

- **manipulation_interface**: MoveIt 与控制器之间的接口（快速桥接实现）。
  - 描述：实现 `arm_trajectory_bridge`（将 MoveIt 发布的 `trajectory_msgs/JointTrajectory` 转发到控制器话题），并包含桥接的 launch 与示例 README。
  - 关键路径：[ros2_ws/src/manipulation_interface](ros2_ws/src/manipulation_interface)
  - 快速启动（桥接）：
    ```bash
    ros2 launch manipulation_interface bridge.launch.py input_topic:=/arm_trajectory output_topic:=/arm_controller/command
    ```
  - 临时方法（若 launch 未安装）：直接运行脚本：
    ```bash
    python3 src/manipulation_interface/src/arm_trajectory_bridge.py
    ```

- **navigation_interface**: 底盘导航接口（Nav2 集成占位）。
  - 描述：负责与 Nav2 协作的接口代码、cmd_vel/odom 的桥接与状态协调（当前为占位，后续将集成 Nav2 启动与协调逻辑）。
  - 关键路径：[ros2_ws/src/navigation_interface](ros2_ws/src/navigation_interface)

- **perception**: 感知组件（相机、点云、物体检测的占位与管线）。
  - 描述：包装相机数据处理、视觉检测、和对 MoveIt 的物体抓取位姿建议（未完全实现，包含演示脚本）。
  - 关键路径：[ros2_ws/src/perception](ros2_ws/src/perception)

- **tf_manager**: TF 管理器（静态与动态变换发布）。
  - 描述：负责发布 `odom->base_link`、`base_link->arm_base` 等静态/动态 TF，以及管理 TF 拓扑以满足 Nav2 与 MoveIt 的需求。
  - 关键路径：[ros2_ws/src/tf_manager](ros2_ws/src/tf_manager)

**其他资源**
- `scripts/`: 示例脚本（`run.py`、`random_agent.py` 等）。
- `outputs/` 与 `logs/`: 训练/运行产出目录（仿真或 RL 日志）。

**如何快速验证 MoveIt -> 执行（控制部分）**
1. 构建并 source 工作区：
```bash
source /opt/ros/humble/setup.bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
2. 启动 robot_description（加载 `arm_only_franka.urdf`）：
```bash
ros2 launch robot_description robot_description_launch.py robot_file:=urdf/arm_only_franka.urdf
```
3. 启动桥接（或直接运行桥接脚本）：
```bash
ros2 launch manipulation_interface bridge.launch.py input_topic:=/arm_trajectory output_topic:=/arm_controller/command
# 或（若 launch 未生效）：
python3 src/manipulation_interface/src/arm_trajectory_bridge.py
```
4. 启动 MoveIt（使用 arm-only 的 moveit_config，或用 MoveIt Setup Assistant 生成配置），在 RViz 中 Plan -> Execute，轨迹将发布到 `/arm_trajectory` 并被桥接至 `/arm_controller/command`。

**注意事项与建议**
- 为避免机械臂与底盘碰撞：在 `arm_only_franka.urdf` 中包含固定碰撞盒 `arm_base`（已实现），或在启动时通过 PlanningScene 动态添加底盘碰撞体。
- 长期集成建议：使用 `ros2_control` 在模拟器中运行控制器，并将 MoveIt 的 controllers 配置为 `FollowJointTrajectory` 接口。

如果你希望我把 `robot_bringup` 的 launch 增加一个 `use_arm_only` 参数（自动切换加载 `arm_only_franka.urdf`），或现在直接生成 MoveIt 配置文件（SRDF、moveit_controllers.yaml），告诉我下一步偏好。
