**MoveIt2 控制—快速上手（针对 arm_only_franka）**

概述：本说明描述如何在当前工作区上使用 MoveIt2 对 `arm_only_franka` 进行规划与执行验证（使用快速桥接 `arm_trajectory_bridge` 执行轨迹）。目标是尽快用官方 demo 流程验证规划与执行闭环，并说明避免与底盘碰撞的几种做法。

前提
- 已配置并编译 ROS2 workspace：在工作区根目录运行 `source /opt/ros/humble/setup.bash`，然后 `colcon build` 并 `source install/setup.bash`。
- 已存在 arm-only URDF 文件：[ros2_ws/src/robot_description/urdf/arm_only_franka.urdf](ros2_ws/src/robot_description/urdf/arm_only_franka.urdf)
- 已实现快速执行桥：`manipulation_interface` 中的 `arm_trajectory_bridge`（用于把 MoveIt 的 JointTrajectory 转发到控制器）

文件与关键点
- 机器人描述（arm-only）：[ros2_ws/src/robot_description/urdf/arm_only_franka.urdf](ros2_ws/src/robot_description/urdf/arm_only_franka.urdf)
- 加载 robot_description 的 launch：[ros2_ws/src/robot_description/launch/robot_description_launch.py](ros2_ws/src/robot_description/launch/robot_description_launch.py)
- Quick bridge launch：`ros2 launch manipulation_interface bridge.launch.py`（可选 remap 话题参数）

快速启动（推荐顺序）
1. 在终端 A：构建并 source 工作区
```bash
source /opt/ros/humble/setup.bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
2. 在终端 B：启动 `robot_state_publisher` 并加载 arm-only URDF（示例：使用 package 的 launch，可传入相对 URDF 路径）
```bash
ros2 launch robot_description robot_description_launch.py robot_file:=urdf/arm_only_franka.urdf
```
该 launch 会把 `robot_description` 参数注入 ROS 参数服务器，使 MoveIt / `robot_state_publisher` 可见。

3. 在终端 C：启动 MoveIt（两种常见方式）
- 如果已有 MoveIt 配置包（`moveit_config`）为 arm-only：
  ```bash
  ros2 launch <your_moveit_config_pkg> moveit_planning_execution.launch.py
  ```
- 如果没有现成的配置：先运行 MoveIt Setup Assistant（生成 SRDF 与控制器配置），或直接使用官方 demo 作为模板。示例：
  ```bash
  ros2 launch moveit_setup_assistant moveit_setup_assistant.launch.py
  ```

4. 在终端 D：启动快速桥接器（将 MoveIt 产出的 `JointTrajectory` 转发到控制器主题）
```bash
ros2 launch manipulation_interface bridge.launch.py input_topic:=/arm_trajectory output_topic:=/arm_controller/command
```
如果你使用的是不同的控制话题，请调整 `output_topic`。

5. 在 RViz 中测试
- 打开 RViz（通常由 MoveIt launch 打开）。确保 `Planning Request` -> `Planning Group` 设置为机械臂组（例如 `panda_arm`），`Planning Frame` 设为 `arm_base` 或者与 `arm_only_franka.urdf` 中定义的根 link 一致。
- 在 MotionPlanning 面板中尝试 `Plan` → `Execute`。MoveIt 会发布一个 `trajectory_msgs/JointTrajectory` 到执行动作点（通常是 `/<group>/follow_joint_trajectory` 或 `arm_trajectory`，取决于配置），`arm_trajectory_bridge` 会把该消息转发到你的控制器话题供仿真或控制器消费。

碰撞避免与规划场景建议
- 已在 `arm_only_franka.urdf` 中包含一个固定的 `arm_base` 碰撞盒（大小约为底盘尺寸），MoveIt 会把它视为固定的自体碰撞体，从而在规划时自动避开底盘。
- 若需更灵活地在运行时调整底盘碰撞体，可用 PlanningScene 动态添加/移除盒子（示例 Python 在下方）。这对于多种底盘配置或动态障碍很有用。

在 MoveIt 中动态添加底盘碰撞体（示例）
```python
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
ps = PlanningSceneInterface()
box_pose = PoseStamped()
box_pose.header.frame_id = "arm_base"
box_pose.pose.position.x = 0.0
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.092
ps.add_box("mobile_base_box", box_pose, size=(0.42, 0.31, 0.184))
```

控制器与 MoveIt 映射（注意）
- 快速验证：使用 `arm_trajectory_bridge` 将 MoveIt 输出直接推送到控制器命令话题（无需 ros2_control）。
- 长期：建议基于 `ros2_control` / `ros_controllers` 在模拟器或真实机器人上运行适当的控制器，并在 MoveIt 的 `moveit_controllers.yaml` 中声明 `FollowJointTrajectory` 接口映射。

故障排查要点
- 若在 RViz 看不到机械臂：检查 `robot_description` 是否已正确加载（`ros2 param get /robot_state_publisher robot_description`）以及 `joint_states` 话题是否发布对应的关节名。
- Plan 失败或碰撞：确认 `Planning Frame`、`Allowed Collision Matrix`（ACM）与 SRDF 中的碰撞禁用项是否正确配置。
- 执行无动作：检查 `arm_trajectory_bridge` 是否正在转发消息，以及 `output_topic` 是否被模拟器或控制器订阅。

下一步建议
- 我可以把 `robot_bringup/launch/bringup.launch.py` 增加一个参数以便选择加载 `arm_only_franka.urdf`（或 `combined_car_franka.urdf`），并加入一个示例 MoveIt launch 对接命令；要我现在实施吗？

---
文件位置参考：
- [ros2_ws/src/robot_description/urdf/arm_only_franka.urdf](ros2_ws/src/robot_description/urdf/arm_only_franka.urdf)
- [ros2_ws/src/manipulation_interface/](ros2_ws/src/manipulation_interface/)
