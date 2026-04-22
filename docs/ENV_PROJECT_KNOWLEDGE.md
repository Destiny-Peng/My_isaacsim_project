# ENV + Project Knowledge (Must Follow)

## 1. 会话开始必读顺序

1. `docs/ENV_PROJECT_KNOWLEDGE.md`
2. `docs/TASK_RECORD.md`
3. `docs/ARCHITECTURE.md`

## 2. 环境与目录约束

- 系统：Ubuntu 22.04
- ROS2：Humble
- Isaac 专用环境：`env_issaclab`（alias: `isaac`）
- 严禁创建新的 Python 虚拟环境。

目录约束（强制）：

- 项目根目录：`isaac_sim_fullstack`
- ROS 工作目录：`isaac_sim_fullstack/ros2_ws`
- 禁止在其他项目目录执行编译/修改。

## 3. 环境切换规则

- 运行 Isaac Sim 相关内容：使用 `isaac` 环境。
- 运行 ROS2/MoveIt/MTC 命令：使用系统环境。

## 4. 标准启动命令

以下命令默认在 `isaac_sim_fullstack` 执行：

- 仿真：
  - `python3 sim/launch/run_combined_car_franka_headless.py --param-file sim/launch/config/sim_default.yaml`
- MoveIt：
  - `python3 ros2_ws/scripts/run_moveit_isaac_test.py --param-file ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- MTC：
  - `python3 ros2_ws/scripts/run_mtc_demo.py --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`

## 5. 每次执行前检查

必须先执行：

- `source /opt/ros/humble/setup.bash`
- `source ros2_ws/install/setup.bash`

必须确认：

- 当前 cwd 正确（项目根目录或 `ros2_ws`）。
- 没有残留旧进程（`move_group`、`mtc_pick_place_demo`、`controller_manager`、`rviz2`、仿真脚本）。
- 不并行启动两套 MoveIt/MTC。

## 6. 关键健康检查

- 话题：
  - `ros2 topic list | grep -E '^/clock$|^/joint_states$|^/joint_commands$'`
- 控制器：
  - `ros2 control list_controllers -c /controller_manager --verbose`
- 节点：
  - `ros2 node list | grep -E 'move_group|controller_manager|mtc|isaac'`

## 7. 测试纪律

- 一次实验只改一个关键变量。
- 每次实验记录：命令、日志路径、结论、下一步假设。
- 失败结论必须附日志证据。

## 8. 文档同步硬规则（强制）

- 任何代码改动，必须同批次更新对应 README（保持原结构）。
- README 必须写清：
  - 新增参数（默认值、作用、调参场景）
  - 新增命令/服务（完整命令、输入、预期行为）
  - 兼容性（旧命令/别名是否保留）
- 同时更新 `docs/TASK_RECORD.md`（根因-修复-验证命令-结果）。

## 9. 当前高优先级事实

- `/clock` 需要发布真实 sim time；wall-anchored 会导致 `use_sim_time` 失真。
- MoveIt/MTC/TF/控制链默认应统一启用 `use_sim_time=true`。
- 抓取失败排查优先级：
  1. 时钟链路
  2. 控制器激活
  3. demo cube 物理属性（刚体、摩擦、CCD、阻尼、接触偏置、求解迭代）
