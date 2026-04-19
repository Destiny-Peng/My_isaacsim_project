# 环境与项目知识收集表（可持续补全）

## 1. 文档目的

用于集中记录 Isaac Sim 联调所需的环境事实、项目约定、启动顺序与排障知识，避免上下文截断后信息丢失。

## 2. 已确认事实（当前有效）

### 2.1 环境隔离原则

- Isaac Sim 专用 Conda 环境：env_issaclab
- 快捷别名：alias isaac='conda activate env_issaclab'
- 约束：只有运行 Isaac Sim 相关内容时使用该 Conda 环境；项目其余内容默认不使用该 Conda 环境。

### 2.2 现有启动顺序（已验证方向）

- 先启动仿真与 bridge。
- 再启动 MoveIt / MTC。

## 3. 填空区：环境层知识

请按需补充，空白处可直接替换为你的答案。

### 3.1 系统与基础依赖

- 操作系统版本： ubuntu22.04
- ROS2 发行版： humble
- Python 版本（系统）： 3.10
- Python 版本（env_issaclab）： 3.11
- CUDA / 驱动版本（如相关）： 12.8

### 3.2 Shell 与环境变量

- 默认 shell： [待补充]
- 常用环境变量：
  - ROS_DOMAIN_ID： 无
  - RMW_IMPLEMENTATION： 无
  - 其他关键变量： 无

### 3.3 环境激活与切换规范

- 进入 Isaac 环境命令： isaac
- 退出 Isaac 环境命令： conda deactivate
- 何时必须在 env_issaclab 中执行： 运行isaacsim仿真环境
- 何时必须在系统环境中执行： ros2相关

## 4. 填空区：项目结构与职责

### 4.1 目录职责

- sim/：见 `sim/launch/README.md`（仿真入口、bridge、参数文件）。
- ros2_ws/：见 `ros2_ws/README.md`（ROS2 工作区构建与运行入口）。
- docs/：见 `docs/ARCHITECTURE.md` 与 `docs/TASK_RECORD.md`（架构边界与调试进展）。

### 4.2 包职责（可逐步完善）

- moveit_robot_config：见 `ros2_ws/src/moveit_robot_config/README.md`。
- moveit_mtc_pick_place_demo：见 `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`。
- robot_description：见 `ros2_ws/src/robot_description/README.md`。
- moveit_task_constructor：见 `ros2_ws/src/moveit_task_constructor/README.md` 与 `ros2_ws/src/moveit_task_constructor/demo/README.md`。

## 5. 填空区：标准运行流程

### 5.1 一键/短命令入口

- 仿真启动命令：`python3 sim/launch/run_combined_car_franka_headless.py --param-file sim/launch/config/sim_default.yaml`
- MoveIt 启动命令：`python3 ros2_ws/scripts/run_moveit_isaac_test.py --param-file ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- MTC 启动命令：`python3 ros2_ws/scripts/run_mtc_demo.py --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`

### 5.2 参数文件约定

- 仿真默认 YAML：`sim/launch/config/sim_default.yaml`
- MoveIt 默认 YAML：`ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- MTC 默认 YAML：`ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`
- 参数覆盖规则（CLI 与 YAML 优先级）：CLI 参数高于 YAML 默认值（见各启动脚本 README）。

## 6. 填空区：验证与验收标准

### 6.1 必查项

- 话题健康检查命令：`ros2 topic list | grep -E '^/clock$|^/joint_states$|^/joint_commands$'`
- 控制器状态检查命令：`ros2 control list_controllers -c /controller_manager --verbose`
- 关键节点在线检查命令：`ros2 node list | grep -E 'move_group|controller_manager|mtc|isaac'`

### 6.2 成功判据

- MoveIt 联动成功判据：`arm_controller` 为 active，且 `/clock`、`/joint_states` 持续更新。
- MTC demo 成功判据：日志出现 `MTC pick and place completed successfully`。
- 失败时第一排查点：先确认是否重复启动了 MoveIt 栈（`mtc_pick_place_demo.launch.py` 已内含 MoveIt）。

## 7. 填空区：故障与处置经验

按“现象 -> 根因 -> 处置 -> 复现条件”记录。

- 案例 1：
  - 现象：MTC 日志持续出现 `Skipping invalid start state` / `Invalid start state`。
  - 根因：尚未完全收敛；已确认并非“未收到 joint_states”单因子。当前证据显示系统可能在零位配置下进入规划，且请求上下文与 stage 组合会触发起始态无效。
  - 处置：
    - 在 `mtc_pick_place_demo.cpp` 增加任务启动前 joint_state 等待。
    - 运行期通过 `/get_planning_scene` 与 `/joint_states` 双向核对当前状态来源。
    - 保持单 MoveIt 栈运行，避免并行栈污染诊断。
  - 复现条件：仅启动 `mtc_pick_place_demo.launch.py`（不并行第二套 MoveIt）时仍可复现。

- 案例 2：
  - 现象： [待补充]
  - 根因： [待补充]
  - 处置： [待补充]
  - 复现条件： [待补充]

## 8. 填空区：后续改进候选

- 候选改进 1： [待补充]
- 候选改进 2： [待补充]
- 候选改进 3： [待补充]

## 9. 维护约定

- 更新时机：每次新增稳定经验、修复关键问题、变更启动参数后。
- 更新方式：优先补充“已确认事实”与“标准运行流程”，再补充故障案例。
- 目标：让新会话仅凭本文件即可恢复 80% 以上上下文。

## 10. 每次任务强制预检（新增）

### 10.1 docs 必读顺序（每次会话开始先读）

- docs/ENV_PROJECT_KNOWLEDGE.md
- docs/TASK_RECORD.md
- docs/ARCHITECTURE.md

### 10.2 常见低级错误清单（必须逐条自检）

- 未 source 工作区环境：每次新终端执行 ROS 命令前先 `source /opt/ros/humble/setup.bash` 与 `source ros2_ws/install/setup.bash`。
- 当前目录错误：执行脚本前确认 cwd 与命令预期一致，避免相对路径误指向。
- 日志窗口不足：运行测试时固定使用落盘日志（例如 `/tmp/*.log`）并查看关键尾段，禁止只看终端可见窗口。
- 未清理上次进程：每次新实验前先清理旧的 `move_group`、`mtc_pick_place_demo`、`controller_manager`、`rviz2`、仿真脚本。
- 重复栈并行：禁止并行启动两套 MoveIt/MTC 栈，以免出现污染性假故障。

### 10.3 测试执行纪律

- 一次实验一个变量：每次仅改一个关键因素（参数或代码）并记录结果。
- 每次实验都要记录：命令、日志路径、结论、下一步假设。
- 失败结论必须有证据：至少包含一条日志关键行或服务快照结果。
