# MoveIt x Isaac Sim 联动改造记录

## 1. 背景与目标

本次改造的目标不是“偶尔能跑通”，而是把 MoveIt 与 Isaac Sim 的控制链路变成可复现、可稳定启动的流程。

核心问题来自三类不稳定因素：

1. 启动竞态：`controller_manager` 服务尚未就绪时，spawner 已开始调用服务。
2. 时间语义：控制栈直接使用仿真时钟时，易受 `/clock` 起始与回退影响。
3. 重复启动污染：测试结束未清理进程，导致后续出现重名节点、重复加载控制器、统计口径失真。

## 2. 具体改动（代码级）

主要修改文件：

- `launch/isaac_sim_moveit.launch.py`

### 2.1 新增可配置启动参数

新增参数用于把不稳定因素显式化：

1. `use_sim_time_for_control`（默认 `false`）
2. `controller_manager_timeout`（默认 `60.0`）
3. `controller_spawn_delay`（默认 `5.0`）
4. `spawn_aux_controllers`（默认 `false`）

原因：

- 把控制栈时钟和 MoveIt 规划时钟解耦，先保证控制器加载稳定。
- 为 spawner 提供更长等待窗口，减少服务未就绪导致的失败。
- 通过启动延迟规避 `ros2_control_node` 刚拉起时的竞态。
- 默认只保证主链路 `arm_controller`，把辅助控制器改为按需开启，降低噪声和干扰面。

### 2.2 spawner 调用方式增强

spawner 统一改为：

1. 显式指定 `--controller-manager /controller_manager`
2. 增加 `--controller-manager-timeout`
3. 增加 `--param-file <ros2_controllers.yaml>`

原因：

- 明确管理器地址，避免命名空间或默认值带来的歧义。
- 在较慢启动场景中提升容错。
- 强制参数来源一致，避免“控制器定义存在但实例化时找不到参数”的不一致行为。

### 2.3 使用 TimerAction 延迟启动控制器

对 `joint_state_broadcaster`、`arm_controller`、`hand_controller` 的 spawner 加入 `TimerAction(period=controller_spawn_delay)`。

原因：

- 把“节点启动”与“服务可用”之间的时间窗显式留出来，降低冷启动阶段失败概率。

### 2.4 辅助控制器条件化

`joint_state_broadcaster` 与 `hand_controller` 受 `spawn_aux_controllers` 条件控制；`arm_controller` 维持默认必启。

原因：

- 你当前的主验证路径是机械臂 7 轴轨迹执行，先收敛主路径可稳定运行。
- 手部链路问题（例如 hand kinematics 噪声）不应阻断 arm 规划与执行。

## 3. 验证结论（本轮）

按“先仿真桥接、后 MoveIt”顺序测试后：

1. `arm_controller` 可达到 `active` 状态。
2. 控制器可 claim `panda_joint1..7/position`。
3. `/arm_controller/follow_joint_trajectory` 与 `/move_action` 在线。

这表明 demo 主链路已经可闭环。

## 4. 为什么之前会看到 clock: 1 pub / 13 sub

这是因为存在未清理的历史 MoveIt 进程。它们仍在订阅 `/clock`，会抬高订阅计数并污染诊断结论。

这不是仿真桥单体的问题，而是“测试收尾未清理”的流程问题。

## 5. 测试收尾规范（必须执行）

每次测试结束后建议执行：

```bash
pkill -f 'run_combined_car_franka_headless.py' || true
pkill -f 'isaac_sim_moveit.launch.py' || true
pkill -f 'ros2_control_node' || true
pkill -f 'move_group' || true
pkill -f 'rviz2' || true
```

建议再次确认：

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

若仅保留系统基础节点而无 MoveIt/仿真节点，即可认为清理完成。

## 6. 对下一步 pick and place 的意义

本次改造提供了可复用的稳定底座：

1. 控制器加载路径已稳定。
2. 时钟与竞态风险可通过参数调优。
3. 测试流程形成“启动顺序 + 收尾清理”的闭环。

在此基础上再实现 pick and place，调试成本会明显降低。
