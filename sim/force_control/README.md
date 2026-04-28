## 4. Franka 阻抗控制示例（Standalone）

新增脚本：`sim/force_control/franka_force_control_standalone.py`

用途：

- 在 Isaac Sim 5.1.0 中直接对 Franka 前 7 个关节执行基于 Pinocchio 动力学的 model-based 关节空间阻抗控制。
- 控制律：`tau = M(q)·(Kp·Δq - Kd·dq) + C(q,dq)·dq + g(q)`，实时补偿科里奥利/离心力和重力。
- 启动后默认将当前姿态作为阻抗平衡点（也可通过参数给定目标位姿）。

关键参数：

| 参数                           | 默认值                          | 说明                                    |
| ------------------------------ | ------------------------------- | --------------------------------------- |
| `--headless`                   | false                           | 无界面运行                              |
| `--param-file`                 | (空)                            | YAML 参数文件路径（仅传此参数即可运行） |
| `--max-steps`                  | 0                               | 最大控制步数；0 表示一直运行到关闭      |
| `--kp`                         | 120,120,110,100,80,60,40        | 7 维关节刚度                            |
| `--damping-ratio`              | 1.0                             | 阻尼比，kd = 2·zeta·sqrt(kp)            |
| `--torque-limit`               | 80,80,80,60,40,30,20            | 7 维关节力矩限幅                        |
| `--q-target`                   | 0,-0.785,0,-2.356,0,1.571,0.785 | 7 维关节目标位置 (rad)                  |
| `--physics-dt`                 | 0.005                           | 物理步长 (s)                            |
| `--urdf-path`                  | auto                            | Franka URDF 路径（用于 Pinocchio）      |
| `--solver-position-iterations` | 32                              | PhysX 求解器位置迭代数                  |
| `--solver-velocity-iterations` | 2                               | PhysX 求解器速度迭代数                  |
| `--record`                     | false                           | 启用 CSV 记录 q/dq/tau_cmd/tau_applied  |
| `--recording-prefix`           | sim/outputs/force_control       | CSV 输出前缀                            |

示例命令：

```bash
# 方式 1：只传 param-file（推荐）
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/force_control/franka_force_control_standalone.py \
  --param-file sim/launch/config/sim_default.yaml

# 方式 2：param-file + CLI 覆盖（CLI 优先级更高）
python3 sim/force_control/franka_force_control_standalone.py \
  --param-file sim/launch/config/sim_default.yaml \
  --headless --max-steps 300 --record

# 方式 3：纯 CLI（兼容原用法）
python3 sim/force_control/franka_force_control_standalone.py \
  --headless \
  --max-steps 300 \
  --kp 120,120,110,100,80,60,40 \
  --damping-ratio 1.0 \
  --torque-limit 80,80,80,60,40,30,20 \
  --q-target 0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
```

YAML 配置（`sim/launch/config/sim_default.yaml` 的 `force_control` 区块）：

```yaml
force_control:
  urdf_path: ros2_ws/src/robot_description/urdf/arm_only_franka.urdf
  kp: [120.0, 120.0, 110.0, 100.0, 80.0, 60.0, 40.0]
  damping_ratio: 1.0
  torque_limit: [80.0, 80.0, 80.0, 60.0, 40.0, 30.0, 20.0]
  q_target: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  physics_dt: 0.005
  max_steps: 0
  headless: false
  solver_position_iterations: 32
  solver_velocity_iterations: 2
  recording_output_prefix: sim/outputs/force_control
```

CSV 记录字段：

`step`, `sim_time_s`, `wall_time_s`, `q1..q7`, `dq1..dq7`, `tau_cmd_1..tau_cmd_7`, `tau_applied_1..tau_applied_7`

说明：

- `simulation.extensions` 已包含 `omni.physx.ui`，启动时自动启用，可在 GUI 中对 Franka 施加外力观察力控效果。
- `physics_dt=0.005` + `solver_position_iterations=32` + `solver_velocity_iterations=2` 专门针对高刚度力控稳定性调整，非 KP 参数，不涉及控制器参数修改。
- 依赖 Pinocchio 机器人学库（v2.7.0，cmake-wheel/pinocchio），已通过 pip 安装。

兼容性说明：

- 继续保留原脚本路径 `sim/force_control/franka_force_control_standalone.py`。
- `--param-file` 为新增入口，未传时保持纯 CLI 用法不变。
