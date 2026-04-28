# TASK RECORD

## 1. 目标

记录当前项目可复用的修复结论，避免重复排障。

## 2. 已完成结论（压缩版）

- `mtc_pick_place_demo` 使用成员级 `Task`，pipeline 只初始化一次；后续 `run()` 复用同一个 task 对象和 cached solution。
- executor 重复注册崩溃已修复：不再在已 spin 的节点上做二次 `spin_some`。
- `/clock` 按 sim time 发布，Isaac 侧 bridge 与 MoveIt/MTC 统一启用 `use_sim_time=true`。
- Isaac 启动脚本不再运行时创建 ActionGraph、cube、table；只保留已有 prim 的位置同步与 reset 能力。
- `mtc_launch_defaults.yaml` 与 `pick_place_params.yaml` 已拆分为 launch-only 和 task-only 两类参数，避免重复入口。
- 外部 pick/place 话题采用一次性消费语义，旧消息不会持续触发重规划。
- 放置阶段碰撞时机已调整，避免回撤路径继续允许 hand 和 object 相交。
- RViz 配置已缩减为单个 `Motion Planning Tasks` 实例，避免同名面板/显示项让人误以为创建了两个 task。
- 新增 `sim/launch/grasp_metric_evaluator.py`：使用 Replicator annotator 记录对象/夹爪时序数据，并计算 success、stability、slippage 指标。
- `run_combined_car_franka_headless.py` 已集成 grasp evaluator：可在同一仿真试验中自动录制并输出 CSV/JSON 指标结果。
- 新增 `sim/force_control/franka_force_control_standalone.py`：基于 `isaacsim.*` 命名空间的 Franka 7-DoF 纯力矩控制模板，提供状态读取与力矩下发接口，可直接插入自定义控制律。
- `sim/force_control/franka_force_control_standalone.py` 已升级为关节空间阻抗控制示例（`tau = Kp(q_ref-q) + Kd(0-dq)`），支持 `--kp`、`--damping-ratio`、`--torque-limit`、`--q-target` 参数化调试。
- `sim/force_control/franka_force_control_standalone.py` 已重构为基于 Pinocchio 的 model-based 关节空间阻抗控制器：
  - 控制律：`τ = M(q)·(Kp·Δq - Kd·dq) + C(q,dq)·dq + g(q)`
  - 使用 URDF 通过 Pinocchio（v2.7.0）构建机器人模型，实时计算 M(q)、C(q,dq)、g(q)
  - 新增 CLI 参数：`--record`、`--urdf-path`、`--solver-position-iterations`、`--solver-velocity-iterations`
  - 新增 YAML 参数：`force_control` 区块中的 `urdf_path`、`solver_position_iterations`、`solver_velocity_iterations`、`recording_output_prefix`
- `sim/launch/config/sim_default.yaml` 已更新：
  - `simulation.extensions` 加入 `omni.physx.ui` 以支持运行时交互施力
  - `force_control` 区块加入 `solver_position_iterations: 32` 和 `solver_velocity_iterations: 2` 以增强力控稳定性
  - `force_control.physics_dt` 从 0.01 降至 0.005 进一步提升平滑度
  - 新增 `recording_output_prefix` 和 `urdf_path` 配置项
- 新增 `JointDataRecorder` 类：以增量 flush 方式（每 100 行写入一次 CSV）记录 q/dq/tau_cmd/tau_applied 关节数据，用于力控效果诊断。CSV 输出示例：`sim/outputs/force_control_20260428_161538.csv`（300 步测试含表头 + 300 行数据）。
- 移除伪包 `pinocchio`（0.4.3，被 nose 框架占位），安装真正 Pinocchio 库 `pin==2.7.0`（来自 cmake-wheel/pinocchio）。

## 3. 近期验证结果

- `colcon build --packages-select moveit_mtc_pick_place_demo` 成功。
- `python3 ros2_ws/scripts/run_mtc_demo.py --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml --dry-run` 只展开 launch 参数，没有无效 `pick_place:=None`。
- 实际联调日志中可见：`MTC task pipeline initialized once and will be reused across service reruns` 与 `MTC pick and place execution completed successfully`。
- `python3 sim/force_control/franka_force_control_standalone.py --headless --max-steps 300` 验证通过，日志包含 `Reached --max-steps=300, exiting.`，无 traceback。
- `python3 sim/force_control/franka_force_control_standalone.py --param-file sim/launch/config/sim_default.yaml --headless --max-steps 300 --record` 验证通过：
  - Pinocchio 模型加载成功（`arm_only_franka.urdf`，nq=9, nv=9）
  - 物理求解器迭代数成功设置为 pos=32, vel=2
  - `omni.physx.ui` 扩展成功启用
  - CSV 记录：`sim/outputs/force_control_20260428_161538.csv`（301 行 = 1 表头 + 300 数据）
  - 日志包含 `Reached max-steps=300, exiting.`，Simulation App 正常关闭，无 Traceback
- `pinocchio` 伪包（0.4.3）被移除，真实 Pinocchio（v2.7.0, cmake-wheel/pinocchio）正常工作

## 4. 当前状态

- 当前代码侧没有构造两次 `Task` 的路径；用户看到的“双 task”更接近 RViz 配置中的重复实例，而不是 C++ 构造函数重复调用。
- 本轮已做一次收敛性清理，后续如果继续扩展，只应在明确的新需求下再动参数或场景语义。

## 5. 关键文件

- `sim/launch/run_combined_car_franka_headless.py`
- `sim/launch/config/sim_default.yaml`
- `sim/launch/README.md`
- `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_pick_place.rviz`
- `ros2_ws/src/moveit_mtc_pick_place_demo/launch/mtc_pick_place_demo.launch.py`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`
- `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
