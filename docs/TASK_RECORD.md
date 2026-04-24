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

## 3. 近期验证结果

- `colcon build --packages-select moveit_mtc_pick_place_demo` 成功。
- `python3 ros2_ws/scripts/run_mtc_demo.py --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml --dry-run` 只展开 launch 参数，没有无效 `pick_place:=None`。
- 实际联调日志中可见：`MTC task pipeline initialized once and will be reused across service reruns` 与 `MTC pick and place execution completed successfully`。

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
