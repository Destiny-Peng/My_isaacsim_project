# TASK RECORD

## 1. 目标

记录当前项目可复用的修复结论，避免重复排障。

## 2. 已完成里程碑（合并版）

### A. 基础联动与启动稳定化

- MoveIt 启动链路稳定：
  - 控制器延迟启动与超时参数可配。
  - `hand_controller` 配置修正并可激活。
- MTC 与 MoveIt 启动编排收敛：
  - MTC launch 显式注入 `robot_description*`。
  - RViz/MTC 面板加载依赖补齐。

### B. MTC 任务生命周期重构

- `mtc_pick_place_demo` 改为成员级 `Task` 管理：pipeline 初始化一次。
- 引入成员级 cached solution：
  - 首次 plan 后缓存。
  - 后续 run 可直接复用 execute，不重复规划。
- reset 语义：
  - `reset_task_state` / `reset_and_run` 清理缓存 + 重置 planning scene。
- 清理了 `enable_*` 分支开关与多处硬编码，参数集中到 YAML/launch。

### C. Isaac 侧场景一致性与 reset 修复

- demo cube / support surface 同步到 Isaac 场景。
- reset 服务完善：
  - cube reset 同时恢复位姿并清零线速度/角速度。
  - 支持 robot reset 与 scene reset。
- support surface 与 cube 坐标/尺寸语义对齐，避免 sim 与 MTC 失配。

### D. 纯物理抓取增强（取消 attach trick）

- 移除 attachment emulator，改为纯物理接触抓取。
- cube/table 绑定 physics material（摩擦、弹性可配）。
- cube 支持 dynamic rigid body + mass + CCD。

### E. 文档与参数文件标准化

- 统一 `--param-file` 启动流程（sim/MoveIt/MTC）。
- 各包 README 已按“概述-启动-参数”结构整理。

## 3. 2026-04-22 本轮增量

### 3.1 时钟链路修复

- 根因：默认 profile 仍为 `clock_mode: wall-anchored`，导致 `/clock` 不是纯 sim time。
- 修复：
  - `sim/launch/config/sim_default.yaml` 改为 `clock_mode: sim`。
  - `run_combined_car_franka_headless.py` 的 `--clock-mode` 默认改为 `sim`。
  - MoveIt/MTC 默认 `use_sim_time_for_control` 统一改为 `true`。

### 3.2 抓取物理稳定性补强

- demo cube 新增可调参数并应用到 PhysX：
  - `demo_object_linear_damping`
  - `demo_object_angular_damping`
  - `demo_object_contact_offset`
  - `demo_object_rest_offset`
  - `demo_object_solver_position_iterations`
  - `demo_object_solver_velocity_iterations`
- 目标：减少夹取时穿透、弹飞、滑脱。

### 3.3 MTC 代码冗余清理

- 删除无效赋值与启动硬等待，保留任务复用语义不变。

### 3.4 MTC 运行时崩溃修复（executor 重复注册）

- 根因：`waitForJointState()` 内调用 `rclcpp::spin_some(shared_from_this())`，节点已在 `rclcpp::spin(node)` 的 executor 中，导致抛出：
  - `Node '/mtc_pick_place_demo' has already been added to an executor.`
- 修复：移除该处 `spin_some`，改为仅等待订阅回调置位 `joint_state_received_`。
- 验证命令：
  - `python3 ros2_ws/scripts/run_mtc_demo.py --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`
- 结果：未再出现 executor 重复注册异常；当前失败转为控制器链路未就绪（`/controller_manager` 服务不可达），与本修复无关。

### 3.5 Isaac 运行脚本改造（仅保留位置接口）

- 背景：用户在 Isaac Sim 手动打开 USD 并手动配置 graph/cube 后可正常夹取，运行脚本重复创建会扰动场景。
- 修复：
  - 删除运行时 ActionGraph 自动创建逻辑。
  - 删除运行时创建/更新 cube 与 support surface 几何/物理属性逻辑。
  - `sync_mtc_demo_object` / `sync_demo_support_surface` 改为仅设置现有 prim 位置。
  - 默认 prim 改为：`/Cube`、`/Plane`。
- 变更文件：
  - `sim/launch/run_combined_car_franka_headless.py`
  - `sim/launch/config/sim_default.yaml`

### 3.6 MTC 无效兜底参数清理

- 清理目标：移除“误判 execute 导致问题”阶段留下的历史兜底逻辑，减少误导与配置噪声。
- 本次删除：
  - C++ 参数与分支：`keep_alive_after_execute_failure`
  - YAML 无效键：`keep_alive_after_execute_failure`、`auto_run_on_start`、`place_retreat_distance`
- 说明：以上参数当前代码路径均不再需要，保留会造成“可调但不应再调”的认知负担。

### 3.7 夹爪/环境碰撞问题可能原因（未验证）

- 结论类型：静态代码分析推测，尚未做实验验证。
- 可能原因 1：抓取阶段显式放开手与物体碰撞
  - `allow collision (hand,object)` stage 会放开 hand group 的碰撞约束；如果 hand group 包含手指链路，规划会允许指-物体穿碰。
- 可能原因 2：`grasp_ik_ignore_collisions=true` 进一步放松 IK 碰撞检查
  - 当前参数文件启用了该开关，可能在抓取位姿生成时放大“可行但贴碰/穿碰”的解。
- 可能原因 3：`additional_allowed_collision_links` 扩大了允许碰撞链路
  - 目前含 `panda_panda_link6`、`panda_panda_link7`，会增加手部附近近碰通过率。
- 可能原因 4：MoveIt 规划场景与 Isaac 实体几何不一致
  - MoveIt 里是参数化 box（`demo_cube`/`demo_table`），Isaac 中是 `/Cube`、`/Plane`；若中心点、厚度、坐标系存在偏差，会出现“规划看似无碰撞，仿真实碰撞”。
- 可能原因 5：SRDF/URDF 中手指碰撞几何或禁碰配置导致约束过宽
  - 若手指 collision mesh 过粗/过小，或在 SRDF 被禁碰，会出现手指穿过 cube/plane 的表观问题。
- 可能原因 6：抓取与下放阶段的路径容差较松
  - `approach/lower` 段使用笛卡尔局部规划，若步长/最小距离设置过于激进，易贴边通过。

建议后续验证顺序（仅计划）：

1. 先关掉 `grasp_ik_ignore_collisions`，观察是否显著改善。
2. 再收紧 `additional_allowed_collision_links`（仅保留必要链路）。
3. 最后对齐 MoveIt 与 Isaac 的 cube/plane 位姿与尺寸语义。

### 3.8 启动参数简化（清理无效项）

- 问题：脚本中保留了大量“已移除功能”的参数，且 `gui`/`headless` 语义容易混淆。
- 修复：
  - 删除了与运行时创建物理体相关的无效参数（size/color/dynamic/mass/friction/damping/contact/solver 等）。
  - 默认 YAML 仅保留仍被脚本消费的字段。
  - 增加旧字段兼容：若 YAML 中存在 `gui` 且未设置 `headless`，自动映射为 `headless = not gui` 并输出弃用告警。

### 3.9 启动脚本参数入口进一步收敛 + /clock 修复

- 问题：同一配置同时存在 YAML 键与 `--` 参数，阅读和维护成本高；`/clock` 在 CLI 观察中出现连续丢包提示。
- 修复：
  - 删除了以下重复 CLI 参数，改为仅从 YAML 读取：
    - `--demo-object-prim`
    - `--demo-object-pick-position`
    - `--support-surface-prim`
    - `--support-surface-center`
    - `--articulation-prim`
  - `/clock` publisher 改为显式 clock QoS（BEST_EFFORT + KEEP_LAST + VOLATILE），兼容 Isaac 内置 rclpy。
  - 默认 `publish_every_steps` 从 `1` 调整为 `4`，降低高频 CLI 订阅时的丢包提示概率。

### 3.10 MTC 流程优化：去冗余等待 + 动态起终点 + 速度均匀化

- 问题 1：执行前 `waitForExecutionActionServers` 属于冗余等待。
  - 现状：若 action server 不可用，`task.execute()` 本身会失败并给出错误码。
  - 修复：删除执行前 action server 等待逻辑，直接进入 execute，缩短链路并保留明确报错。

- 问题 2：pick/place 起终点固定，不利于后续接入外部消息驱动。
  - 修复：新增可选 topic 输入（`geometry_msgs/msg/PoseStamped`）：
    - `pick_pose_topic`（默认 `/mtc_pick_place_demo/pick_pose`）
    - `place_pose_topic`（默认 `/mtc_pick_place_demo/place_pose`）
    - 开关：`use_external_pick_pose`、`use_external_place_pose`
  - 行为：run 前读取最新目标；若目标变化，自动清空 cached solution 并重建 pipeline，保证新目标生效。
  - 约束：`frame_id` 需为空或等于 `world_frame`，否则忽略该消息。

- 问题 3：执行速度段间差异较大，存在抛物风险。
  - 修复：调整默认参数（`pick_place_params.yaml`）：
    - `plan_velocity_scaling: 0.06`
    - `plan_acceleration_scaling: 0.03`
    - `approach_min_distance: 0.02`
    - `lift_min_distance: 0.03`
    - `place_lower_min_distance: 0.03`
    - `cartesian_step_size: 0.003`

- 变更文件：
  - `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp`
  - `ros2_ws/src/moveit_mtc_pick_place_demo/config/pick_place_params.yaml`
  - `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`

- 验证命令（本轮）：
  - `cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws`
  - `source /opt/ros/humble/setup.bash`
  - `colcon build --packages-select moveit_mtc_pick_place_demo`
  - `cd /home/jacy/project/isaac_test/isaac_sim_fullstack`
  - `python3 ros2_ws/scripts/run_mtc_demo.py --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`

## 4. 当前状态

- 编译状态：待本轮改动后重新验证。
- 主风险：端到端抓取稳定性仍需在最新 sim-time + 新物理参数下复测。

## 5. 下一步验证清单

1. 启动仿真后检查 `/clock` 单调递增且与 timeline 同步。
2. 启动 MoveIt/MTC，抽查关键节点 `use_sim_time` 为 `true`。
3. 连续执行至少 3 次抓取，观察是否仍出现明显穿透/打滑。
4. 如失败，记录日志与参数快照并只改一个变量继续实验。

## 6. 关键文件

- `sim/launch/run_combined_car_franka_headless.py`
- `sim/launch/config/sim_default.yaml`
- `ros2_ws/src/moveit_robot_config/launch/isaac_sim_moveit.launch.py`
- `ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp`
- `ros2_ws/src/moveit_mtc_pick_place_demo/launch/mtc_pick_place_demo.launch.py`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`
