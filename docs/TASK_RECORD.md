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
