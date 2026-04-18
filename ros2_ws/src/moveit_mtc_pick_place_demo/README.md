# MTC Pick & Place Demo

这个包按 MoveIt Task Constructor (MTC) 官方教程风格组织，给 `mobile_base_with_franka` 提供一个最小可运行的 pick & place 样例。

## 1. 先启动 Isaac Sim 桥接

```bash
python \
/home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
--headless \
--launcher-backend isaacsim \
--setup-ros2-python-bridge \
--sync-mtc-demo-object \
--clock-mode wall-anchored \
--steps 20000 \
--render-interval 2000
```

说明：`--sync-mtc-demo-object` 会在 IsaacSim 中创建/更新 `demo_cube`，用于和 MTC 侧 CollisionObject 对齐。

## 2. 编译

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
source /opt/ros/humble/setup.bash
colcon build \
   --base-paths /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/src \
   --build-base /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/build \
   --install-base /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws/install \
   --packages-up-to moveit_mtc_pick_place_demo
source install/setup.bash
```

## 3. 运行 MTC demo

```bash
ros2 launch moveit_mtc_pick_place_demo mtc_pick_place_demo.launch.py
```

默认会启动一份带 `Motion Planning Tasks` 面板的 RViz（配置文件：`config/mtc_pick_place.rviz`）。

默认采用 `arm-first` 联动模式：

- `enable_gripper_actions:=false`
- `spawn_aux_controllers:=false`

这样可以避免当前 `hand` 组链路限制导致的初始化/控制器失败，先保证主链路稳定。

如果你只想跑无界面，可关闭：

```bash
ros2 launch moveit_mtc_pick_place_demo mtc_pick_place_demo.launch.py launch_mtc_rviz:=false
```

也可以在 launch 命令中直接覆盖 pick/place 位姿与物体尺寸：

```bash
ros2 launch moveit_mtc_pick_place_demo mtc_pick_place_demo.launch.py \
   pick_x:=0.58 pick_y:=0.02 pick_z:=0.20 \
   place_x:=0.42 place_y:=-0.28 place_z:=0.20 \
   object_size_x:=0.04 object_size_y:=0.04 object_size_z:=0.10
```

若你要启用手爪动作（实验性，要求 hand 控制器可用）：

```bash
ros2 launch moveit_mtc_pick_place_demo mtc_pick_place_demo.launch.py \
   enable_gripper_actions:=true \
   spawn_aux_controllers:=true
```

若要同步 IsaacSim 中的 cube 位置/尺寸，也请在 Isaac 脚本命令中同时覆盖：

```bash
python /home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
   --headless \
   --launcher-backend isaacsim \
   --setup-ros2-python-bridge \
   --sync-mtc-demo-object \
   --demo-object-pick-position 0.58,0.02,0.20 \
   --demo-object-size 0.04,0.04,0.10 \
   --clock-mode wall-anchored \
   --steps 20000 \
   --render-interval 2000
```

## 4. 关键流程（对应教程 stage）

1. CurrentState
2. MoveTo (open hand)
3. Connect (move to pick)
4. SerialContainer pick:
   - MoveRelative (approach)
   - GenerateGraspPose + ComputeIK
   - ModifyPlanningScene (allow collision)
   - MoveTo (close hand)
   - ModifyPlanningScene (attach)
   - MoveRelative (lift)
5. Connect (move to place)
6. SerialContainer place:
   - GeneratePlacePose + ComputeIK
   - MoveTo (open hand)
   - ModifyPlanningScene (forbid collision)
   - ModifyPlanningScene (detach)
   - MoveRelative (retreat)
7. MoveTo (home)

## 5. 参数

在 `config/pick_place_params.yaml` 中调整：

- pick/place 位置
- 物体尺寸
- approach/lift/retreat 距离
- group、frame、对象名

此外，`mtc_pick_place_demo.launch.py` 也支持以下标量参数（便于命令行临时调参）：

- pick_x / pick_y / pick_z
- place_x / place_y / place_z
- object_size_x / object_size_y / object_size_z

## 6. MTC 可视化（Introspection）

本 demo 在成功规划后会调用 `task.introspection().publishSolution(...)`，用于在 RViz 里查看 stage 树和 solution。

默认配置已包含 MTC 面板。如果你改用自己的 RViz 配置且没有面板，可手动添加：

1. Panels -> Add New Panel
2. 选择 `Motion Planning Tasks`（MTC 插件）

添加后可以看到 task 的 stage 分解、候选解与执行状态，便于对照官方教程学习。

## 7. 关于碰撞：MoveIt 与 Isaac Sim 的分工

你的理解是对的：

- MoveIt/MTC 负责规划阶段的碰撞检查（PlanningScene）。
- Isaac Sim 负责执行阶段的物理仿真与接触响应。

因此，`target` 物体应当在 Isaac Sim 中也存在，执行时才会有真实接触/碰撞行为。

同时，为了让规划结果可信，MoveIt 侧也需要有对应的碰撞物体（通常是同 pose/尺寸的 CollisionObject）。

最实用的做法是保持“双场景一致”：

1. 在 Isaac Sim 场景里放置真实物体（USD 里可见）。
2. 在 MTC 中向 PlanningScene 添加同名/同尺寸/同位姿的 CollisionObject。
3. 每次改动物体位姿时，同步更新两侧参数。

> 当前 demo 已在 MTC 中添加了 `demo_cube` CollisionObject；建议你在 USD 里放置同参数立方体与其对齐。

## 8. robot_description_semantic 报错说明

若出现 `Could not find parameter robot_description_semantic`，通常是节点启动时未拿到 MoveIt 描述参数。

本仓库当前版本已在 launch 中显式注入：

- `robot_description`
- `robot_description_semantic`
- `robot_description_kinematics`

并为 MTC 节点增加了 `mtc_start_delay`（默认 `2.0s`）降低初始化竞态。

## 9. 当前联调状态

- MTC package 可构建通过。
- MTC launch 可启动（含 MTC RViz 面板）。
- 已修复 `robot_description_semantic` 丢失导致的崩溃问题。
- 已支持 IsaacSim 对应物体同步（demo cube）。
- 默认 `arm-first` 模式下可绕过 `hand` 组限制并完成主链路联调。

最近一次短时冒烟（`launch_mtc_rviz:=false`）结论：

- `arm_controller` 可以正常加载并激活。
- MTC 失败点已集中在抓取阶段 IK：
   - `grasp pose IK (0/16): eef in collision: demo_cube - panda_panda_hand`

这说明当前主阻塞是抓取位姿/碰撞几何约束，而不是语义参数缺失或控制器初始化失败。

建议下一步先在不改架构的前提下做最小调参：

1. 下调 `pick_z` 或减小 `object_size_z`，降低初始手爪-物体重叠概率。
2. 调整 `approach_distance` 与 grasp pose（例如在 object frame 增加更明确的抓取偏置）。
3. 保持 IsaacSim 与 MTC 的 object pose/size 完全一致后再复测。

联调注意：如果未先启动 IsaacSim bridge，`arm_controller` 的 spawner 可能失败，表现为 launch 中 controller 进程退出。请严格按顺序执行：

1. IsaacSim 脚本先启动。
2. 再启动 `mtc_pick_place_demo.launch.py`。

## 10. 测试后清理

```bash
pkill -f 'run_combined_car_franka_headless.py' || true
pkill -f 'mtc_pick_place_demo' || true
pkill -f 'isaac_sim_moveit.launch.py' || true
pkill -f 'ros2_control_node' || true
pkill -f 'move_group' || true
pkill -f 'rviz2' || true
```
