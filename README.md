# Isaac Sim Fullstack

当前仓库用于 `Isaac Sim + ROS2 + MoveIt2 + MTC` 联动验证，核心目标是跑通并稳定维护 `mobile_base_with_franka` 的规划与仿真执行链路。

## 1. 当前目录结构

- `sim/`: Isaac Sim 启动脚本与联调文档。
- `ros2_ws/`: ROS2 工作区（MoveIt 配置、MTC demo、robot_description、MTC 源码）。
- `docs/`: 架构说明与任务记录文档。

## 2. 快速入口

### 启动 Isaac Sim 桥接（含 MTC 物体同步）

```bash
python /home/jacy/project/isaac_test/isaac_sim_fullstack/sim/launch/run_combined_car_franka_headless.py \
	--headless \
	--launcher-backend isaacsim \
	--setup-ros2-python-bridge \
	--sync-mtc-demo-object \
	--clock-mode wall-anchored \
	--steps 20000 \
	--render-interval 2000
```

### 启动 MoveIt + MTC demo

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch moveit_mtc_pick_place_demo mtc_pick_place_demo.launch.py
```

## 3. 文档导航

- 仿真与桥接说明: `sim/launch/README.md`
- ROS2 workspace 说明: `ros2_ws/README.md`
- MTC demo 说明: `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
- 架构文档: `docs/ARCHITECTURE.md`
- 任务记录（持续更新）: `docs/TASK_RECORD.md`

## 4. 当前状态摘要

- MoveIt 控制链稳定化改造已完成（controller 启停时序、超时、时钟解耦）。
- MTC pick and place demo 已接入并可构建。
- Isaac Sim 侧已支持创建/更新 demo 立方体，用于与 MoveIt PlanningScene 对齐。
