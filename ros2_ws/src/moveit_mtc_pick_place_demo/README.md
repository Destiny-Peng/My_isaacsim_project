# moveit_mtc_pick_place_demo

## 1. 包概述

`moveit_mtc_pick_place_demo` 是本仓库的 MTC 任务示例包，用于验证 `mobile_base_with_franka` 的抓放任务流程与可视化。

组成部分：

- `src/mtc_pick_place_demo.cpp`: 任务 stage 定义与执行。
- `launch/mtc_pick_place_demo.launch.py`: MTC + MoveIt 启动编排。
- `config/pick_place_params.yaml`: 任务参数（抓取目标、偏置、距离）。
- `config/mtc_pick_place.rviz`: MTC 面板配置。

## 2. 如何启动测试

步骤 1：先启动仿真

```bash
cd /home/jacy/project/isaac_test/isaac_sim_fullstack
python3 sim/launch/run_combined_car_franka_headless.py \
  --param-file sim/launch/config/sim_default.yaml
```

步骤 2：启动 MTC demo（参数文件方式）

```bash
python3 ros2_ws/scripts/run_mtc_demo.py \
  --param-file ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml
```

步骤 3：若只想无界面运行，可在 YAML 中将 `launch_mtc_rviz: false`。

## 3. 参数说明

分两类参数文件：

- 启动参数：`config/mtc_launch_defaults.yaml`
  - 控制 `mtc_start_delay`、`spawn_aux_controllers`、`launch_mtc_rviz`、pick/place 标量覆盖。
- 任务参数：`config/pick_place_params.yaml`
  - 控制对象尺寸、抓取偏置与采样参数：
    - `grasp_z_offset`
    - `grasp_rpy`
    - `grasp_angle_delta`
    - `grasp_max_ik_solutions`

建议：

- 保持 `spawn_aux_controllers: true`，确保夹爪控制链可用。
- 保持 IsaacSim demo cube 与 MTC object 参数一致，减少场景不一致导致的失败。
