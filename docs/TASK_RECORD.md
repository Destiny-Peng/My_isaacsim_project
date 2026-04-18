# TASK RECORD

## 0. Purpose

This file tracks current work status, completed changes, pending tasks, and handover links.
It is intended for future agents to quickly resume work without re-discovery.

## 1. Current Progress Snapshot

### Completed

1. MoveIt controller chain stabilization
   - Added launch knobs for spawn delay / timeout / control clock mode.
   - Reduced startup race around `/controller_manager`.

2. MTC pick and place integration
   - Added `moveit_mtc_pick_place_demo` package.
   - Implemented tutorial-style MTC stages in C++.
   - Added launch-level parameter overrides for pick/place/object size.
   - Added MTC RViz preset and integrated it as default in MTC launch.
   - Added arm-first debug mode (`enable_gripper_actions:=false`) for stable end-to-end bring-up.

3. robot_description_semantic startup fix
   - MTC launch now injects full MoveIt description parameters directly.
   - Added `mtc_start_delay` to reduce startup race.

4. IsaacSim side object sync (latest)
   - `run_combined_car_franka_headless.py` now supports `--sync-mtc-demo-object`.
   - Demo cube can be created/updated in simulation with pose/size/color arguments.

5. Documentation refresh
   - Updated project-level README and workspace README.
   - Updated architecture document to reflect actual packages and data flow.

6. Hand/joint controller recovery (high-priority fix)
   - `spawn_aux_controllers` default switched to `true` in MoveIt and MTC launch paths.
   - Fixed `hand_controller` config key (`joint` instead of invalid `joints`) so gripper controller can configure and activate.

7. MTC RViz panel loading stabilization
   - Added explicit runtime dependency on `moveit_task_constructor_visualization`.
   - Injected RViz plugin-related env in MTC launch to avoid panel class lookup failures.

8. Isaac bridge motion smoothing (anti-teleport)
   - Python bridge now supports gradual position-command application with velocity-based per-step clamp.
   - Added CLI knobs: `--disable-position-command-smoothing`, `--default-max-joint-velocity`, `--max-joint-step-rad`.

9. MTC grasp-stage collision debugging round
   - Added grasp pose tunables (`grasp_z_offset`, `grasp_rpy`, `grasp_angle_delta`, `grasp_max_ik_solutions`).
   - Switched CollisionObject pick pose semantics to center-based to align with IsaacSim demo cube center semantics.

### In Progress

1. End-to-end validation of MTC pick/place with synced Isaac object in one command sequence.
2. Consistency checks between MTC CollisionObject pose and IsaacSim cube pose for varied test cases.
3. Resolve current OMPL `Invalid start state` failure after grasp-stage collision bottleneck was removed.

### Latest Debug Notes

1. `robot_description_semantic` crash path has been fixed via explicit launch injection.
2. Hand group remains non-chain; default debug mode now avoids mandatory hand actions.
3. If IsaacSim bridge is not running first, `arm_controller` spawner may fail in MTC launch.
4. 2026-04-01 smoke run (`launch_mtc_rviz:=false`) no longer hits semantic parameter crash.
5. Current failing stage is `grasp pose IK`: `eef in collision: demo_cube - panda_panda_hand`.
6. `arm_controller` still loads and becomes active in the same run, so controller startup is no longer the primary blocker.
7. Subagent meeting consensus:
   - Teleport effect source: bridge wrote `/joint_commands` directly via `set_joint_positions()` without interpolation.
   - Grasp IK root cause: grasp target sampled too close to object center + stage/property wiring issues.
8. After applying grasp-target and stage fixes, failure signature moved from `eef in collision` to planner-side `Invalid start state` retries.
9. This indicates the original grasp-collision bottleneck was mitigated, but start-state validity (scene/trajectory preconditions) still needs final convergence.

### Expected Next Steps

1. Add a single shared config source (optional) for MTC + Isaac object parameters.
2. Resolve `Invalid start state` by tightening pre-grasp approach preconditions and default pick pose envelope.
3. Add a lightweight validation script for:
   - topic health,
   - controller state,
   - object sync consistency.

## 2. Key Decisions

1. Collision responsibilities are split by design:
   - MoveIt/MTC: planning-time collision checks.
   - IsaacSim: execution-time physics contacts.

2. Reliable behavior requires dual-scene consistency:
   - object in MoveIt PlanningScene,
   - same object in IsaacSim scene.

3. MTC launch should own its RViz profile to avoid manual panel setup each run.

## 3. Handover Map (Read First)

- Project root overview: `../README.md`
- Architecture and component boundaries: `ARCHITECTURE.md`
- ROS2 workspace package map: `../ros2_ws/README.md`
- IsaacSim launcher and bridge guide: `../sim/launch/README.md`
- MTC demo usage/details: `../ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
- MoveIt launch stabilization record: `../ros2_ws/src/moveit_robot_config/MOVEIT_ISAAC_INTEGRATION_CHANGELOG.md`

## 4. Changed Files (Recent)

- `sim/launch/run_combined_car_franka_headless.py`
- `sim/launch/README.md`
- `ros2_ws/src/moveit_robot_config/launch/isaac_sim_moveit.launch.py`
- `ros2_ws/src/moveit_mtc_pick_place_demo/launch/mtc_pick_place_demo.launch.py`
- `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp`
- `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_pick_place.rviz`
- `README.md`
- `ros2_ws/README.md`
- `docs/ARCHITECTURE.md`

## 5. Operational Notes

1. Clean up stale processes after each test run to avoid false diagnostics:

```bash
pkill -f 'run_combined_car_franka_headless.py' || true
pkill -f 'mtc_pick_place_demo' || true
pkill -f 'isaac_sim_moveit.launch.py' || true
pkill -f 'ros2_control_node' || true
pkill -f 'move_group' || true
pkill -f 'rviz2' || true
```

2. If SRDF-related error appears again, verify launch injects:
   - `robot_description`
   - `robot_description_semantic`
   - `robot_description_kinematics`

3. For stable startup sequence:
   - start IsaacSim bridge first,
   - then launch MoveIt/MTC.
