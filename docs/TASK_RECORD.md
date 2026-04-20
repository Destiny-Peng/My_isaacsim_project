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

10. Package-level README + param-file standardization (latest)
    - Rewritten all repository README files to a unified 3-part structure:
       - package overview,
       - runnable startup steps,
       - parameter reference.
    - Added param-file workflow to reduce long CLI command errors:
       - IsaacSim launcher supports `--param-file` YAML defaults.
       - Added default YAML files for sim, MoveIt, and MTC launch paths.
       - Added short Python wrapper entrypoints for MoveIt and MTC using `--param-file`.
    - Per-package completion record:
       - root: `README.md` rewritten.
       - sim launcher package: `sim/launch/README.md` rewritten; `sim_default.yaml` added.
       - ROS2 workspace overview: `ros2_ws/README.md` rewritten.
       - `moveit_robot_config`: README added + launch defaults YAML added.
       - `moveit_mtc_pick_place_demo`: README rewritten + launch defaults YAML added.
       - `robot_description`: README rewritten.
       - `moveit_task_constructor` (workspace vendored package): top-level, demo, and scope_guard README rewritten for this project usage context.

11. Context persistence document bootstrap (latest)
    - Added `docs/ENV_PROJECT_KNOWLEDGE.md` as a persistent knowledge template.
    - Recorded confirmed environment constraints:
       - alias `isaac='conda activate env_issaclab'`
       - `env_issaclab` is IsaacSim-only; non-Isaac project tasks should stay on default/system environment.
    - Left fill-in placeholders for user-provided facts (env variables, validation criteria, troubleshooting cases).

12. MTC node lifecycle aligned to official keep-alive behavior (new)
   - Updated `mtc_pick_place_demo.cpp` main lifecycle to use background spinning thread and `spinning_thread.join()`.
   - Removed immediate `rclcpp::shutdown()` after `node->run()` to keep introspection topics alive for RViz after planning.
   - Behavior now matches official demo intent: planning can finish while node remains alive for stage visualization.
13. MTC task lifetime fix for RViz stage/solution browsing (new)
   - Root cause identified: `mtc::Task` was a local variable inside `run()` and got destroyed when `run()` returned.
   - Refactored `mtc_pick_place_demo.cpp` to store task as class member (`std::unique_ptr<mtc::Task> task_`).
   - This preserves full task tree and candidate solutions in RViz after successful plan, matching official demo behavior.

### In Progress

1. End-to-end validation of MTC pick/place with synced Isaac object in one command sequence.
2. Consistency checks between MTC CollisionObject pose and IsaacSim cube pose for varied test cases.
3. Resolve current OMPL `Invalid start state` / grasp collision ping-pong around approach<->grasp transition.
4. Runtime validation of IsaacSim launcher in a fully provisioned IsaacLab environment (`isaaclab` module unavailable in current shell).

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
10. New observation (this round): running `mtc_pick_place_demo.launch.py` while another MoveIt stack is already running introduces stack contention/noisy failures; isolate to `sim + mtc launch` only.
11. New code-level changes (this round):
   - Added `grasp_ik_ignore_collisions` parameter (default false in code; tuned in YAML during tests).
   - Added `grasp_x_offset` / `grasp_y_offset` parameters for grasp pose center offset.
   - Added `additional_allowed_collision_links` parameter for selective collision relaxation.
   - Moved `allow hand-object collision` stage to run after `grasp pose IK`.
12. Current iteration status:
   - with strict collision settings: failure at `grasp pose IK` (`demo_cube - panda_panda_hand`).
   - with relaxed collision settings: solver can produce grasp candidates, but may regress to `approach object` / `Invalid start state` depending on profile.
13. 2026-04-18 deep-dive updates (new):
   - Added startup guard in `mtc_pick_place_demo.cpp`: wait for at least one `/joint_states` message before creating the task.
   - Added richer plan-failure instrumentation (`task.explainFailure(...)`) to improve stage-level diagnostics when plan returns.
   - Verified via logs that MTC now prints `Received joint state from topic '/joint_states'` before task construction.
   - Despite the guard, OMPL still repeatedly reports `Skipping invalid start state` / `Invalid start state`.
   - Live planning-scene snapshot still reports robot state positions as all zeros, matching observed invalid-start loop.
   - Cross-check of `/joint_states` in this run also showed all arm joints at zero, so the system currently enters planning from a zero configuration profile.
   - Quick service validation confirms a manually provided zero joint state can be `valid=True`; therefore the invalid-start loop is likely request/context-specific (not a simple static-collision-only failure).
14. 2026-04-18 baseline-alignment updates (new):
    - Reworked `mtc_pick_place_demo.cpp` to align task skeleton with official `moveit_task_constructor/demo` flow:
       - monitor grasp generation from `open hand` stage output,
       - monitor place generation from `pick object` container output,
       - use CartesianPath for approach/lift/lower/retreat motions,
       - align approach/retreat direction conventions with official demo.
    - Kept project-specific URDF/group/object parameters while removing non-essential deviations from official stage ordering.
    - Updated default params to reduce non-baseline noise:
       - `enable_gripper_actions` default enabled,
       - `approach_min_distance` / `approach_distance` restored to non-zero values,
       - `grasp_ik_ignore_collisions` default restored to false.
15. Process-discipline updates (new):
    - Added mandatory pre-flight checklist in `ENV_PROJECT_KNOWLEDGE.md`:
       - must-read docs order,
       - source/cwd/log-window/process-cleanup checks,
       - one-variable-per-test logging discipline.
16. Official-demo alignment debug progression (new):
   - Initial officialized run failed at `open hand` with `GOAL_STATE_INVALID` under current URDF.
   - Added URDF-adaptation switch: `enable_initial_open_hand_stage` (default false), with monitored stage fallback to `current state`.
   - Next failure shifted to grasp/pick pipeline and then to `approach object: missing ik_frame`; fixed by explicitly setting IK frame on approach stage.
   - Current status: task now gets significantly deeper into planning tree before timeout window, with no regression to earlier `Invalid start state` as first-order blocker in this branch.
17. Pick-only interface-chain fix and validation (new):
   - Root-cause of latest `Task init failed` was confirmed: disabling `enable_move_to_place_stage` previously disabled only the `move to place` connect stage, while the `place object` container remained in task graph.
   - Fixed `mtc_pick_place_demo.cpp` so `enable_move_to_place_stage` now gates the whole place container subtree.
   - Rebuild and pick-only regression run (`enable_move_to_place_stage:=false`, `enable_gripper_actions:=false`) no longer fails during `task.init()` with interface mismatch.
   - Current blocker after this fix: plan still fails in pick pipeline (`move to pick`/`grasp pose IK` path), but this is now a planning-quality issue rather than task-graph wiring failure.
18. 2026-04-20 planning visibility status update (new):
   - User confirms pick and place planning stages can now complete end-to-end in current branch/profile.
   - New issue focus moved to node lifecycle UX: after planning completes, visualization should remain available in RViz without process exit.
   - Lifecycle fix has been applied in code (see Completed #12).
19. 2026-04-20 execute-track planning kickoff (new):
   - Execute path is intentionally treated as separate track from planning-stage closure.
   - Immediate requirement is to define a robust execute strategy that preserves RViz introspection even when execute fails.
   - Three subagent方案 (A/B/C) were organized and consolidated below for implementation handoff.
20. 2026-04-20 RViz red-state follow-up fix (new):
   - User observed that despite planning completion, RViz final task appeared red and stage alternatives were not browsable.
   - Follow-up patch applied: task object lifetime extended beyond `run()` scope (member-owned task).
   - Execute path remains untouched by request; this round only addresses planning visualization continuity.

### Expected Next Steps

1. Finalize one stable default profile for grasp/approach by fixing stage ordering + collision policy + pose offsets together (single tested bundle).
2. Add a lightweight validation script for:
   - topic health,
   - controller state,
   - object sync consistency.
3. (Optional) Add a single shared config source for MTC + Isaac object parameters.

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
- `sim/launch/config/sim_default.yaml`
- `sim/launch/README.md`
- `ros2_ws/src/moveit_robot_config/launch/isaac_sim_moveit.launch.py`
- `ros2_ws/src/moveit_robot_config/config/isaac_sim_moveit_defaults.yaml`
- `ros2_ws/src/moveit_robot_config/README.md`
- `ros2_ws/src/moveit_mtc_pick_place_demo/launch/mtc_pick_place_demo.launch.py`
- `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp`
- `ros2_ws/src/moveit_mtc_pick_place_demo/README.md`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_launch_defaults.yaml`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/mtc_pick_place.rviz`
- `ros2_ws/scripts/run_moveit_isaac_test.py`
- `ros2_ws/scripts/run_mtc_demo.py`
- `README.md`
- `ros2_ws/README.md`
- `ros2_ws/src/robot_description/README.md`
- `ros2_ws/src/moveit_task_constructor/README.md`
- `ros2_ws/src/moveit_task_constructor/demo/README.md`
- `ros2_ws/src/moveit_task_constructor/core/src/scope_guard/README.md`
- `docs/ARCHITECTURE.md`
- `docs/TASK_RECORD.md`
- `docs/ENV_PROJECT_KNOWLEDGE.md`
- `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp`
- `ros2_ws/src/moveit_mtc_pick_place_demo/config/pick_place_params.yaml`
- `ros2_ws/src/moveit_mtc_pick_place_demo/src/mtc_pick_place_demo.cpp` (2026-04-18: joint-state wait + failure-detail logging)

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

## 6. Execute Implementation Proposals (Subagent Round, 2026-04-20)

### Proposal A: Official-style lifecycle baseline + project-safe extensions

1. Core alignment with official demo
   - Keep execution entrypoint pattern: plan -> optional execute.
   - Keep post-plan node keep-alive behavior via spinning thread join.

2. Suggested parameters
   - `execute` (bool): whether to call `task.execute()`.
   - `keep_alive_after_plan` (bool): keep node alive after planning.
   - `keep_alive_after_execute_failure` (bool): keep node alive when execute fails.

3. Failure handling policy
   - Always log `MoveItErrorCodes` value on execute result.
   - On execute failure, do not force process exit when keep-alive is enabled.
   - Preserve introspection context for RViz diagnosis.

4. Risk notes
   - Task object lifetime must outlive visualization window.
   - Avoid executor deadlock when callbacks and execute run concurrently.
   - Scene consistency between MoveIt and Isaac must be verified after execute fail.
   - Ctrl+C shutdown path should remain deterministic.

### Proposal B: State-machine driven execute with keep-alive terminal state

1. Suggested runtime states
   - `PLANNING_SUCCESS` / `PLANNING_FAIL`
   - `EXECUTE_SUCCESS` / `EXECUTE_FAIL`
   - terminal `IDLE_KEEP_ALIVE`

2. Observability points
   - Log plan success/failure with `task.explainFailure(...)` on failure.
   - Log whether execute is skipped by policy.
   - Log execute error code and mapped text reason.
   - Log transition into keep-alive state.

3. Minimal-intrusion refactor guidance
   - Keep current structure; split planning/execution branches clearly.
   - Add execute-related parameters only, avoid broad stage rewrites.
   - Keep main lifecycle simple: spin thread + run once + join.

4. Validation matrix highlights
   - execute on/off
   - execute fail injection
   - plan fail with keep-alive
   - Ctrl+C exit behavior under keep-alive

### Proposal C: Parameter layering + phased rollout

1. Parameter hierarchy
   - Code defaults < YAML defaults < launch override < CLI override.

2. Execute-related defaults (recommended)
   - `execute=false` by default for safer bring-up/debug.
   - optional execution timeout and scaling parameters for runtime safety.

3. Rollout phases
   - Phase 1: add execute policy parameters in code + YAML.
   - Phase 2: expose launch/CLI controls and verify overrides.
   - Phase 3: add acceptance checks and runtime metrics logging.

## 7. Subagent Meeting Minutes (2026-04-20)

### Objective

Define a practical execute implementation plan that does not regress the newly restored planning-stage visibility in RViz.

### Attendees

- Main agent (integration owner)
- Explore subagent A (official lifecycle alignment)
- Explore subagent B (failure-tolerant runtime state design)
- Explore subagent C (parameterization and rollout planning)

### Discussion Summary

1. Agreement points
   - Keep official lifecycle principle: planning and execute happen once, process remains alive for introspection.
   - Execute failure must not automatically destroy debugging context.
   - Parameterized execute policy is required for safe iterative debugging.

2. Divergence points
   - A favored direct alignment with minimal additional structure.
   - B favored explicit state-machine semantics for clearer runtime transitions.
   - C emphasized rollout safety and parameter ownership layering.

3. Consolidated decision
   - Adopt A lifecycle baseline immediately (already completed for keep-alive).
   - Implement B-style explicit logging and branch clarity during execute补全.
   - Use C parameter hierarchy as acceptance contract for launch/YAML/CLI behavior.

4. Action items
   - [ ] Add execute policy parameters to `mtc_pick_place_demo.cpp` and default YAML.
   - [ ] Implement execute branch with failure-tolerant keep-alive behavior.
   - [ ] Add runbook-level acceptance checklist (execute on/off + fail path + Ctrl+C).
