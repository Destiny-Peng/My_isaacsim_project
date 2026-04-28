# Isaac Sim 5.1.0 Standalone Force Control Template

from __future__ import annotations

import argparse
import signal
from typing import Any
from pathlib import Path

import numpy as np

from isaacsim import SimulationApp

ARM_DOF = 7
DEFAULT_PHYSICS_DT = 0.01
DEFAULT_STAGE_UNITS_IN_METERS = 1.0
DEFAULT_KP = np.array([120.0, 120.0, 110.0, 100.0, 80.0, 60.0, 40.0], dtype=np.float64)
DEFAULT_DAMPING_RATIO = 1.0
DEFAULT_TORQUE_LIMIT = np.array([80.0, 80.0, 80.0, 60.0, 40.0, 30.0, 20.0], dtype=np.float64)
DEFAULT_Q_TARGET = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], dtype=np.float64)
FRANKA_PRIM_PATH = "/World/Franka"
FRANKA_NAME = "franka"
ARM_JOINT_INDICES = np.arange(ARM_DOF, dtype=np.int64)


def _parse_vector7(text: str, name: str) -> np.ndarray:
    parts = [p.strip() for p in text.split(",") if p.strip()]
    if len(parts) != ARM_DOF:
        raise ValueError(f"{name} expects {ARM_DOF} comma-separated values, got: {text}")
    return np.array([float(v) for v in parts], dtype=np.float64)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Franka 7-DoF joint impedance control example (Isaac Sim 5.1.0)")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without GUI window")
    parser.add_argument("--param-file", type=str, default="", help="YAML param file to load defaults (only --param-file needed)")
    parser.add_argument("--physics-dt", type=float, default=DEFAULT_PHYSICS_DT, help="Physics step size in seconds")
    parser.add_argument(
        "--auto-fix-physics",
        action="store_true",
        help="Best-effort: attempt to reduce physics_dt at runtime when jitter is suspected",
    )
    parser.add_argument("--max-steps", type=int, default=0, help="Stop after N control steps (0 means run until closed)")
    parser.add_argument(
        "--kp",
        type=str,
        default=",".join(str(v) for v in DEFAULT_KP.tolist()),
        help="7 joint stiffness values, comma-separated",
    )
    parser.add_argument(
        "--damping-ratio",
        type=float,
        default=DEFAULT_DAMPING_RATIO,
        help="Critical damping ratio for kd = 2*zeta*sqrt(kp)",
    )
    parser.add_argument(
        "--torque-limit",
        type=str,
        default=",".join(str(v) for v in DEFAULT_TORQUE_LIMIT.tolist()),
        help="7 absolute torque limits, comma-separated",
    )
    parser.add_argument(
        "--q-target",
        type=str,
        default=",".join(str(v) for v in DEFAULT_Q_TARGET.tolist()),
        help="7 joint target positions (rad), comma-separated",
    )
    return parser


def _load_param_file(param_file: str) -> dict:
    if not param_file:
        return {}

    file_path = Path(param_file).expanduser().resolve()
    if not file_path.exists():
        raise FileNotFoundError(f"Param file not found: {file_path}")

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required for --param-file support") from exc

    with open(file_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    # Support both flattened and nested under `force_control` profiles.
    if isinstance(raw, dict) and "force_control" in raw:
        return raw.get("force_control", {}) or {}
    if isinstance(raw, dict):
        return raw
    return {}


args = _build_arg_parser().parse_args()

# Load param-file overrides if provided. These override CLI defaults.
_param_cfg = _load_param_file(args.param_file) if getattr(args, "param_file", "") else {}

def _cfg_get(key: str, default):
    if key in _param_cfg:
        return _param_cfg[key]
    return default


# Resolve final scalar/list parameters (accept list or comma-separated string)
def _resolve_vector7(value, name: str) -> np.ndarray:
    if isinstance(value, (list, tuple, np.ndarray)):
        arr = np.array(value, dtype=np.float64)
        if arr.size != ARM_DOF:
            raise ValueError(f"{name} expects length {ARM_DOF}, got {arr}")
        return arr
    if isinstance(value, str):
        return _parse_vector7(value, name)
    # fallback to parser-provided CLI string
    return _parse_vector7(getattr(args, name.replace("-", "_")), name)


# Final run-time parameter values (YAML overrides CLI)
KP = _resolve_vector7(_cfg_get("kp", args.kp), "kp")
KD = 2.0 * float(_cfg_get("damping_ratio", args.damping_ratio)) * np.sqrt(KP)
TORQUE_LIMIT = _resolve_vector7(_cfg_get("torque_limit", args.torque_limit), "torque-limit")
Q_TARGET = _resolve_vector7(_cfg_get("q_target", args.q_target), "q-target")

# headless/physics_dt/max_steps honor param-file if present
FINAL_HEADLESS = bool(_cfg_get("headless", args.headless))
FINAL_PHYSICS_DT = float(_cfg_get("physics_dt", args.physics_dt))
FINAL_MAX_STEPS = int(_cfg_get("max_steps", args.max_steps))
AUTO_FIX_PHYSICS = bool(_cfg_get("auto_fix_physics", getattr(args, "auto_fix_physics", False)))

# Isaac Sim 5.1.0 standalone app entry. Create app after resolving headless.
simulation_app = SimulationApp({"headless": bool(FINAL_HEADLESS)})

# Try to enable interactive PhysX UI extension so user can apply forces via UI.
try:
    import omni.kit.app

    try:
        manager = omni.kit.app.get_app().get_extension_manager()
        # Attempt enable immediately; ignore failures but log them.
        try:
            if not manager.get_enabled_extension_id("omni.physx.ui"):
                manager.set_extension_enabled_immediate("omni.physx.ui", True)
                print("[INFO] Enabled extension: omni.physx.ui")
            else:
                print("[INFO] omni.physx.ui already enabled")
        except Exception as _:
            print("[WARN] Could not toggle omni.physx.ui extension immediately")
    except Exception:
        print("[WARN] Extension manager unavailable; skipping extension enable attempt")
except Exception:
    print("[WARN] omni.kit.app not available; cannot enable UI extension")

from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka import Franka

try:
    # Preferred by task requirement; kept first for forward compatibility.
    from isaacsim.core.prims import ArticulationAction  # type: ignore[attr-defined]
except (ImportError, AttributeError):
    # Isaac Sim 5.1.0 provides this class from core.utils.types.
    from isaacsim.core.utils.types import ArticulationAction


PHYSICS_DT = float(FINAL_PHYSICS_DT)
STAGE_UNITS_IN_METERS = DEFAULT_STAGE_UNITS_IN_METERS


stop_requested = False


def _on_sigint(_signum: int, _frame: Any) -> None:
    global stop_requested
    stop_requested = True
    print("[INFO] SIGINT received. Finishing current loop safely...")


signal.signal(signal.SIGINT, _on_sigint)


my_world = World(stage_units_in_meters=STAGE_UNITS_IN_METERS, physics_dt=float(PHYSICS_DT))

# Built-in Franka robot from isaacsim.robot.manipulators examples.
my_franka = my_world.scene.add(Franka(prim_path=FRANKA_PRIM_PATH, name=FRANKA_NAME))
my_world.reset()

# Optional best-effort runtime physics dt adjustment to mitigate high-frequency jitter.
if AUTO_FIX_PHYSICS:
    try:
        # Attempt to lower physics dt by factor of two (best-effort).
        new_dt = float(PHYSICS_DT) / 2.0 if float(PHYSICS_DT) > 0.0 else float(PHYSICS_DT)
        updated = False
        if hasattr(my_world, "set_physics_dt"):
            try:
                my_world.set_physics_dt(new_dt)
                updated = True
            except Exception:
                updated = False
        else:
            try:
                setattr(my_world, "physics_dt", float(new_dt))
                updated = True
            except Exception:
                updated = False

        if updated:
            print(f"[INFO] AUTO_FIX: adjusted runtime physics_dt -> {new_dt}")
        else:
            print("[WARN] AUTO_FIX: no runtime API available to change physics_dt; restart with smaller physics-dt to attempt fix")
    except Exception as exc:
        print(f"[WARN] AUTO_FIX: failed to adjust physics dt: {exc}")

articulation_controller = my_franka.get_articulation_controller()


def _configure_pure_torque_mode() -> None:
    """Set first 7 arm joints gains to zero for torque-only control."""

    kps, kds = articulation_controller.get_gains()
    kps = np.asarray(kps, dtype=np.float32)
    kds = np.asarray(kds, dtype=np.float32)

    if kps.shape[-1] < ARM_DOF or kds.shape[-1] < ARM_DOF:
        raise ValueError(
            f"Robot dof is insufficient for 7-DoF arm control: "
            f"kps.shape={kps.shape}, kds.shape={kds.shape}"
        )

    if kps.ndim == 1:
        kps[ARM_JOINT_INDICES] = 0.0
    elif kps.ndim == 2:
        kps[:, ARM_JOINT_INDICES] = 0.0
    else:
        raise ValueError(f"Unexpected kps shape from get_gains: {kps.shape}")

    if kds.ndim == 1:
        kds[ARM_JOINT_INDICES] = 0.0
    elif kds.ndim == 2:
        kds[:, ARM_JOINT_INDICES] = 0.0
    else:
        raise ValueError(f"Unexpected kds shape from get_gains: {kds.shape}")

    articulation_controller.set_gains(kps=kps, kds=kds)


def get_robot_state() -> dict[str, np.ndarray]:
    """Read first 7 arm-joint state from articulation native API."""

    q_all = np.asarray(my_franka.get_joint_positions(), dtype=np.float64)
    dq_all = np.asarray(my_franka.get_joint_velocities(), dtype=np.float64)
    tau_all = np.asarray(my_franka.get_applied_joint_efforts(), dtype=np.float64)

    q = q_all[ARM_JOINT_INDICES].copy()
    dq = dq_all[ARM_JOINT_INDICES].copy()
    tau = tau_all[ARM_JOINT_INDICES].copy()

    if q.shape[0] != ARM_DOF or dq.shape[0] != ARM_DOF or tau.shape[0] != ARM_DOF:
        raise ValueError(
            "State dimension mismatch: expected length 7 for q/dq/tau, got "
            f"q={q.shape[0]}, dq={dq.shape[0]}, tau={tau.shape[0]}"
        )

    return {"q": q, "dq": dq, "tau": tau}


def compute_control_torques(state: dict[str, np.ndarray]) -> np.ndarray:
    """Joint-space impedance controller.

    tau = Kp * (q_des - q) + Kd * (dq_des - dq), where dq_des = 0.
    Replace this block with your own advanced controller if needed.
    """

    required_keys = ("q", "dq", "tau")
    for key in required_keys:
        if key not in state:
            raise ValueError(f"Missing required state key: {key}")
        if np.asarray(state[key]).shape[0] != ARM_DOF:
            raise ValueError(f"State key '{key}' must have length 7")

    q = np.asarray(state["q"], dtype=np.float64)
    dq = np.asarray(state["dq"], dtype=np.float64)

    pos_err = Q_TARGET - q
    vel_err = -dq
    tau = KP * pos_err + KD * vel_err
    tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

    return tau.astype(np.float32)


def set_robot_torques(torques: np.ndarray) -> None:
    """Apply first 7 joint torques via ArticulationAction."""

    tau = np.asarray(torques, dtype=np.float32).reshape(-1)
    if tau.shape[0] != ARM_DOF:
        raise ValueError(f"Torque dimension mismatch: expected length 7, got {tau.shape[0]}")

    action = ArticulationAction(joint_efforts=tau, joint_indices=ARM_JOINT_INDICES)
    articulation_controller.apply_action(action)


# Apply torque-mode setup right after reset, and again after any world reset.
_configure_pure_torque_mode()


reset_needed = False
step_count = 0
while simulation_app.is_running() and not stop_requested:
    if FINAL_MAX_STEPS > 0 and step_count >= int(FINAL_MAX_STEPS):
        print(f"[INFO] Reached max-steps={FINAL_MAX_STEPS}, exiting.")
        break

        # Match official stacking.py lifecycle: step inside running loop.
    my_world.step(render=not bool(FINAL_HEADLESS))

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            _configure_pure_torque_mode()
            reset_needed = False

        robot_state = get_robot_state()
        tau_cmd = compute_control_torques(robot_state)
        set_robot_torques(tau_cmd)
        step_count += 1

simulation_app.close()
