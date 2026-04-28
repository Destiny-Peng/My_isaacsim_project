# Isaac Sim 5.1.0 Standalone Force Control Template

from __future__ import annotations

import argparse
import csv
import signal
import time
from typing import Any
from pathlib import Path

import numpy as np
import pinocchio as pin

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
ARM_JOINT_NAMES = [f"panda_joint{i+1}" for i in range(ARM_DOF)]


# ---- Data Recorder (incremental CSV flush, same pattern as grasp_eval) ---- #

_FORCE_CONTROL_FIELD_NAMES = [
    "step",
    "sim_time_s",
    "wall_time_s",
    "q1", "q2", "q3", "q4", "q5", "q6", "q7",
    "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7",
    "tau_cmd_1", "tau_cmd_2", "tau_cmd_3", "tau_cmd_4", "tau_cmd_5", "tau_cmd_6", "tau_cmd_7",
    "tau_applied_1", "tau_applied_2", "tau_applied_3", "tau_applied_4", "tau_applied_5", "tau_applied_6", "tau_applied_7",
]


class JointDataRecorder:
    """Record joint q/dq/tau at each control step with incremental CSV flush."""

    def __init__(self, output_prefix: str) -> None:
        self._output_prefix = str(output_prefix)
        self._rows: list[dict[str, Any]] = []
        self._flush_interval = 100
        self._last_flushed = 0
        self._csv_path: Path | None = None

    @property
    def csv_path(self) -> Path | None:
        return self._csv_path

    def record(
        self,
        step: int,
        sim_time_s: float,
        q: np.ndarray,
        dq: np.ndarray,
        tau_cmd: np.ndarray,
        tau_applied: np.ndarray,
    ) -> None:
        row: dict[str, Any] = {"step": step, "sim_time_s": sim_time_s, "wall_time_s": time.time()}
        for i in range(ARM_DOF):
            row[f"q{i+1}"] = float(q[i])
            row[f"dq{i+1}"] = float(dq[i])
            row[f"tau_cmd_{i+1}"] = float(tau_cmd[i])
            row[f"tau_applied_{i+1}"] = float(tau_applied[i])
        self._rows.append(row)

        if len(self._rows) - self._last_flushed >= self._flush_interval:
            self._flush()

    def _flush(self) -> None:
        if not self._rows:
            return
        to_flush = self._rows[self._last_flushed:]
        if not to_flush:
            return
        if self._csv_path is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            self._csv_path = Path(self._output_prefix).expanduser()
            if not self._csv_path.suffix:
                self._csv_path = self._csv_path.parent / f"{self._csv_path.name}_{timestamp}.csv"
            self._csv_path.parent.mkdir(parents=True, exist_ok=True)

        write_header = (not self._csv_path.exists()) or self._csv_path.stat().st_size == 0
        with self._csv_path.open("a", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(_FORCE_CONTROL_FIELD_NAMES)
            for row in to_flush:
                writer.writerow([row.get(name, "") for name in _FORCE_CONTROL_FIELD_NAMES])

        self._last_flushed = len(self._rows)

    def flush_and_close(self) -> None:
        self._flush()
        if self._csv_path and self._csv_path.exists():
            print(f"[INFO] Force-control data recorded: {self._csv_path} ({len(self._rows)} rows)")


# ---- Pinocchio-based impedance controller ---- #

class PinocchioImpedanceController:
    """Model-based joint-space impedance controller using Pinocchio dynamics.

    Control law (computed-torque style):
        tau = M(q) * (Kp * (q_des - q) + Kd * (dq_des - dq))
              + C(q, dq) * dq + g(q)

    This compensates for Coriolis/centrifugal and gravity terms so the
    closed-loop behaves as a decoupled mass-spring-damper system.
    """

    def __init__(
        self,
        urdf_path: str,
        kp: np.ndarray,
        kd: np.ndarray,
        torque_limit: np.ndarray,
    ) -> None:
        urdf_file = Path(urdf_path).expanduser().resolve()
        if not urdf_file.exists():
            raise FileNotFoundError(f"URDF not found: {urdf_file}")
        self._model = pin.buildModelFromUrdf(str(urdf_file))
        self._data = self._model.createData()

        # First 7 active joints are the arm (panda_joint1..panda_joint7).
        # Finger joints 8,9 are excluded from the arm control.
        arm_model_idx = list(range(1, ARM_DOF + 1))  # skip universe joint (idx 0)
        self._arm_q_idx = []  # indices in configuration vector
        self._arm_v_idx = []  # indices in velocity vector
        for jid in range(1, self._model.njoints):
            nq = self._model.joints[jid].nq
            nv = self._model.joints[jid].nv
            if jid in arm_model_idx:
                self._arm_q_idx.append(self._model.joints[jid].idx_q)
                self._arm_v_idx.append(self._model.joints[jid].idx_v)

        if len(self._arm_q_idx) != ARM_DOF:
            raise RuntimeError(
                f"Pinocchio: expected {ARM_DOF} arm joints, got {len(self._arm_q_idx)}. "
                f"Model has {self._model.njoints - 1} active joints."
            )

        self._kp = np.asarray(kp, dtype=np.float64).ravel()
        self._kd = np.asarray(kd, dtype=np.float64).ravel()
        self._torque_limit = np.asarray(torque_limit, dtype=np.float64).ravel()

        print(
            f"[INFO] PinocchioImpedanceController: model={urdf_file.name}, "
            f"arm_joints={ARM_DOF}, nq={self._model.nq}, nv={self._model.nv}"
        )

    def compute(self, q: np.ndarray, dq: np.ndarray, q_des: np.ndarray) -> np.ndarray:
        """Compute model-based impedance torques for the 7 arm joints.

        Parameters
        ----------
        q : ndarray shape (7,)
            Current joint positions (rad).
        dq : ndarray shape (7,)
            Current joint velocities (rad/s).
        q_des : ndarray shape (7,)
            Desired joint positions (rad).

        Returns
        -------
        tau : ndarray shape (7,), float32
            Torque commands for the arm joints.
        """
        q_full = np.zeros(self._model.nq, dtype=np.float64)
        dq_full = np.zeros(self._model.nv, dtype=np.float64)
        q_full[self._arm_q_idx] = q
        dq_full[self._arm_v_idx] = dq

        pin.crba(self._model, self._data, q_full)  # M(q) stored in data.M
        g_vec = pin.computeGeneralizedGravity(self._model, self._data, q_full)
        C_mat = pin.computeCoriolisMatrix(self._model, self._data, q_full, dq_full)
        C_times_dq = C_mat @ dq_full

        M_full = self._data.M  # (nv x nv)
        # Extract arm-only sub-components (first 7 DOFs)
        arm_idx = np.arange(ARM_DOF)
        M_arm = M_full[np.ix_(arm_idx, arm_idx)]
        C_arm = C_times_dq[arm_idx]
        g_arm = g_vec[arm_idx]

        # Desired acceleration: spring-damper (dq_des = 0)
        pos_err = np.asarray(q_des, dtype=np.float64).ravel() - q
        a_des = self._kp * pos_err - self._kd * dq

        # Computed-torque impedance: tau = M * a_des + C + g
        # tau = M_arm @ a_des + C_arm + g_arm

        # do a test, if we remove the C_arm and g_arm compensation, how does it perform?
        tau = M_arm @ a_des
        tau = np.clip(tau, -self._torque_limit, self._torque_limit)

        return tau.astype(np.float32, copy=False)


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
        help="[Deprecated] Best-effort: attempt to reduce physics_dt at runtime when jitter is suspected",
    )
    parser.add_argument("--max-steps", type=int, default=0, help="Stop after N control steps (0 means run until closed)")
    parser.add_argument(
        "--urdf-path",
        type=str,
        default="",
        help="Path to the Franka URDF for Pinocchio (overrides param-file)",
    )
    parser.add_argument(
        "--solver-position-iterations",
        type=int,
        default=0,
        help="PhysX solver position iterations (0 = use engine default)",
    )
    parser.add_argument(
        "--solver-velocity-iterations",
        type=int,
        default=0,
        help="PhysX solver velocity iterations (0 = use engine default)",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Enable CSV recording of joint q/dq/tau data (incremental flush)",
    )
    parser.add_argument(
        "--recording-prefix",
        type=str,
        default="",
        help="Output prefix for CSV recording (overrides param-file)",
    )
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

# Load param-file overrides if provided. CLI args take precedence over YAML.
_param_cfg = _load_param_file(args.param_file) if getattr(args, "param_file", "") else {}


def _merge_param(cli_value, yaml_key: str) -> Any:
    """Return YAML value only when the CLI value matches its argparse default;
    otherwise the explicitly provided CLI value wins."""
    if yaml_key in _param_cfg:
        return _param_cfg[yaml_key]
    return cli_value


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


# Final run-time parameter values (CLI args take precedence over YAML)
def _yaml_or_cli(key: str, cli_val):
    """Use YAML value when present; otherwise fall back to CLI value."""
    if key in _param_cfg:
        return _param_cfg[key]
    return cli_val


KP = _resolve_vector7(_yaml_or_cli("kp", args.kp), "kp")
KD = 2.0 * float(_yaml_or_cli("damping_ratio", args.damping_ratio)) * np.sqrt(KP)
TORQUE_LIMIT = _resolve_vector7(_yaml_or_cli("torque_limit", args.torque_limit), "torque-limit")
Q_TARGET = _resolve_vector7(_yaml_or_cli("q_target", args.q_target), "q-target")

# Helper: apply YAML only when CLI matches parser default.
def _apply_param(cli_val, parser_default, yaml_key: str):
    """If CLI value differs from parser default, use CLI (user intended it).
    Otherwise fall back to YAML value if present, else parser default."""
    if cli_val != parser_default:
        return cli_val
    if yaml_key in _param_cfg:
        return _param_cfg[yaml_key]
    return cli_val


# Store parser defaults for CLI-vs-default detection.
PARSER_DEFAULTS = _build_arg_parser().parse_args([])

FINAL_HEADLESS = bool(_apply_param(args.headless, PARSER_DEFAULTS.headless, "headless"))
FINAL_PHYSICS_DT = float(_apply_param(args.physics_dt, PARSER_DEFAULTS.physics_dt, "physics_dt"))
FINAL_MAX_STEPS = int(_apply_param(args.max_steps, PARSER_DEFAULTS.max_steps, "max_steps"))
AUTO_FIX_PHYSICS = bool(_apply_param(
    getattr(args, "auto_fix_physics", False),
    getattr(PARSER_DEFAULTS, "auto_fix_physics", False),
    "auto_fix_physics",
))

# Pinocchio URDF path (YAML or CLI)
URDF_PATH = str(_apply_param(
    getattr(args, "urdf_path", ""),
    getattr(PARSER_DEFAULTS, "urdf_path", ""),
    "urdf_path",
))
# Solver iterations (0 = engine default)
SOLVER_POS_ITERS = int(_apply_param(
    getattr(args, "solver_position_iterations", 0),
    getattr(PARSER_DEFAULTS, "solver_position_iterations", 0),
    "solver_position_iterations",
))
SOLVER_VEL_ITERS = int(_apply_param(
    getattr(args, "solver_velocity_iterations", 0),
    getattr(PARSER_DEFAULTS, "solver_velocity_iterations", 0),
    "solver_velocity_iterations",
))
# Recording
ENABLE_RECORDING = bool(_apply_param(
    getattr(args, "record", False),
    getattr(PARSER_DEFAULTS, "record", False),
    "record",
))
RECORDING_PREFIX = str(_apply_param(
    getattr(args, "recording_prefix", "sim/outputs/force_control"),
    getattr(PARSER_DEFAULTS, "recording_prefix", "sim/outputs/force_control"),
    "recording_output_prefix",
))

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

# ---- Configure physics solver iterations for stable force control ---- #
if SOLVER_POS_ITERS > 0 or SOLVER_VEL_ITERS > 0:
    try:
        import carb

        physx_iface = carb.settings.get_settings()
        if SOLVER_POS_ITERS > 0:
            physx_iface.set_int("/physics/solverPositionIterationCount", int(SOLVER_POS_ITERS))
            print(f"[INFO] Physics solver position iterations: {SOLVER_POS_ITERS}")
        if SOLVER_VEL_ITERS > 0:
            physx_iface.set_int("/physics/solverVelocityIterationCount", int(SOLVER_VEL_ITERS))
            print(f"[INFO] Physics solver velocity iterations: {SOLVER_VEL_ITERS}")
    except Exception as exc:
        print(f"[WARN] Failed to configure physics solver iterations: {exc}")

# Optional best-effort runtime physics dt adjustment (deprecated).
if AUTO_FIX_PHYSICS:
    try:
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
            print("[WARN] AUTO_FIX: no runtime API available to change physics_dt")
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


def set_robot_torques(torques: np.ndarray) -> None:
    """Apply first 7 joint torques via ArticulationAction."""
    tau = np.asarray(torques, dtype=np.float32).reshape(-1)
    if tau.shape[0] != ARM_DOF:
        raise ValueError(f"Torque dimension mismatch: expected length 7, got {tau.shape[0]}")
    action = ArticulationAction(joint_efforts=tau, joint_indices=ARM_JOINT_INDICES)
    articulation_controller.apply_action(action)


# ---- Initialize Pinocchio impedance controller ---- #
if not URDF_PATH:
    # Default: resolve relative to project root (2 levels up from this script).
    _script_dir = Path(__file__).resolve().parent
    _project_root = _script_dir.parent.parent  # isaac_sim_fullstack/
    _default_urdf = (_project_root / "ros2_ws/src/robot_description/urdf/arm_only_franka.urdf").resolve()
    if _default_urdf.exists():
        URDF_PATH = str(_default_urdf)
    else:
        raise FileNotFoundError(
            f"No URDF found at default path {_default_urdf}. "
            "Specify --urdf-path or add urdf_path to your YAML config."
        )
else:
    # Resolve relative to project root if not absolute
    _urdf_candidate = Path(URDF_PATH)
    if not _urdf_candidate.is_absolute():
        _script_dir = Path(__file__).resolve().parent
        _project_root = _script_dir.parent.parent
        _urdf_candidate = (_project_root / URDF_PATH).resolve()
    if not _urdf_candidate.exists():
        raise FileNotFoundError(f"URDF not found: {_urdf_candidate}")
    URDF_PATH = str(_urdf_candidate)

print(f"[INFO] Using URDF: {URDF_PATH}")
_impedance_ctrl = PinocchioImpedanceController(
    urdf_path=URDF_PATH,
    kp=KP,
    kd=KD,
    torque_limit=TORQUE_LIMIT,
)


def compute_control_torques(state: dict[str, np.ndarray]) -> np.ndarray:
    """Model-based joint-space impedance control via Pinocchio.

    tau = M(q) * (Kp * (q_des - q) + Kd * (0 - dq)) + C(q,dq)*dq + g(q)
    """
    required_keys = ("q", "dq")
    for key in required_keys:
        if key not in state:
            raise ValueError(f"Missing required state key: {key}")
        if np.asarray(state[key]).shape[0] != ARM_DOF:
            raise ValueError(f"State key '{key}' must have length 7")

    q = np.asarray(state["q"], dtype=np.float64)
    dq = np.asarray(state["dq"], dtype=np.float64)
    tau = _impedance_ctrl.compute(q, dq, Q_TARGET)
    return tau


# ---- Recording setup ---- #
_recorder: JointDataRecorder | None = None
if ENABLE_RECORDING:
    _recorder = JointDataRecorder(output_prefix=RECORDING_PREFIX)
    print(f"[INFO] Force-control recording enabled: {RECORDING_PREFIX}")

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

        if _recorder is not None:
            _recorder.record(
                step=step_count,
                sim_time_s=step_count * PHYSICS_DT,
                q=robot_state["q"],
                dq=robot_state["dq"],
                tau_cmd=tau_cmd,
                tau_applied=robot_state["tau"],
            )

        step_count += 1

# ---- Cleanup ---- #
if _recorder is not None:
    _recorder.flush_and_close()
simulation_app.close()
