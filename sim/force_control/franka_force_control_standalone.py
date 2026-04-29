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
FRANKA_PRIM_PATH = "/World/Franka"
FRANKA_NAME = "franka"
ARM_JOINT_INDICES = np.arange(ARM_DOF, dtype=np.int64)


# ---- Data Recorder (incremental CSV flush, same pattern as grasp_eval) ---- #

_FORCE_CONTROL_FIELD_NAMES = [
    "step",
    "sim_time_s",
    "wall_time_s",
    "q1", "q2", "q3", "q4", "q5", "q6", "q7",
    "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7",
    "tau_referenceInput_1", "tau_referenceInput_2", "tau_referenceInput_3", "tau_referenceInput_4", "tau_referenceInput_5", "tau_referenceInput_6", "tau_referenceInput_7",
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
        tau_referenceInput: np.ndarray,
        tau_applied: np.ndarray,
    ) -> None:
        row: dict[str, Any] = {"step": step, "sim_time_s": sim_time_s, "wall_time_s": time.time()}
        for i in range(ARM_DOF):
            row[f"q{i+1}"] = float(q[i])
            row[f"dq{i+1}"] = float(dq[i])
            row[f"tau_referenceInput_{i+1}"] = float(tau_referenceInput[i])
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

    Variable naming conventions used in this controller (matching pseudocode):
        - no subscript: actual state (e.g. q, dq)
        - subscript `desiredTrajectory`: desired/reference trajectory (e.g. q_desiredTrajectory)
        - subscript `trackingError`: tracking error (e.g. q_trackingError)
        - subscript `desiredDynamics`: desired response (accelerations) (e.g. ddq_desiredDynamics)
        - subscript `referenceInput`: reference/control input (e.g. tau_referenceInput)
    Derivatives use `d` / `dd` prefixes: dq, ddq, dq_desiredTrajectory, etc.

    Control law (joint-space computed-torque style):
        tau_referenceInput = M(q) * ddq_desiredDynamics + C(q, dq) * dq + G(q)
    Control law (formal style):
        tau = M(q) * J^-1 * [ddx_des + M(q)^-1 * (F_ext - C(q, dq) * dx_error - Kd * x_error) - dJ * dq]
              + C(q, dq) * dq + G(q)

    This compensates for Coriolis/centrifugal and gravity terms so the
    closed-loop behaves as a decoupled mass-spring-damper system.
    """

    def __init__(
        self,
        urdf_path: str,
    ) -> None:
        urdf_file = Path(urdf_path).expanduser().resolve()
        if not urdf_file.exists():
            raise FileNotFoundError(f"URDF not found: {urdf_file}")
        self._model = pin.buildModelFromUrdf(str(urdf_file))
        self._data = self._model.createData()
        self._end_effector_id = self._model.getFrameId("panda_hand")
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
        # Desired inertial
        self.M_d = np.eye(6)*1.0 
        # self.M_d = np.diag([10,10,10,0.5,0.5,0.5]) 
        # Desired stiffness
        self.K_d = np.diag([500,500,500,80,80,80]) 
        # Desired damping (critical damping Cd = 2*sqrt(Md*Kd))
        self.yita = 2.0
        self.C_d = 2*self.yita*np.sqrt(self.M_d * self.K_d)

        # 期望轨迹初始化 (这里简单设为固定位置)
        self._translation_desiredTrajectory = np.array([0.38936162, 0.004671326, 0.45737252])
        #四元数转换到旋转矩阵
        from scipy.spatial.transform import Rotation as R
        quaternion = [0.92171,0.02599,0.38696,0.00627]
        r = R.from_quat(quaternion)
        self._rotation_desiredTrajectory = r.as_matrix()
        self._dx_desiredTrajectory = np.zeros(6)
        self._ddx_desiredTrajectory = np.zeros(6)
        self.dq_filtered = np.zeros(9)
        self.tau_filterd = np.zeros(9)


        print(
            f"[INFO] PinocchioImpedanceController: model={urdf_file.name}, "
            f"arm_joints={ARM_DOF}, nq={self._model.nq}, nv={self._model.nv}"
        )


    def _compute_kinematics(self, q, dq):
        """
        占位符：留待后续手搓
        返回: x (6), J (6x7), dJ (6x7)
        """
        # 这里仅为示例结构，实际需填充真实计算逻辑
        x = np.zeros(6)
        J = np.zeros((6, 7))
        dJ = np.zeros((6, 7))
        return x, J, dJ

    def _compute_dynamics(self, q, dq):
        """
        占位符：留待后续手搓
        返回 M (7x7), C_vec (7), G_vec (7)
        """
        M = np.eye(7)
        C_vec = np.zeros(7)
        G_vec = np.zeros(7)
        return M, C_vec, G_vec

    def compute(self, q: np.ndarray, dq: np.ndarray,  F_ext: np.ndarray) -> np.ndarray:
        """Compute model-based impedance torques for the 7 arm joints.

        Parameters
        ----------
        q : ndarray shape (7,)
            Current joint positions (rad) (actual state).
        dq : ndarray shape (7,)
            Current joint velocities (rad/s) (actual state).
        F_ext : ndarray shape (6,)
            External wrench at the end-effector (force in Newtons, torque in Nm) (actual state).

        Returns
        -------
        tau_referenceInput : ndarray shape (7,), float32
            Reference torque commands for the arm joints (referenceInput).
        """

        q_full = np.zeros(self._model.nq, dtype=np.float64)
        dq_full = np.zeros(self._model.nv, dtype=np.float64)
        q_full[self._arm_q_idx] = q
        dq_full[self._arm_v_idx] = dq
        alpha = 0.2 # 滤波系数，越小越平滑但延迟越大
        self.dq_filtered = alpha * dq_full + (1 - alpha) * self.dq_filtered
        dq_full = self.dq_filtered
        # print("qfull",q_full)
        # print("dqfull",dq_full)

        # 1. 正向运动学
        pin.forwardKinematics(self._model, self._data, q_full, dq_full)
        pin.updateFramePlacements(self._model, self._data)
        oMf_current = self._data.oMf[self._end_effector_id]
        pos_error = oMf_current.translation - self._translation_desiredTrajectory
        R_error = self._rotation_desiredTrajectory @ oMf_current.rotation.T
        orient_error = pin.log3(R_error)
        x_error = np.concatenate([pos_error,orient_error])

        # 2. 雅可比 J (6x7)
        # LOCAL_WORLD_ALIGNED 表示线速度和角速度都在世界坐标系下表达
        J = pin.computeFrameJacobian(self._model, self._data, q_full, self._end_effector_id, pin.LOCAL_WORLD_ALIGNED)
        # 交换行以匹配 [linear; angular]
        # J = np.vstack([J_pin[3:, :], J_pin[:3, :]])
        dx = J@dq_full
        dx_error = dx - self._dx_desiredTrajectory
        
        # # 3. 雅可比导数 dJ (6x7)
        # dJ = pin.getFrameJacobianTimeVariation(self._model, self._data, self._end_effector_id, pin.LOCAL_WORLD_ALIGNED)
        # # dJ = np.vstack([dJ_pin[3:, :], dJ_pin[:3, :]])

        # # 4. 动力学项
        # M = pin.crba(self._model, self._data, q_full) # 惯性矩阵
        # G = pin.computeGeneralizedGravity(self._model, self._data, q_full) # 重力
        # C_vec = pin.computeCoriolisMatrix(self._model, self._data, q_full, dq_full) @ dq_full # 科氏力向量

        # # 注意：Pinocchio 的 J 通常是 [angular; linear]，如果需要 [linear; angular] 需交换
        # # tau = M(q) * J^-1 * [ddx_des + M(q)^-1 * (F_ext - C(q, dq) * dx_error - Kd * x_error) - dJ * dq]
        # #      + C(q, dq) * dq + G(q)
        # M_d_inv = np.linalg.inv(self.M_d)
        # ddx_desiredDynamics= self._ddx_desiredTrajectory + M_d_inv @ (F_ext - self.C_d @ dx_error - self.K_d @ x_error)
        # # ddx_desiredDynamics= self._ddx_desiredTrajectory + M_d_inv @ (- self.C_d @ dx_error - self.K_d @ x_error)
        # J_pinv=np.linalg.pinv(J)
        # ddq_desiredDynamics = J_pinv @ (ddx_desiredDynamics - dJ@dq_full)
        # # ddq_desiredDynamics = J_pinv @ ddx_desiredDynamics 
        # # tau = 1.0001*G
        # tau = M @ ddq_desiredDynamics+C_vec+G


        #do a test
        # # tau = G(q)-J^T(Kp*x_error+Kd*dx_error)
        Kp = np.diag([80, 80, 80, 10, 10, 10])
        # Kd = np.diag([50,50,50, 2, 2, 2])
        Kd = 2*np.sqrt(Kp)
        
        F_cart = -Kp @ x_error - Kd @ dx_error
        
        # 映射回关节力矩: tau = J.T @ F_cart + G
        tau_arm = J.T @ F_cart
        G = pin.computeGeneralizedGravity(self._model, self._data, q_full) # 重力
        tau = tau_arm + G # 只控制arm joints，finger joints保持不动

        tau = np.clip(tau, -87, 87) # Franka torque limits (Nm)
        # self.tau_filterd = alpha * tau + (1 - alpha) * self.tau_filterd
        return tau.astype(np.float32)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Franka 7-DoF joint impedance control example (Isaac Sim 5.1.0)")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without GUI window")
    parser.add_argument(
        "--render-rate-hz",
        type=float,
        default=30.0,
        help="Viewport render rate in Hz (<=0 renders every step)",
    )
    parser.add_argument("--param-file", type=str, default="", help="YAML param file to load defaults (only --param-file needed)")
    parser.add_argument("--physics-dt", type=float, default=DEFAULT_PHYSICS_DT, help="Physics step size in seconds")
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
        default="sim/outputs/force_control",
        help="Output prefix for CSV recording (overrides param-file)",
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


_parser = _build_arg_parser()
_pre_args, _ = _parser.parse_known_args()
_param_cfg = _load_param_file(_pre_args.param_file) if getattr(_pre_args, "param_file", "") else {}


def _apply_yaml_defaults(parser: argparse.ArgumentParser, param_cfg: dict) -> None:
    yaml_to_dest = {
        "headless": "headless",
        "render_rate_hz": "render_rate_hz",
        "physics_dt": "physics_dt",
        "max_steps": "max_steps",
        "urdf_path": "urdf_path",
        "solver_position_iterations": "solver_position_iterations",
        "solver_velocity_iterations": "solver_velocity_iterations",
        "record": "record",
        "recording_output_prefix": "recording_prefix",
        "recording_prefix": "recording_prefix",
    }
    defaults = {}
    for yaml_key, dest in yaml_to_dest.items():
        if yaml_key in param_cfg:
            defaults[dest] = param_cfg[yaml_key]
    if defaults:
        parser.set_defaults(**defaults)


_apply_yaml_defaults(_parser, _param_cfg)
args = _parser.parse_args()

URDF_PATH = str(args.urdf_path)

# Isaac Sim 5.1.0 standalone app entry. Create app after resolving headless.
simulation_app = SimulationApp({"headless": bool(args.headless)})

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


PHYSICS_DT = float(args.physics_dt)
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

# Default camera light (USD DistantLight).
import omni.usd
from pxr import UsdLux

stage = omni.usd.get_context().get_stage()
light = UsdLux.DistantLight.Define(stage, "/Environment/defaultLight")
light.CreateIntensityAttr().Set(1000.0)

# ---- Configure physics solver iterations for stable force control ---- #
if args.solver_position_iterations > 0 or args.solver_velocity_iterations > 0:
    try:
        import carb

        physx_iface = carb.settings.get_settings()
        if args.solver_position_iterations > 0:
            physx_iface.set_int("/physics/solverPositionIterationCount", int(args.solver_position_iterations))
            print(f"[INFO] Physics solver position iterations: {args.solver_position_iterations}")
        if args.solver_velocity_iterations > 0:
            physx_iface.set_int("/physics/solverVelocityIterationCount", int(args.solver_velocity_iterations))
            print(f"[INFO] Physics solver velocity iterations: {args.solver_velocity_iterations}")
    except Exception as exc:
        print(f"[WARN] Failed to configure physics solver iterations: {exc}")



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
    metadata = my_franka._articulation_view._metadata
    joint_indices = 1 + np.array([metadata.joint_indices["panda_hand_joint"] ])
    f_all = np.asarray(my_franka.get_measured_joint_forces(joint_indices), dtype=np.float64)
    tau_all = np.asarray(my_franka.get_applied_joint_efforts(), dtype=np.float64)

    q = q_all[ARM_JOINT_INDICES].copy()
    dq = dq_all[ARM_JOINT_INDICES].copy()
    f = f_all
    tau = tau_all[ARM_JOINT_INDICES].copy()

    if q.shape[0] != ARM_DOF or dq.shape[0] != ARM_DOF :
        raise ValueError(
            "State dimension mismatch: expected length 7 for q/dq/f, got "
            f"q={q.shape[0]}, dq={dq.shape[0]}, f={f.shape[0]}"
        )
    return {"q": q, "dq": dq, "f_ext": f,"tau": tau}


def set_robot_torques(torques: np.ndarray) -> None:
    """Apply first 7 joint torques via ArticulationAction."""
    f = np.asarray(torques, dtype=np.float32).reshape(-1)
    if f.shape[0] != ARM_DOF:
        raise ValueError(f"Torque dimension mismatch: expected length 7, got {f.shape[0]}")
    action = ArticulationAction(joint_efforts=f, joint_indices=ARM_JOINT_INDICES)
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
)


def compute_control_torques(state: dict[str, np.ndarray]) -> np.ndarray:
    """Model-based joint-space impedance control via Pinocchio.

    Uses the notation:
        - q: actual joint positions
        - q_desiredTrajectory: desired joint positions
        - q_trackingError: q_desiredTrajectory - q
        - ddq_desiredDynamics: desired accelerations (spring-damper)
        - tau_referenceInput: reference torque command (computed-torque)
    Formula:
        tau_referenceInput = M(q) * ddq_desiredDynamics + C(q,dq) * dq + G(q)
    """
    required_keys = ("q", "dq")
    for key in required_keys:
        if key not in state:
            raise ValueError(f"Missing required state key: {key}")
        if np.asarray(state[key]).shape[0] != ARM_DOF:
            raise ValueError(f"State key '{key}' must have length 7")

    q = np.asarray(state["q"], dtype=np.float64)
    dq = np.asarray(state["dq"], dtype=np.float64)
    f = np.asarray(state.get("f_ext", np.zeros(6)), dtype=np.float64).ravel()  # external wrench (force/torque)
    tau_referenceInput = _impedance_ctrl.compute(q, dq, f)
    # clip the tau from 9 dof to arm dof
    tau_referenceInput = tau_referenceInput[:ARM_DOF]
    return tau_referenceInput


# ---- Recording setup ---- #
_recorder: JointDataRecorder | None = None
if args.record:
    _recorder = JointDataRecorder(output_prefix=str(args.recording_prefix))
    print(f"[INFO] Force-control recording enabled: {args.recording_prefix}")

# Apply torque-mode setup right after reset, and again after any world reset.
_configure_pure_torque_mode()


render_interval_s = 0.0
last_render_time_s = 0.0
if not bool(args.headless) and float(args.render_rate_hz) > 0.0:
    render_interval_s = 1.0 / float(args.render_rate_hz)

reset_needed = False
step_count = 0
import time
prev_time = time.time()
# from omni.isaac.core.utils.physics import get_physics_interface
# scene = my_world.get_physics_context()
# scene.set_solver_type("TGS") # 建议用 TGS 稳定性更高
# scene.set_time_steps_per_second(1.0 / PHYSICS_DT) # 例如 400

while simulation_app.is_running() and not stop_requested:
    if args.max_steps > 0 and step_count >= int(args.max_steps):
        print(f"[INFO] Reached max-steps={args.max_steps}, exiting.")
        break


    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            _configure_pure_torque_mode()
            reset_needed = False

        robot_state = get_robot_state()
        tau_referenceInput = compute_control_torques(robot_state)
        set_robot_torques(tau_referenceInput)

        if _recorder is not None:
            _recorder.record(
                step=step_count,
                sim_time_s=step_count * PHYSICS_DT,
                q=robot_state["q"],
                dq=robot_state["dq"],
                tau_referenceInput=tau_referenceInput,
                tau_applied=robot_state["tau"],
            )
        # current_time = time.time()
        # actual_dt = current_time - prev_time # 实际运行的总耗时（物理+渲染）
        # prev_time = current_time
        # print("dt",actual_dt)
        step_count += 1
    # Match official stacking.py lifecycle: step inside running loop.
    should_render = not bool(args.headless)
    if should_render and render_interval_s > 0.0:
        now_s = time.monotonic()
        if last_render_time_s == 0.0 or (now_s - last_render_time_s) >= render_interval_s:
            last_render_time_s = now_s
            should_render = True
        else:
            should_render = False
    # should_render = True
    # print(my_world.current_time)
    my_world.step(render=should_render)

# ---- Cleanup ---- #
if _recorder is not None:
    _recorder.flush_and_close()
simulation_app.close()
