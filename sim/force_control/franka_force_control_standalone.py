# Isaac Sim 5.1.0 Standalone Force Control Template

from __future__ import annotations

import signal
from typing import Any

import numpy as np

from isaacsim import SimulationApp

# Isaac Sim 5.1.0 standalone app entry. Requirement: keep GUI mode by default.
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka import Franka

try:
    # Preferred by task requirement; kept first for forward compatibility.
    from isaacsim.core.prims import ArticulationAction  # type: ignore[attr-defined]
except (ImportError, AttributeError):
    # Isaac Sim 5.1.0 provides this class from core.utils.types.
    from isaacsim.core.utils.types import ArticulationAction


PHYSICS_DT = 0.01
STAGE_UNITS_IN_METERS = 1.0
ARM_DOF = 7
ARM_JOINT_INDICES = np.arange(ARM_DOF, dtype=np.int64)
FRANKA_PRIM_PATH = "/World/Franka"
FRANKA_NAME = "franka"


stop_requested = False


def _on_sigint(_signum: int, _frame: Any) -> None:
    global stop_requested
    stop_requested = True
    print("[INFO] SIGINT received. Finishing current loop safely...")


signal.signal(signal.SIGINT, _on_sigint)


my_world = World(stage_units_in_meters=STAGE_UNITS_IN_METERS, physics_dt=PHYSICS_DT)

# Built-in Franka robot from isaacsim.robot.manipulators examples.
my_franka = my_world.scene.add(Franka(prim_path=FRANKA_PRIM_PATH, name=FRANKA_NAME))
my_world.reset()

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
    """Controller placeholder.

    Insert your SMC / impedance / other torque control law here.
    Input state contains 7-DoF arm vectors: q, dq, tau.
    """

    required_keys = ("q", "dq", "tau")
    for key in required_keys:
        if key not in state:
            raise ValueError(f"Missing required state key: {key}")
        if np.asarray(state[key]).shape[0] != ARM_DOF:
            raise ValueError(f"State key '{key}' must have length 7")

    return np.zeros(ARM_DOF, dtype=np.float32)


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
while simulation_app.is_running() and not stop_requested:
    # Match official stacking.py lifecycle: step inside running loop.
    my_world.step(render=True)

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

simulation_app.close()
