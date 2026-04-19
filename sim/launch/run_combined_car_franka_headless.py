#!/usr/bin/env python3
"""Headless Isaac Sim launcher for combined_car_franka USD.

Usage example:
  /home/jacy/anaconda3/envs/env_issaclab/bin/python \
    sim/launch/run_combined_car_franka_headless.py --steps 5000
"""

from __future__ import annotations

import argparse
import os
import urllib.parse
import sys
import time
import copy
from pathlib import Path

import numpy as np

from isaaclab.app import AppLauncher


class _PythonRos2JointBridge:
    """ROS2 bridge implemented in Python, independent from OmniGraph."""

    def __init__(
        self,
        articulation_prim: str,
        joint_cmd_topic: str,
        joint_state_topic: str,
        clock_topic: str,
        node_name: str,
        publish_every_steps: int,
        clock_mode: str,
        smooth_position_commands: bool,
        default_max_joint_velocity: float,
        max_joint_step_rad: float,
    ) -> None:
        self.articulation_prim = articulation_prim
        self.joint_cmd_topic = joint_cmd_topic
        self.joint_state_topic = joint_state_topic
        self.clock_topic = clock_topic
        self.node_name = node_name
        self.publish_every_steps = max(1, publish_every_steps)
        self.clock_mode = clock_mode
        self.smooth_position_commands = smooth_position_commands
        self.default_max_joint_velocity = max(0.01, default_max_joint_velocity)
        self.max_joint_step_rad = max(1e-4, max_joint_step_rad)

        self._owns_rclpy = False
        self._node = None
        self._joint_state_pub = None
        self._clock_pub = None
        self._joint_cmd_sub = None

        self._art = None
        self._dof_names = []
        self._dof_name_to_idx = {}

        self._latest_pos_cmd = None
        self._latest_vel_cmd = None
        self._latest_eff_cmd = None
        self._max_joint_velocity_limits = None
        self._last_apply_sim_time_s = None

        self._clock_anchor_wall = None
        self._clock_anchor_sim = None
        self._last_pub_time_s = None

    @staticmethod
    def _infer_joint_velocity_limits(dof_names: list[str], default_vel: float) -> np.ndarray:
        # Conservative defaults keep motion continuous even when command topic publishes sparse waypoints.
        name_to_limit = {
            "panda_joint1": 2.175,
            "panda_joint2": 2.175,
            "panda_joint3": 2.175,
            "panda_joint4": 2.175,
            "panda_joint5": 2.61,
            "panda_joint6": 2.61,
            "panda_joint7": 2.61,
            "panda_finger_joint1": 0.2,
            "panda_finger_joint2": 0.2,
        }
        return np.array([name_to_limit.get(name, default_vel) for name in dof_names], dtype=np.float64)

    @staticmethod
    def _to_sec_nsec(sim_time_s: float) -> tuple[int, int]:
        sec = int(sim_time_s)
        nsec = int((sim_time_s - sec) * 1_000_000_000)
        if nsec < 0:
            nsec = 0
        return sec, nsec

    def _resolve_publish_time(self, sim_time_s: float) -> float:
        if self.clock_mode == "wall-anchored":
            if self._clock_anchor_wall is None:
                self._clock_anchor_wall = time.time()
                self._clock_anchor_sim = sim_time_s
            sim_elapsed = max(0.0, sim_time_s - float(self._clock_anchor_sim))
            pub_time_s = float(self._clock_anchor_wall) + sim_elapsed
        else:
            pub_time_s = float(sim_time_s)

        # Prevent backward timestamps that may trigger TF buffer reset warnings.
        if self._last_pub_time_s is not None and pub_time_s < self._last_pub_time_s:
            pub_time_s = self._last_pub_time_s
        self._last_pub_time_s = pub_time_s
        return pub_time_s

    def start(self) -> None:
        import rclpy
        from isaacsim.core.prims import SingleArticulation
        from rclpy.qos import DurabilityPolicy
        from rclpy.qos import HistoryPolicy
        from rclpy.qos import QoSProfile
        from rclpy.qos import ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node = rclpy.create_node(self.node_name)
        self._joint_state_pub = self._node.create_publisher(JointState, self.joint_state_topic, qos)
        self._clock_pub = self._node.create_publisher(Clock, self.clock_topic, qos)
        self._joint_cmd_sub = self._node.create_subscription(JointState, self.joint_cmd_topic, self._on_joint_cmd, qos)

        self._art = SingleArticulation(self.articulation_prim)
        self._art.initialize()

        self._dof_names = list(self._art.dof_names)
        self._dof_name_to_idx = {name: idx for idx, name in enumerate(self._dof_names)}
        self._max_joint_velocity_limits = self._infer_joint_velocity_limits(
            self._dof_names, self.default_max_joint_velocity
        )

        print(f"[INFO] Python ROS2 bridge started: node={self.node_name}")
        print(f"[INFO] Python bridge articulation: {self.articulation_prim}, dof={len(self._dof_names)}")
        print(f"[INFO] Python bridge topics: cmd={self.joint_cmd_topic}, state={self.joint_state_topic}, clock={self.clock_topic}")

    def _on_joint_cmd(self, msg) -> None:
        dof = len(self._dof_names)
        if dof <= 0:
            return

        pos_cmd = None
        vel_cmd = None
        eff_cmd = None

        if msg.name:
            pos_work = np.full((dof,), np.nan, dtype=np.float64)
            vel_work = np.full((dof,), np.nan, dtype=np.float64)
            eff_work = np.full((dof,), np.nan, dtype=np.float64)

            for i, joint_name in enumerate(msg.name):
                idx = self._dof_name_to_idx.get(joint_name)
                if idx is None:
                    continue
                if i < len(msg.position):
                    pos_work[idx] = msg.position[i]
                if i < len(msg.velocity):
                    vel_work[idx] = msg.velocity[i]
                if i < len(msg.effort):
                    eff_work[idx] = msg.effort[i]

            if np.isfinite(pos_work).any():
                pos_cmd = pos_work
            if np.isfinite(vel_work).any():
                vel_cmd = vel_work
            if np.isfinite(eff_work).any():
                eff_cmd = eff_work
        else:
            if len(msg.position) == dof:
                pos_cmd = np.array(msg.position, dtype=np.float64)
            if len(msg.velocity) == dof:
                vel_cmd = np.array(msg.velocity, dtype=np.float64)
            if len(msg.effort) == dof:
                eff_cmd = np.array(msg.effort, dtype=np.float64)

        self._latest_pos_cmd = pos_cmd
        self._latest_vel_cmd = vel_cmd
        self._latest_eff_cmd = eff_cmd

    def _apply_latest_commands(self, sim_time_s: float) -> None:
        if self._art is None:
            return

        if self._latest_pos_cmd is not None:
            current = np.array(self._art.get_joint_positions(), dtype=np.float64)
            mask = np.isfinite(self._latest_pos_cmd)
            if self.smooth_position_commands and self._max_joint_velocity_limits is not None:
                if self._last_apply_sim_time_s is None:
                    dt = 1.0 / 240.0
                else:
                    dt = max(1e-4, sim_time_s - self._last_apply_sim_time_s)

                delta = self._latest_pos_cmd - current
                max_delta = np.minimum(self._max_joint_velocity_limits * dt, self.max_joint_step_rad)
                delta_clamped = np.clip(delta, -max_delta, max_delta)
                current[mask] = current[mask] + delta_clamped[mask]
            else:
                current[mask] = self._latest_pos_cmd[mask]
            self._art.set_joint_positions(current)

        # If position commands are active, avoid immediately overriding with raw velocity commands.
        if self._latest_vel_cmd is not None and self._latest_pos_cmd is None:
            current = np.array(self._art.get_joint_velocities(), dtype=np.float64)
            mask = np.isfinite(self._latest_vel_cmd)
            current[mask] = self._latest_vel_cmd[mask]
            self._art.set_joint_velocities(current)

        if self._latest_eff_cmd is not None:
            current = np.zeros((len(self._dof_names),), dtype=np.float64)
            mask = np.isfinite(self._latest_eff_cmd)
            current[mask] = self._latest_eff_cmd[mask]
            self._art.set_joint_efforts(current)

        self._last_apply_sim_time_s = sim_time_s

    def _publish_joint_state(self, pub_time_s: float) -> None:
        from sensor_msgs.msg import JointState

        if self._joint_state_pub is None or self._art is None:
            return

        sec, nsec = self._to_sec_nsec(pub_time_s)
        msg = JointState()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nsec
        msg.name = list(self._dof_names)
        msg.position = list(np.array(self._art.get_joint_positions(), dtype=np.float64))
        msg.velocity = list(np.array(self._art.get_joint_velocities(), dtype=np.float64))

        try:
            msg.effort = list(np.array(self._art.get_applied_joint_efforts(), dtype=np.float64))
        except Exception:
            msg.effort = [0.0] * len(self._dof_names)

        self._joint_state_pub.publish(msg)

    def _publish_clock(self, pub_time_s: float) -> None:
        from rosgraph_msgs.msg import Clock

        if self._clock_pub is None:
            return

        sec, nsec = self._to_sec_nsec(pub_time_s)
        msg = Clock()
        msg.clock.sec = sec
        msg.clock.nanosec = nsec
        self._clock_pub.publish(msg)

    def step(self, sim_time_s: float, sim_step: int) -> None:
        import rclpy

        if self._node is None:
            return

        rclpy.spin_once(self._node, timeout_sec=0.0)
        self._apply_latest_commands(sim_time_s=sim_time_s)

        if sim_step % self.publish_every_steps == 0:
            pub_time_s = self._resolve_publish_time(sim_time_s=sim_time_s)
            self._publish_joint_state(pub_time_s=pub_time_s)
            self._publish_clock(pub_time_s=pub_time_s)

    def shutdown(self) -> None:
        import rclpy

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def _configure_ros2_env(mode: str, distro: str) -> None:
    """Configure ROS2 runtime env for isaacsim.ros2.bridge.

    - internal: use Isaac Sim bundled ROS2 libs (recommended for this py311 env)
    - system: keep current shell ROS env untouched
    - none: do not touch env (debug only)
    """
    if mode in ("system", "none"):
        print(f"[INFO] ROS2 env mode: {mode}")
        return

    import isaacsim

    isaacsim_root = Path(isaacsim.__file__).resolve().parent
    bridge_ext = isaacsim_root / "exts" / "isaacsim.ros2.bridge"
    ros_lib = bridge_ext / distro / "lib"
    ros_rclpy = bridge_ext / distro / "rclpy"

    if not ros_lib.exists():
        print(f"[WARN] Internal ROS2 lib path does not exist: {ros_lib}")
        return

    os.environ["ROS_DISTRO"] = distro
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    # Avoid importing system ROS python packages (usually py3.10) in this py3.11 runtime.
    current_py = os.environ.get("PYTHONPATH", "")
    kept_paths = []
    for p in current_py.split(":") if current_py else []:
        if "/opt/ros/" in p:
            continue
        kept_paths.append(p)

    current_ld = os.environ.get("LD_LIBRARY_PATH", "")
    ros_lib_str = ros_lib.as_posix()
    if ros_lib_str not in current_ld.split(":"):
        os.environ["LD_LIBRARY_PATH"] = f"{current_ld}:{ros_lib_str}" if current_ld else ros_lib_str

    ros_rclpy_str = ros_rclpy.as_posix()
    if ros_rclpy_str not in kept_paths:
        kept_paths.append(ros_rclpy_str)
    os.environ["PYTHONPATH"] = ":".join([p for p in kept_paths if p])

    # Also sanitize and update current interpreter search path immediately.
    sys.path = [p for p in sys.path if "/opt/ros/" not in p]
    bridge_ext_str = bridge_ext.as_posix()
    if bridge_ext_str not in sys.path:
        sys.path.append(bridge_ext_str)
    if ros_rclpy_str not in sys.path:
        sys.path.append(ros_rclpy_str)

    print(f"[INFO] ROS2 env mode: internal ({distro})")
    print(f"[INFO] Added internal ROS2 lib: {ros_lib_str}")


def _try_enable_ros2_bridge() -> str:
    """Try enabling ROS2 bridge extension with common extension names."""
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for ext_name in ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"):
        ext_id = manager.get_enabled_extension_id(ext_name)
        if ext_id:
            return ext_name
        try:
            if manager.set_extension_enabled_immediate(ext_name, True):
                return ext_name
        except Exception:
            continue
    return ""


def _normalize_stage_identifier(identifier: str) -> str:
    if identifier.startswith("file://"):
        return urllib.parse.unquote(identifier[len("file://") :])
    return identifier


def _wait_until_stage_opened(target_usd: Path, max_wait_s: float = 30.0):
    import omni.kit.app
    import omni.usd

    app = omni.kit.app.get_app()
    start = time.time()
    target_path = target_usd.resolve().as_posix()

    while time.time() - start < max_wait_s:
        app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            continue

        root_layer = stage.GetRootLayer()
        if root_layer is None:
            continue

        root_candidates = [
            (root_layer.realPath or "").strip(),
            _normalize_stage_identifier((root_layer.identifier or "").strip()),
        ]
        for root in root_candidates:
            if not root:
                continue
            try:
                root_norm = Path(root).resolve().as_posix()
            except Exception:
                root_norm = root
            if root_norm == target_path:
                return stage

    return None


def _inspect_graph_prims(stage) -> None:
    """Print lightweight graph-related prim diagnostics."""
    graph_like = []
    ros_like = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        tname = prim.GetTypeName() or ""
        low_path = path.lower()
        low_type = tname.lower()
        if "graph" in low_path or "graph" in low_type:
            graph_like.append((path, tname))
        if "ros2" in low_path or "ros" in low_type:
            ros_like.append((path, tname))

    print(f"[INFO] Graph-like prims: {len(graph_like)}")
    for path, tname in graph_like[:20]:
        print(f"  - {path} ({tname})")
    if len(graph_like) > 20:
        print(f"  ... ({len(graph_like) - 20} more)")

    print(f"[INFO] ROS-like prims: {len(ros_like)}")
    for path, tname in ros_like[:20]:
        print(f"  - {path} ({tname})")
    if len(ros_like) > 20:
        print(f"  ... ({len(ros_like) - 20} more)")


def _parse_vec3(text: str, name: str) -> tuple[float, float, float]:
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name} expects 3 comma-separated values, got: {text}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def _flatten_config_dict(data: dict, out: dict | None = None) -> dict:
    if out is None:
        out = {}
    for key, value in data.items():
        if isinstance(value, dict):
            _flatten_config_dict(value, out)
        else:
            out[key] = value
    return out


def _load_param_file_defaults(param_file: str) -> dict:
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

    if not isinstance(raw, dict):
        raise ValueError(f"Param file must contain a YAML mapping object: {file_path}")

    defaults = _flatten_config_dict(raw)
    print(f"[INFO] Loaded parameter profile: {file_path}")
    return defaults


def _upsert_demo_cube(
    stage,
    prim_path: str,
    size_xyz: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    color_rgb: tuple[float, float, float],
) -> None:
    """Create/update a cube in Isaac Sim to mirror MoveIt PlanningScene object.

    The cube is created as a static collider so it participates in simulation collisions.
    """
    from pxr import Gf
    from pxr import UsdGeom
    from pxr import UsdPhysics

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube_prim = cube.GetPrim()

    # Cube side length defaults to 2.0 in USD, so half-size scales map directly.
    cube.GetSizeAttr().Set(2.0)
    xform = UsdGeom.XformCommonAPI(cube_prim)
    xform.SetTranslate(Gf.Vec3d(center_xyz[0], center_xyz[1], center_xyz[2]))
    xform.SetScale(Gf.Vec3f(size_xyz[0] * 0.5, size_xyz[1] * 0.5, size_xyz[2] * 0.5))

    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])

    # Mark as collider so physics contacts can be resolved in simulation.
    UsdPhysics.CollisionAPI.Apply(cube_prim)

    print(
        "[INFO] Demo cube synced in Isaac Sim: "
        f"prim={prim_path}, center={center_xyz}, size={size_xyz}, color={color_rgb}"
    )


def _setup_ros2_joint_bridge_graph(
    articulation_prim: str,
    joint_cmd_topic: str,
    joint_state_topic: str,
    clock_topic: str,
) -> bool:
    """Create an OmniGraph bridge for MoveIt topic_based_ros2_control.

    Bridge behavior:
    - Subscribe joint commands from ROS2 topic (default: /joint_commands)
    - Apply commands to articulation
    - Publish joint states (default: /joint_states)
    - Publish /clock
    """
    import omni.graph.core as og

    import omni.usd

    keys = og.Controller.Keys
    stage = omni.usd.get_context().get_stage()
    has_action_graph = stage is not None and stage.GetPrimAtPath("/ActionGraph").IsValid()

    # Prefer dedicated runtime graph paths to avoid collisions with existing scene graphs.
    ts = int(time.time() * 1000) % 1000000
    parent_prim_path = articulation_prim.rstrip("/") or "/"
    graph_specs = [
        {"graph_path": f"{parent_prim_path}/MoveItBridgeRuntimeGraph_{ts}_0", "evaluator_name": "execution"},
        {"graph_path": f"{parent_prim_path}/MoveItBridgeRuntimeGraph_{ts}_1", "evaluator_name": "execution"},
    ]
    if has_action_graph:
        graph_specs.append({"graph_path": "/ActionGraph", "evaluator_name": "execution"})

    node_setups = [
        {
            "prefix": "isaacsim.ros2.bridge",
            "read_time": "omni.isaac.core_nodes.IsaacReadSimulationTime",
            "art_ctrl": "omni.isaac.core_nodes.IsaacArticulationController",
        },
        {
            "prefix": "omni.isaac.ros2_bridge",
            "read_time": "omni.isaac.core_nodes.IsaacReadSimulationTime",
            "art_ctrl": "omni.isaac.core_nodes.IsaacArticulationController",
        },
    ]

    for setup in node_setups:
        ros2_prefix = setup["prefix"]
        try:
            for graph_spec in graph_specs:
                try:
                    suffix = int(time.time() * 1000) % 1000000
                    og.Controller.edit(
                        graph_spec,
                        {
                            keys.CREATE_NODES: [
                                (f"RuntimeOnPlaybackTick_{suffix}", "omni.graph.action.OnPlaybackTick"),
                                (f"RuntimeReadSimTime_{suffix}", setup["read_time"]),
                                (f"RuntimeROS2Context_{suffix}", f"{ros2_prefix}.ROS2Context"),
                                (f"RuntimePublishClock_{suffix}", f"{ros2_prefix}.ROS2PublishClock"),
                                (f"RuntimePublishJointState_{suffix}", f"{ros2_prefix}.ROS2PublishJointState"),
                                (f"RuntimeSubscribeJointState_{suffix}", f"{ros2_prefix}.ROS2SubscribeJointState"),
                                (f"RuntimeArticulationController_{suffix}", setup["art_ctrl"]),
                            ],
                            keys.CONNECT: [
                                (
                                    f"RuntimeOnPlaybackTick_{suffix}.outputs:tick",
                                    f"RuntimePublishClock_{suffix}.inputs:execIn",
                                ),
                                (
                                    f"RuntimeOnPlaybackTick_{suffix}.outputs:tick",
                                    f"RuntimePublishJointState_{suffix}.inputs:execIn",
                                ),
                                (
                                    f"RuntimeOnPlaybackTick_{suffix}.outputs:tick",
                                    f"RuntimeSubscribeJointState_{suffix}.inputs:execIn",
                                ),
                                (
                                    f"RuntimeOnPlaybackTick_{suffix}.outputs:tick",
                                    f"RuntimeArticulationController_{suffix}.inputs:execIn",
                                ),
                                (
                                    f"RuntimeReadSimTime_{suffix}.outputs:simulationTime",
                                    f"RuntimePublishClock_{suffix}.inputs:timeStamp",
                                ),
                                (
                                    f"RuntimeROS2Context_{suffix}.outputs:context",
                                    f"RuntimePublishClock_{suffix}.inputs:context",
                                ),
                                (
                                    f"RuntimeROS2Context_{suffix}.outputs:context",
                                    f"RuntimePublishJointState_{suffix}.inputs:context",
                                ),
                                (
                                    f"RuntimeROS2Context_{suffix}.outputs:context",
                                    f"RuntimeSubscribeJointState_{suffix}.inputs:context",
                                ),
                                (
                                    f"RuntimeSubscribeJointState_{suffix}.outputs:jointNames",
                                    f"RuntimeArticulationController_{suffix}.inputs:jointNames",
                                ),
                                (
                                    f"RuntimeSubscribeJointState_{suffix}.outputs:positionCommand",
                                    f"RuntimeArticulationController_{suffix}.inputs:positionCommand",
                                ),
                                (
                                    f"RuntimeSubscribeJointState_{suffix}.outputs:velocityCommand",
                                    f"RuntimeArticulationController_{suffix}.inputs:velocityCommand",
                                ),
                                (
                                    f"RuntimeSubscribeJointState_{suffix}.outputs:effortCommand",
                                    f"RuntimeArticulationController_{suffix}.inputs:effortCommand",
                                ),
                            ],
                            keys.SET_VALUES: [
                                (f"RuntimePublishClock_{suffix}.inputs:topicName", clock_topic),
                                (f"RuntimePublishJointState_{suffix}.inputs:topicName", joint_state_topic),
                                (f"RuntimeSubscribeJointState_{suffix}.inputs:topicName", joint_cmd_topic),
                                (f"RuntimePublishJointState_{suffix}.inputs:targetPrim", [articulation_prim]),
                                (f"RuntimeArticulationController_{suffix}.inputs:targetPrim", [articulation_prim]),
                            ],
                        },
                    )
                    print(f"[INFO] ROS2 bridge graph path: {graph_spec['graph_path']}")
                    return True
                except Exception:
                    continue
        except Exception:
            continue

    return False


def main() -> int:
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--param-file", type=str, default="", help="YAML file with default launch parameters")
    pre_args, _ = pre_parser.parse_known_args()
    yaml_defaults = _load_param_file_defaults(pre_args.param_file) if pre_args.param_file else {}

    parser = argparse.ArgumentParser(description="Run combined_car_franka.usd in headless Isaac Sim")
    parser.add_argument(
        "--param-file",
        type=str,
        default="",
        help="YAML file with default launch parameters. CLI args override YAML values.",
    )
    parser.add_argument(
        "--usd",
        type=str,
        default=(
            Path(__file__).resolve().parents[2]
            / "ros2_ws/src/robot_description/urdf/combined_car_franka/combined_car_franka.usd"
        ).as_posix(),
        help="Absolute path to USD scene",
    )
    parser.add_argument("--steps", type=int, default=3000, help="Simulation steps to run")
    gui_group = parser.add_mutually_exclusive_group()
    gui_group.add_argument("--headless", dest="headless", action="store_true", help="Run in headless mode")
    gui_group.add_argument("--gui", dest="headless", action="store_false", help="Run with Isaac Sim GUI")
    parser.set_defaults(headless=True)
    parser.add_argument("--render-interval", type=int, default=100, help="Print interval in steps")
    parser.add_argument(
        "--setup-ros2-joint-bridge",
        action="store_true",
        help="Create /joint_commands <-> articulation <-> /joint_states bridge graph for MoveIt topic_based_ros2_control",
    )
    parser.add_argument(
        "--setup-ros2-python-bridge",
        action="store_true",
        help="Create pure Python ROS2 bridge (/joint_commands, /joint_states, /clock) without OmniGraph",
    )
    parser.add_argument(
        "--articulation-prim",
        type=str,
        default="/mobile_base_with_franka",
        help="Articulation prim path in stage",
    )
    parser.add_argument("--joint-commands-topic", type=str, default="/joint_commands")
    parser.add_argument("--joint-states-topic", type=str, default="/joint_states")
    parser.add_argument("--clock-topic", type=str, default="/clock")
    parser.add_argument("--ros2-node-name", type=str, default="isaac_python_joint_bridge")
    parser.add_argument(
        "--publish-every-steps",
        type=int,
        default=1,
        help="Publish /joint_states and /clock every N simulation steps",
    )
    parser.add_argument(
        "--clock-mode",
        type=str,
        default="wall-anchored",
        choices=["wall-anchored", "sim"],
        help="Timestamp mode for /clock and /joint_states headers",
    )
    parser.add_argument(
        "--disable-position-command-smoothing",
        action="store_true",
        help="Disable gradual position-command application and revert to direct set_joint_positions behavior",
    )
    parser.add_argument(
        "--default-max-joint-velocity",
        type=float,
        default=2.0,
        help="Fallback max joint velocity (rad/s) for joints without explicit limits",
    )
    parser.add_argument(
        "--max-joint-step-rad",
        type=float,
        default=0.03,
        help="Absolute per-step cap (rad) when smoothing position commands",
    )
    parser.add_argument(
        "--ros2-env-mode",
        type=str,
        default="internal",
        choices=["internal", "system", "none"],
        help="How to configure ROS2 env before launching app",
    )
    parser.add_argument("--ros2-distro", type=str, default="humble", choices=["humble", "jazzy"])
    parser.add_argument("--check-rclpy", action="store_true", help="Try importing rclpy after enabling ROS2 bridge")
    parser.add_argument(
        "--inspect-graph",
        action="store_true",
        help="Inspect stage graph-like/ROS-like prims after stage load",
    )
    parser.add_argument(
        "--launcher-backend",
        type=str,
        default="auto",
        choices=["auto", "isaaclab", "isaacsim"],
        help="Launcher backend: auto(gui->isaacsim, headless->isaaclab)",
    )
    parser.add_argument(
        "--sync-mtc-demo-object",
        action="store_true",
        help="Create/update demo cube in Isaac Sim so MTC PlanningScene object has scene counterpart",
    )
    parser.add_argument(
        "--demo-object-prim",
        type=str,
        default="/World/demo_cube",
        help="USD prim path for demo cube in Isaac Sim",
    )
    parser.add_argument(
        "--demo-object-size",
        type=str,
        default="0.04,0.04,0.12",
        help="Demo cube size xyz in meters",
    )
    parser.add_argument(
        "--demo-object-pick-position",
        type=str,
        default="0.55,0.0,0.60",
        help="Demo cube center xyz in world frame",
    )
    parser.add_argument(
        "--demo-object-color",
        type=str,
        default="0.9,0.3,0.1",
        help="Demo cube display color rgb in range [0,1]",
    )
    if yaml_defaults:
        parser.set_defaults(**yaml_defaults)

    args = parser.parse_args()

    usd_path = Path(args.usd).expanduser().resolve()
    if not usd_path.exists():
        print(f"[ERROR] USD file not found: {usd_path}")
        return 2

    ros2_required = args.setup_ros2_joint_bridge or args.setup_ros2_python_bridge or args.check_rclpy
    if ros2_required:
        _configure_ros2_env(mode=args.ros2_env_mode, distro=args.ros2_distro)
    else:
        print("[INFO] ROS2 bridge not requested; skipping ROS2 env configuration")

    print(f"[INFO] Launch mode: {'headless' if args.headless else 'gui'}")

    backend = args.launcher_backend
    if backend == "auto":
        backend = "isaaclab" if args.headless else "isaacsim"
    print(f"[INFO] Launcher backend: {backend}")

    if backend == "isaacsim":
        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": bool(args.headless)})
    else:
        launcher = AppLauncher(headless=args.headless)
        simulation_app = launcher.app

    import omni.kit.app
    import omni.timeline
    import omni.usd

    graph_bridge_required = args.setup_ros2_joint_bridge
    if graph_bridge_required:
        enabled_bridge = _try_enable_ros2_bridge()
        if enabled_bridge:
            print(f"[INFO] ROS2 bridge enabled: {enabled_bridge}")
        else:
            print("[WARN] ROS2 bridge extension was not enabled. If your USD has embedded ActionGraph this may still work.")
    elif args.setup_ros2_python_bridge:
        print("[INFO] Python ROS2 bridge mode: skip OmniGraph ROS2 extension enabling")

    if args.check_rclpy:
        ros_paths = [p for p in sys.path if "ros" in p.lower() or "rclpy" in p.lower()]
        print(f"[INFO] ROS-related sys.path entries ({len(ros_paths)}):")
        for p in ros_paths:
            print(f"  - {p}")
        try:
            import rclpy  # noqa: F401

            print("[INFO] rclpy import: OK")
        except Exception as exc:
            print(f"[WARN] rclpy import failed: {exc}")

    print(f"[INFO] Opening stage: {usd_path}")
    open_ok = omni.usd.get_context().open_stage(usd_path.as_posix())
    if open_ok is False:
        print(f"[ERROR] open_stage returned False for: {usd_path}")
        simulation_app.close()
        return 3

    stage = _wait_until_stage_opened(target_usd=usd_path, max_wait_s=45.0)
    if stage is None:
        print(f"[ERROR] Target stage failed to open in time: {usd_path}")
        simulation_app.close()
        return 3

    prim_count = sum(1 for _ in stage.Traverse())
    print(f"[INFO] Stage loaded, prim count: {prim_count}")

    if args.sync_mtc_demo_object:
        try:
            demo_size = _parse_vec3(args.demo_object_size, "--demo-object-size")
            demo_center = _parse_vec3(args.demo_object_pick_position, "--demo-object-pick-position")
            demo_color = _parse_vec3(args.demo_object_color, "--demo-object-color")
            _upsert_demo_cube(
                stage=stage,
                prim_path=args.demo_object_prim,
                size_xyz=demo_size,
                center_xyz=demo_center,
                color_rgb=demo_color,
            )
        except Exception as exc:
            print(f"[ERROR] Failed to sync demo cube in Isaac Sim: {exc}")
            simulation_app.close()
            return 4

    if args.inspect_graph:
        _inspect_graph_prims(stage)

    if args.setup_ros2_joint_bridge:
        ok = _setup_ros2_joint_bridge_graph(
            articulation_prim=args.articulation_prim,
            joint_cmd_topic=args.joint_commands_topic,
            joint_state_topic=args.joint_states_topic,
            clock_topic=args.clock_topic,
        )
        if ok:
            print("[INFO] ROS2 MoveIt joint bridge graph created")
            print(
                f"[INFO] Bridge topics: cmd={args.joint_commands_topic}, state={args.joint_states_topic}, clock={args.clock_topic}"
            )
        else:
            print("[WARN] Failed to create ROS2 MoveIt joint bridge graph. Scene will still run.")

    python_bridge = None
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    if args.setup_ros2_python_bridge:
        # Wait a few updates after play() so physics views are available.
        app = omni.kit.app.get_app()
        for _ in range(10):
            app.update()

        python_bridge = _PythonRos2JointBridge(
            articulation_prim=args.articulation_prim,
            joint_cmd_topic=args.joint_commands_topic,
            joint_state_topic=args.joint_states_topic,
            clock_topic=args.clock_topic,
            node_name=args.ros2_node_name,
            publish_every_steps=args.publish_every_steps,
            clock_mode=args.clock_mode,
            smooth_position_commands=not args.disable_position_command_smoothing,
            default_max_joint_velocity=args.default_max_joint_velocity,
            max_joint_step_rad=args.max_joint_step_rad,
        )
        python_bridge.start()

    app = omni.kit.app.get_app()
    for i in range(args.steps):
        app.update()
        if python_bridge is not None:
            sim_time_s = timeline.get_current_time()
            python_bridge.step(sim_time_s=sim_time_s, sim_step=i)
        if i % max(1, args.render_interval) == 0:
            print(f"[SIM] step={i}")

    timeline.stop()
    if python_bridge is not None:
        python_bridge.shutdown()
    print("[INFO] Simulation finished normally")
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())