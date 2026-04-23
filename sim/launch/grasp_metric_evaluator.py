#!/usr/bin/env python3
"""Replicator-based grasp metric evaluation for Isaac Sim trials.

This module records object and gripper pose streams via Replicator annotators,
collects contact information, and computes simple grasp metrics without using
UsdGeom-based manual transform math.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

import numpy as np

try:
    import pandas as pd
except Exception:  # pragma: no cover - pandas is optional at runtime
    pd = None


@dataclass(slots=True)
class GraspMetricResult:
    """Summary returned by :meth:`GraspMetricEvaluator.compute_metrics`."""

    success: bool
    final_z: float
    target_z: float
    stability_z_std: float
    slippage_detected: bool
    slippage_count: int
    total_samples: int
    hold_samples: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "final_z": self.final_z,
            "target_z": self.target_z,
            "stability_z_std": self.stability_z_std,
            "slippage_detected": self.slippage_detected,
            "slippage_count": self.slippage_count,
            "total_samples": self.total_samples,
            "hold_samples": self.hold_samples,
        }


class GraspMetricEvaluator:
    """Record and evaluate grasp metrics using Replicator annotators.

    Parameters
    ----------
    object_prim_path:
        USD prim path of the grasped object.
    gripper_prim_path:
        USD prim path of the gripper or end-effector link.
    table_prim_path:
        Optional USD prim path of the support surface.
    physics_sim_view:
        Optional omni.physx view object used as a fallback for contact queries.
    slippage_velocity_drop_threshold:
        Negative delta in object Z velocity that will be treated as a slippage
        event while the gripper is closed.
    """

    def __init__(
        self,
        object_prim_path: str,
        gripper_prim_path: str,
        table_prim_path: str | None = None,
        physics_sim_view: Any | None = None,
        slippage_velocity_drop_threshold: float = -0.02,
    ) -> None:
        self.object_prim_path = object_prim_path
        self.gripper_prim_path = gripper_prim_path
        self.table_prim_path = table_prim_path
        self.physics_sim_view = physics_sim_view
        self.slippage_velocity_drop_threshold = float(slippage_velocity_drop_threshold)

        self._recording = False
        self._update_subscription = None
        self._gripper_closed = False
        self._hold_window: tuple[float, float] | None = None

        self._object_pose_annotator = None
        self._gripper_pose_annotator = None
        self._contact_annotator = None
        self._object_pose_annotator_name: str | None = None
        self._gripper_pose_annotator_name: str | None = None
        self._contact_annotator_name: str | None = None
        self._pose_fallback_warned: set[str] = set()

        self._rows: list[dict[str, Any]] = []
        self._last_object_position: np.ndarray | None = None
        self._last_gripper_position: np.ndarray | None = None
        self._last_sample_sim_time: float | None = None

    def set_gripper_closed(self, closed: bool) -> None:
        """Set the current gripper state that will be recorded with each sample."""

        self._gripper_closed = bool(closed)

    def set_hold_window(self, start_time_s: float, end_time_s: float) -> None:
        """Manually define the hold phase time window used by metrics."""

        start = float(start_time_s)
        end = float(end_time_s)
        if end < start:
            raise ValueError("hold window end time must be >= start time")
        self._hold_window = (start, end)

    def clear(self) -> None:
        """Clear all recorded samples and inferred state."""

        self._rows.clear()
        self._last_object_position = None
        self._last_gripper_position = None
        self._last_sample_sim_time = None

    def start_recording(self, clear_previous: bool = True) -> None:
        """Attach annotators and subscribe to the Isaac Sim update stream."""

        if self._recording:
            return

        if clear_previous:
            self.clear()

        rep = self._get_replicator_module()
        self._object_pose_annotator = self._create_pose_annotator(rep, self.object_prim_path)
        self._gripper_pose_annotator = self._create_pose_annotator(rep, self.gripper_prim_path)
        self._contact_annotator = self._create_contact_annotator(rep, self.object_prim_path)

        if self._object_pose_annotator is None:
            print(
                "[WARN] GraspMetricEvaluator did not find a usable pose annotator for "
                f"object '{self.object_prim_path}'."
            )
        if self._gripper_pose_annotator is None:
            print(
                "[WARN] GraspMetricEvaluator did not find a usable pose annotator for "
                f"gripper '{self.gripper_prim_path}'."
            )
        if self._contact_annotator is None:
            print(
                "[WARN] GraspMetricEvaluator did not find a usable contact annotator; "
                "contact labels will use physics_sim_view fallback when available."
            )

        import omni.kit.app

        app = omni.kit.app.get_app()
        event_stream = app.get_update_event_stream()
        self._update_subscription = event_stream.create_subscription_to_pop(self._on_update, name="grasp_metric_evaluator_update")
        self._recording = True

    def stop_recording(self, as_dataframe: bool = True):
        """Stop the simulation subscription and optionally return the recorded samples."""

        if self._update_subscription is not None:
            unsubscribe = getattr(self._update_subscription, "unsubscribe", None)
            if callable(unsubscribe):
                try:
                    unsubscribe()
                except Exception:
                    pass
            self._update_subscription = None

        self._recording = False
        if as_dataframe:
            return self.to_dataframe()
        return self.records

    @property
    def records(self) -> list[dict[str, Any]]:
        """Return a copy of the raw recorded rows."""

        return list(self._rows)

    def get_records_slice(self, start_index: int = 0) -> list[dict[str, Any]]:
        """Return a copy of rows in ``[start_index, end)`` for incremental export."""

        start = max(0, int(start_index))
        return list(self._rows[start:])

    def to_dataframe(self):
        """Convert the recorded rows into a pandas DataFrame or NumPy array."""

        if pd is not None:
            return pd.DataFrame.from_records(self._rows)

        dtype = [
            ("sim_time_s", np.float64),
            ("wall_time_s", np.float64),
            ("object_x", np.float64),
            ("object_y", np.float64),
            ("object_z", np.float64),
            ("object_vx", np.float64),
            ("object_vy", np.float64),
            ("object_vz", np.float64),
            ("gripper_x", np.float64),
            ("gripper_y", np.float64),
            ("gripper_z", np.float64),
            ("gripper_vx", np.float64),
            ("gripper_vy", np.float64),
            ("gripper_vz", np.float64),
            ("contact_gripper", np.bool_),
            ("contact_table", np.bool_),
            ("gripper_closed", np.bool_),
            ("sample_index", np.int64),
        ]
        rows = []
        for row in self._rows:
            rows.append(tuple(row.get(name) for name, _ in dtype))
        return np.array(rows, dtype=dtype)

    def compute_metrics(self, target_z: float, hold_window: tuple[float, float] | None = None) -> GraspMetricResult:
        """Compute success, stability, and slippage metrics from the recorded data."""

        frame = self.to_dataframe()
        if self._frame_is_empty(frame):
            return GraspMetricResult(
                success=False,
                final_z=float("nan"),
                target_z=float(target_z),
                stability_z_std=float("nan"),
                slippage_detected=False,
                slippage_count=0,
                total_samples=0,
                hold_samples=0,
            )

        z_values = self._frame_column(frame, "object_z")
        final_z = float(z_values[-1])
        success = final_z > float(target_z)

        hold_mask = self._resolve_hold_mask(frame, hold_window=hold_window)
        hold_z = z_values[hold_mask]
        stability_z_std = float(np.std(hold_z)) if hold_z.size > 0 else float("nan")

        z_velocity = self._frame_column(frame, "object_vz")
        gripper_closed = self._frame_bool_column(frame, "gripper_closed")
        contact_table = self._frame_bool_column(frame, "contact_table")
        dz_velocity = np.diff(z_velocity, prepend=z_velocity[0])
        slip_mask = (
            hold_mask
            & gripper_closed
            & ~contact_table
            & (dz_velocity < self.slippage_velocity_drop_threshold)
        )

        return GraspMetricResult(
            success=success,
            final_z=final_z,
            target_z=float(target_z),
            stability_z_std=stability_z_std,
            slippage_detected=bool(np.any(slip_mask)),
            slippage_count=int(np.count_nonzero(slip_mask)),
            total_samples=int(z_values.size),
            hold_samples=int(np.count_nonzero(hold_mask)),
        )

    def _on_update(self, _event) -> None:
        if not self._recording:
            return

        sim_time_s = self._current_sim_time_s()
        wall_time_s = time.time()

        object_position = self._read_pose_position(self._object_pose_annotator, self.object_prim_path)
        gripper_position = self._read_pose_position(self._gripper_pose_annotator, self.gripper_prim_path)
        contact_pairs = self._read_contact_pairs()

        contact_gripper = self._has_contact_with(contact_pairs, self.gripper_prim_path)
        contact_table = bool(self.table_prim_path and self._has_contact_with(contact_pairs, self.table_prim_path))

        object_velocity = self._estimate_velocity(object_position, self._last_object_position, sim_time_s)
        gripper_velocity = self._estimate_velocity(gripper_position, self._last_gripper_position, sim_time_s)

        self._rows.append(
            {
                "sim_time_s": sim_time_s,
                "wall_time_s": wall_time_s,
                "object_x": float(object_position[0]),
                "object_y": float(object_position[1]),
                "object_z": float(object_position[2]),
                "object_vx": float(object_velocity[0]),
                "object_vy": float(object_velocity[1]),
                "object_vz": float(object_velocity[2]),
                "gripper_x": float(gripper_position[0]),
                "gripper_y": float(gripper_position[1]),
                "gripper_z": float(gripper_position[2]),
                "gripper_vx": float(gripper_velocity[0]),
                "gripper_vy": float(gripper_velocity[1]),
                "gripper_vz": float(gripper_velocity[2]),
                "contact_gripper": bool(contact_gripper),
                "contact_table": bool(contact_table),
                "gripper_closed": bool(self._gripper_closed),
                "sample_index": len(self._rows),
            }
        )

        self._last_object_position = object_position
        self._last_gripper_position = gripper_position
        self._last_sample_sim_time = sim_time_s

    def _get_replicator_module(self):
        import omni.replicator.core as rep

        return rep

    def _create_pose_annotator(self, rep, prim_path: str):
        candidates = ("pose", "IsaacReadWorldPose")
        for name in candidates:
            try:
                annotator = rep.AnnotatorRegistry.get_annotator(name)
            except Exception:
                continue

            try:
                self._attach_annotator(annotator, prim_path)
            except Exception:
                continue

            if prim_path == self.object_prim_path:
                self._object_pose_annotator_name = name
            elif prim_path == self.gripper_prim_path:
                self._gripper_pose_annotator_name = name
            print(f"[INFO] GraspMetricEvaluator using pose annotator '{name}' for {prim_path}")
            return annotator

        self._log_available_annotators(rep, requested="pose")
        return None

    def _create_contact_annotator(self, rep, prim_path: str):
        candidates = ("contact", "Contact", "IsaacReadContactSensor")
        for name in candidates:
            try:
                annotator = rep.AnnotatorRegistry.get_annotator(name)
            except Exception:
                continue

            try:
                self._attach_annotator(annotator, prim_path)
            except Exception:
                continue

            self._contact_annotator_name = name
            print(f"[INFO] GraspMetricEvaluator using contact annotator '{name}'")
            return annotator

        self._log_available_annotators(rep, requested="contact")
        return None

    def _log_available_annotators(self, rep, requested: str) -> None:
        registry = getattr(rep, "AnnotatorRegistry", None)
        if registry is None:
            print(f"[WARN] AnnotatorRegistry missing while requesting '{requested}'")
            return

        available: list[str] = []
        for getter_name in ("get_annotator_names", "get_registered_annotators", "get_available_annotators"):
            getter = getattr(registry, getter_name, None)
            if not callable(getter):
                continue
            try:
                result = getter()
                if isinstance(result, dict):
                    available = sorted([str(k) for k in result.keys()])
                else:
                    available = sorted([str(item) for item in result])
                if available:
                    break
            except Exception:
                continue

        if available:
            print(
                f"[WARN] No usable '{requested}' annotator. "
                f"Available annotators: {available}"
            )
        else:
            print(f"[WARN] No usable '{requested}' annotator.")

    def _attach_annotator(self, annotator, prim_path: str) -> None:
        attach_attempts = (
            ([prim_path],),
            (prim_path,),
        )

        last_error: Exception | None = None
        for attempt in attach_attempts:
            try:
                annotator.attach(*attempt)
                return
            except Exception as exc:
                last_error = exc
                continue

        if last_error is not None:
            raise last_error

    def _current_sim_time_s(self) -> float:
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            current_time = float(timeline.get_current_time())
            if math.isfinite(current_time):
                return current_time
        except Exception:
            pass

        return float(time.time())

    def _read_pose_position(self, annotator, prim_path: str) -> np.ndarray:
        usd_position = self._read_prim_world_position(prim_path)

        if annotator is not None:
            payload = self._safe_get_annotator_data(annotator)
            position = self._extract_vec3_for_prim(payload, prim_path)
            if position is None:
                position = self._extract_vec3(payload)
            if position is not None:
                if (
                    usd_position is not None
                    and float(np.linalg.norm(position)) <= 1e-9
                    and float(np.linalg.norm(usd_position)) > 1e-6
                ):
                    if prim_path not in self._pose_fallback_warned:
                        print(
                            "[WARN] GraspMetricEvaluator pose annotator returned a zero vector; "
                            f"using USD world transform for {prim_path}"
                        )
                        self._pose_fallback_warned.add(prim_path)
                    return usd_position
                return position

        if usd_position is not None:
            if prim_path not in self._pose_fallback_warned:
                print(
                    "[WARN] GraspMetricEvaluator pose annotator returned no usable sample; "
                    f"fallback to USD world transform for {prim_path}"
                )
                self._pose_fallback_warned.add(prim_path)
            return usd_position

        # Fall back to the last known pose if the annotator has not produced a
        # fresh sample yet. This keeps the record shape stable across the first
        # few simulation frames.
        if prim_path == self.object_prim_path and self._last_object_position is not None:
            return self._last_object_position.copy()
        if prim_path == self.gripper_prim_path and self._last_gripper_position is not None:
            return self._last_gripper_position.copy()
        return self._zero_vec3()

    def _read_prim_world_position(self, prim_path: str) -> np.ndarray | None:
        try:
            import omni.usd
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                return None

            prim = stage.GetPrimAtPath(prim_path)
            if prim is None or not prim.IsValid():
                return None

            xform_cache = UsdGeom.XformCache()
            matrix = xform_cache.GetLocalToWorldTransform(prim)
            translation = matrix.ExtractTranslation()
            return np.array([float(translation[0]), float(translation[1]), float(translation[2])], dtype=np.float64)
        except Exception:
            return None

    def _read_contact_pairs(self) -> list[tuple[str, str]]:
        if self._contact_annotator is not None:
            payload = self._safe_get_annotator_data(self._contact_annotator)
            pairs = self._extract_contact_pairs(payload)
            if pairs:
                return pairs

        return self._query_contacts_from_physx_view()

    def _query_contacts_from_physx_view(self) -> list[tuple[str, str]]:
        view = self.physics_sim_view
        if view is None:
            return []

        method_names = ("get_contact_report", "get_contact_pairs", "get_contacts", "get_contact_data")
        for method_name in method_names:
            method = getattr(view, method_name, None)
            if not callable(method):
                continue
            try:
                payload = method()
            except TypeError:
                try:
                    payload = method(self.object_prim_path)
                except Exception:
                    continue
            except Exception:
                continue

            pairs = self._extract_contact_pairs(payload)
            if pairs:
                return pairs

        return []

    def _has_contact_with(self, contact_pairs: list[tuple[str, str]], target_path: str) -> bool:
        if not target_path:
            return False
        return any(self._path_matches(a, target_path) or self._path_matches(b, target_path) for a, b in contact_pairs)

    def _estimate_velocity(
        self,
        current_position: np.ndarray,
        last_position: np.ndarray | None,
        current_time_s: float,
    ) -> np.ndarray:
        if last_position is None or self._last_sample_sim_time is None:
            return self._zero_vec3()

        delta_t = float(current_time_s - self._last_sample_sim_time)
        if delta_t <= 1e-9:
            return self._zero_vec3()

        return (current_position - last_position) / delta_t

    def _resolve_hold_mask(
        self,
        frame,
        hold_window: tuple[float, float] | None = None,
    ) -> np.ndarray:
        timestamps = self._frame_column(frame, "sim_time_s")
        gripper_closed = self._frame_bool_column(frame, "gripper_closed")
        contact_gripper = self._frame_bool_column(frame, "contact_gripper")

        if hold_window is not None:
            start, end = hold_window
        elif self._hold_window is not None:
            start, end = self._hold_window
        else:
            hold_mask = gripper_closed | contact_gripper
            if np.any(hold_mask):
                return hold_mask
            if timestamps.size == 0:
                return np.zeros((0,), dtype=bool)
            pivot = int(max(0, math.floor(0.8 * timestamps.size)))
            mask = np.zeros_like(timestamps, dtype=bool)
            mask[pivot:] = True
            return mask

        return (timestamps >= float(start)) & (timestamps <= float(end))

    def _safe_get_annotator_data(self, annotator):
        try:
            return annotator.get_data()
        except Exception:
            return None

    def _extract_vec3(self, payload) -> np.ndarray | None:
        if payload is None:
            return None

        if isinstance(payload, dict):
            preferred_keys = (
                "translation",
                "position",
                "world_position",
                "worldPosition",
                "pos",
                "center",
            )
            for key in preferred_keys:
                if key in payload:
                    vec = self._extract_vec3(payload[key])
                    if vec is not None:
                        return vec
            for value in payload.values():
                vec = self._extract_vec3(value)
                if vec is not None:
                    return vec
            return None

        if isinstance(payload, np.ndarray):
            arr = np.asarray(payload, dtype=np.float64)
            return self._extract_vec3_from_array(arr)

        if isinstance(payload, (list, tuple)):
            try:
                arr = np.asarray(payload, dtype=np.float64)
            except Exception:
                for item in payload:
                    vec = self._extract_vec3(item)
                    if vec is not None:
                        return vec
                return None
            return self._extract_vec3_from_array(arr)

        if isinstance(payload, (int, float, np.number)):
            return None

        # Last-chance traversal for annotator payloads that wrap values in custom
        # objects but still expose a mapping-like interface.
        values = getattr(payload, "values", None)
        if callable(values):
            try:
                for value in values():
                    vec = self._extract_vec3(value)
                    if vec is not None:
                        return vec
            except Exception:
                pass

        return None

    def _extract_vec3_for_prim(self, payload, prim_path: str) -> np.ndarray | None:
        if payload is None or not prim_path:
            return None

        if isinstance(payload, dict):
            direct = payload.get(prim_path)
            if direct is not None:
                vec = self._extract_vec3(direct)
                if vec is not None:
                    return vec

            path_keys = ("primPaths", "prim_paths", "paths", "path")
            pose_keys = (
                "translations",
                "translation",
                "positions",
                "position",
                "worldPositions",
                "world_position",
                "poses",
                "pose",
                "transforms",
                "transform",
                "data",
            )

            paths = None
            for key in path_keys:
                if key in payload:
                    paths = payload.get(key)
                    break

            if paths is not None:
                try:
                    path_list = list(paths)
                except Exception:
                    path_list = []

                idx = None
                for i, candidate in enumerate(path_list):
                    candidate_str = self._stringify_contact_endpoint(candidate)
                    if candidate_str and self._path_matches(candidate_str, prim_path):
                        idx = i
                        break

                if idx is not None:
                    for key in pose_keys:
                        if key not in payload:
                            continue
                        data = payload.get(key)
                        try:
                            item = data[idx]
                        except Exception:
                            continue
                        vec = self._extract_vec3(item)
                        if vec is not None:
                            return vec

            for key, value in payload.items():
                if isinstance(key, str) and self._path_matches(key, prim_path):
                    vec = self._extract_vec3(value)
                    if vec is not None:
                        return vec

        return None

    def _extract_vec3_from_array(self, arr: np.ndarray) -> np.ndarray | None:
        if arr.ndim == 1:
            if arr.size == 3:
                return arr.astype(np.float64)
            if arr.size in (7, 8):
                return arr[:3].astype(np.float64)
            if arr.size == 16:
                matrix = arr.reshape(4, 4)
                if np.allclose(matrix[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-3):
                    return matrix[:3, 3].astype(np.float64)
                if np.allclose(matrix[:, 3], [0.0, 0.0, 0.0, 1.0], atol=1e-3):
                    return matrix[:3, 3].astype(np.float64)
                return matrix[:3, 3].astype(np.float64)

        if arr.ndim == 2 and arr.shape == (4, 4):
            if np.allclose(arr[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-3):
                return arr[:3, 3].astype(np.float64)
            if np.allclose(arr[:, 3], [0.0, 0.0, 0.0, 1.0], atol=1e-3):
                return arr[:3, 3].astype(np.float64)
            return arr[:3, 3].astype(np.float64)

        if arr.ndim >= 2 and arr.shape[-1] >= 3:
            flat = arr.reshape(-1, arr.shape[-1])
            first = flat[0]
            return first[:3].astype(np.float64)

        return None

    def _extract_contact_pairs(self, payload) -> list[tuple[str, str]]:
        pairs: list[tuple[str, str]] = []

        def walk(node: Any) -> None:
            if node is None:
                return

            if isinstance(node, dict):
                pair = self._contact_pair_from_dict(node)
                if pair is not None:
                    pairs.append(pair)
                for value in node.values():
                    walk(value)
                return

            if isinstance(node, (list, tuple, set)):
                for item in node:
                    walk(item)
                return

            if isinstance(node, np.ndarray):
                if node.dtype == object:
                    for item in node.flat:
                        walk(item)
                return

        walk(payload)

        unique_pairs: list[tuple[str, str]] = []
        seen: set[tuple[str, str]] = set()
        for pair in pairs:
            if pair in seen or (pair[1], pair[0]) in seen:
                continue
            seen.add(pair)
            unique_pairs.append(pair)
        return unique_pairs

    def _contact_pair_from_dict(self, node: dict[str, Any]) -> tuple[str, str] | None:
        key_variants = [
            ("body0", "body1"),
            ("bodyA", "bodyB"),
            ("path0", "path1"),
            ("prim0", "prim1"),
            ("primPath0", "primPath1"),
            ("contactA", "contactB"),
            ("object0", "object1"),
            ("a", "b"),
        ]

        for left_key, right_key in key_variants:
            if left_key in node and right_key in node:
                left = self._stringify_contact_endpoint(node[left_key])
                right = self._stringify_contact_endpoint(node[right_key])
                if left and right:
                    return left, right

        # Some contact payloads nest the endpoints under a pair-like structure.
        for key in ("contact", "pair", "contacts"):
            value = node.get(key)
            if isinstance(value, dict):
                pair = self._contact_pair_from_dict(value)
                if pair is not None:
                    return pair
        return None

    def _stringify_contact_endpoint(self, value: Any) -> str | None:
        if value is None:
            return None
        if isinstance(value, str):
            return value
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="ignore")
        if isinstance(value, dict):
            for key in ("path", "primPath", "body", "name", "object", "target"):
                if key in value:
                    stringified = self._stringify_contact_endpoint(value[key])
                    if stringified:
                        return stringified
            return None
        if isinstance(value, (list, tuple)) and value:
            # If the payload is a list of candidates, use the first string-like one.
            for item in value:
                stringified = self._stringify_contact_endpoint(item)
                if stringified:
                    return stringified
            return None
        return str(value)

    def _path_matches(self, candidate: str, target_path: str) -> bool:
        candidate_norm = candidate.replace("/", "").strip().lower()
        target_norm = target_path.replace("/", "").strip().lower()
        if not candidate_norm or not target_norm:
            return False
        if candidate_norm == target_norm:
            return True
        if target_norm in candidate_norm or candidate_norm in target_norm:
            return True

        candidate_leaf = candidate.split("/")[-1].lower()
        target_leaf = target_path.split("/")[-1].lower()
        return candidate_leaf == target_leaf

    def _frame_is_empty(self, frame) -> bool:
        if pd is not None and isinstance(frame, pd.DataFrame):
            return frame.empty
        return len(frame) == 0

    def _frame_column(self, frame, column_name: str) -> np.ndarray:
        if pd is not None and isinstance(frame, pd.DataFrame):
            return frame[column_name].to_numpy(dtype=np.float64)
        return np.asarray(frame[column_name], dtype=np.float64)

    def _frame_bool_column(self, frame, column_name: str) -> np.ndarray:
        if pd is not None and isinstance(frame, pd.DataFrame):
            return frame[column_name].to_numpy(dtype=bool)
        return np.asarray(frame[column_name], dtype=bool)

    def _zero_vec3(self) -> np.ndarray:
        return np.zeros((3,), dtype=np.float64)
