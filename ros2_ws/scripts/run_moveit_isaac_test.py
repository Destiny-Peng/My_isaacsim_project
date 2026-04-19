#!/usr/bin/env python3
"""Run moveit_robot_config/isaac_sim_moveit.launch.py with YAML defaults."""

from __future__ import annotations

import argparse
import shlex
import subprocess
from pathlib import Path


def _flatten_dict(data: dict, out: dict | None = None) -> dict:
    if out is None:
        out = {}
    for key, value in data.items():
        if isinstance(value, dict):
            _flatten_dict(value, out)
        else:
            out[key] = value
    return out


def _to_launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch MoveIt-Isaac integration from YAML profile")
    parser.add_argument("--param-file", required=True, help="YAML file with launch defaults")
    parser.add_argument("--dry-run", action="store_true", help="Print command only")
    args = parser.parse_args()

    try:
        import yaml
    except ImportError:
        raise RuntimeError("PyYAML is required to read --param-file")

    param_file = Path(args.param_file).expanduser().resolve()
    if not param_file.exists():
        raise FileNotFoundError(f"Param file not found: {param_file}")

    with open(param_file, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    launch_args = _flatten_dict(config)

    ws_root = Path(__file__).resolve().parents[1]
    setup_ros = "/opt/ros/humble/setup.bash"
    setup_ws = ws_root / "install" / "setup.bash"

    ros2_parts = [
        "ros2 launch moveit_robot_config isaac_sim_moveit.launch.py",
    ]
    for key in sorted(launch_args.keys()):
        ros2_parts.append(f"{key}:={_to_launch_value(launch_args[key])}")

    ros2_cmd = " ".join(ros2_parts)
    bash_cmd = (
        f"source {shlex.quote(setup_ros)} && "
        f"source {shlex.quote(str(setup_ws))} && "
        f"{ros2_cmd}"
    )

    print(f"[INFO] Using param file: {param_file}")
    print(f"[INFO] Command: {ros2_cmd}")
    if args.dry_run:
        return 0

    result = subprocess.run(["bash", "-lc", bash_cmd], check=False)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
