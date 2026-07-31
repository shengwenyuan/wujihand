#!/usr/bin/env python3
"""Create an ignored ROS local binding from an existing native device binding."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime import (  # noqa: E402
    ConfigRepository,
    RosProcessEnvironment,
    build_ros_local_runtime_binding,
)


def _runtime_path(value: str | Path) -> str:
    """Return an absolute runtime path without dereferencing venv symlinks."""
    return os.path.abspath(os.path.expanduser(str(value)))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native-binding", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vive-python", required=True)
    parser.add_argument("--glove-python", required=True)
    parser.add_argument("--isaac-python", required=True)
    parser.add_argument("--overlay-setup", required=True)
    parser.add_argument("--ros-domain-id", type=int, default=57)
    parser.add_argument(
        "--rmw-implementation",
        default="rmw_fastrtps_cpp",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    if output.exists():
        raise SystemExit(
            f"refusing to overwrite existing local binding: {output}"
        )
    native = ConfigRepository(ROOT).load_local_device_binding(
        args.native_binding
    )
    setup_scripts = (
        "/opt/ros/jazzy/setup.bash",
        _runtime_path(args.overlay_setup),
    )
    binding = build_ros_local_runtime_binding(
        native,
        binding_id="workstation2_nv5_ros_v2",
        ros_domain_id=args.ros_domain_id,
        rmw_implementation=args.rmw_implementation,
        vive=RosProcessEnvironment(
            executable=_runtime_path(args.vive_python),
            environment_id="workstation2_openvr_ros_jazzy_v1",
            setup_scripts=setup_scripts,
        ),
        glove=RosProcessEnvironment(
            executable=_runtime_path(args.glove_python),
            environment_id="workstation2_wuji_sdk_ros_jazzy_v1",
            setup_scripts=setup_scripts,
        ),
        isaac=RosProcessEnvironment(
            executable=_runtime_path(args.isaac_python),
            environment_id="workstation2_isaac_6_0_1_ros_jazzy_v1",
            setup_scripts=setup_scripts,
        ),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(
            binding.to_mapping(),
            allow_unicode=False,
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    print(f"created {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
