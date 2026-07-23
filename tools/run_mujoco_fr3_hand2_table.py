#!/usr/bin/env python3
"""Run the pinned FR3 v2 + Wuji Hand 2 right MuJoCo desktop scene."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation import MujocoFr3Hand2  # noqa: E402
from wujihand.adapters.simulation.mujoco_fr3_hand2 import (  # noqa: E402
    sha256_file,
    sha256_tree,
)
from wujihand.runtime import load_mujoco_table_scene_config  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene-profile",
        type=Path,
        default=ROOT / "configs/base/mujoco_fr3v2_hand2_right_table_v1.yaml",
    )
    parser.add_argument(
        "--duration-s", type=float, default=5.0, help="Positive simulated duration."
    )
    parser.add_argument("--gui", action="store_true", help="Open MuJoCo's passive viewer.")
    parser.add_argument(
        "--arm-target",
        type=float,
        nargs=7,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="Optional FR3 target in profile joint order; defaults to home.",
    )
    parser.add_argument(
        "--hand-target",
        type=float,
        nargs=20,
        metavar=tuple(f"Q{index}" for index in range(20)),
        help="Optional Hand 2 target in canonical q20 order; defaults to rest.",
    )
    parser.add_argument("--report", type=Path, help="Also write the JSON report to this path.")
    parser.add_argument(
        "--render-ppm",
        type=Path,
        help="Render the final overview camera to a dependency-free binary PPM file.",
    )
    return parser.parse_args()


def _write_ppm(path: Path, rgb: np.ndarray[Any, np.dtype[np.uint8]]) -> None:
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise RuntimeError("MuJoCo renderer returned an unexpected RGB buffer")
    path.parent.mkdir(parents=True, exist_ok=True)
    height, width, _ = rgb.shape
    with path.open("wb") as stream:
        stream.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        stream.write(rgb.tobytes())


def _run_gui(environment: MujocoFr3Hand2, tick_count: int) -> None:
    viewer_module = importlib.import_module("mujoco.viewer")
    period_s = 1.0 / environment.config.control.rate_hz
    next_wall_time = time.monotonic()
    with viewer_module.launch_passive(environment.model, environment.data) as viewer:
        for _ in range(tick_count):
            if not viewer.is_running():
                break
            environment.step()
            viewer.sync()
            next_wall_time += period_s
            time.sleep(max(0.0, next_wall_time - time.monotonic()))


def _state_json(state: Any) -> dict[str, Any]:
    return {
        "simulation_time_s": state.simulation_time_s,
        "contact_count": state.contact_count,
        "arm_q7": state.arm_q7.tolist(),
        "arm_dq7": state.arm_dq7.tolist(),
        "hand_q20": state.hand_q20.tolist(),
        "hand_dq20": state.hand_dq20.tolist(),
        "flange_position_m": state.flange_position_m.tolist(),
        "flange_quat_wxyz": state.flange_quat_wxyz.tolist(),
        "palm_position_m": state.palm_position_m.tolist(),
        "palm_quat_wxyz": state.palm_quat_wxyz.tolist(),
        "fingertip_positions_m": state.fingertip_positions_m.tolist(),
    }


def _relative_quaternion(parent_wxyz: np.ndarray, child_wxyz: np.ndarray) -> np.ndarray:
    """Return conjugate(parent) * child for unit wxyz quaternions."""

    pw, px, py, pz = parent_wxyz
    cw, cx, cy, cz = child_wxyz
    return np.asarray(
        [
            pw * cw + px * cx + py * cy + pz * cz,
            pw * cx - px * cw - py * cz + pz * cy,
            pw * cy + px * cz - py * cw - pz * cx,
            pw * cz - px * cy + py * cx - pz * cw,
        ],
        dtype=np.float64,
    )


def _relative_position(
    parent_position: np.ndarray,
    parent_wxyz: np.ndarray,
    child_position: np.ndarray,
) -> np.ndarray:
    """Express the child-parent offset in the parent frame."""

    w, x, y, z = parent_wxyz
    rotation = np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return rotation.T @ (child_position - parent_position)


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.duration_s) or args.duration_s <= 0.0:
        raise SystemExit("--duration-s must be finite and positive")
    config = load_mujoco_table_scene_config(args.scene_profile)
    environment = MujocoFr3Hand2.from_config(config, project_root=ROOT)
    arm_target = (
        environment.arm_profile.home_position if args.arm_target is None else args.arm_target
    )
    hand_target = (
        environment.hand_profile.rest_position if args.hand_target is None else args.hand_target
    )
    environment.set_joint_targets(arm_target, hand_target)
    initial = environment.observe()
    tick_count = int(round(args.duration_s * config.control.rate_hz))
    if tick_count < 1 or not np.isclose(
        tick_count / config.control.rate_hz, args.duration_s, rtol=0.0, atol=1e-12
    ):
        raise SystemExit("--duration-s must resolve to a whole 100 Hz control tick")
    if args.gui:
        _run_gui(environment, tick_count)
    else:
        environment.step(tick_count)
    final = environment.observe()

    initial_relative = _relative_position(
        initial.flange_position_m, initial.flange_quat_wxyz, initial.palm_position_m
    )
    final_relative = _relative_position(
        final.flange_position_m, final.flange_quat_wxyz, final.palm_position_m
    )
    relative_position_drift_m = float(np.linalg.norm(final_relative - initial_relative))
    initial_relative_quat = _relative_quaternion(
        initial.flange_quat_wxyz, initial.palm_quat_wxyz
    )
    final_relative_quat = _relative_quaternion(
        final.flange_quat_wxyz, final.palm_quat_wxyz
    )
    relative_quaternion_alignment = float(abs(np.dot(initial_relative_quat, final_relative_quat)))
    arm_path = ROOT / config.assets.arm_mjcf
    hand_path = ROOT / config.assets.hand_mjcf
    arm_asset_dir = ROOT / config.assets.arm_asset_dir
    hand_asset_dir = ROOT / config.assets.hand_asset_dir
    mujoco = importlib.import_module("mujoco")
    joint2_position_m = environment.data.xanchor[environment.arm.joint_ids[1]].tolist()
    report = {
        "schema_version": 1,
        "scene": config.name,
        "mujoco_version": mujoco.__version__,
        "finite": bool(
            np.isfinite(
                np.concatenate(
                    (
                        final.arm_q7,
                        final.arm_dq7,
                        final.hand_q20,
                        final.hand_dq20,
                        final.fingertip_positions_m.ravel(),
                    )
                )
            ).all()
        ),
        "dimensions": {
            "nq": environment.model.nq,
            "nv": environment.model.nv,
            "nu": environment.model.nu,
        },
        "scene_geometry": {
            "table_size_m": list(config.table.size_m),
            "table_top_z_m": config.table.top_z_m,
            "pedestal_center_m": list(config.arm_pedestal.center_m),
            "pedestal_height_m": config.arm_pedestal.height_m,
            "pedestal_top_size_m": list(config.arm_pedestal.top_size_m),
            "pedestal_bottom_size_m": list(config.arm_pedestal.bottom_size_m),
            "pedestal_top_z_m": config.arm_pedestal.top_z_m,
            "arm_mount_position_m": list(config.arm_mount.position_m),
            "joint2_position_m": joint2_position_m,
            "joint2_clearance_above_table_m": (
                joint2_position_m[2] - config.table.top_z_m
            ),
        },
        "assets": {
            "arm_mjcf": str(config.assets.arm_mjcf),
            "arm_mjcf_sha256": sha256_file(arm_path),
            "arm_asset_tree_sha256": sha256_tree(arm_asset_dir),
            "hand_mjcf": str(config.assets.hand_mjcf),
            "hand_mjcf_sha256": sha256_file(hand_path),
            "hand_asset_tree_sha256": sha256_tree(hand_asset_dir),
        },
        "control": {
            "rate_hz": config.control.rate_hz,
            "physics_substeps": config.control.physics_substeps,
            "arm_names": list(environment.arm.names),
            "arm_qpos_addresses": list(environment.arm.qpos_addresses),
            "arm_actuator_ids": list(environment.arm.actuator_ids),
            "hand_names": list(environment.hand.names),
            "hand_qpos_addresses": list(environment.hand.qpos_addresses),
            "hand_actuator_ids": list(environment.hand.actuator_ids),
        },
        "attachment": {
            "assumption": config.hand_attachment.assumption,
            "relative_position_drift_m": relative_position_drift_m,
            "relative_quaternion_alignment": relative_quaternion_alignment,
        },
        "initial": _state_json(initial),
        "final": _state_json(final),
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.report is not None:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded + "\n", encoding="utf-8")
    if args.render_ppm is not None:
        _write_ppm(args.render_ppm, environment.render())
    print(encoded)
    attachment_stable = (
        relative_position_drift_m < 1e-12 and 1.0 - relative_quaternion_alignment < 1e-12
    )
    return 0 if report["finite"] and attachment_stable else 1


if __name__ == "__main__":
    raise SystemExit(main())
