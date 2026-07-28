#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Manual Isaac 6.0.1 smoke for two independent instances of the pinned NERO USD.

This is a pre-composition NV-2 asset gate, not the production twin runner.  It
does not load a Session, Hand 2, ROS, CAN, or any hardware SDK.  It proves that
the same derived Binding artifact can be instantiated twice, exposes exactly
two independent q7 articulations, and follows the provisional simulation limits.
"""

from __future__ import annotations

import argparse
from importlib.metadata import version
import json
from pathlib import Path
import sys
from typing import cast


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--asset",
        type=Path,
        default=(
            ROOT
            / "artifacts/derived/isaac/6.0.1/agilex_nero/"
            "nero_description/nero_description.usda"
        ),
    )
    parser.add_argument(
        "--profile",
        type=Path,
        default=ROOT / "configs/profiles/agilex_nero_q7_provisional_v1.yaml",
    )
    parser.add_argument("--frames-per-phase", type=int, default=240)
    parser.add_argument("--amplitude-rad", type=float, default=0.08)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames_per_phase < 1:
    raise SystemExit("--frames-per-phase must be positive")
if not 0.01 <= ARGS.amplitude_rad <= 0.15:
    raise SystemExit("--amplitude-rad must be between 0.01 and 0.15")
if not ARGS.asset.is_file():
    raise SystemExit(f"NERO USD not found: {ARGS.asset}")
if not ARGS.profile.is_file():
    raise SystemExit(f"NERO profile not found: {ARGS.profile}")

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp({"headless": True, "anti_aliasing": 0})

import numpy as np
import numpy.typing as npt
from pxr import Gf, UsdGeom  # type: ignore[import-not-found]

from isaacsim.core.api import World  # type: ignore[import-not-found]
from isaacsim.core.prims import Articulation  # type: ignore[import-not-found]
from isaacsim.core.utils.stage import (  # type: ignore[import-not-found]
    add_reference_to_stage,
)
from wujihand.adapters.simulation.nero_model import (
    NERO_JOINT_NAMES,
    load_nero_model_profile,
)
from wujihand.integrity import sha256_file


PHYSICS_HZ = 120
LEFT_REFERENCE_PATH = "/World/LeftMount/Nero"
RIGHT_REFERENCE_PATH = "/World/RightMount/Nero"
LEFT_ARTICULATION_PATH = f"{LEFT_REFERENCE_PATH}/Geometry/world"
RIGHT_ARTICULATION_PATH = f"{RIGHT_REFERENCE_PATH}/Geometry/world"


def _positions(articulation: Articulation) -> npt.NDArray[np.float64]:
    values = np.asarray(articulation.get_joint_positions(), dtype=np.float64)
    if values.shape != (1, 7) or not np.isfinite(values).all():
        raise RuntimeError(f"invalid NERO q7 feedback shape/value: {values.shape}")
    return cast(npt.NDArray[np.float64], values[0].copy())


def _step(world: World, frames: int) -> None:
    for _ in range(frames):
        world.step(render=False)


def main() -> int:
    profile = load_nero_model_profile(ARGS.profile)
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
        device="cpu",
    )
    stage = world.scene.stage
    left_mount = UsdGeom.Xform.Define(stage, "/World/LeftMount")
    right_mount = UsdGeom.Xform.Define(stage, "/World/RightMount")
    left_mount.AddTranslateOp().Set(Gf.Vec3d(-0.75, 0.0, 0.0))
    right_mount.AddTranslateOp().Set(Gf.Vec3d(0.75, 0.0, 0.0))
    add_reference_to_stage(str(ARGS.asset.resolve()), LEFT_REFERENCE_PATH)
    add_reference_to_stage(str(ARGS.asset.resolve()), RIGHT_REFERENCE_PATH)

    left = world.scene.add(Articulation(LEFT_ARTICULATION_PATH, name="nero_left"))
    right = world.scene.add(Articulation(RIGHT_ARTICULATION_PATH, name="nero_right"))
    world.reset()

    if tuple(left.dof_names) != NERO_JOINT_NAMES:
        raise RuntimeError(f"left NERO q7 layout drifted: {left.dof_names}")
    if tuple(right.dof_names) != NERO_JOINT_NAMES:
        raise RuntimeError(f"right NERO q7 layout drifted: {right.dof_names}")
    expected_limits = np.column_stack(
        (profile.layout.lower, profile.layout.upper)
    )
    for side, articulation in (("left", left), ("right", right)):
        limits = np.asarray(articulation.get_dof_limits(), dtype=np.float64)
        if limits.shape != (1, 7, 2) or not np.allclose(
            limits[0],
            expected_limits,
            atol=1e-4,
        ):
            raise RuntimeError(f"{side} NERO limits differ from q7 profile")

    zero = np.zeros((1, 7), dtype=np.float64)
    left.set_joint_position_targets(zero)
    right.set_joint_position_targets(zero)
    _step(world, 30)
    initial_left = _positions(left)
    initial_right = _positions(right)

    left_target = zero.copy()
    left_target[0, 0] = ARGS.amplitude_rad
    left.set_joint_position_targets(left_target)
    right.set_joint_position_targets(zero)
    _step(world, ARGS.frames_per_phase)
    left_after_phase1 = _positions(left)
    right_after_phase1 = _positions(right)

    right_target = zero.copy()
    right_target[0, 1] = -ARGS.amplitude_rad
    left.set_joint_position_targets(left_target)
    right.set_joint_position_targets(right_target)
    _step(world, ARGS.frames_per_phase)
    left_after_phase2 = _positions(left)
    right_after_phase2 = _positions(right)

    all_feedback = np.vstack(
        (
            initial_left,
            initial_right,
            left_after_phase1,
            right_after_phase1,
            left_after_phase2,
            right_after_phase2,
        )
    )
    within_limits = bool(
        np.all(all_feedback >= np.asarray(profile.layout.lower) - 1e-4)
        and np.all(all_feedback <= np.asarray(profile.layout.upper) + 1e-4)
    )
    left_responded = bool(
        left_after_phase1[0] >= ARGS.amplitude_rad * 0.25
    )
    right_isolated_phase1 = bool(np.max(np.abs(right_after_phase1)) <= 0.01)
    right_responded = bool(
        right_after_phase2[1] <= -ARGS.amplitude_rad * 0.25
    )
    left_held_phase2 = bool(
        abs(left_after_phase2[0] - left_after_phase1[0]) <= 0.02
    )
    passed = (
        within_limits
        and left_responded
        and right_isolated_phase1
        and right_responded
        and left_held_phase2
    )
    report = {
        "schema": "wujihand.isaac_nero_dual_asset_smoke.v1",
        "scope": "NV-2 pre-composition simulation asset gate; no hardware",
        "isaac_distribution": version("isaacsim"),
        "asset_path": ARGS.asset.resolve().as_posix(),
        "asset_sha256": sha256_file(ARGS.asset),
        "profile_path": ARGS.profile.resolve().as_posix(),
        "profile_sha256": sha256_file(ARGS.profile),
        "articulation_paths": [
            LEFT_ARTICULATION_PATH,
            RIGHT_ARTICULATION_PATH,
        ],
        "dof_names": list(NERO_JOINT_NAMES),
        "frames_per_phase": ARGS.frames_per_phase,
        "amplitude_rad": ARGS.amplitude_rad,
        "feedback": {
            "initial_left": initial_left.tolist(),
            "initial_right": initial_right.tolist(),
            "left_after_phase1": left_after_phase1.tolist(),
            "right_after_phase1": right_after_phase1.tolist(),
            "left_after_phase2": left_after_phase2.tolist(),
            "right_after_phase2": right_after_phase2.tolist(),
        },
        "checks": {
            "feedback_finite_and_within_limits": within_limits,
            "left_joint1_responded": left_responded,
            "right_remained_isolated_in_phase1": right_isolated_phase1,
            "right_joint2_responded": right_responded,
            "left_held_while_right_moved": left_held_phase2,
        },
        "passed": passed,
    }
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if ARGS.report is not None:
        ARGS.report.parent.mkdir(parents=True, exist_ok=True)
        ARGS.report.write_text(encoded, encoding="utf-8")
    print(encoded, end="", flush=True)
    return 0 if passed else 2


try:
    raise SystemExit(main())
finally:
    simulation_app.close()
