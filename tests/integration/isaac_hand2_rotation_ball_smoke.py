#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Manual Isaac Sim 5.1 smoke for the Hand 2 rotation mount and dynamic ball.

This is intentionally not collected by the default pytest suite.  It requires a
working Isaac GPU/Vulkan environment and validates only scene construction,
23-DOF discovery, fixed-flange drift, and dynamic-ball/contact-view health.  It
does not claim a successful grasp; scripted/live lift acceptance belongs to the
end-to-end validation gate.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument(
        "--author-only",
        action="store_true",
        help="Validate the USD overlay without creating World/PhysX runtime handles.",
    )
    parser.add_argument(
        "--asset",
        type=Path,
        default=(
            ROOT
            / "third_party/src/wuji-description/v2026.6.27/hand2_beta/body/usd/right/wujihand.usd"
        ),
    )
    parser.add_argument("--target-rpy-deg", nargs=3, type=float, default=(10.0, -10.0, 30.0))
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames < 1:
    raise SystemExit("--frames must be positive")
if not ARGS.asset.is_file():
    raise SystemExit(f"Hand 2 USD not found: {ARGS.asset}")

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True, "anti_aliasing": 0})

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from wujihand.adapters.simulation import (
    Hand2BallConfig,
    Hand2RotationMountConfig,
    add_hand2_ball_scene,
    author_rotation_mount,
    contact_groups_from_force_matrix,
    discover_rotation_mount_dofs,
    set_rotation_mount_targets_rpy,
)
from wujihand.domain import HAND2_RIGHT_LAYOUT


def _runtime_dof_paths(hand: Articulation) -> tuple[str, ...]:
    """Read the Isaac 5.1 articulation-view paths and reject unknown shapes."""

    raw = getattr(hand, "_dof_paths", None)
    if raw is None:
        raise RuntimeError("Isaac Articulation did not expose initialized DOF paths")
    paths = np.asarray(raw, dtype=object)
    if paths.ndim != 2 or paths.shape[0] != 1:
        raise RuntimeError(f"expected one articulation DOF-path row, got shape {paths.shape}")
    return tuple(str(path) for path in paths[0])


def main() -> int:
    if ARGS.author_only:
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        hand_reference = UsdGeom.Xform.Define(stage, "/World/Hand2")
        hand_reference.GetPrim().GetReferences().AddReference(str(ARGS.asset.resolve()))
        mount = author_rotation_mount(stage, Hand2RotationMountConfig())
        targets_deg = set_rotation_mount_targets_rpy(
            stage,
            mount,
            tuple(math.radians(value) for value in ARGS.target_rpy_deg),
        )
        root_paths = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        ]
        old_root_enabled = UsdPhysics.Joint(
            stage.GetPrimAtPath(mount.paths.disabled_root_joint_path)
        ).GetJointEnabledAttr().Get()
        rotation_schemas = list(
            stage.GetPrimAtPath(mount.paths.rotation_joint_path).GetAppliedSchemas()
        )
        report = {
            "mode": "author_only",
            "asset": str(ARGS.asset.resolve()),
            "articulation_roots": root_paths,
            "disabled_upstream_root_enabled": old_root_enabled,
            "rotation_joint_schemas": rotation_schemas,
            "drive_target_rot_xyz_deg": targets_deg,
        }
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
        expected_schemas = {
            "PhysicsLimitAPI:transX",
            "PhysicsLimitAPI:transY",
            "PhysicsLimitAPI:transZ",
            "PhysicsLimitAPI:rotX",
            "PhysicsLimitAPI:rotY",
            "PhysicsDriveAPI:rotX",
            "PhysicsDriveAPI:rotY",
            "PhysicsDriveAPI:rotZ",
        }
        passed = (
            root_paths == [mount.paths.articulation_root_path]
            and old_root_enabled is False
            and set(rotation_schemas) == expected_schemas
        )
        return 0 if passed else 2

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
        device="cpu",
    )
    world.scene.add_default_ground_plane()
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Table",
            name="table",
            position=np.asarray([0.02, 0.0, 0.35]),
            scale=np.asarray([0.80, 0.60, 0.06]),
            size=1.0,
        )
    )
    add_reference_to_stage(str(ARGS.asset.resolve()), "/World/Hand2")
    mount_config = Hand2RotationMountConfig()
    mount = author_rotation_mount(world.scene.stage, mount_config)
    ball_config = Hand2BallConfig()
    ball_scene = add_hand2_ball_scene(world, ball_config)
    hand = world.scene.add(
        Articulation(mount.paths.articulation_root_path, name="hand2_rotation_mount")
    )
    world.reset()
    ball_scene.contact_view.initialize(world.physics_sim_view)
    ball_scene.hand_table_contact_view.initialize(world.physics_sim_view)

    dof_names = tuple(hand.dof_names)
    dof_paths = _runtime_dof_paths(hand)
    partition = discover_rotation_mount_dofs(
        dof_names,
        dof_paths,
        HAND2_RIGHT_LAYOUT.names,
        mount.paths.rotation_joint_path,
    )
    targets = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
    targets[np.asarray(partition.wrist_indices_xyz)] = np.radians(ARGS.target_rpy_deg)
    hand.set_joint_position_targets(targets.reshape(1, -1))

    initial_root_position = np.asarray(hand.get_world_poses()[0], dtype=np.float64)[0]
    max_root_drift_m = 0.0
    for _ in range(ARGS.frames):
        world.step(render=False)
        root_position = np.asarray(hand.get_world_poses()[0], dtype=np.float64)[0]
        max_root_drift_m = max(
            max_root_drift_m, float(np.linalg.norm(root_position - initial_root_position))
        )

    root_paths = [
        str(prim.GetPath())
        for prim in world.scene.stage.Traverse()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        and (
            str(prim.GetPath()).startswith("/World/Hand2/")
            or str(prim.GetPath()).startswith("/World/Hand2Mount/")
        )
    ]
    ball_position, _ = ball_scene.ball.get_world_pose()
    ball_velocity = ball_scene.ball.get_linear_velocity()
    forces = ball_scene.contact_view.get_contact_force_matrix(dt=1.0 / 120.0)
    if forces is None:
        raise RuntimeError("ball contact view did not return a PhysX force matrix")
    contacts = contact_groups_from_force_matrix(forces, ball_scene.filters)
    hand_table_forces = np.asarray(
        ball_scene.hand_table_contact_view.get_contact_force_matrix(dt=1.0 / 120.0),
        dtype=np.float64,
    )
    if (
        hand_table_forces.size == 0
        or hand_table_forces.ndim not in (2, 3)
        or hand_table_forces.shape[-1] != 3
        or (hand_table_forces.ndim == 3 and hand_table_forces.shape[-2] != 1)
        or not np.isfinite(hand_table_forces).all()
    ):
        raise RuntimeError(
            f"hand/table contact view returned invalid shape {hand_table_forces.shape}"
        )
    final_joint_positions = np.asarray(hand.get_joint_positions(), dtype=np.float64)[0]
    wrist_feedback = final_joint_positions[np.asarray(partition.wrist_indices_xyz)]
    wrist_targets = np.radians(np.asarray(ARGS.target_rpy_deg, dtype=np.float64))
    wrist_axis_response = bool(
        np.all(np.abs(wrist_feedback) >= math.radians(2.0))
        and np.all(wrist_feedback * wrist_targets > 0.0)
    )
    report = {
        "isaac_sim": "5.1.0",
        "asset": str(ARGS.asset.resolve()),
        "frames": ARGS.frames,
        "articulation_roots": root_paths,
        "dof_count": len(dof_names),
        "dof_names": dof_names,
        "dof_paths": dof_paths,
        "wrist_indices_xyz": partition.wrist_indices_xyz,
        "finger_indices_q20": partition.finger_indices_q20,
        "max_fixed_flange_drift_m": max_root_drift_m,
        "ball_position_m": np.asarray(ball_position).tolist(),
        "ball_velocity_m_s": np.asarray(ball_velocity).tolist(),
        "contact_groups_last_frame": sorted(contacts),
        "hand_table_contact_matrix_shape": list(hand_table_forces.shape),
        "wrist_target_rad": wrist_targets.tolist(),
        "wrist_feedback_rad": wrist_feedback.tolist(),
        "all_three_wrist_axes_responded": wrist_axis_response,
        "grasp_lift_qualified": False,
        "scope": "structural/runtime smoke only; no grasp claim",
    }
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    passed = (
        root_paths == [mount.paths.articulation_root_path]
        and len(dof_names) == 23
        and max_root_drift_m <= 0.001
        and np.isfinite(ball_position).all()
        and np.isfinite(ball_velocity).all()
        and wrist_axis_response
    )
    return 0 if passed else 2


try:
    raise SystemExit(main())
finally:
    simulation_app.close()
