from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from wujihand.adapters.simulation.nero_model import load_nero_model_profile
from wujihand.adapters.simulation.nero_startup import (
    load_nero_dual_simulation_startup_profile,
)
from wujihand.domain.pose import quaternion_wxyz_to_rotation_matrix
from wujihand.runtime import ConfigRepository, SessionResolver, SourceLock
from wujihand.runtime.isaac_workcell_plan import resolve_isaac_workcell_plan


ROOT = Path(__file__).resolve().parents[2]
SESSION = ROOT / "configs/sessions/isaac_nero_dual_hand2_tframe_inspection_v2026_8_3_v1.yaml"
STARTUP = ROOT / "configs/profiles/isaac_nero_dual_tframe_inspection_startup_v1.yaml"
NERO_PROFILE = ROOT / "configs/profiles/agilex_nero_q7_provisional_v1.yaml"
TABLETOP_STARTUP = ROOT / "configs/profiles/isaac_nero_dual_tabletop_qualification_v1.yaml"


def test_tframe_inspection_applies_only_the_confirmed_left_flange_clocking() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION, verify_artifacts=False)
    nominal = ConfigRepository(ROOT).load_assembly(
        "configs/assemblies/"
        "nero_dual_hand2_d405_wrist_rig_simulation_nominal_v2026_8_3_v1.yaml"
    )

    assert resolved.session.runtime_role == "simulation"
    assert (
        resolved.assembly.assembly_id
        == "nero_dual_hand2_d405_wrist_rig_tframe_v2026_8_3_v1"
    )
    assert resolved.workcell.workcell_id == "isaac_dual_nero_tframe_candidate_20260811_v1"
    assert dict(resolved.session.placements) == {
        "nero_left": "tframe_negative_x_shoulder_mount",
        "nero_right": "tframe_positive_x_shoulder_mount",
    }
    assert {
        (attachment.parent.instance, attachment.child.instance)
        for attachment in resolved.assembly.attachments
    } == {
        ("nero_left", "hand_left"),
        ("hand_left", "mount_left"),
        ("mount_left", "d405_left"),
        ("nero_right", "hand_right"),
        ("hand_right", "mount_right"),
        ("mount_right", "d405_right"),
    }
    tframe_attachments = {
        attachment.attachment_id: attachment
        for attachment in resolved.assembly.attachments
    }
    nominal_attachments = {
        attachment.attachment_id: attachment for attachment in nominal.attachments
    }
    left_attachment_id = "nero_left_link7_to_hand2_left_wrist"
    assert resolved.assembly.instances == nominal.instances
    assert resolved.assembly.roots == nominal.roots
    assert {
        attachment_id: attachment
        for attachment_id, attachment in tframe_attachments.items()
        if attachment_id != left_attachment_id
    } == {
        attachment_id: attachment
        for attachment_id, attachment in nominal_attachments.items()
        if attachment_id != left_attachment_id
    }
    left_tframe = tframe_attachments[left_attachment_id]
    left_nominal = nominal_attachments[left_attachment_id]
    assert left_tframe.transform.position_m == left_nominal.transform.position_m
    relative_rotation = (
        quaternion_wxyz_to_rotation_matrix(left_nominal.transform.quat_wxyz).T
        @ quaternion_wxyz_to_rotation_matrix(left_tframe.transform.quat_wxyz)
    )
    np.testing.assert_allclose(relative_rotation, np.diag([-1.0, -1.0, 1.0]), atol=1e-12)
    for side, outward_x in (("left", -1.0), ("right", 1.0)):
        mount_id = resolved.session.mount_for(f"nero_{side}")
        mount = resolved.workcell.mount(mount_id)
        rotation = quaternion_wxyz_to_rotation_matrix(mount.transform.quat_wxyz)
        np.testing.assert_allclose(
            rotation @ np.asarray([0.0, 0.0, 1.0]),
            [outward_x, 0.0, 0.0],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            rotation @ np.asarray([-1.0, 0.0, 0.0]),
            [0.0, 1.0, 0.0],
            atol=1e-12,
        )

    plan = resolve_isaac_workcell_plan(ROOT, resolved.workcell)
    assert plan.profile_id == "isaac_dual_nero_tframe_candidate_20260811_v1"
    assert plan.imports[0].content.expected_sha256 == (
        "0bc8d7446b026ccdb819a3ce124e516b208e6c2f8416232ba87572e503eea6af"
    )
    assert plan.expectations is not None
    assert plan.expectations.min_colliders == 14


def test_tframe_startup_is_the_mirrored_hanging_l_inspection_pose() -> None:
    profile = load_nero_dual_simulation_startup_profile(STARTUP)
    nero = load_nero_model_profile(NERO_PROFILE)
    expected = {
        "left": (-math.pi / 2.0, -math.pi / 2.0, math.pi / 2.0, math.pi / 2.0, 0.0, 0.0, 0.0),
        "right": (-math.pi / 2.0, math.pi / 2.0, math.pi / 2.0, math.pi / 2.0, 0.0, 0.0, 0.0),
    }

    for side in ("left", "right"):
        position = profile.initial_position(f"nero_{side}", "arm_joints", "agilex_nero_q7_v1")
        assert tuple(position) == expected[side]
        np.testing.assert_array_equal(nero.layout.validate_vector(position), position)
    assert profile.status == "provisional_inspection_only"
    assert profile.teleport_to_initial_position is True


def test_tabletop_startup_compatibility_preserves_drive_from_reset() -> None:
    profile = load_nero_dual_simulation_startup_profile(TABLETOP_STARTUP)

    assert profile.teleport_to_initial_position is False
    assert profile.initial_q7_max_error_rad == 0.08


def test_tframe_usd_wrapper_is_locked_with_both_dependencies() -> None:
    source = SourceLock.load(ConfigRepository(ROOT)).record("dual-nero-tframe-isaac-6-0-1-v1")

    assert dict(source.artifacts) == {
        "tframe.usda": "0bc8d7446b026ccdb819a3ce124e516b208e6c2f8416232ba87572e503eea6af",
        "tframe_collision.usdc": "b4c45479da14bd6b3d0ea124be5e9243bd22f4c0dd64006fb046b01db6a155e8",
        "tframe_visual.usdc": "ed6eade87adc2d42a4b45d2c75a64df1c67b5c07276add84966f71b6c844cda8",
    }
