from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.dataset.profile import load_mini_dataset_profile, load_q54_joint_profile


ROOT = Path(__file__).parents[2]
PROFILE = "configs/profiles/isaac_nero_hand2_q54_dataset_v1.yaml"
DATASET_PROFILE = "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"


def test_q54_profile_freezes_every_dimension_and_source_hash() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)

    assert profile.dimension == 54
    assert len(profile.canonical_names) == 54
    assert profile.canonical_names[:2] == (
        "left.arm.joint1",
        "left.arm.joint2",
    )
    assert profile.canonical_names[7] == "left.hand.l_thumb_cmc_flex"
    assert profile.canonical_names[27] == "right.arm.joint1"
    assert profile.canonical_names[-1] == "right.hand.r_pinky_dip"
    assert all(
        item.real_hardware_mapping_status == "future_unverified_requires_device_readback"
        for item in profile.joints
    )


def test_q54_assembly_uses_explicit_q27_source_indices() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    left = tuple(float(index) for index in range(27))
    right = tuple(float(100 + index) for index in range(27))

    q54 = profile.assemble_from_q27(left_q27_rad=left, right_q27_rad=right)
    left_source = tuple(item.source_index_q27 for item in profile.joints[:27])
    right_source = tuple(item.source_index_q27 for item in profile.joints[27:])

    assert q54[:7] == left[:7]
    assert q54[7:27] == tuple(left[index] for index in left_source[7:])
    assert q54[27:34] == right[:7]
    assert q54[34:] == tuple(right[index] for index in right_source[7:])


def test_runtime_reorder_is_name_based_not_articulation_column_based() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    left_joints = tuple(item for item in profile.joints if item.side == "left")
    right_joints = tuple(item for item in profile.joints if item.side == "right")
    left_names = tuple(item.source_joint_name for item in reversed(left_joints))
    right_names = tuple(item.source_joint_name for item in reversed(right_joints))
    left_positions = tuple(float(item.source_index_q27) for item in reversed(left_joints))
    right_positions = tuple(float(100 + item.source_index_q27) for item in reversed(right_joints))

    q54 = profile.reorder_runtime_positions(
        left_names=left_names,
        left_positions_rad=left_positions,
        right_names=right_names,
        right_positions_rad=right_positions,
    )

    assert q54 == tuple(float(item.source_index_q27) for item in left_joints) + tuple(
        float(100 + item.source_index_q27) for item in right_joints
    )


def test_runtime_reorder_fails_closed_on_missing_or_duplicate_name() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    names = tuple(item.source_joint_name for item in profile.joints[:27])

    with pytest.raises(ValueError, match="unique"):
        profile.reorder_runtime_positions(
            left_names=(*names[:-1], names[0]),
            left_positions_rad=(0.0,) * 27,
            right_names=tuple(item.source_joint_name for item in profile.joints[27:]),
            right_positions_rad=(0.0,) * 27,
        )


def test_q54_velocity_assembly_does_not_apply_position_offsets() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    left = tuple(float(index) for index in range(27))
    right = tuple(float(100 + index) for index in range(27))

    assert profile.assemble_velocity_from_q27(
        left_qdot27_rad_s=left,
        right_qdot27_rad_s=right,
    ) == tuple(left[item.source_index_q27] for item in profile.joints[:27]) + tuple(
        right[item.source_index_q27] for item in profile.joints[27:]
    )


def test_q54_runtime_inventory_closes_names_indices_and_limits() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    left = tuple(item for item in profile.joints if item.side == "left")
    right = tuple(item for item in profile.joints if item.side == "right")
    left_names = tuple(
        next(item.source_joint_name for item in left if item.source_index_q27 == index)
        for index in range(27)
    )
    right_names = tuple(
        next(item.source_joint_name for item in right if item.source_index_q27 == index)
        for index in range(27)
    )
    left_limits = tuple(
        next((item.lower_rad, item.upper_rad) for item in left if item.source_index_q27 == index)
        for index in range(27)
    )
    right_limits = tuple(
        next((item.lower_rad, item.upper_rad) for item in right if item.source_index_q27 == index)
        for index in range(27)
    )

    inventory = profile.validate_runtime_inventory(
        left_names=left_names,
        left_limits_rad=left_limits,
        right_names=right_names,
        right_limits_rad=right_limits,
    )

    assert inventory.canonical_source_indices == tuple(
        item.source_index_q27 for item in profile.joints
    )
    assert inventory.to_mapping()["canonical_names"] == list(profile.canonical_names)


def test_q54_runtime_inventory_rejects_limit_drift() -> None:
    profile = load_q54_joint_profile(ROOT, PROFILE)
    left = tuple(item for item in profile.joints if item.side == "left")
    right = tuple(item for item in profile.joints if item.side == "right")
    left_names = [""] * 27
    right_names = [""] * 27
    left_limits = [(0.0, 0.0)] * 27
    right_limits = [(0.0, 0.0)] * 27
    for item in left:
        left_names[item.source_index_q27] = item.source_joint_name
        left_limits[item.source_index_q27] = (item.lower_rad, item.upper_rad)
    for item in right:
        right_names[item.source_index_q27] = item.source_joint_name
        right_limits[item.source_index_q27] = (item.lower_rad, item.upper_rad)
    left_limits[4] = (left_limits[4][0], left_limits[4][1] + 0.01)

    with pytest.raises(ValueError, match="joint5 runtime limits differ"):
        profile.validate_runtime_inventory(
            left_names=left_names,
            left_limits_rad=left_limits,
            right_names=right_names,
            right_limits_rad=right_limits,
        )


def test_mini_dataset_profile_freezes_rgb_only_triview_and_git_lerobot_pin() -> None:
    profile = load_mini_dataset_profile(ROOT, DATASET_PROFILE)

    assert (profile.physics_hz, profile.control_hz, profile.gui_preview_hz) == (120, 30, 15)
    assert profile.profile_id == "isaac_nero_hand2_triview_q54_mini_dataset_120_30_15_v1"
    assert profile.policy_fps == 30
    assert tuple(camera.logical_id for camera in profile.cameras) == (
        "scene_rgb",
        "left_wrist_rgb",
        "right_wrist_rgb",
    )
    assert all(camera.payload_whitelist == ("rgb",) for camera in profile.cameras)
    assert all(not camera.physical_calibration_compatible for camera in profile.cameras)
    assert profile.lerobot_commit == "7e241bd630a3719a56157a497ce5d08f244784f1"
    assert profile.lerobot_python == ">=3.12,<3.14"
    assert profile.retained_episode_hard_limit == 18
