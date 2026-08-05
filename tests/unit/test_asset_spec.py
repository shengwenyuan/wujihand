from __future__ import annotations

from copy import deepcopy
from typing import Any

import pytest

from wujihand.specs import AssetManifest


def _asset() -> dict[str, Any]:
    return {
        "schema": "wujihand.asset_manifest.v1",
        "asset_id": "wuji_hand2_beta1_right",
        "revision": "beta1",
        "kind": "robot_hand",
        "product": "wuji_hand_2",
        "side": "right",
        "canonical_profile": "configs/profiles/hand2_right.yaml",
        "frames": {"base": "hand_base", "pose_command": "neutral_pose"},
        "control_groups": [
            {
                "group_id": "finger_joints",
                "semantic": "finger_joint_position",
                "layout_id": "wuji_hand2_right_firmware_v1",
                "dof_count": 20,
                "command_interface": "position",
                "joint_profile": "configs/profiles/hand2_right.yaml",
            }
        ],
        "provenance_source": "wuji-description",
    }


def _passive_asset(*, kind: str = "passive_component") -> dict[str, Any]:
    return {
        "schema": "wujihand.asset_manifest.v2",
        "asset_id": "nero_hand2_beta1_d405_mount_v2_right",
        "revision": "v2",
        "kind": kind,
        "product": "nero_hand2_beta1_d405_wrist_mount",
        "side": "right",
        "canonical_profile": None,
        "frames": {
            "hand_interface": "hand_interface",
            "camera_interface": "camera_interface",
        },
        "control_groups": [],
        "provenance_source": "hardware/camera_mounts/d405/README.md",
    }


def test_asset_manifest_round_trip_and_lookup() -> None:
    manifest = AssetManifest.from_mapping(_asset())

    assert AssetManifest.from_mapping(manifest.to_mapping()) == manifest
    assert manifest.frame_name("base") == "hand_base"
    assert manifest.control_group("finger_joints").layout_id == (
        "wuji_hand2_right_firmware_v1"
    )


def test_asset_manifest_rejects_extra_keys_and_invalid_side() -> None:
    extra = _asset()
    extra["usd"] = "hand.usd"
    with pytest.raises(ValueError, match="unexpected"):
        AssetManifest.from_mapping(extra)

    invalid_side = _asset()
    invalid_side["side"] = "starboard"
    with pytest.raises(ValueError, match="side must be one of"):
        AssetManifest.from_mapping(invalid_side)


def test_asset_manifest_rejects_duplicate_frames_and_control_groups() -> None:
    duplicate_frame = _asset()
    duplicate_frame["frames"] = {"base": "hand_base", "flange": "hand_base"}
    with pytest.raises(ValueError, match="frame names must be unique"):
        AssetManifest.from_mapping(duplicate_frame)

    duplicate_group = _asset()
    duplicate_group["control_groups"] = [
        *_asset()["control_groups"],
        deepcopy(_asset()["control_groups"][0]),
    ]
    with pytest.raises(ValueError, match="group_id values must be unique"):
        AssetManifest.from_mapping(duplicate_group)


def test_asset_manifest_requires_explicit_valid_layout_and_safe_profile() -> None:
    invalid_layout = _asset()
    invalid_layout["control_groups"][0]["layout_id"] = ""
    with pytest.raises(ValueError, match="layout_id"):
        AssetManifest.from_mapping(invalid_layout)

    invalid_profile = _asset()
    invalid_profile["canonical_profile"] = "../hand.yaml"
    with pytest.raises(ValueError, match="safe project-relative"):
        AssetManifest.from_mapping(invalid_profile)

    invalid_dof_count = _asset()
    invalid_dof_count["control_groups"][0]["dof_count"] = 0
    with pytest.raises(ValueError, match="positive integer"):
        AssetManifest.from_mapping(invalid_dof_count)


@pytest.mark.parametrize("kind", ("passive_component", "simulated_sensor"))
def test_asset_manifest_v2_allows_only_passive_kinds_without_control_groups(
    kind: str,
) -> None:
    manifest = AssetManifest.from_mapping(_passive_asset(kind=kind))

    assert manifest.kind == kind
    assert manifest.control_groups == ()
    assert AssetManifest.from_mapping(manifest.to_mapping()) == manifest


def test_asset_manifest_versions_preserve_control_group_boundary() -> None:
    v1_empty = _asset()
    v1_empty["control_groups"] = []
    with pytest.raises(ValueError, match="must not be empty"):
        AssetManifest.from_mapping(v1_empty)

    v1_passive = _asset()
    v1_passive["kind"] = "passive_component"
    with pytest.raises(ValueError, match="kind must be one of"):
        AssetManifest.from_mapping(v1_passive)

    controlled_v2 = _passive_asset()
    controlled_v2["kind"] = "robot_hand"
    with pytest.raises(ValueError, match="must not be empty for controlled assets"):
        AssetManifest.from_mapping(controlled_v2)

    passive_with_group = _passive_asset()
    passive_with_group["control_groups"] = deepcopy(_asset()["control_groups"])
    with pytest.raises(ValueError, match="must be empty for passive assets"):
        AssetManifest.from_mapping(passive_with_group)
