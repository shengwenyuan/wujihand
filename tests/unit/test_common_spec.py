from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from wujihand.specs import ConfigRef, PoseSpec


def test_config_ref_and_pose_round_trip_as_immutable_values() -> None:
    reference = ConfigRef.from_mapping(
        {"path": "configs/assets/hand.yaml", "expected_id": "hand_right"}
    )
    pose = PoseSpec.from_mapping(
        {
            "position_m": [0.1, -0.2, 0.3],
            "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
        }
    )

    assert ConfigRef.from_mapping(reference.to_mapping()) == reference
    assert PoseSpec.from_mapping(pose.to_mapping()) == pose
    with pytest.raises(FrozenInstanceError):
        pose.position_m = (0.0, 0.0, 0.0)  # type: ignore[misc]


@pytest.mark.parametrize(
    "path",
    [
        "/etc/passwd",
        "../outside.yaml",
        "configs/../outside.yaml",
        "configs/./ambiguous.yaml",
        "configs//ambiguous.yaml",
        "~/private.yaml",
        r"configs\windows.yaml",
        "C:/windows.yaml",
    ],
)
def test_config_ref_rejects_unsafe_paths(path: str) -> None:
    with pytest.raises(ValueError, match="safe project-relative"):
        ConfigRef.from_mapping({"path": path, "expected_id": "asset"})


def test_pose_rejects_unknown_key_nonfinite_and_nonunit_quaternion() -> None:
    with pytest.raises(ValueError, match="unexpected"):
        PoseSpec.from_mapping(
            {
                "position_m": [0.0, 0.0, 0.0],
                "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
                "degrees": False,
            }
        )
    with pytest.raises(ValueError, match="finite number"):
        PoseSpec.from_mapping(
            {
                "position_m": [0.0, float("nan"), 0.0],
                "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        )
    with pytest.raises(ValueError, match="finite number"):
        PoseSpec.from_mapping(
            {
                "position_m": [10**1000, 0.0, 0.0],
                "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        )
    with pytest.raises(ValueError, match="unit length"):
        PoseSpec.from_mapping(
            {
                "position_m": [0.0, 0.0, 0.0],
                "quat_wxyz": [2.0, 0.0, 0.0, 0.0],
            }
        )
