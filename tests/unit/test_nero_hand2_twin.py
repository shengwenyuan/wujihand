from __future__ import annotations

import numpy as np
import pytest

from wujihand.adapters.simulation import (
    NeroHand2AttachmentConfig,
    author_nero_hand2_attachment,
    discover_nero_hand2_dofs,
)
from wujihand.domain import HAND2_LEFT_LAYOUT


def _config(**overrides: object) -> NeroHand2AttachmentConfig:
    values: dict[str, object] = {
        "side": "left",
        "nero_prim_path": "/World/Robots/NeroLeft",
        "hand_prim_path": "/World/Robots/HandLeft",
        "nero_articulation_root_path": "/World/Robots/NeroLeft/Geometry/world",
        "parent_link_path": (
            "/World/Robots/NeroLeft/Geometry/world/base_link/link1/link2/"
            "link3/link4/link5/link6/link7"
        ),
        "child_base_link_path": "/World/Robots/HandLeft/l_base_link",
        "hand_root_joint_path": "/World/Robots/HandLeft/root_joint",
        "attachment_joint_path": "/World/Attachments/left_flange_to_hand",
    }
    values.update(overrides)
    return NeroHand2AttachmentConfig(**values)  # type: ignore[arg-type]


def test_attachment_config_accepts_explicit_side_and_paths() -> None:
    config = _config()

    assert config.side == "left"
    assert config.position_m == (0.0, 0.0, 0.0)
    assert config.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert config.enable_self_collisions is False


@pytest.mark.parametrize(
    ("field", "value", "match"),
    [
        ("side", "none", "side"),
        ("parent_link_path", "/World/Other/link7", "NERO"),
        (
            "parent_link_path",
            "/World/Robots/NeroLeft/Other/link7",
            "articulation",
        ),
        ("child_base_link_path", "/World/Other/l_base_link", "Hand 2"),
        ("quat_wxyz", (2.0, 0.0, 0.0, 0.0), "unit quaternion"),
        ("position_m", (0.0, np.nan, 0.0), "NaN"),
    ],
)
def test_attachment_config_rejects_ambiguous_topology(
    field: str,
    value: object,
    match: str,
) -> None:
    with pytest.raises(ValueError, match=match):
        _config(**{field: value})


def test_attachment_authoring_rejects_unqualified_self_collision_before_pxr() -> None:
    with pytest.raises(RuntimeError, match="not qualified"):
        author_nero_hand2_attachment(
            None,
            _config(enable_self_collisions=True),
        )


def test_discover_q27_partition_uses_names_and_joint_paths() -> None:
    arm_names = tuple(f"joint{index}" for index in range(1, 8))
    hand_names = HAND2_LEFT_LAYOUT.names
    runtime_names = (
        hand_names[4],
        arm_names[0],
        *hand_names[5:12],
        *arm_names[1:],
        *hand_names[12:],
        *hand_names[:4],
    )
    runtime_paths = tuple(
        (f"/World/Physics/Nero/{name}" if name in arm_names else f"/World/Physics/Hand2/{name}")
        for name in runtime_names
    )

    partition = discover_nero_hand2_dofs(
        runtime_names,
        runtime_paths,
        arm_names,
        hand_names,
        nero_prim_path="/World/Physics/Nero",
        hand_prim_path="/World/Physics/Hand2",
    )

    assert tuple(runtime_names[index] for index in partition.arm_indices_q7) == arm_names
    assert tuple(runtime_names[index] for index in partition.hand_indices_q20) == hand_names
    assert sorted(partition.all_indices) == list(range(27))


def test_discover_q27_partition_rejects_wrong_joint_path() -> None:
    arm_names = tuple(f"joint{index}" for index in range(1, 8))
    hand_names = HAND2_LEFT_LAYOUT.names
    names = (*arm_names, *hand_names)
    paths = [
        *[f"/World/Physics/Nero/{name}" for name in arm_names],
        *[f"/World/Physics/Hand2/{name}" for name in hand_names],
    ]
    paths[-1] = "/World/Physics/Hand2/not_the_pinky_dip"

    with pytest.raises(RuntimeError, match="layout mismatch"):
        discover_nero_hand2_dofs(
            names,
            paths,
            arm_names,
            hand_names,
            nero_prim_path="/World/Physics/Nero",
            hand_prim_path="/World/Physics/Hand2",
        )


def test_discover_q27_partition_rejects_cross_asset_joint_path() -> None:
    arm_names = tuple(f"joint{index}" for index in range(1, 8))
    hand_names = HAND2_LEFT_LAYOUT.names
    names = (*arm_names, *hand_names)
    paths = [
        *[f"/World/Physics/Nero/{name}" for name in arm_names],
        *[f"/World/Physics/Hand2/{name}" for name in hand_names],
    ]
    paths[0] = f"/World/Physics/Hand2/{arm_names[0]}"

    with pytest.raises(RuntimeError, match="layout mismatch"):
        discover_nero_hand2_dofs(
            names,
            paths,
            arm_names,
            hand_names,
            nero_prim_path="/World/Physics/Nero",
            hand_prim_path="/World/Physics/Hand2",
        )
