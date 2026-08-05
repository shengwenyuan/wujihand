from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from wujihand.runtime import SessionResolver
from wujihand.runtime.isaac_d405_wrist_rig import (
    CollisionBox,
    CollisionCapsuleSegment,
    load_compound_collision_proxy,
    resolve_d405_wrist_rig_runtimes,
)
from wujihand.runtime.isaac_dual_scene import resolve_dual_side_runtimes


ROOT = Path(__file__).parents[2]
SESSION = (
    ROOT
    / "configs/sessions/"
    "isaac_nero_dual_hand2_d405_wrist_rig_physical_simulation_nominal_v1.yaml"
)


def test_d405_session_resolves_two_complete_passive_wrist_rigs() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION)
    rigs = resolve_d405_wrist_rig_runtimes(ROOT, resolved)

    assert len(resolved.instances) == 8
    assert len(resolved.session.runtime.control_layouts) == 4
    assert tuple(rig.side for rig in rigs) == ("left", "right")
    assert all(len(rig.mount_collision.primitives) == 14 for rig in rigs)
    assert all(len(rig.camera_collision.primitives) == 1 for rig in rigs)
    assert all(rig.camera_profile_path.name.endswith("wide_angle_140_v1.yaml") for rig in rigs)
    assert all(rig.camera_profile.optics.horizontal_fov_deg == 140.0 for rig in rigs)
    assert all(rig.camera_profile.simulation_only for rig in rigs)
    assert all("not a physical RealSense D405" in rig.simulation_warning for rig in rigs)
    assert tuple(
        side.hand_instance_id
        for side in resolve_dual_side_runtimes(ROOT, resolved)
    ) == ("hand_left", "hand_right")
    assert rigs[0].body_in_hand.translation_m == pytest.approx(
        (-0.055, -0.09, 0.03)
    )
    assert rigs[1].body_in_hand.translation_m == pytest.approx(
        (-0.055, 0.09, 0.03)
    )
    assert rigs[0].optical_in_hand.translation_m[1] == pytest.approx(
        -rigs[1].optical_in_hand.translation_m[1]
    )


def test_mount_proxy_retains_boxes_capsules_and_no_d405_box_in_c2() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION)
    left = resolve_d405_wrist_rig_runtimes(ROOT, resolved)[0]

    boxes = tuple(
        item for item in left.mount_collision.primitives if isinstance(item, CollisionBox)
    )
    capsules = tuple(
        item
        for item in left.mount_collision.primitives
        if isinstance(item, CollisionCapsuleSegment)
    )
    assert len(boxes) == 6
    assert len(capsules) == 8
    assert isinstance(left.camera_collision.primitives[0], CollisionBox)


def test_collision_proxy_rejects_side_drift(tmp_path: Path) -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION)
    right = resolve_d405_wrist_rig_runtimes(ROOT, resolved)[1]
    document = yaml.safe_load(right.mount_collision_path.read_text(encoding="utf-8"))
    document["side"] = "left"
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="side"):
        load_compound_collision_proxy(
            invalid,
            expected_side="right",
            expected_canonical_frame="hand_interface",
        )
def test_runtime_is_immutable_value_data() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION)
    right = resolve_d405_wrist_rig_runtimes(ROOT, resolved)[1]

    changed = replace(right, side="left")
    assert right.side == "right"
    assert changed.side == "left"
