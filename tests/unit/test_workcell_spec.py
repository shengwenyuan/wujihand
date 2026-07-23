from __future__ import annotations

from typing import Any

import pytest

from wujihand.specs import PrimitiveSpec, WorkcellSpec


def _pose() -> dict[str, list[float]]:
    return {
        "position_m": [0.0, 0.0, 0.0],
        "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
    }


def _workcell() -> dict[str, Any]:
    return {
        "schema": "wujihand.workcell.v1",
        "workcell_id": "dual_arm_bench",
        "world_frame": "world",
        "frames": [
            {"frame_id": "bench", "parent": "world", "transform": _pose()},
            {"frame_id": "camera", "parent": "bench", "transform": _pose()},
        ],
        "mounts": [
            {"mount_id": "left_mount", "frame": "bench", "transform": _pose()},
            {"mount_id": "right_mount", "frame": "bench", "transform": _pose()},
        ],
        "entities": [
            {
                "entity_id": "table",
                "frame": "bench",
                "transform": _pose(),
                "primitive": {"kind": "box", "size_m": [1.6, 1.0, 0.06]},
                "mobility": "fixed",
                "mass_kg": None,
            },
            {
                "entity_id": "ball",
                "frame": "bench",
                "transform": _pose(),
                "primitive": {"kind": "sphere", "radius_m": 0.04},
                "mobility": "dynamic",
                "mass_kg": 0.05,
            },
        ],
        "compatibility_profile": None,
    }


def test_workcell_round_trip_and_mount_lookup() -> None:
    workcell = WorkcellSpec.from_mapping(_workcell())

    assert WorkcellSpec.from_mapping(workcell.to_mapping()) == workcell
    assert workcell.mount("right_mount").frame == "bench"


@pytest.mark.parametrize(
    ("mapping", "message"),
    [
        ({"kind": "box", "size_m": [1.0, 0.0, 1.0]}, "positive"),
        ({"kind": "sphere", "radius_m": -1.0}, "positive"),
        ({"kind": "capsule", "radius_m": 1.0}, "kind must be one of"),
        ({"kind": "plane", "size_m": [1.0, 1.0]}, "unexpected"),
    ],
)
def test_primitive_spec_is_closed_and_positive(
    mapping: dict[str, Any], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        PrimitiveSpec.from_mapping(mapping)


def test_workcell_rejects_frame_cycles_unknown_mount_frames_and_extra_keys() -> None:
    cyclic = _workcell()
    cyclic["frames"][0]["parent"] = "camera"
    with pytest.raises(ValueError, match="acyclic graph"):
        WorkcellSpec.from_mapping(cyclic)

    unknown_mount = _workcell()
    unknown_mount["mounts"][0]["frame"] = "missing"
    with pytest.raises(ValueError, match="unknown frame"):
        WorkcellSpec.from_mapping(unknown_mount)

    extra = _workcell()
    extra["backend"] = "mujoco"
    with pytest.raises(ValueError, match="unexpected"):
        WorkcellSpec.from_mapping(extra)


def test_workcell_requires_positive_dynamic_mass_and_null_fixed_mass() -> None:
    zero_mass = _workcell()
    zero_mass["entities"][1]["mass_kg"] = 0.0
    with pytest.raises(ValueError, match="positive"):
        WorkcellSpec.from_mapping(zero_mass)

    fixed_mass = _workcell()
    fixed_mass["entities"][0]["mass_kg"] = 50.0
    with pytest.raises(ValueError, match="must be null"):
        WorkcellSpec.from_mapping(fixed_mass)
