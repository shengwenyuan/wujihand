from __future__ import annotations

from copy import deepcopy
from typing import Any

import pytest

from wujihand.specs import SessionSpec


def _ref(path: str, expected_id: str) -> dict[str, str]:
    return {"path": path, "expected_id": expected_id}


def _session() -> dict[str, Any]:
    return {
        "schema": "wujihand.session.v1",
        "session_id": "isaac_dual_hand_teleop",
        "backend": "isaac",
        "runtime_role": "teleop_consumer",
        "assembly": _ref("configs/assemblies/dual.yaml", "dual"),
        "workcell": _ref("configs/workcells/bench.yaml", "bench"),
        "bindings": {
            "left_hand": _ref("configs/bindings/isaac/left.yaml", "left_isaac"),
            "right_hand": _ref("configs/bindings/isaac/right.yaml", "right_isaac"),
        },
        "placements": {
            "left_hand": "left_mount",
            "right_hand": "right_mount",
        },
        "runtime": {
            "compatibility_profile": None,
            "transport_contract": "wujihand.q20.dual.v1",
            "control_layouts": [
                {
                    "instance_id": "left_hand",
                    "group_id": "finger_joints",
                    "layout_id": "wuji_hand2_left_v1",
                },
                {
                    "instance_id": "right_hand",
                    "group_id": "finger_joints",
                    "layout_id": "wuji_hand2_right_v1",
                },
            ],
        },
    }


def test_session_round_trip_and_mapping_lookup() -> None:
    session = SessionSpec.from_mapping(_session())

    assert SessionSpec.from_mapping(session.to_mapping()) == session
    assert session.binding_for("left_hand").expected_id == "left_isaac"
    assert session.mount_for("right_hand") == "right_mount"


def test_session_v2_owns_dataset_profile_without_launch_level_camera_args() -> None:
    value = _session()
    value["schema"] = "wujihand.session.v2"
    value["dataset_profile"] = _ref(
        "configs/profiles/mini_dataset.yaml",
        "mini_dataset_v1",
    )

    session = SessionSpec.from_mapping(value)

    assert session.dataset_profile is not None
    assert session.dataset_profile.expected_id == "mini_dataset_v1"
    assert SessionSpec.from_mapping(session.to_mapping()) == session


def test_session_rejects_extra_keys_invalid_role_and_unsafe_ref() -> None:
    extra = _session()
    extra["camera"] = "default"
    with pytest.raises(ValueError, match="unexpected"):
        SessionSpec.from_mapping(extra)

    invalid_role = _session()
    invalid_role["runtime_role"] = "everything"
    with pytest.raises(ValueError, match="runtime_role must be one of"):
        SessionSpec.from_mapping(invalid_role)

    unsafe = _session()
    unsafe["assembly"]["path"] = "../assembly.yaml"
    with pytest.raises(ValueError, match="safe project-relative"):
        SessionSpec.from_mapping(unsafe)


def test_session_rejects_duplicate_control_group_routes() -> None:
    duplicate = _session()
    duplicate_route = deepcopy(duplicate["runtime"]["control_layouts"][0])
    duplicate_route["layout_id"] = "same_size_but_different_layout"
    duplicate["runtime"]["control_layouts"].append(duplicate_route)

    with pytest.raises(ValueError, match="at most once"):
        SessionSpec.from_mapping(duplicate)


def test_session_allows_explicitly_absent_transport_and_compatibility_leaf() -> None:
    simulation = _session()
    simulation["backend"] = "mujoco"
    simulation["runtime_role"] = "simulation"
    simulation["runtime"]["transport_contract"] = None
    simulation["runtime"]["compatibility_profile"] = None

    parsed = SessionSpec.from_mapping(simulation)
    assert parsed.runtime.transport_contract is None
    assert parsed.runtime.compatibility_profile is None
