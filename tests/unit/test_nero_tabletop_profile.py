from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml

from wujihand.adapters.simulation.nero_tabletop import (
    NERO_DUAL_TABLETOP_QUALIFICATION_PROFILE_ID,
    NERO_DUAL_TABLETOP_QUALIFICATION_STATUS,
    load_nero_dual_tabletop_qualification_profile,
)


ROOT = Path(__file__).parents[2]
PROFILE = (
    ROOT
    / "configs/profiles/isaac_nero_dual_tabletop_qualification_v1.yaml"
)


def _profile_mapping() -> dict[str, Any]:
    value = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write_profile(tmp_path: Path, value: dict[str, Any]) -> Path:
    path = tmp_path / "nero-tabletop.yaml"
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def test_loads_route_qualified_initial_q7_and_geometry_contract() -> None:
    profile = load_nero_dual_tabletop_qualification_profile(PROFILE)

    assert profile.profile_id == NERO_DUAL_TABLETOP_QUALIFICATION_PROFILE_ID
    assert profile.status == NERO_DUAL_TABLETOP_QUALIFICATION_STATUS
    assert {
        (position.instance_id, position.group_id, position.layout_id)
        for position in profile.initial_arm_positions
    } == {
        ("nero_left", "arm_joints", "agilex_nero_q7_v1"),
        ("nero_right", "arm_joints", "agilex_nero_q7_v1"),
    }
    assert profile.initial_position(
        "nero_left",
        "arm_joints",
        "agilex_nero_q7_v1",
    ) == pytest.approx(
        np.deg2rad([-10.0, -45.0, 0.0, -45.0, -90.0, 0.0, 0.0])
    )
    assert profile.initial_position(
        "nero_right",
        "arm_joints",
        "agilex_nero_q7_v1",
    ) == pytest.approx(
        np.deg2rad([10.0, -45.0, 0.0, -45.0, -90.0, 0.0, 0.0])
    )
    assert profile.geometry_contract.hand_longitudinal_axis_local_xyz == (
        0.0,
        0.0,
        1.0,
    )
    assert profile.geometry_contract.hand_palm_normal_axis_local_xyz == (
        1.0,
        0.0,
        0.0,
    )
    assert profile.geometry_contract.table_down_axis_world_xyz == (
        0.0,
        0.0,
        -1.0,
    )
    assert profile.thresholds.hand_world_vertical_abs_max == pytest.approx(0.10)
    assert profile.thresholds.hand_palm_down_min_dot == pytest.approx(0.99)
    assert profile.thresholds.forearm_world_vertical_abs_max == pytest.approx(0.02)
    assert profile.arm_drive_gains.stiffness == pytest.approx(6000.0)
    assert profile.arm_drive_gains.damping == pytest.approx(212.13203435596427)
    assert (
        "tabletop_q7_is_simulation_nominal_not_a_hardware_safe_pose"
        in profile.assumptions
    )


def test_initial_position_is_an_isolated_copy_and_route_lookup_fails_closed() -> None:
    profile = load_nero_dual_tabletop_qualification_profile(PROFILE)

    first = profile.initial_position(
        "nero_left",
        "arm_joints",
        "agilex_nero_q7_v1",
    )
    first[0] = 99.0
    second = profile.initial_position(
        "nero_left",
        "arm_joints",
        "agilex_nero_q7_v1",
    )

    assert second[0] == pytest.approx(np.deg2rad(-10.0))
    with pytest.raises(KeyError, match="no initial arm position"):
        profile.initial_position(
            "nero_left",
            "arm_joints",
            "unapproved_q7_layout",
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value.update(
                {"schema": "wujihand.nero_dual_tabletop_qualification.v2"}
            ),
            "unsupported",
        ),
        (
            lambda value: value.update({"unexpected": True}),
            "keys differ",
        ),
        (
            lambda value: value["initial_arm_positions"][1].update(
                {"instance_id": "nero_left"}
            ),
            "duplicates route",
        ),
        (
            lambda value: value["initial_arm_positions"][0].update(
                {"q7_rad": [0.0] * 6}
            ),
            "exactly seven",
        ),
        (
            lambda value: value["initial_arm_positions"][0]["q7_rad"].__setitem__(
                2, float("nan")
            ),
            "finite",
        ),
        (
            lambda value: value["geometry_contract"].update(
                {"hand_longitudinal_axis_local_xyz": [0.0, 0.0, 2.0]}
            ),
            "unit vector",
        ),
        (
            lambda value: value["geometry_contract"].pop(
                "hand_palm_normal_axis_local_xyz"
            ),
            "keys differ",
        ),
        (
            lambda value: value["thresholds"].update(
                {"hand_world_vertical_abs_max": 1.01}
            ),
            r"\[0, 1\]",
        ),
        (
            lambda value: value["thresholds"].update(
                {"hand_palm_down_min_dot": -0.01}
            ),
            r"\[0, 1\]",
        ),
        (
            lambda value: value["thresholds"].update(
                {"initial_q7_max_error_rad": -0.01}
            ),
            "non-negative",
        ),
        (
            lambda value: value["arm_drive_gains"].update({"stiffness": 0.0}),
            "must be positive",
        ),
        (
            lambda value: value["arm_drive_gains"].update(
                {"damping": float("inf")}
            ),
            "must be finite",
        ),
        (
            lambda value: value["arm_drive_gains"].update(
                {"unexpected_gain": 1.0}
            ),
            "keys differ",
        ),
        (
            lambda value: value.update({"assumptions": []}),
            "non-empty",
        ),
    ),
)
def test_profile_rejects_schema_route_geometry_and_threshold_drift(
    tmp_path: Path,
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(_profile_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        load_nero_dual_tabletop_qualification_profile(
            _write_profile(tmp_path, value)
        )
