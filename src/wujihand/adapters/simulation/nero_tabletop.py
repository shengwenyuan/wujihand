"""Strict, backend-neutral loader for the dual-NERO tabletop qualification pose."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np
import yaml

from wujihand.domain.joints import FloatArray


NERO_DUAL_TABLETOP_QUALIFICATION_SCHEMA = (
    "wujihand.nero_dual_tabletop_qualification.v1"
)
NERO_DUAL_TABLETOP_QUALIFICATION_PROFILE_ID = (
    "isaac_nero_dual_tabletop_qualification_v1"
)
NERO_DUAL_TABLETOP_QUALIFICATION_STATUS = (
    "simulation_nominal_pending_measured_workcell"
)
NERO_DUAL_TABLETOP_LAYOUT_ID = "agilex_nero_q7_v1"
NERO_DUAL_TABLETOP_GROUP_ID = "arm_joints"
NERO_DUAL_TABLETOP_INSTANCES = ("nero_left", "nero_right")

_ROOT_KEYS = frozenset(
    {
        "schema",
        "profile_id",
        "status",
        "units",
        "initial_arm_positions",
        "geometry_contract",
        "thresholds",
        "arm_drive_gains",
        "assumptions",
    }
)
_POSITION_KEYS = frozenset({"instance_id", "group_id", "layout_id", "q7_rad"})
_GEOMETRY_KEYS = frozenset(
    {
        "base_port_axis_local_xyz",
        "table_outward_axis_world_xyz",
        "hand_longitudinal_axis_local_xyz",
        "hand_palm_normal_axis_local_xyz",
        "table_inward_axis_world_xyz",
        "table_down_axis_world_xyz",
    }
)
_THRESHOLD_KEYS = frozenset(
    {
        "attachment_origin_max_error_m",
        "link6_cylinder_forearm_min_dot",
        "base_port_outward_min_dot",
        "hand_world_inward_min_dot",
        "hand_world_vertical_abs_max",
        "hand_palm_down_min_dot",
        "forearm_world_vertical_abs_max",
        "initial_q7_max_error_rad",
    }
)
_ARM_DRIVE_GAIN_KEYS = frozenset({"stiffness", "damping"})
_EXPECTED_ROUTES = frozenset(
    (
        instance_id,
        NERO_DUAL_TABLETOP_GROUP_ID,
        NERO_DUAL_TABLETOP_LAYOUT_ID,
    )
    for instance_id in NERO_DUAL_TABLETOP_INSTANCES
)


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    return cast(Mapping[str, object], value)


def _exact_mapping(
    value: object,
    *,
    expected: frozenset[str],
    field: str,
) -> Mapping[str, object]:
    data = _mapping(value, field=field)
    actual = frozenset(data)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise ValueError(
            f"{field} keys differ: missing={missing}, unexpected={unexpected}"
        )
    return data


def _non_blank_string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _finite_float(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _unit_axis(value: object, *, field: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field} must contain exactly three values")
    axis = tuple(
        _finite_float(component, field=f"{field}[{index}]")
        for index, component in enumerate(value)
    )
    if not np.isclose(np.linalg.norm(axis), 1.0, rtol=0.0, atol=1e-12):
        raise ValueError(f"{field} must be a unit vector")
    return cast(tuple[float, float, float], axis)


def _unit_interval(value: object, *, field: str) -> float:
    result = _finite_float(value, field=field)
    if not 0.0 <= result <= 1.0:
        raise ValueError(f"{field} must be in [0, 1]")
    return result


@dataclass(frozen=True, slots=True)
class NeroTabletopInitialArmPosition:
    """One route-qualified, backend-neutral q7 initial position."""

    instance_id: str
    group_id: str
    layout_id: str
    q7_rad: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class NeroTabletopGeometryContract:
    """Unit axes used to qualify the nominal tabletop geometry."""

    base_port_axis_local_xyz: tuple[float, float, float]
    table_outward_axis_world_xyz: tuple[float, float, float]
    hand_longitudinal_axis_local_xyz: tuple[float, float, float]
    hand_palm_normal_axis_local_xyz: tuple[float, float, float]
    table_inward_axis_world_xyz: tuple[float, float, float]
    table_down_axis_world_xyz: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class NeroTabletopThresholds:
    """Finite qualification thresholds with explicit dimensions."""

    attachment_origin_max_error_m: float
    link6_cylinder_forearm_min_dot: float
    base_port_outward_min_dot: float
    hand_world_inward_min_dot: float
    hand_world_vertical_abs_max: float
    hand_palm_down_min_dot: float
    forearm_world_vertical_abs_max: float
    initial_q7_max_error_rad: float


@dataclass(frozen=True, slots=True)
class NeroTabletopArmDriveGains:
    """Positive Isaac qualification gains, not hardware-controller facts."""

    stiffness: float
    damping: float


@dataclass(frozen=True, slots=True)
class NeroDualTabletopQualificationProfile:
    """Typed compatibility profile consumed by a simulation composition root."""

    profile_id: str
    status: str
    initial_arm_positions: tuple[NeroTabletopInitialArmPosition, ...]
    geometry_contract: NeroTabletopGeometryContract
    thresholds: NeroTabletopThresholds
    arm_drive_gains: NeroTabletopArmDriveGains
    assumptions: tuple[str, ...]

    def initial_position(
        self,
        instance_id: str,
        group_id: str,
        layout_id: str,
    ) -> FloatArray:
        """Return an isolated q7 copy for one exact instance/group/layout route."""

        requested = (instance_id, group_id, layout_id)
        for position in self.initial_arm_positions:
            route = (
                position.instance_id,
                position.group_id,
                position.layout_id,
            )
            if route == requested:
                return np.asarray(position.q7_rad, dtype=np.float64).copy()
        raise KeyError(
            "no initial arm position for "
            f"instance={instance_id!r}, group={group_id!r}, layout={layout_id!r}"
        )


def load_nero_dual_tabletop_qualification_profile(
    path: str | Path,
) -> NeroDualTabletopQualificationProfile:
    """Load a strict qualification profile without importing runtime or Isaac."""

    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=_ROOT_KEYS,
        field="NERO dual tabletop qualification profile",
    )
    if data["schema"] != NERO_DUAL_TABLETOP_QUALIFICATION_SCHEMA:
        raise ValueError(
            "unsupported NERO dual tabletop qualification schema: "
            f"{data['schema']!r}"
        )
    if data["profile_id"] != NERO_DUAL_TABLETOP_QUALIFICATION_PROFILE_ID:
        raise ValueError(
            "unexpected NERO dual tabletop qualification profile ID: "
            f"{data['profile_id']!r}"
        )
    if data["status"] != NERO_DUAL_TABLETOP_QUALIFICATION_STATUS:
        raise ValueError(
            "unexpected NERO dual tabletop qualification status: "
            f"{data['status']!r}"
        )

    units = _exact_mapping(
        data["units"],
        expected=frozenset({"position"}),
        field="NERO dual tabletop qualification profile.units",
    )
    if units["position"] != "rad":
        raise ValueError("NERO dual tabletop qualification position unit must be rad")

    raw_positions = data["initial_arm_positions"]
    if not isinstance(raw_positions, list):
        raise ValueError(
            "NERO dual tabletop qualification profile.initial_arm_positions "
            "must be a list"
        )
    positions: list[NeroTabletopInitialArmPosition] = []
    routes: list[tuple[str, str, str]] = []
    for index, raw_position in enumerate(raw_positions):
        field = (
            "NERO dual tabletop qualification "
            f"profile.initial_arm_positions[{index}]"
        )
        position = _exact_mapping(
            raw_position,
            expected=_POSITION_KEYS,
            field=field,
        )
        instance_id = _non_blank_string(
            position["instance_id"],
            field=f"{field}.instance_id",
        )
        group_id = _non_blank_string(
            position["group_id"],
            field=f"{field}.group_id",
        )
        layout_id = _non_blank_string(
            position["layout_id"],
            field=f"{field}.layout_id",
        )
        route = (instance_id, group_id, layout_id)
        if route in routes:
            raise ValueError(f"{field} duplicates route {route!r}")
        routes.append(route)

        raw_q7 = position["q7_rad"]
        if not isinstance(raw_q7, list) or len(raw_q7) != 7:
            raise ValueError(f"{field}.q7_rad must contain exactly seven values")
        q7 = tuple(
            _finite_float(value, field=f"{field}.q7_rad[{joint_index}]")
            for joint_index, value in enumerate(raw_q7)
        )
        positions.append(
            NeroTabletopInitialArmPosition(
                instance_id=instance_id,
                group_id=group_id,
                layout_id=layout_id,
                q7_rad=cast(
                    tuple[float, float, float, float, float, float, float],
                    q7,
                ),
            )
        )
    if frozenset(routes) != _EXPECTED_ROUTES or len(routes) != len(_EXPECTED_ROUTES):
        missing = sorted(_EXPECTED_ROUTES - frozenset(routes))
        unexpected = sorted(frozenset(routes) - _EXPECTED_ROUTES)
        raise ValueError(
            "NERO dual tabletop qualification routes differ: "
            f"missing={missing}, unexpected={unexpected}"
        )

    geometry_data = _exact_mapping(
        data["geometry_contract"],
        expected=_GEOMETRY_KEYS,
        field="NERO dual tabletop qualification profile.geometry_contract",
    )
    geometry = NeroTabletopGeometryContract(
        base_port_axis_local_xyz=_unit_axis(
            geometry_data["base_port_axis_local_xyz"],
            field="geometry_contract.base_port_axis_local_xyz",
        ),
        table_outward_axis_world_xyz=_unit_axis(
            geometry_data["table_outward_axis_world_xyz"],
            field="geometry_contract.table_outward_axis_world_xyz",
        ),
        hand_longitudinal_axis_local_xyz=_unit_axis(
            geometry_data["hand_longitudinal_axis_local_xyz"],
            field="geometry_contract.hand_longitudinal_axis_local_xyz",
        ),
        hand_palm_normal_axis_local_xyz=_unit_axis(
            geometry_data["hand_palm_normal_axis_local_xyz"],
            field="geometry_contract.hand_palm_normal_axis_local_xyz",
        ),
        table_inward_axis_world_xyz=_unit_axis(
            geometry_data["table_inward_axis_world_xyz"],
            field="geometry_contract.table_inward_axis_world_xyz",
        ),
        table_down_axis_world_xyz=_unit_axis(
            geometry_data["table_down_axis_world_xyz"],
            field="geometry_contract.table_down_axis_world_xyz",
        ),
    )

    threshold_data = _exact_mapping(
        data["thresholds"],
        expected=_THRESHOLD_KEYS,
        field="NERO dual tabletop qualification profile.thresholds",
    )
    initial_q7_max_error_rad = _finite_float(
        threshold_data["initial_q7_max_error_rad"],
        field="thresholds.initial_q7_max_error_rad",
    )
    if initial_q7_max_error_rad < 0.0:
        raise ValueError(
            "thresholds.initial_q7_max_error_rad must be non-negative"
        )
    attachment_origin_max_error_m = _finite_float(
        threshold_data["attachment_origin_max_error_m"],
        field="thresholds.attachment_origin_max_error_m",
    )
    if attachment_origin_max_error_m < 0.0:
        raise ValueError(
            "thresholds.attachment_origin_max_error_m must be non-negative"
        )
    thresholds = NeroTabletopThresholds(
        attachment_origin_max_error_m=attachment_origin_max_error_m,
        link6_cylinder_forearm_min_dot=_unit_interval(
            threshold_data["link6_cylinder_forearm_min_dot"],
            field="thresholds.link6_cylinder_forearm_min_dot",
        ),
        base_port_outward_min_dot=_unit_interval(
            threshold_data["base_port_outward_min_dot"],
            field="thresholds.base_port_outward_min_dot",
        ),
        hand_world_inward_min_dot=_unit_interval(
            threshold_data["hand_world_inward_min_dot"],
            field="thresholds.hand_world_inward_min_dot",
        ),
        hand_world_vertical_abs_max=_unit_interval(
            threshold_data["hand_world_vertical_abs_max"],
            field="thresholds.hand_world_vertical_abs_max",
        ),
        hand_palm_down_min_dot=_unit_interval(
            threshold_data["hand_palm_down_min_dot"],
            field="thresholds.hand_palm_down_min_dot",
        ),
        forearm_world_vertical_abs_max=_unit_interval(
            threshold_data["forearm_world_vertical_abs_max"],
            field="thresholds.forearm_world_vertical_abs_max",
        ),
        initial_q7_max_error_rad=initial_q7_max_error_rad,
    )

    arm_drive_gain_data = _exact_mapping(
        data["arm_drive_gains"],
        expected=_ARM_DRIVE_GAIN_KEYS,
        field="NERO dual tabletop qualification profile.arm_drive_gains",
    )
    stiffness = _finite_float(
        arm_drive_gain_data["stiffness"],
        field="arm_drive_gains.stiffness",
    )
    damping = _finite_float(
        arm_drive_gain_data["damping"],
        field="arm_drive_gains.damping",
    )
    if stiffness <= 0.0 or damping <= 0.0:
        raise ValueError("NERO tabletop arm drive gains must be positive")
    arm_drive_gains = NeroTabletopArmDriveGains(
        stiffness=stiffness,
        damping=damping,
    )

    raw_assumptions = data["assumptions"]
    if not isinstance(raw_assumptions, list) or not raw_assumptions:
        raise ValueError(
            "NERO dual tabletop qualification profile.assumptions "
            "must be a non-empty list"
        )
    assumptions = tuple(
        _non_blank_string(
            assumption,
            field=(
                "NERO dual tabletop qualification "
                f"profile.assumptions[{index}]"
            ),
        )
        for index, assumption in enumerate(raw_assumptions)
    )
    if len(set(assumptions)) != len(assumptions):
        raise ValueError(
            "NERO dual tabletop qualification profile.assumptions "
            "must be unique"
        )

    return NeroDualTabletopQualificationProfile(
        profile_id=NERO_DUAL_TABLETOP_QUALIFICATION_PROFILE_ID,
        status=NERO_DUAL_TABLETOP_QUALIFICATION_STATUS,
        initial_arm_positions=tuple(positions),
        geometry_contract=geometry,
        thresholds=thresholds,
        arm_drive_gains=arm_drive_gains,
        assumptions=assumptions,
    )


__all__ = [
    "NERO_DUAL_TABLETOP_GROUP_ID",
    "NERO_DUAL_TABLETOP_INSTANCES",
    "NERO_DUAL_TABLETOP_LAYOUT_ID",
    "NERO_DUAL_TABLETOP_QUALIFICATION_PROFILE_ID",
    "NERO_DUAL_TABLETOP_QUALIFICATION_SCHEMA",
    "NERO_DUAL_TABLETOP_QUALIFICATION_STATUS",
    "NeroDualTabletopQualificationProfile",
    "NeroTabletopArmDriveGains",
    "NeroTabletopGeometryContract",
    "NeroTabletopInitialArmPosition",
    "NeroTabletopThresholds",
    "load_nero_dual_tabletop_qualification_profile",
]
