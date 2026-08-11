"""Scene-neutral dual-NERO simulation startup profile."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np
import yaml

from wujihand.domain.joints import FloatArray


SCHEMA = "wujihand.nero_dual_simulation_startup.v1"


@dataclass(frozen=True, slots=True)
class NeroSimulationArmDriveGains:
    stiffness: float
    damping: float


@dataclass(frozen=True, slots=True)
class NeroSimulationInitialArmPosition:
    instance_id: str
    group_id: str
    layout_id: str
    q7_rad: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class NeroDualSimulationStartupProfile:
    profile_id: str
    status: str
    initial_arm_positions: tuple[NeroSimulationInitialArmPosition, ...]
    arm_drive_gains: NeroSimulationArmDriveGains
    assumptions: tuple[str, ...]

    def initial_position(
        self,
        instance_id: str,
        group_id: str,
        layout_id: str,
    ) -> FloatArray:
        route = (instance_id, group_id, layout_id)
        for position in self.initial_arm_positions:
            if (position.instance_id, position.group_id, position.layout_id) == route:
                return np.asarray(position.q7_rad, dtype=np.float64)
        raise KeyError(f"startup profile has no route {route!r}")


def load_nero_dual_simulation_startup_profile(
    path: str | Path,
) -> NeroDualSimulationStartupProfile:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping) or raw.get("schema") != SCHEMA:
        raise ValueError(f"NERO startup profile schema must be {SCHEMA!r}")

    positions = tuple(_position(item) for item in raw["initial_arm_positions"])
    routes = {
        (item.instance_id, item.group_id, item.layout_id) for item in positions
    }
    expected = {
        ("nero_left", "arm_joints", "agilex_nero_q7_v1"),
        ("nero_right", "arm_joints", "agilex_nero_q7_v1"),
    }
    if routes != expected or len(positions) != 2:
        raise ValueError("NERO startup profile must define the two exact arm routes")

    gains = cast(Mapping[str, object], raw["arm_drive_gains"])
    stiffness = float(cast(Any, gains["stiffness"]))
    damping = float(cast(Any, gains["damping"]))
    if stiffness <= 0.0 or damping <= 0.0:
        raise ValueError("NERO simulation drive gains must be positive")

    return NeroDualSimulationStartupProfile(
        profile_id=str(raw["profile_id"]),
        status=str(raw["status"]),
        initial_arm_positions=positions,
        arm_drive_gains=NeroSimulationArmDriveGains(
            stiffness=stiffness,
            damping=damping,
        ),
        assumptions=tuple(str(item) for item in raw["assumptions"]),
    )


def _position(value: object) -> NeroSimulationInitialArmPosition:
    item = cast(Mapping[str, object], value)
    values = tuple(float(number) for number in cast(list[float], item["q7_rad"]))
    if len(values) != 7 or not np.isfinite(values).all():
        raise ValueError("NERO startup q7 must contain seven finite values")
    q7 = (
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
    )
    return NeroSimulationInitialArmPosition(
        instance_id=str(item["instance_id"]),
        group_id=str(item["group_id"]),
        layout_id=str(item["layout_id"]),
        q7_rad=q7,
    )


__all__ = [
    "NeroDualSimulationStartupProfile",
    "NeroSimulationArmDriveGains",
    "NeroSimulationInitialArmPosition",
    "load_nero_dual_simulation_startup_profile",
]
