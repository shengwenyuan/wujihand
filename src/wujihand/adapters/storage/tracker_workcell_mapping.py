"""Strict YAML loader for simulation-only Tracker-to-workcell calibration."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Final, cast

import yaml

from wujihand.domain.pose import validate_rotation_matrix


TRACKER_WORKCELL_MAPPING_SCHEMA: Final = "wujihand.tracker_workcell_mapping.v1"
WORKCELL_SPATIAL_DELTA: Final = "workcell_spatial_delta"
SIMULATION_ONLY_SCOPE: Final = "simulation_only"

_TOKEN = re.compile(r"^[A-Za-z][A-Za-z0-9_.:/+-]{0,127}$")
_FIELDS: Final = frozenset(
    {
        "schema",
        "mapping_id",
        "tracking_frame",
        "workcell_frame",
        "tracker_to_workcell",
        "translation_scale",
        "max_translation_delta_m",
        "rotation_scale",
        "max_rotation_delta_deg",
        "relative_rotation_semantics",
        "scope",
        "provenance",
    }
)


def _token(value: object, *, field: str) -> str:
    if type(value) is not str or _TOKEN.fullmatch(value) is None:
        raise ValueError(f"{field} must be a bounded identifier")
    return value


def _number(
    value: object,
    *,
    field: str,
    lower_exclusive: float,
    upper_inclusive: float,
) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{field} must be a number")
    result = float(cast(int | float, value))
    if not math.isfinite(result) or result <= lower_exclusive or result > upper_inclusive:
        raise ValueError(f"{field} must be in ({lower_exclusive}, {upper_inclusive}]")
    return result


@dataclass(frozen=True, slots=True)
class TrackerWorkcellMapping:
    """Validated mapping and bounded simulation gains."""

    schema: str
    mapping_id: str
    tracking_frame: str
    workcell_frame: str
    tracker_to_workcell: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    translation_scale: float
    max_translation_delta_m: float
    rotation_scale: float
    max_rotation_delta_deg: float
    relative_rotation_semantics: str
    scope: str
    provenance: str

    @property
    def max_rotation_delta_rad(self) -> float:
        return math.radians(self.max_rotation_delta_deg)


def load_tracker_workcell_mapping(
    path: str | Path,
) -> TrackerWorkcellMapping:
    """Load one exact, simulation-only mapping profile."""

    source = Path(path)
    try:
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"cannot read Tracker mapping profile: {source}") from exc
    if not isinstance(payload, Mapping) or set(payload) != _FIELDS:
        raise ValueError("Tracker mapping profile fields do not match schema")
    if payload["schema"] != TRACKER_WORKCELL_MAPPING_SCHEMA:
        raise ValueError(f"Tracker mapping schema must be {TRACKER_WORKCELL_MAPPING_SCHEMA!r}")
    if payload["relative_rotation_semantics"] != WORKCELL_SPATIAL_DELTA:
        raise ValueError(f"relative_rotation_semantics must be {WORKCELL_SPATIAL_DELTA!r}")
    if payload["scope"] != SIMULATION_ONLY_SCOPE:
        raise ValueError(f"scope must be {SIMULATION_ONLY_SCOPE!r}")

    matrix = validate_rotation_matrix(payload["tracker_to_workcell"])
    matrix_tuple = cast(
        tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ],
        tuple(tuple(float(value) for value in row) for row in matrix),
    )
    return TrackerWorkcellMapping(
        schema=TRACKER_WORKCELL_MAPPING_SCHEMA,
        mapping_id=_token(payload["mapping_id"], field="mapping_id"),
        tracking_frame=_token(
            payload["tracking_frame"],
            field="tracking_frame",
        ),
        workcell_frame=_token(
            payload["workcell_frame"],
            field="workcell_frame",
        ),
        tracker_to_workcell=matrix_tuple,
        translation_scale=_number(
            payload["translation_scale"],
            field="translation_scale",
            lower_exclusive=0.0,
            upper_inclusive=1.0,
        ),
        max_translation_delta_m=_number(
            payload["max_translation_delta_m"],
            field="max_translation_delta_m",
            lower_exclusive=0.0,
            upper_inclusive=0.5,
        ),
        rotation_scale=_number(
            payload["rotation_scale"],
            field="rotation_scale",
            lower_exclusive=0.0,
            upper_inclusive=1.0,
        ),
        max_rotation_delta_deg=_number(
            payload["max_rotation_delta_deg"],
            field="max_rotation_delta_deg",
            lower_exclusive=0.0,
            upper_inclusive=90.0,
        ),
        relative_rotation_semantics=WORKCELL_SPATIAL_DELTA,
        scope=SIMULATION_ONLY_SCOPE,
        provenance=_token(payload["provenance"], field="provenance"),
    )


__all__ = [
    "SIMULATION_ONLY_SCOPE",
    "TRACKER_WORKCELL_MAPPING_SCHEMA",
    "WORKCELL_SPATIAL_DELTA",
    "TrackerWorkcellMapping",
    "load_tracker_workcell_mapping",
]
