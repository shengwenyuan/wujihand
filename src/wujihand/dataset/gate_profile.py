"""Strict loader for the offline integrity and quality gate profile."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Final, cast

import yaml

from wujihand.domain.recording import validate_recording_token

from .release import EpisodeQualityConfig, ReleaseGateConfig


MINI_DATASET_GATE_PROFILE_SCHEMA: Final = "wujihand.mini_dataset_gate_profile.v1"


@dataclass(frozen=True, slots=True)
class MiniDatasetGateProfile:
    profile_id: str
    integrity: ReleaseGateConfig
    quality: EpisodeQualityConfig
    replay_link_position_limit_m: float
    file_sha256: str


def _mapping(value: object, *, field: str, keys: frozenset[str]) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or frozenset(value) != keys:
        raise ValueError(f"{field} keys differ")
    return cast(Mapping[str, object], value)


def _number(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    return float(value)


def _triple(value: object, *, field: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a numeric triple")
    values = tuple(_number(item, field=field) for item in value)
    if len(values) != 3:
        raise ValueError(f"{field} must be a numeric triple")
    return values


def load_mini_dataset_gate_profile(
    project_root: str | Path,
    profile_path: str | Path,
) -> MiniDatasetGateProfile:
    root = Path(project_root).resolve()
    raw_path = Path(profile_path)
    path = raw_path.resolve() if raw_path.is_absolute() else (root / raw_path).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError("mini dataset gate profile must remain inside project root") from exc
    if path.is_symlink() or not path.is_file():
        raise ValueError("mini dataset gate profile is missing or unsafe")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("mini dataset gate profile YAML is invalid") from exc
    root_value = _mapping(
        value,
        field="gate profile",
        keys=frozenset({"schema", "profile_id", "integrity", "quality"}),
    )
    if root_value["schema"] != MINI_DATASET_GATE_PROFILE_SCHEMA:
        raise ValueError("mini dataset gate profile schema differs")
    profile_id = validate_recording_token(root_value["profile_id"], field="profile_id")
    integrity = _mapping(
        root_value["integrity"],
        field="gate profile integrity",
        keys=frozenset(
            {
                "expected_control_hz",
                "expected_physics_hz",
                "q54_continuity_atol_rad",
                "simulation_time_atol_s",
                "physics_grid_time_atol_s",
                "static_reference_translation_limit_m",
                "static_reference_rotation_limit_rad",
                "replay_link_position_limit_m",
            }
        ),
    )
    quality_keys = frozenset(EpisodeQualityConfig.__dataclass_fields__)
    quality = _mapping(
        root_value["quality"],
        field="gate profile quality",
        keys=quality_keys,
    )
    release_config = ReleaseGateConfig(
        expected_control_hz=_number(
            integrity["expected_control_hz"], field="expected_control_hz"
        ),
        expected_physics_hz=_number(
            integrity["expected_physics_hz"], field="expected_physics_hz"
        ),
        q54_continuity_atol_rad=_number(
            integrity["q54_continuity_atol_rad"], field="q54_continuity_atol_rad"
        ),
        simulation_time_atol_s=_number(
            integrity["simulation_time_atol_s"], field="simulation_time_atol_s"
        ),
        physics_grid_time_atol_s=_number(
            integrity["physics_grid_time_atol_s"], field="physics_grid_time_atol_s"
        ),
        fixture_translation_drift_limit_m=_number(
            integrity["static_reference_translation_limit_m"],
            field="static_reference_translation_limit_m",
        ),
        fixture_rotation_drift_limit_rad=_number(
            integrity["static_reference_rotation_limit_rad"],
            field="static_reference_rotation_limit_rad",
        ),
    )
    quality_config = EpisodeQualityConfig(
        **{
            key: _triple(quality[key], field=key)
            for key in EpisodeQualityConfig.__dataclass_fields__
        }
    )
    return MiniDatasetGateProfile(
        profile_id=profile_id,
        integrity=release_config,
        quality=quality_config,
        replay_link_position_limit_m=_number(
            integrity["replay_link_position_limit_m"],
            field="replay_link_position_limit_m",
        ),
        file_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    )


__all__ = [
    "MINI_DATASET_GATE_PROFILE_SCHEMA",
    "MiniDatasetGateProfile",
    "load_mini_dataset_gate_profile",
]
