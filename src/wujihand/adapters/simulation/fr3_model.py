"""Strict, simulator-neutral loader for the pinned Menagerie FR3 v2 layout."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import numpy.typing as npt

from wujihand.domain.joints import FloatArray

import yaml


@dataclass(frozen=True, slots=True)
class Fr3ModelProfile:
    """The position contract exposed by the pinned FR3 v2 MJCF.

    Velocity limits are deliberately absent: Menagerie does not encode them,
    and a simulator profile must not masquerade as a future libfranka safety
    contract.
    """

    names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    home_position: FloatArray
    base_body_name: str
    flange_body_name: str
    home_keyframe_name: str
    provenance: dict[str, str]

    @property
    def size(self) -> int:
        return len(self.names)

    def validate_position(
        self, values: Sequence[float] | npt.NDArray[np.floating[Any]]
    ) -> FloatArray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (self.size,):
            raise ValueError(f"expected arm vector shape {(self.size,)}, got {array.shape}")
        if not np.isfinite(array).all():
            raise ValueError("arm vector contains NaN or infinity")
        lower = np.asarray(self.lower, dtype=np.float64)
        upper = np.asarray(self.upper, dtype=np.float64)
        if np.any(array < lower) or np.any(array > upper):
            raise ValueError("arm vector exceeds the pinned FR3 v2 joint range")
        return array


def load_fr3_model_profile(path: str | Path) -> Fr3ModelProfile:
    """Load and fully validate the pinned Menagerie FR3 v2 profile."""

    data: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported FR3 model profile schema")
    if data.get("robot") != "franka_fr3_v2" or data.get("backend") != "mujoco_menagerie":
        raise ValueError("profile is not the MuJoCo Menagerie FR3 v2 model")
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != 7:
        raise ValueError("FR3 v2 profile must contain seven joints")
    names = tuple(str(joint["name"]) for joint in joints)
    if len(set(names)) != 7:
        raise ValueError("FR3 v2 joint names must be unique")
    lower = tuple(float(joint["lower"]) for joint in joints)
    upper = tuple(float(joint["upper"]) for joint in joints)
    limits = np.asarray([lower, upper], dtype=np.float64)
    if not np.isfinite(limits).all() or np.any(limits[0] >= limits[1]):
        raise ValueError("FR3 v2 joint ranges must be finite and ordered")
    home = np.asarray(data["home_position"], dtype=np.float64)
    provenance = {key: str(value) for key, value in data["derived_from"].items()}
    profile = Fr3ModelProfile(
        names=names,
        lower=lower,
        upper=upper,
        home_position=home,
        base_body_name=str(data["base_body"]),
        flange_body_name=str(data["flange_body"]),
        home_keyframe_name=str(data["home_keyframe"]),
        provenance=provenance,
    )
    profile.validate_position(home)
    return profile


__all__ = ["Fr3ModelProfile", "load_fr3_model_profile"]
