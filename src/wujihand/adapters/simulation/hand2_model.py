"""Load the versioned Wuji Hand 2 model profile without importing Isaac."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from wujihand.domain.joints import FloatArray, JointLayout


@dataclass(frozen=True, slots=True)
class Hand2ModelProfile:
    layout: JointLayout
    rest_position: FloatArray
    provenance: dict[str, str]

    def firmware_to_backend(self, q20: Sequence[float], backend_names: Sequence[str]) -> FloatArray:
        """Reorder a firmware-layout command into simulator DOF order."""

        values = self.layout.validate_vector(q20)
        indices = self.layout.indices_for(backend_names)
        return values[np.asarray(indices, dtype=np.int64)]

    def _finger_indices_in_backend(self, backend_names: Sequence[str]) -> tuple[int, ...]:
        """Return backend indices for q20 in canonical firmware order.

        The fixed-base asset exposes exactly 20 DOFs, while the rotation-mount
        derivative exposes three additional wrist DOFs.  This method deliberately
        permits those extra DOFs but still fails closed if any canonical finger
        joint is missing or duplicated.
        """

        target = tuple(backend_names)
        if len(set(target)) != len(target):
            raise ValueError("backend DOF names must be unique")
        backend_index = {name: index for index, name in enumerate(target)}
        missing = [name for name in self.layout.names if name not in backend_index]
        if missing:
            raise ValueError(f"backend is missing Hand 2 finger DOFs: {missing}")
        return tuple(backend_index[name] for name in self.layout.names)

    def backend_to_firmware(
        self, backend_values: Sequence[float], backend_names: Sequence[str]
    ) -> FloatArray:
        """Reorder simulator feedback into the canonical firmware layout."""

        values = self.layout.validate_vector(backend_values)
        indices = np.asarray(self.layout.indices_for(backend_names), dtype=np.int64)
        firmware = np.empty(self.layout.size, dtype=np.float64)
        firmware[indices] = values
        return firmware

    def backend_full_to_firmware(
        self, backend_values: Sequence[float], backend_names: Sequence[str]
    ) -> FloatArray:
        """Select canonical q20 feedback from a backend that may have wrist DOFs."""

        values = np.asarray(backend_values, dtype=np.float64)
        if values.shape != (len(backend_names),):
            raise ValueError(
                f"expected backend vector shape {(len(backend_names),)}, got {values.shape}"
            )
        if not np.isfinite(values).all():
            raise ValueError("backend vector contains NaN or infinity")
        indices = np.asarray(self._finger_indices_in_backend(backend_names), dtype=np.int64)
        return self.layout.validate_vector(values[indices]).copy()


def load_hand2_model_profile(path: str | Path) -> Hand2ModelProfile:
    data: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported Hand 2 model profile schema")
    if data.get("product") != "wuji_hand_2_beta_1" or data.get("side") != "right":
        raise ValueError("profile is not Wuji Hand 2 Beta 1 right")
    joints = data["joints"]
    layout = JointLayout(
        names=tuple(joint["name"] for joint in joints),
        lower=tuple(float(joint["lower"]) for joint in joints),
        upper=tuple(float(joint["upper"]) for joint in joints),
        velocity=tuple(float(joint["velocity"]) for joint in joints),
    )
    if layout.size != 20:
        raise ValueError(f"expected 20 Hand 2 joints, got {layout.size}")
    rest = layout.validate_vector(data["rest_position"])
    provenance = {key: str(value) for key, value in data["derived_from"].items()}
    return Hand2ModelProfile(layout=layout, rest_position=rest, provenance=provenance)
