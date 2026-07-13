"""Canonical named joint layouts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class JointLayout:
    """Named revolute-joint layout with radian limits."""

    names: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    velocity: tuple[float, ...]

    def __post_init__(self) -> None:
        size = len(self.names)
        if size == 0 or len(set(self.names)) != size:
            raise ValueError("joint names must be non-empty and unique")
        if not (len(self.lower) == len(self.upper) == len(self.velocity) == size):
            raise ValueError("joint layout fields must have equal length")
        values = np.asarray([self.lower, self.upper, self.velocity], dtype=np.float64)
        if not np.isfinite(values).all():
            raise ValueError("joint limits must be finite")
        if np.any(values[0] >= values[1]):
            raise ValueError("every lower limit must be below its upper limit")
        if np.any(values[2] <= 0.0):
            raise ValueError("every velocity limit must be positive")

    @property
    def size(self) -> int:
        return len(self.names)

    def validate_vector(self, values: Sequence[float] | npt.NDArray[np.floating]) -> FloatArray:
        array = np.asarray(values, dtype=np.float64)
        if array.shape != (self.size,):
            raise ValueError(f"expected joint vector shape {(self.size,)}, got {array.shape}")
        if not np.isfinite(array).all():
            raise ValueError("joint vector contains NaN or infinity")
        return array

    def clamp(self, values: Sequence[float] | npt.NDArray[np.floating]) -> FloatArray:
        array = self.validate_vector(values)
        return np.clip(array, np.asarray(self.lower), np.asarray(self.upper))

    def indices_for(self, target_names: Iterable[str]) -> tuple[int, ...]:
        """Return source indices needed to emit values in ``target_names`` order."""

        target = tuple(target_names)
        if len(target) != self.size or len(set(target)) != self.size:
            raise ValueError("target joint names must be a unique full layout")
        source_index = {name: index for index, name in enumerate(self.names)}
        missing = [name for name in target if name not in source_index]
        extra = [name for name in self.names if name not in set(target)]
        if missing or extra:
            raise ValueError(f"joint layouts differ: missing={missing}, extra={extra}")
        return tuple(source_index[name] for name in target)
