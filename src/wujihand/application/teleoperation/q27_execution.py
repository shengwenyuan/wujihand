"""Narrow q27 composition and execution boundary."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class Q27Target:
    side: str
    positions: FloatArray

    def __post_init__(self) -> None:
        if self.side not in {"left", "right"}:
            raise ValueError("q27 target side must be left or right")
        values = np.asarray(self.positions, dtype=np.float64)
        if values.shape != (27,) or not np.isfinite(values).all():
            raise ValueError("q27 target must contain 27 finite positions")
        object.__setattr__(self, "positions", values.copy())


class Q27ExecutionPort(Protocol):
    def read_feedback_q27(self, side: str) -> FloatArray: ...

    def apply_target_q27(self, target: Q27Target) -> None: ...


def compose_partitioned_q27_target(
    *,
    side: str,
    arm_indices_q7: Iterable[int],
    hand_indices_q20: Iterable[int],
    arm_q7: object,
    hand_q20: object,
) -> Q27Target:
    """Compose exactly one canonical q7 and q20 partition."""

    arm_indices = _indices(
        arm_indices_q7,
        expected_size=7,
        field="arm_indices_q7",
    )
    hand_indices = _indices(
        hand_indices_q20,
        expected_size=20,
        field="hand_indices_q20",
    )
    if set(arm_indices) & set(hand_indices):
        raise ValueError("q7 and q20 partitions must be disjoint")
    if set(arm_indices + hand_indices) != set(range(27)):
        raise ValueError("q7 and q20 partitions must exactly cover q27")
    arm = _vector(arm_q7, size=7, field="arm_q7")
    hand = _vector(hand_q20, size=20, field="hand_q20")
    result = np.empty(27, dtype=np.float64)
    result[np.asarray(arm_indices, dtype=np.int64)] = arm
    result[np.asarray(hand_indices, dtype=np.int64)] = hand
    return Q27Target(side=side, positions=result)


def _indices(
    values: Iterable[int],
    *,
    expected_size: int,
    field: str,
) -> tuple[int, ...]:
    if isinstance(values, (str, bytes, bytearray)):
        raise ValueError(f"{field} must contain q27 indices")
    items = tuple(values)
    if (
        len(items) != expected_size
        or any(type(index) is not int or not 0 <= index < 27 for index in items)
        or len(set(items)) != expected_size
    ):
        raise ValueError(
            f"{field} must contain {expected_size} unique q27 indices"
        )
    return items


def _vector(value: object, *, size: int, field: str) -> FloatArray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            f"{field} must contain {size} finite positions"
        ) from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must contain {size} finite positions")
    return result


__all__ = [
    "Q27ExecutionPort",
    "Q27Target",
    "compose_partitioned_q27_target",
]
