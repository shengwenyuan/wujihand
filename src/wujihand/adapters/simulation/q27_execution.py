"""Isaac articulation adapter for the narrow q27 execution port."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol, cast

import numpy as np
import numpy.typing as npt

from wujihand.application.teleoperation import Q27Target


class Q27Articulation(Protocol):
    def get_joint_positions(self) -> object: ...

    def set_joint_position_targets(self, positions: object) -> None: ...


class IsaacQ27ExecutionAdapter:
    """Own q27 feedback/apply calls without importing Isaac into application."""

    def __init__(
        self,
        articulations: Mapping[str, Q27Articulation],
    ) -> None:
        if set(articulations) != {"left", "right"}:
            raise ValueError(
                "q27 execution requires left and right articulations"
            )
        self._articulations = dict(articulations)

    def read_feedback_q27(
        self,
        side: str,
    ) -> npt.NDArray[np.float64]:
        articulation = self._articulation(side)
        values = np.asarray(
            articulation.get_joint_positions(),
            dtype=np.float64,
        )
        if values.shape != (1, 27) or not np.isfinite(values).all():
            raise RuntimeError(
                f"invalid {side} q27 feedback shape/value: {values.shape}"
            )
        return cast(npt.NDArray[np.float64], values[0].copy())

    def apply_target_q27(self, target: Q27Target) -> None:
        self._articulation(target.side).set_joint_position_targets(
            target.positions[np.newaxis, :]
        )

    def _articulation(self, side: str) -> Q27Articulation:
        try:
            return self._articulations[side]
        except KeyError as exc:
            raise ValueError("q27 execution side must be left or right") from exc


__all__ = ["IsaacQ27ExecutionAdapter", "Q27Articulation"]
