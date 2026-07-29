"""Application boundary for simulator-specific arm kinematics."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Protocol, runtime_checkable

import numpy as np

from wujihand.domain.pose import validate_unit_quaternion_wxyz


def _vector(
    value: object,
    *,
    size: int,
    field: str,
) -> tuple[float, ...]:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} must be a finite {size}-vector") from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite {size}-vector")
    return tuple(float(item) for item in result)


@dataclass(frozen=True, slots=True)
class ArmEndEffectorPose:
    """One backend-computed end-effector pose in the Workcell world frame."""

    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "position_m",
            _vector(self.position_m, size=3, field="position_m"),
        )
        quaternion = validate_unit_quaternion_wxyz(self.quat_wxyz)
        object.__setattr__(
            self,
            "quat_wxyz",
            tuple(float(item) for item in quaternion),
        )


@dataclass(frozen=True, slots=True)
class ArmKinematicsResult:
    """One IK attempt with a validated optional q7 candidate and residuals."""

    succeeded: bool
    solver_reported_success: bool
    candidate_q7_rad: tuple[float, ...] | None
    position_residual_m: float | None
    orientation_residual_rad: float | None
    reason: str

    def __post_init__(self) -> None:
        if type(self.succeeded) is not bool or type(
            self.solver_reported_success
        ) is not bool:
            raise ValueError("IK success fields must be booleans")
        if self.candidate_q7_rad is not None:
            object.__setattr__(
                self,
                "candidate_q7_rad",
                _vector(
                    self.candidate_q7_rad,
                    size=7,
                    field="candidate_q7_rad",
                ),
            )
        if self.succeeded and self.candidate_q7_rad is None:
            raise ValueError("successful IK requires a q7 candidate")
        for field in ("position_residual_m", "orientation_residual_rad"):
            value = getattr(self, field)
            if value is not None and (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(float(value))
                or float(value) < 0.0
            ):
                raise ValueError(
                    f"{field} must be finite, non-negative or None"
                )
            if value is not None:
                object.__setattr__(self, field, float(value))
        if (
            not isinstance(self.reason, str)
            or not self.reason
            or len(self.reason) > 128
        ):
            raise ValueError("reason must be a bounded non-empty string")


@runtime_checkable
class ArmKinematicsPort(Protocol):
    """One side-specific forward/inverse kinematics adapter."""

    def forward(self, q7_rad: Sequence[float]) -> ArmEndEffectorPose:
        """Return the end-effector pose for one canonical q7."""

        ...

    def solve(
        self,
        *,
        target_position_m: Sequence[float],
        target_orientation_wxyz: Sequence[float],
        warm_start_q7_rad: Sequence[float],
    ) -> ArmKinematicsResult:
        """Return one bounded IK result without mutating scene state."""

        ...


__all__ = [
    "ArmEndEffectorPose",
    "ArmKinematicsPort",
    "ArmKinematicsResult",
]
