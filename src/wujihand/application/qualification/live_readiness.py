"""Backend-neutral q27 readiness policies for interactive simulation inputs."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
import re

import numpy as np


_POLICY_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


@dataclass(frozen=True, slots=True)
class Q27ReadinessPolicy:
    """Bounded window policy applied before an interactive input is armed."""

    policy_id: str
    window_frames: int
    minimum_windows: int
    maximum_windows: int
    max_window_delta_rad: float
    require_convergence: bool

    def __post_init__(self) -> None:
        if type(self.policy_id) is not str or _POLICY_ID.fullmatch(self.policy_id) is None:
            raise ValueError("policy_id must be a bounded identifier")
        if type(self.window_frames) is not int or self.window_frames < 1:
            raise ValueError("window_frames must be a positive integer")
        if type(self.minimum_windows) is not int or self.minimum_windows < 2:
            raise ValueError("minimum_windows must be an integer of at least two")
        if type(self.maximum_windows) is not int or self.maximum_windows < self.minimum_windows:
            raise ValueError("maximum_windows must be at least minimum_windows")
        if (
            isinstance(self.max_window_delta_rad, bool)
            or not isinstance(self.max_window_delta_rad, (int, float))
            or not math.isfinite(float(self.max_window_delta_rad))
            or float(self.max_window_delta_rad) <= 0.0
        ):
            raise ValueError("max_window_delta_rad must be finite and positive")
        if type(self.require_convergence) is not bool:
            raise ValueError("require_convergence must be a bool")
        object.__setattr__(
            self,
            "max_window_delta_rad",
            float(self.max_window_delta_rad),
        )


FULL_SCRIPTED_Q27_SETTLING_POLICY = Q27ReadinessPolicy(
    policy_id="nv2.scripted_q27_settling.v1",
    window_frames=60,
    minimum_windows=2,
    maximum_windows=8,
    max_window_delta_rad=0.005,
    require_convergence=True,
)


GLOVE_LIVE_Q27_READINESS_POLICY = Q27ReadinessPolicy(
    policy_id="nv2.glove_live_q27_readiness.v1",
    window_frames=15,
    minimum_windows=2,
    maximum_windows=4,
    max_window_delta_rad=0.03,
    require_convergence=False,
)


ROS_TELEOP_Q27_SETTLING_POLICY = Q27ReadinessPolicy(
    policy_id="nv5.ros_teleop_q27_settling.v1",
    window_frames=60,
    minimum_windows=2,
    maximum_windows=10,
    max_window_delta_rad=0.005,
    require_convergence=True,
)


def q27_window_max_delta_rad(
    previous: Mapping[str, object],
    current: Mapping[str, object],
) -> float:
    """Return the maximum absolute delta across matching finite q27 sides."""

    if not previous or set(previous) != set(current):
        raise ValueError("q27 readiness windows must contain the same non-empty sides")
    maximum = 0.0
    for side in sorted(previous):
        previous_values = np.asarray(previous[side], dtype=np.float64)
        current_values = np.asarray(current[side], dtype=np.float64)
        if (
            previous_values.shape != (27,)
            or current_values.shape != (27,)
            or not np.isfinite(previous_values).all()
            or not np.isfinite(current_values).all()
        ):
            raise ValueError("each q27 readiness side must contain 27 finite values")
        maximum = max(
            maximum,
            float(np.max(np.abs(current_values - previous_values))),
        )
    return maximum


def joint_target_max_errors_rad(
    actual: Mapping[str, object],
    target: Mapping[str, object],
) -> dict[str, float]:
    """Return the maximum absolute joint-target error for every side."""

    if not actual or set(actual) != set(target):
        raise ValueError("joint target inputs must contain the same non-empty sides")
    errors: dict[str, float] = {}
    for side in sorted(actual):
        actual_values = np.asarray(actual[side], dtype=np.float64)
        target_values = np.asarray(target[side], dtype=np.float64)
        if (
            actual_values.ndim != 1
            or actual_values.size == 0
            or actual_values.shape != target_values.shape
            or not np.isfinite(actual_values).all()
            or not np.isfinite(target_values).all()
        ):
            raise ValueError("joint target inputs must contain matching finite vectors")
        errors[side] = float(np.max(np.abs(actual_values - target_values)))
    return errors


__all__ = [
    "FULL_SCRIPTED_Q27_SETTLING_POLICY",
    "GLOVE_LIVE_Q27_READINESS_POLICY",
    "ROS_TELEOP_Q27_SETTLING_POLICY",
    "Q27ReadinessPolicy",
    "joint_target_max_errors_rad",
    "q27_window_max_delta_rad",
]
