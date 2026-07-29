"""Backend-neutral diagnostics for Tracker arm teleoperation resets."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import numpy.typing as npt

from wujihand.domain.joints import FloatArray, JointLayout
from wujihand.domain.pose import (
    quaternion_geodesic_distance_rad,
    validate_host_time_ns,
    validate_unit_quaternion_wxyz,
)


@dataclass(frozen=True, slots=True)
class TrackerTargetMotion:
    """Cartesian target change between two accepted Tracker samples."""

    sample_interval_s: float
    translation_step_m: float
    translation_speed_m_s: float
    rotation_step_rad: float
    rotation_speed_rad_s: float


@dataclass(frozen=True, slots=True)
class JointLimitMargin:
    """One joint position and its signed margin to the nearest limit."""

    joint_name: str
    position_rad: float
    lower_limit_rad: float
    upper_limit_rad: float
    nearest_limit: str
    margin_to_nearest_limit_rad: float
    within_limits: bool


def tracker_target_motion(
    previous_position_m: object,
    previous_orientation_wxyz: Sequence[float] | npt.NDArray[np.floating],
    previous_time_ns: int,
    current_position_m: object,
    current_orientation_wxyz: Sequence[float] | npt.NDArray[np.floating],
    current_time_ns: int,
) -> TrackerTargetMotion:
    """Measure target step and rate without depending on Isaac or transport."""

    previous_time = validate_host_time_ns(previous_time_ns)
    current_time = validate_host_time_ns(current_time_ns)
    if current_time <= previous_time:
        raise ValueError("current_time_ns must be greater than previous_time_ns")
    previous_position = _position(previous_position_m, field="previous_position_m")
    current_position = _position(current_position_m, field="current_position_m")
    previous_orientation = validate_unit_quaternion_wxyz(previous_orientation_wxyz)
    current_orientation = validate_unit_quaternion_wxyz(current_orientation_wxyz)
    interval_s = (current_time - previous_time) / 1_000_000_000
    translation_step_m = float(np.linalg.norm(current_position - previous_position))
    rotation_step_rad = quaternion_geodesic_distance_rad(
        previous_orientation,
        current_orientation,
    )
    return TrackerTargetMotion(
        sample_interval_s=interval_s,
        translation_step_m=translation_step_m,
        translation_speed_m_s=translation_step_m / interval_s,
        rotation_step_rad=rotation_step_rad,
        rotation_speed_rad_s=rotation_step_rad / interval_s,
    )


def joint_limit_margins(
    layout: JointLayout,
    positions_rad: Sequence[float] | npt.NDArray[np.floating],
) -> tuple[JointLimitMargin, ...]:
    """Return signed per-joint margins using the canonical simulation layout."""

    if not isinstance(layout, JointLayout):
        raise TypeError("layout must be a JointLayout")
    positions = layout.validate_vector(positions_rad)
    result: list[JointLimitMargin] = []
    for name, position, lower, upper in zip(
        layout.names,
        positions,
        layout.lower,
        layout.upper,
        strict=True,
    ):
        lower_margin = float(position - lower)
        upper_margin = float(upper - position)
        if lower_margin <= upper_margin:
            nearest_limit = "lower"
            nearest_margin = lower_margin
        else:
            nearest_limit = "upper"
            nearest_margin = upper_margin
        result.append(
            JointLimitMargin(
                joint_name=name,
                position_rad=float(position),
                lower_limit_rad=float(lower),
                upper_limit_rad=float(upper),
                nearest_limit=nearest_limit,
                margin_to_nearest_limit_rad=nearest_margin,
                within_limits=lower_margin >= 0.0 and upper_margin >= 0.0,
            )
        )
    return tuple(result)


def nearest_joint_limit_margin(
    layout: JointLayout,
    positions_rad: Sequence[float] | npt.NDArray[np.floating],
) -> JointLimitMargin:
    """Return the joint with the smallest signed limit margin."""

    return min(
        joint_limit_margins(layout, positions_rad),
        key=lambda item: item.margin_to_nearest_limit_rad,
    )


def _position(value: object, *, field: str) -> FloatArray:
    try:
        position = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} must be a finite three-vector") from exc
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError(f"{field} must be a finite three-vector")
    return position


__all__ = [
    "JointLimitMargin",
    "TrackerTargetMotion",
    "joint_limit_margins",
    "nearest_joint_limit_margin",
    "tracker_target_motion",
]
