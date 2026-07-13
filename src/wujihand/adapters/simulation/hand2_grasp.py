"""Physics-based acceptance criteria for the Hand 2 ball-grasp task."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
import numpy.typing as npt

from .hand2_ball_scene import FINGER_CONTACT_GROUPS, TABLE_CONTACT_GROUP


def _finite_vector(
    values: Sequence[float] | npt.NDArray[np.floating],
    field_name: str,
) -> npt.NDArray[np.float64]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (3,):
        raise ValueError(f"{field_name} must have shape (3,), got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{field_name} contains NaN or infinity")
    return array


@dataclass(frozen=True, slots=True)
class BallLiftCriteria:
    """Thresholds for a continuous, detached, low-slip hold."""

    table_top_z_m: float = 0.38
    ball_radius_m: float = 0.025
    min_bottom_clearance_m: float = 0.02
    min_hold_s: float = 1.0
    min_opposing_fingers: int = 2
    max_palm_relative_slip_m: float | None = 0.005

    def __post_init__(self) -> None:
        for name, value, positive in (
            ("table_top_z_m", self.table_top_z_m, False),
            ("ball_radius_m", self.ball_radius_m, True),
            ("min_bottom_clearance_m", self.min_bottom_clearance_m, False),
            ("min_hold_s", self.min_hold_s, False),
        ):
            if (
                not math.isfinite(value)
                or (positive and value <= 0.0)
                or (not positive and name != "table_top_z_m" and value < 0.0)
            ):
                raise ValueError(f"{name} has an invalid value")
        if (
            not isinstance(self.min_opposing_fingers, int)
            or not 1 <= self.min_opposing_fingers <= 4
        ):
            raise ValueError("min_opposing_fingers must be an integer in [1, 4]")
        if self.max_palm_relative_slip_m is not None and (
            not math.isfinite(self.max_palm_relative_slip_m)
            or self.max_palm_relative_slip_m < 0.0
        ):
            raise ValueError("max_palm_relative_slip_m must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class BallLiftSample:
    time_s: float
    ball_center_xyz_m: tuple[float, float, float]
    contact_groups: frozenset[str]
    ball_in_palm_xyz_m: tuple[float, float, float] | None

    def __post_init__(self) -> None:
        if not math.isfinite(self.time_s) or self.time_s < 0.0:
            raise ValueError("time_s must be finite and non-negative")
        _finite_vector(self.ball_center_xyz_m, "ball_center_xyz_m")
        allowed = set(FINGER_CONTACT_GROUPS) | {TABLE_CONTACT_GROUP}
        unknown = sorted(set(self.contact_groups) - allowed)
        if unknown:
            raise ValueError(f"unknown contact groups: {unknown}")
        if self.ball_in_palm_xyz_m is not None:
            _finite_vector(self.ball_in_palm_xyz_m, "ball_in_palm_xyz_m")


@dataclass(frozen=True, slots=True)
class BallLiftResult:
    passed: bool
    qualified: bool
    lifted: bool
    table_clear: bool
    thumb_contact: bool
    opposing_contact_count: int
    palm_relative_slip_m: float | None
    hold_duration_s: float
    reasons: tuple[str, ...]


class BallLiftEvaluator:
    """Evaluate each physics sample against the continuous-hold contract."""

    def __init__(self, criteria: BallLiftCriteria | None = None) -> None:
        self.criteria = criteria or BallLiftCriteria()
        self._last_time_s: float | None = None
        self._qualified_since_s: float | None = None
        self._reference_ball_in_palm: npt.NDArray[np.float64] | None = None

    def update(self, sample: BallLiftSample) -> BallLiftResult:
        if self._last_time_s is not None and sample.time_s < self._last_time_s:
            raise ValueError("BallLiftSample time_s must be monotonic")
        self._last_time_s = sample.time_s
        criteria = self.criteria
        center = _finite_vector(sample.ball_center_xyz_m, "ball_center_xyz_m")
        bottom_clearance = center[2] - criteria.ball_radius_m - criteria.table_top_z_m
        lifted = bool(bottom_clearance >= criteria.min_bottom_clearance_m)
        table_clear = TABLE_CONTACT_GROUP not in sample.contact_groups
        thumb_contact = "thumb" in sample.contact_groups
        opposing = len(set(sample.contact_groups) & {"index", "middle", "ring", "pinky"})

        reasons: list[str] = []
        if not lifted:
            reasons.append("insufficient_ball_lift")
        if not table_clear:
            reasons.append("table_contact_present")
        if not thumb_contact:
            reasons.append("thumb_contact_missing")
        if opposing < criteria.min_opposing_fingers:
            reasons.append("opposing_finger_contacts_missing")

        relative = (
            None
            if sample.ball_in_palm_xyz_m is None
            else _finite_vector(sample.ball_in_palm_xyz_m, "ball_in_palm_xyz_m")
        )
        slip: float | None = None
        if criteria.max_palm_relative_slip_m is not None and relative is None:
            reasons.append("palm_relative_position_missing")

        qualified = not reasons
        if qualified and self._qualified_since_s is None:
            self._qualified_since_s = sample.time_s
            self._reference_ball_in_palm = None if relative is None else relative.copy()
        if qualified and relative is not None and self._reference_ball_in_palm is not None:
            slip = float(np.linalg.norm(relative - self._reference_ball_in_palm))
            if (
                criteria.max_palm_relative_slip_m is not None
                and slip > criteria.max_palm_relative_slip_m
            ):
                reasons.append("palm_relative_slip_exceeded")
                qualified = False

        if not qualified:
            self._qualified_since_s = None
            self._reference_ball_in_palm = None
            hold_duration = 0.0
        else:
            assert self._qualified_since_s is not None
            hold_duration = sample.time_s - self._qualified_since_s
        passed = qualified and hold_duration >= criteria.min_hold_s
        if qualified and not passed:
            reasons.append("hold_window_incomplete")
        return BallLiftResult(
            passed=passed,
            qualified=qualified,
            lifted=lifted,
            table_clear=table_clear,
            thumb_contact=thumb_contact,
            opposing_contact_count=opposing,
            palm_relative_slip_m=slip,
            hold_duration_s=hold_duration,
            reasons=tuple(reasons),
        )


__all__ = ["BallLiftCriteria", "BallLiftEvaluator", "BallLiftResult", "BallLiftSample"]
