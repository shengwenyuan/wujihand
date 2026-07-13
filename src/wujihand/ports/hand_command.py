"""Canonical atomic hand command shared by input and simulation adapters."""

from __future__ import annotations

import math
import uuid
from dataclasses import dataclass
from numbers import Real
from typing import Final, Sequence

import numpy as np

from wujihand.domain import HAND2_RIGHT_LAYOUT
from wujihand.domain.joints import FloatArray


HAND_COMMAND_SCHEMA: Final = "wujihand.hand_command.v2"
HAND_COMMAND_LAYOUT: Final = "wuji_hand2_right_firmware_v1"
HAND_COMMAND_POSE_FRAME: Final = "hand2_right_neutral"
HAND_COMMAND_QUAT_ORDER: Final = "wxyz"
QUATERNION_NORM_TOLERANCE: Final = 1.0e-6


def _readonly_vector(
    values: Sequence[float] | FloatArray,
    *,
    name: str,
    size: int,
) -> FloatArray:
    try:
        vector = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a numeric vector") from exc
    if vector.shape != (size,):
        raise ValueError(f"{name} must have shape {(size,)}, got {vector.shape}")
    if not np.isfinite(vector).all():
        raise ValueError(f"{name} contains NaN or infinity")
    result = vector.copy()
    result.flags.writeable = False
    return result


@dataclass(frozen=True, slots=True, kw_only=True, eq=False)
class HandCommand:
    """One indivisible finger-and-root command.

    ``host_time_ns`` is a reading from the producer host's monotonic clock, not a
    Unix/wall-clock timestamp. ``root_delta_quat_wxyz`` is the active rotation
    relative to the calibrated Hand 2 neutral frame.
    """

    session_id: str
    sequence: int
    host_time_ns: int
    q20: FloatArray
    root_delta_quat_wxyz: FloatArray
    quality: float
    calibration_id: str
    schema: str = HAND_COMMAND_SCHEMA
    layout: str = HAND_COMMAND_LAYOUT
    pose_frame: str = HAND_COMMAND_POSE_FRAME
    quat_order: str = HAND_COMMAND_QUAT_ORDER

    def __post_init__(self) -> None:
        if type(self.schema) is not str or self.schema != HAND_COMMAND_SCHEMA:
            raise ValueError(f"schema must be {HAND_COMMAND_SCHEMA!r}")
        if type(self.layout) is not str or self.layout != HAND_COMMAND_LAYOUT:
            raise ValueError(f"layout must be {HAND_COMMAND_LAYOUT!r}")
        if type(self.pose_frame) is not str or self.pose_frame != HAND_COMMAND_POSE_FRAME:
            raise ValueError(f"pose_frame must be {HAND_COMMAND_POSE_FRAME!r}")
        if type(self.quat_order) is not str or self.quat_order != HAND_COMMAND_QUAT_ORDER:
            raise ValueError(f"quat_order must be {HAND_COMMAND_QUAT_ORDER!r}")

        if type(self.session_id) is not str:
            raise ValueError("session_id must be a UUID string")
        try:
            uuid.UUID(self.session_id)
        except ValueError as exc:
            raise ValueError("session_id must be a UUID string") from exc
        if type(self.sequence) is not int or type(self.host_time_ns) is not int:
            raise ValueError("sequence and host_time_ns must be integers")
        if self.sequence < 0 or self.host_time_ns < 0:
            raise ValueError("sequence and host_time_ns must be non-negative")

        q20 = _readonly_vector(self.q20, name="q20", size=HAND2_RIGHT_LAYOUT.size)
        HAND2_RIGHT_LAYOUT.validate_vector(q20)
        root_quat = _readonly_vector(
            self.root_delta_quat_wxyz,
            name="root_delta_quat_wxyz",
            size=4,
        )
        norm = float(np.linalg.norm(root_quat))
        if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
            raise ValueError("root_delta_quat_wxyz must have unit norm")

        if isinstance(self.quality, bool) or not isinstance(self.quality, Real):
            raise ValueError("quality must be a finite number in [0, 1]")
        quality = float(self.quality)
        if not math.isfinite(quality) or not 0.0 <= quality <= 1.0:
            raise ValueError("quality must be a finite number in [0, 1]")
        if type(self.calibration_id) is not str or not self.calibration_id.strip():
            raise ValueError("calibration_id must be a non-empty string")
        if len(self.calibration_id) > 128:
            raise ValueError("calibration_id must not exceed 128 characters")

        object.__setattr__(self, "q20", q20)
        object.__setattr__(self, "root_delta_quat_wxyz", root_quat)
        object.__setattr__(self, "quality", quality)
