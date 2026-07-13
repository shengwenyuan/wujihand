"""Convert MediaPipe right-hand world landmarks into a palm orientation.

This adapter consumes the numeric ``(21, 3)`` result only; no MediaPipe SDK
object crosses into the application layer.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np
import numpy.typing as npt

from wujihand.domain.pose import (
    OrientationSample,
    align_quaternion_hemisphere,
    rotation_matrix_to_quaternion_wxyz,
    validate_frame_id,
    validate_host_time_ns,
)


class MediaPipePalmOrientationEstimator:
    """Estimate the Hand 2 right palm frame from landmarks 0, 5, 9, and 17.

    The estimated axes intentionally match the Hand 2 upright-neutral design:

    * ``+Z``: wrist landmark 0 toward middle MCP landmark 9;
    * ``+Y``: pinky MCP landmark 17 toward index MCP landmark 5, projected
      perpendicular to ``+Z``;
    * ``+X``: ``+Y cross +Z`` (the right-palm normal).

    The resulting matrix has these axes as columns.  Successive quaternion
    signs are aligned to the same hemisphere to avoid representation flips.
    """

    LANDMARK_COUNT = 21
    WRIST = 0
    INDEX_MCP = 5
    MIDDLE_MCP = 9
    PINKY_MCP = 17

    def __init__(
        self,
        *,
        frame_id: str = "mediapipe_right_palm",
        min_axis_length_m: float = 1e-4,
    ) -> None:
        self.frame_id = validate_frame_id(frame_id)
        if not math.isfinite(min_axis_length_m) or min_axis_length_m <= 0.0:
            raise ValueError("min_axis_length_m must be finite and positive")
        self.min_axis_length_m = min_axis_length_m
        self._previous_quaternion: npt.NDArray[np.float64] | None = None

    def reset(self) -> None:
        """Forget quaternion-sign history between independent streams."""

        self._previous_quaternion = None

    def estimate(
        self,
        world_landmarks_m: Sequence[Sequence[float]] | npt.NDArray[np.floating],
        *,
        host_time_ns: int,
        quality: float = 1.0,
    ) -> OrientationSample:
        """Return a strict, scalar-first palm-orientation sample."""

        timestamp = validate_host_time_ns(host_time_ns)
        if not math.isfinite(quality) or not 0.0 <= quality <= 1.0:
            raise ValueError("quality must be finite and in [0, 1]")
        landmarks = np.asarray(world_landmarks_m, dtype=np.float64)
        expected_shape = (self.LANDMARK_COUNT, 3)
        if landmarks.shape != expected_shape:
            raise ValueError(f"expected world landmark shape {expected_shape}, got {landmarks.shape}")
        if not np.isfinite(landmarks).all():
            raise ValueError("world landmarks contain NaN or infinity")

        wrist = landmarks[self.WRIST]
        z_raw = landmarks[self.MIDDLE_MCP] - wrist
        z_norm = float(np.linalg.norm(z_raw))
        if z_norm < self.min_axis_length_m:
            raise ValueError("wrist-to-middle axis is degenerate")
        z_axis = z_raw / z_norm

        across_raw = landmarks[self.INDEX_MCP] - landmarks[self.PINKY_MCP]
        if float(np.linalg.norm(across_raw)) < self.min_axis_length_m:
            raise ValueError("pinky-to-index axis is degenerate")
        y_orthogonal = across_raw - float(np.dot(across_raw, z_axis)) * z_axis
        y_norm = float(np.linalg.norm(y_orthogonal))
        if y_norm < self.min_axis_length_m:
            raise ValueError("palm axes are collinear")
        y_axis = y_orthogonal / y_norm

        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        rotation = np.column_stack((x_axis, y_axis, z_axis))
        quaternion = rotation_matrix_to_quaternion_wxyz(rotation)
        if self._previous_quaternion is not None:
            quaternion = align_quaternion_hemisphere(quaternion, self._previous_quaternion)
        self._previous_quaternion = quaternion.copy()
        return OrientationSample(
            quat_wxyz=(
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ),
            frame_id=self.frame_id,
            host_time_ns=timestamp,
            quality=quality,
        )
