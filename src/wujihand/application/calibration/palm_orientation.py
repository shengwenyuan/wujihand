"""Neutral-pose calibration for rotation-only MediaPipe wrist control."""

from __future__ import annotations

import math
import uuid
from collections.abc import Sequence

import numpy as np
import numpy.typing as npt

from wujihand.domain.pose import (
    IDENTITY_QUATERNION_WXYZ,
    OrientationSample,
    PoseIntent,
    align_quaternion_hemisphere,
    normalized_quaternion_wxyz,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_rotation_matrix,
    rotation_matrix_to_quaternion_wxyz,
    validate_frame_id,
)


class StablePalmOrientationWindow:
    """Require a bounded run of stable palm samples before neutral capture.

    The window is deliberately separate from :class:`PalmOrientationCalibrator`:
    stability decides *when* a calibration may be captured, while the calibrator
    owns the frame transform and calibration epoch.  A motion outside the
    configured SO(3) radius restarts the consecutive window from that sample.
    """

    def __init__(
        self,
        *,
        required_samples: int = 15,
        max_spread_rad: float = math.radians(8.0),
        min_quality: float = 0.5,
        max_sample_gap_s: float = 0.10,
        frame_id: str = "mediapipe_right_palm",
    ) -> None:
        if isinstance(required_samples, bool) or not isinstance(required_samples, int):
            raise ValueError("required_samples must be an integer")
        if required_samples < 2:
            raise ValueError("required_samples must be at least 2")
        if not math.isfinite(max_spread_rad) or not 0.0 < max_spread_rad < math.pi:
            raise ValueError("max_spread_rad must be finite and in (0, pi)")
        if not math.isfinite(min_quality) or not 0.0 <= min_quality <= 1.0:
            raise ValueError("min_quality must be finite and in [0, 1]")
        if not math.isfinite(max_sample_gap_s) or max_sample_gap_s <= 0.0:
            raise ValueError("max_sample_gap_s must be finite and positive")
        self.required_samples = required_samples
        self.max_spread_rad = max_spread_rad
        self.min_quality = min_quality
        self.max_sample_gap_ns = int(max_sample_gap_s * 1e9)
        self.frame_id = validate_frame_id(frame_id)
        self._samples: list[OrientationSample] = []

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def reset(self) -> None:
        self._samples.clear()

    def add(self, sample: OrientationSample) -> OrientationSample | None:
        """Add one sample and return the averaged neutral once the window is full."""

        if not isinstance(sample, OrientationSample):
            raise TypeError("sample must be an OrientationSample")
        if sample.frame_id != self.frame_id:
            raise ValueError(
                f"expected orientation frame {self.frame_id!r}, got {sample.frame_id!r}"
            )
        if self._samples and sample.host_time_ns <= self._samples[-1].host_time_ns:
            raise ValueError("orientation window timestamps must increase strictly")
        if sample.quality < self.min_quality:
            self.reset()
            return None
        if (
            self._samples
            and sample.host_time_ns - self._samples[-1].host_time_ns > self.max_sample_gap_ns
        ):
            self._samples = [sample]
            return None

        if self._samples:
            reference = self._samples[0].quat_wxyz
            if quaternion_geodesic_distance_rad(reference, sample.quat_wxyz) > self.max_spread_rad:
                self._samples = [sample]
                return None
        else:
            self._samples.append(sample)
            return None

        self._samples.append(sample)
        if len(self._samples) < self.required_samples:
            return None

        selected: Sequence[OrientationSample] = self._samples[-self.required_samples :]
        reference_quaternion = selected[0].as_array()
        aligned = [
            align_quaternion_hemisphere(item.quat_wxyz, reference_quaternion)
            for item in selected
        ]
        mean_quaternion = normalized_quaternion_wxyz(np.sum(aligned, axis=0))
        if any(
            quaternion_geodesic_distance_rad(mean_quaternion, item.quat_wxyz)
            > self.max_spread_rad
            for item in selected
        ):
            self._samples = [sample]
            return None

        result = OrientationSample(
            quat_wxyz=(
                float(mean_quaternion[0]),
                float(mean_quaternion[1]),
                float(mean_quaternion[2]),
                float(mean_quaternion[3]),
            ),
            frame_id=self.frame_id,
            host_time_ns=selected[-1].host_time_ns,
            quality=min(item.quality for item in selected),
        )
        self.reset()
        return result


class PalmOrientationCalibrator:
    """Map measured palm orientation to a neutral-relative Hand 2 intent.

    Each clutch stores ``R0``.  Subsequent samples produce exactly
    ``R_relative = R0.T @ R``.  A clutch begins a new ``calibration_id`` and
    returns identity, which is the only event accepted for arming the pose
    supervisor.
    """

    def __init__(
        self,
        *,
        input_frame_id: str = "mediapipe_right_palm",
        output_frame_id: str = "hand2_right_neutral",
    ) -> None:
        self.input_frame_id = validate_frame_id(input_frame_id)
        self.output_frame_id = validate_frame_id(output_frame_id)
        self._neutral_rotation: npt.NDArray[np.float64] | None = None
        self._last_output = np.asarray(IDENTITY_QUATERNION_WXYZ, dtype=np.float64)
        self._calibration_id: str | None = None
        self._clutch_time_ns: int | None = None
        self._last_sample_time_ns: int | None = None

    @property
    def is_calibrated(self) -> bool:
        return self._neutral_rotation is not None

    @property
    def calibration_id(self) -> str | None:
        return self._calibration_id

    def reset(self) -> None:
        """Invalidate calibration; a new clutch is required before output."""

        self._neutral_rotation = None
        self._last_output = np.asarray(IDENTITY_QUATERNION_WXYZ, dtype=np.float64)
        self._clutch_time_ns = None
        self._last_sample_time_ns = None

    def clutch(self, sample: OrientationSample) -> PoseIntent:
        """Capture a new neutral palm orientation and emit identity."""

        self._validate_sample(sample)
        self._neutral_rotation = quaternion_wxyz_to_rotation_matrix(sample.quat_wxyz)
        self._last_output = np.asarray(IDENTITY_QUATERNION_WXYZ, dtype=np.float64)
        self._calibration_id = uuid.uuid4().hex
        self._clutch_time_ns = sample.host_time_ns
        self._last_sample_time_ns = sample.host_time_ns
        return self._intent(self._last_output, sample)

    def capture_neutral(self, sample: OrientationSample) -> PoseIntent:
        """Explicit alias for the first clutch/calibration action."""

        return self.clutch(sample)

    def apply(self, sample: OrientationSample) -> PoseIntent:
        """Apply ``R0.T @ R`` and preserve quaternion hemisphere continuity."""

        self._validate_sample(sample)
        if self._neutral_rotation is None or self._clutch_time_ns is None:
            raise RuntimeError("palm orientation is not calibrated; clutch first")
        if sample.host_time_ns < self._clutch_time_ns:
            raise ValueError("sample predates the active clutch")
        if self._last_sample_time_ns is not None and sample.host_time_ns < self._last_sample_time_ns:
            raise ValueError("orientation samples must be monotonic")

        current_rotation = quaternion_wxyz_to_rotation_matrix(sample.quat_wxyz)
        relative_rotation = self._neutral_rotation.T @ current_rotation
        relative_quaternion = rotation_matrix_to_quaternion_wxyz(relative_rotation)
        relative_quaternion = align_quaternion_hemisphere(relative_quaternion, self._last_output)
        self._last_output = relative_quaternion
        self._last_sample_time_ns = sample.host_time_ns
        return self._intent(relative_quaternion, sample)

    def _validate_sample(self, sample: OrientationSample) -> None:
        if not isinstance(sample, OrientationSample):
            raise TypeError("sample must be an OrientationSample")
        if sample.frame_id != self.input_frame_id:
            raise ValueError(
                f"expected orientation frame {self.input_frame_id!r}, got {sample.frame_id!r}"
            )

    def _intent(
        self,
        quaternion: npt.NDArray[np.float64],
        sample: OrientationSample,
    ) -> PoseIntent:
        if self._calibration_id is None:
            raise RuntimeError("palm orientation is not calibrated; clutch first")
        return PoseIntent(
            quat_wxyz=(
                float(quaternion[0]),
                float(quaternion[1]),
                float(quaternion[2]),
                float(quaternion[3]),
            ),
            frame_id=self.output_frame_id,
            host_time_ns=sample.host_time_ns,
            quality=sample.quality,
            calibration_id=self._calibration_id,
        )
