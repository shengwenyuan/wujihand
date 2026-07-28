"""Pure metrics for one tracked-rigid-body sample stream."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import math

import numpy as np
import numpy.typing as npt

from wujihand.domain.tracking import TrackedRigidBodySample


FloatArray = npt.NDArray[np.float64]
_NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class TrackingMetrics:
    """Summary of one acquisition-ordered tracking stream.

    Stationary spread is calculated from valid poses only. Position errors are
    distances from the arithmetic mean position; orientation errors are the
    shortest SO(3) distances from a sign-invariant mean quaternion.

    A dropout is one contiguous run of invalid poses. ``dropout_durations_s``
    contains the observed time from the first to the last invalid sample in
    every run, including a final open run. ``reacquisition_durations_s`` is
    present only for closed runs and spans the first invalid sample through the
    first following valid sample.
    """

    sample_count: int
    valid_sample_count: int
    sample_rate_hz: float | None
    valid_ratio: float
    timestamp_violation_count: int
    stationary_position_rms_m: float | None
    stationary_position_peak_m: float | None
    stationary_orientation_rms_rad: float | None
    stationary_orientation_peak_rad: float | None
    position_drift_m: float | None
    orientation_drift_rad: float | None
    dropout_count: int
    dropout_durations_s: tuple[float, ...]
    reacquisition_durations_s: tuple[float, ...]


def compute_tracking_metrics(
    samples: Iterable[TrackedRigidBodySample],
) -> TrackingMetrics:
    """Compute deterministic metrics without sorting acquisition order.

    Rate uses only strictly increasing adjacent host-time intervals. Equal or
    decreasing timestamps are counted as violations and excluded from rate and
    duration arithmetic.
    """

    ordered = tuple(samples)
    _require_single_stream(ordered)

    valid = tuple(sample for sample in ordered if sample.pose_valid)
    positive_intervals_ns: list[int] = []
    timestamp_violations = 0
    for previous, current in zip(ordered, ordered[1:], strict=False):
        interval_ns = current.host_time_ns - previous.host_time_ns
        if interval_ns <= 0:
            timestamp_violations += 1
        else:
            positive_intervals_ns.append(interval_ns)

    rate_hz = _sample_rate_hz(positive_intervals_ns)
    position_rms, position_peak = _stationary_position_spread(valid)
    orientation_rms, orientation_peak = _stationary_orientation_spread(valid)
    position_drift, orientation_drift = _pose_drift(valid)
    dropout_durations, reacquisition_durations = _dropout_durations(ordered)
    sample_count = len(ordered)

    return TrackingMetrics(
        sample_count=sample_count,
        valid_sample_count=len(valid),
        sample_rate_hz=rate_hz,
        valid_ratio=(len(valid) / sample_count if sample_count else 0.0),
        timestamp_violation_count=timestamp_violations,
        stationary_position_rms_m=position_rms,
        stationary_position_peak_m=position_peak,
        stationary_orientation_rms_rad=orientation_rms,
        stationary_orientation_peak_rad=orientation_peak,
        position_drift_m=position_drift,
        orientation_drift_rad=orientation_drift,
        dropout_count=len(dropout_durations),
        dropout_durations_s=dropout_durations,
        reacquisition_durations_s=reacquisition_durations,
    )


def _require_single_stream(samples: Sequence[TrackedRigidBodySample]) -> None:
    if not samples:
        return
    identity = (
        samples[0].stream_id,
        samples[0].device_serial,
        samples[0].tracking_frame,
    )
    if any(
        (sample.stream_id, sample.device_serial, sample.tracking_frame) != identity
        for sample in samples[1:]
    ):
        raise ValueError("tracking metrics require one stream, device serial, and tracking frame")


def _sample_rate_hz(positive_intervals_ns: Sequence[int]) -> float | None:
    if not positive_intervals_ns:
        return None
    elapsed_ns = sum(positive_intervals_ns)
    return (
        len(positive_intervals_ns) * _NANOSECONDS_PER_SECOND / elapsed_ns
        if elapsed_ns > 0
        else None
    )


def _stationary_position_spread(
    samples: Sequence[TrackedRigidBodySample],
) -> tuple[float | None, float | None]:
    if not samples:
        return None, None
    positions = np.asarray([sample.position_m for sample in samples], dtype=np.float64)
    center = np.mean(positions, axis=0)
    errors = np.linalg.norm(positions - center, axis=1)
    return _rms_and_peak(errors)


def _stationary_orientation_spread(
    samples: Sequence[TrackedRigidBodySample],
) -> tuple[float | None, float | None]:
    if not samples:
        return None, None
    quaternions = np.asarray(
        [sample.quat_wxyz for sample in samples],
        dtype=np.float64,
    )
    mean_quaternion = _mean_quaternion_wxyz(quaternions)
    dots = np.clip(
        np.abs(quaternions @ mean_quaternion),
        0.0,
        1.0,
    )
    errors = 2.0 * np.arccos(dots)
    return _rms_and_peak(errors)


def _mean_quaternion_wxyz(quaternions: FloatArray) -> FloatArray:
    accumulator = quaternions.T @ quaternions
    _, eigenvectors = np.linalg.eigh(accumulator)
    mean_quaternion = eigenvectors[:, -1]
    norm = float(np.linalg.norm(mean_quaternion))
    if not math.isfinite(norm) or norm <= np.finfo(np.float64).eps:
        raise ValueError("cannot compute a stationary mean orientation")
    return mean_quaternion / norm


def _rms_and_peak(errors: FloatArray) -> tuple[float, float]:
    squared = np.square(errors)
    return float(np.sqrt(np.mean(squared))), float(np.max(errors))


def _pose_drift(
    samples: Sequence[TrackedRigidBodySample],
) -> tuple[float | None, float | None]:
    if len(samples) < 2:
        return None, None
    first = samples[0]
    last = samples[-1]
    first_position = np.asarray(first.position_m, dtype=np.float64)
    last_position = np.asarray(last.position_m, dtype=np.float64)
    position_drift_m = float(np.linalg.norm(last_position - first_position))

    first_quaternion = np.asarray(first.quat_wxyz, dtype=np.float64)
    last_quaternion = np.asarray(last.quat_wxyz, dtype=np.float64)
    dot = float(np.clip(abs(float(first_quaternion @ last_quaternion)), 0.0, 1.0))
    orientation_drift_rad = 2.0 * math.acos(dot)
    return position_drift_m, orientation_drift_rad


def _dropout_durations(
    samples: Sequence[TrackedRigidBodySample],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    dropout_durations: list[float] = []
    reacquisition_durations: list[float] = []
    dropout_start_ns: int | None = None
    last_invalid_ns: int | None = None

    for sample in samples:
        if not sample.pose_valid:
            if dropout_start_ns is None:
                dropout_start_ns = sample.host_time_ns
            last_invalid_ns = sample.host_time_ns
            continue

        if dropout_start_ns is None:
            continue
        assert last_invalid_ns is not None
        dropout_durations.append(_nonnegative_duration_s(dropout_start_ns, last_invalid_ns))
        reacquisition_durations.append(
            _nonnegative_duration_s(dropout_start_ns, sample.host_time_ns)
        )
        dropout_start_ns = None
        last_invalid_ns = None

    if dropout_start_ns is not None:
        assert last_invalid_ns is not None
        dropout_durations.append(_nonnegative_duration_s(dropout_start_ns, last_invalid_ns))

    return tuple(dropout_durations), tuple(reacquisition_durations)


def _nonnegative_duration_s(start_ns: int, end_ns: int) -> float:
    return max(0, end_ns - start_ns) / _NANOSECONDS_PER_SECOND
