"""Backend-neutral relative translation mapping for one tracked robot arm.

The mapper consumes only the canonical rigid-body contract.  It owns the
reference epoch, coordinate-frame mapping, translation scale, workspace clamp,
and stale-input behavior; OpenVR and Isaac objects stay in their adapters and
composition roots.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, cast

import numpy as np
import numpy.typing as npt

from wujihand.domain import TrackedRigidBodySample, TrackingState
from wujihand.domain.pose import validate_host_time_ns, validate_rotation_matrix


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
FloatArray = npt.NDArray[np.float64]


def _finite_vector3(value: object, *, field: str) -> FloatArray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} must be a finite three-vector") from exc
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite three-vector")
    return result


@dataclass(frozen=True, slots=True)
class TrackerTranslationDecision:
    """One mapping result, including explicit reference-loss semantics."""

    target_position_m: Vector3 | None
    tracker_delta_m: Vector3 | None
    world_delta_m: Vector3 | None
    input_host_time_ns: int | None
    accepted: bool
    clamped: bool
    requires_reference: bool
    reason: str


class RelativeTrackerTranslationMapper:
    """Map one Tracker reference epoch into a bounded world-frame XYZ target."""

    def __init__(
        self,
        *,
        stream_id: str,
        device_serial: str,
        logical_role: str,
        tracking_frame: str,
        tracker_to_world: Sequence[Sequence[float]],
        scale: float,
        max_delta_m: float,
        stale_after_s: float,
        min_quality: float = 0.5,
    ) -> None:
        for field, value in (
            ("stream_id", stream_id),
            ("device_serial", device_serial),
            ("logical_role", logical_role),
            ("tracking_frame", tracking_frame),
        ):
            if not isinstance(value, str) or not value or len(value) > 128:
                raise ValueError(f"{field} must be a bounded non-empty string")
        if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
            raise ValueError("scale must be finite and in (0, 1]")
        if not math.isfinite(max_delta_m) or not 0.0 < max_delta_m <= 0.5:
            raise ValueError("max_delta_m must be finite and in (0, 0.5]")
        if not math.isfinite(stale_after_s) or not 0.0 < stale_after_s <= 5.0:
            raise ValueError("stale_after_s must be finite and in (0, 5]")
        if not math.isfinite(min_quality) or not 0.0 < min_quality <= 1.0:
            raise ValueError("min_quality must be finite and in (0, 1]")

        self.stream_id = stream_id
        self.device_serial = device_serial
        self.logical_role = logical_role
        self.tracking_frame = tracking_frame
        self.tracker_to_world = validate_rotation_matrix(tracker_to_world)
        self.scale = float(scale)
        self.max_delta_m = float(max_delta_m)
        self.stale_after_ns = round(stale_after_s * 1_000_000_000)
        self.min_quality = float(min_quality)
        self._reference_tracker_m: FloatArray | None = None
        self._reference_target_m: FloatArray | None = None
        self._last_target_m: FloatArray | None = None
        self._last_sequence: int | None = None
        self._last_input_time_ns: int | None = None
        self._last_step_ns: int | None = None

    @property
    def requires_reference(self) -> bool:
        return self._reference_tracker_m is None

    def arm(
        self,
        sample: TrackedRigidBodySample,
        reference_target_position_m: object,
        *,
        now_ns: int,
    ) -> TrackerTranslationDecision:
        """Create a new reference epoch from one fresh actionable sample."""

        now = validate_host_time_ns(now_ns)
        reason = self._invalid_sample_reason(sample, now)
        if reason is not None:
            raise ValueError(f"cannot establish Tracker reference: {reason}")
        assert sample.position_m is not None
        target = _finite_vector3(
            reference_target_position_m,
            field="reference_target_position_m",
        )
        self._reference_tracker_m = np.asarray(sample.position_m, dtype=np.float64)
        self._reference_target_m = target.copy()
        self._last_target_m = target.copy()
        self._last_sequence = sample.sequence
        self._last_input_time_ns = sample.host_time_ns
        self._last_step_ns = now
        return self._decision(
            target=target,
            tracker_delta=np.zeros(3, dtype=np.float64),
            world_delta=np.zeros(3, dtype=np.float64),
            input_host_time_ns=sample.host_time_ns,
            accepted=True,
            clamped=False,
            requires_reference=False,
            reason="reference_established",
        )

    def advance(
        self,
        sample: TrackedRigidBodySample | None,
        *,
        now_ns: int,
    ) -> TrackerTranslationDecision:
        """Advance one mapping tick or explicitly require a new reference."""

        now = validate_host_time_ns(now_ns)
        if self._last_step_ns is not None and now <= self._last_step_ns:
            raise ValueError("now_ns must increase strictly")
        self._last_step_ns = now
        if self.requires_reference:
            return self._reference_required("reference_required")

        assert self._last_target_m is not None
        assert self._last_input_time_ns is not None
        if sample is None:
            if now - self._last_input_time_ns > self.stale_after_ns:
                self.disarm()
                return self._reference_required("stale_input_reference_required")
            return self._decision(
                target=self._last_target_m,
                tracker_delta=None,
                world_delta=None,
                input_host_time_ns=self._last_input_time_ns,
                accepted=False,
                clamped=False,
                requires_reference=False,
                reason="no_new_sample_hold",
            )

        reason = self._invalid_sample_reason(sample, now)
        if reason is None and (
            self._last_sequence is None or sample.sequence <= self._last_sequence
        ):
            reason = "non_monotonic_sequence"
        if reason is None and sample.host_time_ns <= self._last_input_time_ns:
            reason = "non_monotonic_timestamp"
        if reason is not None:
            self.disarm()
            return self._reference_required(f"{reason}_reference_required")

        assert sample.position_m is not None
        assert self._reference_tracker_m is not None
        assert self._reference_target_m is not None
        tracker_delta = np.asarray(sample.position_m, dtype=np.float64) - self._reference_tracker_m
        raw_world_delta = self.scale * (self.tracker_to_world @ tracker_delta)
        world_delta = np.clip(
            raw_world_delta,
            -self.max_delta_m,
            self.max_delta_m,
        )
        target = self._reference_target_m + world_delta
        clamped = not np.allclose(
            raw_world_delta,
            world_delta,
            rtol=0.0,
            atol=1e-12,
        )
        self._last_target_m = target.copy()
        self._last_sequence = sample.sequence
        self._last_input_time_ns = sample.host_time_ns
        return self._decision(
            target=target,
            tracker_delta=tracker_delta,
            world_delta=world_delta,
            input_host_time_ns=sample.host_time_ns,
            accepted=True,
            clamped=clamped,
            requires_reference=False,
            reason="tracking_clamped" if clamped else "tracking",
        )

    def disarm(self) -> None:
        """Forget the complete epoch so old poses can never re-arm motion."""

        self._reference_tracker_m = None
        self._reference_target_m = None
        self._last_target_m = None
        self._last_sequence = None
        self._last_input_time_ns = None

    def _invalid_sample_reason(
        self,
        sample: object,
        now_ns: int,
    ) -> str | None:
        if type(sample) is not TrackedRigidBodySample:
            return "non_canonical_sample"
        typed = sample
        if (
            typed.stream_id != self.stream_id
            or typed.device_serial != self.device_serial
            or typed.logical_role != self.logical_role
            or typed.tracking_frame != self.tracking_frame
        ):
            return "identity_or_frame_mismatch"
        if (
            not typed.connected
            or not typed.pose_valid
            or typed.tracking_state is not TrackingState.RUNNING
            or typed.position_m is None
        ):
            return f"tracking_{typed.tracking_state.value}"
        if typed.quality is None or typed.quality < self.min_quality:
            return "quality_below_minimum"
        if typed.host_time_ns > now_ns:
            return "future_timestamp"
        if now_ns - typed.host_time_ns > self.stale_after_ns:
            return "stale_timestamp"
        return None

    def _reference_required(self, reason: str) -> TrackerTranslationDecision:
        return TrackerTranslationDecision(
            target_position_m=None,
            tracker_delta_m=None,
            world_delta_m=None,
            input_host_time_ns=None,
            accepted=False,
            clamped=False,
            requires_reference=True,
            reason=reason,
        )

    @staticmethod
    def _decision(
        *,
        target: FloatArray,
        tracker_delta: FloatArray | None,
        world_delta: FloatArray | None,
        input_host_time_ns: int,
        accepted: bool,
        clamped: bool,
        requires_reference: bool,
        reason: str,
    ) -> TrackerTranslationDecision:
        def vector(value: FloatArray | None) -> Vector3 | None:
            if value is None:
                return None
            return cast(Vector3, tuple(float(item) for item in value))

        return TrackerTranslationDecision(
            target_position_m=vector(target),
            tracker_delta_m=vector(tracker_delta),
            world_delta_m=vector(world_delta),
            input_host_time_ns=input_host_time_ns,
            accepted=accepted,
            clamped=clamped,
            requires_reference=requires_reference,
            reason=reason,
        )


__all__ = [
    "Matrix3",
    "RelativeTrackerTranslationMapper",
    "TrackerTranslationDecision",
    "Vector3",
]
