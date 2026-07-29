"""Backend-neutral relative pose mapping for one tracked robot arm.

The mapper consumes only the canonical rigid-body contract. It owns the
reference epoch, coordinate-frame mapping, translation/rotation scale, bounded
relative motion, and stale-input behavior; OpenVR and Isaac objects stay in
their adapters and composition roots.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, cast

import numpy as np
import numpy.typing as npt

from wujihand.domain import TrackedRigidBodySample, TrackingState
from wujihand.domain.pose import (
    IDENTITY_QUATERNION_WXYZ,
    quaternion_wxyz_to_rotation_matrix,
    rotation_matrix_to_quaternion_wxyz,
    validate_host_time_ns,
    validate_rotation_matrix,
    validate_unit_quaternion_wxyz,
)


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
QuaternionWxyz = tuple[float, float, float, float]
FloatArray = npt.NDArray[np.float64]


def _finite_vector3(value: object, *, field: str) -> FloatArray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} must be a finite three-vector") from exc
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{field} must be a finite three-vector")
    return result


def _scaled_clamped_rotation(
    rotation: FloatArray,
    *,
    scale: float,
    max_delta_rad: float,
) -> tuple[FloatArray, FloatArray, float, bool]:
    """Scale and clamp one proper rotation along its shortest axis-angle arc."""

    quaternion = rotation_matrix_to_quaternion_wxyz(rotation)
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    sin_half = float(np.linalg.norm(quaternion[1:]))
    if sin_half <= np.finfo(np.float64).eps:
        identity = np.eye(3, dtype=np.float64)
        return (
            identity,
            np.asarray(IDENTITY_QUATERNION_WXYZ, dtype=np.float64),
            0.0,
            False,
        )

    raw_angle = 2.0 * math.atan2(sin_half, float(quaternion[0]))
    scaled_angle = scale * raw_angle
    applied_angle = min(scaled_angle, max_delta_rad)
    axis = quaternion[1:] / sin_half
    applied_quaternion = np.concatenate(
        (
            np.asarray((math.cos(applied_angle / 2.0),), dtype=np.float64),
            axis * math.sin(applied_angle / 2.0),
        )
    )
    applied_rotation = quaternion_wxyz_to_rotation_matrix(applied_quaternion)
    return (
        applied_rotation,
        applied_quaternion,
        applied_angle,
        scaled_angle > max_delta_rad + 1e-12,
    )


@dataclass(frozen=True, slots=True)
class TrackerPoseDecision:
    """One relative SE(3) result with explicit reference-loss semantics."""

    target_position_m: Vector3 | None
    target_orientation_wxyz: QuaternionWxyz | None
    tracker_delta_m: Vector3 | None
    world_delta_m: Vector3 | None
    tracker_delta_rotation_wxyz: QuaternionWxyz | None
    workcell_delta_rotation_wxyz: QuaternionWxyz | None
    rotation_delta_rad: float | None
    input_host_time_ns: int | None
    accepted: bool
    translation_clamped: bool
    rotation_clamped: bool
    requires_reference: bool
    reason: str

    @property
    def clamped(self) -> bool:
        """Compatibility aggregate for consumers that do not split bounds."""

        return self.translation_clamped or self.rotation_clamped

    @property
    def workcell_delta_m(self) -> Vector3 | None:
        """Preferred name for the legacy ``world_delta_m`` field."""

        return self.world_delta_m


# Compatibility name retained for the translation-only public API.
TrackerTranslationDecision = TrackerPoseDecision


@dataclass(frozen=True, slots=True)
class TrackerReferenceReadiness:
    """Result of qualifying a continuous canonical ``RUNNING`` window."""

    ready: bool
    consecutive_running_samples: int
    stable_duration_s: float
    reason: str


class TrackerReferenceReadinessGate:
    """Require stable tracking before a teleoperation reference may be armed.

    The gate consumes canonical observations only. It never rewrites an
    OpenVR state, suppresses transport, or reuses a non-actionable pose.
    Any observed non-``RUNNING`` sample resets the continuous readiness
    window, as does a sample gap beyond the configured freshness bound,
    while the producer remains free to transmit every state.
    """

    def __init__(
        self,
        *,
        stream_id: str,
        device_serial: str,
        logical_role: str,
        tracking_frame: str,
        stable_after_s: float,
        max_sample_gap_s: float,
    ) -> None:
        for field, value in (
            ("stream_id", stream_id),
            ("device_serial", device_serial),
            ("logical_role", logical_role),
            ("tracking_frame", tracking_frame),
        ):
            if not isinstance(value, str) or not value or len(value) > 128:
                raise ValueError(f"{field} must be a bounded non-empty string")
        if (
            not math.isfinite(stable_after_s)
            or not 0.0 < stable_after_s <= 5.0
        ):
            raise ValueError("stable_after_s must be finite and in (0, 5]")
        if (
            not math.isfinite(max_sample_gap_s)
            or not 0.0 < max_sample_gap_s <= 5.0
        ):
            raise ValueError(
                "max_sample_gap_s must be finite and in (0, 5]"
            )

        self.stream_id = stream_id
        self.device_serial = device_serial
        self.logical_role = logical_role
        self.tracking_frame = tracking_frame
        self.stable_after_ns = round(stable_after_s * 1_000_000_000)
        self.max_sample_gap_ns = round(
            max_sample_gap_s * 1_000_000_000
        )
        self._first_running_time_ns: int | None = None
        self._consecutive_running_samples = 0
        self._last_sequence: int | None = None
        self._last_host_time_ns: int | None = None

    def observe(
        self,
        sample: TrackedRigidBodySample,
    ) -> TrackerReferenceReadiness:
        """Observe one transmitted sample without hiding degraded states."""

        if type(sample) is not TrackedRigidBodySample:
            self._reset_running_window()
            return self._decision(reason="non_canonical_sample")
        if (
            sample.stream_id != self.stream_id
            or sample.device_serial != self.device_serial
            or sample.logical_role != self.logical_role
            or sample.tracking_frame != self.tracking_frame
        ):
            self._reset_running_window()
            return self._decision(reason="identity_or_frame_mismatch")
        if (
            self._last_sequence is not None
            and sample.sequence <= self._last_sequence
        ):
            self._reset_running_window()
            return self._decision(reason="non_monotonic_sequence")
        if (
            self._last_host_time_ns is not None
            and sample.host_time_ns <= self._last_host_time_ns
        ):
            self._reset_running_window()
            return self._decision(reason="non_monotonic_timestamp")

        sample_gap_exceeded = (
            self._last_host_time_ns is not None
            and sample.host_time_ns - self._last_host_time_ns
            > self.max_sample_gap_ns
        )
        self._last_sequence = sample.sequence
        self._last_host_time_ns = sample.host_time_ns
        if (
            not sample.connected
            or not sample.pose_valid
            or sample.tracking_state is not TrackingState.RUNNING
        ):
            self._reset_running_window()
            return self._decision(
                reason=f"tracking_{sample.tracking_state.value}"
            )

        if sample_gap_exceeded:
            self._reset_running_window()
        if self._first_running_time_ns is None:
            self._first_running_time_ns = sample.host_time_ns
        self._consecutive_running_samples += 1
        stable_duration_ns = (
            sample.host_time_ns - self._first_running_time_ns
        )
        ready = stable_duration_ns >= self.stable_after_ns
        return TrackerReferenceReadiness(
            ready=ready,
            consecutive_running_samples=self._consecutive_running_samples,
            stable_duration_s=stable_duration_ns / 1_000_000_000,
            reason=(
                "stable_running"
                if ready
                else (
                    "stabilizing_running_after_gap"
                    if sample_gap_exceeded
                    else "stabilizing_running"
                )
            ),
        )

    def reset(self) -> None:
        """Forget both transport ordering and the current running window."""

        self._reset_running_window()
        self._last_sequence = None
        self._last_host_time_ns = None

    def _reset_running_window(self) -> None:
        self._first_running_time_ns = None
        self._consecutive_running_samples = 0

    def _decision(self, *, reason: str) -> TrackerReferenceReadiness:
        return TrackerReferenceReadiness(
            ready=False,
            consecutive_running_samples=self._consecutive_running_samples,
            stable_duration_s=0.0,
            reason=reason,
        )


class RelativeTrackerPoseMapper:
    """Map one Tracker reference epoch into a bounded workcell-frame pose."""

    def __init__(
        self,
        *,
        stream_id: str,
        device_serial: str,
        logical_role: str,
        tracking_frame: str,
        tracker_to_workcell: Sequence[Sequence[float]],
        translation_scale: float,
        max_translation_delta_m: float,
        rotation_scale: float,
        max_rotation_delta_rad: float,
        stale_after_s: float,
        min_quality: float = 0.5,
        translation_enabled: bool = True,
        rotation_enabled: bool = True,
    ) -> None:
        for field, value in (
            ("stream_id", stream_id),
            ("device_serial", device_serial),
            ("logical_role", logical_role),
            ("tracking_frame", tracking_frame),
        ):
            if not isinstance(value, str) or not value or len(value) > 128:
                raise ValueError(f"{field} must be a bounded non-empty string")
        if not math.isfinite(translation_scale) or not 0.0 < translation_scale <= 1.0:
            raise ValueError("translation_scale must be finite and in (0, 1]")
        if not math.isfinite(max_translation_delta_m) or not 0.0 < max_translation_delta_m <= 0.5:
            raise ValueError("max_translation_delta_m must be finite and in (0, 0.5]")
        if not math.isfinite(rotation_scale) or not 0.0 < rotation_scale <= 1.0:
            raise ValueError("rotation_scale must be finite and in (0, 1]")
        if not math.isfinite(max_rotation_delta_rad) or not 0.0 < max_rotation_delta_rad <= math.pi:
            raise ValueError("max_rotation_delta_rad must be finite and in (0, pi]")
        if not math.isfinite(stale_after_s) or not 0.0 < stale_after_s <= 5.0:
            raise ValueError("stale_after_s must be finite and in (0, 5]")
        if not math.isfinite(min_quality) or not 0.0 < min_quality <= 1.0:
            raise ValueError("min_quality must be finite and in (0, 1]")
        if type(rotation_enabled) is not bool:
            raise ValueError("rotation_enabled must be a boolean")
        if type(translation_enabled) is not bool:
            raise ValueError("translation_enabled must be a boolean")

        self.stream_id = stream_id
        self.device_serial = device_serial
        self.logical_role = logical_role
        self.tracking_frame = tracking_frame
        self.tracker_to_workcell = validate_rotation_matrix(tracker_to_workcell)
        self.translation_scale = float(translation_scale)
        self.max_translation_delta_m = float(max_translation_delta_m)
        self.rotation_scale = float(rotation_scale)
        self.max_rotation_delta_rad = float(max_rotation_delta_rad)
        self.stale_after_ns = round(stale_after_s * 1_000_000_000)
        self.min_quality = float(min_quality)
        self.translation_enabled = translation_enabled
        self.rotation_enabled = rotation_enabled
        self._reference_tracker_m: FloatArray | None = None
        self._reference_tracker_rotation: FloatArray | None = None
        self._reference_target_m: FloatArray | None = None
        self._reference_target_rotation: FloatArray | None = None
        self._last_target_m: FloatArray | None = None
        self._last_target_orientation: FloatArray | None = None
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
        reference_target_orientation_wxyz: Sequence[float],
        *,
        now_ns: int,
    ) -> TrackerPoseDecision:
        """Create a new reference epoch from one fresh actionable sample."""

        now = validate_host_time_ns(now_ns)
        reason = self._invalid_sample_reason(sample, now)
        if reason is not None:
            raise ValueError(f"cannot establish Tracker reference: {reason}")
        assert sample.position_m is not None
        assert sample.quat_wxyz is not None
        target = _finite_vector3(
            reference_target_position_m,
            field="reference_target_position_m",
        )
        target_orientation = validate_unit_quaternion_wxyz(reference_target_orientation_wxyz)
        self._reference_tracker_m = np.asarray(
            sample.position_m,
            dtype=np.float64,
        )
        self._reference_tracker_rotation = quaternion_wxyz_to_rotation_matrix(sample.quat_wxyz)
        self._reference_target_m = target.copy()
        self._reference_target_rotation = quaternion_wxyz_to_rotation_matrix(target_orientation)
        self._last_target_m = target.copy()
        self._last_target_orientation = target_orientation.copy()
        self._last_sequence = sample.sequence
        self._last_input_time_ns = sample.host_time_ns
        self._last_step_ns = now
        return self._decision(
            target=target,
            target_orientation=target_orientation,
            tracker_delta=np.zeros(3, dtype=np.float64),
            world_delta=np.zeros(3, dtype=np.float64),
            tracker_delta_rotation=np.asarray(
                IDENTITY_QUATERNION_WXYZ,
                dtype=np.float64,
            ),
            workcell_delta_rotation=np.asarray(
                IDENTITY_QUATERNION_WXYZ,
                dtype=np.float64,
            ),
            rotation_delta_rad=0.0,
            input_host_time_ns=sample.host_time_ns,
            accepted=True,
            translation_clamped=False,
            rotation_clamped=False,
            requires_reference=False,
            reason="reference_established",
        )

    def advance(
        self,
        sample: TrackedRigidBodySample | None,
        *,
        now_ns: int,
    ) -> TrackerPoseDecision:
        """Advance one mapping tick or explicitly require a new reference."""

        now = validate_host_time_ns(now_ns)
        if self._last_step_ns is not None and now <= self._last_step_ns:
            raise ValueError("now_ns must increase strictly")
        self._last_step_ns = now
        if self.requires_reference:
            return self._reference_required("reference_required")

        assert self._last_target_m is not None
        assert self._last_target_orientation is not None
        assert self._last_input_time_ns is not None
        if sample is None:
            return self._hold_last_target_until_stale(
                now_ns=now,
                hold_reason="no_new_sample_hold",
                stale_reason="stale_input_reference_required",
            )

        reason = self._invalid_sample_reason(sample, now)
        if (
            reason is not None
            and reason.startswith("tracking_")
            and type(sample) is TrackedRigidBodySample
            and sample.connected
            and sample.tracking_state is not TrackingState.RUNNING
        ):
            return self._hold_last_target_until_stale(
                now_ns=now,
                hold_reason=f"{reason}_hold",
                stale_reason=f"{reason}_reference_required",
            )
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
        assert sample.quat_wxyz is not None
        assert self._reference_tracker_m is not None
        assert self._reference_tracker_rotation is not None
        assert self._reference_target_m is not None
        assert self._reference_target_rotation is not None
        tracker_delta = np.asarray(sample.position_m, dtype=np.float64) - self._reference_tracker_m
        if self.translation_enabled:
            raw_world_delta = self.translation_scale * (self.tracker_to_workcell @ tracker_delta)
            world_delta = np.clip(
                raw_world_delta,
                -self.max_translation_delta_m,
                self.max_translation_delta_m,
            )
        else:
            raw_world_delta = np.zeros(3, dtype=np.float64)
            world_delta = raw_world_delta.copy()
        target = self._reference_target_m + world_delta
        translation_clamped = not np.allclose(
            raw_world_delta,
            world_delta,
            rtol=0.0,
            atol=1e-12,
        )

        tracker_rotation = quaternion_wxyz_to_rotation_matrix(sample.quat_wxyz)
        tracker_delta_rotation_matrix = tracker_rotation @ self._reference_tracker_rotation.T
        tracker_delta_rotation = rotation_matrix_to_quaternion_wxyz(tracker_delta_rotation_matrix)
        if self.rotation_enabled:
            raw_workcell_delta_rotation = (
                self.tracker_to_workcell
                @ tracker_delta_rotation_matrix
                @ self.tracker_to_workcell.T
            )
            (
                workcell_delta_rotation_matrix,
                workcell_delta_rotation,
                rotation_delta_rad,
                rotation_clamped,
            ) = _scaled_clamped_rotation(
                raw_workcell_delta_rotation,
                scale=self.rotation_scale,
                max_delta_rad=self.max_rotation_delta_rad,
            )
            target_rotation = workcell_delta_rotation_matrix @ self._reference_target_rotation
            target_orientation = rotation_matrix_to_quaternion_wxyz(target_rotation)
        else:
            workcell_delta_rotation = np.asarray(
                IDENTITY_QUATERNION_WXYZ,
                dtype=np.float64,
            )
            rotation_delta_rad = 0.0
            rotation_clamped = False
            target_orientation = rotation_matrix_to_quaternion_wxyz(self._reference_target_rotation)

        self._last_target_m = target.copy()
        self._last_target_orientation = target_orientation.copy()
        self._last_sequence = sample.sequence
        self._last_input_time_ns = sample.host_time_ns
        clamped = translation_clamped or rotation_clamped
        return self._decision(
            target=target,
            target_orientation=target_orientation,
            tracker_delta=tracker_delta,
            world_delta=world_delta,
            tracker_delta_rotation=tracker_delta_rotation,
            workcell_delta_rotation=workcell_delta_rotation,
            rotation_delta_rad=rotation_delta_rad,
            input_host_time_ns=sample.host_time_ns,
            accepted=True,
            translation_clamped=translation_clamped,
            rotation_clamped=rotation_clamped,
            requires_reference=False,
            reason="tracking_clamped" if clamped else "tracking",
        )

    def disarm(self) -> None:
        """Forget the complete epoch so old poses can never re-arm motion."""

        self._reference_tracker_m = None
        self._reference_tracker_rotation = None
        self._reference_target_m = None
        self._reference_target_rotation = None
        self._last_target_m = None
        self._last_target_orientation = None
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
            or typed.quat_wxyz is None
        ):
            return f"tracking_{typed.tracking_state.value}"
        if typed.quality is None or typed.quality < self.min_quality:
            return "quality_below_minimum"
        if typed.host_time_ns > now_ns:
            return "future_timestamp"
        if now_ns - typed.host_time_ns > self.stale_after_ns:
            return "stale_timestamp"
        return None

    def _reference_required(self, reason: str) -> TrackerPoseDecision:
        return TrackerPoseDecision(
            target_position_m=None,
            target_orientation_wxyz=None,
            tracker_delta_m=None,
            world_delta_m=None,
            tracker_delta_rotation_wxyz=None,
            workcell_delta_rotation_wxyz=None,
            rotation_delta_rad=None,
            input_host_time_ns=None,
            accepted=False,
            translation_clamped=False,
            rotation_clamped=False,
            requires_reference=True,
            reason=reason,
        )

    def _hold_last_target_until_stale(
        self,
        *,
        now_ns: int,
        hold_reason: str,
        stale_reason: str,
    ) -> TrackerPoseDecision:
        """Hold a non-actionable observation briefly without refreshing it."""

        assert self._last_target_m is not None
        assert self._last_target_orientation is not None
        assert self._last_input_time_ns is not None
        if now_ns - self._last_input_time_ns > self.stale_after_ns:
            self.disarm()
            return self._reference_required(stale_reason)
        return self._decision(
            target=self._last_target_m,
            target_orientation=self._last_target_orientation,
            tracker_delta=None,
            world_delta=None,
            tracker_delta_rotation=None,
            workcell_delta_rotation=None,
            rotation_delta_rad=None,
            input_host_time_ns=self._last_input_time_ns,
            accepted=False,
            translation_clamped=False,
            rotation_clamped=False,
            requires_reference=False,
            reason=hold_reason,
        )

    @staticmethod
    def _decision(
        *,
        target: FloatArray,
        target_orientation: FloatArray,
        tracker_delta: FloatArray | None,
        world_delta: FloatArray | None,
        tracker_delta_rotation: FloatArray | None,
        workcell_delta_rotation: FloatArray | None,
        rotation_delta_rad: float | None,
        input_host_time_ns: int,
        accepted: bool,
        translation_clamped: bool,
        rotation_clamped: bool,
        requires_reference: bool,
        reason: str,
    ) -> TrackerPoseDecision:
        def vector(value: FloatArray | None) -> Vector3 | None:
            if value is None:
                return None
            return cast(Vector3, tuple(float(item) for item in value))

        def quaternion(value: FloatArray | None) -> QuaternionWxyz | None:
            if value is None:
                return None
            return cast(
                QuaternionWxyz,
                tuple(float(item) for item in value),
            )

        return TrackerPoseDecision(
            target_position_m=vector(target),
            target_orientation_wxyz=quaternion(target_orientation),
            tracker_delta_m=vector(tracker_delta),
            world_delta_m=vector(world_delta),
            tracker_delta_rotation_wxyz=quaternion(tracker_delta_rotation),
            workcell_delta_rotation_wxyz=quaternion(workcell_delta_rotation),
            rotation_delta_rad=rotation_delta_rad,
            input_host_time_ns=input_host_time_ns,
            accepted=accepted,
            translation_clamped=translation_clamped,
            rotation_clamped=rotation_clamped,
            requires_reference=requires_reference,
            reason=reason,
        )


class RelativeTrackerTranslationMapper(RelativeTrackerPoseMapper):
    """Compatibility wrapper that freezes target orientation at reference."""

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
        super().__init__(
            stream_id=stream_id,
            device_serial=device_serial,
            logical_role=logical_role,
            tracking_frame=tracking_frame,
            tracker_to_workcell=tracker_to_world,
            translation_scale=scale,
            max_translation_delta_m=max_delta_m,
            rotation_scale=1.0,
            max_rotation_delta_rad=math.pi,
            stale_after_s=stale_after_s,
            min_quality=min_quality,
            rotation_enabled=False,
        )

    def arm(
        self,
        sample: TrackedRigidBodySample,
        reference_target_position_m: object,
        reference_target_orientation_wxyz: Sequence[float] = IDENTITY_QUATERNION_WXYZ,
        *,
        now_ns: int,
    ) -> TrackerPoseDecision:
        return super().arm(
            sample,
            reference_target_position_m,
            reference_target_orientation_wxyz,
            now_ns=now_ns,
        )


__all__ = [
    "Matrix3",
    "QuaternionWxyz",
    "RelativeTrackerPoseMapper",
    "RelativeTrackerTranslationMapper",
    "TrackerPoseDecision",
    "TrackerReferenceReadiness",
    "TrackerReferenceReadinessGate",
    "TrackerTranslationDecision",
    "Vector3",
]
