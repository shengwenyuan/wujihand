"""Fail-closed supervision for a fixed-position, rotation-only hand root."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from wujihand.domain.pose import (
    IDENTITY_QUATERNION_WXYZ,
    PoseIntent,
    align_quaternion_hemisphere,
    clamp_pitch_roll_wxyz,
    euler_zyx_to_quaternion_wxyz,
    multiply_quaternions_wxyz,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_euler_zyx,
    validate_frame_id,
    validate_host_time_ns,
)

from .joint_supervisor import SafetyState


@dataclass(frozen=True, slots=True)
class PoseSafetyDecision:
    """A supervised root-orientation command and its safety state."""

    command_quat_wxyz: tuple[float, float, float, float]
    state: SafetyState
    reason: str
    tilt_limited: bool
    rate_limited: bool
    requires_clutch: bool
    calibration_id: str | None


class PoseSupervisor:
    """Limit tilt/rate, hold stale poses, and require clutch after disarm.

    Fresh input is accepted for ``degraded_after_s``.  At that age the last
    command is held in ``DEGRADED`` state; at ``disarm_after_s`` the supervisor
    enters ``DISARMED`` and will not consume another delta until an explicit
    identity clutch with a newer ``calibration_id`` is supplied.

    Re-clutching is continuous: the current command becomes the anchor for the
    new identity-relative calibration epoch.  This also permits accumulated
    yaw beyond a human wrist's single-turn range without translating the root.
    """

    def __init__(
        self,
        *,
        frame_id: str = "hand2_right_neutral",
        degraded_after_s: float = 0.25,
        disarm_after_s: float = 0.50,
        max_angular_speed_rad_s: float = math.radians(180.0),
        max_pitch_rad: float = math.radians(89.0),
        max_roll_rad: float = math.radians(89.0),
        min_quality: float = 0.5,
    ) -> None:
        self.frame_id = validate_frame_id(frame_id)
        if not math.isfinite(degraded_after_s) or degraded_after_s <= 0.0:
            raise ValueError("degraded_after_s must be finite and positive")
        if not math.isfinite(disarm_after_s) or disarm_after_s <= degraded_after_s:
            raise ValueError("disarm_after_s must be greater than degraded_after_s")
        if not math.isfinite(max_angular_speed_rad_s) or max_angular_speed_rad_s <= 0.0:
            raise ValueError("max_angular_speed_rad_s must be finite and positive")
        for name, limit in (("max_pitch_rad", max_pitch_rad), ("max_roll_rad", max_roll_rad)):
            if not math.isfinite(limit) or not 0.0 < limit < math.pi / 2.0:
                raise ValueError(f"{name} must be finite and in (0, pi/2)")
        if not math.isfinite(min_quality) or not 0.0 <= min_quality <= 1.0:
            raise ValueError("min_quality must be finite and in [0, 1]")

        self.degraded_after_ns = int(degraded_after_s * 1e9)
        self.disarm_after_ns = int(disarm_after_s * 1e9)
        self.max_angular_speed_rad_s = max_angular_speed_rad_s
        self.max_pitch_rad = max_pitch_rad
        self.max_roll_rad = max_roll_rad
        self.min_quality = min_quality

        self.state = SafetyState.DISARMED
        self._command = np.asarray(IDENTITY_QUATERNION_WXYZ, dtype=np.float64)
        self._anchor = self._command.copy()
        self._calibration_id: str | None = None
        self._seen_calibration_ids: set[str] = set()
        self._last_input_time_ns: int | None = None
        self._last_step_ns: int | None = None

    @property
    def requires_clutch(self) -> bool:
        return self.state is SafetyState.DISARMED

    @property
    def command_quat_wxyz(self) -> tuple[float, float, float, float]:
        return self._command_tuple()

    def arm_with_clutch(self, intent: PoseIntent, *, now_ns: int) -> PoseSafetyDecision:
        """Arm or re-center using a newer, identity-valued clutch intent."""

        now = validate_host_time_ns(now_ns)
        if not isinstance(intent, PoseIntent):
            raise TypeError("intent must be a PoseIntent")
        if intent.frame_id != self.frame_id:
            raise ValueError(f"expected pose frame {self.frame_id!r}, got {intent.frame_id!r}")
        if intent.quality < self.min_quality:
            raise ValueError("clutch quality is below min_quality")
        if intent.host_time_ns > now:
            raise ValueError("clutch timestamp cannot be in the future")
        if now - intent.host_time_ns >= self.degraded_after_ns:
            raise ValueError("clutch intent must be fresh")
        if intent.calibration_id in self._seen_calibration_ids:
            raise ValueError("clutch must use a previously unseen calibration_id")
        if quaternion_geodesic_distance_rad(
            intent.quat_wxyz, IDENTITY_QUATERNION_WXYZ
        ) > 1e-6:
            raise ValueError("clutch intent must be identity-relative")
        if self._last_step_ns is not None and self.state is not SafetyState.DISARMED:
            if now <= self._last_step_ns:
                raise ValueError("now_ns must increase strictly while armed")

        # Keep the physical command continuous: identity in the new epoch is
        # interpreted relative to the pose already being held.
        self._anchor = self._command.copy()
        self._calibration_id = intent.calibration_id
        self._seen_calibration_ids.add(intent.calibration_id)
        self._last_input_time_ns = intent.host_time_ns
        self._last_step_ns = now
        self.state = SafetyState.TRACKING
        return self._decision("armed_with_clutch", False, False)

    def disarm(self) -> PoseSafetyDecision:
        """Hold the last safe command and require a newer clutch."""

        self.state = SafetyState.DISARMED
        self._last_step_ns = None
        return self._decision("disarmed_hold_requires_clutch", False, False)

    def step(
        self,
        intent: PoseIntent | None,
        *,
        now_ns: int,
    ) -> PoseSafetyDecision:
        """Supervise one orientation intent without ever translating the root."""

        now = validate_host_time_ns(now_ns)
        if self.state is SafetyState.DISARMED:
            return self._decision("disarmed_hold_requires_clutch", False, False)
        if self._last_step_ns is None or now <= self._last_step_ns:
            raise ValueError("now_ns must increase strictly while armed")

        if intent is not None and not isinstance(intent, PoseIntent):
            raise TypeError("intent must be a PoseIntent or None")
        invalid_reason = self._invalid_intent_reason(intent, now)
        if invalid_reason == "calibration_change_requires_arm_with_clutch":
            self.state = SafetyState.DISARMED
            self._last_step_ns = now
            return self._decision(invalid_reason, False, False)

        if invalid_reason is None and intent is not None:
            delta_quaternion = intent.as_array()
            target = multiply_quaternions_wxyz(self._anchor, delta_quaternion)
            target, tilt_limited = clamp_pitch_roll_wxyz(
                target,
                max_pitch_rad=self.max_pitch_rad,
                max_roll_rad=self.max_roll_rad,
            )
            target = align_quaternion_hemisphere(target, self._command)

            dt_s = (now - self._last_step_ns) / 1e9
            max_step_rad = self.max_angular_speed_rad_s * dt_s
            distance_rad = quaternion_geodesic_distance_rad(self._command, target)
            rate_limited = distance_rad > max_step_rad + 1e-12
            if rate_limited:
                current_yaw, current_pitch, current_roll = quaternion_wxyz_to_euler_zyx(
                    self._command
                )
                target_yaw, target_pitch, target_roll = quaternion_wxyz_to_euler_zyx(target)
                yaw_delta = math.atan2(
                    math.sin(target_yaw - current_yaw),
                    math.cos(target_yaw - current_yaw),
                )
                pitch_delta = target_pitch - current_pitch
                roll_delta = target_roll - current_roll
                # A straight ZYX-parameter path stays inside the rectangular
                # pitch/roll envelope.  Bounding its L1 angular path length also
                # bounds the actual SO(3) step by the triangle inequality.
                parameter_path_rad = (
                    abs(yaw_delta) + abs(pitch_delta) + abs(roll_delta)
                )
                fraction = min(1.0, max_step_rad / parameter_path_rad)
                self._command = euler_zyx_to_quaternion_wxyz(
                    yaw=current_yaw + fraction * yaw_delta,
                    pitch=current_pitch + fraction * pitch_delta,
                    roll=current_roll + fraction * roll_delta,
                )
                self._command = align_quaternion_hemisphere(
                    self._command, target
                )
            else:
                self._command = target
            self._last_input_time_ns = intent.host_time_ns
            self._last_step_ns = now
            self.state = SafetyState.TRACKING
            reason = "tracking"
            if tilt_limited and rate_limited:
                reason = "tracking_tilt_and_rate_limited"
            elif tilt_limited:
                reason = "tracking_tilt_limited"
            elif rate_limited:
                reason = "tracking_rate_limited"
            return self._decision(reason, tilt_limited, rate_limited)

        self._last_step_ns = now
        age_ns = self._input_age_ns(now)
        if age_ns >= self.disarm_after_ns:
            self.state = SafetyState.DISARMED
            return self._decision("stale_disarmed_hold_requires_clutch", False, False)
        if invalid_reason is not None:
            self.state = SafetyState.DEGRADED
            return self._decision(invalid_reason, False, False)
        if age_ns >= self.degraded_after_ns:
            self.state = SafetyState.DEGRADED
            return self._decision("stale_degraded_hold", False, False)
        reason = "missing_input_hold"
        if self.state is SafetyState.DEGRADED:
            reason = "degraded_hold_waiting_for_valid_input"
        return self._decision(reason, False, False)

    def _invalid_intent_reason(self, intent: PoseIntent | None, now_ns: int) -> str | None:
        if intent is None:
            return None
        if intent.frame_id != self.frame_id:
            return "invalid_pose_frame_hold"
        if intent.calibration_id != self._calibration_id:
            return "calibration_change_requires_arm_with_clutch"
        if intent.host_time_ns > now_ns:
            return "future_input_timestamp_hold"
        if self._last_input_time_ns is not None and intent.host_time_ns < self._last_input_time_ns:
            return "out_of_order_input_hold"
        if intent.quality < self.min_quality:
            return "low_quality_input_hold"
        if now_ns - intent.host_time_ns >= self.degraded_after_ns:
            return "stale_input_hold"
        return None

    def _input_age_ns(self, now_ns: int) -> int:
        if self._last_input_time_ns is None:
            return self.disarm_after_ns
        return max(0, now_ns - self._last_input_time_ns)

    def _decision(
        self,
        reason: str,
        tilt_limited: bool,
        rate_limited: bool,
    ) -> PoseSafetyDecision:
        return PoseSafetyDecision(
            command_quat_wxyz=self._command_tuple(),
            state=self.state,
            reason=reason,
            tilt_limited=tilt_limited,
            rate_limited=rate_limited,
            requires_clutch=self.requires_clutch,
            calibration_id=self._calibration_id,
        )

    def _command_tuple(self) -> tuple[float, float, float, float]:
        return (
            float(self._command[0]),
            float(self._command[1]),
            float(self._command[2]),
            float(self._command[3]),
        )
