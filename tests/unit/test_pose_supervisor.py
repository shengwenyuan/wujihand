from __future__ import annotations

import math

import numpy as np
import pytest

from wujihand.application.supervision import PoseSupervisor, SafetyState
from wujihand.domain.pose import (
    PoseIntent,
    euler_zyx_to_quaternion_wxyz,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_euler_zyx,
)


def intent(
    quaternion: np.ndarray | tuple[float, float, float, float],
    host_time_ns: int,
    *,
    calibration_id: str = "cal-1",
    frame_id: str = "hand2_right_neutral",
    quality: float = 1.0,
) -> PoseIntent:
    return PoseIntent(
        tuple(float(value) for value in quaternion),
        frame_id,
        host_time_ns,
        quality,
        calibration_id,
    )


def identity_intent(host_time_ns: int, *, calibration_id: str = "cal-1") -> PoseIntent:
    return intent((1.0, 0.0, 0.0, 0.0), host_time_ns, calibration_id=calibration_id)


def test_disarmed_holds_identity_until_explicit_fresh_clutch() -> None:
    supervisor = PoseSupervisor()
    yaw = euler_zyx_to_quaternion_wxyz(yaw=0.5, pitch=0.0, roll=0.0)
    decision = supervisor.step(intent(yaw, 0), now_ns=1)
    assert decision.state is SafetyState.DISARMED
    assert decision.requires_clutch
    assert decision.command_quat_wxyz == (1.0, 0.0, 0.0, 0.0)

    with pytest.raises(ValueError, match="identity"):
        supervisor.arm_with_clutch(intent(yaw, 2), now_ns=2)
    armed = supervisor.arm_with_clutch(identity_intent(3), now_ns=3)
    assert armed.state is SafetyState.TRACKING
    assert not armed.requires_clutch


def test_geodesic_rate_limit_uses_shortest_quaternion_arc() -> None:
    supervisor = PoseSupervisor(max_angular_speed_rad_s=math.radians(90.0))
    supervisor.arm_with_clutch(identity_intent(0), now_ns=0)
    target = euler_zyx_to_quaternion_wxyz(yaw=math.radians(90.0), pitch=0.0, roll=0.0)
    decision = supervisor.step(intent(-target, 100_000_000), now_ns=100_000_000)
    assert decision.rate_limited
    assert math.isclose(
        quaternion_geodesic_distance_rad((1.0, 0.0, 0.0, 0.0), decision.command_quat_wxyz),
        math.radians(9.0),
        abs_tol=1e-10,
    )
    assert math.isclose(np.linalg.norm(decision.command_quat_wxyz), 1.0, abs_tol=1e-12)


def test_pitch_and_roll_are_limited_below_gimbal_singularity() -> None:
    supervisor = PoseSupervisor(
        max_angular_speed_rad_s=1e6,
        max_pitch_rad=math.radians(40.0),
        max_roll_rad=math.radians(30.0),
    )
    supervisor.arm_with_clutch(identity_intent(0), now_ns=0)
    target = euler_zyx_to_quaternion_wxyz(
        yaw=math.radians(20.0),
        pitch=math.radians(70.0),
        roll=math.radians(-60.0),
    )
    decision = supervisor.step(intent(target, 10_000_000), now_ns=10_000_000)
    yaw, pitch, roll = quaternion_wxyz_to_euler_zyx(decision.command_quat_wxyz)
    assert decision.tilt_limited
    assert math.isclose(yaw, math.radians(20.0), abs_tol=1e-10)
    assert math.isclose(pitch, math.radians(40.0), abs_tol=1e-10)
    assert math.isclose(roll, math.radians(-30.0), abs_tol=1e-10)


def test_rate_limited_path_never_leaves_pitch_roll_envelope() -> None:
    supervisor = PoseSupervisor(
        max_angular_speed_rad_s=math.radians(45.0),
        max_pitch_rad=math.radians(89.0),
        max_roll_rad=math.radians(89.0),
    )
    supervisor.arm_with_clutch(identity_intent(0), now_ns=0)
    start = euler_zyx_to_quaternion_wxyz(
        yaw=math.radians(-141.31),
        pitch=math.radians(-19.19),
        roll=math.radians(67.01),
    )
    supervisor.step(intent(start, 10_000_000_000), now_ns=10_000_000_000)
    target = euler_zyx_to_quaternion_wxyz(
        yaw=math.radians(150.35),
        pitch=math.radians(-47.44),
        roll=math.radians(-74.99),
    )

    previous = np.asarray(supervisor.command_quat_wxyz)
    for index in range(1, 21):
        now_ns = (index + 10) * 1_000_000_000
        decision = supervisor.step(
            intent(target, now_ns),
            now_ns=now_ns,
        )
        _, pitch, roll = quaternion_wxyz_to_euler_zyx(decision.command_quat_wxyz)
        step = quaternion_geodesic_distance_rad(previous, decision.command_quat_wxyz)
        assert abs(pitch) <= math.radians(89.0) + 1e-12
        assert abs(roll) <= math.radians(89.0) + 1e-12
        assert step <= math.radians(45.0) + 1e-12
        previous = np.asarray(decision.command_quat_wxyz)


def test_250ms_degraded_hold_and_500ms_disarm_are_exact() -> None:
    supervisor = PoseSupervisor()
    supervisor.arm_with_clutch(identity_intent(0), now_ns=0)
    fresh_hold = supervisor.step(None, now_ns=249_999_999)
    assert fresh_hold.state is SafetyState.TRACKING

    degraded = supervisor.step(None, now_ns=250_000_000)
    assert degraded.state is SafetyState.DEGRADED
    assert degraded.reason == "stale_degraded_hold"
    assert not degraded.requires_clutch

    disarmed = supervisor.step(None, now_ns=500_000_000)
    assert disarmed.state is SafetyState.DISARMED
    assert disarmed.reason == "stale_disarmed_hold_requires_clutch"
    assert disarmed.requires_clutch


def test_rearm_requires_new_clutch_and_keeps_accumulated_pose_continuous() -> None:
    supervisor = PoseSupervisor(max_angular_speed_rad_s=1e6)
    supervisor.arm_with_clutch(identity_intent(0), now_ns=0)
    yaw_30 = euler_zyx_to_quaternion_wxyz(yaw=math.radians(30.0), pitch=0.0, roll=0.0)
    moved = supervisor.step(intent(yaw_30, 10_000_000), now_ns=10_000_000)
    supervisor.disarm()
    held = moved.command_quat_wxyz

    with pytest.raises(ValueError, match="unseen"):
        supervisor.arm_with_clutch(identity_intent(20_000_000), now_ns=20_000_000)
    rearmed = supervisor.arm_with_clutch(
        identity_intent(20_000_000, calibration_id="cal-2"),
        now_ns=20_000_000,
    )
    assert quaternion_geodesic_distance_rad(rearmed.command_quat_wxyz, held) < 1e-12

    yaw_20 = euler_zyx_to_quaternion_wxyz(yaw=math.radians(20.0), pitch=0.0, roll=0.0)
    accumulated = supervisor.step(
        intent(yaw_20, 30_000_000, calibration_id="cal-2"),
        now_ns=30_000_000,
    )
    yaw, _, _ = quaternion_wxyz_to_euler_zyx(accumulated.command_quat_wxyz)
    assert math.isclose(yaw, math.radians(50.0), abs_tol=1e-10)


def test_new_calibration_cannot_enter_through_normal_step() -> None:
    supervisor = PoseSupervisor()
    supervisor.arm_with_clutch(identity_intent(0), now_ns=0)
    decision = supervisor.step(
        identity_intent(1, calibration_id="cal-2"),
        now_ns=1,
    )
    assert decision.state is SafetyState.DISARMED
    assert decision.reason == "calibration_change_requires_arm_with_clutch"


def test_low_quality_wrong_frame_and_out_of_order_inputs_fail_closed() -> None:
    supervisor = PoseSupervisor(min_quality=0.8)
    supervisor.arm_with_clutch(identity_intent(100), now_ns=100)

    low_quality = supervisor.step(
        intent((1.0, 0.0, 0.0, 0.0), 101, quality=0.2),
        now_ns=101,
    )
    assert low_quality.state is SafetyState.DEGRADED
    assert low_quality.reason == "low_quality_input_hold"

    wrong_frame = supervisor.step(
        intent((1.0, 0.0, 0.0, 0.0), 102, frame_id="other_frame"),
        now_ns=102,
    )
    assert wrong_frame.reason == "invalid_pose_frame_hold"

    out_of_order = supervisor.step(identity_intent(99), now_ns=103)
    assert out_of_order.reason == "out_of_order_input_hold"
    assert out_of_order.command_quat_wxyz == (1.0, 0.0, 0.0, 0.0)
