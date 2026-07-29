from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.teleoperation import (
    InteractiveTrackerArmController,
    InteractiveTrackerArmState,
    RelativeTrackerPoseMapper,
    RelativeTrackerTranslationMapper,
    TrackerReferenceReadinessGate,
)
from wujihand.domain import TrackedRigidBodySample, TrackingState
from wujihand.domain.pose import (
    euler_zyx_to_quaternion_wxyz,
    multiply_quaternions_wxyz,
    quaternion_wxyz_to_rotation_matrix,
)


def sample(
    *,
    sequence: int,
    host_time_ns: int,
    position_m: tuple[float, float, float] = (1.0, 2.0, 3.0),
    quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    state: TrackingState = TrackingState.RUNNING,
) -> TrackedRigidBodySample:
    valid = state is TrackingState.RUNNING
    return TrackedRigidBodySample(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=position_m if valid else None,
        quat_wxyz=quat_wxyz if valid else None,
        connected=True,
        pose_valid=valid,
        tracking_state=state,
        quality=1.0 if valid else None,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


def mapper() -> RelativeTrackerTranslationMapper:
    return RelativeTrackerTranslationMapper(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
        # Workstation2: body right=-Z, forward=-X, up=+Y.
        tracker_to_world=((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        scale=0.25,
        max_delta_m=0.08,
        stale_after_s=0.25,
    )


def pose_mapper(
    *,
    max_rotation_delta_rad: float = np.deg2rad(15.0),
    translation_enabled: bool = True,
) -> RelativeTrackerPoseMapper:
    return RelativeTrackerPoseMapper(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
        tracker_to_workcell=(
            (0.0, 0.0, -1.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        ),
        translation_scale=0.25,
        max_translation_delta_m=0.08,
        rotation_scale=1.0,
        max_rotation_delta_rad=max_rotation_delta_rad,
        stale_after_s=0.25,
        translation_enabled=translation_enabled,
    )


def reference_gate(
    *,
    stable_after_s: float = 0.25,
) -> TrackerReferenceReadinessGate:
    return TrackerReferenceReadinessGate(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
        stable_after_s=stable_after_s,
        max_sample_gap_s=0.25,
    )


def test_reference_gate_requires_a_continuous_running_window() -> None:
    subject = reference_gate(stable_after_s=0.25)

    first = subject.observe(
        sample(sequence=0, host_time_ns=1_000_000_000)
    )
    almost_ready = subject.observe(
        sample(sequence=1, host_time_ns=1_249_999_999)
    )
    ready = subject.observe(
        sample(sequence=2, host_time_ns=1_250_000_000)
    )

    assert first.reason == "stabilizing_running"
    assert first.consecutive_running_samples == 1
    assert not almost_ready.ready
    assert ready.ready
    assert ready.reason == "stable_running"
    assert ready.consecutive_running_samples == 3
    assert ready.stable_duration_s == pytest.approx(0.25)


def test_reference_gate_resets_on_every_observed_calibrating_sample() -> None:
    subject = reference_gate(stable_after_s=0.20)

    subject.observe(sample(sequence=0, host_time_ns=1_000_000_000))
    subject.observe(sample(sequence=1, host_time_ns=1_150_000_000))
    calibrating = subject.observe(
        sample(
            sequence=2,
            host_time_ns=1_190_000_000,
            state=TrackingState.CALIBRATING,
        )
    )
    restarted = subject.observe(
        sample(sequence=3, host_time_ns=1_300_000_000)
    )
    ready = subject.observe(
        sample(sequence=4, host_time_ns=1_500_000_000)
    )

    assert calibrating.reason == "tracking_calibrating"
    assert calibrating.consecutive_running_samples == 0
    assert not calibrating.ready
    assert restarted.consecutive_running_samples == 1
    assert not restarted.ready
    assert ready.ready
    assert ready.consecutive_running_samples == 2


def test_reference_gate_restarts_after_a_stale_sample_gap() -> None:
    subject = reference_gate(stable_after_s=0.20)

    subject.observe(sample(sequence=0, host_time_ns=1_000_000_000))
    subject.observe(sample(sequence=1, host_time_ns=1_150_000_000))
    restarted = subject.observe(
        sample(sequence=2, host_time_ns=1_500_000_001)
    )
    ready = subject.observe(
        sample(sequence=3, host_time_ns=1_700_000_001)
    )

    assert restarted.reason == "stabilizing_running_after_gap"
    assert restarted.consecutive_running_samples == 1
    assert restarted.stable_duration_s == 0.0
    assert ready.ready
    assert ready.consecutive_running_samples == 2


def test_reference_gate_rejects_invalid_configuration_and_ordering() -> None:
    with pytest.raises(ValueError, match="stable_after_s"):
        reference_gate(stable_after_s=0.0)
    with pytest.raises(ValueError, match="max_sample_gap_s"):
        TrackerReferenceReadinessGate(
            stream_id="vive.right",
            device_serial="LHR-24B6E288",
            logical_role="operator_right",
            tracking_frame="vive_tracking",
            stable_after_s=0.25,
            max_sample_gap_s=0.0,
        )

    subject = reference_gate()
    subject.observe(sample(sequence=2, host_time_ns=200))
    decision = subject.observe(sample(sequence=2, host_time_ns=201))

    assert decision.reason == "non_monotonic_sequence"
    assert not decision.ready


def test_interactive_controller_arms_on_first_fresh_running_sample() -> None:
    subject = InteractiveTrackerArmController(pose_mapper())
    target_position = (0.4, 0.5, 0.6)
    target_orientation = euler_zyx_to_quaternion_wxyz(
        yaw=0.1,
        pitch=-0.2,
        roll=0.3,
    )

    rejected = subject.establish_reference(
        sample(
            sequence=0,
            host_time_ns=1_000_000_000,
            state=TrackingState.CALIBRATING,
        ),
        target_position,
        target_orientation,
        now_ns=1_000_000_001,
    )
    established = subject.establish_reference(
        sample(sequence=1, host_time_ns=1_010_000_000),
        target_position,
        target_orientation,
        now_ns=1_010_000_001,
    )

    assert rejected.state is InteractiveTrackerArmState.WAITING_REFERENCE
    assert rejected.mapping is None
    assert rejected.reference_epoch == 0
    assert established.state is InteractiveTrackerArmState.TRACKING
    assert established.mapping is not None
    assert established.mapping.target_position_m == pytest.approx(
        target_position
    )
    assert established.mapping.target_orientation_wxyz == pytest.approx(
        target_orientation
    )
    assert established.reference_epoch == 1


def test_interactive_controller_holds_then_references_current_robot_pose() -> None:
    subject = InteractiveTrackerArmController(pose_mapper())
    first_target = (0.1, 0.2, 0.3)
    first_orientation = (1.0, 0.0, 0.0, 0.0)
    subject.establish_reference(
        sample(sequence=0, host_time_ns=1_000_000_000),
        first_target,
        first_orientation,
        now_ns=1_000_000_001,
    )

    held = subject.advance(
        sample(
            sequence=1,
            host_time_ns=1_100_000_000,
            state=TrackingState.CALIBRATING,
        ),
        now_ns=1_100_000_001,
    )
    waiting = subject.advance(
        sample(
            sequence=2,
            host_time_ns=1_300_000_000,
            state=TrackingState.CALIBRATING,
        ),
        now_ns=1_300_000_001,
    )

    reacquired_target = (0.31, 0.22, 0.43)
    reacquired_orientation = euler_zyx_to_quaternion_wxyz(
        yaw=-0.1,
        pitch=0.15,
        roll=-0.2,
    )
    reacquired = subject.establish_reference(
        sample(
            sequence=3,
            host_time_ns=1_310_000_000,
            position_m=(4.0, 5.0, 6.0),
        ),
        reacquired_target,
        reacquired_orientation,
        now_ns=1_310_000_001,
    )

    assert held.state is InteractiveTrackerArmState.HOLD
    assert held.mapping is not None
    assert held.mapping.target_position_m == pytest.approx(first_target)
    assert waiting.state is InteractiveTrackerArmState.WAITING_REFERENCE
    assert waiting.mapping is not None
    assert waiting.mapping.requires_reference
    assert reacquired.reference_epoch == 2
    assert reacquired.mapping is not None
    assert reacquired.mapping.target_position_m == pytest.approx(
        reacquired_target
    )
    assert reacquired.mapping.target_orientation_wxyz == pytest.approx(
        reacquired_orientation
    )
    assert reacquired.mapping.tracker_delta_m == pytest.approx((0.0, 0.0, 0.0))


def test_interactive_controller_recovers_from_repeated_ik_failure() -> None:
    subject = InteractiveTrackerArmController(
        pose_mapper(),
        max_consecutive_ik_failures=5,
    )
    subject.establish_reference(
        sample(sequence=0, host_time_ns=100),
        (0.1, 0.2, 0.3),
        (1.0, 0.0, 0.0, 0.0),
        now_ns=101,
    )

    for _ in range(4):
        assert not subject.record_ik_result(False)
    assert subject.record_ik_result(False)

    assert subject.state is InteractiveTrackerArmState.WAITING_REFERENCE
    assert subject.requires_reference
    assert subject.ik_recoveries == 1
    assert subject.reference_epoch == 1


def test_interactive_controller_rejects_invalid_failure_threshold() -> None:
    with pytest.raises(ValueError, match="positive integer"):
        InteractiveTrackerArmController(
            pose_mapper(),
            max_consecutive_ik_failures=0,
        )


def test_translation_compatibility_mapper_freezes_orientation_and_maps_xyz() -> None:
    subject = mapper()
    reference = subject.arm(
        sample(sequence=10, host_time_ns=1_000_000_000),
        (0.4, 0.5, 0.6),
        now_ns=1_010_000_000,
    )
    decision = subject.advance(
        sample(
            sequence=11,
            host_time_ns=1_020_000_000,
            position_m=(1.2, 1.8, 2.8),
        ),
        now_ns=1_030_000_000,
    )

    assert reference.reason == "reference_established"
    assert decision.accepted
    assert not decision.clamped
    assert decision.tracker_delta_m == pytest.approx((0.2, -0.2, -0.2))
    assert decision.world_delta_m == pytest.approx((0.05, -0.05, -0.05), abs=1e-12)
    assert decision.target_position_m == pytest.approx((0.45, 0.45, 0.55))
    assert decision.target_orientation_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_each_world_axis_is_clamped_about_reference() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        (0.0, 0.0, 0.0),
        now_ns=101,
    )
    decision = subject.advance(
        sample(
            sequence=1,
            host_time_ns=102,
            position_m=(11.0, 12.0, -7.0),
        ),
        now_ns=103,
    )

    assert decision.clamped
    assert decision.world_delta_m == pytest.approx((0.08, -0.08, 0.08))
    assert decision.target_position_m == pytest.approx((0.08, -0.08, 0.08))


@pytest.mark.parametrize(
    ("tracker_delta", "expected_workcell_delta"),
    (
        (
            euler_zyx_to_quaternion_wxyz(
                yaw=np.deg2rad(-10.0),
                pitch=0.0,
                roll=0.0,
            ),
            euler_zyx_to_quaternion_wxyz(
                yaw=0.0,
                pitch=0.0,
                roll=np.deg2rad(10.0),
            ),
        ),
        (
            euler_zyx_to_quaternion_wxyz(
                yaw=0.0,
                pitch=0.0,
                roll=np.deg2rad(-10.0),
            ),
            euler_zyx_to_quaternion_wxyz(
                yaw=0.0,
                pitch=np.deg2rad(10.0),
                roll=0.0,
            ),
        ),
        (
            euler_zyx_to_quaternion_wxyz(
                yaw=0.0,
                pitch=np.deg2rad(10.0),
                roll=0.0,
            ),
            euler_zyx_to_quaternion_wxyz(
                yaw=np.deg2rad(10.0),
                pitch=0.0,
                roll=0.0,
            ),
        ),
    ),
    ids=(
        "tracker-minus-z-to-workcell-plus-x",
        "tracker-minus-x-to-workcell-plus-y",
        "tracker-plus-y-to-workcell-plus-z",
    ),
)
def test_physical_tracker_rotation_axes_map_to_workcell_axes(
    tracker_delta: np.ndarray,
    expected_workcell_delta: np.ndarray,
) -> None:
    subject = pose_mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        (0.4, 0.5, 0.6),
        (1.0, 0.0, 0.0, 0.0),
        now_ns=101,
    )
    decision = subject.advance(
        sample(
            sequence=1,
            host_time_ns=102,
            quat_wxyz=tuple(float(value) for value in tracker_delta),
        ),
        now_ns=103,
    )

    assert decision.accepted
    assert not decision.clamped
    assert decision.rotation_delta_rad == pytest.approx(np.deg2rad(10.0))
    assert decision.target_orientation_wxyz is not None
    np.testing.assert_allclose(
        quaternion_wxyz_to_rotation_matrix(decision.target_orientation_wxyz),
        quaternion_wxyz_to_rotation_matrix(expected_workcell_delta),
        atol=1e-12,
    )


def test_rotation_uses_reference_epoch_and_clamps_shortest_delta() -> None:
    subject = pose_mapper(max_rotation_delta_rad=np.deg2rad(15.0))
    tracker_reference = euler_zyx_to_quaternion_wxyz(
        yaw=np.deg2rad(25.0),
        pitch=0.0,
        roll=0.0,
    )
    tracker_delta = euler_zyx_to_quaternion_wxyz(
        yaw=0.0,
        pitch=np.deg2rad(60.0),
        roll=0.0,
    )
    tracker_current = multiply_quaternions_wxyz(
        tracker_delta,
        tracker_reference,
    )
    target_reference = euler_zyx_to_quaternion_wxyz(
        yaw=0.0,
        pitch=0.0,
        roll=np.deg2rad(-20.0),
    )
    subject.arm(
        sample(
            sequence=0,
            host_time_ns=100,
            quat_wxyz=tuple(float(value) for value in tracker_reference),
        ),
        (0.4, 0.5, 0.6),
        target_reference,
        now_ns=101,
    )
    decision = subject.advance(
        sample(
            sequence=1,
            host_time_ns=102,
            quat_wxyz=tuple(float(value) for value in tracker_current),
        ),
        now_ns=103,
    )

    expected_delta = euler_zyx_to_quaternion_wxyz(
        yaw=np.deg2rad(15.0),
        pitch=0.0,
        roll=0.0,
    )
    expected_target = multiply_quaternions_wxyz(
        expected_delta,
        target_reference,
    )
    assert decision.rotation_clamped
    assert decision.clamped
    assert decision.rotation_delta_rad == pytest.approx(np.deg2rad(15.0))
    assert decision.target_orientation_wxyz is not None
    np.testing.assert_allclose(
        quaternion_wxyz_to_rotation_matrix(decision.target_orientation_wxyz),
        quaternion_wxyz_to_rotation_matrix(expected_target),
        atol=1e-12,
    )


def test_rotation_only_mode_freezes_position_despite_tracker_translation() -> None:
    subject = pose_mapper(translation_enabled=False)
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        (0.4, 0.5, 0.6),
        (1.0, 0.0, 0.0, 0.0),
        now_ns=101,
    )
    tracker_rotation = euler_zyx_to_quaternion_wxyz(
        yaw=0.0,
        pitch=np.deg2rad(8.0),
        roll=0.0,
    )
    decision = subject.advance(
        sample(
            sequence=1,
            host_time_ns=102,
            position_m=(3.0, -4.0, 5.0),
            quat_wxyz=tuple(float(value) for value in tracker_rotation),
        ),
        now_ns=103,
    )

    assert decision.tracker_delta_m == pytest.approx((2.0, -6.0, 2.0))
    assert decision.world_delta_m == pytest.approx((0.0, 0.0, 0.0))
    assert decision.target_position_m == pytest.approx((0.4, 0.5, 0.6))
    assert decision.rotation_delta_rad == pytest.approx(np.deg2rad(8.0))


def test_missing_sample_holds_briefly_then_requires_new_reference() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=1_000_000_000),
        (0.1, 0.2, 0.3),
        now_ns=1_000_000_001,
    )

    held = subject.advance(None, now_ns=1_100_000_000)
    stale = subject.advance(None, now_ns=1_300_000_001)

    assert held.reason == "no_new_sample_hold"
    assert held.target_position_m == pytest.approx((0.1, 0.2, 0.3))
    assert stale.reason == "stale_input_reference_required"
    assert stale.target_position_m is None
    assert stale.requires_reference
    assert subject.requires_reference


def test_wrong_identity_sample_disarms_epoch() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        np.zeros(3),
        now_ns=101,
    )
    values: dict[str, object] = {
        "stream_id": "vive.right",
        "device_serial": "LHR-24B6E288",
        "logical_role": "operator_right",
        "sequence": 1,
        "tracking_frame": "vive_tracking",
        "position_m": (1.0, 2.0, 3.0),
        "quat_wxyz": (1.0, 0.0, 0.0, 0.0),
        "connected": True,
        "pose_valid": True,
        "tracking_state": TrackingState.RUNNING,
        "quality": 1.0,
        "host_time_ns": 102,
        "device_time_ns": None,
    }
    values.update(stream_id="vive.left")
    decision = subject.advance(
        TrackedRigidBodySample(**values),  # type: ignore[arg-type]
        now_ns=103,
    )

    assert decision.reason == "identity_or_frame_mismatch_reference_required"
    assert decision.requires_reference
    assert subject.requires_reference


def test_disconnected_tracker_disarms_epoch_immediately() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        np.zeros(3),
        now_ns=101,
    )
    decision = subject.advance(
        TrackedRigidBodySample(
            stream_id="vive.right",
            device_serial="LHR-24B6E288",
            logical_role="operator_right",
            sequence=1,
            tracking_frame="vive_tracking",
            position_m=None,
            quat_wxyz=None,
            connected=False,
            pose_valid=False,
            tracking_state=TrackingState.LOST,
            quality=None,
            host_time_ns=102,
            device_time_ns=None,
        ),
        now_ns=103,
    )

    assert decision.reason == "tracking_lost_reference_required"
    assert decision.requires_reference
    assert subject.requires_reference


def test_connected_invalid_tracking_holds_briefly_then_requires_reference() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=1_000_000_000),
        (0.1, 0.2, 0.3),
        now_ns=1_000_000_001,
    )

    held = subject.advance(
        sample(
            sequence=1,
            host_time_ns=1_100_000_000,
            state=TrackingState.CALIBRATING,
        ),
        now_ns=1_100_000_001,
    )
    stale = subject.advance(
        sample(
            sequence=2,
            host_time_ns=1_300_000_000,
            state=TrackingState.CALIBRATING,
        ),
        now_ns=1_300_000_001,
    )

    assert held.reason == "tracking_calibrating_hold"
    assert held.target_position_m == pytest.approx((0.1, 0.2, 0.3))
    assert held.input_host_time_ns == 1_000_000_000
    assert not held.accepted
    assert not held.requires_reference
    assert stale.reason == "tracking_calibrating_reference_required"
    assert stale.target_position_m is None
    assert stale.requires_reference
    assert subject.requires_reference


def test_running_sample_resumes_same_reference_after_transient_invalid_state() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=1_000_000_000),
        (0.1, 0.2, 0.3),
        now_ns=1_000_000_001,
    )
    subject.advance(
        sample(
            sequence=1,
            host_time_ns=1_100_000_000,
            state=TrackingState.OUT_OF_RANGE,
        ),
        now_ns=1_100_000_001,
    )

    resumed = subject.advance(
        sample(
            sequence=2,
            host_time_ns=1_150_000_000,
            position_m=(1.2, 2.0, 3.0),
        ),
        now_ns=1_150_000_001,
    )

    assert resumed.reason == "tracking"
    assert resumed.accepted
    assert resumed.target_position_m == pytest.approx((0.1, 0.15, 0.3))
    assert not resumed.requires_reference


def test_non_monotonic_sample_disarms_instead_of_reusing_pose() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=3, host_time_ns=100),
        (0.0, 0.0, 0.0),
        now_ns=101,
    )
    decision = subject.advance(
        sample(sequence=3, host_time_ns=102),
        now_ns=103,
    )

    assert decision.reason == "non_monotonic_sequence_reference_required"
    assert decision.requires_reference
