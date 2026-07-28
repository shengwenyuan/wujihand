from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.teleoperation import (
    RelativeTrackerPoseMapper,
    RelativeTrackerTranslationMapper,
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


@pytest.mark.parametrize(
    ("replacement", "reason"),
    (
        ({"tracking_state": TrackingState.LOST}, "tracking_lost_reference_required"),
        ({"stream_id": "vive.left"}, "identity_or_frame_mismatch_reference_required"),
    ),
)
def test_invalid_or_wrong_identity_sample_disarms_epoch(
    replacement: dict[str, object],
    reason: str,
) -> None:
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
    values.update(replacement)
    if values["tracking_state"] is not TrackingState.RUNNING:
        values.update(position_m=None, quat_wxyz=None, pose_valid=False, quality=None)
    decision = subject.advance(
        TrackedRigidBodySample(**values),  # type: ignore[arg-type]
        now_ns=103,
    )

    assert decision.reason == reason
    assert decision.requires_reference
    assert subject.requires_reference


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
