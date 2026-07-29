from __future__ import annotations

import math

import pytest

from wujihand.application.qualification import compute_tracking_metrics
from wujihand.domain.pose import euler_zyx_to_quaternion_wxyz
from wujihand.domain.tracking import TrackedRigidBodySample, TrackingState


def tracked_sample(
    sequence: int,
    host_time_ns: int,
    *,
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
    quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    stream_id: str = "right_operator",
    device_serial: str = "tracker_test_001",
    tracking_frame: str = "openvr_standing",
) -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id=stream_id,
        device_serial=device_serial,
        logical_role="right_operator",
        producer_instance="openvr_fixture",
        transport_epoch=1,
        tracking_setup_revision="standing_fixture_v1",
        sequence=sequence,
        tracking_frame=tracking_frame,
        position_m=position_m,
        quat_wxyz=quat_wxyz,
        connected=True,
        pose_valid=True,
        tracking_state=TrackingState.RUNNING,
        quality=1.0,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


def lost_sample(
    sequence: int,
    host_time_ns: int,
    *,
    stream_id: str = "right_operator",
    device_serial: str = "tracker_test_001",
    tracking_frame: str = "openvr_standing",
) -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id=stream_id,
        device_serial=device_serial,
        logical_role="right_operator",
        producer_instance="openvr_fixture",
        transport_epoch=1,
        tracking_setup_revision="standing_fixture_v1",
        sequence=sequence,
        tracking_frame=tracking_frame,
        position_m=None,
        quat_wxyz=None,
        connected=True,
        pose_valid=False,
        tracking_state=TrackingState.LOST,
        quality=None,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


def test_empty_stream_has_no_rate_or_stationary_spread() -> None:
    metrics = compute_tracking_metrics(())

    assert metrics.sample_count == 0
    assert metrics.valid_sample_count == 0
    assert metrics.sample_rate_hz is None
    assert metrics.valid_ratio == 0.0
    assert metrics.timestamp_violation_count == 0
    assert metrics.stationary_position_rms_m is None
    assert metrics.stationary_position_peak_m is None
    assert metrics.stationary_orientation_rms_rad is None
    assert metrics.stationary_orientation_peak_rad is None
    assert metrics.position_drift_m is None
    assert metrics.orientation_drift_rad is None
    assert metrics.dropout_count == 0
    assert metrics.dropout_durations_s == ()
    assert metrics.reacquisition_durations_s == ()


def test_constant_valid_stream_reports_rate_ratio_and_zero_spread() -> None:
    metrics = compute_tracking_metrics(
        tracked_sample(index, index * 100_000_000) for index in range(3)
    )

    assert metrics.sample_count == 3
    assert metrics.valid_sample_count == 3
    assert metrics.sample_rate_hz == pytest.approx(10.0)
    assert metrics.valid_ratio == 1.0
    assert metrics.timestamp_violation_count == 0
    assert metrics.stationary_position_rms_m == pytest.approx(0.0)
    assert metrics.stationary_position_peak_m == pytest.approx(0.0)
    assert metrics.stationary_orientation_rms_rad == pytest.approx(0.0)
    assert metrics.stationary_orientation_peak_rad == pytest.approx(0.0)
    assert metrics.position_drift_m == pytest.approx(0.0)
    assert metrics.orientation_drift_rad == pytest.approx(0.0)
    assert metrics.dropout_count == 0


def test_stationary_spread_uses_mean_position_and_so3_mean_orientation() -> None:
    angle_rad = 0.1
    negative = euler_zyx_to_quaternion_wxyz(
        yaw=-angle_rad,
        pitch=0.0,
        roll=0.0,
    )
    negative_yaw = (
        float(negative[0]),
        float(negative[1]),
        float(negative[2]),
        float(negative[3]),
    )
    positive = euler_zyx_to_quaternion_wxyz(
        yaw=angle_rad,
        pitch=0.0,
        roll=0.0,
    )
    positive_yaw = (
        float(positive[0]),
        float(positive[1]),
        float(positive[2]),
        float(positive[3]),
    )
    metrics = compute_tracking_metrics(
        (
            tracked_sample(
                0,
                0,
                position_m=(-1.0, 0.0, 0.0),
                quat_wxyz=negative_yaw,
            ),
            tracked_sample(1, 100_000_000),
            tracked_sample(
                2,
                200_000_000,
                position_m=(1.0, 0.0, 0.0),
                quat_wxyz=positive_yaw,
            ),
        )
    )

    expected_rms_factor = math.sqrt(2.0 / 3.0)
    assert metrics.stationary_position_rms_m == pytest.approx(expected_rms_factor)
    assert metrics.stationary_position_peak_m == pytest.approx(1.0)
    assert metrics.stationary_orientation_rms_rad == pytest.approx(expected_rms_factor * angle_rad)
    assert metrics.stationary_orientation_peak_rad == pytest.approx(angle_rad)


def test_orientation_spread_is_invariant_to_quaternion_sign() -> None:
    metrics = compute_tracking_metrics(
        (
            tracked_sample(0, 0),
            tracked_sample(1, 100_000_000, quat_wxyz=(-1.0, 0.0, 0.0, 0.0)),
        )
    )

    assert metrics.stationary_orientation_rms_rad == pytest.approx(0.0)
    assert metrics.stationary_orientation_peak_rad == pytest.approx(0.0)
    assert metrics.orientation_drift_rad == pytest.approx(0.0)


def test_drift_uses_first_and_last_valid_pose_and_shortest_so3_distance() -> None:
    angle_rad = 0.25
    final_orientation = euler_zyx_to_quaternion_wxyz(
        yaw=angle_rad,
        pitch=0.0,
        roll=0.0,
    )
    sign_flipped_final = (
        -float(final_orientation[0]),
        -float(final_orientation[1]),
        -float(final_orientation[2]),
        -float(final_orientation[3]),
    )
    metrics = compute_tracking_metrics(
        (
            tracked_sample(0, 0),
            lost_sample(1, 100_000_000),
            tracked_sample(
                2,
                200_000_000,
                position_m=(0.3, 0.4, 0.0),
                quat_wxyz=sign_flipped_final,
            ),
        )
    )

    assert metrics.position_drift_m == pytest.approx(0.5)
    assert metrics.orientation_drift_rad == pytest.approx(angle_rad)


def test_rate_excludes_nonincreasing_intervals_and_counts_violations() -> None:
    times_ns = (0, 100_000_000, 100_000_000, 50_000_000, 150_000_000)
    metrics = compute_tracking_metrics(
        tracked_sample(index, timestamp) for index, timestamp in enumerate(times_ns)
    )

    assert metrics.sample_rate_hz == pytest.approx(10.0)
    assert metrics.timestamp_violation_count == 2


def test_dropout_and_reacquisition_runs_are_reported_separately() -> None:
    metrics = compute_tracking_metrics(
        (
            tracked_sample(0, 0),
            lost_sample(1, 100_000_000),
            lost_sample(2, 200_000_000),
            tracked_sample(3, 300_000_000),
            lost_sample(4, 400_000_000),
        )
    )

    assert metrics.sample_count == 5
    assert metrics.valid_sample_count == 2
    assert metrics.valid_ratio == pytest.approx(0.4)
    assert metrics.dropout_count == 2
    assert metrics.dropout_durations_s == pytest.approx((0.1, 0.0))
    assert metrics.reacquisition_durations_s == pytest.approx((0.2,))


def test_all_invalid_open_dropout_has_no_reacquisition_or_spread() -> None:
    metrics = compute_tracking_metrics(
        (
            lost_sample(0, 100_000_000),
            lost_sample(1, 300_000_000),
        )
    )

    assert metrics.valid_ratio == 0.0
    assert metrics.dropout_count == 1
    assert metrics.dropout_durations_s == pytest.approx((0.2,))
    assert metrics.reacquisition_durations_s == ()
    assert metrics.stationary_position_rms_m is None
    assert metrics.stationary_orientation_rms_rad is None
    assert metrics.position_drift_m is None
    assert metrics.orientation_drift_rad is None


def test_one_valid_sample_has_no_drift() -> None:
    metrics = compute_tracking_metrics((tracked_sample(0, 0),))

    assert metrics.position_drift_m is None
    assert metrics.orientation_drift_rad is None


@pytest.mark.parametrize(
    "sample",
    [
        tracked_sample(1, 100_000_000, stream_id="left_operator"),
        tracked_sample(1, 100_000_000, device_serial="tracker_test_002"),
        tracked_sample(1, 100_000_000, tracking_frame="openvr_seated"),
    ],
)
def test_metrics_reject_mixed_stream_identity(
    sample: TrackedRigidBodySample,
) -> None:
    with pytest.raises(ValueError, match="one stream"):
        compute_tracking_metrics((tracked_sample(0, 0), sample))
