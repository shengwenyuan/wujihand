from __future__ import annotations

import math

import numpy as np
import pytest

from wujihand.application.calibration import (
    PalmOrientationCalibrator,
    StablePalmOrientationWindow,
)
from wujihand.domain.pose import (
    OrientationSample,
    euler_zyx_to_quaternion_wxyz,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_rotation_matrix,
)


def sample(
    quaternion: np.ndarray,
    time_ns: int,
    *,
    frame_id: str = "mediapipe_right_palm",
    quality: float = 0.9,
) -> OrientationSample:
    return OrientationSample(
        tuple(float(value) for value in quaternion),
        frame_id,
        time_ns,
        quality,
    )


def test_first_capture_and_reclutch_each_emit_positive_identity() -> None:
    calibrator = PalmOrientationCalibrator()
    first_orientation = euler_zyx_to_quaternion_wxyz(yaw=0.4, pitch=-0.2, roll=0.1)
    first = calibrator.capture_neutral(sample(first_orientation, 10))
    assert first.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert first.frame_id == "hand2_right_neutral"
    assert first.calibration_id

    moved = euler_zyx_to_quaternion_wxyz(yaw=-1.0, pitch=0.3, roll=-0.4)
    second = calibrator.clutch(sample(moved, 20))
    assert second.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert second.calibration_id != first.calibration_id
    same_pose = calibrator.apply(sample(moved, 20))
    assert same_pose.quat_wxyz == (1.0, 0.0, 0.0, 0.0)


def test_calibration_is_exactly_r0_transpose_times_r() -> None:
    calibrator = PalmOrientationCalibrator()
    neutral_quaternion = euler_zyx_to_quaternion_wxyz(yaw=0.7, pitch=-0.3, roll=0.2)
    current_quaternion = euler_zyx_to_quaternion_wxyz(yaw=-0.2, pitch=0.5, roll=-0.6)
    calibrator.clutch(sample(neutral_quaternion, 1))
    result = calibrator.apply(sample(current_quaternion, 2))

    neutral_rotation = quaternion_wxyz_to_rotation_matrix(neutral_quaternion)
    current_rotation = quaternion_wxyz_to_rotation_matrix(current_quaternion)
    expected = neutral_rotation.T @ current_rotation
    np.testing.assert_allclose(
        quaternion_wxyz_to_rotation_matrix(result.quat_wxyz),
        expected,
        atol=1e-12,
    )


def test_apply_requires_clutch_matching_frame_and_monotonic_time() -> None:
    calibrator = PalmOrientationCalibrator()
    identity = euler_zyx_to_quaternion_wxyz(yaw=0.0, pitch=0.0, roll=0.0)
    with pytest.raises(RuntimeError, match="clutch"):
        calibrator.apply(sample(identity, 0))
    with pytest.raises(ValueError, match="frame"):
        calibrator.clutch(sample(identity, 0, frame_id="left_palm"))

    calibrator.clutch(sample(identity, 10))
    calibrator.apply(sample(identity, 12))
    with pytest.raises(ValueError, match="monotonic"):
        calibrator.apply(sample(identity, 11))


def test_relative_quaternion_has_no_sign_flip_near_half_turn() -> None:
    calibrator = PalmOrientationCalibrator()
    identity = euler_zyx_to_quaternion_wxyz(yaw=0.0, pitch=0.0, roll=0.0)
    calibrator.clutch(sample(identity, 0))
    before = calibrator.apply(
        sample(euler_zyx_to_quaternion_wxyz(yaw=math.radians(179.0), pitch=0.0, roll=0.0), 1)
    )
    after = calibrator.apply(
        sample(euler_zyx_to_quaternion_wxyz(yaw=math.radians(181.0), pitch=0.0, roll=0.0), 2)
    )
    assert float(np.dot(before.quat_wxyz, after.quat_wxyz)) > 0.99
    assert quaternion_geodesic_distance_rad(before.quat_wxyz, after.quat_wxyz) < math.radians(3.0)


def test_stable_window_requires_fifteen_consecutive_samples_and_averages() -> None:
    window = StablePalmOrientationWindow()
    neutral = euler_zyx_to_quaternion_wxyz(yaw=0.4, pitch=-0.1, roll=0.2)

    result = None
    for index in range(15):
        perturbation = euler_zyx_to_quaternion_wxyz(
            yaw=0.4 + math.radians((index % 3) - 1),
            pitch=-0.1,
            roll=0.2,
        )
        result = window.add(sample(perturbation, index + 1))

    assert result is not None
    assert window.sample_count == 0
    assert result.host_time_ns == 15
    assert quaternion_geodesic_distance_rad(result.quat_wxyz, neutral) < math.radians(0.2)


def test_stable_window_restarts_after_motion_and_rejects_bad_timestamps() -> None:
    window = StablePalmOrientationWindow(required_samples=3, max_spread_rad=math.radians(5.0))
    identity = euler_zyx_to_quaternion_wxyz(yaw=0.0, pitch=0.0, roll=0.0)
    moved = euler_zyx_to_quaternion_wxyz(yaw=math.radians(20.0), pitch=0.0, roll=0.0)

    assert window.add(sample(identity, 1)) is None
    assert window.add(sample(identity, 2)) is None
    assert window.add(sample(moved, 3)) is None
    assert window.sample_count == 1
    with pytest.raises(ValueError, match="increase strictly"):
        window.add(sample(moved, 3))
    assert window.add(sample(moved, 4)) is None
    assert window.add(sample(moved, 5)) is not None


def test_stable_window_requires_quality_and_bounded_inter_sample_gap() -> None:
    window = StablePalmOrientationWindow(
        required_samples=3,
        min_quality=0.5,
        max_sample_gap_s=0.1,
    )
    identity = euler_zyx_to_quaternion_wxyz(yaw=0.0, pitch=0.0, roll=0.0)

    assert window.add(sample(identity, 1)) is None
    assert window.add(sample(identity, 2, quality=0.49)) is None
    assert window.sample_count == 0
    assert window.add(sample(identity, 3)) is None
    assert window.add(sample(identity, 200_000_004)) is None
    assert window.sample_count == 1
    assert window.add(sample(identity, 200_000_005)) is None
    assert window.add(sample(identity, 200_000_006)) is not None
