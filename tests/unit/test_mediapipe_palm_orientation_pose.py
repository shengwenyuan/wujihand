from __future__ import annotations

import math

import numpy as np
import pytest

from wujihand.adapters.input import MediaPipePalmOrientationEstimator
from wujihand.domain.pose import (
    euler_zyx_to_quaternion_wxyz,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_rotation_matrix,
)


def upright_landmarks() -> np.ndarray:
    landmarks = np.zeros((21, 3), dtype=np.float64)
    landmarks[0] = [0.0, 0.0, 0.0]
    landmarks[5] = [0.0, 0.04, 0.08]
    landmarks[9] = [0.0, 0.0, 0.09]
    landmarks[17] = [0.0, -0.04, 0.07]
    return landmarks


def rotate_landmarks(landmarks: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    rotation = quaternion_wxyz_to_rotation_matrix(quaternion)
    return (rotation @ landmarks.T).T


def test_upright_right_palm_maps_to_identity_hand2_frame() -> None:
    estimator = MediaPipePalmOrientationEstimator()
    sample = estimator.estimate(upright_landmarks(), host_time_ns=12, quality=0.9)
    np.testing.assert_allclose(sample.quat_wxyz, [1.0, 0.0, 0.0, 0.0], atol=1e-12)
    assert sample.frame_id == "mediapipe_right_palm"
    assert sample.host_time_ns == 12
    assert sample.quality == 0.9


def test_known_3d_rotation_is_recovered_from_landmarks_0_5_9_17() -> None:
    expected = euler_zyx_to_quaternion_wxyz(
        yaw=math.radians(35.0),
        pitch=math.radians(-20.0),
        roll=math.radians(15.0),
    )
    estimator = MediaPipePalmOrientationEstimator()
    sample = estimator.estimate(
        rotate_landmarks(upright_landmarks(), expected),
        host_time_ns=100,
    )
    assert quaternion_geodesic_distance_rad(sample.quat_wxyz, expected) < 1e-7


def test_quaternion_sign_is_continuous_across_180_degree_yaw() -> None:
    estimator = MediaPipePalmOrientationEstimator()
    before = euler_zyx_to_quaternion_wxyz(yaw=math.radians(179.0), pitch=0.0, roll=0.0)
    after = euler_zyx_to_quaternion_wxyz(yaw=math.radians(181.0), pitch=0.0, roll=0.0)
    first = estimator.estimate(rotate_landmarks(upright_landmarks(), before), host_time_ns=1)
    second = estimator.estimate(rotate_landmarks(upright_landmarks(), after), host_time_ns=2)
    assert float(np.dot(first.quat_wxyz, second.quat_wxyz)) > 0.99


@pytest.mark.parametrize("shape", ((20, 3), (21, 2), (1, 21, 3)))
def test_wrong_landmark_shape_is_rejected(shape: tuple[int, ...]) -> None:
    estimator = MediaPipePalmOrientationEstimator()
    with pytest.raises(ValueError, match="shape"):
        estimator.estimate(np.zeros(shape), host_time_ns=0)


def test_nonfinite_and_degenerate_palm_frames_are_rejected() -> None:
    estimator = MediaPipePalmOrientationEstimator()
    nonfinite = upright_landmarks()
    nonfinite[9, 0] = np.nan
    with pytest.raises(ValueError, match="NaN"):
        estimator.estimate(nonfinite, host_time_ns=0)

    degenerate = upright_landmarks()
    degenerate[9] = degenerate[0]
    with pytest.raises(ValueError, match="degenerate"):
        estimator.estimate(degenerate, host_time_ns=0)

    collinear = upright_landmarks()
    collinear[5] = [0.0, 0.0, 0.10]
    collinear[17] = [0.0, 0.0, 0.05]
    with pytest.raises(ValueError, match="collinear"):
        estimator.estimate(collinear, host_time_ns=0)
