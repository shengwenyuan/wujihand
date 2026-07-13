from __future__ import annotations

import math

import numpy as np
import pytest

from wujihand.domain.pose import (
    OrientationSample,
    PoseIntent,
    align_quaternion_hemisphere,
    clamp_pitch_roll_wxyz,
    euler_zyx_to_quaternion_wxyz,
    quaternion_geodesic_distance_rad,
    quaternion_wxyz_to_euler_zyx,
    quaternion_wxyz_to_rotation_matrix,
    rotation_matrix_to_quaternion_wxyz,
)


def test_rotation_matrix_quaternion_round_trip_at_half_turn() -> None:
    quaternion = euler_zyx_to_quaternion_wxyz(yaw=math.pi, pitch=0.0, roll=0.0)
    matrix = quaternion_wxyz_to_rotation_matrix(quaternion)
    recovered = rotation_matrix_to_quaternion_wxyz(matrix)
    assert quaternion_geodesic_distance_rad(quaternion, recovered) < 1e-12


def test_hemisphere_alignment_uses_equivalent_short_arc_sign() -> None:
    start = euler_zyx_to_quaternion_wxyz(yaw=math.radians(170.0), pitch=0.0, roll=0.0)
    equivalent_negative = -start
    aligned = align_quaternion_hemisphere(equivalent_negative, start)
    np.testing.assert_allclose(aligned, start, atol=1e-12)

def test_pitch_roll_clamp_preserves_yaw() -> None:
    raw = euler_zyx_to_quaternion_wxyz(
        yaw=math.radians(42.0),
        pitch=math.radians(70.0),
        roll=math.radians(-60.0),
    )
    limited, changed = clamp_pitch_roll_wxyz(
        raw,
        max_pitch_rad=math.radians(30.0),
        max_roll_rad=math.radians(20.0),
    )
    yaw, pitch, roll = quaternion_wxyz_to_euler_zyx(limited)
    assert changed
    assert math.isclose(yaw, math.radians(42.0), abs_tol=1e-12)
    assert math.isclose(pitch, math.radians(30.0), abs_tol=1e-12)
    assert math.isclose(roll, math.radians(-20.0), abs_tol=1e-12)


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("quat_wxyz", (1.0, 0.0, 0.0)),
        ("quat_wxyz", (2.0, 0.0, 0.0, 0.0)),
        ("quat_wxyz", (np.nan, 0.0, 0.0, 0.0)),
        ("frame_id", "bad frame"),
        ("host_time_ns", -1),
        ("quality", 1.01),
        ("calibration_id", "   "),
        ("calibration_id", "x" * 129),
    ),
)
def test_pose_intent_strictly_rejects_invalid_fields(field: str, value: object) -> None:
    fields: dict[str, object] = {
        "quat_wxyz": (1.0, 0.0, 0.0, 0.0),
        "frame_id": "hand2_right_neutral",
        "host_time_ns": 1,
        "quality": 1.0,
        "calibration_id": "cal-1",
    }
    fields[field] = value
    with pytest.raises(ValueError):
        PoseIntent(**fields)  # type: ignore[arg-type]


def test_orientation_sample_copies_numeric_contract_to_tuple() -> None:
    quaternion = np.asarray([1.0, 0.0, 0.0, 0.0])
    sample = OrientationSample(tuple(quaternion), "mediapipe_right_palm", 4, 0.75)
    quaternion[0] = 0.0
    assert sample.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert sample.quality == 0.75
