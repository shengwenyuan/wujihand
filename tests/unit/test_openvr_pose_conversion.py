from __future__ import annotations

import math

import numpy as np
import pytest

from wujihand.adapters.input.openvr_tracker import matrix34_to_pose_m_wxyz
from wujihand.domain.pose import quaternion_wxyz_to_rotation_matrix


def test_identity_matrix_preserves_metre_translation() -> None:
    position, quaternion = matrix34_to_pose_m_wxyz(
        (
            (1.0, 0.0, 0.0, 1.25),
            (0.0, 1.0, 0.0, -0.5),
            (0.0, 0.0, 1.0, 2.0),
        )
    )

    assert position == pytest.approx((1.25, -0.5, 2.0))
    assert quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_matrix_rotation_becomes_active_scalar_first_quaternion() -> None:
    angle = math.pi / 2.0
    rotation = np.asarray(
        (
            (math.cos(angle), -math.sin(angle), 0.0),
            (math.sin(angle), math.cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    matrix = np.column_stack((rotation, np.zeros(3)))

    _, quaternion = matrix34_to_pose_m_wxyz(matrix)

    assert quaternion == pytest.approx(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    assert quaternion_wxyz_to_rotation_matrix(quaternion) == pytest.approx(rotation)


def test_float32_rotation_noise_is_projected_without_changing_convention() -> None:
    matrix = np.asarray(
        (
            (0.0, -1.0000001, 0.0, 0.1),
            (0.9999999, 0.0, 0.0, 0.2),
            (0.0, 0.0, 1.0, 0.3),
        ),
        dtype=np.float32,
    )

    position, quaternion = matrix34_to_pose_m_wxyz(matrix)

    assert position == pytest.approx((0.1, 0.2, 0.3))
    np.testing.assert_allclose(
        quaternion_wxyz_to_rotation_matrix(quaternion),
        np.asarray(
            (
                (0.0, -1.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
            )
        ),
        atol=1e-6,
    )


@pytest.mark.parametrize(
    "matrix, message",
    [
        (np.eye(4), "shape"),
        (
            (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, math.nan, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
            ),
            "NaN",
        ),
        (
            (
                (2.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
            ),
            "orthonormal",
        ),
        (
            (
                (-1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
            ),
            "right-handed",
        ),
    ],
)
def test_malformed_or_non_rigid_matrix_is_rejected(
    matrix: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        matrix34_to_pose_m_wxyz(matrix)  # type: ignore[arg-type]
