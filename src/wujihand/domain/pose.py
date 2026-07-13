"""Simulator-independent orientation contracts and quaternion math.

The rotation-only wrist slice deliberately carries no translation.  All
quaternions use scalar-first ``wxyz`` order and represent active rotations.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
from typing import Sequence

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]

_FRAME_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_.:/-]{0,127}$")
_QUATERNION_NORM_ATOL = 1e-6
IDENTITY_QUATERNION_WXYZ: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


def validate_frame_id(frame_id: str) -> str:
    """Validate a bounded, transport-safe coordinate-frame identifier."""

    if not isinstance(frame_id, str) or _FRAME_ID.fullmatch(frame_id) is None:
        raise ValueError("frame_id must be a non-empty coordinate-frame identifier")
    return frame_id


def validate_host_time_ns(host_time_ns: int) -> int:
    """Validate a non-negative host monotonic timestamp."""

    if isinstance(host_time_ns, bool) or not isinstance(host_time_ns, int):
        raise ValueError("host_time_ns must be an integer")
    if host_time_ns < 0:
        raise ValueError("host_time_ns must be non-negative")
    return host_time_ns


def validate_calibration_id(calibration_id: str) -> str:
    """Validate a non-empty opaque calibration token without rewriting it."""

    if not isinstance(calibration_id, str):
        raise ValueError("calibration_id must be a string")
    if not 1 <= len(calibration_id) <= 128 or not calibration_id.strip():
        raise ValueError("calibration_id must contain 1..128 non-blank characters")
    return calibration_id


def _as_quaternion_wxyz(values: Sequence[float] | npt.NDArray[np.floating]) -> FloatArray:
    quaternion = np.asarray(values, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError(f"expected quaternion shape (4,), got {quaternion.shape}")
    if not np.isfinite(quaternion).all():
        raise ValueError("quaternion contains NaN or infinity")
    return quaternion


def validate_unit_quaternion_wxyz(
    values: Sequence[float] | npt.NDArray[np.floating],
) -> FloatArray:
    """Return a copy of a finite unit quaternion or raise ``ValueError``."""

    quaternion = _as_quaternion_wxyz(values)
    norm = float(np.linalg.norm(quaternion))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=_QUATERNION_NORM_ATOL):
        raise ValueError(f"quaternion must have unit norm, got {norm}")
    return quaternion.copy()


def normalized_quaternion_wxyz(
    values: Sequence[float] | npt.NDArray[np.floating],
) -> FloatArray:
    """Normalize a finite non-zero quaternion for internal numerical results."""

    quaternion = _as_quaternion_wxyz(values)
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError("cannot normalize a zero quaternion")
    return quaternion / norm


def validate_rotation_matrix(
    values: Sequence[Sequence[float]] | npt.NDArray[np.floating],
    *,
    atol: float = 1e-6,
) -> FloatArray:
    """Validate an orthonormal, right-handed 3x3 rotation matrix."""

    matrix = np.asarray(values, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"expected rotation matrix shape (3, 3), got {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise ValueError("rotation matrix contains NaN or infinity")
    if atol <= 0.0 or not math.isfinite(atol):
        raise ValueError("atol must be finite and positive")
    if not np.allclose(matrix.T @ matrix, np.eye(3), rtol=0.0, atol=atol):
        raise ValueError("rotation matrix is not orthonormal")
    determinant = float(np.linalg.det(matrix))
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=atol):
        raise ValueError("rotation matrix must be right-handed with determinant +1")
    return matrix.copy()


def quaternion_wxyz_to_rotation_matrix(
    values: Sequence[float] | npt.NDArray[np.floating],
) -> FloatArray:
    """Convert a unit ``wxyz`` quaternion to a rotation matrix."""

    w, x, y, z = validate_unit_quaternion_wxyz(values)
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def rotation_matrix_to_quaternion_wxyz(
    values: Sequence[Sequence[float]] | npt.NDArray[np.floating],
) -> FloatArray:
    """Convert a proper rotation matrix to a scalar-first quaternion."""

    matrix = validate_rotation_matrix(values)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            (
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
        )
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        quaternion = np.asarray(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            )
        )
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        quaternion = np.asarray(
            (
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            )
        )
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        quaternion = np.asarray(
            (
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            )
        )
    return normalized_quaternion_wxyz(quaternion)


def align_quaternion_hemisphere(
    values: Sequence[float] | npt.NDArray[np.floating],
    reference: Sequence[float] | npt.NDArray[np.floating],
) -> FloatArray:
    """Choose the quaternion sign closest to ``reference``."""

    quaternion = validate_unit_quaternion_wxyz(values)
    reference_quaternion = validate_unit_quaternion_wxyz(reference)
    if float(np.dot(quaternion, reference_quaternion)) < 0.0:
        quaternion *= -1.0
    return quaternion


def multiply_quaternions_wxyz(
    first: Sequence[float] | npt.NDArray[np.floating],
    second: Sequence[float] | npt.NDArray[np.floating],
) -> FloatArray:
    """Compose two active rotations as ``R(first) @ R(second)``."""

    aw, ax, ay, az = validate_unit_quaternion_wxyz(first)
    bw, bx, by, bz = validate_unit_quaternion_wxyz(second)
    return normalized_quaternion_wxyz(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def quaternion_geodesic_distance_rad(
    first: Sequence[float] | npt.NDArray[np.floating],
    second: Sequence[float] | npt.NDArray[np.floating],
) -> float:
    """Return the shortest SO(3) distance between two orientations."""

    first_quaternion = validate_unit_quaternion_wxyz(first)
    second_quaternion = validate_unit_quaternion_wxyz(second)
    dot = min(1.0, abs(float(np.dot(first_quaternion, second_quaternion))))
    return 2.0 * math.acos(dot)


def euler_zyx_to_quaternion_wxyz(*, yaw: float, pitch: float, roll: float) -> FloatArray:
    """Build a quaternion for ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""

    if not all(math.isfinite(value) for value in (yaw, pitch, roll)):
        raise ValueError("Euler angles must be finite")
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return normalized_quaternion_wxyz(
        (
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        )
    )


def quaternion_wxyz_to_euler_zyx(
    values: Sequence[float] | npt.NDArray[np.floating],
) -> tuple[float, float, float]:
    """Return ``(yaw, pitch, roll)`` for the ZYX convention."""

    matrix = quaternion_wxyz_to_rotation_matrix(values)
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
    return yaw, pitch, roll


def clamp_pitch_roll_wxyz(
    values: Sequence[float] | npt.NDArray[np.floating],
    *,
    max_pitch_rad: float,
    max_roll_rad: float,
) -> tuple[FloatArray, bool]:
    """Clamp ZYX pitch and roll while preserving the reported yaw."""

    for name, limit in (("max_pitch_rad", max_pitch_rad), ("max_roll_rad", max_roll_rad)):
        if not math.isfinite(limit) or not 0.0 < limit < math.pi / 2.0:
            raise ValueError(f"{name} must be finite and in (0, pi/2)")
    yaw, pitch, roll = quaternion_wxyz_to_euler_zyx(values)
    limited_pitch = float(np.clip(pitch, -max_pitch_rad, max_pitch_rad))
    limited_roll = float(np.clip(roll, -max_roll_rad, max_roll_rad))
    changed = not (
        math.isclose(pitch, limited_pitch, rel_tol=0.0, abs_tol=1e-12)
        and math.isclose(roll, limited_roll, rel_tol=0.0, abs_tol=1e-12)
    )
    return (
        euler_zyx_to_quaternion_wxyz(yaw=yaw, pitch=limited_pitch, roll=limited_roll),
        changed,
    )


@dataclass(frozen=True, slots=True)
class OrientationSample:
    """A measured frame orientation before neutral-pose calibration."""

    quat_wxyz: tuple[float, float, float, float]
    frame_id: str
    host_time_ns: int
    quality: float = 1.0

    def __post_init__(self) -> None:
        quaternion = validate_unit_quaternion_wxyz(self.quat_wxyz)
        object.__setattr__(self, "quat_wxyz", tuple(float(value) for value in quaternion))
        object.__setattr__(self, "frame_id", validate_frame_id(self.frame_id))
        object.__setattr__(self, "host_time_ns", validate_host_time_ns(self.host_time_ns))
        if not math.isfinite(self.quality) or not 0.0 <= self.quality <= 1.0:
            raise ValueError("quality must be finite and in [0, 1]")

    def as_array(self) -> FloatArray:
        return np.asarray(self.quat_wxyz, dtype=np.float64)


@dataclass(frozen=True, slots=True)
class PoseIntent:
    """Calibrated rotation-only root intent for Hand 2 right."""

    quat_wxyz: tuple[float, float, float, float]
    frame_id: str
    host_time_ns: int
    quality: float
    calibration_id: str

    def __post_init__(self) -> None:
        quaternion = validate_unit_quaternion_wxyz(self.quat_wxyz)
        object.__setattr__(self, "quat_wxyz", tuple(float(value) for value in quaternion))
        object.__setattr__(self, "frame_id", validate_frame_id(self.frame_id))
        object.__setattr__(self, "host_time_ns", validate_host_time_ns(self.host_time_ns))
        if not math.isfinite(self.quality) or not 0.0 <= self.quality <= 1.0:
            raise ValueError("quality must be finite and in [0, 1]")
        object.__setattr__(
            self, "calibration_id", validate_calibration_id(self.calibration_id)
        )

    def as_array(self) -> FloatArray:
        return np.asarray(self.quat_wxyz, dtype=np.float64)
