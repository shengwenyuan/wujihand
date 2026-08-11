"""Deterministic canonical Wuji Glove signals for device-free qualification."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import numpy.typing as npt

from wujihand.domain import (
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
)


WUJI_GLOVE_STUB_POSES = (
    "open",
    "index_opposition",
    "middle_opposition",
    "ring_opposition",
    "pinky_opposition",
)


def _right_open() -> npt.NDArray[np.float32]:
    return np.asarray(
        [
            (0.000, 0.000, 0.000),
            (0.025, 0.018, 0.000),
            (0.042, 0.033, 0.000),
            (0.055, 0.048, 0.000),
            (0.067, 0.062, 0.000),
            (0.029, 0.055, 0.000),
            (0.030, 0.090, 0.000),
            (0.030, 0.117, 0.000),
            (0.030, 0.139, 0.000),
            (0.008, 0.061, 0.000),
            (0.008, 0.099, 0.000),
            (0.008, 0.130, 0.000),
            (0.008, 0.155, 0.000),
            (-0.014, 0.058, 0.000),
            (-0.014, 0.094, 0.000),
            (-0.014, 0.122, 0.000),
            (-0.014, 0.144, 0.000),
            (-0.034, 0.051, 0.000),
            (-0.035, 0.082, 0.000),
            (-0.036, 0.106, 0.000),
            (-0.037, 0.125, 0.000),
        ],
        dtype=np.float32,
    )


def wuji_glove_stub_keypoints(side: HandSide, pose: str) -> npt.NDArray[np.float32]:
    """Return one finite MediaPipe-order skeleton in metres.

    The fixture only verifies signal routing.  It is not a user-anatomy model or
    a numerical acceptance target for physical fingertip contact.
    """

    if type(side) is not HandSide:
        raise ValueError("side must be a HandSide")
    if pose not in WUJI_GLOVE_STUB_POSES:
        raise ValueError(f"unsupported Wuji Glove stub pose: {pose!r}")

    points = _right_open().copy()
    if pose != "open":
        specifications = {
            "index_opposition": (5, (0.043, 0.082, -0.026)),
            "middle_opposition": (9, (0.023, 0.084, -0.030)),
            "ring_opposition": (13, (0.002, 0.081, -0.029)),
            "pinky_opposition": (17, (-0.020, 0.075, -0.025)),
        }
        finger_start, raw_contact = specifications[pose]
        contact = np.asarray(raw_contact, dtype=np.float32)
        thumb_base = points[1].copy()
        points[2] = thumb_base * 0.60 + contact * 0.40 + (0.006, -0.002, 0.006)
        points[3] = thumb_base * 0.30 + contact * 0.70 + (0.004, 0.003, 0.002)
        points[4] = contact
        finger_base = points[finger_start].copy()
        points[finger_start + 1] = (
            finger_base * 0.68 + contact * 0.32 + (0.000, 0.018, 0.002)
        )
        points[finger_start + 2] = (
            finger_base * 0.35 + contact * 0.65 + (0.000, 0.021, -0.003)
        )
        points[finger_start + 3] = contact
    if side is HandSide.LEFT:
        points[:, 0] *= -1.0
    return np.ascontiguousarray(points, dtype=np.float32)


def build_wuji_glove_stub_observations(
    side: HandSide,
    *,
    calibration_id: str,
    poses: Sequence[str] = WUJI_GLOVE_STUB_POSES,
    frames_per_pose: int = 60,
) -> tuple[CanonicalHandObservation, ...]:
    """Build a bounded signal stream suitable for the canonical replay port."""

    if type(frames_per_pose) is not int or frames_per_pose < 1:
        raise ValueError("frames_per_pose must be a positive integer")
    pose_names = tuple(poses)
    if not pose_names:
        raise ValueError("poses must not be empty")

    records: list[CanonicalHandObservation] = []
    sequence = 0
    for pose in pose_names:
        points = wuji_glove_stub_keypoints(side, pose)
        for _ in range(frames_per_pose):
            records.append(
                CanonicalHandObservation(
                    side=side,
                    sequence=sequence,
                    source_id=f"wuji_glove.{side.value}.qualification_stub",
                    calibration_id=calibration_id,
                    transform_id="wuji_glove.hand_skeleton.v1",
                    source_time_ns=None,
                    receive_time_ns=sequence + 1,
                    device_time_ns=sequence + 1,
                    device_clock_domain="wuji_glove_stub_clock",
                    frame_id=f"{side.value[0]}_wrist",
                    landmarks=tuple(
                        HandLandmark(
                            name=name,
                            position_m=(
                                float(points[index, 0]),
                                float(points[index, 1]),
                                float(points[index, 2]),
                            ),
                            confidence=1.0,
                        )
                        for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
                    ),
                )
            )
            sequence += 1
    return tuple(records)


__all__ = [
    "WUJI_GLOVE_STUB_POSES",
    "build_wuji_glove_stub_observations",
    "wuji_glove_stub_keypoints",
]
