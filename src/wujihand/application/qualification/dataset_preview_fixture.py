"""Deterministic A-B-A canonical inputs for the ROS2-Isaac GUI qualification."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Final, Literal

from wujihand.domain import HandSide, MEDIAPIPE_HAND_LANDMARK_NAMES


FIXTURE_PROFILE_ID: Final = "dataset_preview_e2e_aba_v1"
FIXTURE_PRODUCER: Final = "nv5-dataset-preview-fixture"
FIXTURE_RATE_HZ: Final = 120
REFERENCE_FRAMES: Final = 600
MOTION_FRAMES: Final = 600
RETURN_FRAMES: Final = 600
REQUIRED_FRAMES: Final = REFERENCE_FRAMES + MOTION_FRAMES + RETURN_FRAMES

FixturePhase = Literal["a_reference", "b_motion", "a_return"]


@dataclass(frozen=True, slots=True)
class FixtureInputState:
    phase: FixturePhase
    tracker_position_m: tuple[float, float, float]
    tracker_quat_wxyz: tuple[float, float, float, float]
    hand_landmarks_m: tuple[tuple[float, float, float], ...]


_OPEN_RIGHT = (
    (0.000, 0.000, 0.000),
    (-0.020, 0.015, 0.000),
    (-0.035, 0.030, 0.000),
    (-0.048, 0.045, 0.000),
    (-0.060, 0.060, 0.000),
    (-0.022, 0.042, 0.000),
    (-0.022, 0.076, 0.000),
    (-0.022, 0.103, 0.000),
    (-0.022, 0.130, 0.000),
    (0.000, 0.046, 0.000),
    (0.000, 0.084, 0.000),
    (0.000, 0.115, 0.000),
    (0.000, 0.146, 0.000),
    (0.020, 0.042, 0.000),
    (0.020, 0.077, 0.000),
    (0.020, 0.104, 0.000),
    (0.020, 0.131, 0.000),
    (0.038, 0.034, 0.000),
    (0.043, 0.064, 0.000),
    (0.047, 0.087, 0.000),
    (0.051, 0.110, 0.000),
)

_CURLED_RIGHT = (
    (0.000, 0.000, 0.000),
    (-0.020, 0.015, 0.000),
    (-0.030, 0.030, 0.008),
    (-0.027, 0.040, 0.026),
    (-0.014, 0.033, 0.043),
    (-0.022, 0.042, 0.000),
    (-0.022, 0.066, 0.020),
    (-0.021, 0.061, 0.043),
    (-0.018, 0.042, 0.052),
    (0.000, 0.046, 0.000),
    (0.000, 0.070, 0.022),
    (0.001, 0.063, 0.047),
    (0.002, 0.042, 0.056),
    (0.020, 0.042, 0.000),
    (0.020, 0.065, 0.020),
    (0.020, 0.059, 0.043),
    (0.019, 0.040, 0.052),
    (0.038, 0.034, 0.000),
    (0.040, 0.055, 0.018),
    (0.039, 0.051, 0.038),
    (0.035, 0.036, 0.047),
)


def _mirror_for_side(
    values: tuple[tuple[float, float, float], ...],
    side: HandSide,
) -> tuple[tuple[float, float, float], ...]:
    if side is HandSide.RIGHT:
        return values
    return tuple((-x, y, z) for x, y, z in values)


def phase_for_sequence(sequence: int) -> FixturePhase:
    if type(sequence) is not int or sequence < 0:
        raise ValueError("fixture sequence must be a non-negative integer")
    if sequence < REFERENCE_FRAMES:
        return "a_reference"
    if sequence < REFERENCE_FRAMES + MOTION_FRAMES:
        return "b_motion"
    return "a_return"


def input_state(side: HandSide, sequence: int) -> FixtureInputState:
    if type(side) is not HandSide:
        raise ValueError("fixture side must be a HandSide")
    phase = phase_for_sequence(sequence)
    if phase == "b_motion":
        half = math.radians(8.0) / 2.0
        tracker_position = (0.0, 0.080, 1.0)
        tracker_quaternion = (math.cos(half), 0.0, math.sin(half), 0.0)
        landmarks = _CURLED_RIGHT
    else:
        tracker_position = (0.0, 0.0, 1.0)
        tracker_quaternion = (1.0, 0.0, 0.0, 0.0)
        landmarks = _OPEN_RIGHT
    return FixtureInputState(
        phase=phase,
        tracker_position_m=tracker_position,
        tracker_quat_wxyz=tracker_quaternion,
        hand_landmarks_m=_mirror_for_side(landmarks, side),
    )


def fixture_profile_mapping() -> dict[str, object]:
    def state_payload(sequence: int) -> dict[str, object]:
        return {
            side.value: {
                "tracker_position_m": list(input_state(side, sequence).tracker_position_m),
                "tracker_quat_wxyz": list(input_state(side, sequence).tracker_quat_wxyz),
                "hand_landmark_names": [item.value for item in MEDIAPIPE_HAND_LANDMARK_NAMES],
                "hand_landmarks_m": [
                    list(item) for item in input_state(side, sequence).hand_landmarks_m
                ],
            }
            for side in (HandSide.LEFT, HandSide.RIGHT)
        }

    return {
        "schema": "wujihand.dataset_preview_fixture_profile.v1",
        "profile_id": FIXTURE_PROFILE_ID,
        "producer_instance": FIXTURE_PRODUCER,
        "rate_hz": FIXTURE_RATE_HZ,
        "phases": [
            {"phase": "a_reference", "start_sequence": 0, "frames": REFERENCE_FRAMES},
            {
                "phase": "b_motion",
                "start_sequence": REFERENCE_FRAMES,
                "frames": MOTION_FRAMES,
            },
            {
                "phase": "a_return",
                "start_sequence": REFERENCE_FRAMES + MOTION_FRAMES,
                "frames": RETURN_FRAMES,
            },
        ],
        "states": {
            "a": state_payload(0),
            "b": state_payload(REFERENCE_FRAMES),
        },
    }


def fixture_profile_sha256() -> str:
    payload = json.dumps(
        fixture_profile_mapping(),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


__all__ = [
    "FIXTURE_PRODUCER",
    "FIXTURE_PROFILE_ID",
    "FIXTURE_RATE_HZ",
    "MOTION_FRAMES",
    "REFERENCE_FRAMES",
    "REQUIRED_FRAMES",
    "RETURN_FRAMES",
    "FixtureInputState",
    "fixture_profile_mapping",
    "fixture_profile_sha256",
    "input_state",
    "phase_for_sequence",
]
