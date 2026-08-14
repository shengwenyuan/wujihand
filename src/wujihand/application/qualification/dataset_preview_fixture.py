"""Deterministic A-B-A canonical inputs for the ROS2-Isaac GUI qualification."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
import math
from typing import Final, Literal

from wujihand.domain import HandSide, MEDIAPIPE_HAND_LANDMARK_NAMES


FIXTURE_PROFILE_ID: Final = "dataset_preview_e2e_aba_v1"
FIXTURE_PRODUCER: Final = "nv5-dataset-preview-fixture"
SELF_COLLISION_FIXTURE_PROFILE_ID: Final = "self_collision_aba_contact_v1"
SELF_COLLISION_FIXTURE_PRODUCER: Final = "nv5-self-collision-fixture"
FIXTURE_RATE_HZ: Final = 120
REFERENCE_FRAMES: Final = 1050
MOTION_FRAMES: Final = 300
RETURN_FRAMES: Final = 1050
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

_SELF_COLLISION_CONTACT_LEFT = (
    (0.0, 0.0, 0.0),
    (0.0344339981675148, 0.0, -0.0288929995149374),
    (0.04517986252903938, -0.004365371074527502, -0.03787568211555481),
    (0.014397011138498783, -0.03787853941321373, -0.062743179500103),
    (-0.01339135691523552, -0.06786182522773743, -0.08489497005939484),
    (0.029433999210596085, -6.938893903907228e-18, -0.0919170007109642),
    (0.03759230673313141, 0.0060501378029584885, -0.12622903287410736),
    (0.04138929024338722, -0.010757172480225563, -0.14565838873386383),
    (0.04592209309339523, -0.03124144859611988, -0.16892702877521515),
    (-8.673617379884035e-19, -6.938893903907228e-18, -0.08954399824142456),
    (0.0028264245484024286, 0.00884720403701067, -0.13971899449825287),
    (0.0034265450667589903, -0.002344677923247218, -0.1526770442724228),
    (0.004648631438612938, -0.02561512589454651, -0.17914937436580658),
    (-0.013712000101804733, 0.0, -0.08442900329828262),
    (-0.008173410780727863, -0.01735491119325161, -0.12161149829626083),
    (-0.005932845640927553, -0.03781843185424805, -0.13037876784801483),
    (-0.005587238818407059, -0.07336213439702988, -0.11661440134048462),
    (-0.034001000225543976, -6.938893903907228e-18, -0.0742809996008873),
    (-0.025343574583530426, -0.02955852635204792, -0.08721296489238739),
    (-0.01908143237233162, -0.051827095448970795, -0.09453702718019485),
    (-0.015720002353191376, -0.06883865594863892, -0.08690719306468964),
)

_SELF_COLLISION_CONTACT_RIGHT = (
    (0.0, 0.0, 0.0),
    (0.0344339981675148, 0.0, -0.0288929995149374),
    (0.04646183177828789, 0.00852001179009676, -0.04645237699151039),
    (0.04442812502384186, 0.045363590121269226, -0.0789194256067276),
    (0.03590509667992592, 0.07866372913122177, -0.10288771241903305),
    (0.029433999210596085, -6.938893903907228e-18, -0.0919170007109642),
    (0.03606526181101799, -0.005563606973737478, -0.12346978485584259),
    (0.039827462285757065, 0.0037401241715997458, -0.14356808364391327),
    (0.04583761841058731, 0.01897844672203064, -0.17574159801006317),
    (0.0, -6.938893903907228e-18, -0.08954399824142456),
    (0.021671218797564507, 0.027613870799541473, -0.11988089978694916),
    (0.030413515865802765, 0.04685183987021446, -0.12474752217531204),
    (0.033802371472120285, 0.07751651108264923, -0.10550980269908905),
    (-0.013712000101804733, -6.938893903907228e-18, -0.08442900329828262),
    (-0.008045727387070656, 0.02661951072514057, -0.12231700122356415),
    (-0.005132663995027542, 0.045083124190568924, -0.13843822479248047),
    (-0.0027467808686196804, 0.07124850898981094, -0.14388324320316315),
    (-0.034001000225543976, -6.938893903907228e-18, -0.0742809996008873),
    (-0.04100652039051056, 0.008302995003759861, -0.10491465777158737),
    (-0.044992364943027496, 0.023567117750644684, -0.11948711425065994),
    (-0.04928288236260414, 0.04191207513213158, -0.1346546858549118),
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


def self_collision_input_state(side: HandSide, sequence: int) -> FixtureInputState:
    if type(side) is not HandSide:
        raise ValueError("fixture side must be a HandSide")
    phase = phase_for_sequence(sequence)
    if phase == "b_motion":
        half = math.radians(8.0) / 2.0
        tracker_position = (0.0, 0.080, 1.0)
        tracker_quaternion = (math.cos(half), 0.0, math.sin(half), 0.0)
        landmarks = (
            _SELF_COLLISION_CONTACT_LEFT if side is HandSide.LEFT else _SELF_COLLISION_CONTACT_RIGHT
        )
    else:
        tracker_position = (0.0, 0.0, 1.0)
        tracker_quaternion = (1.0, 0.0, 0.0, 0.0)
        landmarks = _mirror_for_side(_OPEN_RIGHT, side)
    return FixtureInputState(
        phase=phase,
        tracker_position_m=tracker_position,
        tracker_quat_wxyz=tracker_quaternion,
        hand_landmarks_m=landmarks,
    )


def fixture_profile_mapping() -> dict[str, object]:
    return _profile_mapping(FIXTURE_PROFILE_ID, FIXTURE_PRODUCER, input_state)


def self_collision_fixture_profile_mapping() -> dict[str, object]:
    return _profile_mapping(
        SELF_COLLISION_FIXTURE_PROFILE_ID,
        SELF_COLLISION_FIXTURE_PRODUCER,
        self_collision_input_state,
    )


def _profile_mapping(
    profile_id: str,
    producer: str,
    state: Callable[[HandSide, int], FixtureInputState],
) -> dict[str, object]:
    def state_payload(sequence: int) -> dict[str, object]:
        return {
            side.value: {
                "tracker_position_m": list(state(side, sequence).tracker_position_m),
                "tracker_quat_wxyz": list(state(side, sequence).tracker_quat_wxyz),
                "hand_landmark_names": [item.value for item in MEDIAPIPE_HAND_LANDMARK_NAMES],
                "hand_landmarks_m": [list(item) for item in state(side, sequence).hand_landmarks_m],
            }
            for side in (HandSide.LEFT, HandSide.RIGHT)
        }

    return {
        "schema": "wujihand.dataset_preview_fixture_profile.v1",
        "profile_id": profile_id,
        "producer_instance": producer,
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


def self_collision_fixture_profile_sha256() -> str:
    payload = json.dumps(
        self_collision_fixture_profile_mapping(),
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
    "SELF_COLLISION_FIXTURE_PRODUCER",
    "SELF_COLLISION_FIXTURE_PROFILE_ID",
    "FixtureInputState",
    "fixture_profile_mapping",
    "fixture_profile_sha256",
    "input_state",
    "phase_for_sequence",
    "self_collision_fixture_profile_mapping",
    "self_collision_fixture_profile_sha256",
    "self_collision_input_state",
]
