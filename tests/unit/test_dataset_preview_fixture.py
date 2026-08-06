from __future__ import annotations

import numpy as np

from wujihand.application.qualification.dataset_preview_fixture import (
    MOTION_FRAMES,
    REFERENCE_FRAMES,
    REQUIRED_FRAMES,
    RETURN_FRAMES,
    fixture_profile_mapping,
    fixture_profile_sha256,
    input_state,
    phase_for_sequence,
)
from wujihand.domain import HandSide


def test_fixture_uses_fixed_a_b_a_phases() -> None:
    assert phase_for_sequence(0) == "a_reference"
    assert phase_for_sequence(REFERENCE_FRAMES - 1) == "a_reference"
    assert phase_for_sequence(REFERENCE_FRAMES) == "b_motion"
    assert phase_for_sequence(REFERENCE_FRAMES + MOTION_FRAMES - 1) == "b_motion"
    assert phase_for_sequence(REFERENCE_FRAMES + MOTION_FRAMES) == "a_return"
    assert phase_for_sequence(REQUIRED_FRAMES + 500) == "a_return"
    assert REQUIRED_FRAMES == REFERENCE_FRAMES + MOTION_FRAMES + RETURN_FRAMES


def test_fixture_a_return_is_exact_and_b_is_materially_different() -> None:
    for side in (HandSide.LEFT, HandSide.RIGHT):
        reference = input_state(side, 0)
        motion = input_state(side, REFERENCE_FRAMES)
        returned = input_state(side, REQUIRED_FRAMES - 1)
        assert reference.tracker_position_m == returned.tracker_position_m
        assert reference.tracker_quat_wxyz == returned.tracker_quat_wxyz
        assert reference.hand_landmarks_m == returned.hand_landmarks_m
        assert (
            np.max(
                np.abs(np.asarray(motion.hand_landmarks_m) - np.asarray(reference.hand_landmarks_m))
            )
            >= 0.04
        )
        assert (
            np.linalg.norm(
                np.asarray(motion.tracker_position_m) - np.asarray(reference.tracker_position_m)
            )
            == 0.08
        )


def test_fixture_profile_hash_is_stable_and_covers_both_sides() -> None:
    mapping = fixture_profile_mapping()
    assert mapping["profile_id"] == "dataset_preview_e2e_aba_v1"
    assert set(mapping["states"]["a"]) == {"left", "right"}  # type: ignore[index]
    assert len(fixture_profile_sha256()) == 64
    assert fixture_profile_sha256() == fixture_profile_sha256()
