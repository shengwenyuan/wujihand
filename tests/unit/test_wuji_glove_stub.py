from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.qualification import (
    WUJI_GLOVE_STUB_POSES,
    build_wuji_glove_stub_observations,
    wuji_glove_stub_keypoints,
)
from wujihand.domain import MEDIAPIPE_HAND_LANDMARK_NAMES, HandSide


def test_stub_is_side_mirrored_finite_media_pipe_signal() -> None:
    right = wuji_glove_stub_keypoints(HandSide.RIGHT, "index_opposition")
    left = wuji_glove_stub_keypoints(HandSide.LEFT, "index_opposition")

    assert right.shape == (21, 3)
    assert right.dtype == np.float32
    assert right.flags.c_contiguous
    assert np.isfinite(right).all()
    np.testing.assert_allclose(left[:, 0], -right[:, 0])
    np.testing.assert_allclose(left[:, 1:], right[:, 1:])
    np.testing.assert_allclose(right[4], right[8])


@pytest.mark.parametrize("side", tuple(HandSide))
def test_stub_observations_are_bounded_canonical_stream(side: HandSide) -> None:
    records = build_wuji_glove_stub_observations(
        side,
        calibration_id="wuji_sdk.user.fixture.sdk_2026.8.3",
        frames_per_pose=2,
    )

    assert len(records) == len(WUJI_GLOVE_STUB_POSES) * 2
    assert tuple(record.sequence for record in records) == tuple(range(len(records)))
    assert all(record.side is side for record in records)
    assert all(
        tuple(landmark.name for landmark in record.landmarks)
        == MEDIAPIPE_HAND_LANDMARK_NAMES
        for record in records
    )
    assert records[0].source_id.endswith("qualification_stub")


def test_stub_rejects_unknown_pose_and_empty_sequence() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        wuji_glove_stub_keypoints(HandSide.RIGHT, "fist")
    with pytest.raises(ValueError, match="must not be empty"):
        build_wuji_glove_stub_observations(
            HandSide.RIGHT,
            calibration_id="fixture",
            poses=(),
        )
