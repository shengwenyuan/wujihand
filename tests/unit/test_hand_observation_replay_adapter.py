from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.adapters.storage import (
    CanonicalHandObservationReplayAdapter,
    HandObservationReplayExhausted,
    write_canonical_hand_observations_jsonl,
)
from wujihand.domain import (
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
)
from wujihand.ports import HandObservationInputPort


def _observation(
    *,
    side: HandSide,
    sequence: int,
    source_time_ns: int | None = 800,
    receive_time_ns: int = 1_000,
) -> CanonicalHandObservation:
    return CanonicalHandObservation(
        side=side,
        sequence=sequence,
        source_id=f"sanitized.glove.{side.value}.fixture",
        calibration_id=f"sanitized-user.{side.value}.v1",
        transform_id="wuji_glove.hand_skeleton.v1",
        source_time_ns=source_time_ns,
        receive_time_ns=receive_time_ns,
        device_time_ns=700 + sequence,
        device_clock_domain="wuji_glove_device_clock",
        frame_id="l_wrist" if side is HandSide.LEFT else "r_wrist",
        landmarks=tuple(
            HandLandmark(
                name=name,
                position_m=(index / 100.0, 0.0, 0.0),
                confidence=0.9,
            )
            for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
        ),
    )


def test_replay_implements_input_port_and_rebases_fresh_time_only() -> None:
    left = _observation(side=HandSide.LEFT, sequence=21)
    right = _observation(
        side=HandSide.RIGHT,
        sequence=34,
        source_time_ns=None,
        receive_time_ns=1_100,
    )
    replay = CanonicalHandObservationReplayAdapter((left, right))

    assert isinstance(replay, HandObservationInputPort)
    replay.start()
    replayed_left = replay.poll(receive_time_ns=10_000)
    replayed_right = replay.poll(receive_time_ns=10_010)

    assert replayed_left.receive_time_ns == 10_000
    assert replayed_left.source_time_ns == 9_800
    assert replayed_right.receive_time_ns == 10_010
    assert replayed_right.source_time_ns is None
    for replayed, recorded in (
        (replayed_left, left),
        (replayed_right, right),
    ):
        assert replayed.side is recorded.side
        assert replayed.sequence == recorded.sequence
        assert replayed.source_id == recorded.source_id
        assert replayed.calibration_id == recorded.calibration_id
        assert replayed.transform_id == recorded.transform_id
        assert replayed.device_time_ns == recorded.device_time_ns
        assert replayed.device_clock_domain == recorded.device_clock_domain
        assert replayed.frame_id == recorded.frame_id
        assert replayed.landmarks == recorded.landmarks


def test_replay_eof_is_explicit_and_does_not_advance_on_bad_timestamp() -> None:
    replay = CanonicalHandObservationReplayAdapter((_observation(side=HandSide.LEFT, sequence=1),))
    replay.start()

    with pytest.raises(ValueError, match="too small"):
        replay.poll(receive_time_ns=100)
    assert replay.poll(receive_time_ns=2_000).sequence == 1
    with pytest.raises(HandObservationReplayExhausted, match="end of fixture"):
        replay.poll(receive_time_ns=2_001)


def test_reset_and_restart_begin_at_record_zero_with_a_fresh_time_epoch() -> None:
    record = _observation(side=HandSide.RIGHT, sequence=8)
    replay = CanonicalHandObservationReplayAdapter((record,))
    replay.start()

    assert replay.poll(receive_time_ns=2_000).sequence == 8
    replay.reset()
    assert replay.poll(receive_time_ns=1_500).sequence == 8

    replay.close()
    replay.close()
    with pytest.raises(RuntimeError, match=r"start\(\)"):
        replay.poll(receive_time_ns=2_500)

    replay.start()
    assert replay.poll(receive_time_ns=3_000).sequence == 8


def test_from_jsonl_validates_fixture_before_start_and_close_is_idempotent(
    tmp_path: Path,
) -> None:
    path = tmp_path / "hand.jsonl"
    record = _observation(side=HandSide.LEFT, sequence=5)
    write_canonical_hand_observations_jsonl(path, (record,))

    replay = CanonicalHandObservationReplayAdapter.from_jsonl(path)
    replay.start()
    result = replay.poll(receive_time_ns=3_000)
    replay.close()
    replay.close()

    assert result.side is HandSide.LEFT
    assert result.sequence == 5


def test_replay_rejects_double_start_and_nonincreasing_receive_time() -> None:
    records = (
        _observation(side=HandSide.LEFT, sequence=1),
        _observation(side=HandSide.LEFT, sequence=2),
    )
    replay = CanonicalHandObservationReplayAdapter(records)
    replay.start()
    with pytest.raises(RuntimeError, match="already started"):
        replay.start()
    replay.poll(receive_time_ns=2_000)
    with pytest.raises(ValueError, match="increase strictly"):
        replay.poll(receive_time_ns=2_000)

    assert replay.poll(receive_time_ns=2_001).sequence == 2
