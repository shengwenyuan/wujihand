from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path

import pytest

from wujihand.adapters.storage import (
    decode_canonical_hand_observation_json,
    encode_canonical_hand_observation_json,
    read_canonical_hand_observations_jsonl,
    write_canonical_hand_observations_jsonl,
)
from wujihand.domain import (
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
)


def _observation(
    *,
    side: HandSide,
    sequence: int = 12,
    receive_time_ns: int = 1_100,
) -> CanonicalHandObservation:
    return CanonicalHandObservation(
        side=side,
        sequence=sequence,
        source_id=f"sanitized.wuji_glove.{side.value}.fixture_01",
        calibration_id=f"sanitized-user.{side.value}.v1",
        transform_id="wuji_glove.hand_skeleton.v1",
        source_time_ns=receive_time_ns - 100,
        receive_time_ns=receive_time_ns,
        device_time_ns=900 + sequence,
        device_clock_domain="wuji_glove_device_clock",
        frame_id="l_wrist" if side is HandSide.LEFT else "r_wrist",
        landmarks=tuple(
            HandLandmark(
                name=name,
                position_m=(
                    index / 100.0,
                    index / 200.0,
                    index / 400.0,
                ),
                confidence=0.97,
            )
            for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
        ),
    )


@pytest.mark.parametrize("side", tuple(HandSide))
def test_canonical_hand_json_round_trip_preserves_every_domain_field(
    side: HandSide,
) -> None:
    observation = _observation(side=side)

    encoded = encode_canonical_hand_observation_json(observation)
    decoded = decode_canonical_hand_observation_json(encoded)

    assert decoded == observation
    assert len(decoded.landmarks) == 21
    assert tuple(landmark.name for landmark in decoded.landmarks) == (MEDIAPIPE_HAND_LANDMARK_NAMES)


def test_missing_landmark_position_round_trips_without_inventing_geometry() -> None:
    observation = _observation(side=HandSide.LEFT)
    landmarks = list(observation.landmarks)
    landmarks[8] = HandLandmark(
        name=landmarks[8].name,
        position_m=None,
        confidence=0.0,
    )
    observation = CanonicalHandObservation(
        side=observation.side,
        sequence=observation.sequence,
        source_id=observation.source_id,
        calibration_id=observation.calibration_id,
        transform_id=observation.transform_id,
        source_time_ns=observation.source_time_ns,
        receive_time_ns=observation.receive_time_ns,
        device_time_ns=observation.device_time_ns,
        device_clock_domain=observation.device_clock_domain,
        frame_id=observation.frame_id,
        landmarks=tuple(landmarks),
    )

    decoded = decode_canonical_hand_observation_json(
        encode_canonical_hand_observation_json(observation)
    )

    assert decoded.landmarks[8].position_m is None
    assert decoded.landmarks[8].confidence == 0.0


@pytest.mark.parametrize(
    "mutation",
    (
        lambda payload: payload.pop("transform_id"),
        lambda payload: payload.update({"extra": "forbidden"}),
        lambda payload: payload.update({"side": "unknown"}),
        lambda payload: payload.update({"sequence": 1.5}),
        lambda payload: payload.update({"receive_time_ns": True}),
        lambda payload: payload.update({"landmarks": payload["landmarks"][:-1]}),
        lambda payload: payload["landmarks"][0].pop("confidence"),
        lambda payload: payload["landmarks"][0].update({"extra": 1}),
        lambda payload: payload["landmarks"][1].update({"name": payload["landmarks"][0]["name"]}),
    ),
)
def test_decoder_rejects_missing_extra_or_ambiguous_fields(
    mutation: Callable[[dict[str, object]], object],
) -> None:
    payload = json.loads(encode_canonical_hand_observation_json(_observation(side=HandSide.RIGHT)))
    mutation(payload)

    with pytest.raises(ValueError):
        decode_canonical_hand_observation_json(json.dumps(payload))


def test_decoder_rejects_duplicate_fields_at_outer_and_landmark_levels() -> None:
    encoded = encode_canonical_hand_observation_json(_observation(side=HandSide.LEFT))
    outer_duplicate = encoded[:-1] + ',"side":"left"}'
    landmark_duplicate = encoded.replace(
        '"confidence":0.97,',
        '"confidence":0.97,"confidence":0.96,',
        1,
    )

    with pytest.raises(ValueError, match="strict JSON"):
        decode_canonical_hand_observation_json(outer_duplicate)
    with pytest.raises(ValueError, match="strict JSON"):
        decode_canonical_hand_observation_json(landmark_duplicate)


@pytest.mark.parametrize(
    "invalid",
    (
        '"confidence":NaN',
        '"confidence":Infinity',
        '"confidence":-Infinity',
        '"position_m":[0.0,1e999,0.0]',
    ),
)
def test_decoder_rejects_every_nonfinite_json_number(invalid: str) -> None:
    encoded = encode_canonical_hand_observation_json(_observation(side=HandSide.RIGHT))
    if invalid.startswith('"position_m"'):
        mutated = encoded.replace('"position_m":[0.0,0.0,0.0]', invalid, 1)
    else:
        mutated = encoded.replace('"confidence":0.97', invalid, 1)

    with pytest.raises(ValueError):
        decode_canonical_hand_observation_json(mutated)


def test_jsonl_round_trip_is_bounded_and_rejects_truncation(tmp_path: Path) -> None:
    path = tmp_path / "sanitized_hand_observations.jsonl"
    observations = (
        _observation(side=HandSide.LEFT, sequence=0),
        _observation(
            side=HandSide.RIGHT,
            sequence=1,
            receive_time_ns=1_200,
        ),
    )
    write_canonical_hand_observations_jsonl(path, observations)

    assert read_canonical_hand_observations_jsonl(path) == observations

    path.write_text(path.read_text(encoding="utf-8").rstrip("\n"), encoding="utf-8")
    with pytest.raises(ValueError, match="truncated"):
        read_canonical_hand_observations_jsonl(path)
