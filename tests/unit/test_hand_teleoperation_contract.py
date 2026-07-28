from __future__ import annotations

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from wujihand.domain import (
    CANONICAL_HAND_OBSERVATION_SCHEMA,
    HAND2_LAYOUT_IDS,
    HAND_INTENT_SCHEMA,
    HAND_JOINT_POSITION_UNIT,
    HAND_POSITION_UNIT,
    MEDIAPIPE_HAND_LANDMARK_LAYOUT,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandIntent,
    HandLandmark,
    HandSide,
    MediaPipeHandLandmark,
    RetargetStatus,
    hand2_layout,
)
from wujihand.ports import HandObservationInputPort, RetargetPort


def canonical_landmarks() -> tuple[HandLandmark, ...]:
    return tuple(
        HandLandmark(
            name=name,
            position_m=(index / 100.0, index / 200.0, index / 400.0),
            confidence=0.95,
        )
        for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
    )


def observation(**overrides: object) -> CanonicalHandObservation:
    values: dict[str, object] = {
        "side": HandSide.RIGHT,
        "sequence": 41,
        "source_id": "wuji_glove.right.SN001",
        "calibration_id": "glove-user-cal-v3",
        "transform_id": "emf_to_wrist.v1",
        "source_time_ns": 1_000,
        "receive_time_ns": 1_100,
        "device_time_ns": 900,
        "device_clock_domain": "wuji_glove_device_monotonic",
        "clock_domain": "host_monotonic",
        "frame_id": "r_wrist",
        "landmarks": canonical_landmarks(),
    }
    values.update(overrides)
    return CanonicalHandObservation(**values)  # type: ignore[arg-type]


def intent(
    *,
    source: CanonicalHandObservation | None = None,
    **overrides: object,
) -> HandIntent:
    source_observation = observation() if source is None else source
    values: dict[str, object] = {
        "side": source_observation.side,
        "sequence": 9,
        "source_observation": source_observation,
        "q20_rad": tuple(index / 100.0 for index in range(20)),
        "layout_id": HAND2_LAYOUT_IDS[source_observation.side.value],
        "produced_time_ns": source_observation.receive_time_ns + 25,
        "retarget_status": RetargetStatus.SUCCESS,
        "retarget_confidence": 0.88,
        "retarget_model_id": "wuji_sdk.WujiHand2.2026_7_2",
        "retarget_config_id": "sha256:0123456789abcdef",
        "clock_domain": source_observation.clock_domain,
    }
    values.update(overrides)
    return HandIntent(**values)  # type: ignore[arg-type]


def test_canonical_observation_freezes_named_metric_landmark_contract() -> None:
    source_landmarks = list(canonical_landmarks())
    sample = observation(landmarks=source_landmarks)
    source_landmarks.reverse()

    assert sample.schema == CANONICAL_HAND_OBSERVATION_SCHEMA
    assert sample.side is HandSide.RIGHT
    assert sample.landmark_layout == MEDIAPIPE_HAND_LANDMARK_LAYOUT
    assert sample.position_unit == HAND_POSITION_UNIT == "m"
    assert tuple(landmark.name for landmark in sample.landmarks) == (
        MEDIAPIPE_HAND_LANDMARK_NAMES
    )
    assert len(sample.landmarks) == 21
    assert sample.source_time_ns == 1_000
    assert sample.receive_time_ns == 1_100
    assert sample.device_time_ns == 900
    assert sample.device_clock_domain == "wuji_glove_device_monotonic"
    assert sample.calibration_id == "glove-user-cal-v3"
    assert sample.transform_id == "emf_to_wrist.v1"
    with pytest.raises(FrozenInstanceError):
        sample.sequence = 42  # type: ignore[misc]


def test_missing_landmark_is_explicit_and_never_encoded_as_zero_position() -> None:
    values = list(canonical_landmarks())
    values[8] = HandLandmark(
        name=MediaPipeHandLandmark.INDEX_FINGER_TIP,
        position_m=None,
        confidence=0.0,
    )
    sample = observation(landmarks=values)

    assert sample.landmarks[8].position_m is None
    assert sample.landmarks[8].confidence == 0.0

    with pytest.raises(ValueError, match="confidence=0"):
        HandLandmark(
            name=MediaPipeHandLandmark.INDEX_FINGER_TIP,
            position_m=None,
            confidence=0.1,
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("name", "wrist", "MediaPipeHandLandmark"),
        ("position_m", (0.0, 0.0), "exactly 3"),
        ("position_m", (0.0, np.nan, 0.0), "finite"),
        ("position_m", (0.0, True, 0.0), "finite"),
        ("confidence", -0.01, r"\[0, 1\]"),
        ("confidence", np.inf, r"\[0, 1\]"),
        ("confidence", True, r"\[0, 1\]"),
    ),
)
def test_landmark_rejects_malformed_values(
    field: str,
    value: object,
    message: str,
) -> None:
    values: dict[str, object] = {
        "name": MediaPipeHandLandmark.WRIST,
        "position_m": (0.0, 0.0, 0.0),
        "confidence": 1.0,
    }
    values[field] = value
    with pytest.raises(ValueError, match=message):
        HandLandmark(**values)  # type: ignore[arg-type]


def test_observation_requires_exact_canonical_media_pipe_name_order() -> None:
    landmarks = list(canonical_landmarks())
    landmarks[0], landmarks[1] = landmarks[1], landmarks[0]
    with pytest.raises(ValueError, match="canonical MediaPipe 21-name order"):
        observation(landmarks=landmarks)

    with pytest.raises(ValueError, match="canonical MediaPipe 21-name order"):
        observation(landmarks=canonical_landmarks()[:-1])


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("schema", "wrong.v1", "schema"),
        ("side", "right", "HandSide"),
        ("sequence", True, "sequence"),
        ("sequence", -1, "sequence"),
        ("source_id", "bad source", "source_id"),
        ("calibration_id", " ", "calibration_id"),
        ("transform_id", "bad transform", "transform_id"),
        ("source_time_ns", -1, "source_time_ns"),
        ("receive_time_ns", -1, "host_time_ns"),
        ("device_time_ns", -1, "device_time_ns"),
        ("clock_domain", "unix_wall_clock", "clock_domain"),
        ("frame_id", "bad frame", "frame_id"),
        ("landmark_layout", "custom", "landmark_layout"),
        ("position_unit", "mm", "position_unit"),
    ),
)
def test_observation_rejects_ambiguous_metadata(
    field: str,
    value: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        observation(**{field: value})


def test_observation_preserves_unavailable_source_and_device_times_honestly() -> None:
    sample = observation(
        source_time_ns=None,
        device_time_ns=None,
        device_clock_domain=None,
    )
    assert sample.source_time_ns is None
    assert sample.device_time_ns is None
    assert sample.device_clock_domain is None
    assert intent(source=sample).source_age_ns == 25

    with pytest.raises(ValueError, match="not be later"):
        observation(source_time_ns=1_101)
    with pytest.raises(ValueError, match="requires device_time_ns"):
        observation(device_time_ns=None, device_clock_domain="device_monotonic")
    with pytest.raises(ValueError, match="device_clock_domain"):
        observation(device_time_ns=900, device_clock_domain=None)


@pytest.mark.parametrize("side", tuple(HandSide))
def test_hand_intent_uses_side_specific_hand2_q20_layout(side: HandSide) -> None:
    source = observation(
        side=side,
        source_id=f"wuji_glove.{side.value}.SN001",
        frame_id=f"{side.value[0]}_wrist",
    )
    q20 = np.linspace(0.0, 0.19, 20)
    result = intent(source=source, q20_rad=q20)
    q20[:] = 0.0

    assert result.schema == HAND_INTENT_SCHEMA
    assert result.side is side
    assert result.layout_id == HAND2_LAYOUT_IDS[side.value]
    assert result.joint_position_unit == HAND_JOINT_POSITION_UNIT == "rad"
    assert len(result.q20_rad) == hand2_layout(side.value).size == 20
    assert result.q20_rad[-1] == pytest.approx(0.19)
    assert result.source_observation is source
    assert result.source_age_ns == 125


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("schema", "wrong.v1", "schema"),
        ("sequence", True, "sequence"),
        ("sequence", -1, "sequence"),
        ("q20_rad", (0.0,) * 19, "exactly 20"),
        ("q20_rad", (0.0,) * 19 + (np.nan,), "finite"),
        ("q20_rad", (0.0,) * 19 + (True,), "finite"),
        ("layout_id", "wuji_hand2_left_firmware_v1", "layout_id"),
        ("produced_time_ns", 1_099, "must not precede"),
        ("retarget_status", "success", "RetargetStatus"),
        ("retarget_confidence", -0.1, r"\[0, 1\]"),
        ("retarget_model_id", "bad model", "retarget_model_id"),
        ("retarget_config_id", "", "retarget_config_id"),
        ("clock_domain", "steady_clock", "clock_domain"),
        ("joint_position_unit", "degree", "joint_position_unit"),
    ),
)
def test_hand_intent_rejects_malformed_or_untraceable_results(
    field: str,
    value: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        intent(**{field: value})


def test_hand_intent_rejects_source_side_mismatch() -> None:
    source = observation(side=HandSide.LEFT, source_id="glove.left", frame_id="l_wrist")
    with pytest.raises(ValueError, match="side must match"):
        intent(
            source=source,
            side=HandSide.RIGHT,
            layout_id=HAND2_LAYOUT_IDS["right"],
        )


class FakeHandInput:
    def __init__(self, sample: CanonicalHandObservation) -> None:
        self.sample = sample
        self.started = False

    def start(self) -> None:
        self.started = True

    def poll(self, *, receive_time_ns: int | None = None) -> CanonicalHandObservation:
        if receive_time_ns is not None and receive_time_ns != self.sample.receive_time_ns:
            raise ValueError("fixture receive time mismatch")
        return self.sample

    def close(self) -> None:
        self.started = False


class FakeRetargeter:
    def retarget(
        self,
        sample: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        result_time_ns = sample.receive_time_ns + 1
        if produced_time_ns is not None:
            result_time_ns = produced_time_ns
        return intent(
            source=sample,
            sequence=sequence,
            produced_time_ns=result_time_ns,
        )

    def reset(self) -> None:
        return None


def test_input_and_retarget_ports_are_structural_and_backend_neutral() -> None:
    sample = observation()
    source = FakeHandInput(sample)
    retargeter = FakeRetargeter()

    assert isinstance(source, HandObservationInputPort)
    assert isinstance(retargeter, RetargetPort)
    source.start()
    polled = source.poll(receive_time_ns=sample.receive_time_ns)
    result = retargeter.retarget(
        polled,
        sequence=77,
        produced_time_ns=polled.receive_time_ns + 5,
    )
    assert result.sequence == 77
    assert result.source_observation is sample
    retargeter.reset()
    source.close()
    assert not source.started
