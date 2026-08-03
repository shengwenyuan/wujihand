from __future__ import annotations

from collections.abc import Callable

import numpy as np
import numpy.typing as npt
import pytest

from wujihand.adapters.retargeting.wuji_hand2 import WujiHand2RetargetAdapter
from wujihand.domain import (
    HAND2_LAYOUT_IDS,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    RetargetStatus,
)
from wujihand.ports import RetargetPort


class _Session:
    def __init__(self, output: object | None = None) -> None:
        self.output = np.linspace(0.0, 0.19, 20, dtype=np.float32) if output is None else output
        self.inputs: list[npt.NDArray[np.float32]] = []
        self.reset_calls = 0
        self.close_calls = 0

    def step(
        self,
        keypoints: npt.NDArray[np.float32],
    ) -> npt.NDArray[np.float32]:
        self.inputs.append(keypoints.copy())
        return self.output  # type: ignore[return-value]

    def reset(self) -> None:
        self.reset_calls += 1

    def close(self) -> None:
        self.close_calls += 1


def _observation(
    *,
    side: HandSide = HandSide.RIGHT,
    sequence: int = 41,
    receive_time_ns: int = 1_100,
    source_id: str = "wuji_glove.right.SN_TEST",
    calibration_id: str = "wuji_sdk.default_user.builtin",
    confidence: float = 0.96,
    missing_index: int | None = None,
) -> CanonicalHandObservation:
    landmarks = tuple(
        HandLandmark(
            name=name,
            position_m=(
                None if index == missing_index else (index / 100.0, index / 200.0, index / 400.0)
            ),
            confidence=0.0 if index == missing_index else confidence,
        )
        for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
    )
    return CanonicalHandObservation(
        side=side,
        sequence=sequence,
        source_id=source_id,
        calibration_id=calibration_id,
        transform_id="wuji_glove.hand_skeleton.v1",
        source_time_ns=None,
        receive_time_ns=receive_time_ns,
        device_time_ns=1_000 + sequence,
        device_clock_domain="wuji_glove_device_clock",
        frame_id="l_wrist" if side is HandSide.LEFT else "r_wrist",
        landmarks=landmarks,
    )


def _factory(
    session: _Session,
    calls: list[HandSide],
) -> Callable[[HandSide], _Session]:
    def create(side: HandSide) -> _Session:
        calls.append(side)
        return session

    return create


def test_retarget_uses_exact_float32_media_pipe_matrix_and_q20_provenance() -> None:
    session = _Session()
    factory_calls: list[HandSide] = []
    adapter = WujiHand2RetargetAdapter(
        HandSide.RIGHT,
        session_factory=_factory(session, factory_calls),
        sdk_version="2026.7.21",
    )
    observation = _observation()

    assert isinstance(adapter, RetargetPort)
    assert factory_calls == []
    intent = adapter.retarget(
        observation,
        sequence=5,
        produced_time_ns=1_200,
    )

    assert factory_calls == [HandSide.RIGHT]
    assert len(session.inputs) == 1
    assert session.inputs[0].shape == (21, 3)
    assert session.inputs[0].dtype == np.float32
    assert session.inputs[0].flags.c_contiguous
    assert session.inputs[0][8] == pytest.approx((0.08, 0.04, 0.02))
    assert intent.side is HandSide.RIGHT
    assert intent.sequence == 5
    assert intent.source_observation is observation
    assert intent.q20_rad[-1] == pytest.approx(0.19)
    assert intent.layout_id == HAND2_LAYOUT_IDS["right"]
    assert intent.retarget_status is RetargetStatus.SUCCESS
    assert intent.retarget_confidence == pytest.approx(0.96)
    assert intent.retarget_model_id == "wuji_sdk.WujiHand2.2026.7.21"
    assert intent.retarget_config_id == (
        "wuji_sdk.builtin.WujiHand2.right.2026.7.21."
        "confidence_floor_0.000.success_0.600"
    )


def test_low_confidence_is_degraded_and_reset_admits_new_source_epoch() -> None:
    session = _Session()
    adapter = WujiHand2RetargetAdapter(
        HandSide.RIGHT,
        session_factory=lambda side: session,
        sdk_version="2026.7.21",
    )
    first = _observation(confidence=0.48)
    intent = adapter.retarget(first, sequence=0, produced_time_ns=1_200)
    assert intent.retarget_status is RetargetStatus.DEGRADED
    assert intent.retarget_confidence == pytest.approx(0.48)
    assert len(session.inputs) == 1

    changed = _observation(
        sequence=42,
        receive_time_ns=1_300,
        source_id="wuji_glove.right.RECONNECTED",
    )
    with pytest.raises(RuntimeError, match=r"call reset\(\)"):
        adapter.retarget(changed, sequence=1, produced_time_ns=1_350)

    adapter.reset()
    second = adapter.retarget(changed, sequence=1, produced_time_ns=1_350)
    assert second.source_observation is changed
    assert session.reset_calls == 1
    assert len(session.inputs) == 2


@pytest.mark.parametrize(
    ("observation", "produced_time_ns", "message"),
    (
        (_observation(side=HandSide.LEFT), 1_200, "observation side"),
        (_observation(missing_index=8), 1_200, "all 21"),
        (_observation(), 300_001_101, "stale"),
    ),
)
def test_retarget_fails_closed_before_sdk_step(
    observation: CanonicalHandObservation,
    produced_time_ns: int,
    message: str,
) -> None:
    session = _Session()
    adapter = WujiHand2RetargetAdapter(
        HandSide.RIGHT,
        session_factory=lambda side: session,
        sdk_version="2026.7.21",
    )

    with pytest.raises(ValueError, match=message):
        adapter.retarget(
            observation,
            sequence=0,
            produced_time_ns=produced_time_ns,
        )
    assert session.inputs == []


def test_explicit_confidence_floor_remains_available_for_strict_deployments() -> None:
    session = _Session()
    adapter = WujiHand2RetargetAdapter(
        HandSide.RIGHT,
        minimum_landmark_confidence=0.75,
        success_landmark_confidence=0.90,
        session_factory=lambda side: session,
        sdk_version="2026.7.21",
    )

    with pytest.raises(ValueError, match="confidence"):
        adapter.retarget(
            _observation(confidence=0.74),
            sequence=0,
            produced_time_ns=1_200,
        )

    assert session.inputs == []


@pytest.mark.parametrize(
    ("output", "message"),
    (
        (np.zeros(19, dtype=np.float32), "shape"),
        (np.full(20, np.nan, dtype=np.float32), "NaN"),
        (["not-a-number"] * 20, "non-numeric"),
    ),
)
def test_retarget_rejects_malformed_sdk_output(
    output: object,
    message: str,
) -> None:
    session = _Session(output)
    adapter = WujiHand2RetargetAdapter(
        HandSide.RIGHT,
        session_factory=lambda side: session,
        sdk_version="2026.7.21",
    )

    with pytest.raises(ValueError, match=message):
        adapter.retarget(_observation(), sequence=0, produced_time_ns=1_200)


def test_stream_order_reset_and_terminal_close_semantics() -> None:
    session = _Session()
    adapter = WujiHand2RetargetAdapter(
        HandSide.RIGHT,
        session_factory=lambda side: session,
        sdk_version="2026.7.21",
    )
    first = _observation()
    adapter.retarget(first, sequence=0, produced_time_ns=1_200)

    with pytest.raises(ValueError, match="sequence must increase"):
        adapter.retarget(first, sequence=1, produced_time_ns=1_201)

    adapter.close()
    adapter.close()
    assert session.close_calls == 1
    with pytest.raises(RuntimeError, match="closed"):
        adapter.reset()
    with pytest.raises(RuntimeError, match="closed"):
        adapter.retarget(
            _observation(sequence=42, receive_time_ns=1_300),
            sequence=2,
            produced_time_ns=1_400,
        )


def test_injected_factory_requires_explicit_sdk_version() -> None:
    with pytest.raises(ValueError, match="sdk_version"):
        WujiHand2RetargetAdapter(
            HandSide.RIGHT,
            session_factory=lambda side: _Session(),
        )
