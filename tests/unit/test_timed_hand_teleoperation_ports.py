from __future__ import annotations

from collections.abc import Callable, Iterator

import pytest

from wujihand.adapters.observability import (
    DurationRecorder,
    TimedHandObservationInputAdapter,
    TimedRetargetAdapter,
)
from wujihand.domain import (
    HAND2_LAYOUT_IDS,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    CanonicalHandObservation,
    HandIntent,
    HandLandmark,
    HandSide,
    RetargetStatus,
)
from wujihand.ports import HandObservationInputPort, RetargetPort


def _observation() -> CanonicalHandObservation:
    return CanonicalHandObservation(
        side=HandSide.RIGHT,
        sequence=1,
        source_id="fixture.right",
        calibration_id="fixture.calibration",
        transform_id="fixture.transform",
        source_time_ns=None,
        receive_time_ns=100,
        device_time_ns=10,
        device_clock_domain="fixture.clock",
        frame_id="r_wrist",
        landmarks=tuple(
            HandLandmark(
                name=name,
                position_m=(float(index), 0.0, 0.0),
                confidence=0.5,
            )
            for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
        ),
    )


class _Input:
    def __init__(self, observation: CanonicalHandObservation) -> None:
        self.observation = observation
        self.started = False
        self.closed = False
        self.failure: Exception | None = None

    def start(self) -> None:
        self.started = True

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        assert receive_time_ns is not None
        if self.failure is not None:
            raise self.failure
        return self.observation

    def close(self) -> None:
        self.closed = True


class _Retargeter:
    def __init__(self) -> None:
        self.reset_calls = 0
        self.close_calls = 0
        self.failure: Exception | None = None

    def retarget(
        self,
        observation: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        if self.failure is not None:
            raise self.failure
        assert produced_time_ns is not None
        return HandIntent(
            side=HandSide.RIGHT,
            sequence=sequence,
            source_observation=observation,
            q20_rad=(0.0,) * 20,
            layout_id=HAND2_LAYOUT_IDS["right"],
            produced_time_ns=produced_time_ns,
            retarget_status=RetargetStatus.DEGRADED,
            retarget_confidence=0.5,
            retarget_model_id="fixture.model",
            retarget_config_id="fixture.config",
        )

    def reset(self) -> None:
        self.reset_calls += 1

    def close(self) -> None:
        self.close_calls += 1


def _clock(values: tuple[int, ...]) -> Callable[[], int]:
    iterator: Iterator[int] = iter(values)
    return lambda: next(iterator)


def test_duration_recorder_reports_empty_and_interpolated_percentiles() -> None:
    recorder = DurationRecorder()
    assert recorder.summary().to_report() == {
        "count": 0,
        "mean_ms": None,
        "p50_ms": None,
        "p95_ms": None,
        "max_ms": None,
    }

    for duration_ns in (1_000_000, 2_000_000, 3_000_000):
        recorder.observe_ns(duration_ns)

    summary = recorder.summary()
    assert summary.count == 3
    assert summary.mean_ms == pytest.approx(2.0)
    assert summary.p50_ms == pytest.approx(2.0)
    assert summary.p95_ms == pytest.approx(2.9)
    assert summary.max_ms == pytest.approx(3.0)


def test_timed_input_preserves_port_lifecycle_and_records_failures() -> None:
    delegate = _Input(_observation())
    adapter = TimedHandObservationInputAdapter(
        delegate,
        clock_ns=_clock((10, 1_000_010, 2_000_000, 4_000_000)),
    )
    assert isinstance(adapter, HandObservationInputPort)

    adapter.start()
    assert adapter.poll(receive_time_ns=100) is delegate.observation
    delegate.failure = RuntimeError("input failed")
    with pytest.raises(RuntimeError, match="input failed"):
        adapter.poll(receive_time_ns=200)
    adapter.close()

    assert delegate.started
    assert delegate.closed
    summary = adapter.recorder.summary()
    assert summary.count == 2
    assert summary.mean_ms == pytest.approx(1.5)
    assert summary.max_ms == pytest.approx(2.0)


def test_timed_retarget_preserves_port_lifecycle_and_records_failures() -> None:
    observation = _observation()
    delegate = _Retargeter()
    adapter = TimedRetargetAdapter(
        delegate,
        clock_ns=_clock((100, 900_100, 1_000_000, 3_000_000)),
    )
    assert isinstance(adapter, RetargetPort)

    adapter.reset()
    intent = adapter.retarget(observation, sequence=0, produced_time_ns=200)
    assert intent.source_observation is observation
    delegate.failure = ValueError("solve failed")
    with pytest.raises(ValueError, match="solve failed"):
        adapter.retarget(observation, sequence=1, produced_time_ns=300)
    adapter.close()

    assert delegate.reset_calls == 1
    assert delegate.close_calls == 1
    summary = adapter.recorder.summary()
    assert summary.count == 2
    assert summary.mean_ms == pytest.approx(1.45)
    assert summary.max_ms == pytest.approx(2.0)


@pytest.mark.parametrize("duration_ns", (-1, 1.5, True))
def test_duration_recorder_rejects_invalid_values(duration_ns: object) -> None:
    with pytest.raises(ValueError, match="duration_ns"):
        DurationRecorder().observe_ns(duration_ns)  # type: ignore[arg-type]
