"""Timing-only decorators for canonical hand teleoperation ports.

The decorators preserve the wrapped port contracts and record host-side call
duration without exposing SDK objects to the application layer.  They do not
interpret confidence, alter observations, or change command ownership.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import time

from wujihand.domain import CanonicalHandObservation, HandIntent
from wujihand.ports import HandObservationInputPort, RetargetPort


ClockNs = Callable[[], int]


@dataclass(frozen=True, slots=True)
class DurationSummary:
    """Bounded summary of observed host-side call durations."""

    count: int
    mean_ms: float | None
    p50_ms: float | None
    p95_ms: float | None
    max_ms: float | None

    def to_report(self) -> dict[str, int | float | None]:
        return {
            "count": self.count,
            "mean_ms": self.mean_ms,
            "p50_ms": self.p50_ms,
            "p95_ms": self.p95_ms,
            "max_ms": self.max_ms,
        }


class DurationRecorder:
    """Collect non-negative nanosecond durations for one named runtime stage."""

    def __init__(self) -> None:
        self._durations_ns: list[int] = []

    def observe_ns(self, duration_ns: int) -> None:
        if type(duration_ns) is not int or duration_ns < 0:
            raise ValueError("duration_ns must be a non-negative integer")
        self._durations_ns.append(duration_ns)

    def summary(self) -> DurationSummary:
        if not self._durations_ns:
            return DurationSummary(
                count=0,
                mean_ms=None,
                p50_ms=None,
                p95_ms=None,
                max_ms=None,
            )
        ordered = sorted(self._durations_ns)
        return DurationSummary(
            count=len(ordered),
            mean_ms=(sum(ordered) / len(ordered)) / 1_000_000.0,
            p50_ms=_percentile_ns(ordered, 0.50) / 1_000_000.0,
            p95_ms=_percentile_ns(ordered, 0.95) / 1_000_000.0,
            max_ms=ordered[-1] / 1_000_000.0,
        )


class TimedHandObservationInputAdapter:
    """Measure canonical input polling while preserving the input port."""

    def __init__(
        self,
        delegate: HandObservationInputPort,
        *,
        clock_ns: ClockNs = time.monotonic_ns,
        recorder: DurationRecorder | None = None,
    ) -> None:
        self.delegate = delegate
        self.clock_ns = clock_ns
        self.recorder = DurationRecorder() if recorder is None else recorder

    def start(self) -> None:
        self.delegate.start()

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        started_ns = self.clock_ns()
        try:
            return self.delegate.poll(receive_time_ns=receive_time_ns)
        finally:
            self._record_since(started_ns)

    def close(self) -> None:
        self.delegate.close()

    def _record_since(self, started_ns: int) -> None:
        finished_ns = self.clock_ns()
        self.recorder.observe_ns(_duration_ns(started_ns, finished_ns))


class TimedRetargetAdapter:
    """Measure canonical-to-q20 retargeting while preserving the port."""

    def __init__(
        self,
        delegate: RetargetPort,
        *,
        clock_ns: ClockNs = time.monotonic_ns,
        recorder: DurationRecorder | None = None,
    ) -> None:
        self.delegate = delegate
        self.clock_ns = clock_ns
        self.recorder = DurationRecorder() if recorder is None else recorder

    def retarget(
        self,
        observation: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        started_ns = self.clock_ns()
        try:
            return self.delegate.retarget(
                observation,
                sequence=sequence,
                produced_time_ns=produced_time_ns,
            )
        finally:
            self._record_since(started_ns)

    def reset(self) -> None:
        self.delegate.reset()

    def close(self) -> None:
        close = getattr(self.delegate, "close", None)
        if callable(close):
            close()

    def _record_since(self, started_ns: int) -> None:
        finished_ns = self.clock_ns()
        self.recorder.observe_ns(_duration_ns(started_ns, finished_ns))


def _duration_ns(started_ns: object, finished_ns: object) -> int:
    if (
        type(started_ns) is not int
        or type(finished_ns) is not int
        or started_ns < 0
        or finished_ns < started_ns
    ):
        raise RuntimeError("timing clock must return increasing non-negative integer nanoseconds")
    return finished_ns - started_ns


def _percentile_ns(ordered: list[int], quantile: float) -> float:
    if not ordered:
        raise ValueError("ordered durations must not be empty")
    if not math.isfinite(quantile) or not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be finite and in [0, 1]")
    index = (len(ordered) - 1) * quantile
    lower_index = math.floor(index)
    upper_index = math.ceil(index)
    if lower_index == upper_index:
        return float(ordered[lower_index])
    weight = index - lower_index
    return ordered[lower_index] * (1.0 - weight) + ordered[upper_index] * weight


__all__ = [
    "ClockNs",
    "DurationRecorder",
    "DurationSummary",
    "TimedHandObservationInputAdapter",
    "TimedRetargetAdapter",
]
