"""Monotonic fixed-rate scheduling with explicitly bounded catch-up."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable


NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass(frozen=True, slots=True)
class ScheduledTick:
    control_index: int
    schedule_slot: int
    deadline_ns: int
    wake_time_ns: int
    lateness_ns: int
    missed_periods_before_tick: int


class FixedRateScheduler:
    """Schedule monotonic ticks with a bounded consecutive catch-up streak."""

    def __init__(
        self,
        *,
        rate_hz: int,
        start_ns: int,
        maximum_catch_up_ticks: int = 0,
    ) -> None:
        if type(rate_hz) is not int or not 1 <= rate_hz <= 1_000:
            raise ValueError("rate_hz must be an integer in [1, 1000]")
        if type(start_ns) is not int or start_ns < 0:
            raise ValueError("start_ns must be a non-negative integer")
        if type(maximum_catch_up_ticks) is not int or not 0 <= maximum_catch_up_ticks <= 2:
            raise ValueError("maximum_catch_up_ticks must be an integer in [0, 2]")
        self.rate_hz = rate_hz
        self.start_ns = start_ns
        self.maximum_catch_up_ticks = maximum_catch_up_ticks
        self._control_index = 0
        self._next_slot = 0
        self._next_catch_up_streak = 0
        self._missed_periods = 0
        self._active_tick: ScheduledTick | None = None
        self._active_catch_up_streak = 0

    def wait_next(
        self,
        *,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> ScheduledTick:
        if self._active_tick is not None:
            raise RuntimeError("complete the active tick before requesting another")
        deadline_ns = self._deadline_ns(self._next_slot)
        while True:
            now_ns = clock_ns()
            remaining_ns = deadline_ns - now_ns
            if remaining_ns <= 0:
                break
            sleep(remaining_ns / NANOSECONDS_PER_SECOND)
        wake_time_ns = clock_ns()
        tick = ScheduledTick(
            control_index=self._control_index,
            schedule_slot=self._next_slot,
            deadline_ns=deadline_ns,
            wake_time_ns=wake_time_ns,
            lateness_ns=max(0, wake_time_ns - deadline_ns),
            missed_periods_before_tick=self._missed_periods,
        )
        self._active_tick = tick
        self._active_catch_up_streak = self._next_catch_up_streak
        return tick

    def complete(self, *, completed_ns: int) -> None:
        if type(completed_ns) is not int or completed_ns < 0:
            raise ValueError("completed_ns must be a non-negative integer")
        tick = self._active_tick
        if tick is None:
            raise RuntimeError("no active tick to complete")
        if completed_ns < tick.wake_time_ns:
            raise ValueError("completed_ns must not precede wake_time_ns")
        candidate = tick.schedule_slot + 1
        while self._deadline_ns(candidate) <= completed_ns:
            candidate += 1
        overdue_periods = candidate - tick.schedule_slot - 1
        remaining_catch_up_ticks = max(
            0,
            self.maximum_catch_up_ticks - self._active_catch_up_streak,
        )
        retained_catch_up_ticks = min(overdue_periods, remaining_catch_up_ticks)
        candidate -= retained_catch_up_ticks
        self._control_index += 1
        self._next_slot = candidate
        self._missed_periods = candidate - tick.schedule_slot - 1
        self._next_catch_up_streak = (
            self._active_catch_up_streak + 1 if retained_catch_up_ticks else 0
        )
        self._active_tick = None
        self._active_catch_up_streak = 0

    def _deadline_ns(self, slot: int) -> int:
        offset_ns = (slot * NANOSECONDS_PER_SECOND + self.rate_hz // 2) // self.rate_hz
        return self.start_ns + offset_ns


__all__ = ["FixedRateScheduler", "ScheduledTick"]
