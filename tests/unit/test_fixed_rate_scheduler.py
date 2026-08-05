from __future__ import annotations

import pytest

from wujihand.runtime import FixedRateScheduler


class FakeClock:
    def __init__(self, now_ns: int) -> None:
        self.now_ns = now_ns

    def __call__(self) -> int:
        return self.now_ns

    def sleep(self, duration_s: float) -> None:
        self.now_ns += round(duration_s * 1_000_000_000)


def test_fixed_rate_scheduler_uses_rational_deadlines_without_drift() -> None:
    clock = FakeClock(1_000)
    scheduler = FixedRateScheduler(rate_hz=60, start_ns=clock.now_ns)

    first = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=clock.now_ns + 1_000_000)
    clock.now_ns += 1_000_000
    second = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=clock.now_ns)

    assert first.control_index == 0
    assert first.deadline_ns == 1_000
    assert second.control_index == 1
    assert second.deadline_ns == 16_667_667
    assert second.wake_time_ns == second.deadline_ns
    assert second.missed_periods_before_tick == 0


def test_fixed_rate_scheduler_skips_missed_slots_without_catch_up() -> None:
    clock = FakeClock(0)
    scheduler = FixedRateScheduler(rate_hz=10, start_ns=0)

    first = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=250_000_000)
    clock.now_ns = 250_000_000
    second = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)

    assert first.schedule_slot == 0
    assert second.control_index == 1
    assert second.schedule_slot == 3
    assert second.deadline_ns == 300_000_000
    assert second.missed_periods_before_tick == 2


def test_fixed_rate_scheduler_requires_completion_and_valid_time() -> None:
    clock = FakeClock(0)
    scheduler = FixedRateScheduler(rate_hz=60, start_ns=0)
    scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)

    with pytest.raises(RuntimeError, match="complete"):
        scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    with pytest.raises(ValueError, match="non-negative"):
        scheduler.complete(completed_ns=-1)


def test_fixed_rate_scheduler_can_retain_one_overdue_tick() -> None:
    clock = FakeClock(0)
    scheduler = FixedRateScheduler(
        rate_hz=10,
        start_ns=0,
        maximum_catch_up_ticks=1,
    )

    first = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=150_000_000)
    clock.now_ns = 150_000_000
    catch_up = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=180_000_000)
    clock.now_ns = 180_000_000
    recovered = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)

    assert first.schedule_slot == 0
    assert catch_up.schedule_slot == 1
    assert catch_up.missed_periods_before_tick == 0
    assert recovered.schedule_slot == 2
    assert recovered.missed_periods_before_tick == 0


def test_fixed_rate_scheduler_never_runs_consecutive_catch_up_ticks() -> None:
    clock = FakeClock(0)
    scheduler = FixedRateScheduler(
        rate_hz=10,
        start_ns=0,
        maximum_catch_up_ticks=1,
    )

    scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=150_000_000)
    clock.now_ns = 150_000_000
    scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=250_000_000)
    clock.now_ns = 250_000_000
    after_catch_up = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)

    assert after_catch_up.schedule_slot == 3
    assert after_catch_up.missed_periods_before_tick == 1


def test_fixed_rate_scheduler_can_recover_with_two_catch_up_ticks() -> None:
    clock = FakeClock(0)
    scheduler = FixedRateScheduler(
        rate_hz=10,
        start_ns=0,
        maximum_catch_up_ticks=2,
    )

    scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=250_000_000)
    clock.now_ns = 250_000_000
    first_catch_up = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=280_000_000)
    clock.now_ns = 280_000_000
    second_catch_up = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)
    scheduler.complete(completed_ns=290_000_000)
    clock.now_ns = 290_000_000
    recovered = scheduler.wait_next(clock_ns=clock, sleep=clock.sleep)

    assert first_catch_up.schedule_slot == 1
    assert second_catch_up.schedule_slot == 2
    assert recovered.schedule_slot == 3
    assert recovered.deadline_ns == 300_000_000
    assert all(
        tick.missed_periods_before_tick == 0
        for tick in (first_catch_up, second_catch_up, recovered)
    )


@pytest.mark.parametrize("value", (-1, 3, 1.0, True))
def test_fixed_rate_scheduler_rejects_unbounded_catch_up(value: object) -> None:
    with pytest.raises(ValueError, match="maximum_catch_up_ticks"):
        FixedRateScheduler(
            rate_hz=60,
            start_ns=0,
            maximum_catch_up_ticks=value,  # type: ignore[arg-type]
        )
