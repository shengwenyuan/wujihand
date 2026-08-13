from pathlib import Path

import pytest
from fakes import FakeClient, FakeClock, FakeSession, target

from wujihand_hand2_hardware.api import qualify_readonly
from wujihand_hand2_hardware.qualification import run_readonly_qualification
from wujihand_hand2_hardware.types import (
    QualificationPolicy,
    QualificationReport,
)


def run(session: FakeSession, *, duration_s: float = 0.01) -> tuple[QualificationReport, FakeClock]:
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(duration_s=duration_s, warmup_s=0, stale_timeout_s=0.1),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    return report, clock


def test_clean_readonly_qualification_passes() -> None:
    report, _ = run(FakeSession())
    assert report.passed
    assert report.failures == ()
    assert report.summary["observed_error_codes"] == [0]


def test_project_response_floor_accepts_exactly_85_percent() -> None:
    report, _ = run(FakeSession(response_rate_pct=85.0))
    assert report.passed
    assert report.summary["minimum_response_rate_pct"] == 85.0


def test_policy_cannot_lower_the_project_response_floor() -> None:
    with pytest.raises(ValueError, match="project floor 85"):
        QualificationPolicy(duration_s=1.0, minimum_response_rate_pct=84.0)


def test_readonly_warmup_ignores_initial_unpopulated_response_window() -> None:
    session = FakeSession(initial_response_rate_zero_frames=4)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(duration_s=0.01, warmup_s=0.005, stale_timeout_s=0.1),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert report.passed
    assert report.summary["warmup_elapsed_s"] >= 0.005
    assert report.summary["minimum_response_rate_pct"] == 100.0


def test_readonly_warmup_allows_initial_transport_sentinel() -> None:
    session = FakeSession(initial_transport_uninitialized_frames=4)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(duration_s=0.01, warmup_s=0.005, stale_timeout_s=0.1),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert report.passed


def test_readonly_warmup_requires_clean_final_response_window() -> None:
    session = FakeSession(initial_response_rate_zero_frames=100)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(duration_s=0.01, warmup_s=0.005, stale_timeout_s=0.1),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert not report.passed
    assert any("warm-up ended with minimum motor response rate" in item for item in report.failures)


def test_readonly_qualification_collects_full_response_window_before_failing() -> None:
    session = FakeSession(initial_response_rate_zero_frames=1)
    report, clock = run(session, duration_s=0.01)
    assert not report.passed
    assert clock.now >= 0.01
    assert report.summary["minimum_response_rate_pct"] == 0.0


@pytest.mark.parametrize(
    ("session", "message"),
    [
        (FakeSession(error_code=6), "nonzero or missing error-code baseline"),
        (FakeSession(missing_nid=24), "unexpected NIDs"),
        (FakeSession(sequence_step=2), "sequence is not contiguous"),
    ],
)
def test_readonly_qualification_fails_closed(session: FakeSession, message: str) -> None:
    report, _ = run(session)
    assert not report.passed
    assert any(message in failure for failure in report.failures)


def test_temperature_guard_stops_on_relative_rise() -> None:
    session = FakeSession(temperature_step_c=0.5)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(
            duration_s=0.1,
            warmup_s=0,
            stale_timeout_s=0.1,
            temperature_sample_period_s=0.01,
            max_temperature_rise_c=2.0,
        ),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert not report.passed
    assert any("temperature rise" in failure for failure in report.failures)
    assert clock.now < 0.1


def test_timeout_counter_is_recorded_without_blocking() -> None:
    session = FakeSession(timeout_delta=1)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(
            duration_s=0.1,
            warmup_s=0,
            stale_timeout_s=0.1,
            temperature_sample_period_s=0.01,
            max_temperature_rise_c=5.0,
        ),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert report.passed
    assert clock.now >= 0.1
    assert report.summary["communication_gate_passed"]
    assert report.summary["communication_observations"]


def test_sub_100_response_above_floor_is_recorded_without_blocking() -> None:
    session = FakeSession(response_rate_pct=87.0)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(
            duration_s=0.02,
            warmup_s=0,
            stale_timeout_s=0.1,
            temperature_sample_period_s=0.01,
            minimum_response_rate_pct=85.0,
        ),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert report.passed
    assert report.summary["communication_gate_passed"]
    assert report.summary["response_windows_below_100_pct"] > 0
    assert any(
        "response rate is 87.000%" in item for item in report.summary["communication_issues"]
    )


def test_response_below_floor_fails_after_collecting_the_window() -> None:
    session = FakeSession(response_rate_pct=84.0)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(
            duration_s=0.02,
            warmup_s=0,
            stale_timeout_s=0.1,
            minimum_response_rate_pct=85.0,
        ),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert not report.passed
    assert clock.now >= 0.02
    assert any("response rate is 84.000%" in failure for failure in report.failures)


def test_temperature_monitor_still_fails_on_missing_joint() -> None:
    session = FakeSession(missing_nid=24)
    clock = FakeClock()
    report = run_readonly_qualification(
        session,
        target(),
        QualificationPolicy(
            duration_s=0.1,
            warmup_s=0,
            stale_timeout_s=0.1,
            temperature_sample_period_s=0.01,
        ),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    assert not report.passed
    assert any("unexpected NIDs" in failure for failure in report.failures)
    assert clock.now < 0.1


def test_public_api_closes_session_and_writes_receipt(tmp_path: Path) -> None:
    session = FakeSession()
    report = qualify_readonly(
        target(),
        QualificationPolicy(duration_s=0.01, warmup_s=0, stale_timeout_s=0.1),
        tmp_path / "run",
        client=FakeClient(session),
    )
    assert report.passed
    assert session.closed
    assert (tmp_path / "run" / "qualification.json").is_file()
    assert (tmp_path / "run" / "events.jsonl").is_file()
    assert (tmp_path / "run" / "communication.jsonl").is_file()
