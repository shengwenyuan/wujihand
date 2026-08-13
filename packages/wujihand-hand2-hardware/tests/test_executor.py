import json
from pathlib import Path

from fakes import FakeClock, FakeMotionClient, FakeMotionSession, target

from wujihand_hand2_hardware.executor import (
    H2_SEQUENCE_WAIVER_ID,
    H3_SEQUENCE_PROFILE,
    run_joint_sequence,
)
from wujihand_hand2_hardware.mapping import H3_S1_SEQUENCE_LABELS
from wujihand_hand2_hardware.types import (
    JointMotionStep,
    JointSequencePolicy,
    MotionPlan,
)


def policy(
    *,
    delta_rad: float = 0.12,
    minimum_response_rate_pct: float = 85.0,
) -> JointSequencePolicy:
    return JointSequencePolicy(
        profile_name=H3_SEQUENCE_PROFILE,
        steps=tuple(
            JointMotionStep(joint_label=label, delta_rad=delta_rad)
            for label in H3_S1_SEQUENCE_LABELS
        ),
        idle_sleep_s=0.01,
        minimum_response_rate_pct=minimum_response_rate_pct,
    )


def run(
    tmp_path: Path,
    session: FakeMotionSession,
    *,
    confirm: bool = True,
):
    clock = FakeClock()

    def confirmation(plan: MotionPlan) -> bool:
        session.call_order.append("confirm")
        assert len(plan.steps) == 5
        return confirm

    report = run_joint_sequence(
        target(),
        policy(),
        tmp_path / "run",
        waiver_id=H2_SEQUENCE_WAIVER_ID,
        confirm=confirmation,
        client=FakeMotionClient(session),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )
    return report, clock


def test_s1_sequence_is_serial_bounded_and_returns_to_baseline(tmp_path: Path) -> None:
    session = FakeMotionSession()
    report, _ = run(tmp_path, session)

    assert report.automatic_checks_passed
    assert report.operator_observation_required
    assert session.call_order.index("confirm") < session.call_order.index("client_open")
    assert session.call_order.index("send_command") < session.call_order.index("enable_selected")
    assert session.call_order.count("enable_selected") == 5
    assert session.call_order.count("disable_selected") == 5
    assert session.call_order[-1] == "close_command_stream"
    assert [mask.index(1) for mask in session.enabled_masks] == [0, 4, 8, 12, 16]
    assert all(sum(mask) == 1 for mask in session.enabled_masks)
    assert session.enabled_mask == [0] * 20
    assert session.commands
    assert all(len(command) == 20 for command in session.commands)
    for index in (0, 4, 8, 12, 16):
        assert max(command[index].position_rad for command in session.commands) <= 0.12
        assert max(command[index].position_rad for command in session.commands) >= 0.119
    assert all(abs(position) <= 1e-12 for position in session.positions)
    assert (tmp_path / "run" / "commands.jsonl").is_file()
    payload = json.loads((tmp_path / "run" / "qualification.json").read_text())
    assert payload["automatic_checks_passed"]
    assert len(payload["summary"]["step_results"]) == 5
    assert [result["preview"]["nid"] for result in payload["summary"]["step_results"]] == [
        1,
        6,
        11,
        16,
        21,
    ]


def test_empty_line_confirmation_is_before_any_connection_or_write(tmp_path: Path) -> None:
    session = FakeMotionSession()
    report, _ = run(tmp_path, session, confirm=False)

    assert not report.automatic_checks_passed
    assert session.call_order == ["confirm"]
    assert not session.commands
    assert not session.closed


def test_hot_device_is_rejected_before_write(tmp_path: Path) -> None:
    session = FakeMotionSession(temperature_c=65.0)
    report, _ = run(tmp_path, session)

    assert not report.automatic_checks_passed
    assert any("temperature" in failure for failure in report.failures)
    assert not session.commands
    assert "open_command_stream" not in session.call_order


def test_state_watchdog_disables_whole_hand(tmp_path: Path) -> None:
    session = FakeMotionSession(stale_state_after_enable=True)
    report, _ = run(tmp_path, session)

    assert not report.automatic_checks_passed
    assert "disable_selected_all" in session.call_order
    assert session.enabled_mask == [0] * 20
    assert not session.emergency_stopped


def test_project_response_floor_accepts_exactly_85_percent(tmp_path: Path) -> None:
    session = FakeMotionSession(response_rate_after_enable_pct=85.0)
    report, _ = run(tmp_path, session)

    assert report.automatic_checks_passed
    assert session.call_order.count("enable_selected") == 5


def test_preflight_rejects_response_below_project_floor(tmp_path: Path) -> None:
    session = FakeMotionSession(preflight_response_rate_pct=84.0)
    report, _ = run(tmp_path, session)

    assert not report.automatic_checks_passed
    assert report.summary["preflight"]["summary"]["communication_observations"]
    assert "open_command_stream" not in session.call_order


def test_motion_rejects_response_below_project_floor(tmp_path: Path) -> None:
    session = FakeMotionSession(response_rate_after_enable_pct=84.0)
    clock = FakeClock()
    report = run_joint_sequence(
        target(),
        policy(),
        tmp_path / "run",
        waiver_id=H2_SEQUENCE_WAIVER_ID,
        confirm=lambda plan: True,
        client=FakeMotionClient(session),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert not report.automatic_checks_passed
    assert any(
        "response rate 84.000% is below project minimum 85.000%" in failure
        for failure in report.failures
    )
    assert "disable_selected_all" in session.call_order


def test_disable_failure_escalates_to_emergency_stop(tmp_path: Path) -> None:
    session = FakeMotionSession(send_failure_at=2, disable_fails=True)
    report, _ = run(tmp_path, session)

    assert not report.automatic_checks_passed
    assert session.emergency_stopped
    assert "emergency_stop_all" in session.call_order


def test_keyboard_interrupt_disables_whole_hand(tmp_path: Path) -> None:
    session = FakeMotionSession(interrupt_at=25)
    report, _ = run(tmp_path, session)

    assert not report.automatic_checks_passed
    assert any("KeyboardInterrupt" in failure for failure in report.failures)
    assert "disable_selected_all" in session.call_order
    assert session.enabled_mask == [0] * 20


def test_wrong_waiver_never_confirms_connects_or_writes(tmp_path: Path) -> None:
    session = FakeMotionSession()
    clock = FakeClock()
    confirmed = False

    def confirmation(plan: MotionPlan) -> bool:
        nonlocal confirmed
        del plan
        confirmed = True
        return True

    report = run_joint_sequence(
        target(),
        policy(),
        tmp_path / "run",
        waiver_id="WRONG",
        confirm=confirmation,
        client=FakeMotionClient(session),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert not report.automatic_checks_passed
    assert not confirmed
    assert not session.closed
    assert not session.call_order


def test_delta_above_sequence_ceiling_never_connects(tmp_path: Path) -> None:
    session = FakeMotionSession()
    clock = FakeClock()
    report = run_joint_sequence(
        target(),
        policy(delta_rad=0.16),
        tmp_path / "run",
        waiver_id=H2_SEQUENCE_WAIVER_ID,
        confirm=lambda plan: True,
        client=FakeMotionClient(session),
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert not report.automatic_checks_passed
    assert not session.closed
    assert not session.call_order
