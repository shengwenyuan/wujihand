from __future__ import annotations

import ast
import importlib.util
from io import StringIO
import json
from pathlib import Path
import sys
from typing import cast

import pytest

from wujihand.adapters.input.openvr_tracker import JsonValue
from wujihand.domain import (
    ClutchEdge,
    ClutchEvent,
    TrackedRigidBodySample,
    TrackingState,
)
from wujihand.ports import TrackerInventoryItem, TrackingPoll


ROOT = Path(__file__).parents[2]
SPEC = importlib.util.spec_from_file_location(
    "qualify_vive_tracker",
    ROOT / "tools/qualify_vive_tracker.py",
)
assert SPEC is not None and SPEC.loader is not None
cli = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = cli
SPEC.loader.exec_module(cli)

SERIAL = "LHR-QUALIFY-TEST"


class _FakeClock:
    def __init__(self) -> None:
        self.now_ns = 1_000_000_000

    def __call__(self) -> int:
        self.now_ns += 1
        return self.now_ns

    def sleep(self, duration_s: float) -> None:
        self.now_ns += round(duration_s * 1_000_000_000)


class _FakeProcessClock:
    def __init__(self) -> None:
        self.now_ns = 100_000_000

    def __call__(self) -> int:
        value = self.now_ns
        self.now_ns += 10_000_000
        return value


class _FakeAdapter:
    instances: list[_FakeAdapter] = []
    valid_pose = True

    def __init__(
        self,
        tracker_serial: str | None,
        stream_id: str,
        logical_role: str,
        *,
        tracking_frame: str = "vive_tracking",
        clutch_button_id: int | None = None,
        clutch_input_id: str = "tracker_clutch",
    ) -> None:
        self.tracker_serial = tracker_serial
        self.stream_id = stream_id
        self.logical_role = logical_role
        self.tracking_frame = tracking_frame
        self.clutch_button_id = clutch_button_id
        self.clutch_input_id = clutch_input_id
        self.sequence = 0
        self.closed = False
        self._last_raw_record: dict[str, JsonValue] | None = None
        self.instances.append(self)

    @property
    def last_raw_record(self) -> dict[str, JsonValue] | None:
        return self._last_raw_record

    def inventory(self) -> tuple[TrackerInventoryItem, ...]:
        return (
            TrackerInventoryItem(
                serial=SERIAL,
                device_class="generic_tracker",
                model="VIVE Tracker",
                manufacturer="HTC",
                connected=True,
            ),
        )

    def start(self) -> TrackerInventoryItem:
        assert self.tracker_serial == SERIAL
        return self.inventory()[0]

    def poll(self, *, host_time_ns: int | None = None) -> TrackingPoll:
        assert host_time_ns is not None
        valid = self.valid_pose
        state = TrackingState.RUNNING if valid else TrackingState.LOST
        sample = TrackedRigidBodySample(
            stream_id=self.stream_id,
            device_serial=cast(str, self.tracker_serial),
            logical_role=self.logical_role,
            sequence=self.sequence,
            tracking_frame=self.tracking_frame,
            position_m=(0.1, 0.2, 0.3) if valid else None,
            quat_wxyz=(1.0, 0.0, 0.0, 0.0) if valid else None,
            connected=True,
            pose_valid=valid,
            tracking_state=state,
            quality=1.0 if valid else None,
            host_time_ns=host_time_ns,
            device_time_ns=None,
        )
        events: tuple[ClutchEvent, ...] = ()
        if self.sequence == 1:
            events = (
                ClutchEvent(
                    stream_id=self.stream_id,
                    device_serial=cast(str, self.tracker_serial),
                    logical_role=self.logical_role,
                    input_id=self.clutch_input_id,
                    edge=ClutchEdge.PRESSED,
                    sequence=0,
                    host_time_ns=host_time_ns,
                    epoch_request=True,
                ),
            )
        self._last_raw_record = {
            "host_time_ns": host_time_ns,
            "serial": cast(str, self.tracker_serial),
            "pose_valid": valid,
            "matrix_3x4": [
                [1.0, 0.0, 0.0, 0.1],
                [0.0, 1.0, 0.0, 0.2],
                [0.0, 0.0, 1.0, 0.3],
            ]
            if valid
            else None,
        }
        self.sequence += 1
        return TrackingPoll(sample=sample, clutch_events=events)

    def close(self) -> None:
        self.closed = True


@pytest.fixture(autouse=True)
def _reset_fake_adapter() -> None:
    _FakeAdapter.instances.clear()
    _FakeAdapter.valid_pose = True


def _capture_args(output_dir: Path) -> list[str]:
    return [
        "capture",
        "--serial",
        SERIAL,
        "--stream-id",
        "vive.right",
        "--logical-role",
        "operator_right",
        "--scenario",
        "stationary",
        "--duration-s",
        "0.1",
        "--poll-hz",
        "20",
        "--output-dir",
        str(output_dir),
        "--clutch-button-id",
        "2",
    ]


def test_cli_has_no_session_backend_or_robot_runtime_imports() -> None:
    assert cli.__file__ is not None
    path = Path(cli.__file__)
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    modules = {
        node.module or ""
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }
    modules.update(
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    )

    forbidden = ("isaac", "rclpy", "nero", "runtime.session", "backend")
    assert not any(
        token in module.lower()
        for module in modules
        for token in forbidden
    )
    assert "--session" not in cli.build_parser().format_help()


def test_inventory_outputs_stable_identity_and_closes_runtime() -> None:
    output = StringIO()

    result = cli.main(
        ["inventory"],
        adapter_factory=_FakeAdapter,
        output=output,
    )

    assert result == 0
    payload = json.loads(output.getvalue())
    assert payload["schema"] == cli.INVENTORY_SCHEMA
    assert payload["devices"] == [
        {
            "connected": True,
            "device_class": "generic_tracker",
            "manufacturer": "HTC",
            "model": "VIVE Tracker",
            "serial": SERIAL,
        }
    ]
    assert _FakeAdapter.instances[0].closed


def test_blank_runtime_exception_reports_its_type() -> None:
    class BlankRuntimeError(RuntimeError):
        pass

    def failing_factory(*_: object, **__: object) -> _FakeAdapter:
        raise BlankRuntimeError

    error = StringIO()
    result = cli.main(
        ["inventory"],
        adapter_factory=failing_factory,
        output=StringIO(),
        error=error,
    )

    assert result == 1
    assert error.getvalue() == "error: BlankRuntimeError\n"


def test_capture_is_bounded_and_writes_all_strict_artifacts(tmp_path: Path) -> None:
    clock = _FakeClock()
    output = StringIO()
    output_dir = tmp_path / "stationary"

    result = cli.main(
        _capture_args(output_dir),
        adapter_factory=_FakeAdapter,
        clock_ns=clock,
        process_clock_ns=_FakeProcessClock(),
        sleeper=clock.sleep,
        output=output,
    )

    assert result == 0
    assert {path.name for path in output_dir.iterdir()} == {
        "manifest.json",
        "raw_openvr.jsonl",
        "samples.jsonl",
        "events.jsonl",
        "summary.json",
    }
    manifest = json.loads((output_dir / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    raw_records = [
        json.loads(line)
        for line in (output_dir / "raw_openvr.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    samples = cli.read_tracking_samples_jsonl(output_dir / "samples.jsonl")
    events = cli.read_clutch_events_jsonl(output_dir / "events.jsonl")

    assert manifest["selected_tracker"]["serial"] == SERIAL
    assert manifest["scenario"] == "stationary"
    assert summary["status"] == "PASS"
    assert summary["metrics"]["sample_count"] == 2
    assert summary["metrics"]["valid_sample_count"] == 2
    assert summary["event_count"] == 1
    assert summary["capture_performance"] == manifest["capture_performance"]
    assert summary["capture_performance"]["process_cpu_time_s"] == pytest.approx(0.01)
    assert (
        summary["capture_performance"]["cpu_percent_normalization"]
        == "one_logical_core_equals_100_percent"
    )
    assert len(raw_records) == len(samples) == 2
    assert len(events) == 1
    assert len(_FakeAdapter.instances) == 1
    assert _FakeAdapter.instances[0].sequence == 2
    assert _FakeAdapter.instances[0].closed
    assert json.loads(output.getvalue()) == summary


def test_capture_with_no_valid_pose_writes_evidence_and_exits_nonzero(
    tmp_path: Path,
) -> None:
    _FakeAdapter.valid_pose = False
    clock = _FakeClock()
    output_dir = tmp_path / "lost"

    result = cli.main(
        _capture_args(output_dir),
        adapter_factory=_FakeAdapter,
        clock_ns=clock,
        process_clock_ns=_FakeProcessClock(),
        sleeper=clock.sleep,
        output=StringIO(),
    )

    assert result == cli.NO_VALID_POSE_EXIT
    summary = json.loads((output_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["status"] == "NO_VALID_POSE"
    assert summary["metrics"]["valid_sample_count"] == 0
    assert summary["metrics"]["sample_count"] == 2


def test_capture_rejects_nonfinite_raw_data_before_writing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original_poll = _FakeAdapter.poll

    def poll_with_nan(
        self: _FakeAdapter,
        *,
        host_time_ns: int | None = None,
    ) -> TrackingPoll:
        result = original_poll(self, host_time_ns=host_time_ns)
        assert self._last_raw_record is not None
        self._last_raw_record["bad"] = float("nan")
        return result

    monkeypatch.setattr(_FakeAdapter, "poll", poll_with_nan)
    clock = _FakeClock()
    output_dir = tmp_path / "nan"
    error = StringIO()

    result = cli.main(
        _capture_args(output_dir),
        adapter_factory=_FakeAdapter,
        clock_ns=clock,
        process_clock_ns=_FakeProcessClock(),
        sleeper=clock.sleep,
        output=StringIO(),
        error=error,
    )

    assert result == 1
    assert "Out of range float values" in error.getvalue()
    assert not output_dir.exists()
    assert _FakeAdapter.instances[0].closed


def test_replay_recomputes_metrics_from_canonical_jsonl(tmp_path: Path) -> None:
    clock = _FakeClock()
    capture_dir = tmp_path / "capture"
    capture_result = cli.main(
        _capture_args(capture_dir),
        adapter_factory=_FakeAdapter,
        clock_ns=clock,
        process_clock_ns=_FakeProcessClock(),
        sleeper=clock.sleep,
        output=StringIO(),
    )
    assert capture_result == 0

    replay_output = StringIO()
    replay_summary = tmp_path / "replay-summary.json"
    replay_result = cli.main(
        [
            "replay",
            "--input-dir",
            str(capture_dir),
            "--summary-output",
            str(replay_summary),
        ],
        output=replay_output,
    )

    assert replay_result == 0
    replay_payload = json.loads(replay_output.getvalue())
    stored_payload = json.loads(replay_summary.read_text(encoding="utf-8"))
    capture_payload = json.loads((capture_dir / "summary.json").read_text(encoding="utf-8"))
    assert replay_payload == stored_payload
    assert replay_payload["metrics"] == capture_payload["metrics"]
    assert replay_payload["event_count"] == capture_payload["event_count"]
    assert replay_payload["scenario"] is None


@pytest.mark.parametrize(
    "option,value",
    [
        ("--duration-s", "nan"),
        ("--duration-s", "901"),
        ("--poll-hz", "inf"),
        ("--poll-hz", "501"),
    ],
)
def test_capture_rejects_unbounded_or_nonfinite_schedule(
    tmp_path: Path,
    option: str,
    value: str,
) -> None:
    args = _capture_args(tmp_path / "invalid")
    option_index = args.index(option)
    args[option_index + 1] = value

    with pytest.raises(SystemExit) as exc_info:
        cli.main(args, adapter_factory=_FakeAdapter)

    assert exc_info.value.code == 2
