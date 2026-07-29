#!/usr/bin/env python3
"""Qualify one VIVE Tracker without starting a robot or simulation runtime.

This diagnostic wiring is intentionally outside the five-layer Session
composition root.  It only enumerates an input adapter, captures canonical
tracking contracts, or replays bounded JSONL artifacts through pure
qualification metrics.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
import sys
import time
from typing import IO, TYPE_CHECKING, Protocol


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.input.openvr_tracker import (  # noqa: E402
    JsonValue,
    OpenVrTrackerAdapter,
)
from wujihand.adapters.storage import (  # noqa: E402
    read_clutch_events_jsonl,
    read_tracking_samples_jsonl,
    write_clutch_events_jsonl,
    write_tracking_samples_jsonl,
)
from wujihand.application.qualification import compute_tracking_metrics  # noqa: E402
from wujihand.ports import TrackerInventoryItem, TrackingInputPort  # noqa: E402

if TYPE_CHECKING:
    from wujihand.domain import ClutchEvent, TrackedRigidBodySample  # noqa: E402


MANIFEST_SCHEMA = "wujihand.vive_tracking_qualification_manifest.v1"
SUMMARY_SCHEMA = "wujihand.vive_tracking_qualification_summary.v1"
INVENTORY_SCHEMA = "wujihand.vive_tracking_inventory.v1"
NO_VALID_POSE_EXIT = 3

_NANOSECONDS_PER_SECOND = 1_000_000_000
_MIN_DURATION_S = 0.1
_MAX_DURATION_S = 900.0
_MIN_POLL_HZ = 1.0
_MAX_POLL_HZ = 500.0
_SCENARIO = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+-]{0,63}$")
_CAPTURE_ARTIFACTS = (
    "manifest.json",
    "raw_openvr.jsonl",
    "samples.jsonl",
    "events.jsonl",
    "summary.json",
)


class _RawTrackingInput(TrackingInputPort, Protocol):
    @property
    def last_raw_record(self) -> Mapping[str, JsonValue] | None:
        """Return the JSON-safe raw observation corresponding to the last poll."""


AdapterFactory = Callable[..., _RawTrackingInput]
ClockNs = Callable[[], int]
Sleeper = Callable[[float], None]


@dataclass(frozen=True, slots=True)
class CaptureConfig:
    serial: str
    stream_id: str
    logical_role: str
    scenario: str
    duration_s: float
    poll_hz: float
    output_dir: Path
    tracking_frame: str
    clutch_button_id: int | None
    clutch_input_id: str


@dataclass(frozen=True, slots=True)
class ReplayConfig:
    input_dir: Path
    summary_output: Path | None


def _finite_bounded_float(
    value: str,
    *,
    option: str,
    minimum: float,
    maximum: float,
) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{option} must be a number") from exc
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise argparse.ArgumentTypeError(
            f"{option} must be finite and in [{minimum:g}, {maximum:g}]"
        )
    return parsed


def _duration_s(value: str) -> float:
    return _finite_bounded_float(
        value,
        option="--duration-s",
        minimum=_MIN_DURATION_S,
        maximum=_MAX_DURATION_S,
    )


def _poll_hz(value: str) -> float:
    return _finite_bounded_float(
        value,
        option="--poll-hz",
        minimum=_MIN_POLL_HZ,
        maximum=_MAX_POLL_HZ,
    )


def _scenario(value: str) -> str:
    if _SCENARIO.fullmatch(value) is None:
        raise argparse.ArgumentTypeError(
            "--scenario must be a bounded identifier using letters, digits, '.', '_', '+', or '-'"
        )
    return value


def _clutch_button_id(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--clutch-button-id must be an integer") from exc
    if not 0 <= parsed < 64:
        raise argparse.ArgumentTypeError("--clutch-button-id must be in [0, 63]")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser(
        "inventory",
        help="Print the stable OpenVR device inventory as strict JSON.",
    )

    capture = subparsers.add_parser(
        "capture",
        help="Capture one bounded, serial-addressed tracker qualification run.",
    )
    capture.add_argument("--serial", required=True)
    capture.add_argument("--stream-id", required=True)
    capture.add_argument("--logical-role", required=True)
    capture.add_argument("--scenario", required=True, type=_scenario)
    capture.add_argument("--duration-s", default=10.0, type=_duration_s)
    capture.add_argument("--poll-hz", default=90.0, type=_poll_hz)
    capture.add_argument("--output-dir", required=True, type=Path)
    capture.add_argument("--tracking-frame", default="vive_tracking")
    capture.add_argument("--clutch-button-id", type=_clutch_button_id)
    capture.add_argument("--clutch-input-id", default="tracker_clutch")

    replay = subparsers.add_parser(
        "replay",
        help="Recompute metrics from canonical samples/events JSONL.",
    )
    replay.add_argument("--input-dir", required=True, type=Path)
    replay.add_argument("--summary-output", type=Path)
    return parser


def _inventory_mapping(item: TrackerInventoryItem) -> dict[str, object]:
    return {
        "serial": item.serial,
        "device_class": item.device_class,
        "model": item.model,
        "manufacturer": item.manufacturer,
        "connected": item.connected,
    }


def _strict_json(payload: object, *, pretty: bool) -> str:
    return json.dumps(
        payload,
        allow_nan=False,
        indent=2 if pretty else None,
        separators=None if pretty else (",", ":"),
        sort_keys=True,
    )


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{_strict_json(payload, pretty=True)}\n", encoding="utf-8")


def _write_raw_jsonl(
    path: Path,
    records: Sequence[Mapping[str, JsonValue]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = "".join(f"{_strict_json(record, pretty=False)}\n" for record in records)
    path.write_text(data, encoding="utf-8")


def _print_json(payload: object, *, output: IO[str]) -> None:
    output.write(f"{_strict_json(payload, pretty=True)}\n")


def _assert_capture_targets_available(output_dir: Path) -> None:
    conflicts = [name for name in _CAPTURE_ARTIFACTS if (output_dir / name).exists()]
    if conflicts:
        names = ", ".join(conflicts)
        raise FileExistsError(f"capture artifacts already exist in {output_dir}: {names}")


def _assert_summary_target_available(path: Path | None) -> None:
    if path is not None and path.exists():
        raise FileExistsError(f"summary output already exists: {path}")


def _summary_payload(
    *,
    samples: Sequence[TrackedRigidBodySample],
    event_count: int,
    scenario: str | None,
) -> dict[str, object]:
    metrics = compute_tracking_metrics(samples)
    valid_pose_observed = metrics.valid_sample_count > 0
    return {
        "schema": SUMMARY_SCHEMA,
        "status": "PASS" if valid_pose_observed else "NO_VALID_POSE",
        "scenario": scenario,
        "valid_pose_observed": valid_pose_observed,
        "event_count": event_count,
        "metrics": asdict(metrics),
    }


def run_inventory(
    *,
    adapter_factory: AdapterFactory = OpenVrTrackerAdapter,
    output: IO[str] = sys.stdout,
) -> int:
    adapter = adapter_factory(
        tracker_serial=None,
        stream_id="vive.inventory",
        logical_role="qualification",
        producer_instance="vive_qualification",
        transport_epoch=0,
        tracking_setup_revision="steamvr_standing_unqualified",
    )
    try:
        devices = adapter.inventory()
    finally:
        adapter.close()
    _print_json(
        {
            "schema": INVENTORY_SCHEMA,
            "devices": [_inventory_mapping(device) for device in devices],
        },
        output=output,
    )
    return 0


def run_capture(
    config: CaptureConfig,
    *,
    adapter_factory: AdapterFactory = OpenVrTrackerAdapter,
    clock_ns: ClockNs = time.monotonic_ns,
    process_clock_ns: ClockNs = time.process_time_ns,
    sleeper: Sleeper = time.sleep,
    output: IO[str] = sys.stdout,
) -> int:
    _assert_capture_targets_available(config.output_dir)
    adapter = adapter_factory(
        tracker_serial=config.serial,
        stream_id=config.stream_id,
        logical_role=config.logical_role,
        producer_instance="vive_qualification",
        transport_epoch=0,
        tracking_setup_revision="steamvr_standing_unqualified",
        tracking_frame=config.tracking_frame,
        clutch_button_id=config.clutch_button_id,
        clutch_input_id=config.clutch_input_id,
    )
    samples: list[TrackedRigidBodySample] = []
    events: list[ClutchEvent] = []
    raw_records: list[Mapping[str, JsonValue]] = []
    selected: TrackerInventoryItem
    duration_ns = round(config.duration_s * _NANOSECONDS_PER_SECOND)
    period_ns = max(1, round(_NANOSECONDS_PER_SECOND / config.poll_hz))
    sample_limit = math.ceil(config.duration_s * config.poll_hz) + 1

    try:
        selected = adapter.start()
        started_ns = clock_ns()
        process_started_ns = process_clock_ns()
        deadline_ns = started_ns + duration_ns
        next_poll_ns = started_ns
        while len(samples) < sample_limit:
            now_ns = clock_ns()
            if samples and now_ns >= deadline_ns:
                break
            if now_ns < next_poll_ns:
                sleeper((next_poll_ns - now_ns) / _NANOSECONDS_PER_SECOND)
            host_time_ns = clock_ns()
            if samples and host_time_ns >= deadline_ns:
                break

            poll = adapter.poll(host_time_ns=host_time_ns)
            samples.append(poll.sample)
            events.extend(poll.clutch_events)
            raw_record = adapter.last_raw_record
            if raw_record is None:
                raise RuntimeError("tracking adapter did not expose a raw record after poll")
            # Strict encoding now rejects non-JSON values and non-finite numbers
            # before any qualification artifact is written.
            _strict_json(raw_record, pretty=False)
            raw_records.append(dict(raw_record))

            next_poll_ns += period_ns
            if next_poll_ns <= host_time_ns:
                skipped_periods = ((host_time_ns - next_poll_ns) // period_ns) + 1
                next_poll_ns += skipped_periods * period_ns
        finished_ns = clock_ns()
        process_finished_ns = process_clock_ns()
    finally:
        adapter.close()

    wall_time_ns = max(0, finished_ns - started_ns)
    process_cpu_time_ns = max(0, process_finished_ns - process_started_ns)
    average_process_cpu_percent = (
        100.0 * process_cpu_time_ns / wall_time_ns if wall_time_ns else 0.0
    )
    capture_performance = {
        "wall_time_s": wall_time_ns / _NANOSECONDS_PER_SECOND,
        "process_cpu_time_s": process_cpu_time_ns / _NANOSECONDS_PER_SECOND,
        "average_process_cpu_percent": average_process_cpu_percent,
        "cpu_percent_normalization": "one_logical_core_equals_100_percent",
    }
    summary = _summary_payload(
        samples=samples,
        event_count=len(events),
        scenario=config.scenario,
    )
    summary["capture_performance"] = capture_performance
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "scenario": config.scenario,
        "capture_started_host_time_ns": started_ns,
        "capture_finished_host_time_ns": finished_ns,
        "requested_duration_s": config.duration_s,
        "requested_poll_hz": config.poll_hz,
        "stream_id": config.stream_id,
        "logical_role": config.logical_role,
        "tracking_frame": config.tracking_frame,
        "selected_tracker": _inventory_mapping(selected),
        "clutch_button_id": config.clutch_button_id,
        "clutch_input_id": config.clutch_input_id,
        "capture_performance": capture_performance,
        "artifacts": {
            "raw_openvr": "raw_openvr.jsonl",
            "samples": "samples.jsonl",
            "events": "events.jsonl",
            "summary": "summary.json",
        },
    }

    config.output_dir.mkdir(parents=True, exist_ok=True)
    _write_json(config.output_dir / "manifest.json", manifest)
    _write_raw_jsonl(config.output_dir / "raw_openvr.jsonl", raw_records)
    write_tracking_samples_jsonl(config.output_dir / "samples.jsonl", samples)
    write_clutch_events_jsonl(config.output_dir / "events.jsonl", events)
    _write_json(config.output_dir / "summary.json", summary)
    _print_json(summary, output=output)
    return 0 if bool(summary["valid_pose_observed"]) else NO_VALID_POSE_EXIT


def run_replay(
    config: ReplayConfig,
    *,
    output: IO[str] = sys.stdout,
) -> int:
    _assert_summary_target_available(config.summary_output)
    samples = read_tracking_samples_jsonl(config.input_dir / "samples.jsonl")
    events = read_clutch_events_jsonl(config.input_dir / "events.jsonl")
    summary = _summary_payload(samples=samples, event_count=len(events), scenario=None)
    if config.summary_output is not None:
        _write_json(config.summary_output, summary)
    _print_json(summary, output=output)
    return 0 if bool(summary["valid_pose_observed"]) else NO_VALID_POSE_EXIT


def _capture_config(args: argparse.Namespace) -> CaptureConfig:
    return CaptureConfig(
        serial=args.serial,
        stream_id=args.stream_id,
        logical_role=args.logical_role,
        scenario=args.scenario,
        duration_s=args.duration_s,
        poll_hz=args.poll_hz,
        output_dir=args.output_dir,
        tracking_frame=args.tracking_frame,
        clutch_button_id=args.clutch_button_id,
        clutch_input_id=args.clutch_input_id,
    )


def _replay_config(args: argparse.Namespace) -> ReplayConfig:
    return ReplayConfig(
        input_dir=args.input_dir,
        summary_output=args.summary_output,
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    adapter_factory: AdapterFactory = OpenVrTrackerAdapter,
    clock_ns: ClockNs = time.monotonic_ns,
    process_clock_ns: ClockNs = time.process_time_ns,
    sleeper: Sleeper = time.sleep,
    output: IO[str] = sys.stdout,
    error: IO[str] = sys.stderr,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "inventory":
            return run_inventory(adapter_factory=adapter_factory, output=output)
        if args.command == "capture":
            return run_capture(
                _capture_config(args),
                adapter_factory=adapter_factory,
                clock_ns=clock_ns,
                process_clock_ns=process_clock_ns,
                sleeper=sleeper,
                output=output,
            )
        if args.command == "replay":
            return run_replay(_replay_config(args), output=output)
        raise AssertionError(f"unsupported command: {args.command}")
    except Exception as exc:
        # OpenVR bindings expose runtime-specific exception classes.  Keep the
        # diagnostic boundary predictable without hiding SystemExit or signals.
        message = str(exc).strip() or type(exc).__name__
        error.write(f"error: {message}\n")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
