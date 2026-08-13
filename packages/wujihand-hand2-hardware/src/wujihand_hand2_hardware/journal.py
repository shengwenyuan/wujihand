from __future__ import annotations

import json
import os
import tempfile
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path

from .types import (
    CommunicationSample,
    JsonValue,
    MotionReport,
    QualificationReport,
    SafetyState,
    TemperatureSample,
)


def _dump(path: Path, payload: JsonValue) -> None:
    with tempfile.NamedTemporaryFile(
        "w", dir=path.parent, delete=False, encoding="utf-8"
    ) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        temporary = Path(stream.name)
    os.replace(temporary, path)


class RunArtifacts:
    def __init__(self, output_dir: Path) -> None:
        output_dir.mkdir(parents=True, exist_ok=False)
        self.output_dir = output_dir
        self._events = (output_dir / "events.jsonl").open("a", encoding="utf-8")
        self._temperatures = (output_dir / "temperature.jsonl").open("a", encoding="utf-8")
        self._communications = (output_dir / "communication.jsonl").open("a", encoding="utf-8")

    def event(
        self,
        kind: str,
        state: SafetyState,
        payload: dict[str, JsonValue] | None = None,
    ) -> None:
        item: dict[str, JsonValue] = {
            "event_kind": kind,
            "host_monotonic_ns": time.monotonic_ns(),
            "host_wall_time": datetime.now(UTC).isoformat(),
            "safety_state": state.value,
            "payload_schema_revision": "hand2_hardware_event_v1",
            "payload": payload or {},
        }
        self._events.write(json.dumps(item, sort_keys=True) + "\n")
        self._events.flush()

    def temperature(self, sample: TemperatureSample) -> None:
        self._temperatures.write(json.dumps(sample.as_json(), sort_keys=True) + "\n")
        self._temperatures.flush()

    def communication(self, sample: CommunicationSample) -> None:
        self._communications.write(json.dumps(sample.as_json(), sort_keys=True) + "\n")
        self._communications.flush()

    def report(self, report: QualificationReport) -> None:
        _dump(self.output_dir / "qualification.json", report.as_json())

    def manifest(self, payload: dict[str, JsonValue]) -> None:
        _dump(self.output_dir / "manifest.json", payload)

    def close(self) -> None:
        self._events.close()
        self._temperatures.close()
        self._communications.close()


class MotionArtifacts:
    def __init__(self, output_dir: Path) -> None:
        output_dir.mkdir(parents=True, exist_ok=False)
        self.output_dir = output_dir
        self._events = (output_dir / "events.jsonl").open("a", encoding="utf-8")
        self._commands = (output_dir / "commands.jsonl").open("a", encoding="utf-8")
        self._states = (output_dir / "motion_states.jsonl").open("a", encoding="utf-8")

    def manifest(self, payload: dict[str, JsonValue]) -> None:
        _dump(self.output_dir / "manifest.json", payload)

    def event(
        self,
        kind: str,
        state: SafetyState,
        *,
        serial: str,
        side: str,
        correlation_id: str | None = None,
        payload: dict[str, JsonValue] | None = None,
        device_sequence: int | None = None,
        device_timestamp_us: int | None = None,
    ) -> None:
        item: dict[str, JsonValue] = {
            "event_id": str(uuid.uuid4()),
            "event_kind": kind,
            "device_serial": serial,
            "side": side,
            "host_monotonic_ns": time.monotonic_ns(),
            "host_wall_time": datetime.now(UTC).isoformat(),
            "device_sequence": device_sequence,
            "device_timestamp_us": device_timestamp_us,
            "command_correlation_id": correlation_id,
            "safety_state": state.value,
            "payload_schema_revision": "hand2_hardware_event_v1",
            "payload": payload or {},
        }
        self._events.write(json.dumps(item, sort_keys=True) + "\n")
        self._events.flush()

    def command(
        self,
        kind: str,
        state: SafetyState,
        *,
        serial: str,
        side: str,
        correlation_id: str,
        sequence: int,
        positions_rad: tuple[float, ...],
        details: dict[str, JsonValue],
        error: str | None = None,
    ) -> None:
        item: dict[str, JsonValue] = {
            "event_kind": kind,
            "device_serial": serial,
            "side": side,
            "host_monotonic_ns": time.monotonic_ns(),
            "host_wall_time": datetime.now(UTC).isoformat(),
            "command_correlation_id": correlation_id,
            "command_sequence": sequence,
            "safety_state": state.value,
            "payload_schema_revision": "hand2_joint_command_event_v1",
            "positions_rad": list(positions_rad),
            "velocity_rad_s": [0.0] * 20,
            "effort_a": [0.0] * 20,
            "details": details,
            "error": error,
        }
        self._commands.write(json.dumps(item, sort_keys=True) + "\n")
        self._commands.flush()

    def state(
        self,
        *,
        serial: str,
        side: str,
        correlation_id: str,
        sequence: int,
        device_timestamp_us: int,
        positions_rad: tuple[float, ...],
        details: dict[str, JsonValue],
    ) -> None:
        item: dict[str, JsonValue] = {
            "event_kind": "HARDWARE_STATE_OBSERVED",
            "device_serial": serial,
            "side": side,
            "host_monotonic_ns": time.monotonic_ns(),
            "host_wall_time": datetime.now(UTC).isoformat(),
            "device_sequence": sequence,
            "device_timestamp_us": device_timestamp_us,
            "command_correlation_id": correlation_id,
            "payload_schema_revision": "hand2_motion_state_v1",
            "positions_rad": list(positions_rad),
            "details": details,
        }
        self._states.write(json.dumps(item, sort_keys=True) + "\n")
        self._states.flush()

    def report(self, report: MotionReport) -> None:
        _dump(self.output_dir / "qualification.json", report.as_json())

    def close(self) -> None:
        self._events.close()
        self._commands.close()
        self._states.close()
