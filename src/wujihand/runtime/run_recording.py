"""Run-unique artifact lifecycle for passive ROS teleoperation recording."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import uuid

from wujihand.domain import (
    RUN_MANIFEST_SCHEMA,
    RUN_RECEIPT_SCHEMA,
    RunRecordingState,
    validate_run_id,
)


class SignalStopRequest:
    """Latch the first process signal without interrupting Python cleanup."""

    def __init__(self) -> None:
        self.requested_signal: int | None = None

    @property
    def requested(self) -> bool:
        return self.requested_signal is not None

    def __call__(self, signum: int, frame: object) -> None:
        del frame
        if self.requested_signal is None:
            self.requested_signal = signum


def new_run_id(*, prefix: str = "ros2-sim") -> str:
    prefix = validate_run_id(prefix, field="prefix")
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return validate_run_id(
        f"{prefix}-{timestamp}-{uuid.uuid4().hex[:8]}",
        field="run_id",
    )


def run_root(
    project_root: str | Path,
    report_root: str | Path,
    run_id: str,
) -> Path:
    identifier = validate_run_id(run_id)
    root = Path(project_root).resolve()
    report = (root / report_root).resolve()
    try:
        report.relative_to(root)
    except ValueError as exc:
        raise ValueError("report_root must stay inside project_root") from exc
    return report / identifier


def write_manifest(
    destination: str | Path,
    *,
    run_id: str,
    payload: dict[str, object],
) -> Path:
    identifier = validate_run_id(run_id)
    root = Path(destination)
    root.mkdir(parents=True, exist_ok=True)
    path = root / "manifest.json"
    if path.exists():
        raise FileExistsError(f"run manifest already exists: {path}")
    value = {
        **payload,
        "schema": RUN_MANIFEST_SCHEMA,
        "run_id": identifier,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    _write_json_atomic(path, value)
    return path


def write_consumer_receipt(
    destination: str | Path,
    *,
    run_id: str,
    state: RunRecordingState,
    payload: dict[str, object],
) -> Path:
    if state not in {
        RunRecordingState.CONSUMER_COMPLETED,
        RunRecordingState.INCOMPLETE,
    }:
        raise ValueError("consumer receipt state must be completed/incomplete")
    identifier = validate_run_id(run_id)
    root = Path(destination)
    root.mkdir(parents=True, exist_ok=True)
    path = root / "receipt.json"
    existing: dict[str, object] = {}
    if path.is_file():
        loaded = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError("receipt must contain a JSON object")
        existing = loaded
        if existing.get("run_id") != identifier:
            raise ValueError("existing receipt run_id differs")
        if existing.get("recording_finalized") is True:
            return path
    consumer_value = {
        **payload,
        "schema": RUN_RECEIPT_SCHEMA,
        "run_id": identifier,
        "state": state.value,
        "recording_finalized": False,
        "consumer_state": state.value,
        "consumer_closed_utc": datetime.now(timezone.utc).isoformat(),
    }
    value = {**existing, **consumer_value}
    _write_json_atomic(path, value)
    return path


def consumer_receipt_is_terminal(
    destination: str | Path,
    *,
    run_id: str,
) -> bool:
    """Return whether the consumer atomically published its terminal state."""

    identifier = validate_run_id(run_id)
    path = Path(destination) / "receipt.json"
    if not path.is_file():
        return False
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return False
    return bool(
        isinstance(value, dict)
        and value.get("schema") == RUN_RECEIPT_SCHEMA
        and value.get("run_id") == identifier
        and value.get("consumer_state")
        in {
            RunRecordingState.CONSUMER_COMPLETED.value,
            RunRecordingState.INCOMPLETE.value,
        }
    )


def finalize_rosbag_recording(
    destination: str | Path,
    *,
    run_id: str,
    recorder_exit_code: int,
) -> dict[str, object]:
    """Close the immutable raw artifact after rosbag2 has exited."""

    identifier = validate_run_id(run_id)
    root = Path(destination)
    receipt_path = root / "receipt.json"
    receipt: dict[str, object]
    if receipt_path.is_file():
        loaded = json.loads(receipt_path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError("receipt must contain a JSON object")
        receipt = loaded
    else:
        receipt = {
            "schema": RUN_RECEIPT_SCHEMA,
            "run_id": identifier,
            "state": RunRecordingState.INCOMPLETE.value,
            "recording_finalized": False,
            "reason": "consumer_receipt_missing",
        }
    if receipt.get("run_id") != identifier:
        raise ValueError("receipt run_id differs from requested run")
    if receipt.get("recording_finalized") is True:
        return receipt

    raw_root = root / "raw" / "rosbag2"
    raw_files = (
        tuple(sorted(path for path in raw_root.rglob("*") if path.is_file()))
        if raw_root.is_dir()
        else ()
    )
    manifest_path = root / "manifest.json"
    recorder_metadata_path = root / "recorder.json"
    manifest_present = manifest_path.is_file()
    recorder_metadata_present = recorder_metadata_path.is_file()
    manifest_valid = _json_matches_run(
        manifest_path,
        schema=RUN_MANIFEST_SCHEMA,
        run_id=identifier,
    )
    recorder_metadata_valid = _json_matches_run(
        recorder_metadata_path,
        schema="wujihand.rosbag2_recorder.v1",
        run_id=identifier,
    )
    metadata_present = any(
        path.name == "metadata.yaml" and path.stat().st_size > 0 for path in raw_files
    )
    mcap_present = any(path.suffix == ".mcap" and path.stat().st_size > 0 for path in raw_files)
    raw_total_bytes = sum(path.stat().st_size for path in raw_files)
    raw_mcap_segments = sum(
        path.suffix == ".mcap" and path.stat().st_size > 0 for path in raw_files
    )
    consumer_completed = receipt.get("consumer_state") == RunRecordingState.CONSUMER_COMPLETED.value
    receipt_schema_valid = receipt.get("schema") == RUN_RECEIPT_SCHEMA
    complete = all(
        (
            recorder_exit_code == 0,
            consumer_completed,
            receipt_schema_valid,
            manifest_valid,
            recorder_metadata_valid,
            metadata_present,
            mcap_present,
        )
    )
    receipt.update(
        {
            "state": (
                RunRecordingState.COMPLETE.value if complete else RunRecordingState.INCOMPLETE.value
            ),
            "recording_finalized": True,
            "recorder_exit_code": recorder_exit_code,
            "finalized_utc": datetime.now(timezone.utc).isoformat(),
            "manifest_present": manifest_present,
            "manifest_valid": manifest_valid,
            "receipt_schema_valid": receipt_schema_valid,
            "recorder_metadata_present": recorder_metadata_present,
            "recorder_metadata_valid": recorder_metadata_valid,
            "raw_metadata_present": metadata_present,
            "raw_mcap_present": mcap_present,
            "raw_total_bytes": raw_total_bytes,
            "raw_mcap_segments": raw_mcap_segments,
            "raw_files": [path.relative_to(root).as_posix() for path in raw_files],
        }
    )
    if not complete and "reason" not in receipt:
        receipt["reason"] = "recording_closure_incomplete"
    _write_json_atomic(receipt_path, receipt)

    try:
        material = tuple(
            path
            for path in (
                root / "manifest.json",
                root / "recorder.json",
                receipt_path,
                *raw_files,
            )
            if path.is_file()
        )
        checksum_lines = [
            f"{_sha256_file(path)}  {path.relative_to(root).as_posix()}" for path in material
        ]
        _write_text_atomic(
            root / "checksums.sha256",
            "\n".join(checksum_lines) + ("\n" if checksum_lines else ""),
        )
    except OSError as exc:
        receipt["state"] = RunRecordingState.INCOMPLETE.value
        receipt["reason"] = "checksum_closure_failed"
        receipt["checksum_error"] = type(exc).__name__
        _write_json_atomic(receipt_path, receipt)
        raise
    return receipt


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_matches_run(path: Path, *, schema: str, run_id: str) -> bool:
    if not path.is_file():
        return False
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return False
    return (
        isinstance(value, dict) and value.get("schema") == schema and value.get("run_id") == run_id
    )


def _write_json_atomic(path: Path, value: object) -> None:
    _write_text_atomic(
        path,
        json.dumps(
            value,
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
    )


def _write_text_atomic(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


__all__ = [
    "SignalStopRequest",
    "consumer_receipt_is_terminal",
    "finalize_rosbag_recording",
    "new_run_id",
    "run_root",
    "write_consumer_receipt",
    "write_manifest",
]
