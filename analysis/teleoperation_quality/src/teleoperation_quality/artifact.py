"""Fail-closed validation of one immutable input run."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

MANIFEST_SCHEMA = "wujihand.teleoperation_run_manifest.v1"
RECEIPT_SCHEMA = "wujihand.teleoperation_run_receipt.v1"
RECORDER_SCHEMA = "wujihand.rosbag2_recorder.v1"


@dataclass(frozen=True, slots=True)
class RunArtifact:
    root: Path
    run_id: str
    manifest: dict[str, Any]
    receipt: dict[str, Any]
    recorder: dict[str, Any]
    rosbag_metadata: dict[str, Any]
    input_checksums: dict[str, str]
    expected_topics: tuple[str, ...]
    mcap_paths: tuple[Path, ...]


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read valid JSON from {path}") from exc
    if not isinstance(value, dict):
        raise TypeError(f"{path} must contain one JSON object")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_checksums(root: Path) -> dict[str, str]:
    path = root / "checksums.sha256"
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise ValueError(f"cannot read {path}") from exc
    checksums: dict[str, str] = {}
    for line_number, line in enumerate(lines, start=1):
        if not line:
            continue
        parts = line.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise ValueError(f"invalid checksum line {line_number}")
        relative_text = parts[1].lstrip("*")
        relative = Path(relative_text)
        if relative.is_absolute() or ".." in relative.parts or relative_text in checksums:
            raise ValueError(f"unsafe or duplicate checksum path: {relative_text}")
        target = root / relative
        if not target.is_file():
            raise ValueError(f"checksummed file is missing: {relative_text}")
        expected = parts[0].lower()
        if _sha256(target) != expected:
            raise ValueError(f"checksum mismatch: {relative_text}")
        checksums[relative_text] = expected
    if not checksums:
        raise ValueError("checksums.sha256 must not be empty")
    return checksums


def _schema(value: dict[str, Any], expected: str, *, file: str) -> None:
    if value.get("schema") != expected:
        raise ValueError(f"{file} schema must be {expected!r}")


def _load_rosbag_metadata(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read valid YAML from {path}") from exc
    if not isinstance(value, dict):
        raise TypeError("rosbag2 metadata must contain one mapping")
    information = value.get("rosbag2_bagfile_information")
    if not isinstance(information, dict):
        raise TypeError("rosbag2 metadata is missing bagfile information")
    return information


def load_run_artifact(run_root: str | Path) -> RunArtifact:
    """Validate and return one complete run without modifying it."""

    root = Path(run_root).expanduser().resolve()
    if not root.is_dir():
        raise ValueError(f"run root is not a directory: {root}")
    manifest = _load_json(root / "manifest.json")
    receipt = _load_json(root / "receipt.json")
    recorder = _load_json(root / "recorder.json")
    _schema(manifest, MANIFEST_SCHEMA, file="manifest.json")
    _schema(receipt, RECEIPT_SCHEMA, file="receipt.json")
    _schema(recorder, RECORDER_SCHEMA, file="recorder.json")

    run_ids = {manifest.get("run_id"), receipt.get("run_id"), recorder.get("run_id")}
    if len(run_ids) != 1 or not all(isinstance(value, str) and value for value in run_ids):
        raise ValueError("manifest, receipt and recorder run_id values must match")
    run_id = str(next(iter(run_ids)))
    if root.name != run_id:
        raise ValueError("run directory name must equal run_id")
    if receipt.get("state") != "complete":
        raise ValueError("offline quality analysis requires receipt.state == complete")
    if receipt.get("consumer_state") != "consumer_completed":
        raise ValueError("complete run must contain a terminal consumer receipt")
    if receipt.get("recorder_exit_code") != 0 or recorder.get("exit_code") != 0:
        raise ValueError("complete run must have zero recorder exit codes")
    if recorder.get("consumer_terminal_observed") is not True:
        raise ValueError("recorder must have observed the terminal consumer receipt")
    if receipt.get("recording_finalized") is not True:
        raise ValueError("complete run must be finalized")
    if recorder.get("state") != "exited" or recorder.get("storage") != "mcap":
        raise ValueError("recorder must have exited using MCAP storage")

    checksums = _parse_checksums(root)
    required = {
        "manifest.json",
        "receipt.json",
        "recorder.json",
        "raw/rosbag2/metadata.yaml",
    }
    missing = sorted(required - checksums.keys())
    if missing:
        raise ValueError(f"required files are absent from checksums.sha256: {missing}")
    mcap_relative = sorted(
        path for path in checksums if path.startswith("raw/rosbag2/") and path.endswith(".mcap")
    )
    if not mcap_relative:
        raise ValueError("checksums.sha256 contains no MCAP segment")
    mcap_paths = tuple(root / path for path in mcap_relative)
    if any(path.stat().st_size <= 0 for path in mcap_paths):
        raise ValueError("MCAP segments must be non-empty")
    material_paths = {
        path.relative_to(root).as_posix()
        for path in (
            root / "manifest.json",
            root / "receipt.json",
            root / "recorder.json",
            *(path for path in (root / "raw").rglob("*") if path.is_file()),
        )
    }
    unchecksummed = sorted(material_paths - checksums.keys())
    if unchecksummed:
        raise ValueError(f"run contains unchecksummed material files: {unchecksummed}")

    inventory = manifest.get("recording_inventory")
    topics_value = inventory.get("topics") if isinstance(inventory, dict) else None
    if not isinstance(topics_value, list) or not topics_value:
        raise ValueError("manifest recording_inventory.topics must be a non-empty list")
    if not all(isinstance(topic, str) and topic.startswith("/") for topic in topics_value):
        raise ValueError("manifest recording topics must be absolute ROS names")
    expected_topics = tuple(str(topic) for topic in topics_value)
    if len(set(expected_topics)) != len(expected_topics):
        raise ValueError("manifest recording topics must be unique")
    recorder_topics = recorder.get("topics")
    if recorder_topics != list(expected_topics):
        raise ValueError("recorder topic allowlist must equal the manifest inventory")

    metadata = _load_rosbag_metadata(root / "raw" / "rosbag2" / "metadata.yaml")
    if metadata.get("storage_identifier") != "mcap":
        raise ValueError("rosbag2 metadata storage_identifier must be mcap")
    relative_files = metadata.get("relative_file_paths")
    expected_mcap_names = [path.name for path in mcap_paths]
    if relative_files != expected_mcap_names:
        raise ValueError("rosbag2 metadata MCAP inventory does not match checksums")
    topic_entries = metadata.get("topics_with_message_count")
    if not isinstance(topic_entries, list):
        raise TypeError("rosbag2 metadata topic inventory must be a list")
    metadata_topics = []
    metadata_message_count = 0
    for entry in topic_entries:
        if not isinstance(entry, dict) or not isinstance(entry.get("topic_metadata"), dict):
            raise TypeError("rosbag2 metadata contains an invalid topic entry")
        metadata_topics.append(str(entry["topic_metadata"].get("name")))
        metadata_message_count += int(entry.get("message_count", -1))
    if set(metadata_topics) != set(expected_topics) or len(metadata_topics) != len(expected_topics):
        raise ValueError("rosbag2 metadata topics do not match the frozen allowlist")
    if metadata_message_count != int(metadata.get("message_count", -1)):
        raise ValueError("rosbag2 metadata per-topic counts do not sum to message_count")
    if int(receipt.get("raw_mcap_segments", -1)) != len(mcap_paths):
        raise ValueError("receipt raw_mcap_segments does not match checksummed MCAP files")

    return RunArtifact(
        root=root,
        run_id=run_id,
        manifest=manifest,
        receipt=receipt,
        recorder=recorder,
        rosbag_metadata=metadata,
        input_checksums=checksums,
        expected_topics=expected_topics,
        mcap_paths=mcap_paths,
    )


__all__ = ["RunArtifact", "load_run_artifact"]
