"""Atomic, deterministic writers for current 30 Hz alignment artifacts."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Final, cast

from wujihand.domain.recording import validate_run_id

from .alignment import (
    ALIGNMENT_SCHEMA,
    AlignmentFrame,
    ExactAlignment,
    _alignment_digest,
)


ALIGNMENT_ARTIFACT_SCHEMA: Final = "wujihand.dataset_alignment_artifact.v4"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_bytes(value: object) -> bytes:
    return (
        json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"
    ).encode("utf-8")


def _safe_run_root(run_root: str | Path, *, expected_run_id: str) -> Path:
    raw = Path(run_root).expanduser()
    if raw.is_symlink():
        raise ValueError("run root must not be a symbolic link")
    root = raw.resolve()
    if not root.is_dir() or root.name != expected_run_id:
        raise ValueError("run root must exist and its name must equal run_id")
    return root


def _verify_checksums(root: Path) -> bool:
    path = root / "checksums.sha256"
    if not path.is_file():
        return False
    seen: set[str] = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            return False
        relative_text = parts[1].lstrip("*")
        relative = Path(relative_text)
        if relative.is_absolute() or ".." in relative.parts or relative_text in seen:
            return False
        target = root / relative
        if target.is_symlink() or not target.is_file() or _sha256(target) != parts[0]:
            return False
        seen.add(relative_text)
    return bool(seen)


def _load_json(path: Path, *, field: str) -> object:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{field} is not valid JSON") from exc


def load_alignment_artifact(
    alignment_root: str | Path,
    *,
    expected_run_id: str | None = None,
) -> ExactAlignment:
    """Load and re-derive one current alignment artifact."""

    raw_root = Path(alignment_root)
    if raw_root.is_symlink():
        raise ValueError("alignment root must not be a symbolic link")
    root = raw_root.resolve()
    if not root.is_dir():
        raise ValueError("alignment root must be a directory")
    expected_files = {
        "checksums.sha256",
        "frames.jsonl",
        "gap_ticks.json",
        "manifest.json",
    }
    actual_files = {path.name for path in root.iterdir() if path.is_file()}
    if actual_files != expected_files or any(path.is_symlink() for path in root.iterdir()):
        raise ValueError("alignment artifact file inventory differs")
    if not _verify_checksums(root):
        raise ValueError("alignment artifact checksums differ")

    manifest_value = _load_json(root / "manifest.json", field="alignment manifest")
    manifest_keys = frozenset(
        {
            "schema",
            "run_id",
            "alignment_digest_sha256",
            "selection",
            "fps",
            "frame_count",
            "source_transition_count",
            "files",
        }
    )
    if not isinstance(manifest_value, dict) or frozenset(manifest_value) != manifest_keys:
        raise ValueError("alignment manifest schema or keys differ")
    manifest = cast(dict[str, object], manifest_value)
    if manifest.get("schema") != ALIGNMENT_ARTIFACT_SCHEMA:
        raise ValueError("alignment manifest schema or keys differ")
    run_id = validate_run_id(manifest.get("run_id"))
    if expected_run_id is not None and run_id != validate_run_id(expected_run_id):
        raise ValueError("alignment and expected run IDs differ")
    if manifest.get("selection") != "relative_all_control_index_no_interpolation_v1":
        raise ValueError("alignment selection contract differs")
    if manifest.get("fps") != 30:
        raise ValueError("alignment fps must be 30")
    frame_count = manifest.get("frame_count")
    source_count = manifest.get("source_transition_count")
    if type(frame_count) is not int or frame_count <= 0 or source_count != frame_count:
        raise ValueError("alignment frame/source counts must be equal and positive")
    digest = manifest.get("alignment_digest_sha256")
    if not isinstance(digest, str) or len(digest) != 64 or any(
        char not in "0123456789abcdef" for char in digest
    ):
        raise ValueError("alignment digest must be a lowercase SHA-256")

    material_names = ("frames.jsonl", "gap_ticks.json")
    files = manifest.get("files")
    if not isinstance(files, dict) or frozenset(files) != frozenset(material_names):
        raise ValueError("alignment manifest material file inventory differs")
    for name in material_names:
        entry = files[name]
        path = root / name
        if (
            not isinstance(entry, dict)
            or frozenset(entry) != {"sha256", "bytes"}
            or entry.get("sha256") != _sha256(path)
            or entry.get("bytes") != path.stat().st_size
        ):
            raise ValueError(f"alignment manifest file metadata differs: {name}")

    frames: list[AlignmentFrame] = []
    for line_number, line in enumerate(
        (root / "frames.jsonl").read_text(encoding="utf-8").splitlines(),
        start=1,
    ):
        if not line:
            raise ValueError("alignment frame table must not contain blank rows")
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"alignment frame line {line_number} is invalid") from exc
        frames.append(AlignmentFrame.from_mapping(value, field=f"frames[{line_number}]"))
    if len(frames) != frame_count:
        raise ValueError("alignment frame count differs")
    if tuple(frame.dataset_frame_index for frame in frames) != tuple(range(frame_count)):
        raise ValueError("alignment frame indices must be contiguous from zero")
    first = frames[0].source_control_index
    expected_source_indices = tuple(range(first, first + frame_count))
    if tuple(frame.source_control_index for frame in frames) != expected_source_indices:
        raise ValueError("alignment source indices must map one-to-one in order")
    if any(
        current.simulation_time_s <= previous.simulation_time_s
        for previous, current in zip(frames, frames[1:], strict=False)
    ):
        raise ValueError("alignment simulation time must be strictly increasing")

    gap_value = _load_json(root / "gap_ticks.json", field="alignment control gap sidecar")
    if not isinstance(gap_value, dict) or frozenset(gap_value) != {
        "schema",
        "run_id",
        "purpose",
        "gaps",
    }:
        raise ValueError("alignment control gap sidecar schema or keys differ")
    if (
        gap_value.get("schema") != ALIGNMENT_SCHEMA
        or gap_value.get("run_id") != run_id
        or gap_value.get("purpose") != "missed_control_period_gap_mask"
        or not isinstance(gap_value.get("gaps"), list)
    ):
        raise ValueError("alignment control gap sidecar identity differs")
    parsed_gaps: list[tuple[int, int]] = []
    for index, value in enumerate(cast(list[object], gap_value["gaps"])):
        if not isinstance(value, dict) or frozenset(value) != {
            "control_index",
            "missing_control_periods_before",
        }:
            raise ValueError(f"alignment control gap {index} differs")
        control_index = value.get("control_index")
        missing = value.get("missing_control_periods_before")
        if (
            type(control_index) is not int
            or type(missing) is not int
            or control_index not in expected_source_indices
            or missing <= 0
        ):
            raise ValueError(f"alignment control gap {index} values differ")
        parsed_gaps.append((control_index, missing))
    gap_ticks = tuple(parsed_gaps)
    if tuple(sorted(gap_ticks)) != gap_ticks or len(dict(gap_ticks)) != len(gap_ticks):
        raise ValueError("alignment control gaps must be unique and ordered")

    missing_by_index = dict(gap_ticks)
    expected_segment = 0
    for frame_index, frame in enumerate(frames):
        missing_before = missing_by_index.get(frame.source_control_index, 0)
        if frame_index > 0 and missing_before > 0:
            expected_segment += 1
        expected_transition_valid = (
            frame_index + 1 < len(frames)
            and missing_by_index.get(frames[frame_index + 1].source_control_index, 0) == 0
        )
        if (
            frame.missing_control_periods_before != missing_before
            or frame.temporal_continuity != (missing_before == 0)
            or frame.temporal_segment_index != expected_segment
            or frame.gap_before_row != (missing_before > 0)
            or frame.transition_valid != expected_transition_valid
        ):
            raise ValueError("alignment frame gap mask closure differs")

    payload: dict[str, object] = {
        "schema": ALIGNMENT_SCHEMA,
        "run_id": run_id,
        "source_first_control_index": first,
        "source_last_control_index": expected_source_indices[-1],
        "source_transition_count": source_count,
        "selection": "relative_all_control_index_no_interpolation_v1",
        "fps": 30,
        "frames": [frame.to_mapping() for frame in frames],
        "gap_ticks": [
            {"control_index": index, "missing_control_periods_before": missing}
            for index, missing in gap_ticks
        ],
    }
    derived_digest = _alignment_digest(payload)
    if digest != derived_digest:
        raise ValueError("alignment digest differs from canonical rows")
    return ExactAlignment(
        run_id=run_id,
        source_first_control_index=first,
        source_last_control_index=expected_source_indices[-1],
        source_transition_count=source_count,
        frames=tuple(frames),
        digest_sha256=digest,
        gap_ticks=gap_ticks,
    )


def write_alignment_artifact(run_root: str | Path, alignment: ExactAlignment) -> Path:
    """Publish canonical 30 Hz rows atomically without touching raw recordings."""

    root = _safe_run_root(run_root, expected_run_id=alignment.run_id)
    derived = root / "derived"
    derived.mkdir(exist_ok=True)
    if derived.is_symlink():
        raise ValueError("derived root must not be a symbolic link")
    destination = derived / "alignment"
    if destination.exists():
        manifest_path = destination / "manifest.json"
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise FileExistsError("existing alignment artifact is not valid") from exc
        if (
            isinstance(manifest, dict)
            and manifest.get("alignment_digest_sha256") == alignment.digest_sha256
            and _verify_checksums(destination)
        ):
            return destination
        raise FileExistsError("a different or incomplete alignment artifact already exists")

    temporary = Path(tempfile.mkdtemp(prefix=".alignment-", dir=derived))
    try:
        frames_path = temporary / "frames.jsonl"
        frames_path.write_bytes(
            b"".join(
                json.dumps(
                    frame.to_mapping(),
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
                + b"\n"
                for frame in alignment.frames
            )
        )
        gap_path = temporary / "gap_ticks.json"
        gap_path.write_bytes(
            _json_bytes(
                {
                    "schema": ALIGNMENT_SCHEMA,
                    "run_id": alignment.run_id,
                    "purpose": "missed_control_period_gap_mask",
                    "gaps": [
                        {
                            "control_index": index,
                            "missing_control_periods_before": missing,
                        }
                        for index, missing in alignment.gap_ticks
                    ],
                }
            )
        )
        material = (frames_path, gap_path)
        manifest = {
            "schema": ALIGNMENT_ARTIFACT_SCHEMA,
            "run_id": alignment.run_id,
            "alignment_digest_sha256": alignment.digest_sha256,
            "selection": "relative_all_control_index_no_interpolation_v1",
            "fps": 30,
            "frame_count": len(alignment.frames),
            "source_transition_count": alignment.source_transition_count,
            "files": {
                item.name: {"sha256": _sha256(item), "bytes": item.stat().st_size}
                for item in material
            },
        }
        manifest_path = temporary / "manifest.json"
        manifest_path.write_bytes(_json_bytes(manifest))
        checksummed = (*material, manifest_path)
        (temporary / "checksums.sha256").write_text(
            "".join(f"{_sha256(item)}  {item.name}\n" for item in checksummed),
            encoding="utf-8",
        )
        os.replace(temporary, destination)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return destination


__all__ = [
    "ALIGNMENT_ARTIFACT_SCHEMA",
    "load_alignment_artifact",
    "write_alignment_artifact",
]
