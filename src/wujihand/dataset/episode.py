"""Minimal, task-only annotation for one run-equals-episode dataset unit."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile
from typing import Final, cast

from wujihand.domain.recording import validate_run_id


EPISODE_ANNOTATION_SCHEMA: Final = "wujihand.dataset_episode_annotation.v1"


def _bounded_text(value: object, *, field: str, limit: int, allow_empty: bool) -> str:
    if not isinstance(value, str) or value != value.strip() or len(value) > limit:
        raise ValueError(f"{field} must be trimmed text of at most {limit} characters")
    if not allow_empty and not value:
        raise ValueError(f"{field} must not be empty")
    if any(ord(char) < 32 and char not in "\t" for char in value):
        raise ValueError(f"{field} must not contain control characters")
    return value


def _run_root(value: str | Path, *, expected_run_id: str | None = None) -> Path:
    raw = Path(value)
    if raw.is_symlink():
        raise ValueError("run root must not be a symbolic link")
    root = raw.resolve()
    if not root.is_dir():
        raise ValueError("run root must be a directory")
    run_id = validate_run_id(root.name)
    if expected_run_id is not None and run_id != validate_run_id(expected_run_id):
        raise ValueError("run root and expected run IDs differ")
    return root


@dataclass(frozen=True, slots=True)
class DatasetEpisodeAnnotation:
    run_id: str
    task: str
    operator_note: str

    def __post_init__(self) -> None:
        validate_run_id(self.run_id)
        _bounded_text(self.task, field="task", limit=512, allow_empty=False)
        _bounded_text(
            self.operator_note,
            field="operator_note",
            limit=1024,
            allow_empty=True,
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": EPISODE_ANNOTATION_SCHEMA,
            "run_id": self.run_id,
            "task": self.task,
            "operator_note": self.operator_note,
        }

    @classmethod
    def from_mapping(cls, value: object) -> DatasetEpisodeAnnotation:
        if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
            raise ValueError("episode annotation must be a string-keyed mapping")
        data = cast(dict[str, object], value)
        if frozenset(data) != {"schema", "run_id", "task", "operator_note"}:
            raise ValueError("episode annotation keys differ")
        if data.get("schema") != EPISODE_ANNOTATION_SCHEMA:
            raise ValueError("episode annotation schema differs")
        run_id = data.get("run_id")
        if not isinstance(run_id, str):
            raise ValueError("episode annotation run_id must be a string")
        return cls(
            run_id=run_id,
            task=_bounded_text(
                data.get("task"),
                field="task",
                limit=512,
                allow_empty=False,
            ),
            operator_note=_bounded_text(
                data.get("operator_note"),
                field="operator_note",
                limit=1024,
                allow_empty=True,
            ),
        )


def load_episode_annotation(
    run_root: str | Path,
    *,
    expected_run_id: str | None = None,
) -> DatasetEpisodeAnnotation:
    root = _run_root(run_root, expected_run_id=expected_run_id)
    path = root / "annotation.json"
    if path.is_symlink():
        raise ValueError("episode annotation must not be a symbolic link")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("episode annotation is not valid JSON") from exc
    annotation = DatasetEpisodeAnnotation.from_mapping(value)
    if annotation.run_id != root.name:
        raise ValueError("episode annotation and run root IDs differ")
    return annotation


def write_episode_annotation(
    run_root: str | Path,
    annotation: DatasetEpisodeAnnotation,
) -> Path:
    """Create an annotation atomically; a different existing annotation is immutable."""

    root = _run_root(run_root, expected_run_id=annotation.run_id)
    destination = root / "annotation.json"
    payload = (
        json.dumps(annotation.to_mapping(), sort_keys=True, indent=2, allow_nan=False) + "\n"
    ).encode("utf-8")
    if destination.exists() or destination.is_symlink():
        if destination.is_file() and not destination.is_symlink() and destination.read_bytes() == payload:
            return destination
        raise FileExistsError("a different or unsafe episode annotation already exists")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".annotation-",
        suffix=".tmp",
        dir=root,
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary_name, destination)
        except FileExistsError as exc:
            raise FileExistsError(
                "episode annotation appeared during publication"
            ) from exc
        os.unlink(temporary_name)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return destination


__all__ = [
    "EPISODE_ANNOTATION_SCHEMA",
    "DatasetEpisodeAnnotation",
    "load_episode_annotation",
    "write_episode_annotation",
]
