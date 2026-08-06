"""Checksum-closed episode bundle manifest over immutable raw and derived artifacts."""

from __future__ import annotations

from dataclasses import dataclass
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Final, Iterator
from contextlib import contextmanager

from wujihand.domain.recording import validate_recording_token, validate_run_id

from .release_artifact import load_release_decision_artifact


EPISODE_BUNDLE_SCHEMA: Final = "wujihand.dataset_episode_bundle.v1"
_REQUIRED_FILES: Final = (
    "manifest.json",
    "recorder.json",
    "receipt.json",
    "checksums.sha256",
    "annotation.json",
)
_REQUIRED_DIRECTORIES: Final = (
    "raw/rosbag2",
    "derived/normalized",
    "derived/release",
    "derived/alignment",
    "derived/vision",
)


def _digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def _tree(root: Path) -> tuple[str, int, int]:
    if not root.is_dir() or root.is_symlink():
        raise ValueError(f"bundle directory is missing or unsafe: {root}")
    rows: list[tuple[str, str, int]] = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError("episode bundle refuses symbolic links")
        if path.is_file():
            rows.append(
                (
                    path.relative_to(root).as_posix(),
                    _digest(path),
                    path.stat().st_size,
                )
            )
    if not rows:
        raise ValueError(f"bundle directory contains no files: {root}")
    encoded = json.dumps(rows, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest(), len(rows), sum(row[2] for row in rows)


def _run_root(value: str | Path) -> Path:
    raw = Path(value)
    if raw.is_symlink():
        raise ValueError("run root must not be a symbolic link")
    root = raw.resolve()
    if not root.is_dir():
        raise ValueError("run root must be a directory")
    validate_run_id(root.name)
    return root


@contextmanager
def _locked(root: Path) -> Iterator[None]:
    with (root / ".dataset-bundle.lock").open("a+b") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


@dataclass(frozen=True, slots=True)
class EpisodeBundleArtifact:
    path: Path
    manifest: dict[str, object]
    manifest_sha256: str


def _dependencies(root: Path) -> tuple[dict[str, object], ...]:
    result: list[dict[str, object]] = []
    for relative in _REQUIRED_FILES:
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"bundle file is missing or unsafe: {relative}")
        result.append(
            {
                "relative_path": relative,
                "kind": "file",
                "sha256": _digest(path),
                "file_count": 1,
                "byte_count": path.stat().st_size,
            }
        )
    for relative in _REQUIRED_DIRECTORIES:
        digest, count, size = _tree(root / relative)
        result.append(
            {
                "relative_path": relative,
                "kind": "directory_tree",
                "sha256": digest,
                "file_count": count,
                "byte_count": size,
            }
        )
    quality = root / "derived" / "quality"
    if quality.exists() or quality.is_symlink():
        digest, count, size = _tree(quality)
        result.append(
            {
                "relative_path": "derived/quality",
                "kind": "directory_tree",
                "sha256": digest,
                "file_count": count,
                "byte_count": size,
            }
        )
    return tuple(result)


def write_episode_bundle(
    run_root: str | Path,
    *,
    collection_id: str,
    dataset_profile_id: str,
    dataset_profile_sha256: str,
    deployment_hash: str,
    session_hash: str,
    assembly_hash: str,
    workcell_hash: str,
) -> EpisodeBundleArtifact:
    root = _run_root(run_root)
    collection = validate_recording_token(collection_id, field="collection_id")
    for field, value in (
        ("dataset_profile_sha256", dataset_profile_sha256),
        ("deployment_hash", deployment_hash),
        ("session_hash", session_hash),
        ("assembly_hash", assembly_hash),
        ("workcell_hash", workcell_hash),
    ):
        if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
            raise ValueError(f"{field} must be a lowercase SHA-256")
    release = load_release_decision_artifact(
        root / "derived" / "release",
        expected_run_id=root.name,
    )
    if not release.decision.passed:
        raise ValueError("a rejected episode cannot publish a bundle manifest")
    with _locked(root):
        dependencies = _dependencies(root)
        mapping: dict[str, object] = {
            "schema": EPISODE_BUNDLE_SCHEMA,
            "collection_id": collection,
            "run_id": root.name,
            "episode_id": root.name,
            "dataset_profile_id": validate_recording_token(
                dataset_profile_id,
                field="dataset_profile_id",
            ),
            "dataset_profile_sha256": dataset_profile_sha256,
            "deployment_hash": deployment_hash,
            "session_hash": session_hash,
            "assembly_hash": assembly_hash,
            "workcell_hash": workcell_hash,
            "release_decision_sha256": release.decision_sha256,
            "dependencies": list(dependencies),
        }
        payload = (
            json.dumps(mapping, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
        ).encode()
        destination_root = root / "derived" / "bundle"
        destination = destination_root / "manifest.json"
        if destination_root.is_symlink():
            raise ValueError("bundle root must not be a symbolic link")
        destination_root.mkdir(exist_ok=True)
        if destination.exists() or destination.is_symlink():
            if (
                destination.is_file()
                and not destination.is_symlink()
                and destination.read_bytes() == payload
            ):
                return EpisodeBundleArtifact(
                    path=destination,
                    manifest=mapping,
                    manifest_sha256=hashlib.sha256(payload).hexdigest(),
                )
            raise FileExistsError("a different or stale bundle manifest already exists")
        descriptor, temporary = tempfile.mkstemp(prefix=".manifest-", dir=destination_root)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, destination)
            os.unlink(temporary)
        except BaseException:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass
            raise
        return EpisodeBundleArtifact(
            path=destination,
            manifest=mapping,
            manifest_sha256=hashlib.sha256(payload).hexdigest(),
        )


def validate_episode_bundle(run_root: str | Path) -> EpisodeBundleArtifact:
    root = _run_root(run_root)
    path = root / "derived" / "bundle" / "manifest.json"
    if not path.is_file() or path.is_symlink():
        raise ValueError("episode bundle manifest is missing or unsafe")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("episode bundle manifest is invalid JSON") from exc
    expected_keys = {
        "schema",
        "collection_id",
        "run_id",
        "episode_id",
        "dataset_profile_id",
        "dataset_profile_sha256",
        "deployment_hash",
        "session_hash",
        "assembly_hash",
        "workcell_hash",
        "release_decision_sha256",
        "dependencies",
    }
    if (
        not isinstance(value, dict)
        or set(value) != expected_keys
        or value.get("schema") != EPISODE_BUNDLE_SCHEMA
    ):
        raise ValueError("episode bundle schema differs")
    if value.get("run_id") != root.name or value.get("episode_id") != root.name:
        raise ValueError("episode bundle identity differs")
    validate_recording_token(value.get("collection_id"), field="collection_id")
    validate_recording_token(value.get("dataset_profile_id"), field="dataset_profile_id")
    for field in (
        "dataset_profile_sha256",
        "deployment_hash",
        "session_hash",
        "assembly_hash",
        "workcell_hash",
        "release_decision_sha256",
    ):
        candidate = value.get(field)
        if not isinstance(candidate, str) or len(candidate) != 64 or any(
            char not in "0123456789abcdef" for char in candidate
        ):
            raise ValueError(f"episode bundle {field} differs")
    release = load_release_decision_artifact(
        root / "derived" / "release",
        expected_run_id=root.name,
    )
    if not release.decision.passed or value["release_decision_sha256"] != (
        release.decision_sha256
    ):
        raise ValueError("episode bundle release decision is stale")
    if value.get("dependencies") != list(_dependencies(root)):
        raise ValueError("episode bundle dependency closure is stale")
    payload = path.read_bytes()
    return EpisodeBundleArtifact(
        path=path,
        manifest=value,
        manifest_sha256=hashlib.sha256(payload).hexdigest(),
    )


__all__ = [
    "EPISODE_BUNDLE_SCHEMA",
    "EpisodeBundleArtifact",
    "validate_episode_bundle",
    "write_episode_bundle",
]
