"""Pinned ModelScope dataset acquisition and verification.

The module is deliberately independent of Isaac Sim. Network access occurs only
through :func:`ensure_modelscope_dataset`; normal configuration resolution never
downloads content.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import time
from typing import Any, Iterator, cast
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen

from .config_repository import ConfigRepository
from .source_lock import SourceRecord


MODELSCOPE_MANIFEST_SCHEMA = "wujihand.modelscope_manifest.v1"
MODELSCOPE_RECEIPT_SCHEMA = "wujihand.modelscope_verification_receipt.v1"
_TREE_API = "https://modelscope.cn/api/v1/datasets/{dataset_id}/repo/tree"
_DOWNLOAD_URL = (
    "https://modelscope.cn/datasets/{dataset_id}/resolve/{revision}/{path}"
)
_PAGE_SIZE = 3000
_CHUNK_SIZE = 1024 * 1024


@dataclass(frozen=True, slots=True)
class ModelScopeDatasetPin:
    """One immutable ModelScope dataset snapshot declared by SourceLock."""

    source_name: str
    dataset_id: str
    revision: str
    local_runtime_path: str
    manifest_sha256: str
    expected_blob_count: int
    expected_tree_count: int
    expected_total_size_bytes: int

    @classmethod
    def from_source_record(cls, record: SourceRecord) -> ModelScopeDatasetPin:
        revision = dict(record.revision).get("commit")
        missing = [
            name
            for name, value in (
                ("dataset_id", record.dataset_id),
                ("commit", revision),
                ("manifest_sha256", record.manifest_sha256),
                ("expected_blob_count", record.expected_blob_count),
                ("expected_tree_count", record.expected_tree_count),
                ("expected_total_size_bytes", record.expected_total_size_bytes),
            )
            if value is None
        ]
        if missing:
            raise ValueError(
                f"source {record.name!r} is not a complete ModelScope pin: "
                f"missing={missing}"
            )
        return cls(
            source_name=record.name,
            dataset_id=cast(str, record.dataset_id),
            revision=cast(str, revision),
            local_runtime_path=record.local_runtime_path,
            manifest_sha256=cast(str, record.manifest_sha256),
            expected_blob_count=cast(int, record.expected_blob_count),
            expected_tree_count=cast(int, record.expected_tree_count),
            expected_total_size_bytes=cast(
                int, record.expected_total_size_bytes
            ),
        )


@dataclass(frozen=True, slots=True)
class ModelScopeManifestEntry:
    path: str
    kind: str
    size: int
    sha256: str | None

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str,
    ) -> ModelScopeManifestEntry:
        if not isinstance(value, Mapping):
            raise ValueError(f"{field} must be a mapping")
        data = cast(Mapping[object, object], value)
        path = data.get("path")
        kind = data.get("kind")
        size = data.get("size")
        sha256 = data.get("sha256")
        if not isinstance(path, str) or not _safe_relative_path(path):
            raise ValueError(f"{field}.path must be a safe relative path")
        if kind not in ("blob", "tree"):
            raise ValueError(f"{field}.kind must be blob or tree")
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            raise ValueError(f"{field}.size must be a non-negative integer")
        if kind == "blob":
            if not _is_sha256(sha256):
                raise ValueError(f"{field}.sha256 must be a lowercase SHA-256")
        elif sha256 is not None and sha256 != "":
            raise ValueError(f"{field}.sha256 must be null for a tree")
        return cls(
            path=path,
            kind=kind,
            size=size,
            sha256=cast(str | None, sha256 or None),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "path": self.path,
            "kind": self.kind,
            "size": self.size,
            "sha256": self.sha256,
        }


@dataclass(frozen=True, slots=True)
class ModelScopeManifest:
    entries: tuple[ModelScopeManifestEntry, ...]
    sha256: str
    blob_count: int
    tree_count: int
    total_size_bytes: int

    @classmethod
    def build(
        cls,
        entries: Iterable[ModelScopeManifestEntry],
    ) -> ModelScopeManifest:
        ordered = tuple(sorted(entries, key=lambda item: item.path))
        paths = tuple(entry.path for entry in ordered)
        if len(set(paths)) != len(paths):
            raise ValueError("ModelScope manifest paths must be unique")
        encoded = json.dumps(
            [
                {
                    "path": entry.path,
                    "type": entry.kind,
                    "size": entry.size,
                    "sha256": entry.sha256 or "",
                }
                for entry in ordered
            ],
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode()
        blobs = tuple(entry for entry in ordered if entry.kind == "blob")
        return cls(
            entries=ordered,
            sha256=hashlib.sha256(encoded).hexdigest(),
            blob_count=len(blobs),
            tree_count=len(ordered) - len(blobs),
            total_size_bytes=sum(entry.size for entry in blobs),
        )

    def validate_pin(self, pin: ModelScopeDatasetPin) -> None:
        actual = (
            self.sha256,
            self.blob_count,
            self.tree_count,
            self.total_size_bytes,
        )
        expected = (
            pin.manifest_sha256,
            pin.expected_blob_count,
            pin.expected_tree_count,
            pin.expected_total_size_bytes,
        )
        if actual != expected:
            raise RuntimeError(
                "ModelScope manifest differs from SourceLock: "
                f"expected={expected}, actual={actual}"
            )


@dataclass(frozen=True, slots=True)
class ModelScopeEnsureResult:
    source_name: str
    target: Path
    receipt: Path
    manifest_sha256: str
    blob_count: int
    tree_count: int
    total_size_bytes: int
    action: str

    def to_mapping(self) -> dict[str, object]:
        return {
            "source": self.source_name,
            "target": self.target.as_posix(),
            "receipt": self.receipt.as_posix(),
            "manifest_sha256": self.manifest_sha256,
            "blob_count": self.blob_count,
            "tree_count": self.tree_count,
            "total_size_bytes": self.total_size_bytes,
            "action": self.action,
        }


def fetch_modelscope_manifest(pin: ModelScopeDatasetPin) -> ModelScopeManifest:
    """Fetch and canonicalize the exact pinned tree manifest."""

    entries: list[ModelScopeManifestEntry] = []
    page = 1
    total_count: int | None = None
    while total_count is None or len(entries) < total_count:
        query = urlencode(
            {
                "Revision": pin.revision,
                "Recursive": "True",
                "PageNumber": page,
                "PageSize": _PAGE_SIZE,
            }
        )
        url = _TREE_API.format(dataset_id=pin.dataset_id) + "?" + query
        with urlopen(url, timeout=120) as response:
            payload: Any = json.load(response)
        root = _string_mapping(payload, field="ModelScope response")
        if root.get("Code") != 200:
            raise RuntimeError(
                f"ModelScope manifest request failed: {root.get('Message')!r}"
            )
        data = _string_mapping(root.get("Data"), field="ModelScope response.Data")
        raw_files = data.get("Files")
        if not isinstance(raw_files, list):
            raise RuntimeError("ModelScope response.Data.Files must be a list")
        for index, raw in enumerate(raw_files):
            item = _string_mapping(
                raw,
                field=f"ModelScope response.Data.Files[{index}]",
            )
            entries.append(
                ModelScopeManifestEntry.from_mapping(
                    {
                        "path": item.get("Path"),
                        "kind": item.get("Type"),
                        "size": item.get("Size"),
                        "sha256": item.get("Sha256"),
                    },
                    field=f"ModelScope response.Data.Files[{index}]",
                )
            )
        raw_total = data.get("TotalCount", root.get("TotalCount"))
        if isinstance(raw_total, bool) or not isinstance(raw_total, int):
            raise RuntimeError("ModelScope response total count is invalid")
        total_count = raw_total
        latest = _string_mapping(
            data.get("LatestCommitter"),
            field="ModelScope response.Data.LatestCommitter",
        )
        short_revision = latest.get("ShortId")
        if (
            not isinstance(short_revision, str)
            or not pin.revision.startswith(short_revision)
        ):
            raise RuntimeError(
                "ModelScope resolved a different revision: "
                f"expected={pin.revision}, actual={short_revision!r}"
            )
        if not raw_files and len(entries) < total_count:
            raise RuntimeError("ModelScope manifest pagination stopped early")
        page += 1
    if len(entries) != total_count:
        raise RuntimeError(
            f"ModelScope manifest count mismatch: {len(entries)} != {total_count}"
        )
    manifest = ModelScopeManifest.build(entries)
    manifest.validate_pin(pin)
    return manifest


def verify_modelscope_snapshot(
    root: Path,
    manifest: ModelScopeManifest,
) -> None:
    """Verify an exact tree without following symbolic links."""

    if not root.is_dir():
        raise FileNotFoundError(f"ModelScope snapshot directory not found: {root}")
    expected_files = {
        entry.path: entry for entry in manifest.entries if entry.kind == "blob"
    }
    expected_dirs = {
        entry.path for entry in manifest.entries if entry.kind == "tree"
    }
    actual_files: set[str] = set()
    actual_dirs: set[str] = set()
    for current, directories, filenames in os.walk(root, followlinks=False):
        current_path = Path(current)
        for name in directories:
            path = current_path / name
            relative = path.relative_to(root).as_posix()
            if path.is_symlink():
                raise RuntimeError(
                    f"ModelScope snapshot contains a symbolic link: {relative}"
                )
            actual_dirs.add(relative)
        for name in filenames:
            path = current_path / name
            relative = path.relative_to(root).as_posix()
            if path.is_symlink() or not path.is_file():
                raise RuntimeError(
                    f"ModelScope snapshot contains a non-regular file: {relative}"
                )
            actual_files.add(relative)
    if actual_files != set(expected_files):
        _raise_path_difference("files", set(expected_files), actual_files)
    if actual_dirs != expected_dirs:
        _raise_path_difference("directories", expected_dirs, actual_dirs)
    for relative_path, entry in expected_files.items():
        path = root / relative_path
        if path.stat().st_size != entry.size:
            raise RuntimeError(
                f"ModelScope snapshot size mismatch for {relative_path}"
            )
        actual_sha256 = _sha256_file(path)
        if actual_sha256 != entry.sha256:
            raise RuntimeError(
                f"ModelScope snapshot SHA-256 mismatch for {relative_path}: "
                f"expected={entry.sha256}, actual={actual_sha256}"
            )


def ensure_modelscope_dataset(
    project_root: str | Path,
    pin: ModelScopeDatasetPin,
    *,
    seed_from: str | Path | None = None,
    allow_network: bool = True,
    workers: int = 8,
    force_full_verify: bool = False,
) -> ModelScopeEnsureResult:
    """Restore one snapshot through staging, verification, and atomic promotion."""

    if not 1 <= workers <= 32:
        raise ValueError("workers must be in [1, 32]")
    repository = ConfigRepository(project_root)
    target = repository.resolve_project_path(
        pin.local_runtime_path,
        field=f"source {pin.source_name} root",
        must_exist=False,
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    verification_dir = target.parent / ".verification"
    receipt_path = verification_dir / f"{pin.revision}.json"
    manifest_path = verification_dir / f"{pin.revision}.manifest.json"
    lock_path = target.parent / ".ensure.lock"
    with _exclusive_lock(lock_path):
        if (
            not force_full_verify
            and _receipt_matches(receipt_path, target, pin)
        ):
            return _ensure_result(pin, target, receipt_path, action="ready")

        manifest = _load_cached_manifest(manifest_path, pin)
        if manifest is None:
            if not allow_network:
                raise RuntimeError(
                    "ModelScope manifest is not cached and network access is disabled"
                )
            manifest = fetch_modelscope_manifest(pin)
            verification_dir.mkdir(parents=True, exist_ok=True)
            _write_json_atomic(
                manifest_path,
                {
                    "schema": MODELSCOPE_MANIFEST_SCHEMA,
                    "source": pin.source_name,
                    "dataset_id": pin.dataset_id,
                    "revision": pin.revision,
                    "entries": [
                        entry.to_mapping() for entry in manifest.entries
                    ],
                },
            )

        if target.exists():
            verify_modelscope_snapshot(target, manifest)
            action = "verified"
        else:
            staging = target.parent / f".{pin.revision}.staging"
            staging.mkdir(parents=True, exist_ok=True)
            if seed_from is not None:
                _seed_staging(Path(seed_from), staging, manifest)
                action = "seeded"
            else:
                if not allow_network:
                    raise RuntimeError(
                        "ModelScope snapshot is missing and network access is disabled"
                    )
                _download_manifest(pin, staging, manifest, workers=workers)
                action = "downloaded"
            verify_modelscope_snapshot(staging, manifest)
            staging.replace(target)

        verification_dir.mkdir(parents=True, exist_ok=True)
        _write_json_atomic(
            receipt_path,
            {
                "schema": MODELSCOPE_RECEIPT_SCHEMA,
                "source": pin.source_name,
                "dataset_id": pin.dataset_id,
                "revision": pin.revision,
                "target": repository.project_relative(
                    target,
                    field=f"source {pin.source_name} root",
                ),
                "manifest_sha256": manifest.sha256,
                "blob_count": manifest.blob_count,
                "tree_count": manifest.tree_count,
                "total_size_bytes": manifest.total_size_bytes,
                "verified_at": datetime.now(timezone.utc).isoformat(),
            },
        )
        return _ensure_result(pin, target, receipt_path, action=action)


def _load_cached_manifest(
    path: Path,
    pin: ModelScopeDatasetPin,
) -> ModelScopeManifest | None:
    if not path.is_file():
        return None
    value: Any = json.loads(path.read_text(encoding="utf-8"))
    root = _string_mapping(value, field="cached ModelScope manifest")
    if (
        root.get("schema") != MODELSCOPE_MANIFEST_SCHEMA
        or root.get("source") != pin.source_name
        or root.get("dataset_id") != pin.dataset_id
        or root.get("revision") != pin.revision
    ):
        raise RuntimeError(f"cached ModelScope manifest identity differs: {path}")
    raw_entries = root.get("entries")
    if not isinstance(raw_entries, list):
        raise RuntimeError(f"cached ModelScope manifest has no entries: {path}")
    manifest = ModelScopeManifest.build(
        ModelScopeManifestEntry.from_mapping(
            entry,
            field=f"cached ModelScope manifest.entries[{index}]",
        )
        for index, entry in enumerate(raw_entries)
    )
    manifest.validate_pin(pin)
    return manifest


def _receipt_matches(
    path: Path,
    target: Path,
    pin: ModelScopeDatasetPin,
) -> bool:
    if not path.is_file() or not target.is_dir():
        return False
    try:
        value: Any = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not isinstance(value, Mapping):
        return False
    return (
        value.get("schema") == MODELSCOPE_RECEIPT_SCHEMA
        and value.get("source") == pin.source_name
        and value.get("dataset_id") == pin.dataset_id
        and value.get("revision") == pin.revision
        and value.get("manifest_sha256") == pin.manifest_sha256
        and value.get("blob_count") == pin.expected_blob_count
        and value.get("tree_count") == pin.expected_tree_count
        and value.get("total_size_bytes") == pin.expected_total_size_bytes
    )


def _seed_staging(
    source: Path,
    staging: Path,
    manifest: ModelScopeManifest,
) -> None:
    if not source.is_dir():
        raise FileNotFoundError(f"ModelScope seed directory not found: {source}")
    for entry in manifest.entries:
        destination = staging / entry.path
        if entry.kind == "tree":
            destination.mkdir(parents=True, exist_ok=True)
            continue
        origin = source / entry.path
        if origin.is_symlink() or not origin.is_file():
            raise FileNotFoundError(f"ModelScope seed file not found: {origin}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        if (
            destination.is_file()
            and not destination.is_symlink()
            and destination.stat().st_size == entry.size
            and _sha256_file(destination) == entry.sha256
        ):
            continue
        if destination.is_symlink() or (
            destination.exists() and not destination.is_file()
        ):
            raise RuntimeError(
                f"ModelScope staging path is not a regular file: {destination}"
            )
        destination.unlink(missing_ok=True)
        try:
            os.link(origin, destination)
        except OSError:
            shutil.copy2(origin, destination)


def _download_manifest(
    pin: ModelScopeDatasetPin,
    staging: Path,
    manifest: ModelScopeManifest,
    *,
    workers: int,
) -> None:
    for entry in manifest.entries:
        if entry.kind == "tree":
            (staging / entry.path).mkdir(parents=True, exist_ok=True)
    blobs = tuple(entry for entry in manifest.entries if entry.kind == "blob")
    with ThreadPoolExecutor(max_workers=workers) as executor:
        tuple(
            executor.map(
                lambda entry: _download_blob(pin, staging, entry),
                blobs,
            )
        )


def _download_blob(
    pin: ModelScopeDatasetPin,
    staging: Path,
    entry: ModelScopeManifestEntry,
) -> None:
    destination = staging / entry.path
    destination.parent.mkdir(parents=True, exist_ok=True)
    if (
        destination.is_file()
        and destination.stat().st_size == entry.size
        and _sha256_file(destination) == entry.sha256
    ):
        return
    part = destination.with_name(destination.name + ".part")
    if part.exists() and part.stat().st_size > entry.size:
        part.unlink()
    url = _DOWNLOAD_URL.format(
        dataset_id=quote(pin.dataset_id, safe="/"),
        revision=pin.revision,
        path=quote(entry.path, safe="/"),
    )
    last_error: Exception | None = None
    for attempt in range(3):
        try:
            _download_once(url, part, expected_size=entry.size)
            if _sha256_file(part) != entry.sha256:
                part.unlink(missing_ok=True)
                raise RuntimeError(
                    f"downloaded SHA-256 differs for {entry.path}"
                )
            part.replace(destination)
            return
        except Exception as exc:  # noqa: BLE001 - retry preserves the cause
            last_error = exc
            if attempt < 2:
                time.sleep(2**attempt)
    raise RuntimeError(f"failed to download {entry.path}") from last_error


def _download_once(url: str, part: Path, *, expected_size: int) -> None:
    offset = part.stat().st_size if part.is_file() else 0
    headers = {} if offset == 0 else {"Range": f"bytes={offset}-"}
    request = Request(url, headers=headers)
    with urlopen(request, timeout=120) as response:
        append = offset > 0 and getattr(response, "status", None) == 206
        mode = "ab" if append else "wb"
        with part.open(mode) as stream:
            shutil.copyfileobj(response, stream, length=_CHUNK_SIZE)
    if part.stat().st_size != expected_size:
        raise RuntimeError(
            f"downloaded size differs: expected={expected_size}, "
            f"actual={part.stat().st_size}"
        )


def _ensure_result(
    pin: ModelScopeDatasetPin,
    target: Path,
    receipt: Path,
    *,
    action: str,
) -> ModelScopeEnsureResult:
    return ModelScopeEnsureResult(
        source_name=pin.source_name,
        target=target,
        receipt=receipt,
        manifest_sha256=pin.manifest_sha256,
        blob_count=pin.expected_blob_count,
        tree_count=pin.expected_tree_count,
        total_size_bytes=pin.expected_total_size_bytes,
        action=action,
    )


@contextmanager
def _exclusive_lock(path: Path) -> Iterator[None]:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def _write_json_atomic(path: Path, value: Mapping[str, object]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(_CHUNK_SIZE), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_relative_path(value: str) -> bool:
    path = PurePosixPath(value)
    return (
        bool(value)
        and not path.is_absolute()
        and "\\" not in value
        and all(part not in {"", ".", ".."} for part in path.parts)
    )


def _is_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _string_mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(
        not isinstance(key, str) for key in value
    ):
        raise RuntimeError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _raise_path_difference(
    label: str,
    expected: set[str],
    actual: set[str],
) -> None:
    missing = sorted(expected - actual)[:10]
    extra = sorted(actual - expected)[:10]
    raise RuntimeError(
        f"ModelScope snapshot {label} differ: missing={missing}, extra={extra}"
    )


__all__ = [
    "MODELSCOPE_MANIFEST_SCHEMA",
    "MODELSCOPE_RECEIPT_SCHEMA",
    "ModelScopeDatasetPin",
    "ModelScopeEnsureResult",
    "ModelScopeManifest",
    "ModelScopeManifestEntry",
    "ensure_modelscope_dataset",
    "fetch_modelscope_manifest",
    "verify_modelscope_snapshot",
]
