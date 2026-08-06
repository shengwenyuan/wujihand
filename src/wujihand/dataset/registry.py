"""Recoverable collection membership and episode disposition management."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Final, Iterator

from wujihand.domain.recording import validate_recording_token, validate_run_id


COLLECTION_REGISTRY_SCHEMA: Final = "wujihand.dataset_collection_registry.v1"
PURGE_TOMBSTONE_SCHEMA: Final = "wujihand.dataset_episode_purge_tombstone.v1"
COLLECTION_EXPORT_SCHEMA: Final = "wujihand.dataset_collection_export.v1"


class EpisodeDisposition(str, Enum):
    CANDIDATE = "candidate"
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    INCOMPLETE = "incomplete"
    QUARANTINED_PURGE = "quarantined_purge"


@dataclass(frozen=True, slots=True)
class EpisodeRegistryRecord:
    episode_id: str
    run_root: str
    disposition: EpisodeDisposition
    reason: str
    release_decision_sha256: str | None
    updated_utc: str
    trash_path: str | None = None

    def to_mapping(self) -> dict[str, object]:
        return {
            "episode_id": self.episode_id,
            "run_root": self.run_root,
            "disposition": self.disposition.value,
            "reason": self.reason,
            "release_decision_sha256": self.release_decision_sha256,
            "updated_utc": self.updated_utc,
            "trash_path": self.trash_path,
        }


@dataclass(frozen=True, slots=True)
class CollectionExportRecord:
    collection_id: str
    revision_id: str
    dataset_root: str
    manifest_sha256: str
    episode_ids: tuple[str, ...]
    stale_episode_ids: tuple[str, ...]
    created_utc: str
    updated_utc: str

    @property
    def stale(self) -> bool:
        return bool(self.stale_episode_ids)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": COLLECTION_EXPORT_SCHEMA,
            "collection_id": self.collection_id,
            "revision_id": self.revision_id,
            "dataset_root": self.dataset_root,
            "manifest_sha256": self.manifest_sha256,
            "episode_ids": list(self.episode_ids),
            "stale_episode_ids": list(self.stale_episode_ids),
            "created_utc": self.created_utc,
            "updated_utc": self.updated_utc,
        }


def _utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_relative(value: str, *, field: str) -> Path:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or value.startswith("~") or "\\" in value:
        raise ValueError(f"{field} must be a safe project-relative path")
    return path


def _sha256_or_none(value: object, *, field: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or len(value) != 64 or any(
        char not in "0123456789abcdef" for char in value
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256 or null")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _directory_size(root: Path) -> int:
    total = 0
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError("episode purge refuses symbolic links anywhere in the run")
        if path.is_file():
            total += path.stat().st_size
    return total


def _write_new_json(path: Path, value: object) -> None:
    payload = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}-",
        suffix=".tmp",
        dir=path.parent,
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary_name, path)
        except FileExistsError as exc:
            raise FileExistsError(f"tombstone already exists: {path.name}") from exc
        os.unlink(temporary_name)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def _replace_json(path: Path, value: object) -> None:
    payload = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}-",
        suffix=".tmp",
        dir=path.parent,
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


class CollectionRegistry:
    def __init__(
        self,
        project_root: str | Path,
        collection_root: str | Path,
        *,
        collection_id: str,
    ) -> None:
        self.project_root = Path(project_root).resolve()
        if not self.project_root.is_dir():
            raise ValueError("project root must be a directory")
        self.collection_id = validate_recording_token(collection_id, field="collection_id")
        raw_collection = Path(collection_root)
        if raw_collection.is_absolute():
            candidate = raw_collection
        else:
            candidate = self.project_root / raw_collection
        if candidate.is_symlink():
            raise ValueError("collection root must not be a symbolic link")
        self.collection_root = candidate.resolve()
        try:
            self.collection_root.relative_to(self.project_root)
        except ValueError as exc:
            raise ValueError("collection root escapes project root") from exc
        self.collection_root.mkdir(parents=True, exist_ok=True)
        self.registry_path = self.collection_root / "registry.json"
        self.lock_path = self.collection_root / ".registry.lock"

    @contextmanager
    def _locked(self) -> Iterator[None]:
        with self.lock_path.open("a+b") as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    def _load(self) -> dict[str, EpisodeRegistryRecord]:
        if not self.registry_path.exists():
            return {}
        try:
            value = json.loads(self.registry_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError("collection registry is not valid JSON") from exc
        if not isinstance(value, dict) or value.get("schema") != COLLECTION_REGISTRY_SCHEMA:
            raise ValueError("collection registry schema is invalid")
        if value.get("collection_id") != self.collection_id:
            raise ValueError("collection registry ID differs")
        raw_records = value.get("episodes")
        if not isinstance(raw_records, list):
            raise ValueError("collection registry episodes must be a list")
        records: dict[str, EpisodeRegistryRecord] = {}
        for index, raw in enumerate(raw_records):
            if not isinstance(raw, dict):
                raise ValueError(f"episodes[{index}] must be a mapping")
            episode_id = validate_run_id(raw.get("episode_id"), field="episode_id")
            record = EpisodeRegistryRecord(
                episode_id=episode_id,
                run_root=_safe_relative(str(raw.get("run_root")), field="run_root").as_posix(),
                disposition=EpisodeDisposition(str(raw.get("disposition"))),
                reason=str(raw.get("reason")),
                release_decision_sha256=_sha256_or_none(
                    raw.get("release_decision_sha256"),
                    field="release_decision_sha256",
                ),
                updated_utc=str(raw.get("updated_utc")),
                trash_path=(
                    None
                    if raw.get("trash_path") is None
                    else _safe_relative(str(raw["trash_path"]), field="trash_path").as_posix()
                ),
            )
            if episode_id in records:
                raise ValueError("collection registry contains duplicate episode IDs")
            records[episode_id] = record
        return records

    def _write(self, records: dict[str, EpisodeRegistryRecord]) -> None:
        value = {
            "schema": COLLECTION_REGISTRY_SCHEMA,
            "collection_id": self.collection_id,
            "episodes": [records[key].to_mapping() for key in sorted(records)],
        }
        payload = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode("utf-8")
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".registry-",
            suffix=".tmp",
            dir=self.collection_root,
        )
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self.registry_path)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    def records(self) -> tuple[EpisodeRegistryRecord, ...]:
        with self._locked():
            records = self._load()
            return tuple(records[key] for key in sorted(records))

    def exports(self) -> tuple[CollectionExportRecord, ...]:
        with self._locked():
            exports = self._load_exports_locked()
            return tuple(exports[key] for key in sorted(exports))

    def stale_exports_for(self, episode_id: str) -> tuple[CollectionExportRecord, ...]:
        identifier = validate_run_id(episode_id, field="episode_id")
        return tuple(
            record for record in self.exports() if identifier in record.stale_episode_ids
        )

    def record_export(
        self,
        *,
        revision_id: str,
        dataset_root: str | Path,
        manifest_sha256: str,
        episode_ids: tuple[str, ...],
    ) -> CollectionExportRecord:
        revision = validate_recording_token(revision_id, field="revision_id")
        digest = _sha256_or_none(manifest_sha256, field="manifest_sha256")
        assert digest is not None
        identifiers = tuple(
            validate_run_id(value, field=f"episode_ids[{index}]")
            for index, value in enumerate(episode_ids)
        )
        if not identifiers or len(set(identifiers)) != len(identifiers):
            raise ValueError("export episode IDs must be non-empty and unique")
        raw_root = Path(dataset_root)
        candidate = raw_root if raw_root.is_absolute() else self.project_root / raw_root
        if candidate.is_symlink():
            raise ValueError("export dataset root must not be a symbolic link")
        resolved = candidate.resolve()
        try:
            relative = resolved.relative_to(self.project_root)
        except ValueError as exc:
            raise ValueError("export dataset root escapes project root") from exc
        if not resolved.is_dir():
            raise ValueError("export dataset root must be a directory")
        current = self.project_root
        for part in relative.parts:
            current = current / part
            if current.is_symlink():
                raise ValueError("export dataset root path contains a symbolic link")
        with self._locked():
            episodes = self._load()
            if any(
                identifier not in episodes
                or episodes[identifier].disposition is not EpisodeDisposition.ACCEPTED
                for identifier in identifiers
            ):
                raise ValueError("an export may contain only currently accepted episodes")
            exports = self._load_exports_locked()
            existing = exports.get(revision)
            if existing is not None:
                expected = (
                    relative.as_posix(),
                    digest,
                    identifiers,
                )
                actual = (
                    existing.dataset_root,
                    existing.manifest_sha256,
                    existing.episode_ids,
                )
                if actual == expected:
                    return existing
                raise FileExistsError("a different collection export revision already exists")
            export_root = self.collection_root / "exports"
            if export_root.is_symlink():
                raise ValueError("collection export root must not be a symbolic link")
            export_root.mkdir(exist_ok=True)
            timestamp = _utc()
            record = CollectionExportRecord(
                collection_id=self.collection_id,
                revision_id=revision,
                dataset_root=relative.as_posix(),
                manifest_sha256=digest,
                episode_ids=identifiers,
                stale_episode_ids=(),
                created_utc=timestamp,
                updated_utc=timestamp,
            )
            _write_new_json(export_root / f"{revision}.json", record.to_mapping())
            return record

    def _load_exports_locked(self) -> dict[str, CollectionExportRecord]:
        root = self.collection_root / "exports"
        if not root.exists():
            return {}
        if root.is_symlink() or not root.is_dir():
            raise ValueError("collection export root is unsafe")
        records: dict[str, CollectionExportRecord] = {}
        for path in sorted(root.iterdir()):
            if path.is_symlink() or not path.is_file() or path.suffix != ".json":
                raise ValueError("collection export inventory contains an unsafe entry")
            try:
                raw = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise ValueError("collection export record is not valid JSON") from exc
            if not isinstance(raw, dict) or frozenset(raw) != {
                "schema",
                "collection_id",
                "revision_id",
                "dataset_root",
                "manifest_sha256",
                "episode_ids",
                "stale_episode_ids",
                "created_utc",
                "updated_utc",
            }:
                raise ValueError("collection export record keys differ")
            if raw["schema"] != COLLECTION_EXPORT_SCHEMA or raw[
                "collection_id"
            ] != self.collection_id:
                raise ValueError("collection export record identity differs")
            revision = validate_recording_token(raw["revision_id"], field="revision_id")
            if path.name != f"{revision}.json" or revision in records:
                raise ValueError("collection export filename/revision differs")
            raw_episode_ids = raw["episode_ids"]
            raw_stale_ids = raw["stale_episode_ids"]
            if not isinstance(raw_episode_ids, list) or not isinstance(raw_stale_ids, list):
                raise ValueError("collection export episode inventories must be lists")
            episode_ids = tuple(
                validate_run_id(item, field=f"{revision}.episode_ids[{index}]")
                for index, item in enumerate(raw_episode_ids)
            )
            stale_ids = tuple(
                validate_run_id(item, field=f"{revision}.stale_episode_ids[{index}]")
                for index, item in enumerate(raw_stale_ids)
            )
            if (
                not episode_ids
                or len(set(episode_ids)) != len(episode_ids)
                or len(set(stale_ids)) != len(stale_ids)
                or not set(stale_ids) <= set(episode_ids)
            ):
                raise ValueError("collection export episode inventories differ")
            manifest_digest = _sha256_or_none(
                raw["manifest_sha256"], field="manifest_sha256"
            )
            if manifest_digest is None:
                raise ValueError("collection export manifest digest must not be null")
            records[revision] = CollectionExportRecord(
                collection_id=self.collection_id,
                revision_id=revision,
                dataset_root=_safe_relative(
                    str(raw["dataset_root"]), field="dataset_root"
                ).as_posix(),
                manifest_sha256=manifest_digest,
                episode_ids=episode_ids,
                stale_episode_ids=stale_ids,
                created_utc=str(raw["created_utc"]),
                updated_utc=str(raw["updated_utc"]),
            )
        return records

    def _mark_exports_stale_locked(self, episode_id: str) -> None:
        for record in self._load_exports_locked().values():
            if episode_id not in record.episode_ids or episode_id in record.stale_episode_ids:
                continue
            stale = tuple(
                identifier
                for identifier in record.episode_ids
                if identifier in {*record.stale_episode_ids, episode_id}
            )
            updated = CollectionExportRecord(
                collection_id=record.collection_id,
                revision_id=record.revision_id,
                dataset_root=record.dataset_root,
                manifest_sha256=record.manifest_sha256,
                episode_ids=record.episode_ids,
                stale_episode_ids=stale,
                created_utc=record.created_utc,
                updated_utc=_utc(),
            )
            _replace_json(
                self.collection_root / "exports" / f"{record.revision_id}.json",
                updated.to_mapping(),
            )

    def register(
        self,
        episode_id: str,
        run_root: str | Path,
        *,
        incomplete: bool = False,
        reason: str = "registered",
    ) -> EpisodeRegistryRecord:
        identifier = validate_run_id(episode_id, field="episode_id")
        relative = self._validated_run_root(run_root, episode_id=identifier)
        disposition = (
            EpisodeDisposition.INCOMPLETE if incomplete else EpisodeDisposition.CANDIDATE
        )
        with self._locked():
            records = self._load()
            existing = records.get(identifier)
            if existing is not None:
                if existing.run_root == relative.as_posix():
                    return existing
                raise ValueError("episode is already registered to a different run root")
            record = EpisodeRegistryRecord(
                episode_id=identifier,
                run_root=relative.as_posix(),
                disposition=disposition,
                reason=reason,
                release_decision_sha256=None,
                updated_utc=_utc(),
            )
            records[identifier] = record
            self._write(records)
            return record

    def reject(self, episode_id: str, *, reason: str) -> EpisodeRegistryRecord:
        return self._set_disposition(
            episode_id,
            disposition=EpisodeDisposition.REJECTED,
            reason=reason,
            release_decision_sha256=None,
            mark_exports_stale=True,
        )

    def restore(self, episode_id: str, *, reason: str = "restored_for_regate") -> EpisodeRegistryRecord:
        return self._set_disposition(
            episode_id,
            disposition=EpisodeDisposition.CANDIDATE,
            reason=reason,
            release_decision_sha256=None,
            allowed_from=frozenset({EpisodeDisposition.REJECTED}),
        )

    def accept(
        self,
        episode_id: str,
        *,
        release_decision_sha256: str,
        reason: str = "release_gates_passed",
    ) -> EpisodeRegistryRecord:
        digest = _sha256_or_none(
            release_decision_sha256,
            field="release_decision_sha256",
        )
        assert digest is not None
        return self._set_disposition(
            episode_id,
            disposition=EpisodeDisposition.ACCEPTED,
            reason=reason,
            release_decision_sha256=digest,
            allowed_from=frozenset({EpisodeDisposition.CANDIDATE}),
        )

    def quarantine_for_purge(
        self,
        episode_id: str,
        *,
        confirmation: str,
        reason: str,
    ) -> EpisodeRegistryRecord:
        """Move one rejected run to project trash and persist a recovery tombstone."""

        identifier = validate_run_id(episode_id, field="episode_id")
        if confirmation != identifier:
            raise ValueError("purge confirmation must exactly equal episode_id")
        if not reason or len(reason) > 256:
            raise ValueError("purge reason must be a bounded non-empty string")
        with self._locked():
            records = self._load()
            if identifier not in records:
                raise KeyError(f"episode is not registered: {identifier}")
            existing = records[identifier]
            if existing.disposition is EpisodeDisposition.QUARANTINED_PURGE:
                return existing
            if existing.disposition is not EpisodeDisposition.REJECTED:
                raise ValueError("only a rejected episode may be moved to project trash")
            source_relative = self._validated_run_root(
                existing.run_root,
                episode_id=identifier,
            )
            source = self.project_root / source_relative
            checksum_path = source / "checksums.sha256"
            if checksum_path.is_symlink() or not checksum_path.is_file():
                raise ValueError("purge requires the immutable source checksum file")
            source_checksum_sha256 = _sha256(checksum_path)
            source_bytes = _directory_size(source)

            trash_root = self.collection_root / "trash"
            if trash_root.is_symlink():
                raise ValueError("project trash must not be a symbolic link")
            trash_root.mkdir(exist_ok=True)
            destination = trash_root / identifier
            if destination.exists() or destination.is_symlink():
                raise FileExistsError("project trash already contains this episode")
            tombstone_root = self.collection_root / "tombstones"
            if tombstone_root.is_symlink():
                raise ValueError("tombstone root must not be a symbolic link")
            tombstone_root.mkdir(exist_ok=True)
            tombstone_path = tombstone_root / f"{identifier}.json"
            if tombstone_path.exists() or tombstone_path.is_symlink():
                raise FileExistsError("episode purge tombstone already exists")

            trash_relative = destination.relative_to(self.project_root).as_posix()
            timestamp = _utc()
            record = EpisodeRegistryRecord(
                episode_id=identifier,
                run_root=existing.run_root,
                disposition=EpisodeDisposition.QUARANTINED_PURGE,
                reason=reason,
                release_decision_sha256=None,
                updated_utc=timestamp,
                trash_path=trash_relative,
            )
            moved = False
            registry_written = False
            try:
                os.rename(source, destination)
                moved = True
                records[identifier] = record
                self._write(records)
                registry_written = True
                _write_new_json(
                    tombstone_path,
                    {
                        "schema": PURGE_TOMBSTONE_SCHEMA,
                        "collection_id": self.collection_id,
                        "episode_id": identifier,
                        "original_run_root": existing.run_root,
                        "project_trash_path": trash_relative,
                        "source_checksums_sha256": source_checksum_sha256,
                        "source_bytes": source_bytes,
                        "reason": reason,
                        "quarantined_utc": timestamp,
                        "recoverable": True,
                    },
                )
            except BaseException:
                if moved and destination.exists() and not source.exists():
                    os.rename(destination, source)
                if registry_written:
                    records[identifier] = existing
                    self._write(records)
                tombstone_path.unlink(missing_ok=True)
                raise
            return record

    def _set_disposition(
        self,
        episode_id: str,
        *,
        disposition: EpisodeDisposition,
        reason: str,
        release_decision_sha256: str | None,
        allowed_from: frozenset[EpisodeDisposition] | None = None,
        mark_exports_stale: bool = False,
    ) -> EpisodeRegistryRecord:
        identifier = validate_run_id(episode_id, field="episode_id")
        if not reason or len(reason) > 256:
            raise ValueError("reason must be a bounded non-empty string")
        with self._locked():
            records = self._load()
            if identifier not in records:
                raise KeyError(f"episode is not registered: {identifier}")
            existing = records[identifier]
            if existing.disposition is disposition and existing.reason == reason:
                if mark_exports_stale:
                    self._mark_exports_stale_locked(identifier)
                return existing
            if allowed_from is not None and existing.disposition not in allowed_from:
                raise ValueError(
                    f"cannot change {identifier} from {existing.disposition.value} "
                    f"to {disposition.value}"
                )
            record = EpisodeRegistryRecord(
                episode_id=identifier,
                run_root=existing.run_root,
                disposition=disposition,
                reason=reason,
                release_decision_sha256=release_decision_sha256,
                updated_utc=_utc(),
                trash_path=existing.trash_path,
            )
            if mark_exports_stale:
                self._mark_exports_stale_locked(identifier)
            records[identifier] = record
            self._write(records)
            return record

    def _validated_run_root(self, value: str | Path, *, episode_id: str) -> Path:
        raw = Path(value)
        candidate = raw if raw.is_absolute() else self.project_root / raw
        if candidate.is_symlink():
            raise ValueError("run root must not be a symbolic link")
        resolved = candidate.resolve()
        try:
            relative = resolved.relative_to(self.project_root)
        except ValueError as exc:
            raise ValueError("run root escapes project root") from exc
        if not resolved.is_dir() or resolved.name != episode_id:
            raise ValueError("run root must exist and its name must equal episode_id")
        current = self.project_root
        for part in relative.parts:
            current = current / part
            if current.is_symlink():
                raise ValueError("run root path must not contain symbolic links")
        return relative


__all__ = [
    "COLLECTION_REGISTRY_SCHEMA",
    "COLLECTION_EXPORT_SCHEMA",
    "PURGE_TOMBSTONE_SCHEMA",
    "CollectionExportRecord",
    "CollectionRegistry",
    "EpisodeDisposition",
    "EpisodeRegistryRecord",
]
