"""Resolve Backend Binding artifacts through ``third_party/sources.lock.yaml``."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
import re
from typing import cast

from wujihand.integrity import sha256_file, sha256_tree
from wujihand.specs import ArtifactSpec

from .config_repository import ConfigRepository
from .yaml_loader import load_yaml_strict


_SHA256 = re.compile(r"^[0-9a-f]{64}$")

@dataclass(frozen=True, slots=True)
class SourceRecord:
    name: str
    local_runtime_path: str
    artifacts: tuple[tuple[str, str], ...]
    asset_trees: tuple[tuple[str, str], ...]
    revision: tuple[tuple[str, str], ...]

    def expected_artifact_hash(self, relative_path: str) -> str:
        for path, digest in self.artifacts:
            if path == relative_path:
                return digest
        raise ValueError(
            f"source {self.name!r} does not lock artifact {relative_path!r}"
        )

    def expected_tree_hash(self, relative_path: str) -> str:
        for path, digest in self.asset_trees:
            if path == relative_path:
                return digest
        raise ValueError(
            f"source {self.name!r} does not lock asset tree {relative_path!r}"
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "name": self.name,
            "local_runtime_path": self.local_runtime_path,
            "revision": dict(self.revision),
        }


@dataclass(frozen=True, slots=True)
class ResolvedArtifact:
    source: SourceRecord
    relative_path: str
    absolute_path: Path
    expected_sha256: str
    kind: str

    def to_mapping(self) -> dict[str, object]:
        return {
            "source": self.source.name,
            "path": self.relative_path,
            "expected_sha256": self.expected_sha256,
            "kind": self.kind,
        }


class SourceLock:
    """Typed subset of the heterogeneous source-lock file used by bindings."""

    def __init__(
        self,
        repository: ConfigRepository,
        records: tuple[SourceRecord, ...],
        *,
        lock_path: str,
    ) -> None:
        if len({record.name for record in records}) != len(records):
            raise ValueError("source lock names must be unique")
        self._repository = repository
        self._records = records
        self.lock_path = lock_path

    @classmethod
    def load(
        cls,
        repository: ConfigRepository,
        path: str | Path = "third_party/sources.lock.yaml",
    ) -> SourceLock:
        resolved = repository.resolve_project_path(path, field="source lock")
        document = load_yaml_strict(resolved.read_text(encoding="utf-8"))
        if not isinstance(document, Mapping):
            raise ValueError("source lock must contain a mapping")
        root = cast(Mapping[str, object], document)
        if root.get("schema_version") != 1:
            raise ValueError("unsupported source lock schema")
        raw_sources = root.get("sources")
        if not isinstance(raw_sources, list):
            raise ValueError("source lock sources must be a list")
        records = tuple(
            _source_record(value, index=index) for index, value in enumerate(raw_sources)
        )
        return cls(
            repository,
            records,
            lock_path=repository.project_relative(resolved, field="source lock"),
        )

    @property
    def records(self) -> tuple[SourceRecord, ...]:
        return self._records

    def record(self, name: str) -> SourceRecord:
        for record in self._records:
            if record.name == name:
                return record
        raise ValueError(f"binding references unknown source-lock entry: {name!r}")

    def resolve(
        self,
        artifact: ArtifactSpec,
        *,
        tree: bool = False,
        verify: bool = False,
    ) -> ResolvedArtifact:
        record = self.record(artifact.source)
        revision_kind, _, expected_revision = artifact.source_revision.partition(":")
        actual_revision = dict(record.revision).get(revision_kind)
        if actual_revision != expected_revision:
            raise ValueError(
                f"source {record.name!r} does not match pinned revision "
                f"{artifact.source_revision!r}"
            )
        expected = (
            record.expected_tree_hash(artifact.path)
            if tree
            else record.expected_artifact_hash(artifact.path)
        )
        source_root = self._repository.resolve_project_path(
            record.local_runtime_path,
            field=f"source {record.name} root",
            must_exist=False,
        )
        absolute = self._repository.resolve_project_path(
            source_root / artifact.path,
            field=f"source {record.name} artifact",
            must_exist=verify,
            expect_directory=tree,
        )
        try:
            absolute.relative_to(source_root)
        except ValueError as exc:
            raise ValueError(
                f"source {record.name!r} artifact escapes its source root: {artifact.path}"
            ) from exc
        if verify:
            actual = sha256_tree(absolute) if tree else sha256_file(absolute)
            if actual != expected:
                label = "asset tree" if tree else "artifact"
                raise RuntimeError(
                    f"{record.name} {label} SHA-256 mismatch for {artifact.path}: "
                    f"expected {expected}, got {actual}"
                )
        return ResolvedArtifact(
            source=record,
            relative_path=artifact.path,
            absolute_path=absolute,
            expected_sha256=expected,
            kind="asset_tree" if tree else "artifact",
        )


def _source_record(value: object, *, index: int) -> SourceRecord:
    if not isinstance(value, Mapping):
        raise ValueError(f"source lock sources[{index}] must be a mapping")
    data = cast(Mapping[str, object], value)
    name = data.get("name")
    local_runtime_path = data.get("local_runtime_path")
    if not isinstance(name, str) or not name:
        raise ValueError(f"source lock sources[{index}].name must be non-blank")
    if not isinstance(local_runtime_path, str) or not local_runtime_path:
        raise ValueError(
            f"source lock sources[{index}].local_runtime_path must be non-blank"
        )
    artifacts = _digest_pairs(data.get("artifacts", {}), field=f"sources[{index}].artifacts")
    trees = _digest_pairs(
        data.get("asset_trees", {}), field=f"sources[{index}].asset_trees"
    )
    revision = tuple(
        sorted(
            (key, str(data[key]))
            for key in ("kind", "url", "tag", "commit", "sha256")
            if data.get(key) is not None
        )
    )
    return SourceRecord(
        name=name,
        local_runtime_path=local_runtime_path,
        artifacts=artifacts,
        asset_trees=trees,
        revision=revision,
    )


def _digest_pairs(value: object, *, field: str) -> tuple[tuple[str, str], ...]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    mapping = cast(Mapping[object, object], value)
    pairs: list[tuple[str, str]] = []
    for path, digest in mapping.items():
        if not isinstance(path, str) or not path:
            raise ValueError(f"{field} paths must be non-blank strings")
        if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
            raise ValueError(f"{field}.{path} must be a lowercase SHA-256")
        pairs.append((path, digest))
    return tuple(sorted(pairs))


__all__ = [
    "ResolvedArtifact",
    "SourceLock",
    "SourceRecord",
    "sha256_file",
    "sha256_tree",
]
