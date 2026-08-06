"""Immutable normalized release input extracted from a raw episode artifact."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Final, cast

from .release import NormalizedEpisodeFacts


NORMALIZED_ARTIFACT_SCHEMA: Final = "wujihand.normalized_episode_artifact.v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_root(value: str | Path, *, expected_name: str | None = None) -> Path:
    raw = Path(value)
    if raw.is_symlink():
        raise ValueError("normalized artifact root must not be a symbolic link")
    root = raw.resolve()
    if not root.is_dir():
        raise ValueError("normalized artifact root must be a directory")
    if expected_name is not None and root.name != expected_name:
        raise ValueError("run root name and run_id differ")
    return root


@dataclass(frozen=True, slots=True)
class NormalizedEpisodeArtifact:
    root: Path
    facts: NormalizedEpisodeFacts
    facts_sha256: str


def load_normalized_episode_artifact(
    artifact_root: str | Path,
    *,
    expected_run_id: str,
) -> NormalizedEpisodeArtifact:
    root = _safe_root(artifact_root)
    entries = tuple(root.iterdir())
    if {item.name for item in entries} != {
        "facts.json",
        "manifest.json",
        "checksums.sha256",
    } or any(item.is_symlink() for item in entries):
        raise ValueError("normalized artifact inventory differs")
    try:
        value = json.loads((root / "facts.json").read_text(encoding="utf-8"))
        manifest_value = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("normalized artifact JSON is invalid") from exc
    facts = NormalizedEpisodeFacts.from_mapping(value)
    if facts.run_id != expected_run_id:
        raise ValueError("normalized facts and expected run IDs differ")
    if not isinstance(manifest_value, dict) or frozenset(manifest_value) != {
        "schema",
        "run_id",
        "facts_sha256",
        "tick_count",
    }:
        raise ValueError("normalized manifest keys differ")
    manifest = cast(dict[str, object], manifest_value)
    facts_sha256 = _sha256(root / "facts.json")
    if manifest != {
        "schema": NORMALIZED_ARTIFACT_SCHEMA,
        "run_id": expected_run_id,
        "facts_sha256": facts_sha256,
        "tick_count": len(facts.ticks),
    }:
        raise ValueError("normalized manifest closure differs")
    checksums: dict[str, str] = {}
    for line in (root / "checksums.sha256").read_text(encoding="utf-8").splitlines():
        fields = line.split(maxsplit=1)
        if len(fields) != 2 or len(fields[0]) != 64:
            raise ValueError("normalized checksum line is invalid")
        relative = fields[1].lstrip("*")
        if relative in checksums:
            raise ValueError("normalized checksum path is duplicated")
        checksums[relative] = fields[0]
    if checksums != {
        "facts.json": facts_sha256,
        "manifest.json": _sha256(root / "manifest.json"),
    }:
        raise ValueError("normalized artifact checksums differ")
    return NormalizedEpisodeArtifact(root=root, facts=facts, facts_sha256=facts_sha256)


def write_normalized_episode_artifact(
    run_root: str | Path,
    facts: NormalizedEpisodeFacts,
) -> NormalizedEpisodeArtifact:
    root = _safe_root(run_root, expected_name=facts.run_id)
    derived = root / "derived"
    derived.mkdir(exist_ok=True)
    if derived.is_symlink():
        raise ValueError("derived root must not be a symbolic link")
    destination = derived / "normalized"
    payload = (
        json.dumps(
            facts.to_mapping(),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    payload_digest = hashlib.sha256(payload).hexdigest()
    if destination.exists() or destination.is_symlink():
        existing = load_normalized_episode_artifact(
            destination,
            expected_run_id=facts.run_id,
        )
        if existing.facts_sha256 == payload_digest:
            return existing
        raise FileExistsError("a different normalized artifact already exists")
    temporary = Path(tempfile.mkdtemp(prefix=".normalized-", dir=derived))
    try:
        (temporary / "facts.json").write_bytes(payload)
        manifest_path = temporary / "manifest.json"
        manifest_path.write_text(
            json.dumps(
                {
                    "schema": NORMALIZED_ARTIFACT_SCHEMA,
                    "run_id": facts.run_id,
                    "facts_sha256": payload_digest,
                    "tick_count": len(facts.ticks),
                },
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n",
            encoding="utf-8",
        )
        (temporary / "checksums.sha256").write_text(
            f"{payload_digest}  facts.json\n{_sha256(manifest_path)}  manifest.json\n",
            encoding="utf-8",
        )
        os.rename(temporary, destination)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return load_normalized_episode_artifact(
        destination,
        expected_run_id=facts.run_id,
    )


__all__ = [
    "NORMALIZED_ARTIFACT_SCHEMA",
    "NormalizedEpisodeArtifact",
    "load_normalized_episode_artifact",
    "write_normalized_episode_artifact",
]
