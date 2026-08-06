"""Atomic persistence and strict loading of one offline release decision."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Final

from .release import ReleaseDecision


RELEASE_ARTIFACT_SCHEMA: Final = "wujihand.dataset_release_artifact.v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _run_root(value: str | Path, *, run_id: str) -> Path:
    raw = Path(value)
    if raw.is_symlink():
        raise ValueError("release run root must not be a symbolic link")
    root = raw.resolve()
    if not root.is_dir() or root.name != run_id:
        raise ValueError("release run root must exist and its name must equal run_id")
    return root


@dataclass(frozen=True, slots=True)
class ReleaseDecisionArtifact:
    root: Path
    decision: ReleaseDecision
    decision_sha256: str


def load_release_decision_artifact(
    artifact_root: str | Path,
    *,
    expected_run_id: str,
) -> ReleaseDecisionArtifact:
    raw = Path(artifact_root)
    if raw.is_symlink():
        raise ValueError("release artifact root must not be a symbolic link")
    root = raw.resolve()
    if not root.is_dir():
        raise ValueError("release artifact root must be a directory")
    expected_files = {"decision.json", "manifest.json", "checksums.sha256"}
    if {path.name for path in root.iterdir()} != expected_files or any(
        path.is_symlink() for path in root.iterdir()
    ):
        raise ValueError("release artifact file inventory differs")
    try:
        decision_value = json.loads((root / "decision.json").read_text(encoding="utf-8"))
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("release artifact JSON is invalid") from exc
    decision = ReleaseDecision.from_mapping(decision_value)
    if decision.run_id != expected_run_id:
        raise ValueError("release decision and expected run IDs differ")
    if not isinstance(manifest, dict) or frozenset(manifest) != {
        "schema",
        "run_id",
        "decision_sha256",
        "passed",
    }:
        raise ValueError("release artifact manifest keys differ")
    decision_sha256 = _sha256(root / "decision.json")
    if (
        manifest.get("schema") != RELEASE_ARTIFACT_SCHEMA
        or manifest.get("run_id") != expected_run_id
        or manifest.get("decision_sha256") != decision_sha256
        or manifest.get("passed") is not decision.passed
    ):
        raise ValueError("release artifact manifest closure differs")
    expected_checksums = {
        "decision.json": decision_sha256,
        "manifest.json": _sha256(root / "manifest.json"),
    }
    checksums: dict[str, str] = {}
    for line in (root / "checksums.sha256").read_text(encoding="utf-8").splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise ValueError("release artifact checksum line is invalid")
        relative = parts[1].lstrip("*")
        if relative in checksums:
            raise ValueError("release artifact checksum path is duplicated")
        checksums[relative] = parts[0]
    if checksums != expected_checksums:
        raise ValueError("release artifact checksums differ")
    return ReleaseDecisionArtifact(
        root=root,
        decision=decision,
        decision_sha256=decision_sha256,
    )


def write_release_decision_artifact(
    run_root: str | Path,
    decision: ReleaseDecision,
) -> ReleaseDecisionArtifact:
    root = _run_root(run_root, run_id=decision.run_id)
    derived = root / "derived"
    derived.mkdir(exist_ok=True)
    if derived.is_symlink():
        raise ValueError("release derived root must not be a symbolic link")
    destination = derived / "release"
    if destination.exists() or destination.is_symlink():
        existing = load_release_decision_artifact(
            destination,
            expected_run_id=decision.run_id,
        )
        expected_payload = (
            json.dumps(decision.to_mapping(), sort_keys=True, indent=2, allow_nan=False)
            + "\n"
        ).encode("utf-8")
        if _sha256(destination / "decision.json") == hashlib.sha256(
            expected_payload
        ).hexdigest():
            return existing
        raise FileExistsError("a different release decision artifact already exists")

    temporary = Path(tempfile.mkdtemp(prefix=".release-", dir=derived))
    try:
        decision_path = temporary / "decision.json"
        decision_path.write_text(
            json.dumps(
                decision.to_mapping(),
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        decision_sha256 = _sha256(decision_path)
        manifest_path = temporary / "manifest.json"
        manifest_path.write_text(
            json.dumps(
                {
                    "schema": RELEASE_ARTIFACT_SCHEMA,
                    "run_id": decision.run_id,
                    "decision_sha256": decision_sha256,
                    "passed": decision.passed,
                },
                sort_keys=True,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        (temporary / "checksums.sha256").write_text(
            f"{decision_sha256}  decision.json\n"
            f"{_sha256(manifest_path)}  manifest.json\n",
            encoding="utf-8",
        )
        os.rename(temporary, destination)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return load_release_decision_artifact(
        destination,
        expected_run_id=decision.run_id,
    )


__all__ = [
    "RELEASE_ARTIFACT_SCHEMA",
    "ReleaseDecisionArtifact",
    "load_release_decision_artifact",
    "write_release_decision_artifact",
]
