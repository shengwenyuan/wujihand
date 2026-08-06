from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.dataset.bundle import validate_episode_bundle, write_episode_bundle
from wujihand.dataset.release import ReleaseDecision, ReleaseGateResult
from wujihand.dataset.release_artifact import write_release_decision_artifact


DIGEST = "a" * 64


def _run(tmp_path: Path) -> Path:
    root = tmp_path / "episode-001"
    root.mkdir()
    for name in (
        "manifest.json",
        "recorder.json",
        "receipt.json",
        "checksums.sha256",
        "annotation.json",
    ):
        (root / name).write_text(f"{name}\n", encoding="utf-8")
    for relative in (
        "raw/rosbag2",
        "derived/normalized",
        "derived/alignment",
        "derived/vision",
    ):
        path = root / relative
        path.mkdir(parents=True)
        (path / "payload").write_text(relative, encoding="utf-8")
    write_release_decision_artifact(
        root,
        ReleaseDecision(
            run_id=root.name,
            passed=True,
            gates=(ReleaseGateResult("all", True, True, True, "passed"),),
        ),
    )
    return root


def test_bundle_closes_raw_and_derived_dependencies(tmp_path: Path) -> None:
    root = _run(tmp_path)
    artifact = write_episode_bundle(
        root,
        collection_id="mini-v1",
        dataset_profile_id="mini-profile-v1",
        dataset_profile_sha256=DIGEST,
        deployment_hash=DIGEST,
        session_hash=DIGEST,
        assembly_hash=DIGEST,
        workcell_hash=DIGEST,
    )

    assert validate_episode_bundle(root) == artifact
    assert (
        write_episode_bundle(
            root,
            collection_id="mini-v1",
            dataset_profile_id="mini-profile-v1",
            dataset_profile_sha256=DIGEST,
            deployment_hash=DIGEST,
            session_hash=DIGEST,
            assembly_hash=DIGEST,
            workcell_hash=DIGEST,
        )
        == artifact
    )


def test_bundle_detects_stale_dependency(tmp_path: Path) -> None:
    root = _run(tmp_path)
    write_episode_bundle(
        root,
        collection_id="mini-v1",
        dataset_profile_id="mini-profile-v1",
        dataset_profile_sha256=DIGEST,
        deployment_hash=DIGEST,
        session_hash=DIGEST,
        assembly_hash=DIGEST,
        workcell_hash=DIGEST,
    )
    (root / "derived" / "vision" / "payload").write_text("changed", encoding="utf-8")

    with pytest.raises(ValueError, match="stale"):
        validate_episode_bundle(root)
