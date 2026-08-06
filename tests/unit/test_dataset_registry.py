from __future__ import annotations

import json
from pathlib import Path

import pytest

from wujihand.dataset.registry import CollectionRegistry, EpisodeDisposition


def _registry(tmp_path: Path) -> tuple[CollectionRegistry, Path]:
    run = tmp_path / "artifacts" / "runs" / "episode-001"
    run.mkdir(parents=True)
    registry = CollectionRegistry(
        tmp_path,
        "artifacts/collections/mini-v1",
        collection_id="mini-v1",
    )
    return registry, run


def test_reject_is_one_step_recoverable_and_does_not_delete_raw(tmp_path: Path) -> None:
    registry, run = _registry(tmp_path)
    raw = run / "raw" / "rosbag2"
    raw.mkdir(parents=True)
    (raw / "episode.mcap").write_bytes(b"immutable")

    registered = registry.register("episode-001", run)
    rejected = registry.reject("episode-001", reason="operator_mistake")
    restored = registry.restore("episode-001")

    assert registered.disposition is EpisodeDisposition.CANDIDATE
    assert rejected.disposition is EpisodeDisposition.REJECTED
    assert restored.disposition is EpisodeDisposition.CANDIDATE
    assert (raw / "episode.mcap").read_bytes() == b"immutable"


def test_accept_requires_candidate_and_release_decision_digest(tmp_path: Path) -> None:
    registry, run = _registry(tmp_path)
    registry.register("episode-001", run)

    accepted = registry.accept(
        "episode-001",
        release_decision_sha256="a" * 64,
    )

    assert accepted.disposition is EpisodeDisposition.ACCEPTED
    assert accepted.release_decision_sha256 == "a" * 64
    with pytest.raises(ValueError, match="cannot change"):
        registry.restore("episode-001")


def test_registry_rejects_escape_and_symlink_run_root(tmp_path: Path) -> None:
    registry, run = _registry(tmp_path)
    outside = tmp_path.parent / "outside-episode-001"

    with pytest.raises(ValueError, match="escapes"):
        registry.register("episode-001", outside)

    link = tmp_path / "artifacts" / "runs" / "linked-episode"
    link.symlink_to(run, target_is_directory=True)
    with pytest.raises(ValueError, match="symbolic link"):
        registry.register("linked-episode", link)


def test_purge_moves_only_rejected_episode_to_recoverable_project_trash(
    tmp_path: Path,
) -> None:
    registry, run = _registry(tmp_path)
    (run / "checksums.sha256").write_text("a" * 64 + "  raw/episode.mcap\n")
    registry.register("episode-001", run)
    registry.reject("episode-001", reason="operator_mistake")

    purged = registry.quarantine_for_purge(
        "episode-001",
        confirmation="episode-001",
        reason="operator_confirmed_cleanup",
    )

    assert purged.disposition is EpisodeDisposition.QUARANTINED_PURGE
    assert not run.exists()
    assert purged.trash_path is not None
    trash = tmp_path / purged.trash_path
    assert trash.is_dir()
    tombstone = json.loads(
        (
            tmp_path
            / "artifacts"
            / "collections"
            / "mini-v1"
            / "tombstones"
            / "episode-001.json"
        ).read_text()
    )
    assert tombstone["episode_id"] == "episode-001"
    assert tombstone["recoverable"] is True
    assert registry.quarantine_for_purge(
        "episode-001",
        confirmation="episode-001",
        reason="operator_confirmed_cleanup",
    ) == purged


def test_purge_requires_exact_confirmation_and_rejected_state(tmp_path: Path) -> None:
    registry, run = _registry(tmp_path)
    (run / "checksums.sha256").write_text("a" * 64 + "  raw/episode.mcap\n")
    registry.register("episode-001", run)

    with pytest.raises(ValueError, match="confirmation"):
        registry.quarantine_for_purge(
            "episode-001",
            confirmation="episode-002",
            reason="cleanup",
        )
    with pytest.raises(ValueError, match="only a rejected"):
        registry.quarantine_for_purge(
            "episode-001",
            confirmation="episode-001",
            reason="cleanup",
        )
    assert run.is_dir()


def test_reject_marks_every_existing_export_revision_stale_and_restore_keeps_it(
    tmp_path: Path,
) -> None:
    registry, run = _registry(tmp_path)
    registry.register("episode-001", run)
    registry.accept("episode-001", release_decision_sha256="a" * 64)
    export = tmp_path / "artifacts" / "datasets" / "revision-001"
    export.mkdir(parents=True)
    recorded = registry.record_export(
        revision_id="revision-001",
        dataset_root=export,
        manifest_sha256="b" * 64,
        episode_ids=("episode-001",),
    )
    assert not recorded.stale

    registry.reject("episode-001", reason="operator_rejected_after_export")

    stale = registry.stale_exports_for("episode-001")
    assert len(stale) == 1
    assert stale[0].revision_id == "revision-001"
    assert stale[0].stale_episode_ids == ("episode-001",)
    registry.restore("episode-001")
    assert registry.stale_exports_for("episode-001") == stale


def test_collection_export_requires_accepted_episode_and_project_bounded_root(
    tmp_path: Path,
) -> None:
    registry, run = _registry(tmp_path)
    registry.register("episode-001", run)
    export = tmp_path / "artifacts" / "datasets" / "revision-001"
    export.mkdir(parents=True)

    with pytest.raises(ValueError, match="currently accepted"):
        registry.record_export(
            revision_id="revision-001",
            dataset_root=export,
            manifest_sha256="b" * 64,
            episode_ids=("episode-001",),
        )

    registry.accept("episode-001", release_decision_sha256="a" * 64)
    outside = tmp_path.parent / "outside-export"
    outside.mkdir(exist_ok=True)
    with pytest.raises(ValueError, match="escapes"):
        registry.record_export(
            revision_id="revision-001",
            dataset_root=outside,
            manifest_sha256="b" * 64,
            episode_ids=("episode-001",),
        )
