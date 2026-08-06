from __future__ import annotations

from pathlib import Path

from test_dataset_policy import _episode as policy_episode
from test_dataset_release import _episode as release_episode
from wujihand.dataset.bundle import validate_episode_bundle, write_episode_bundle
from wujihand.dataset.normalized import write_normalized_episode_artifact
from wujihand.dataset.policy import load_policy_episode
from wujihand.dataset.quality import build_quality_report
from wujihand.dataset.registry import CollectionRegistry, EpisodeDisposition
from wujihand.dataset.release import validate_episode_release
from wujihand.dataset.release_artifact import write_release_decision_artifact


def test_device_free_episode_closes_quality_bundle_and_recoverable_registry(
    tmp_path: Path,
) -> None:
    facts, q54_profile = release_episode()
    run = policy_episode(
        tmp_path,
        tuple(tick.transition for tick in facts.ticks),
    )
    normalized = write_normalized_episode_artifact(run, facts)
    release = write_release_decision_artifact(
        run,
        validate_episode_release(facts, q54_profile),
    )
    raw = run / "raw" / "rosbag2"
    raw.mkdir(parents=True)
    (raw / "episode.mcap").write_bytes(b"synthetic-mcap-contract")
    for name in ("manifest.json", "recorder.json", "receipt.json", "checksums.sha256"):
        (run / name).write_text(f"{name}\n", encoding="utf-8")

    policy = load_policy_episode(run)
    quality = build_quality_report(run, q54_profile)
    bundle = write_episode_bundle(
        run,
        collection_id="mini-v1",
        dataset_profile_id="isaac_nero_hand2_triview_q54_mini_dataset_v1",
        dataset_profile_sha256=policy.vision.provenance.dataset_profile_sha256,
        deployment_hash="1" * 64,
        session_hash="2" * 64,
        assembly_hash="3" * 64,
        workcell_hash="4" * 64,
    )

    registry = CollectionRegistry(
        tmp_path,
        "artifacts/collections/mini-v1",
        collection_id="mini-v1",
    )
    registered = registry.register(run.name, run)
    accepted = registry.accept(
        run.name,
        release_decision_sha256=release.decision_sha256,
    )
    export_root = tmp_path / "artifacts" / "datasets" / "revision-001"
    export_root.mkdir(parents=True)
    registry.record_export(
        revision_id="revision-001",
        dataset_root=export_root,
        manifest_sha256="5" * 64,
        episode_ids=(run.name,),
    )
    rejected = registry.reject(run.name, reason="operator_rejected_after_review")
    restored = registry.restore(run.name)

    assert normalized.root.is_dir()
    assert quality.root.is_dir()
    assert validate_episode_bundle(run) == bundle
    assert registered.disposition is EpisodeDisposition.CANDIDATE
    assert accepted.disposition is EpisodeDisposition.ACCEPTED
    assert rejected.disposition is EpisodeDisposition.REJECTED
    assert restored.disposition is EpisodeDisposition.CANDIDATE
    assert registry.stale_exports_for(run.name)[0].revision_id == "revision-001"
