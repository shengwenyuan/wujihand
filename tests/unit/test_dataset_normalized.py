from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.dataset.normalized import (
    load_normalized_episode_artifact,
    write_normalized_episode_artifact,
)

from test_dataset_release import _episode


def test_normalized_artifact_is_atomic_idempotent_and_strict(tmp_path: Path) -> None:
    facts, _ = _episode()
    run = tmp_path / facts.run_id
    run.mkdir()

    first = write_normalized_episode_artifact(run, facts)
    second = write_normalized_episode_artifact(run, facts)

    assert first.facts == facts
    assert first.facts_sha256 == second.facts_sha256
    assert (
        load_normalized_episode_artifact(
            first.root,
            expected_run_id=facts.run_id,
        )
        == first
    )


def test_normalized_artifact_rejects_tampering(tmp_path: Path) -> None:
    facts, _ = _episode()
    run = tmp_path / facts.run_id
    run.mkdir()
    artifact = write_normalized_episode_artifact(run, facts)
    (artifact.root / "facts.json").write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError):
        load_normalized_episode_artifact(
            artifact.root,
            expected_run_id=facts.run_id,
        )
