from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.dataset.release import ReleaseDecision, ReleaseGateResult
from wujihand.dataset.release_artifact import (
    load_release_decision_artifact,
    write_release_decision_artifact,
)


def _decision(*, passed: bool = True) -> ReleaseDecision:
    return ReleaseDecision(
        run_id="episode-001",
        passed=passed,
        gates=(
            ReleaseGateResult(
                name="artifact_closure",
                passed=passed,
                expected=True,
                observed=passed,
                reason="passed" if passed else "artifact_or_checksum_incomplete",
            ),
        ),
    )


def test_release_decision_mapping_and_atomic_artifact_round_trip(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    decision = _decision()

    first = write_release_decision_artifact(run, decision)
    second = write_release_decision_artifact(run, decision)

    assert first.decision == decision
    assert second.decision_sha256 == first.decision_sha256
    assert ReleaseDecision.from_mapping(decision.to_mapping()) == decision
    assert load_release_decision_artifact(
        run / "derived" / "release",
        expected_run_id=run.name,
    ) == first


def test_release_artifact_rejects_tampering_and_different_overwrite(
    tmp_path: Path,
) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    artifact = write_release_decision_artifact(run, _decision())

    with pytest.raises(FileExistsError, match="different"):
        write_release_decision_artifact(run, _decision(passed=False))

    (artifact.root / "decision.json").write_text("{}\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_release_decision_artifact(
            artifact.root,
            expected_run_id=run.name,
        )
