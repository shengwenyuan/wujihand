from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from wujihand.dataset.alignment import RawTransition, build_exact_30hz_alignment
from wujihand.dataset.artifacts import load_alignment_artifact, write_alignment_artifact


def _alignment():
    rows = []
    for index in range(3):
        rows.append(
            RawTransition(
                run_id="episode-001",
                control_index=index,
                tick_id=index,
                simulation_time_before_s=index / 60.0,
                simulation_time_after_s=(index + 1) / 60.0,
                pre_feedback_q54_rad=(float(index),) * 54,
                applied_target_q54_rad=(float(index) + 0.5,) * 54,
                post_feedback_q54_rad=(float(index + 1),) * 54,
                pre_action_state_digest=hashlib.sha256(str(index).encode()).hexdigest(),
            )
        )
    return build_exact_30hz_alignment(rows)


def test_alignment_artifact_is_atomic_checksummed_and_idempotent(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    alignment = _alignment()

    first = write_alignment_artifact(run, alignment)
    second = write_alignment_artifact(run, alignment)

    assert first == second
    manifest = json.loads((first / "manifest.json").read_text())
    assert manifest["alignment_digest_sha256"] == alignment.digest_sha256
    assert manifest["frame_count"] == 2
    assert len((first / "frames.jsonl").read_text().splitlines()) == 2
    assert not tuple((run / "derived").glob(".alignment-*"))

    loaded = load_alignment_artifact(first, expected_run_id="episode-001")
    assert loaded == alignment


def test_alignment_artifact_refuses_existing_different_output(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    destination = run / "derived" / "alignment"
    destination.mkdir(parents=True)
    (destination / "manifest.json").write_text("{}\n")

    with pytest.raises(FileExistsError, match="different or incomplete"):
        write_alignment_artifact(run, _alignment())


def test_alignment_artifact_round_trips_control_gap_mask(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    rows = tuple(
        RawTransition(
            run_id="episode-001",
            control_index=index,
            tick_id=index,
            simulation_time_before_s=index / 60.0,
            simulation_time_after_s=(index + 1) / 60.0,
            pre_feedback_q54_rad=(float(index),) * 54,
            applied_target_q54_rad=(float(index) + 0.5,) * 54,
            post_feedback_q54_rad=(float(index + 1),) * 54,
            pre_action_state_digest=hashlib.sha256(str(index).encode()).hexdigest(),
        )
        for index in range(5)
    )
    alignment = build_exact_30hz_alignment(
        rows,
        missed_control_periods_before_tick={1: 1},
    )

    output = write_alignment_artifact(run, alignment)
    loaded = load_alignment_artifact(output)

    assert loaded == alignment
    assert json.loads((output / "gap_ticks.json").read_text())["gaps"] == [
        {"control_index": 1, "missing_control_periods_before": 1}
    ]


def test_alignment_artifact_rejects_symlink_run_root(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    link = tmp_path / "linked-episode"
    link.symlink_to(run, target_is_directory=True)

    with pytest.raises(ValueError, match="symbolic link"):
        write_alignment_artifact(link, _alignment())


def test_alignment_loader_rejects_canonical_row_tampering(tmp_path: Path) -> None:
    run = tmp_path / "episode-001"
    run.mkdir()
    output = write_alignment_artifact(run, _alignment())
    frames_path = output / "frames.jsonl"
    rows = [json.loads(line) for line in frames_path.read_text().splitlines()]
    rows[1]["source_control_index"] = 3
    frames_path.write_text(
        "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows)
    )

    with pytest.raises(ValueError, match="checksums differ"):
        load_alignment_artifact(output)
