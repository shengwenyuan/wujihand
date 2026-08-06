from __future__ import annotations

import json
from pathlib import Path

import pytest

from wujihand.dataset.normalized import write_normalized_episode_artifact
from wujihand.dataset.quality import build_quality_report
from wujihand.dataset.release import validate_episode_release
from wujihand.dataset.release_artifact import write_release_decision_artifact

from test_dataset_policy import _episode as policy_episode
from test_dataset_release import _episode as release_episode


def test_quality_report_is_deterministic_and_contains_no_outcome_label(
    tmp_path: Path,
) -> None:
    facts, profile = release_episode()
    run = policy_episode(
        tmp_path,
        tuple(tick.transition for tick in facts.ticks),
    )
    write_normalized_episode_artifact(run, facts)
    write_release_decision_artifact(run, validate_episode_release(facts, profile))

    first = build_quality_report(run, profile)
    second = build_quality_report(run, profile)
    summary = json.loads((first.root / "summary.json").read_text(encoding="utf-8"))

    assert first == second
    assert summary["frame_count"] == 2
    assert summary["camera_frame_counts"] == {
        "scene_rgb": 2,
        "left_wrist_rgb": 2,
        "right_wrist_rgb": 2,
    }
    assert "success" not in json.dumps(summary).lower()
    assert (first.root / "plots" / "q54_groups.svg").is_file()
    assert (first.root / "plots" / "tracking_error.svg").is_file()
    assert (first.root / "plots" / "input_age.svg").is_file()
    assert (first.root / "plots" / "camera_motion.svg").is_file()
    assert (first.root / "plots" / "vision_samples.html").is_file()
    assert (first.root / "camera_metrics.csv").is_file()
    assert (first.root / "group_metrics.csv").is_file()
    assert summary["control_timing"]["observed_control_hz"] == pytest.approx(60.0)
    assert summary["object_metrics"] == {}
