from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest

from wujihand.dataset.normalized import write_normalized_episode_artifact
from wujihand.dataset.quality import build_quality_report
from wujihand.dataset.release import validate_episode_release
from wujihand.dataset.release_artifact import write_release_decision_artifact
from wujihand.domain.dataset_recording import (
    DynamicRigidBodyTruth,
    SimulationStateFrame,
)

from test_dataset_policy import _episode as policy_episode
from test_dataset_release import _episode as release_episode


def _with_dynamic_objects(
    frame: SimulationStateFrame,
) -> SimulationStateFrame:
    bodies = tuple(
        DynamicRigidBodyTruth(
            logical_object_id=logical_id,
            prim_path=f"/World/Environment/task/{logical_id}",
            position_m=(x + frame.control_index * 0.01, y, z),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
            linear_velocity_m_s=(0.01, 0.0, 0.0),
            angular_velocity_rad_s=(0.0, 0.0, 0.0),
            sleeping=False,
            kinematic=False,
            valid=True,
        )
        for logical_id, x, y, z in (
            ("banana", 0.1, 0.5, 0.22),
            ("bowl", -0.15, 0.56, 0.23),
        )
    )
    return SimulationStateFrame.create(
        run_id=frame.run_id,
        episode_id=frame.episode_id,
        control_index=frame.control_index,
        tick_id=frame.tick_id,
        phase=frame.phase,
        simulation_time_s=frame.simulation_time_s,
        physics_boundary_index=frame.physics_boundary_index,
        q54_rad=frame.q54_rad,
        qdot54_rad_s=frame.qdot54_rad_s,
        rigid_bodies=bodies,
        kinematic_links=frame.kinematic_links,
        expected_rigid_body_count=len(bodies),
        expected_kinematic_link_count=frame.expected_kinematic_link_count,
    )


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
    assert summary["frame_count"] == 4
    assert summary["camera_frame_counts"] == {
        "scene_rgb": 4,
        "left_wrist_rgb": 4,
        "right_wrist_rgb": 4,
    }
    assert "success" not in json.dumps(summary).lower()
    assert (first.root / "plots" / "q54_groups.svg").is_file()
    assert (first.root / "plots" / "tracking_error.svg").is_file()
    assert (first.root / "plots" / "input_age.svg").is_file()
    assert (first.root / "plots" / "camera_motion.svg").is_file()
    assert (first.root / "plots" / "vision_samples.html").is_file()
    assert (first.root / "camera_metrics.csv").is_file()
    assert (first.root / "group_metrics.csv").is_file()
    assert summary["control_timing"]["observed_control_hz"] == pytest.approx(30.0)
    assert summary["object_metrics"] == {}


def test_quality_report_keeps_metrics_for_every_scene_object(
    tmp_path: Path,
) -> None:
    facts, profile = release_episode()
    ticks = []
    for tick in facts.ticks:
        pre = _with_dynamic_objects(tick.pre_action_frame)
        post = _with_dynamic_objects(tick.post_action_frame)
        ticks.append(
            replace(
                tick,
                transition=replace(
                    tick.transition,
                    pre_action_state_digest=pre.payload_digest_sha256,
                ),
                pre_action_frame=pre,
                post_action_frame=post,
            )
        )
    facts = replace(facts, ticks=tuple(ticks))
    run = policy_episode(
        tmp_path,
        tuple(tick.transition for tick in facts.ticks),
    )
    write_normalized_episode_artifact(run, facts)
    write_release_decision_artifact(run, validate_episode_release(facts, profile))

    report = build_quality_report(run, profile)
    summary = json.loads((report.root / "summary.json").read_text(encoding="utf-8"))

    assert summary["dynamic_object_ids"] == ["banana", "bowl"]
    assert set(summary["object_metrics"]) == {"banana", "bowl"}
    assert all(
        metrics["sample_count"] == 4
        for metrics in summary["object_metrics"].values()
    )
