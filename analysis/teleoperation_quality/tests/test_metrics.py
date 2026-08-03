from __future__ import annotations

from dataclasses import replace

import pytest

from teleoperation_quality.artifact import RunArtifact
from teleoperation_quality.metrics import AnalysisConfig, compute_metrics
from teleoperation_quality.model import BagDataset


def _row(rows: tuple[dict[str, object], ...], *, side: str, chain: str) -> dict[str, object]:
    return next(row for row in rows if row["side"] == side and row["chain"] == chain)


def test_synthetic_causal_chain_has_known_metrics(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> None:
    bundle = compute_metrics(
        artifact,
        dataset,
        AnalysisConfig(
            expected_control_hz=50.0,
            control_rate_tolerance_fraction=0.01,
            p95_tick_interval_limit_ms=20.1,
            p95_comparable_input_age_limit_ms=10.0,
        ),
    )

    assert bundle.summary["structural_gates_passed"] is True
    assert bundle.summary["planned_targets_passed"] is True
    assert bundle.summary["control"]["effective_hz"] == pytest.approx(50.0)
    assert bundle.summary["control"]["planned_period_miss_ratio"] == 0.0
    assert bundle.summary["control"]["p95_limit_exceedance_ratio"] == 0.0
    assert bundle.summary["control"]["dominant_exclusive_stage_by_p95"] == "world_step_ms"
    assert bundle.summary["receipt_inbox_selection_accounted"] is True
    assert bundle.summary["routes"]["four_stream_actionable_coverage"] == pytest.approx(1.0)
    assert bundle.summary["causal_join"]["all_source_references_join_exactly_once"] is True

    arm = _row(bundle.tables["route_metrics"], side="left", chain="arm")
    hand = _row(bundle.tables["route_metrics"], side="left", chain="hand")
    assert arm["input_age_basis"] == "source_time_ns"
    assert arm["new_source_effective_hz"] == pytest.approx(50.0)
    assert arm["new_source_full_window_hz"] == pytest.approx(50.0)
    assert arm["input_age_ms_p95"] == pytest.approx(5.0)
    assert hand["input_age_basis"] == "receive_time_ns"
    assert hand["input_age_ms_p95"] == pytest.approx(6.0)
    assert hand["source_age_ms_count"] == 0
    receipt = next(
        row
        for row in bundle.tables["receipt_input_metrics"]
        if row["kind"] == "glove" and row["side"] == "left"
    )
    assert receipt["trace_selected_count"] == 4
    assert receipt["accepted_selection_accounted"] is True
    assert arm["command_feedback_rmse_rad"] == pytest.approx(0.01)
    assert hand["command_feedback_rmse_rad"] == pytest.approx(0.01)
    assert arm["applied_composition_max_abs_rad"] == 0.0
    assert hand["applied_composition_max_abs_rad"] == 0.0

    assert len(bundle.derived_tables["q27_samples"]) == 8
    glove_sample = next(
        row for row in bundle.derived_tables["source_samples"] if row["kind"] == "glove"
    )
    assert glove_sample["landmark_20_confidence"] == pytest.approx(0.9)
    hand_tick = next(
        row
        for row in bundle.derived_tables["aligned_ticks"]
        if row["side"] == "left" and row["chain"] == "hand"
    )
    assert hand_tick["intent_j19"] is not None
    assert hand_tick["command_j19"] is not None


def test_missing_acquisition_time_stays_missing(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> None:
    bundle = compute_metrics(artifact, dataset)
    hand = _row(bundle.tables["route_metrics"], side="right", chain="hand")

    assert hand["source_age_ms_count"] == 0
    assert hand["source_age_ms_missing"] == 4
    assert hand["input_age_ms_count"] == 4


def test_duration_weighted_coverage_is_not_a_frame_count(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> None:
    ticks = list(dataset.ticks)
    target_index = next(
        index for index, tick in enumerate(ticks) if tick.side == "left" and tick.tick_id == 1
    )
    target = ticks[target_index]
    ticks[target_index] = replace(
        target,
        arm=replace(
            target.arm,
            active_source=None,
            safety_state="degraded",
            safety_reason="synthetic_hold",
        ),
    )

    bundle = compute_metrics(artifact, replace(dataset, ticks=tuple(ticks)))
    arm = _row(bundle.tables["route_metrics"], side="left", chain="arm")

    assert arm["actionable_tick_ratio"] == pytest.approx(0.75)
    assert arm["actionable_coverage"] == pytest.approx(2.0 / 3.0)


def test_broken_causal_join_and_q27_composition_fail_structural_gates(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> None:
    ticks = list(dataset.ticks)
    target = ticks[0]
    assert target.arm.source is not None
    unmatched = replace(target.arm.source, sequence=999)
    applied = list(target.applied_target_q27_rad)
    applied[0] += 0.2
    ticks[0] = replace(
        target,
        arm=replace(target.arm, source=unmatched, active_source=unmatched),
        applied_target_q27_rad=tuple(applied),
    )

    bundle = compute_metrics(artifact, replace(dataset, ticks=tuple(ticks)))
    gates = {row["name"]: row for row in bundle.tables["gates"]}

    assert gates["tick_source_references_join_raw_inputs"]["passed"] is False
    assert gates["q27_composition_exact"]["passed"] is False
    assert gates["q27_composition_exact"]["observed"] == pytest.approx(0.2)
    assert bundle.summary["structural_gates_passed"] is False


def test_receipt_inbox_counts_are_reconciled_with_trace_selection(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> None:
    input_health = {
        route: {
            **value,
            "inbox": {
                **value["inbox"],
                "accepted": 5 if route == "tracker_left" else value["inbox"]["accepted"],
                "overwritten": 1 if route == "tracker_left" else value["inbox"]["overwritten"],
            },
        }
        for route, value in artifact.receipt["input_health"].items()
    }
    accounted_artifact = replace(
        artifact,
        receipt={**artifact.receipt, "input_health": input_health},
    )
    accounted = compute_metrics(accounted_artifact, dataset)

    assert accounted.summary["receipt_inbox_selection_accounted"] is True

    input_health["tracker_left"]["inbox"]["overwritten"] = 0
    unaccounted_artifact = replace(
        artifact,
        receipt={**artifact.receipt, "input_health": input_health},
    )
    unaccounted = compute_metrics(unaccounted_artifact, dataset)
    gates = {row["name"]: row for row in unaccounted.tables["gates"]}

    assert gates["receipt_inbox_selection_accounted"]["passed"] is False
    assert unaccounted.summary["structural_gates_passed"] is False
