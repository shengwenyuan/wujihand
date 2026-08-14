from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from teleoperation_quality.artifact import RunArtifact
from teleoperation_quality.metrics import AnalysisConfig, compute_metrics
from teleoperation_quality.model import BagDataset, TickExecution
from teleoperation_quality.plots import write_plots


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


def test_v2_execution_trace_enforces_120_30_15_schedule(
    artifact: RunArtifact,
    dataset: BagDataset,
    tmp_path: Path,
) -> None:
    ticks = []
    for tick in dataset.ticks:
        simulation_before_s = tick.tick_id / 30.0
        rendered = tick.tick_id % 2 == 1
        desired_tick_time_ns = 1_002_000_000 + round(tick.tick_id * 1_000_000_000 / 30)
        shift_ns = desired_tick_time_ns - tick.times.tick_time_ns
        times = replace(
            tick.times,
            spin_start_ns=tick.times.spin_start_ns + shift_ns,
            spin_end_ns=tick.times.spin_end_ns + shift_ns,
            tick_time_ns=desired_tick_time_ns,
            control_start_ns=tick.times.control_start_ns + shift_ns,
            control_end_ns=tick.times.control_end_ns + shift_ns,
            apply_start_ns=tick.times.apply_start_ns + shift_ns,
            apply_end_ns=tick.times.apply_end_ns + shift_ns,
            world_step_start_ns=tick.times.world_step_start_ns + shift_ns,
            world_step_end_ns=tick.times.world_step_end_ns + shift_ns,
            trace_time_ns=tick.times.trace_time_ns + shift_ns,
        )
        arm_source = replace(
            tick.arm.source,
            source_time_ns=tick.arm.source.source_time_ns + shift_ns,
            receive_time_ns=tick.arm.source.receive_time_ns + shift_ns,
            callback_time_ns=tick.arm.source.callback_time_ns + shift_ns,
        )
        assert tick.hand is not None and tick.hand.source is not None
        hand_source = replace(
            tick.hand.source,
            receive_time_ns=tick.hand.source.receive_time_ns + shift_ns,
            callback_time_ns=tick.hand.source.callback_time_ns + shift_ns,
        )
        ticks.append(
            replace(
                tick,
                schema="wujihand.teleoperation_tick_trace.v2",
                times=times,
                arm=replace(tick.arm, source=arm_source, active_source=arm_source),
                hand=replace(tick.hand, source=hand_source, active_source=hand_source),
                execution=TickExecution(
                    control_index=tick.tick_id,
                    schedule_slot=tick.tick_id,
                    scheduled_control_time_ns=tick.times.tick_time_ns,
                    control_lateness_ns=0,
                    missed_control_periods_before_tick=0,
                    simulation_time_before_s=simulation_before_s,
                    simulation_time_after_s=simulation_before_s + 1.0 / 30.0,
                    target_effective_start_sim_time_s=simulation_before_s,
                    target_effective_end_sim_time_s=simulation_before_s + 1.0 / 30.0,
                    physics_substep_indices=tuple(
                        tick.tick_id * 4 + index for index in range(4)
                    ),
                    physics_substep_sim_times_s=tuple(
                        simulation_before_s + (index + 1) / 120.0 for index in range(4)
                    ),
                    physics_substep_start_ns=tuple(
                        times.world_step_start_ns + index * 200_000 for index in range(4)
                    ),
                    physics_substep_end_ns=tuple(
                        times.world_step_start_ns + (index + 1) * 200_000
                        for index in range(4)
                    ),
                    rendered=rendered,
                    render_index=(tick.tick_id // 2 if rendered else None),
                ),
            )
        )
    v2_input_health = {
        route: {
            **value,
            "inbox": {
                **value["inbox"],
                "drained": value["inbox"]["accepted"],
                "discarded": 0,
                "pending": 0,
            },
        }
        for route, value in artifact.receipt["input_health"].items()
    }
    v2_artifact = replace(
        artifact,
        manifest={
            **artifact.manifest,
            "simulation_timing": {
                "gui": True,
                "physics_hz": 120,
                "control_hz": 30,
                "rendering_hz": 15,
                "physics_substeps_per_control": 4,
                "control_ticks_per_render": 2,
            },
        },
        receipt={
            **artifact.receipt,
            "completed_physics_steps": 16,
            "completed_renders": 2,
            "configured_timing": {
                "physics_hz": 120,
                "control_hz": 30,
                "render_hz": 15,
                "physics_substeps_per_control": 4,
                "control_ticks_per_render": 2,
            },
            "input_health": v2_input_health,
        },
    )

    bundle = compute_metrics(
        v2_artifact,
        replace(dataset, ticks=tuple(ticks)),
        AnalysisConfig(
            expected_control_hz=30.0,
            expected_physics_hz=120.0,
            expected_render_hz=15.0,
            control_rate_tolerance_fraction=0.01,
            p95_tick_interval_limit_ms=35.0,
            gui_p95_tick_interval_limit_ms=35.0,
            p95_comparable_input_age_limit_ms=10.0,
        ),
    )
    gates = {row["name"]: row for row in bundle.tables["gates"]}

    assert bundle.summary["structural_gates_passed"] is True
    assert bundle.summary["planned_targets_passed"] is True
    assert bundle.summary["control"]["execution"]["physics_substep_count"] == 16
    assert bundle.summary["control"]["execution"]["render_effective_hz"] == pytest.approx(15.0)
    assert bundle.summary["config"]["effective_p95_tick_interval_limit_ms"] == 35.0
    assert bundle.summary["control"]["p95_limit_exceedance_ratio"] == 0.0
    assert gates["v2_execution_facts_complete"]["passed"] is True
    assert gates["physics_substep_dt"]["passed"] is True
    assert gates["render_rate"]["passed"] is True
    figures = write_plots(bundle, tmp_path / "v2-plots")
    assert len(figures) == 13
    assert (tmp_path / "v2-plots" / "13_scheduler_physics_render.png").is_file()


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
