from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from wujihand.dataset.alignment import RawTransition
from wujihand.dataset.profile import Q54JointProfile, load_q54_joint_profile
from wujihand.dataset.release import (
    REQUIRED_ROUTE_FACTS,
    ControlTickFacts,
    NormalizedEpisodeFacts,
    SourceEpochFact,
    evaluate_rgb_frame_grid,
    validate_episode_release,
)
from wujihand.domain.dataset_recording import (
    DatasetEpisodeBoundary,
    DatasetEpisodeEvent,
    DatasetSourceMode,
    SimulationFramePhase,
    SimulationStateFrame,
)


ROOT = Path(__file__).parents[2]
PROFILE_PATH = "configs/profiles/isaac_nero_hand2_q54_dataset_v1.yaml"


def _frame(index: int, phase: SimulationFramePhase) -> SimulationStateFrame:
    value = float(index if phase is SimulationFramePhase.PRE_ACTION else index + 1)
    simulation_time = (index + (phase is SimulationFramePhase.POST_ACTION)) / 60.0
    return SimulationStateFrame.create(
        run_id="episode-001",
        episode_id="episode-001",
        control_index=index,
        tick_id=index,
        phase=phase,
        simulation_time_s=simulation_time,
        physics_boundary_index=2 * index + (2 if phase is SimulationFramePhase.POST_ACTION else 0),
        q54_rad=(value,) * 54,
        qdot54_rad_s=(0.0,) * 54,
        rigid_bodies=(),
        kinematic_links=(),
        expected_rigid_body_count=0,
        expected_kinematic_link_count=0,
    )


def _tick(index: int) -> ControlTickFacts:
    pre = _frame(index, SimulationFramePhase.PRE_ACTION)
    post = _frame(index, SimulationFramePhase.POST_ACTION)
    transition = RawTransition(
        run_id="episode-001",
        control_index=index,
        tick_id=index,
        simulation_time_before_s=index / 60.0,
        simulation_time_after_s=(index + 1) / 60.0,
        pre_feedback_q54_rad=(float(index),) * 54,
        applied_target_q54_rad=(float(index) + 0.5,) * 54,
        post_feedback_q54_rad=(float(index + 1),) * 54,
        pre_action_state_digest=pre.payload_digest_sha256,
    )
    return ControlTickFacts(
        transition=transition,
        tick_time_ns=1_000_000_000 + index * 16_666_667,
        schedule_slot=index,
        missed_control_periods_before_tick=0,
        physics_substep_indices=(2 * index, 2 * index + 1),
        route_fact_keys=REQUIRED_ROUTE_FACTS,
        source_epochs=(
            SourceEpochFact("tracker_left", "tracker-left-p1", 1),
            SourceEpochFact("tracker_right", "tracker-right-p1", 1),
            SourceEpochFact("glove_left", "glove-left-p1", 1),
            SourceEpochFact("glove_right", "glove-right-p1", 1),
        ),
        comparable_input_age_ms=(
            ("tracker_left", 4.0),
            ("tracker_right", 4.0),
            ("glove_left", 6.0),
            ("glove_right", 6.0),
        ),
        pre_action_frame=pre,
        post_action_frame=post,
    )


def _boundary(event: DatasetEpisodeEvent, final: int) -> DatasetEpisodeBoundary:
    indexed = event in {DatasetEpisodeEvent.STOP_REQUESTED, DatasetEpisodeEvent.CLOSED}
    return DatasetEpisodeBoundary(
        run_id="episode-001",
        episode_id="episode-001",
        collection_id="mini-v1",
        event=event,
        reason=event.value,
        host_time_ns=100 + tuple(DatasetEpisodeEvent).index(event),
        control_index=final if indexed else None,
        tick_id=final if indexed else None,
        simulation_time_s=(final + 1) / 60.0 if indexed else None,
        recorder_ready=True,
        inputs_ready=True,
        references_ready=True,
        scene_settled=True,
        source_mode=DatasetSourceMode.LIVE_TELEOPERATION,
        dataset_eligible=True,
        requested_signal=2 if event is DatasetEpisodeEvent.STOP_REQUESTED else None,
        effective_final_control_index=final if indexed else None,
    )


def _episode(*, count: int = 4) -> tuple[NormalizedEpisodeFacts, Q54JointProfile]:
    profile = load_q54_joint_profile(ROOT, PROFILE_PATH)
    ticks = tuple(_tick(index) for index in range(count))
    final = count - 1
    facts = NormalizedEpisodeFacts(
        run_id="episode-001",
        boundaries=tuple(_boundary(event, final) for event in DatasetEpisodeEvent),
        ticks=ticks,
        q54_profile_id=profile.profile_id,
        q54_profile_sha256=profile.file_sha256,
        q54_runtime_names=profile.canonical_names,
        artifact_complete=True,
        checksums_verified=True,
        recorder_inventory_complete=True,
        unknown_schemas=(),
        fixture_translation_drift_m=0.0,
        fixture_rotation_drift_rad=0.0,
    )
    return facts, profile


def test_complete_episode_passes_every_release_gate() -> None:
    facts, profile = _episode()

    decision = validate_episode_release(facts, profile)

    assert decision.passed is True
    assert decision.grade == "strict_qualified"
    assert decision.rejection_reasons == ()
    assert all(gate.passed for gate in decision.gates)


def test_normalized_episode_facts_strict_round_trip() -> None:
    facts, _ = _episode()

    assert NormalizedEpisodeFacts.from_mapping(facts.to_mapping()) == facts


def test_schedule_miss_and_missing_q21_fail_with_machine_reasons() -> None:
    facts, profile = _episode()
    bad_tick = replace(
        facts.ticks[1],
        missed_control_periods_before_tick=1,
        route_fact_keys=frozenset(REQUIRED_ROUTE_FACTS - {"left.glove.q21_selected"}),
    )

    decision = validate_episode_release(
        replace(facts, ticks=(facts.ticks[0], bad_tick, *facts.ticks[2:])),
        profile,
    )

    assert decision.passed is False
    assert "q21_q20_q7_or_q27_fact_missing" in decision.rejection_reasons
    assert (
        "control_schedule_gap_exceeds_budget_or_hits_critical_motion"
        in decision.rejection_reasons
    )


def _declare_one_schedule_miss(
    facts: NormalizedEpisodeFacts,
    *,
    control_index: int,
) -> NormalizedEpisodeFacts:
    period_ns = 16_666_667
    ticks = tuple(
        replace(
            tick,
            tick_time_ns=(
                tick.tick_time_ns + (period_ns if index >= control_index else 0)
            ),
            schedule_slot=(
                tick.schedule_slot + (1 if index >= control_index else 0)
            ),
            missed_control_periods_before_tick=(1 if index == control_index else 0),
        )
        for index, tick in enumerate(facts.ticks)
    )
    return replace(facts, ticks=ticks)


def test_isolated_schedule_miss_within_budget_is_usable_warning() -> None:
    facts, profile = _episode(count=240)

    decision = validate_episode_release(
        _declare_one_schedule_miss(facts, control_index=120),
        profile,
    )

    assert decision.passed is True
    assert decision.grade == "usable_with_warnings"
    assert decision.rejection_reasons == ()
    assert decision.warning_reasons == (
        "control_schedule_gap_within_warning_budget",
    )
    gate = next(item for item in decision.gates if item.name == "control_schedule_gaps")
    assert gate.observed["missed_fraction"] == pytest.approx(1.0 / 241.0)


def test_schedule_miss_above_fraction_budget_is_rejected() -> None:
    facts, profile = _episode(count=100)

    decision = validate_episode_release(
        _declare_one_schedule_miss(facts, control_index=50),
        profile,
    )

    assert decision.passed is False
    assert decision.grade == "rejected"
    assert (
        "control_schedule_gap_exceeds_budget_or_hits_critical_motion"
        in decision.rejection_reasons
    )


def test_source_epoch_change_fails_closed() -> None:
    facts, profile = _episode()
    changed = replace(
        facts.ticks[-1],
        source_epochs=(
            *facts.ticks[-1].source_epochs[:-1],
            SourceEpochFact("glove_right", "glove-right-p2", 2),
        ),
    )

    decision = validate_episode_release(
        replace(facts, ticks=(*facts.ticks[:-1], changed)),
        profile,
    )

    assert decision.passed is False
    assert "source_epoch_changed_or_missing" in decision.rejection_reasons


def _rgb_availability(frame_count: int) -> dict[tuple[int, str], tuple[int, int] | None]:
    return {
        (frame_index, camera_id): (frame_index, 30)
        for frame_index in range(frame_count)
        for camera_id in ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb")
    }


def test_rgb_grid_is_strict_when_complete_and_warns_for_one_isolated_gap() -> None:
    availability = _rgb_availability(100)
    strict = evaluate_rgb_frame_grid(
        expected_frame_count=100,
        availability=availability,
    )
    availability[(50, "left_wrist_rgb")] = None
    warning = evaluate_rgb_frame_grid(
        expected_frame_count=100,
        availability=availability,
    )

    assert strict.passed is True
    assert warning.passed is False
    assert warning.severity == "warning"
    assert warning.reason == "rgb_isolated_missing_frames_within_warning_budget"


def test_rgb_grid_rejects_consecutive_gap_or_reference_mismatch() -> None:
    consecutive = _rgb_availability(100)
    consecutive[(50, "scene_rgb")] = None
    consecutive[(51, "scene_rgb")] = None
    mismatch = _rgb_availability(100)
    mismatch[(50, "right_wrist_rgb")] = (1_000, 30)

    consecutive_gate = evaluate_rgb_frame_grid(
        expected_frame_count=100,
        availability=consecutive,
    )
    mismatch_gate = evaluate_rgb_frame_grid(
        expected_frame_count=100,
        availability=mismatch,
    )

    assert consecutive_gate.severity == "hard"
    assert mismatch_gate.severity == "hard"
    assert consecutive_gate.reason == "rgb_gap_budget_or_reference_closure_failed"
    assert mismatch_gate.reason == "rgb_gap_budget_or_reference_closure_failed"
