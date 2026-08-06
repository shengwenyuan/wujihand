from __future__ import annotations

import hashlib
import json
from types import SimpleNamespace

import pytest

from teleoperation_quality.ros2_reader import (
    _dataset_boundary,
    _simulation_state,
    _tick,
)


def v2_tick_message() -> SimpleNamespace:
    return SimpleNamespace(
        schema="wujihand.teleoperation_tick_trace.v2",
        run_id="fixture-run",
        tick_id=0,
        side="left",
        clock_domain="host_monotonic",
        tick_time_ns=1_000,
        snapshot_start_ns=1_001,
        snapshot_end_ns=1_010,
        control_start_ns=1_020,
        control_end_ns=1_030,
        apply_start_ns=1_040,
        apply_end_ns=1_050,
        physics_start_ns=1_060,
        physics_end_ns=1_080,
        trace_time_ns=1_090,
        control_index=0,
        schedule_slot=0,
        scheduled_control_time_ns=900,
        control_lateness_ns=100,
        missed_control_periods_before_tick=0,
        simulation_time_before_s=1.0,
        simulation_time_after_s=1.0 + 1.0 / 60.0,
        target_effective_start_sim_time_s=1.0,
        target_effective_end_sim_time_s=1.0 + 1.0 / 60.0,
        physics_substep_indices=(0, 1),
        physics_substep_sim_times_s=(1.0 + 1.0 / 120.0, 1.0 + 1.0 / 60.0),
        physics_substep_start_ns=(1_061, 1_070),
        physics_substep_end_ns=(1_069, 1_079),
        rendered=False,
        has_render_index=False,
        render_index=0,
        has_tracker_source=False,
        has_arm_active_source=False,
        arm_controller_state="waiting",
        arm_controller_reason="missing_input",
        arm_reference_epoch=0,
        arm_reference_established=False,
        arm_reference_revoked=False,
        has_arm_mapping=False,
        arm_mapping_accepted=False,
        arm_translation_clamped=False,
        arm_rotation_clamped=False,
        arm_requires_reference=True,
        arm_mapping_reason="missing_input",
        has_arm_target_pose=False,
        has_arm_input_time=False,
        has_arm_kinematics=False,
        arm_ik_succeeded=False,
        arm_solver_reported_success=False,
        arm_kinematics_reason="missing_input",
        has_arm_q7_candidate=False,
        has_arm_position_residual=False,
        has_arm_orientation_residual=False,
        arm_command_q7_rad=(0.0,) * 7,
        arm_safety_state="degraded",
        arm_safety_reason="missing_input",
        arm_position_clamped=False,
        arm_rate_limited=False,
        has_hand_route=False,
        pre_feedback_q27_rad=(0.0,) * 27,
        applied_target_q27_rad=(0.0,) * 27,
        post_feedback_q27_rad=(0.0,) * 27,
    )


def test_v2_tick_reader_preserves_schedule_and_substeps() -> None:
    record = _tick(v2_tick_message(), bag_time_ns=2_000, expected_run_id="fixture-run")

    assert record.schema == "wujihand.teleoperation_tick_trace.v2"
    assert record.execution is not None
    assert record.execution.physics_substep_indices == (0, 1)
    assert record.execution.control_lateness_ns == 100


def test_v2_tick_reader_rejects_inconsistent_lateness() -> None:
    message = v2_tick_message()
    message.control_lateness_ns = 99

    with pytest.raises(ValueError, match="schedule fields"):
        _tick(message, bag_time_ns=2_000, expected_run_id="fixture-run")


def test_dataset_boundary_reader_preserves_lifecycle_optionals() -> None:
    message = SimpleNamespace(
        schema="wujihand.dataset_episode_boundary.v1",
        run_id="fixture-run",
        episode_id="fixture-run",
        collection_id="mini-v1",
        event="stop_requested",
        reason="requested_after_complete_control_tick",
        host_time_ns=4_000,
        clock_domain="host_monotonic",
        has_control_index=True,
        control_index=9,
        has_tick_id=True,
        tick_id=9,
        has_simulation_time=True,
        simulation_time_s=0.15,
        recorder_ready=True,
        inputs_ready=True,
        references_ready=True,
        scene_settled=True,
        source_mode="live_teleoperation",
        dataset_eligible=True,
        has_requested_signal=True,
        requested_signal=2,
        has_effective_final_control_index=True,
        effective_final_control_index=9,
    )

    record = _dataset_boundary(message, bag_time_ns=4_100, expected_run_id="fixture-run")

    assert record.event == "stop_requested"
    assert record.control_index == 9
    assert record.requested_signal == 2


def simulation_state_message() -> SimpleNamespace:
    conventions = {
        "state_source_api": "isaac_articulation_usd_physics_v1",
        "world_frame_id": "world",
        "quaternion_order": "wxyz",
        "joint_position_unit": "rad",
        "joint_velocity_unit": "rad_s",
        "angular_velocity_unit": "rad_s",
    }
    payload = {
        "run_id": "fixture-run",
        "episode_id": "fixture-run",
        "control_index": 3,
        "tick_id": 3,
        "phase": "pre_action",
        "simulation_time_s": 0.05,
        "physics_boundary_index": 6,
        **conventions,
        "q54_rad": [0.0] * 54,
        "qdot54_rad_s": [0.1] * 54,
        "rigid_bodies": [],
        "kinematic_links": [],
        "expected_rigid_body_count": 0,
        "expected_kinematic_link_count": 0,
    }
    digest = hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    return SimpleNamespace(
        schema="wujihand.simulation_state_frame.v1",
        **payload,
        payload_digest_sha256=digest,
    )


def test_simulation_state_reader_verifies_conventions_and_digest() -> None:
    message = simulation_state_message()

    record = _simulation_state(message, bag_time_ns=5_000, expected_run_id="fixture-run")

    assert record.phase == "pre_action"
    assert record.qdot54_rad_s == (0.1,) * 54

    message.payload_digest_sha256 = "0" * 64
    with pytest.raises(ValueError, match="digest"):
        _simulation_state(message, bag_time_ns=5_000, expected_run_id="fixture-run")
