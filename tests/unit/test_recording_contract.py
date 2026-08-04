from __future__ import annotations

from pathlib import Path
import json
from types import SimpleNamespace

import pytest

from wujihand.domain import (
    ArmControlTrace,
    ArmKinematicsTrace,
    ArmMappingTrace,
    HandControlTrace,
    HandIntentTrace,
    RouteDecisionTrace,
    RunRecordingState,
    RunRecordingStatus,
    SceneRigidBodyState,
    SourceSelectionTrace,
    TeleoperationTickTrace,
    TickExecutionTrace,
    TickStageTimes,
)
from wujihand.runtime import (
    SignalStopRequest,
    consumer_receipt_is_terminal,
    finalize_rosbag_recording,
    run_root,
    write_consumer_receipt,
    write_manifest,
)
from wujihand_ros2.conversion import (
    run_recording_status_from_message,
    run_recording_status_to_message,
    scene_rigid_body_state_from_message,
    scene_rigid_body_state_to_message,
    teleoperation_tick_trace_from_message,
    teleoperation_tick_trace_to_message,
)


def _message() -> SimpleNamespace:
    return SimpleNamespace()


def test_signal_stop_request_latches_first_signal_without_raising() -> None:
    request = SignalStopRequest()

    request(2, object())
    request(15, object())

    assert request.requested is True
    assert request.requested_signal == 2


def _source(source_id: str, sequence: int) -> SourceSelectionTrace:
    return SourceSelectionTrace(
        source_id=source_id,
        producer_instance=f"{source_id}-producer",
        transport_epoch=2,
        sequence=sequence,
        source_time_ns=100,
        receive_time_ns=110,
        callback_time_ns=120,
    )


def _trace() -> TeleoperationTickTrace:
    arm_decision = RouteDecisionTrace(
        instance_id="nero_left",
        group_id="arm_joints",
        layout_id="agilex_nero.q7.v1",
        command_rad=tuple(index * 0.1 for index in range(7)),
        safety_state="tracking",
        reason="tracking",
        position_clamped=False,
        rate_limited=True,
    )
    hand_decision = RouteDecisionTrace(
        instance_id="hand_left",
        group_id="finger_joints",
        layout_id="wuji_hand2.left.q20.v1",
        command_rad=tuple(index * 0.01 for index in range(20)),
        safety_state="tracking",
        reason="tracking",
        position_clamped=False,
        rate_limited=False,
    )
    return TeleoperationTickTrace(
        run_id="fixture-run",
        tick_id=3,
        side="left",
        times=TickStageTimes(
            tick_time_ns=1000,
            snapshot_start_ns=1001,
            snapshot_end_ns=1010,
            control_start_ns=1020,
            control_end_ns=1030,
            apply_start_ns=1040,
            apply_end_ns=1050,
            physics_start_ns=1060,
            physics_end_ns=1080,
            trace_time_ns=1090,
        ),
        execution=TickExecutionTrace(
            control_index=3,
            schedule_slot=3,
            scheduled_control_time_ns=900,
            control_lateness_ns=100,
            missed_control_periods_before_tick=0,
            simulation_time_before_s=1.0,
            simulation_time_after_s=1.0 + 1.0 / 60.0,
            target_effective_start_sim_time_s=1.0,
            target_effective_end_sim_time_s=1.0 + 1.0 / 60.0,
            physics_substep_indices=(6, 7),
            physics_substep_sim_times_s=(
                1.0 + 1.0 / 120.0,
                1.0 + 1.0 / 60.0,
            ),
            physics_substep_start_ns=(1061, 1070),
            physics_substep_end_ns=(1069, 1079),
            rendered=True,
            render_index=1,
        ),
        pre_feedback_q27_rad=(0.0,) * 27,
        applied_target_q27_rad=tuple(index * 0.01 for index in range(27)),
        post_feedback_q27_rad=tuple(index * 0.02 for index in range(27)),
        arm=ArmControlTrace(
            source=_source("tracker_left", 5),
            active_source=_source("tracker_left", 5),
            controller_state="tracking",
            controller_reason="ik_succeeded",
            reference_epoch=1,
            reference_established=False,
            reference_revoked=False,
            mapping=ArmMappingTrace(
                target_position_m=(0.1, 0.2, 0.3),
                target_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
                tracker_delta_m=(0.01, 0.02, 0.03),
                workcell_delta_m=(0.03, 0.02, 0.01),
                tracker_delta_rotation_wxyz=(1.0, 0.0, 0.0, 0.0),
                workcell_delta_rotation_wxyz=(1.0, 0.0, 0.0, 0.0),
                rotation_delta_rad=0.0,
                input_host_time_ns=100,
                accepted=True,
                translation_clamped=False,
                rotation_clamped=False,
                requires_reference=False,
                reason="tracking",
            ),
            kinematics=ArmKinematicsTrace(
                succeeded=True,
                solver_reported_success=True,
                candidate_q7_rad=(0.0,) * 7,
                position_residual_m=0.001,
                orientation_residual_rad=0.002,
                reason="ik_succeeded",
            ),
            decision=arm_decision,
        ),
        hand=HandControlTrace(
            source=_source("glove_left", 7),
            active_source=_source("glove_left", 7),
            intent=HandIntentTrace(
                sequence=4,
                q20_rad=tuple(index * 0.01 for index in range(20)),
                layout_id="wuji_hand2.left.q20.v1",
                produced_time_ns=1025,
                retarget_status="success",
                retarget_confidence=0.9,
                retarget_model_id="wuji_retarget.v1",
                retarget_config_id="left_default.v1",
            ),
            intent_is_new=True,
            rejection_reason=None,
            decision=hand_decision,
        ),
    )


def test_tick_trace_ros_projection_round_trip_preserves_q20_and_q27() -> None:
    trace = _trace()

    message = teleoperation_tick_trace_to_message(
        trace,
        factory=_message,
    )

    assert teleoperation_tick_trace_from_message(message) == trace
    assert trace.hand is not None
    assert trace.hand.intent is not None
    assert tuple(message.hand_intent_q20_rad) == trace.hand.intent.q20_rad
    assert tuple(message.applied_target_q27_rad) == (trace.applied_target_q27_rad)


def test_tick_trace_rejects_non_monotonic_stage_times() -> None:
    with pytest.raises(ValueError, match="stage times"):
        TickStageTimes(
            tick_time_ns=10,
            snapshot_start_ns=11,
            snapshot_end_ns=9,
            control_start_ns=11,
            control_end_ns=12,
            apply_start_ns=13,
            apply_end_ns=14,
            physics_start_ns=15,
            physics_end_ns=16,
            trace_time_ns=17,
        )


def test_scene_and_recording_status_ros_projection_round_trip() -> None:
    scene = SceneRigidBodyState(
        run_id="fixture-run",
        tick_id=2,
        prim_path="/World/Environment/Banana",
        recorded_time_ns=500,
        position_m=(0.1, 0.2, 0.3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_m_s=(0.0, 0.0, 0.0),
        angular_velocity_deg_s=None,
        kinematic_enabled=False,
    )
    status = RunRecordingStatus(
        run_id="fixture-run",
        state=RunRecordingState.STARTED,
        reason="consumer_started",
        host_time_ns=100,
    )

    assert (
        scene_rigid_body_state_from_message(
            scene_rigid_body_state_to_message(scene, factory=_message)
        )
        == scene
    )
    assert (
        run_recording_status_from_message(run_recording_status_to_message(status, factory=_message))
        == status
    )


def test_run_artifact_only_completes_after_receipt_and_mcap(
    tmp_path: Path,
) -> None:
    root = tmp_path / "fixture-run"
    write_manifest(
        root,
        run_id="fixture-run",
        payload={"recording_inventory": {"topics": ["/fixture"]}},
    )
    write_consumer_receipt(
        root,
        run_id="fixture-run",
        state=RunRecordingState.CONSUMER_COMPLETED,
        payload={"completed_ticks": 3},
    )
    (root / "recorder.json").write_text(
        json.dumps(
            {
                "schema": "wujihand.rosbag2_recorder.v1",
                "run_id": "fixture-run",
            }
        )
        + "\n"
    )
    raw = root / "raw" / "rosbag2"
    raw.mkdir(parents=True)
    (raw / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (raw / "trace_0.mcap").write_bytes(b"fixture")

    receipt = finalize_rosbag_recording(
        root,
        run_id="fixture-run",
        recorder_exit_code=0,
    )

    assert receipt["state"] == "complete"
    checksums = (root / "checksums.sha256").read_text()
    assert "raw/rosbag2/trace_0.mcap" in checksums


def test_consumer_terminal_receipt_is_visible_before_recorder_finalizes(
    tmp_path: Path,
) -> None:
    root = tmp_path / "fixture-run"

    assert not consumer_receipt_is_terminal(
        root,
        run_id="fixture-run",
    )
    write_consumer_receipt(
        root,
        run_id="fixture-run",
        state=RunRecordingState.CONSUMER_COMPLETED,
        payload={"completed_ticks": 3},
    )

    assert consumer_receipt_is_terminal(
        root,
        run_id="fixture-run",
    )


def test_run_artifact_fails_closed_without_mcap(tmp_path: Path) -> None:
    root = tmp_path / "fixture-run"
    write_manifest(root, run_id="fixture-run", payload={})
    write_consumer_receipt(
        root,
        run_id="fixture-run",
        state=RunRecordingState.CONSUMER_COMPLETED,
        payload={"completed_ticks": 0},
    )

    receipt = finalize_rosbag_recording(
        root,
        run_id="fixture-run",
        recorder_exit_code=0,
    )

    assert receipt["state"] == "incomplete"
    assert receipt["raw_mcap_present"] is False


def test_late_consumer_receipt_cannot_reopen_finalized_incomplete_run(
    tmp_path: Path,
) -> None:
    root = tmp_path / "fixture-run"
    write_manifest(root, run_id="fixture-run", payload={})
    finalize_rosbag_recording(
        root,
        run_id="fixture-run",
        recorder_exit_code=1,
    )
    receipt_path = root / "receipt.json"
    original_receipt = receipt_path.read_bytes()
    original_checksums = (root / "checksums.sha256").read_bytes()

    write_consumer_receipt(
        root,
        run_id="fixture-run",
        state=RunRecordingState.CONSUMER_COMPLETED,
        payload={"completed_ticks": 2},
    )

    receipt = json.loads(receipt_path.read_text())
    assert receipt["state"] == "incomplete"
    assert receipt["recording_finalized"] is True
    assert receipt.get("consumer_state") != "consumer_completed"
    assert receipt_path.read_bytes() == original_receipt
    assert (root / "checksums.sha256").read_bytes() == original_checksums


def test_finalize_is_idempotent_after_artifact_closure(tmp_path: Path) -> None:
    root = tmp_path / "fixture-run"
    write_manifest(root, run_id="fixture-run", payload={})

    first = finalize_rosbag_recording(
        root,
        run_id="fixture-run",
        recorder_exit_code=1,
    )
    receipt_bytes = (root / "receipt.json").read_bytes()
    checksum_bytes = (root / "checksums.sha256").read_bytes()
    second = finalize_rosbag_recording(
        root,
        run_id="fixture-run",
        recorder_exit_code=0,
    )

    assert second == first
    assert (root / "receipt.json").read_bytes() == receipt_bytes
    assert (root / "checksums.sha256").read_bytes() == checksum_bytes


@pytest.mark.parametrize("run_id", ("../escape", "safe/../../escape", "nested/run"))
def test_run_id_cannot_escape_flat_report_directory(
    tmp_path: Path,
    run_id: str,
) -> None:
    with pytest.raises(ValueError, match="flat recording run"):
        run_root(tmp_path, "artifacts/runs", run_id)
