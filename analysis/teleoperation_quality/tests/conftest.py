from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import pytest
import yaml

from teleoperation_quality.artifact import RunArtifact, load_run_artifact
from teleoperation_quality.model import (
    ArmTick,
    BagDataset,
    GloveSample,
    HandTick,
    RecordingStatusRecord,
    SceneRecord,
    SourceRef,
    StageTimes,
    TickRecord,
    TopicObservation,
    TrackerSample,
)

ARM_INDICES = tuple(range(7))
HAND_INDICES = (11, 16, 21, 26, 7, 12, 17, 22, 8, 13, 18, 23, 10, 15, 20, 25, 9, 14, 19, 24)


def _json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")


def rewrite_checksums(root: Path) -> None:
    paths = (
        root / "manifest.json",
        root / "receipt.json",
        root / "recorder.json",
        *(path for path in (root / "raw").rglob("*") if path.is_file()),
    )
    lines = []
    for path in sorted(paths):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {path.relative_to(root).as_posix()}\n")
    (root / "checksums.sha256").write_text("".join(lines), encoding="utf-8")


def make_run_root(tmp_path: Path, *, completed_ticks: int = 4) -> Path:
    root = tmp_path / "fixture-run"
    raw = root / "raw" / "rosbag2"
    raw.mkdir(parents=True)
    manifest = {
        "schema": "wujihand.teleoperation_run_manifest.v1",
        "run_id": root.name,
        "state": "started",
        "clock_domain": "host_monotonic",
        "recording_inventory": {"topics": ["/fixture"]},
        "q27_partitions": {
            side: {
                "arm_indices_q7": list(ARM_INDICES),
                "hand_indices_q20": list(HAND_INDICES),
            }
            for side in ("left", "right")
        },
        "control": {"glove": {"success_landmark_confidence": 0.6}},
        "capabilities": {
            "dynamic_rigid_body_pose": True,
            "task_truth": False,
            "raw_contact": False,
        },
        "scene": {"rigid_body_paths": ["/World/banana"]},
    }
    receipt = {
        "schema": "wujihand.teleoperation_run_receipt.v1",
        "run_id": root.name,
        "state": "complete",
        "consumer_state": "consumer_completed",
        "recorder_exit_code": 0,
        "recording_finalized": True,
        "completed_ticks": completed_ticks,
        "raw_mcap_segments": 1,
        "input_health": {
            f"{kind}_{side}": {
                "inbox": {
                    "accepted": completed_ticks,
                    "overwritten": 0,
                    "rebinds": 0,
                    "rejected_old_epoch": 0,
                    "rejected_old_producer": 0,
                    "rejected_sequence": 0,
                },
                "rejected_contract": 0,
                "rejected_future_time": 0,
                "rejected_identity": 0,
                "lifecycle_resets": 0,
            }
            for kind in ("tracker", "glove")
            for side in ("left", "right")
        },
        "controller_health": {"recording.terminal_status_acked": 1},
    }
    recorder = {
        "schema": "wujihand.rosbag2_recorder.v1",
        "run_id": root.name,
        "exit_code": 0,
        "consumer_terminal_observed": True,
        "state": "exited",
        "storage": "mcap",
        "topics": ["/fixture"],
    }
    _json(root / "manifest.json", manifest)
    _json(root / "receipt.json", receipt)
    _json(root / "recorder.json", recorder)
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 9,
            "storage_identifier": "mcap",
            "message_count": 1,
            "topics_with_message_count": [
                {
                    "topic_metadata": {
                        "name": "/fixture",
                        "type": "fixture/msg/Fact",
                    },
                    "message_count": 1,
                }
            ],
            "relative_file_paths": ["fixture_0.mcap"],
        }
    }
    (raw / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    (raw / "fixture_0.mcap").write_bytes(b"synthetic-mcap-placeholder")
    rewrite_checksums(root)
    return root


def _source(side: str, kind: str, sequence: int, tick_time_ns: int) -> SourceRef:
    source_time = tick_time_ns - 5_000_000 if kind == "tracker" else None
    receive_time = tick_time_ns - (4_000_000 if kind == "tracker" else 6_000_000)
    callback_time = receive_time + 1_000_000
    return SourceRef(
        source_id=f"{kind}_{side}",
        producer_instance=f"{kind}_producer",
        transport_epoch=1,
        sequence=sequence,
        source_time_ns=source_time,
        receive_time_ns=receive_time,
        callback_time_ns=callback_time,
    )


def _q27(arm: tuple[float, ...], hand: tuple[float, ...]) -> tuple[float, ...]:
    values = [0.0] * 27
    for index, value in zip(ARM_INDICES, arm, strict=True):
        values[index] = value
    for index, value in zip(HAND_INDICES, hand, strict=True):
        values[index] = value
    return tuple(values)


def make_dataset(*, tick_count: int = 4) -> BagDataset:
    trackers = []
    gloves = []
    ticks = []
    scenes = []
    base_ns = 1_000_000_000
    for sequence in range(tick_count):
        tick_time_ns = base_ns + sequence * 20_000_000 + 2_000_000
        for side in ("left", "right"):
            tracker_source = _source(side, "tracker", sequence, tick_time_ns)
            glove_source = _source(side, "glove", sequence, tick_time_ns)
            assert tracker_source.source_time_ns is not None
            trackers.append(
                TrackerSample(
                    side=side,
                    source_id=tracker_source.source_id,
                    producer_instance=tracker_source.producer_instance,
                    transport_epoch=tracker_source.transport_epoch,
                    sequence=sequence,
                    host_time_ns=tracker_source.source_time_ns,
                    bag_time_ns=tick_time_ns,
                    pose_valid=True,
                    connected=True,
                    tracking_state="running",
                    quality=1.0,
                    position_m=(0.1 * sequence, 0.0, 1.0),
                    quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                )
            )
            positions = tuple(float(index) / 1000.0 for index in range(63))
            gloves.append(
                GloveSample(
                    side=side,
                    source_id=glove_source.source_id,
                    producer_instance=glove_source.producer_instance,
                    transport_epoch=glove_source.transport_epoch,
                    sequence=sequence,
                    source_time_ns=None,
                    receive_time_ns=glove_source.receive_time_ns,
                    bag_time_ns=tick_time_ns,
                    calibration_id="calibration_v1",
                    transform_id="transform_v1",
                    frame_id=f"{side}_wrist",
                    landmark_layout="mediapipe.hand_landmarks.v1",
                    landmark_valid=(True,) * 21,
                    landmark_positions_m=positions,
                    landmark_confidence=(0.9,) * 21,
                    valid_landmarks=21,
                    minimum_confidence=0.9,
                    median_confidence=0.9,
                )
            )
            arm_command = tuple(0.1 * sequence + index * 0.01 for index in range(7))
            hand_intent = tuple(0.05 * sequence + index * 0.005 for index in range(20))
            hand_command = tuple(value + 0.001 for value in hand_intent)
            applied = _q27(arm_command, hand_command)
            post = tuple(value + 0.01 for value in applied)
            times = StageTimes(
                spin_start_ns=tick_time_ns - 2_000_000,
                spin_end_ns=tick_time_ns - 1_500_000,
                tick_time_ns=tick_time_ns,
                control_start_ns=tick_time_ns + 100_000,
                control_end_ns=tick_time_ns + 600_000,
                apply_start_ns=tick_time_ns + 700_000,
                apply_end_ns=tick_time_ns + 900_000,
                world_step_start_ns=tick_time_ns + 1_000_000,
                world_step_end_ns=tick_time_ns + 1_800_000,
                trace_time_ns=tick_time_ns + 1_900_000,
            )
            arm = ArmTick(
                source=tracker_source,
                active_source=tracker_source,
                controller_state="tracking",
                controller_reason="ik_accepted",
                reference_epoch=1,
                reference_established=sequence == 0,
                reference_revoked=False,
                has_mapping=True,
                mapping_accepted=True,
                translation_clamped=False,
                rotation_clamped=False,
                mapping_requires_reference=False,
                mapping_reason="tracking",
                target_position_m=(0.2, 0.1, 0.8),
                target_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                mapping_input_time_ns=tracker_source.source_time_ns,
                has_kinematics=True,
                ik_succeeded=True,
                solver_reported_success=True,
                kinematics_reason="ik_accepted",
                candidate_q7_rad=arm_command,
                position_residual_m=0.001,
                orientation_residual_rad=0.01,
                command_q7_rad=arm_command,
                safety_state="tracking",
                safety_reason="tracking",
                position_clamped=False,
                rate_limited=False,
            )
            hand = HandTick(
                source=glove_source,
                active_source=glove_source,
                has_intent=True,
                intent_is_new=True,
                intent_sequence=sequence,
                intent_q20_rad=hand_intent,
                intent_layout_id=f"hand2_{side}",
                intent_produced_time_ns=glove_source.callback_time_ns,
                retarget_status="success",
                retarget_confidence=0.9,
                rejection_reason=None,
                command_q20_rad=hand_command,
                safety_state="tracking",
                safety_reason="tracking",
                position_clamped=False,
                rate_limited=False,
            )
            ticks.append(
                TickRecord(
                    side=side,
                    tick_id=sequence,
                    bag_time_ns=tick_time_ns,
                    times=times,
                    arm=arm,
                    hand=hand,
                    pre_feedback_q27_rad=applied,
                    applied_target_q27_rad=applied,
                    post_feedback_q27_rad=post,
                )
            )
        scenes.append(
            SceneRecord(
                tick_id=sequence,
                bag_time_ns=tick_time_ns,
                recorded_time_ns=tick_time_ns + 1_850_000,
                prim_path="/World/banana",
                position_m=(0.01 * sequence, 0.0, 0.8 + 0.02 * sequence),
                linear_velocity_m_s=(0.5, 0.0, 1.0),
                angular_velocity_deg_s=None,
                kinematic_enabled=False,
            )
        )
    return BagDataset(
        topics=(
            TopicObservation(
                topic="/fixture",
                message_type="fixture/msg/Fact",
                count=1,
                validated_count=1,
                first_bag_time_ns=base_ns,
                last_bag_time_ns=base_ns,
            ),
        ),
        trackers=tuple(trackers),
        gloves=tuple(gloves),
        ticks=tuple(ticks),
        scenes=tuple(scenes),
        statuses=(
            RecordingStatusRecord(
                bag_time_ns=base_ns,
                state="started",
                reason="consumer_started",
                host_time_ns=base_ns,
            ),
            RecordingStatusRecord(
                bag_time_ns=base_ns + tick_count * 20_000_000,
                state="consumer_completed",
                reason="consumer_closed",
                host_time_ns=base_ns + tick_count * 20_000_000,
            ),
        ),
    )


@pytest.fixture
def run_root(tmp_path: Path) -> Path:
    return make_run_root(tmp_path)


@pytest.fixture
def artifact(run_root: Path) -> RunArtifact:
    return load_run_artifact(run_root)


@pytest.fixture
def dataset() -> BagDataset:
    return make_dataset()
