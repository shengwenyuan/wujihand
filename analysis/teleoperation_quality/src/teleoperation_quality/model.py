"""Normalized, ROS-independent records consumed by the metric layer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

Side = Literal["left", "right"]


@dataclass(frozen=True, slots=True)
class TopicObservation:
    topic: str
    message_type: str
    count: int
    validated_count: int
    first_bag_time_ns: int | None
    last_bag_time_ns: int | None


@dataclass(frozen=True, slots=True)
class SourceRef:
    source_id: str
    producer_instance: str
    transport_epoch: int
    sequence: int
    source_time_ns: int | None
    receive_time_ns: int
    callback_time_ns: int


@dataclass(frozen=True, slots=True)
class TrackerSample:
    side: Side
    source_id: str
    producer_instance: str
    transport_epoch: int
    sequence: int
    host_time_ns: int
    bag_time_ns: int
    pose_valid: bool
    connected: bool
    tracking_state: str
    quality: float | None
    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class GloveSample:
    side: Side
    source_id: str
    producer_instance: str
    transport_epoch: int
    sequence: int
    source_time_ns: int | None
    receive_time_ns: int
    bag_time_ns: int
    calibration_id: str
    transform_id: str
    frame_id: str
    landmark_layout: str
    landmark_valid: tuple[bool, ...]
    landmark_positions_m: tuple[float, ...]
    landmark_confidence: tuple[float, ...]
    valid_landmarks: int
    minimum_confidence: float | None
    median_confidence: float | None


@dataclass(frozen=True, slots=True)
class StageTimes:
    spin_start_ns: int
    spin_end_ns: int
    tick_time_ns: int
    control_start_ns: int
    control_end_ns: int
    apply_start_ns: int
    apply_end_ns: int
    world_step_start_ns: int
    world_step_end_ns: int
    trace_time_ns: int


@dataclass(frozen=True, slots=True)
class ArmTick:
    source: SourceRef | None
    active_source: SourceRef | None
    controller_state: str
    controller_reason: str
    reference_epoch: int
    reference_established: bool
    reference_revoked: bool
    has_mapping: bool
    mapping_accepted: bool
    translation_clamped: bool
    rotation_clamped: bool
    mapping_requires_reference: bool
    mapping_reason: str
    target_position_m: tuple[float, float, float] | None
    target_quaternion_wxyz: tuple[float, float, float, float] | None
    mapping_input_time_ns: int | None
    has_kinematics: bool
    ik_succeeded: bool
    solver_reported_success: bool
    kinematics_reason: str
    candidate_q7_rad: tuple[float, ...] | None
    position_residual_m: float | None
    orientation_residual_rad: float | None
    command_q7_rad: tuple[float, ...]
    safety_state: str
    safety_reason: str
    position_clamped: bool
    rate_limited: bool


@dataclass(frozen=True, slots=True)
class HandTick:
    source: SourceRef | None
    active_source: SourceRef | None
    has_intent: bool
    intent_is_new: bool
    intent_sequence: int | None
    intent_q20_rad: tuple[float, ...] | None
    intent_layout_id: str | None
    intent_produced_time_ns: int | None
    retarget_status: str | None
    retarget_confidence: float | None
    rejection_reason: str | None
    command_q20_rad: tuple[float, ...]
    safety_state: str
    safety_reason: str
    position_clamped: bool
    rate_limited: bool


@dataclass(frozen=True, slots=True)
class TickRecord:
    side: Side
    tick_id: int
    bag_time_ns: int
    times: StageTimes
    arm: ArmTick
    hand: HandTick | None
    pre_feedback_q27_rad: tuple[float, ...]
    applied_target_q27_rad: tuple[float, ...]
    post_feedback_q27_rad: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class SceneRecord:
    tick_id: int
    bag_time_ns: int
    recorded_time_ns: int
    prim_path: str
    position_m: tuple[float, float, float]
    linear_velocity_m_s: tuple[float, float, float] | None
    angular_velocity_deg_s: tuple[float, float, float] | None
    kinematic_enabled: bool


@dataclass(frozen=True, slots=True)
class RecordingStatusRecord:
    bag_time_ns: int
    state: str
    reason: str
    host_time_ns: int


@dataclass(frozen=True, slots=True)
class BagDataset:
    topics: tuple[TopicObservation, ...]
    trackers: tuple[TrackerSample, ...]
    gloves: tuple[GloveSample, ...]
    ticks: tuple[TickRecord, ...]
    scenes: tuple[SceneRecord, ...]
    statuses: tuple[RecordingStatusRecord, ...]
