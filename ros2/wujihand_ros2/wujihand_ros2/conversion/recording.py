"""ROS projection for transport-neutral raw recording contracts."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, TypeVar

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
    TickStageTimes,
)

from ._message import new_message


MessageT = TypeVar("MessageT")


def _zeros(size: int) -> tuple[float, ...]:
    return (0.0,) * size


def _set_source(
    message: Any,
    *,
    prefix: str,
    source: SourceSelectionTrace | None,
) -> None:
    setattr(message, f"has_{prefix}_source", source is not None)
    setattr(message, f"{prefix}_source_id", "" if source is None else source.source_id)
    setattr(
        message,
        f"{prefix}_producer_instance",
        "" if source is None else source.producer_instance,
    )
    setattr(
        message,
        f"{prefix}_transport_epoch",
        0 if source is None else source.transport_epoch,
    )
    setattr(
        message,
        f"{prefix}_sequence",
        0 if source is None else source.sequence,
    )
    source_time = None if source is None else source.source_time_ns
    setattr(message, f"{prefix}_has_source_time", source_time is not None)
    setattr(message, f"{prefix}_source_time_ns", source_time or 0)
    setattr(
        message,
        f"{prefix}_receive_time_ns",
        0 if source is None else source.receive_time_ns,
    )
    setattr(
        message,
        f"{prefix}_callback_time_ns",
        0 if source is None else source.callback_time_ns,
    )


def _source_from_message(message: Any, *, prefix: str) -> SourceSelectionTrace | None:
    if not bool(getattr(message, f"has_{prefix}_source")):
        return None
    return SourceSelectionTrace(
        source_id=str(getattr(message, f"{prefix}_source_id")),
        producer_instance=str(getattr(message, f"{prefix}_producer_instance")),
        transport_epoch=int(getattr(message, f"{prefix}_transport_epoch")),
        sequence=int(getattr(message, f"{prefix}_sequence")),
        source_time_ns=(
            int(getattr(message, f"{prefix}_source_time_ns"))
            if bool(getattr(message, f"{prefix}_has_source_time"))
            else None
        ),
        receive_time_ns=int(getattr(message, f"{prefix}_receive_time_ns")),
        callback_time_ns=int(getattr(message, f"{prefix}_callback_time_ns")),
    )


def teleoperation_tick_trace_to_message(
    trace: TeleoperationTickTrace,
    *,
    factory: Callable[[], MessageT] | None = None,
) -> MessageT:
    message = new_message(factory, class_name="TeleoperationTickTrace")
    target: Any = message
    target.schema = trace.schema
    target.run_id = trace.run_id
    target.tick_id = trace.tick_id
    target.side = trace.side
    target.clock_domain = "host_monotonic"
    for field in (
        "spin_start_ns",
        "spin_end_ns",
        "tick_time_ns",
        "control_start_ns",
        "control_end_ns",
        "apply_start_ns",
        "apply_end_ns",
        "world_step_start_ns",
        "world_step_end_ns",
        "trace_time_ns",
    ):
        setattr(target, field, getattr(trace.times, field))

    arm = trace.arm
    _set_source(target, prefix="tracker", source=arm.source)
    _set_source(
        target,
        prefix="arm_active",
        source=arm.active_source,
    )
    target.arm_controller_state = arm.controller_state
    target.arm_controller_reason = arm.controller_reason
    target.arm_reference_epoch = arm.reference_epoch
    target.arm_reference_established = arm.reference_established
    target.arm_reference_revoked = arm.reference_revoked

    mapping = arm.mapping
    target.has_arm_mapping = mapping is not None
    target.arm_mapping_accepted = False if mapping is None else mapping.accepted
    target.arm_translation_clamped = False if mapping is None else mapping.translation_clamped
    target.arm_rotation_clamped = False if mapping is None else mapping.rotation_clamped
    target.arm_requires_reference = True if mapping is None else mapping.requires_reference
    target.arm_mapping_reason = "" if mapping is None else mapping.reason
    target.has_arm_target_pose = bool(
        mapping is not None
        and mapping.target_position_m is not None
        and mapping.target_orientation_wxyz is not None
    )
    target.arm_target_position_m = (
        _zeros(3)
        if mapping is None or mapping.target_position_m is None
        else mapping.target_position_m
    )
    target.arm_target_quat_wxyz = (
        _zeros(4)
        if mapping is None or mapping.target_orientation_wxyz is None
        else mapping.target_orientation_wxyz
    )
    optional_vectors = (
        ("tracker_delta", "tracker_delta_m", 3),
        ("workcell_delta", "workcell_delta_m", 3),
        (
            "tracker_delta_rotation",
            "tracker_delta_rotation_wxyz",
            4,
        ),
        (
            "workcell_delta_rotation",
            "workcell_delta_rotation_wxyz",
            4,
        ),
    )
    for wire_name, field, size in optional_vectors:
        value = None if mapping is None else getattr(mapping, field)
        setattr(target, f"has_arm_{wire_name}", value is not None)
        setattr(
            target,
            f"arm_{wire_name}_m" if field.endswith("_m") else f"arm_{wire_name}_wxyz",
            _zeros(size) if value is None else value,
        )
    rotation_delta = None if mapping is None else mapping.rotation_delta_rad
    target.has_arm_rotation_delta = rotation_delta is not None
    target.arm_rotation_delta_rad = rotation_delta or 0.0
    input_time = None if mapping is None else mapping.input_host_time_ns
    target.has_arm_input_time = input_time is not None
    target.arm_input_time_ns = input_time or 0

    kinematics = arm.kinematics
    target.has_arm_kinematics = kinematics is not None
    target.arm_ik_succeeded = False if kinematics is None else kinematics.succeeded
    target.arm_solver_reported_success = (
        False if kinematics is None else kinematics.solver_reported_success
    )
    target.arm_kinematics_reason = "" if kinematics is None else kinematics.reason
    candidate = None if kinematics is None else kinematics.candidate_q7_rad
    target.has_arm_q7_candidate = candidate is not None
    target.arm_q7_candidate_rad = _zeros(7) if candidate is None else candidate
    position_residual = None if kinematics is None else kinematics.position_residual_m
    target.has_arm_position_residual = position_residual is not None
    target.arm_position_residual_m = position_residual or 0.0
    orientation_residual = None if kinematics is None else kinematics.orientation_residual_rad
    target.has_arm_orientation_residual = orientation_residual is not None
    target.arm_orientation_residual_rad = orientation_residual or 0.0

    arm_decision = arm.decision
    target.arm_instance_id = arm_decision.instance_id
    target.arm_layout_id = arm_decision.layout_id
    target.arm_command_q7_rad = arm_decision.command_rad
    target.arm_safety_state = arm_decision.safety_state
    target.arm_safety_reason = arm_decision.reason
    target.arm_position_clamped = arm_decision.position_clamped
    target.arm_rate_limited = arm_decision.rate_limited

    hand = trace.hand
    target.has_hand_route = hand is not None
    _set_source(
        target,
        prefix="hand",
        source=None if hand is None else hand.source,
    )
    _set_source(
        target,
        prefix="hand_active",
        source=None if hand is None else hand.active_source,
    )
    intent = None if hand is None else hand.intent
    target.has_hand_intent = intent is not None
    target.hand_intent_is_new = False if hand is None else hand.intent_is_new
    target.hand_intent_sequence = 0 if intent is None else intent.sequence
    target.hand_intent_q20_rad = _zeros(20) if intent is None else intent.q20_rad
    target.hand_intent_layout_id = "" if intent is None else intent.layout_id
    target.hand_intent_produced_time_ns = 0 if intent is None else intent.produced_time_ns
    target.hand_retarget_status = "" if intent is None else intent.retarget_status
    target.hand_retarget_confidence = 0.0 if intent is None else intent.retarget_confidence
    target.hand_retarget_model_id = "" if intent is None else intent.retarget_model_id
    target.hand_retarget_config_id = "" if intent is None else intent.retarget_config_id
    rejection = None if hand is None else hand.rejection_reason
    target.has_hand_rejection = rejection is not None
    target.hand_rejection_reason = rejection or ""
    decision = None if hand is None else hand.decision
    target.hand_instance_id = "" if decision is None else decision.instance_id
    target.hand_layout_id = "" if decision is None else decision.layout_id
    target.hand_command_q20_rad = _zeros(20) if decision is None else decision.command_rad
    target.hand_safety_state = "" if decision is None else decision.safety_state
    target.hand_safety_reason = "" if decision is None else decision.reason
    target.hand_position_clamped = False if decision is None else decision.position_clamped
    target.hand_rate_limited = False if decision is None else decision.rate_limited

    target.pre_feedback_q27_rad = trace.pre_feedback_q27_rad
    target.applied_target_q27_rad = trace.applied_target_q27_rad
    target.post_feedback_q27_rad = trace.post_feedback_q27_rad
    return message


def teleoperation_tick_trace_from_message(
    message: Any,
) -> TeleoperationTickTrace:
    if message.clock_domain != "host_monotonic":
        raise ValueError("teleoperation trace clock must be host_monotonic")
    mapping = None
    if message.has_arm_mapping:
        mapping = ArmMappingTrace(
            target_position_m=(
                tuple(message.arm_target_position_m) if message.has_arm_target_pose else None
            ),
            target_orientation_wxyz=(
                tuple(message.arm_target_quat_wxyz) if message.has_arm_target_pose else None
            ),
            tracker_delta_m=(
                tuple(message.arm_tracker_delta_m) if message.has_arm_tracker_delta else None
            ),
            workcell_delta_m=(
                tuple(message.arm_workcell_delta_m) if message.has_arm_workcell_delta else None
            ),
            tracker_delta_rotation_wxyz=(
                tuple(message.arm_tracker_delta_rotation_wxyz)
                if message.has_arm_tracker_delta_rotation
                else None
            ),
            workcell_delta_rotation_wxyz=(
                tuple(message.arm_workcell_delta_rotation_wxyz)
                if message.has_arm_workcell_delta_rotation
                else None
            ),
            rotation_delta_rad=(
                float(message.arm_rotation_delta_rad) if message.has_arm_rotation_delta else None
            ),
            input_host_time_ns=(
                int(message.arm_input_time_ns) if message.has_arm_input_time else None
            ),
            accepted=bool(message.arm_mapping_accepted),
            translation_clamped=bool(message.arm_translation_clamped),
            rotation_clamped=bool(message.arm_rotation_clamped),
            requires_reference=bool(message.arm_requires_reference),
            reason=str(message.arm_mapping_reason),
        )
    kinematics = None
    if message.has_arm_kinematics:
        kinematics = ArmKinematicsTrace(
            succeeded=bool(message.arm_ik_succeeded),
            solver_reported_success=bool(message.arm_solver_reported_success),
            candidate_q7_rad=(
                tuple(message.arm_q7_candidate_rad) if message.has_arm_q7_candidate else None
            ),
            position_residual_m=(
                float(message.arm_position_residual_m)
                if message.has_arm_position_residual
                else None
            ),
            orientation_residual_rad=(
                float(message.arm_orientation_residual_rad)
                if message.has_arm_orientation_residual
                else None
            ),
            reason=str(message.arm_kinematics_reason),
        )
    arm = ArmControlTrace(
        source=_source_from_message(message, prefix="tracker"),
        active_source=_source_from_message(
            message,
            prefix="arm_active",
        ),
        controller_state=str(message.arm_controller_state),
        controller_reason=str(message.arm_controller_reason),
        reference_epoch=int(message.arm_reference_epoch),
        reference_established=bool(message.arm_reference_established),
        reference_revoked=bool(message.arm_reference_revoked),
        mapping=mapping,
        kinematics=kinematics,
        decision=RouteDecisionTrace(
            instance_id=str(message.arm_instance_id),
            group_id="arm_joints",
            layout_id=str(message.arm_layout_id),
            command_rad=tuple(message.arm_command_q7_rad),
            safety_state=str(message.arm_safety_state),
            reason=str(message.arm_safety_reason),
            position_clamped=bool(message.arm_position_clamped),
            rate_limited=bool(message.arm_rate_limited),
        ),
    )
    hand = None
    if message.has_hand_route:
        intent = None
        if message.has_hand_intent:
            intent = HandIntentTrace(
                sequence=int(message.hand_intent_sequence),
                q20_rad=tuple(message.hand_intent_q20_rad),
                layout_id=str(message.hand_intent_layout_id),
                produced_time_ns=int(message.hand_intent_produced_time_ns),
                retarget_status=str(message.hand_retarget_status),
                retarget_confidence=float(message.hand_retarget_confidence),
                retarget_model_id=str(message.hand_retarget_model_id),
                retarget_config_id=str(message.hand_retarget_config_id),
            )
        hand = HandControlTrace(
            source=_source_from_message(message, prefix="hand"),
            active_source=_source_from_message(
                message,
                prefix="hand_active",
            ),
            intent=intent,
            intent_is_new=bool(message.hand_intent_is_new),
            rejection_reason=(
                str(message.hand_rejection_reason) if message.has_hand_rejection else None
            ),
            decision=RouteDecisionTrace(
                instance_id=str(message.hand_instance_id),
                group_id="finger_joints",
                layout_id=str(message.hand_layout_id),
                command_rad=tuple(message.hand_command_q20_rad),
                safety_state=str(message.hand_safety_state),
                reason=str(message.hand_safety_reason),
                position_clamped=bool(message.hand_position_clamped),
                rate_limited=bool(message.hand_rate_limited),
            ),
        )
    return TeleoperationTickTrace(
        schema=str(message.schema),
        run_id=str(message.run_id),
        tick_id=int(message.tick_id),
        side=str(message.side),
        times=TickStageTimes(
            spin_start_ns=int(message.spin_start_ns),
            spin_end_ns=int(message.spin_end_ns),
            tick_time_ns=int(message.tick_time_ns),
            control_start_ns=int(message.control_start_ns),
            control_end_ns=int(message.control_end_ns),
            apply_start_ns=int(message.apply_start_ns),
            apply_end_ns=int(message.apply_end_ns),
            world_step_start_ns=int(message.world_step_start_ns),
            world_step_end_ns=int(message.world_step_end_ns),
            trace_time_ns=int(message.trace_time_ns),
        ),
        pre_feedback_q27_rad=tuple(message.pre_feedback_q27_rad),
        applied_target_q27_rad=tuple(message.applied_target_q27_rad),
        post_feedback_q27_rad=tuple(message.post_feedback_q27_rad),
        arm=arm,
        hand=hand,
    )


def scene_rigid_body_state_to_message(
    state: SceneRigidBodyState,
    *,
    factory: Callable[[], MessageT] | None = None,
) -> MessageT:
    message = new_message(factory, class_name="SceneRigidBodyState")
    target: Any = message
    target.schema = state.schema
    target.run_id = state.run_id
    target.tick_id = state.tick_id
    target.prim_path = state.prim_path
    target.recorded_time_ns = state.recorded_time_ns
    target.clock_domain = "host_monotonic"
    target.position_m = state.position_m
    target.quat_wxyz = state.quat_wxyz
    target.has_linear_velocity = state.linear_velocity_m_s is not None
    target.linear_velocity_m_s = state.linear_velocity_m_s or _zeros(3)
    target.has_angular_velocity = state.angular_velocity_deg_s is not None
    target.angular_velocity_deg_s = state.angular_velocity_deg_s or _zeros(3)
    target.kinematic_enabled = state.kinematic_enabled
    return message


def scene_rigid_body_state_from_message(message: Any) -> SceneRigidBodyState:
    if message.clock_domain != "host_monotonic":
        raise ValueError("scene state clock must be host_monotonic")
    return SceneRigidBodyState(
        schema=str(message.schema),
        run_id=str(message.run_id),
        tick_id=int(message.tick_id),
        prim_path=str(message.prim_path),
        recorded_time_ns=int(message.recorded_time_ns),
        position_m=tuple(message.position_m),
        quat_wxyz=tuple(message.quat_wxyz),
        linear_velocity_m_s=(
            tuple(message.linear_velocity_m_s) if message.has_linear_velocity else None
        ),
        angular_velocity_deg_s=(
            tuple(message.angular_velocity_deg_s) if message.has_angular_velocity else None
        ),
        kinematic_enabled=bool(message.kinematic_enabled),
    )


def run_recording_status_to_message(
    status: RunRecordingStatus,
    *,
    factory: Callable[[], MessageT] | None = None,
) -> MessageT:
    message = new_message(factory, class_name="RunRecordingStatus")
    target: Any = message
    target.schema = status.schema
    target.run_id = status.run_id
    target.state = status.state.value
    target.reason = status.reason
    target.host_time_ns = status.host_time_ns
    target.clock_domain = "host_monotonic"
    return message


def run_recording_status_from_message(message: Any) -> RunRecordingStatus:
    if message.clock_domain != "host_monotonic":
        raise ValueError("recording status clock must be host_monotonic")
    return RunRecordingStatus(
        schema=str(message.schema),
        run_id=str(message.run_id),
        state=RunRecordingState(str(message.state)),
        reason=str(message.reason),
        host_time_ns=int(message.host_time_ns),
    )


__all__ = [
    "run_recording_status_from_message",
    "run_recording_status_to_message",
    "scene_rigid_body_state_from_message",
    "scene_rigid_body_state_to_message",
    "teleoperation_tick_trace_from_message",
    "teleoperation_tick_trace_to_message",
]
