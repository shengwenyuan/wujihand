"""ROS 2 Jazzy adapter that normalizes one rosbag2 MCAP into pure records."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import numpy as np

from .camera_integrity import (
    CameraIntegrityAccumulator,
    expected_camera_message_type,
    is_camera_topic,
)
from .model import (
    ArmTick,
    BagDataset,
    GloveSample,
    HandTick,
    RecordingStatusRecord,
    SceneRecord,
    Side,
    SourceRef,
    StageTimes,
    TickExecution,
    TickRecord,
    TopicObservation,
    TrackerSample,
)


@dataclass(slots=True)
class _TopicCounter:
    message_type: str
    count: int = 0
    validated_count: int = 0
    first_ns: int | None = None
    last_ns: int | None = None

    def observe(self, bag_time_ns: int) -> None:
        self.count += 1
        if self.first_ns is None:
            self.first_ns = bag_time_ns
        self.last_ns = bag_time_ns

    def validated(self) -> None:
        self.validated_count += 1


HOST_MONOTONIC = "host_monotonic"
TRACKER_SCHEMA = "wujihand.tracked_rigid_body_sample.v2"
TRACKING_LIFECYCLE_SCHEMA = "wujihand.tracking_lifecycle_event.v1"
GLOVE_ENVELOPE_SCHEMA = "wujihand.ros_hand_observation_envelope.v1"
GLOVE_OBSERVATION_SCHEMA = "wujihand.canonical_hand_observation.v1"
TICK_SCHEMA_V1 = "wujihand.teleoperation_tick_trace.v1"
TICK_SCHEMA_V2 = "wujihand.teleoperation_tick_trace.v2"
TICK_SCHEMAS = frozenset({TICK_SCHEMA_V1, TICK_SCHEMA_V2})
TICK_MESSAGE_V1 = "wujihand_interfaces/msg/TeleoperationTickTrace"
TICK_MESSAGE_V2 = "wujihand_interfaces/msg/TeleoperationTickTraceV2"
SCENE_SCHEMA = "wujihand.scene_rigid_body_state.v1"
STATUS_SCHEMA = "wujihand.run_recording_status.v1"
ROUTE_COMMAND_SCHEMA = "wujihand.ros_route_command.v1"
SAFETY_EVENT_SCHEMA = "wujihand.ros_safety_event.v1"
LANDMARK_LAYOUT = "mediapipe.hand_landmarks.v1"
LANDMARK_NAMES = (
    "wrist",
    "thumb_cmc",
    "thumb_mcp",
    "thumb_ip",
    "thumb_tip",
    "index_finger_mcp",
    "index_finger_pip",
    "index_finger_dip",
    "index_finger_tip",
    "middle_finger_mcp",
    "middle_finger_pip",
    "middle_finger_dip",
    "middle_finger_tip",
    "ring_finger_mcp",
    "ring_finger_pip",
    "ring_finger_dip",
    "ring_finger_tip",
    "pinky_mcp",
    "pinky_pip",
    "pinky_dip",
    "pinky_tip",
)


def _require_equal(value: object, expected: object, *, field: str) -> None:
    if value != expected:
        raise ValueError(f"{field} must be {expected!r}, got {value!r}")


def _require_schema(message: Any, expected: str) -> None:
    _require_equal(str(message.schema), expected, field="message schema")


def _require_run(message: Any, expected_run_id: str) -> None:
    _require_equal(str(message.run_id), expected_run_id, field="message run_id")


def _require_host_clock(message: Any) -> None:
    _require_equal(str(message.clock_domain), HOST_MONOTONIC, field="clock_domain")


def _side(value: object) -> Side:
    if value not in {"left", "right"}:
        raise ValueError(f"recorded side must be left or right, got {value!r}")
    return cast(Side, value)


def _vector(value: object, size: int, *, field: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in cast(Any, value))
    if len(result) != size or not np.isfinite(np.asarray(result, dtype=np.float64)).all():
        raise ValueError(f"{field} must contain {size} finite values")
    return result


def _non_negative_int(value: object, *, field: str) -> int:
    result: int = int(cast(Any, value))
    if result < 0:
        raise ValueError(f"{field} must be non-negative")
    return result


def _non_negative_float(value: object, *, field: str) -> float:
    result: float = float(cast(Any, value))
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{field} must be finite and non-negative")
    return result


def _source(message: Any, prefix: str) -> SourceRef | None:
    if not bool(getattr(message, f"has_{prefix}_source")):
        return None
    has_source_time = bool(getattr(message, f"{prefix}_has_source_time"))
    result = SourceRef(
        source_id=str(getattr(message, f"{prefix}_source_id")),
        producer_instance=str(getattr(message, f"{prefix}_producer_instance")),
        transport_epoch=int(getattr(message, f"{prefix}_transport_epoch")),
        sequence=int(getattr(message, f"{prefix}_sequence")),
        source_time_ns=(
            int(getattr(message, f"{prefix}_source_time_ns")) if has_source_time else None
        ),
        receive_time_ns=int(getattr(message, f"{prefix}_receive_time_ns")),
        callback_time_ns=int(getattr(message, f"{prefix}_callback_time_ns")),
    )
    if result.source_time_ns is not None and result.source_time_ns > result.receive_time_ns:
        raise ValueError(f"{prefix} source time exceeds receive time")
    if result.receive_time_ns > result.callback_time_ns:
        raise ValueError(f"{prefix} receive time exceeds callback time")
    return result


def _tracker(message: Any, *, bag_time_ns: int) -> TrackerSample:
    _require_schema(message, TRACKER_SCHEMA)
    _require_host_clock(message)
    quality = float(message.quality) if bool(message.has_quality) else None
    position = _vector(message.position_m, 3, field="position")
    quaternion = _vector(message.quat_wxyz, 4, field="quaternion")
    if bool(message.pose_valid):
        if not np.isclose(np.linalg.norm(quaternion), 1.0, rtol=0.0, atol=1e-6):
            raise ValueError("valid tracker quaternion must be normalized")
    elif any(position) or any(quaternion):
        raise ValueError("invalid tracker pose must use zero wire sentinels")
    if quality is not None and not 0.0 < quality <= 1.0:
        raise ValueError("tracker quality must be in (0, 1]")
    return TrackerSample(
        side=_side(message.logical_role.removeprefix("operator_")),
        source_id=str(message.stream_id),
        producer_instance=str(message.producer_instance),
        transport_epoch=int(message.transport_epoch),
        sequence=int(message.sequence),
        host_time_ns=int(message.host_time_ns),
        bag_time_ns=bag_time_ns,
        pose_valid=bool(message.pose_valid),
        connected=bool(message.connected),
        tracking_state=str(message.tracking_state),
        quality=quality,
        position_m=cast(tuple[float, float, float], position),
        quaternion_wxyz=cast(tuple[float, float, float, float], quaternion),
    )


def _glove(message: Any, *, bag_time_ns: int) -> GloveSample:
    _require_schema(message, GLOVE_ENVELOPE_SCHEMA)
    _require_host_clock(message)
    _require_equal(
        str(message.observation_schema),
        GLOVE_OBSERVATION_SCHEMA,
        field="glove observation schema",
    )
    _require_equal(str(message.landmark_layout), LANDMARK_LAYOUT, field="landmark layout")
    _require_equal(str(message.position_unit), "m", field="glove position unit")
    if tuple(str(value) for value in message.landmark_names) != LANDMARK_NAMES:
        raise ValueError("glove landmark names must use canonical MediaPipe order")
    validity = tuple(bool(value) for value in message.landmark_valid)
    positions = _vector(message.landmark_positions_m, 63, field="glove landmarks")
    confidence_tuple = _vector(message.landmark_confidence, 21, field="glove confidence")
    confidence = np.asarray(confidence_tuple, dtype=np.float64)
    if len(validity) != 21 or not np.logical_and(confidence >= 0.0, confidence <= 1.0).all():
        raise ValueError("glove landmark validity/confidence must contain 21 finite entries")
    for index, valid in enumerate(validity):
        xyz = positions[index * 3 : index * 3 + 3]
        if not valid and (any(xyz) or confidence_tuple[index] != 0.0):
            raise ValueError("invalid glove landmark must use zero wire sentinels")
    valid_confidence = confidence[np.asarray(validity, dtype=np.bool_)]
    return GloveSample(
        side=_side(message.side),
        source_id=str(message.source_id),
        producer_instance=str(message.producer_instance),
        transport_epoch=int(message.transport_epoch),
        sequence=int(message.sequence),
        source_time_ns=(int(message.source_time_ns) if bool(message.has_source_time) else None),
        receive_time_ns=int(message.receive_time_ns),
        bag_time_ns=bag_time_ns,
        calibration_id=str(message.calibration_id),
        transform_id=str(message.transform_id),
        frame_id=str(message.frame_id),
        landmark_layout=str(message.landmark_layout),
        landmark_valid=validity,
        landmark_positions_m=positions,
        landmark_confidence=confidence_tuple,
        valid_landmarks=sum(validity),
        minimum_confidence=(float(np.min(valid_confidence)) if valid_confidence.size else None),
        median_confidence=(float(np.median(valid_confidence)) if valid_confidence.size else None),
    )


def _tick(message: Any, *, bag_time_ns: int, expected_run_id: str) -> TickRecord:
    schema = str(message.schema)
    if schema not in TICK_SCHEMAS:
        raise ValueError(f"message schema must be one of {sorted(TICK_SCHEMAS)}, got {schema!r}")
    _require_run(message, expected_run_id)
    _require_host_clock(message)
    if schema == TICK_SCHEMA_V1:
        times = StageTimes(
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
        )
        stage_values = (
            times.spin_start_ns,
            times.spin_end_ns,
            times.tick_time_ns,
            times.control_start_ns,
            times.control_end_ns,
            times.apply_start_ns,
            times.apply_end_ns,
            times.world_step_start_ns,
            times.world_step_end_ns,
            times.trace_time_ns,
        )
        execution = None
    else:
        times = StageTimes(
            spin_start_ns=int(message.snapshot_start_ns),
            spin_end_ns=int(message.snapshot_end_ns),
            tick_time_ns=int(message.tick_time_ns),
            control_start_ns=int(message.control_start_ns),
            control_end_ns=int(message.control_end_ns),
            apply_start_ns=int(message.apply_start_ns),
            apply_end_ns=int(message.apply_end_ns),
            world_step_start_ns=int(message.physics_start_ns),
            world_step_end_ns=int(message.physics_end_ns),
            trace_time_ns=int(message.trace_time_ns),
        )
        stage_values = (
            times.tick_time_ns,
            times.spin_start_ns,
            times.spin_end_ns,
            times.control_start_ns,
            times.control_end_ns,
            times.apply_start_ns,
            times.apply_end_ns,
            times.world_step_start_ns,
            times.world_step_end_ns,
            times.trace_time_ns,
        )
        indices = tuple(
            _non_negative_int(value, field="physics substep index")
            for value in message.physics_substep_indices
        )
        simulation_times = _vector(
            message.physics_substep_sim_times_s,
            2,
            field="physics substep simulation times",
        )
        substep_starts = tuple(
            _non_negative_int(value, field="physics substep start")
            for value in message.physics_substep_start_ns
        )
        substep_ends = tuple(
            _non_negative_int(value, field="physics substep end")
            for value in message.physics_substep_end_ns
        )
        if (
            len(indices) != 2
            or indices[1] != indices[0] + 1
            or len(substep_starts) != 2
            or len(substep_ends) != 2
            or any(start > end for start, end in zip(substep_starts, substep_ends, strict=True))
            or substep_ends[0] > substep_starts[1]
        ):
            raise ValueError("tick must contain two consecutive monotonic physics substeps")
        control_index = _non_negative_int(message.control_index, field="control index")
        schedule_slot = _non_negative_int(message.schedule_slot, field="schedule slot")
        scheduled_time_ns = _non_negative_int(
            message.scheduled_control_time_ns,
            field="scheduled control time",
        )
        lateness_ns = _non_negative_int(message.control_lateness_ns, field="control lateness")
        missed_periods = _non_negative_int(
            message.missed_control_periods_before_tick,
            field="missed control periods",
        )
        simulation_before = _non_negative_float(
            message.simulation_time_before_s,
            field="simulation time before",
        )
        simulation_after = _non_negative_float(
            message.simulation_time_after_s,
            field="simulation time after",
        )
        target_start = _non_negative_float(
            message.target_effective_start_sim_time_s,
            field="target-effective start",
        )
        target_end = _non_negative_float(
            message.target_effective_end_sim_time_s,
            field="target-effective end",
        )
        if (
            control_index != int(message.tick_id)
            or schedule_slot < control_index
            or missed_periods > schedule_slot
            or scheduled_time_ns > times.tick_time_ns
            or lateness_ns != times.tick_time_ns - scheduled_time_ns
        ):
            raise ValueError("control schedule fields are inconsistent")
        if not (
            times.world_step_start_ns
            <= substep_starts[0]
            <= substep_ends[1]
            <= times.world_step_end_ns
        ):
            raise ValueError("physics substeps fall outside the physics stage")
        if (
            not (
                0.0
                <= simulation_before
                <= simulation_times[0]
                <= simulation_times[1]
                <= simulation_after
            )
            or target_start != simulation_before
            or target_end != simulation_after
        ):
            raise ValueError("simulation times or target-effective interval are inconsistent")
        rendered = bool(message.rendered)
        has_render_index = bool(message.has_render_index)
        if rendered != has_render_index:
            raise ValueError("render index must be present exactly when rendered")
        execution = TickExecution(
            control_index=control_index,
            schedule_slot=schedule_slot,
            scheduled_control_time_ns=scheduled_time_ns,
            control_lateness_ns=lateness_ns,
            missed_control_periods_before_tick=missed_periods,
            simulation_time_before_s=simulation_before,
            simulation_time_after_s=simulation_after,
            target_effective_start_sim_time_s=target_start,
            target_effective_end_sim_time_s=target_end,
            physics_substep_indices=indices,
            physics_substep_sim_times_s=cast(tuple[float, float], simulation_times),
            physics_substep_start_ns=substep_starts,
            physics_substep_end_ns=substep_ends,
            rendered=rendered,
            render_index=(int(message.render_index) if has_render_index else None),
        )
    if stage_values != tuple(sorted(stage_values)):
        raise ValueError("tick stage timestamps must be monotonic")
    has_target_pose = bool(message.has_arm_target_pose)
    has_candidate = bool(message.has_arm_q7_candidate)
    has_mapping_input_time = bool(message.has_arm_input_time)
    arm = ArmTick(
        source=_source(message, "tracker"),
        active_source=_source(message, "arm_active"),
        controller_state=str(message.arm_controller_state),
        controller_reason=str(message.arm_controller_reason),
        reference_epoch=int(message.arm_reference_epoch),
        reference_established=bool(message.arm_reference_established),
        reference_revoked=bool(message.arm_reference_revoked),
        has_mapping=bool(message.has_arm_mapping),
        mapping_accepted=bool(message.arm_mapping_accepted),
        translation_clamped=bool(message.arm_translation_clamped),
        rotation_clamped=bool(message.arm_rotation_clamped),
        mapping_requires_reference=bool(message.arm_requires_reference),
        mapping_reason=str(message.arm_mapping_reason),
        target_position_m=(
            cast(
                tuple[float, float, float],
                _vector(message.arm_target_position_m, 3, field="arm target position"),
            )
            if has_target_pose
            else None
        ),
        target_quaternion_wxyz=(
            cast(
                tuple[float, float, float, float],
                _vector(
                    message.arm_target_quat_wxyz,
                    4,
                    field="arm target quaternion",
                ),
            )
            if has_target_pose
            else None
        ),
        mapping_input_time_ns=(int(message.arm_input_time_ns) if has_mapping_input_time else None),
        has_kinematics=bool(message.has_arm_kinematics),
        ik_succeeded=bool(message.arm_ik_succeeded),
        solver_reported_success=bool(message.arm_solver_reported_success),
        kinematics_reason=str(message.arm_kinematics_reason),
        candidate_q7_rad=(
            _vector(message.arm_q7_candidate_rad, 7, field="arm IK candidate")
            if has_candidate
            else None
        ),
        position_residual_m=(
            float(message.arm_position_residual_m)
            if bool(message.has_arm_position_residual)
            else None
        ),
        orientation_residual_rad=(
            float(message.arm_orientation_residual_rad)
            if bool(message.has_arm_orientation_residual)
            else None
        ),
        command_q7_rad=_vector(message.arm_command_q7_rad, 7, field="arm command"),
        safety_state=str(message.arm_safety_state),
        safety_reason=str(message.arm_safety_reason),
        position_clamped=bool(message.arm_position_clamped),
        rate_limited=bool(message.arm_rate_limited),
    )
    hand: HandTick | None = None
    if bool(message.has_hand_route):
        has_intent = bool(message.has_hand_intent)
        hand = HandTick(
            source=_source(message, "hand"),
            active_source=_source(message, "hand_active"),
            has_intent=has_intent,
            intent_is_new=bool(message.hand_intent_is_new),
            intent_sequence=int(message.hand_intent_sequence) if has_intent else None,
            intent_q20_rad=(
                _vector(message.hand_intent_q20_rad, 20, field="hand intent")
                if has_intent
                else None
            ),
            intent_layout_id=(str(message.hand_intent_layout_id) if has_intent else None),
            intent_produced_time_ns=(
                int(message.hand_intent_produced_time_ns) if has_intent else None
            ),
            retarget_status=(str(message.hand_retarget_status) if has_intent else None),
            retarget_confidence=(float(message.hand_retarget_confidence) if has_intent else None),
            rejection_reason=(
                str(message.hand_rejection_reason) if bool(message.has_hand_rejection) else None
            ),
            command_q20_rad=_vector(message.hand_command_q20_rad, 20, field="hand command"),
            safety_state=str(message.hand_safety_state),
            safety_reason=str(message.hand_safety_reason),
            position_clamped=bool(message.hand_position_clamped),
            rate_limited=bool(message.hand_rate_limited),
        )
    selected_sources = (
        arm.source,
        arm.active_source,
        None if hand is None else hand.source,
        None if hand is None else hand.active_source,
    )
    if any(
        source is not None and source.callback_time_ns > times.tick_time_ns
        for source in selected_sources
    ):
        raise ValueError("selected source callback exceeds the atomic tick snapshot")
    return TickRecord(
        side=_side(message.side),
        tick_id=int(message.tick_id),
        bag_time_ns=bag_time_ns,
        times=times,
        arm=arm,
        hand=hand,
        pre_feedback_q27_rad=_vector(message.pre_feedback_q27_rad, 27, field="pre feedback"),
        applied_target_q27_rad=_vector(message.applied_target_q27_rad, 27, field="applied target"),
        post_feedback_q27_rad=_vector(message.post_feedback_q27_rad, 27, field="post feedback"),
        schema=schema,
        execution=execution,
    )


def _scene(message: Any, *, bag_time_ns: int, expected_run_id: str) -> SceneRecord:
    _require_schema(message, SCENE_SCHEMA)
    _require_run(message, expected_run_id)
    _require_host_clock(message)
    quaternion = _vector(message.quat_wxyz, 4, field="scene quaternion")
    if not np.isclose(np.linalg.norm(quaternion), 1.0, rtol=0.0, atol=1e-6):
        raise ValueError("scene quaternion must be normalized")
    return SceneRecord(
        tick_id=int(message.tick_id),
        bag_time_ns=bag_time_ns,
        recorded_time_ns=int(message.recorded_time_ns),
        prim_path=str(message.prim_path),
        position_m=cast(
            tuple[float, float, float], _vector(message.position_m, 3, field="scene position")
        ),
        linear_velocity_m_s=(
            cast(
                tuple[float, float, float],
                _vector(message.linear_velocity_m_s, 3, field="scene linear velocity"),
            )
            if bool(message.has_linear_velocity)
            else None
        ),
        angular_velocity_deg_s=(
            cast(
                tuple[float, float, float],
                _vector(message.angular_velocity_deg_s, 3, field="scene angular velocity"),
            )
            if bool(message.has_angular_velocity)
            else None
        ),
        kinematic_enabled=bool(message.kinematic_enabled),
    )


def _status(
    message: Any,
    *,
    bag_time_ns: int,
    expected_run_id: str,
) -> RecordingStatusRecord:
    _require_schema(message, STATUS_SCHEMA)
    _require_run(message, expected_run_id)
    _require_host_clock(message)
    if str(message.state) not in {"started", "consumer_completed"}:
        raise ValueError(f"unexpected recording status state: {message.state!r}")
    return RecordingStatusRecord(
        bag_time_ns=bag_time_ns,
        state=str(message.state),
        reason=str(message.reason),
        host_time_ns=int(message.host_time_ns),
    )


def _expected_message_type(topic: str) -> str:
    camera_type = expected_camera_message_type(topic)
    if camera_type is not None:
        return camera_type
    if topic.endswith(("/input/tracker/left/sample", "/input/tracker/right/sample")):
        return "wujihand_interfaces/msg/TrackedRigidBodySample"
    if topic.endswith("/input/tracker/lifecycle"):
        return "wujihand_interfaces/msg/TrackingLifecycleEvent"
    if topic.endswith(("/input/glove/left/observation", "/input/glove/right/observation")):
        return "wujihand_interfaces/msg/HandObservationEnvelope"
    if topic.endswith("/command"):
        return "wujihand_interfaces/msg/RouteCommand"
    if topic.endswith("/feedback"):
        return "sensor_msgs/msg/JointState"
    if topic.endswith("/safety"):
        return "wujihand_interfaces/msg/SafetyEvent"
    if topic.endswith("/runtime/tick"):
        return TICK_MESSAGE_V1
    if topic.endswith("/scene/rigid_body_state"):
        return "wujihand_interfaces/msg/SceneRigidBodyState"
    if topic.endswith("/recording/status"):
        return "wujihand_interfaces/msg/RunRecordingStatus"
    raise ValueError(f"the analyzer has no schema contract for topic {topic!r}")


def _validate_auxiliary(topic: str, message: Any) -> None:
    if topic.endswith("/input/tracker/lifecycle"):
        _require_schema(message, TRACKING_LIFECYCLE_SCHEMA)
        _require_host_clock(message)
        return
    if topic.endswith("/command"):
        _require_schema(message, ROUTE_COMMAND_SCHEMA)
        _require_host_clock(message)
        size = 7 if "/arm/" in topic else 20
        _vector(message.positions, size, field=f"{topic} positions")
        return
    if topic.endswith("/feedback"):
        size = 7 if "/arm/" in topic else 20
        _vector(message.position, size, field=f"{topic} position")
        return
    if topic.endswith("/safety"):
        _require_schema(message, SAFETY_EVENT_SCHEMA)
        _require_host_clock(message)
        if str(message.safety_state) not in {"disarmed", "tracking", "degraded"}:
            raise ValueError(f"invalid safety state on {topic}")
        return
    raise ValueError(f"no auxiliary validator for topic {topic!r}")


class Ros2BagReader:
    """Read exactly one rosbag2 directory using the sourced ROS installation."""

    def read(
        self,
        rosbag_root: str | Path,
        *,
        expected_run_id: str | None = None,
    ) -> BagDataset:
        try:
            import rosbag2_py  # type: ignore[import-not-found]
            from rclpy.serialization import deserialize_message  # type: ignore[import-not-found]
            from rosidl_runtime_py.utilities import get_message  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 Jazzy and the recorded wujihand_interfaces overlay must be sourced"
            ) from exc

        root = Path(rosbag_root).expanduser().resolve()
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(root), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="",
                output_serialization_format="",
            ),
        )
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        for topic, message_type in topic_types.items():
            expected_type = _expected_message_type(topic)
            accepted_types = (
                frozenset({TICK_MESSAGE_V1, TICK_MESSAGE_V2})
                if topic.endswith("/runtime/tick")
                else frozenset({expected_type})
            )
            if message_type not in accepted_types:
                raise ValueError(
                    f"topic {topic} type is {message_type!r}, "
                    f"expected one of {sorted(accepted_types)!r}"
                )
        counters = {
            topic: _TopicCounter(message_type=message_type)
            for topic, message_type in topic_types.items()
        }
        decoded_types: dict[str, Any] = {}
        trackers: list[TrackerSample] = []
        gloves: list[GloveSample] = []
        ticks: list[TickRecord] = []
        scenes: list[SceneRecord] = []
        statuses: list[RecordingStatusRecord] = []
        camera_integrity = CameraIntegrityAccumulator(expected_run_id=expected_run_id)

        def decode(topic: str, payload: bytes) -> Any:
            if topic not in decoded_types:
                decoded_types[topic] = get_message(topic_types[topic])
            return deserialize_message(payload, decoded_types[topic])

        while reader.has_next():
            topic, payload, bag_time_ns = reader.read_next()
            bag_time = int(bag_time_ns)
            counters[topic].observe(bag_time)
            message = decode(topic, payload)
            if topic.endswith(("/input/tracker/left/sample", "/input/tracker/right/sample")):
                trackers.append(_tracker(message, bag_time_ns=bag_time))
            elif topic.endswith(
                ("/input/glove/left/observation", "/input/glove/right/observation")
            ):
                gloves.append(_glove(message, bag_time_ns=bag_time))
            elif topic.endswith("/runtime/tick"):
                if expected_run_id is None:
                    raise ValueError("expected_run_id is required to validate tick messages")
                expected_schema = (
                    TICK_SCHEMA_V2 if topic_types[topic] == TICK_MESSAGE_V2 else TICK_SCHEMA_V1
                )
                _require_equal(
                    str(message.schema),
                    expected_schema,
                    field="tick wire type/schema pair",
                )
                ticks.append(
                    _tick(
                        message,
                        bag_time_ns=bag_time,
                        expected_run_id=expected_run_id,
                    )
                )
            elif topic.endswith("/scene/rigid_body_state"):
                if expected_run_id is None:
                    raise ValueError("expected_run_id is required to validate scene messages")
                scenes.append(
                    _scene(
                        message,
                        bag_time_ns=bag_time,
                        expected_run_id=expected_run_id,
                    )
                )
            elif topic.endswith("/recording/status"):
                if expected_run_id is None:
                    raise ValueError("expected_run_id is required to validate status messages")
                statuses.append(
                    _status(
                        message,
                        bag_time_ns=bag_time,
                        expected_run_id=expected_run_id,
                    )
                )
            elif is_camera_topic(topic):
                camera_integrity.observe_camera(
                    topic,
                    message,
                    bag_time_ns=bag_time,
                )
            elif topic in {"/tf", "/tf_static"}:
                camera_integrity.observe_tf(
                    topic,
                    message,
                    bag_time_ns=bag_time,
                )
            else:
                _validate_auxiliary(topic, message)
            counters[topic].validated()

        camera_frames, transforms = camera_integrity.finalize(declared_topics=set(topic_types))
        topics = tuple(
            TopicObservation(
                topic=topic,
                message_type=counter.message_type,
                count=counter.count,
                validated_count=counter.validated_count,
                first_bag_time_ns=counter.first_ns,
                last_bag_time_ns=counter.last_ns,
            )
            for topic, counter in sorted(counters.items())
        )
        return BagDataset(
            topics=topics,
            trackers=tuple(trackers),
            gloves=tuple(gloves),
            ticks=tuple(ticks),
            scenes=tuple(scenes),
            statuses=tuple(statuses),
            camera_frames=camera_frames,
            transforms=transforms,
        )


__all__ = ["Ros2BagReader"]
