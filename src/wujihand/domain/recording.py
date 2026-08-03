"""Transport-neutral contracts for raw teleoperation recording facts.

These contracts deliberately contain no quality metrics.  They preserve the
causal inputs, intermediate control results, applied targets, backend feedback,
and raw stage timestamps needed by an offline analyzer.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
import re
from typing import Final, Iterable


TELEOPERATION_TICK_TRACE_SCHEMA: Final = "wujihand.teleoperation_tick_trace.v1"
SCENE_RIGID_BODY_STATE_SCHEMA: Final = "wujihand.scene_rigid_body_state.v1"
RUN_RECORDING_STATUS_SCHEMA: Final = "wujihand.run_recording_status.v1"
RUN_MANIFEST_SCHEMA: Final = "wujihand.teleoperation_run_manifest.v1"
RUN_RECEIPT_SCHEMA: Final = "wujihand.teleoperation_run_receipt.v1"

_TOKEN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/+-]{0,127}$")
_RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+-]{0,127}$")


def validate_recording_token(value: object, *, field: str) -> str:
    if type(value) is not str or _TOKEN.fullmatch(value) is None:
        raise ValueError(f"{field} must be a bounded recording-safe identifier")
    return value


def validate_run_id(value: object, *, field: str = "run_id") -> str:
    if type(value) is not str or _RUN_ID.fullmatch(value) is None or ".." in value:
        raise ValueError(f"{field} must be a flat recording run identifier")
    return value


def _bounded_string(value: object, *, field: str) -> str:
    if type(value) is not str or not value or len(value) > 128:
        raise ValueError(f"{field} must be a bounded non-empty string")
    return value


def _non_negative_int(value: object, *, field: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _optional_time(value: object, *, field: str) -> int | None:
    if value is None:
        return None
    return _non_negative_int(value, field=field)


def _finite_vector(
    value: object,
    *,
    size: int,
    field: str,
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(
        value,
        Iterable,
    ):
        raise ValueError(f"{field} must contain {size} finite numbers")
    items = tuple(value)
    if len(items) != size:
        raise ValueError(f"{field} must contain {size} finite numbers")
    result: list[float] = []
    for item in items:
        if isinstance(item, bool) or not isinstance(item, Real):
            raise ValueError(f"{field} must contain {size} finite numbers")
        number = float(item)
        if not math.isfinite(number):
            raise ValueError(f"{field} must contain {size} finite numbers")
        result.append(number)
    return tuple(result)


def _optional_non_negative_float(
    value: object,
    *,
    field: str,
) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field} must be finite, non-negative or None")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{field} must be finite, non-negative or None")
    return result


@dataclass(frozen=True, slots=True, kw_only=True)
class SourceSelectionTrace:
    """One source observation selected by a control tick.

    The full Tracker pose or Glove 21-landmark payload remains in its typed raw
    input topic.  This record supplies the exact identity needed to join that
    payload to a control tick without timestamp guessing.
    """

    source_id: str
    producer_instance: str
    transport_epoch: int
    sequence: int
    source_time_ns: int | None
    receive_time_ns: int
    callback_time_ns: int

    def __post_init__(self) -> None:
        for field in ("source_id", "producer_instance"):
            object.__setattr__(
                self,
                field,
                validate_recording_token(getattr(self, field), field=field),
            )
        for field in (
            "transport_epoch",
            "sequence",
            "receive_time_ns",
            "callback_time_ns",
        ):
            object.__setattr__(
                self,
                field,
                _non_negative_int(getattr(self, field), field=field),
            )
        object.__setattr__(
            self,
            "source_time_ns",
            _optional_time(self.source_time_ns, field="source_time_ns"),
        )
        if self.source_time_ns is not None and (self.source_time_ns > self.receive_time_ns):
            raise ValueError("source_time_ns must not exceed receive_time_ns")
        if self.receive_time_ns > self.callback_time_ns:
            raise ValueError("receive_time_ns must not exceed callback_time_ns")


@dataclass(frozen=True, slots=True, kw_only=True)
class ArmMappingTrace:
    target_position_m: tuple[float, ...] | None
    target_orientation_wxyz: tuple[float, ...] | None
    tracker_delta_m: tuple[float, ...] | None
    workcell_delta_m: tuple[float, ...] | None
    tracker_delta_rotation_wxyz: tuple[float, ...] | None
    workcell_delta_rotation_wxyz: tuple[float, ...] | None
    rotation_delta_rad: float | None
    input_host_time_ns: int | None
    accepted: bool
    translation_clamped: bool
    rotation_clamped: bool
    requires_reference: bool
    reason: str

    def __post_init__(self) -> None:
        vector_sizes = {
            "target_position_m": 3,
            "target_orientation_wxyz": 4,
            "tracker_delta_m": 3,
            "workcell_delta_m": 3,
            "tracker_delta_rotation_wxyz": 4,
            "workcell_delta_rotation_wxyz": 4,
        }
        for field, size in vector_sizes.items():
            value = getattr(self, field)
            if value is not None:
                object.__setattr__(
                    self,
                    field,
                    _finite_vector(value, size=size, field=field),
                )
        object.__setattr__(
            self,
            "rotation_delta_rad",
            _optional_non_negative_float(
                self.rotation_delta_rad,
                field="rotation_delta_rad",
            ),
        )
        object.__setattr__(
            self,
            "input_host_time_ns",
            _optional_time(
                self.input_host_time_ns,
                field="input_host_time_ns",
            ),
        )
        for field in (
            "accepted",
            "translation_clamped",
            "rotation_clamped",
            "requires_reference",
        ):
            if type(getattr(self, field)) is not bool:
                raise ValueError(f"{field} must be a boolean")
        object.__setattr__(
            self,
            "reason",
            _bounded_string(self.reason, field="reason"),
        )
        target_fields = (
            self.target_position_m,
            self.target_orientation_wxyz,
        )
        if self.accepted and any(value is None for value in target_fields):
            raise ValueError("accepted arm mapping requires a target pose")


@dataclass(frozen=True, slots=True, kw_only=True)
class ArmKinematicsTrace:
    succeeded: bool
    solver_reported_success: bool
    candidate_q7_rad: tuple[float, ...] | None
    position_residual_m: float | None
    orientation_residual_rad: float | None
    reason: str

    def __post_init__(self) -> None:
        if type(self.succeeded) is not bool or type(self.solver_reported_success) is not bool:
            raise ValueError("kinematics success fields must be booleans")
        if self.candidate_q7_rad is not None:
            object.__setattr__(
                self,
                "candidate_q7_rad",
                _finite_vector(
                    self.candidate_q7_rad,
                    size=7,
                    field="candidate_q7_rad",
                ),
            )
        if self.succeeded and self.candidate_q7_rad is None:
            raise ValueError("successful kinematics requires q7 candidate")
        for field in ("position_residual_m", "orientation_residual_rad"):
            object.__setattr__(
                self,
                field,
                _optional_non_negative_float(
                    getattr(self, field),
                    field=field,
                ),
            )
        object.__setattr__(
            self,
            "reason",
            _bounded_string(self.reason, field="reason"),
        )


@dataclass(frozen=True, slots=True, kw_only=True)
class HandIntentTrace:
    sequence: int
    q20_rad: tuple[float, ...]
    layout_id: str
    produced_time_ns: int
    retarget_status: str
    retarget_confidence: float
    retarget_model_id: str
    retarget_config_id: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "sequence",
            _non_negative_int(self.sequence, field="sequence"),
        )
        object.__setattr__(
            self,
            "q20_rad",
            _finite_vector(self.q20_rad, size=20, field="q20_rad"),
        )
        object.__setattr__(
            self,
            "produced_time_ns",
            _non_negative_int(
                self.produced_time_ns,
                field="produced_time_ns",
            ),
        )
        for field in (
            "layout_id",
            "retarget_status",
            "retarget_model_id",
            "retarget_config_id",
        ):
            object.__setattr__(
                self,
                field,
                validate_recording_token(getattr(self, field), field=field),
            )
        if isinstance(self.retarget_confidence, bool) or not isinstance(
            self.retarget_confidence,
            Real,
        ):
            raise ValueError("retarget_confidence must be in [0, 1]")
        confidence = float(self.retarget_confidence)
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("retarget_confidence must be in [0, 1]")
        object.__setattr__(self, "retarget_confidence", confidence)


@dataclass(frozen=True, slots=True, kw_only=True)
class RouteDecisionTrace:
    instance_id: str
    group_id: str
    layout_id: str
    command_rad: tuple[float, ...]
    safety_state: str
    reason: str
    position_clamped: bool
    rate_limited: bool

    def __post_init__(self) -> None:
        for field in (
            "instance_id",
            "group_id",
            "layout_id",
            "safety_state",
        ):
            object.__setattr__(
                self,
                field,
                validate_recording_token(getattr(self, field), field=field),
            )
        object.__setattr__(
            self,
            "reason",
            _bounded_string(self.reason, field="reason"),
        )
        if self.group_id == "arm_joints":
            size = 7
        elif self.group_id == "finger_joints":
            size = 20
        else:
            raise ValueError("group_id must be arm_joints or finger_joints")
        object.__setattr__(
            self,
            "command_rad",
            _finite_vector(
                self.command_rad,
                size=size,
                field="command_rad",
            ),
        )
        if type(self.position_clamped) is not bool or type(self.rate_limited) is not bool:
            raise ValueError("decision clamp fields must be booleans")


@dataclass(frozen=True, slots=True, kw_only=True)
class ArmControlTrace:
    source: SourceSelectionTrace | None
    active_source: SourceSelectionTrace | None
    controller_state: str
    controller_reason: str
    reference_epoch: int
    reference_established: bool
    reference_revoked: bool
    mapping: ArmMappingTrace | None
    kinematics: ArmKinematicsTrace | None
    decision: RouteDecisionTrace

    def __post_init__(self) -> None:
        if self.source is not None and type(self.source) is not SourceSelectionTrace:
            raise ValueError("source must be SourceSelectionTrace or None")
        if self.active_source is not None and type(self.active_source) is not SourceSelectionTrace:
            raise ValueError("active_source must be SourceSelectionTrace or None")
        if self.mapping is not None and type(self.mapping) is not ArmMappingTrace:
            raise ValueError("mapping must be ArmMappingTrace or None")
        if self.kinematics is not None and type(self.kinematics) is not ArmKinematicsTrace:
            raise ValueError("kinematics must be ArmKinematicsTrace or None")
        if type(self.decision) is not RouteDecisionTrace:
            raise ValueError("decision must be RouteDecisionTrace")
        if self.decision.group_id != "arm_joints":
            raise ValueError("arm decision must use arm_joints")
        object.__setattr__(
            self,
            "controller_state",
            validate_recording_token(
                self.controller_state,
                field="controller_state",
            ),
        )
        for field in ("controller_reason",):
            object.__setattr__(
                self,
                field,
                _bounded_string(getattr(self, field), field=field),
            )
        object.__setattr__(
            self,
            "reference_epoch",
            _non_negative_int(
                self.reference_epoch,
                field="reference_epoch",
            ),
        )
        if type(self.reference_established) is not bool or type(self.reference_revoked) is not bool:
            raise ValueError("reference transition fields must be booleans")


@dataclass(frozen=True, slots=True, kw_only=True)
class HandControlTrace:
    source: SourceSelectionTrace | None
    active_source: SourceSelectionTrace | None
    intent: HandIntentTrace | None
    intent_is_new: bool
    rejection_reason: str | None
    decision: RouteDecisionTrace

    def __post_init__(self) -> None:
        if self.source is not None and type(self.source) is not SourceSelectionTrace:
            raise ValueError("source must be SourceSelectionTrace or None")
        if self.active_source is not None and type(self.active_source) is not SourceSelectionTrace:
            raise ValueError("active_source must be SourceSelectionTrace or None")
        if self.intent is not None and type(self.intent) is not HandIntentTrace:
            raise ValueError("intent must be HandIntentTrace or None")
        if type(self.intent_is_new) is not bool:
            raise ValueError("intent_is_new must be a boolean")
        if self.intent_is_new and self.intent is None:
            raise ValueError("new hand intent requires an active intent")
        if type(self.decision) is not RouteDecisionTrace:
            raise ValueError("decision must be RouteDecisionTrace")
        if self.decision.group_id != "finger_joints":
            raise ValueError("hand decision must use finger_joints")
        if self.rejection_reason is not None:
            object.__setattr__(
                self,
                "rejection_reason",
                _bounded_string(
                    self.rejection_reason,
                    field="rejection_reason",
                ),
            )


@dataclass(frozen=True, slots=True, kw_only=True)
class TickStageTimes:
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

    def __post_init__(self) -> None:
        fields = (
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
        )
        values = []
        for field in fields:
            value = _non_negative_int(getattr(self, field), field=field)
            object.__setattr__(self, field, value)
            values.append(value)
        if values != sorted(values):
            raise ValueError("tick stage times must be monotonic")


@dataclass(frozen=True, slots=True, kw_only=True)
class TeleoperationTickTrace:
    run_id: str
    tick_id: int
    side: str
    times: TickStageTimes
    pre_feedback_q27_rad: tuple[float, ...]
    applied_target_q27_rad: tuple[float, ...]
    post_feedback_q27_rad: tuple[float, ...]
    arm: ArmControlTrace
    hand: HandControlTrace | None
    schema: str = TELEOPERATION_TICK_TRACE_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != TELEOPERATION_TICK_TRACE_SCHEMA:
            raise ValueError(f"schema must be {TELEOPERATION_TICK_TRACE_SCHEMA!r}")
        object.__setattr__(
            self,
            "run_id",
            validate_run_id(self.run_id),
        )
        object.__setattr__(
            self,
            "tick_id",
            _non_negative_int(self.tick_id, field="tick_id"),
        )
        if self.side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if type(self.times) is not TickStageTimes:
            raise ValueError("times must be TickStageTimes")
        for field in (
            "pre_feedback_q27_rad",
            "applied_target_q27_rad",
            "post_feedback_q27_rad",
        ):
            object.__setattr__(
                self,
                field,
                _finite_vector(
                    getattr(self, field),
                    size=27,
                    field=field,
                ),
            )
        if type(self.arm) is not ArmControlTrace:
            raise ValueError("arm must be ArmControlTrace")
        if self.hand is not None and type(self.hand) is not HandControlTrace:
            raise ValueError("hand must be HandControlTrace or None")


@dataclass(frozen=True, slots=True, kw_only=True)
class SceneRigidBodyState:
    run_id: str
    tick_id: int
    prim_path: str
    recorded_time_ns: int
    position_m: tuple[float, ...]
    quat_wxyz: tuple[float, ...]
    linear_velocity_m_s: tuple[float, ...] | None
    angular_velocity_deg_s: tuple[float, ...] | None
    kinematic_enabled: bool
    schema: str = SCENE_RIGID_BODY_STATE_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != SCENE_RIGID_BODY_STATE_SCHEMA:
            raise ValueError(f"schema must be {SCENE_RIGID_BODY_STATE_SCHEMA!r}")
        object.__setattr__(
            self,
            "run_id",
            validate_run_id(self.run_id),
        )
        object.__setattr__(
            self,
            "tick_id",
            _non_negative_int(self.tick_id, field="tick_id"),
        )
        if type(self.prim_path) is not str or not self.prim_path.startswith("/"):
            raise ValueError("prim_path must be an absolute USD prim path")
        if len(self.prim_path) > 512:
            raise ValueError("prim_path must not exceed 512 characters")
        object.__setattr__(
            self,
            "recorded_time_ns",
            _non_negative_int(
                self.recorded_time_ns,
                field="recorded_time_ns",
            ),
        )
        object.__setattr__(
            self,
            "position_m",
            _finite_vector(self.position_m, size=3, field="position_m"),
        )
        object.__setattr__(
            self,
            "quat_wxyz",
            _finite_vector(self.quat_wxyz, size=4, field="quat_wxyz"),
        )
        for field in (
            "linear_velocity_m_s",
            "angular_velocity_deg_s",
        ):
            value = getattr(self, field)
            if value is not None:
                object.__setattr__(
                    self,
                    field,
                    _finite_vector(value, size=3, field=field),
                )
        if type(self.kinematic_enabled) is not bool:
            raise ValueError("kinematic_enabled must be a boolean")


class RunRecordingState(str, Enum):
    STARTED = "started"
    CONSUMER_COMPLETED = "consumer_completed"
    COMPLETE = "complete"
    INCOMPLETE = "incomplete"


@dataclass(frozen=True, slots=True, kw_only=True)
class RunRecordingStatus:
    run_id: str
    state: RunRecordingState
    reason: str
    host_time_ns: int
    schema: str = RUN_RECORDING_STATUS_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != RUN_RECORDING_STATUS_SCHEMA:
            raise ValueError(f"schema must be {RUN_RECORDING_STATUS_SCHEMA!r}")
        object.__setattr__(
            self,
            "run_id",
            validate_run_id(self.run_id),
        )
        if type(self.state) is not RunRecordingState:
            raise ValueError("state must be a RunRecordingState")
        object.__setattr__(
            self,
            "reason",
            _bounded_string(self.reason, field="reason"),
        )
        object.__setattr__(
            self,
            "host_time_ns",
            _non_negative_int(self.host_time_ns, field="host_time_ns"),
        )


__all__ = [
    "RUN_MANIFEST_SCHEMA",
    "RUN_RECEIPT_SCHEMA",
    "RUN_RECORDING_STATUS_SCHEMA",
    "SCENE_RIGID_BODY_STATE_SCHEMA",
    "TELEOPERATION_TICK_TRACE_SCHEMA",
    "ArmControlTrace",
    "ArmKinematicsTrace",
    "ArmMappingTrace",
    "HandControlTrace",
    "HandIntentTrace",
    "RouteDecisionTrace",
    "RunRecordingState",
    "RunRecordingStatus",
    "SceneRigidBodyState",
    "SourceSelectionTrace",
    "TeleoperationTickTrace",
    "TickStageTimes",
    "validate_recording_token",
    "validate_run_id",
]
