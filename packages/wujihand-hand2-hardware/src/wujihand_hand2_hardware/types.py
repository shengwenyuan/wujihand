from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, TypeAlias

# JSON is the package's artifact boundary. Keeping this alias explicit avoids leaking
# SDK objects while allowing ordinary homogeneous Python containers.
JsonValue: TypeAlias = Any
PROJECT_MINIMUM_RESPONSE_RATE_PCT = 85.0


class Side(str, Enum):
    LEFT = "left"
    RIGHT = "right"


class SafetyState(str, Enum):
    DISCONNECTED = "disconnected"
    READ_ONLY = "read_only"
    ARMED = "armed"
    ENABLED = "enabled"
    FAULTED = "faulted"
    ESTOPPED = "estopped"


@dataclass(frozen=True, slots=True)
class DeviceTarget:
    serial: str
    address: str
    side: Side
    expected_firmware: str
    expected_hardware: str | None = None
    expected_sdk: str = "2026.8.3"

    def __post_init__(self) -> None:
        for name, value in (
            ("serial", self.serial),
            ("address", self.address),
            ("expected_firmware", self.expected_firmware),
            ("expected_sdk", self.expected_sdk),
        ):
            if not value.strip():
                raise ValueError(f"{name} must be non-empty")


@dataclass(frozen=True, slots=True)
class DeviceIdentity:
    serial: str
    address: str
    side: Side
    firmware: str
    hardware: str
    sdk: str
    online_joints: int
    device_type: str


@dataclass(frozen=True, slots=True)
class FrameHeader:
    sequence: int
    device_timestamp_us: int


@dataclass(frozen=True, slots=True)
class JointState:
    nid: int
    position_rad: float
    velocity_rad_s: float
    effort_a: float


@dataclass(frozen=True, slots=True)
class StateFrame:
    header: FrameHeader
    joints: tuple[JointState, ...]


@dataclass(frozen=True, slots=True)
class TransportDiagnostics:
    age_ms: int
    e2e_received: int
    e2e_lost: int
    e2e_reordered: int
    e2e_duplicates: int
    e2e_window_loss_x100: int
    rpc_total: int
    rpc_retries: int
    rpc_timeouts: int
    comm_get_failures: int
    sdk_dropped: int


@dataclass(frozen=True, slots=True)
class JointDiagnostics:
    nid: int
    status: str
    current_a: float
    bus_voltage_v: float
    mcu_temperature_c: float
    error_code: int
    response_rate_pct: float
    timeout_total: int
    position_limit_active: bool
    velocity_limit_active: bool
    current_limit_active: bool


@dataclass(frozen=True, slots=True)
class DiagnosticsFrame:
    header: FrameHeader
    joints: tuple[JointDiagnostics, ...]
    transport: TransportDiagnostics


@dataclass(frozen=True, slots=True)
class CommunicationNode:
    slot: int
    node_type: int
    online: bool
    request_total: int
    response_total: int
    timeout_total: int
    response_rate_pct: float
    age_ms: int


@dataclass(frozen=True, slots=True)
class FingerCommunication:
    finger_index: int
    crc_errors: int
    format_errors: int
    uart_errors: int
    error_per_second: float
    nodes: tuple[CommunicationNode, ...]


@dataclass(frozen=True, slots=True)
class CommunicationSnapshot:
    fingers: tuple[FingerCommunication, ...]


@dataclass(frozen=True, slots=True)
class QualificationPolicy:
    duration_s: float
    warmup_s: float = 3.0
    stale_timeout_s: float = 0.5
    idle_sleep_s: float = 0.0001
    temperature_sample_period_s: float | None = None
    max_temperature_rise_c: float | None = None
    max_temperature_c: float | None = None
    minimum_response_rate_pct: float = PROJECT_MINIMUM_RESPONSE_RATE_PCT

    def __post_init__(self) -> None:
        if self.duration_s <= 0:
            raise ValueError("duration_s must be positive")
        if self.warmup_s < 0:
            raise ValueError("warmup_s cannot be negative")
        if self.stale_timeout_s <= 0 or self.idle_sleep_s <= 0:
            raise ValueError("timeouts must be positive")
        if self.temperature_sample_period_s is not None:
            if self.temperature_sample_period_s <= 0:
                raise ValueError("temperature_sample_period_s must be positive")
            if self.temperature_sample_period_s > self.duration_s:
                raise ValueError("temperature sample period cannot exceed duration")
        if self.max_temperature_rise_c is not None and self.max_temperature_rise_c <= 0:
            raise ValueError("max_temperature_rise_c must be positive")
        if not PROJECT_MINIMUM_RESPONSE_RATE_PCT <= self.minimum_response_rate_pct <= 100:
            raise ValueError(
                "minimum_response_rate_pct must be between the project floor "
                f"{PROJECT_MINIMUM_RESPONSE_RATE_PCT:.0f} and 100"
            )


@dataclass(frozen=True, slots=True)
class TemperatureSample:
    elapsed_s: float
    minimum_c: float
    maximum_c: float
    rise_from_baseline_c: float
    maximum_by_nid_c: dict[str, float]

    def as_json(self) -> dict[str, JsonValue]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class CommunicationSample:
    elapsed_s: float
    source: str
    issues: tuple[str, ...]
    gate_issues: tuple[str, ...]
    details: dict[str, JsonValue]

    def as_json(self) -> dict[str, JsonValue]:
        return {
            "schema_revision": "hand2_communication_sample_v1",
            **asdict(self),
        }


@dataclass(frozen=True, slots=True)
class QualificationReport:
    passed: bool
    failures: tuple[str, ...]
    started_at: str
    ended_at: str
    summary: dict[str, JsonValue]

    def as_json(self) -> dict[str, JsonValue]:
        return {
            "passed": self.passed,
            "failures": list(self.failures),
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "summary": self.summary,
        }


@dataclass(frozen=True, slots=True)
class MitParameters:
    kp: float
    kd: float


@dataclass(frozen=True, slots=True)
class ControlReadback:
    effort_limits_a: tuple[float | None, ...]
    mit_parameters: tuple[MitParameters | None, ...]


@dataclass(frozen=True, slots=True)
class JointCommandValue:
    position_rad: float
    velocity_rad_s: float = 0.0
    effort_a: float = 0.0


@dataclass(frozen=True, slots=True)
class JointMotionStep:
    joint_label: str
    delta_rad: float

    def __post_init__(self) -> None:
        if not self.joint_label.strip():
            raise ValueError("joint_label must be non-empty")
        if not math.isfinite(self.delta_rad) or self.delta_rad == 0:
            raise ValueError("delta_rad must be finite and non-zero")


@dataclass(frozen=True, slots=True)
class JointSequencePolicy:
    profile_name: str
    steps: tuple[JointMotionStep, ...]
    preflight_duration_s: float = 30.0
    warmup_s: float = 4.0
    baseline_s: float = 1.0
    ready_hold_s: float = 3.0
    inter_step_hold_s: float = 1.0
    pre_enable_hold_s: float = 0.2
    ramp_s: float = 1.5
    hold_s: float = 0.5
    return_s: float = 1.5
    settle_s: float = 0.5
    post_disable_s: float = 0.5
    command_rate_hz: float = 100.0
    stale_timeout_s: float = 0.1
    idle_sleep_s: float = 0.0001
    max_temperature_c: float = 58.0
    max_temperature_rise_c: float = 1.0
    max_baseline_span_rad: float = 0.005
    max_baseline_velocity_rad_s: float = 0.02
    non_target_tolerance_rad: float = 0.01
    return_tolerance_rad: float = 0.02
    minimum_target_fraction: float = 0.5
    minimum_response_rate_pct: float = PROJECT_MINIMUM_RESPONSE_RATE_PCT

    def __post_init__(self) -> None:
        numeric = (
            self.preflight_duration_s,
            self.warmup_s,
            self.baseline_s,
            self.ready_hold_s,
            self.inter_step_hold_s,
            self.pre_enable_hold_s,
            self.ramp_s,
            self.hold_s,
            self.return_s,
            self.settle_s,
            self.post_disable_s,
            self.command_rate_hz,
            self.stale_timeout_s,
            self.idle_sleep_s,
            self.max_temperature_c,
            self.max_temperature_rise_c,
            self.max_baseline_span_rad,
            self.max_baseline_velocity_rad_s,
            self.non_target_tolerance_rad,
            self.return_tolerance_rad,
            self.minimum_target_fraction,
            self.minimum_response_rate_pct,
        )
        if not self.profile_name.strip():
            raise ValueError("profile_name must be non-empty")
        if not self.steps:
            raise ValueError("steps must be non-empty")
        if len({step.joint_label for step in self.steps}) != len(self.steps):
            raise ValueError("sequence joint labels must be unique")
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("motion policy values must be finite")
        if self.preflight_duration_s < 30.0 or self.warmup_s < 4.0:
            raise ValueError("H3 requires at least 30 s preflight and 4 s warm-up")
        if self.baseline_s < 1.0 or self.ready_hold_s < 2.0:
            raise ValueError("H3 baseline and ready hold are too short")
        if self.inter_step_hold_s < 0.5 or self.pre_enable_hold_s < 0.1:
            raise ValueError("H3 baseline and pre-enable hold are too short")
        if self.ramp_s < 1.0 or self.return_s < 1.0:
            raise ValueError("H3 outbound and return ramps must each be at least 1 s")
        if self.hold_s < 0 or self.hold_s > 0.5 or self.settle_s < 0.5 or self.post_disable_s < 0.5:
            raise ValueError("H3 hold/settle duration is outside the pilot envelope")
        if not 1.0 <= self.command_rate_hz <= 100.0:
            raise ValueError("command_rate_hz must be between 1 and 100")
        if (
            min(
                self.stale_timeout_s,
                self.idle_sleep_s,
                self.max_temperature_c,
                self.max_temperature_rise_c,
                self.max_baseline_span_rad,
                self.max_baseline_velocity_rad_s,
                self.non_target_tolerance_rad,
                self.return_tolerance_rad,
            )
            <= 0
        ):
            raise ValueError("motion safety thresholds must be positive")
        if not 0 < self.minimum_target_fraction <= 1:
            raise ValueError("minimum_target_fraction must be in (0, 1]")
        if not PROJECT_MINIMUM_RESPONSE_RATE_PCT <= self.minimum_response_rate_pct <= 100:
            raise ValueError(
                "minimum_response_rate_pct must be between the project floor "
                f"{PROJECT_MINIMUM_RESPONSE_RATE_PCT:.0f} and 100"
            )


@dataclass(frozen=True, slots=True)
class MotionPreview:
    step_number: int
    serial: str
    side: Side
    joint_label: str
    joint_index: int
    nid: int
    description_joint: str
    baseline_position_rad: float
    target_position_rad: float
    delta_rad: float
    effort_limit_a: float
    mit_parameters: MitParameters

    def as_json(self) -> dict[str, JsonValue]:
        payload = asdict(self)
        payload["side"] = self.side.value
        return payload


@dataclass(frozen=True, slots=True)
class MotionPlan:
    serial: str
    side: Side
    profile_name: str
    steps: tuple[dict[str, JsonValue], ...]
    preflight_duration_s: float
    warmup_s: float
    ready_hold_s: float

    def as_json(self) -> dict[str, JsonValue]:
        return {
            "serial": self.serial,
            "side": self.side.value,
            "profile_name": self.profile_name,
            "steps": list(self.steps),
            "preflight_duration_s": self.preflight_duration_s,
            "warmup_s": self.warmup_s,
            "ready_hold_s": self.ready_hold_s,
        }


@dataclass(frozen=True, slots=True)
class MotionReport:
    automatic_checks_passed: bool
    operator_observation_required: bool
    failures: tuple[str, ...]
    started_at: str
    ended_at: str
    summary: dict[str, JsonValue]

    def as_json(self) -> dict[str, JsonValue]:
        return {
            "automatic_checks_passed": self.automatic_checks_passed,
            "operator_observation_required": self.operator_observation_required,
            "failures": list(self.failures),
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "summary": self.summary,
        }
