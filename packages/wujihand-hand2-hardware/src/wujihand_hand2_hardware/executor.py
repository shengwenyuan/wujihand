from __future__ import annotations

import math
import statistics
import time
import uuid
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Protocol

from .journal import MotionArtifacts
from .mapping import (
    H3_DESCRIPTION_LIMIT_MARGIN_RAD,
    H3_MAX_DELTA_RAD,
    H3_S1_SEQUENCE_LABELS,
    Q20_DESCRIPTION_NAMES,
    Q20_INDEX_BY_LABEL,
    Q20_LOWER_RAD,
    Q20_NIDS,
    Q20_UPPER_RAD,
)
from .qualification import run_readonly_qualification
from .safety import MotionLifecycle
from .sdk_client import ReadOnlySession
from .sdk_motion import WujiSdkMotionClient
from .types import (
    ControlReadback,
    DeviceTarget,
    DiagnosticsFrame,
    JointCommandValue,
    JointSequencePolicy,
    JsonValue,
    MotionPlan,
    MotionPreview,
    MotionReport,
    QualificationPolicy,
    SafetyState,
    Side,
    StateFrame,
)

H2_SEQUENCE_WAIVER_ID = "H2-WAIVER-20260812-RIGHT-S1-SEQUENCE"
H3_SEQUENCE_PROFILE = "right-s1-flexion-v1"


class MotionSession(ReadOnlySession, Protocol):
    def control_readback(self) -> ControlReadback: ...

    def open_command_stream(self) -> None: ...

    def send_command(self, command: tuple[JointCommandValue, ...]) -> None: ...

    def enable_selected(self, mask: tuple[int, ...]) -> None: ...

    def disable_selected(self, mask: tuple[int, ...] | None = None) -> None: ...

    def emergency_stop_all(self) -> None: ...

    def close_command_stream(self) -> None: ...


class MotionClient(Protocol):
    @contextmanager
    def open(self, target: DeviceTarget) -> Iterator[MotionSession]: ...


Confirmation = Callable[[MotionPlan], bool]
ReadyCallback = Callable[[tuple[MotionPreview, ...]], None]
StepCallback = Callable[[MotionPreview], None]


class MotionRejected(RuntimeError):
    pass


@dataclass(frozen=True, slots=True)
class _Baseline:
    positions_rad: tuple[float, ...]
    spans_rad: tuple[float, ...]
    state: StateFrame
    diagnostics: DiagnosticsFrame


def _state_q20(frame: StateFrame) -> tuple[float, ...]:
    observed = {joint.nid: joint for joint in frame.joints}
    if len(frame.joints) != 20 or len(observed) != 20 or set(observed) != set(Q20_NIDS):
        raise MotionRejected(f"state frame has unexpected NIDs: {sorted(observed)}")
    values = tuple(observed[nid].position_rad for nid in Q20_NIDS)
    if not all(math.isfinite(value) for value in values):
        raise MotionRejected("state frame contains non-finite positions")
    return values


def _state_velocity_max(frame: StateFrame) -> float:
    values = tuple(joint.velocity_rad_s for joint in frame.joints)
    if not all(math.isfinite(value) for value in values):
        raise MotionRejected("state frame contains non-finite velocities")
    return max(abs(value) for value in values)


def _diagnostic_failures(
    frame: DiagnosticsFrame,
    *,
    selected_index: int,
    allow_selected_enabled: bool,
    temperature_baseline_c: float | None,
    max_temperature_c: float,
    max_temperature_rise_c: float,
    minimum_response_rate_pct: float,
) -> list[str]:
    observed = {joint.nid: joint for joint in frame.joints}
    if len(frame.joints) != 20 or len(observed) != 20 or set(observed) != set(Q20_NIDS):
        return [f"diagnostics frame has unexpected NIDs: {sorted(observed)}"]
    failures: list[str] = []
    for index, nid in enumerate(Q20_NIDS):
        joint = observed[nid]
        values = (joint.current_a, joint.bus_voltage_v, joint.mcu_temperature_c)
        if not all(math.isfinite(value) for value in values):
            failures.append(f"NID {nid} has non-finite diagnostics")
        if joint.error_code:
            failures.append(f"NID {nid} reported error {joint.error_code}")
        allowed_statuses = {"Ready"}
        if allow_selected_enabled and index == selected_index:
            allowed_statuses.add("Enabled")
        if joint.status not in allowed_statuses:
            failures.append(f"NID {nid} is {joint.status}, expected {sorted(allowed_statuses)}")
        if any(
            (
                joint.position_limit_active,
                joint.velocity_limit_active,
                joint.current_limit_active,
            )
        ):
            failures.append(f"NID {nid} has an active limit")
        if joint.response_rate_pct < minimum_response_rate_pct:
            failures.append(
                f"NID {nid} response rate {joint.response_rate_pct:.3f}% is below "
                f"project minimum {minimum_response_rate_pct:.3f}%"
            )
        if joint.mcu_temperature_c >= max_temperature_c:
            failures.append(
                f"NID {nid} temperature reached project maximum {max_temperature_c:.3f} C"
            )
        if (
            temperature_baseline_c is not None
            and joint.mcu_temperature_c - temperature_baseline_c >= max_temperature_rise_c
        ):
            failures.append(f"NID {nid} temperature rise reached {max_temperature_rise_c:.3f} C")
    return failures


def _collect_baseline(
    session: MotionSession,
    policy: JointSequencePolicy,
    *,
    duration_s: float,
    diagnostic_reference: DiagnosticsFrame | None,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> _Baseline:
    started = monotonic()
    last_state_at = started
    last_diagnostics_at = started
    samples: list[list[float]] = [[] for _ in Q20_NIDS]
    last_state: StateFrame | None = None
    last_diagnostics: DiagnosticsFrame | None = None
    temperature_baseline = (
        max(joint.mcu_temperature_c for joint in diagnostic_reference.joints)
        if diagnostic_reference is not None
        else None
    )
    while monotonic() - started < duration_s:
        now = monotonic()
        state = session.poll_state()
        if state is not None:
            q20 = _state_q20(state)
            velocity_max = _state_velocity_max(state)
            if velocity_max > policy.max_baseline_velocity_rad_s:
                raise MotionRejected(f"baseline velocity {velocity_max:.6f} rad/s exceeds guard")
            for values, value in zip(samples, q20, strict=True):
                values.append(value)
            last_state = state
            last_state_at = now
        diagnostics = session.poll_diagnostics()
        if diagnostics is not None:
            failures = _diagnostic_failures(
                diagnostics,
                selected_index=0,
                allow_selected_enabled=False,
                temperature_baseline_c=temperature_baseline,
                max_temperature_c=policy.max_temperature_c,
                max_temperature_rise_c=policy.max_temperature_rise_c,
                minimum_response_rate_pct=policy.minimum_response_rate_pct,
            )
            if failures:
                raise MotionRejected("; ".join(dict.fromkeys(failures)))
            last_diagnostics = diagnostics
            last_diagnostics_at = now
        if now - last_state_at > policy.stale_timeout_s:
            raise MotionRejected("joint state stream became stale during baseline")
        if now - last_diagnostics_at > policy.stale_timeout_s:
            raise MotionRejected("joint diagnostics stream became stale during baseline")
        sleep(policy.idle_sleep_s)
    if last_state is None or any(not values for values in samples):
        raise MotionRejected("no complete joint state baseline received")
    if last_diagnostics is None:
        raise MotionRejected("no joint diagnostics baseline received")
    positions = tuple(statistics.fmean(values) for values in samples)
    spans = tuple(max(values) - min(values) for values in samples)
    widest = max(spans)
    if widest > policy.max_baseline_span_rad:
        raise MotionRejected(f"baseline span {widest:.6f} rad exceeds guard")
    return _Baseline(
        positions_rad=positions,
        spans_rad=spans,
        state=last_state,
        diagnostics=last_diagnostics,
    )


def _validate_control_readback(readback: ControlReadback) -> None:
    if len(readback.effort_limits_a) != 20 or len(readback.mit_parameters) != 20:
        raise MotionRejected("control readback must contain exactly 20 joints")
    for index, (effort, params) in enumerate(
        zip(readback.effort_limits_a, readback.mit_parameters, strict=True)
    ):
        if effort is None or not math.isfinite(effort) or effort <= 0:
            raise MotionRejected(f"q{index} effort limit is missing or invalid")
        if params is None or not all(math.isfinite(value) for value in (params.kp, params.kd)):
            raise MotionRejected(f"q{index} MIT parameters are missing or invalid")
        if params.kp < 0 or params.kd < 0:
            raise MotionRejected(f"q{index} MIT parameters are negative")


def _make_plan(target: DeviceTarget, policy: JointSequencePolicy) -> MotionPlan:
    steps: list[dict[str, JsonValue]] = []
    for number, step in enumerate(policy.steps, start=1):
        index = Q20_INDEX_BY_LABEL[step.joint_label]
        steps.append(
            {
                "step_number": number,
                "joint_label": step.joint_label,
                "joint_index": index,
                "nid": Q20_NIDS[index],
                "description_joint": Q20_DESCRIPTION_NAMES[index],
                "delta_rad": step.delta_rad,
                "delta_deg": math.degrees(step.delta_rad),
            }
        )
    return MotionPlan(
        serial=target.serial,
        side=target.side,
        profile_name=policy.profile_name,
        steps=tuple(steps),
        preflight_duration_s=policy.preflight_duration_s,
        warmup_s=policy.warmup_s,
        ready_hold_s=policy.ready_hold_s,
    )


def _make_preview(
    target: DeviceTarget,
    policy: JointSequencePolicy,
    baseline: _Baseline,
    readback: ControlReadback,
    *,
    step_number: int,
) -> MotionPreview:
    step = policy.steps[step_number - 1]
    index = Q20_INDEX_BY_LABEL[step.joint_label]
    baseline_position = baseline.positions_rad[index]
    target_position = baseline_position + step.delta_rad
    lower = Q20_LOWER_RAD[index] + H3_DESCRIPTION_LIMIT_MARGIN_RAD
    upper = Q20_UPPER_RAD[index] - H3_DESCRIPTION_LIMIT_MARGIN_RAD
    if not lower <= baseline_position <= upper:
        raise MotionRejected(
            f"baseline q{index}={baseline_position:.6f} rad is outside the guarded envelope"
        )
    if not lower <= target_position <= upper:
        raise MotionRejected(
            f"target q{index}={target_position:.6f} rad is outside the guarded envelope"
        )
    effort = readback.effort_limits_a[index]
    params = readback.mit_parameters[index]
    if effort is None or params is None:
        raise MotionRejected("selected joint control readback is missing")
    return MotionPreview(
        step_number=step_number,
        serial=target.serial,
        side=target.side,
        joint_label=step.joint_label,
        joint_index=index,
        nid=Q20_NIDS[index],
        description_joint=Q20_DESCRIPTION_NAMES[index],
        baseline_position_rad=baseline_position,
        target_position_rad=target_position,
        delta_rad=step.delta_rad,
        effort_limit_a=effort,
        mit_parameters=params,
    )


@dataclass(slots=True)
class _MotionTracker:
    baseline: _Baseline
    safety_reference: _Baseline
    selected_index: int
    delta_rad: float
    policy: JointSequencePolicy
    last_state: StateFrame
    last_diagnostics: DiagnosticsFrame
    last_state_at: float
    last_diagnostics_at: float
    maximum_signed_target_excursion_rad: float = 0.0
    maximum_non_target_excursion_rad: float = 0.0
    maximum_temperature_c: float = -math.inf
    recorded_state_sequence: int | None = None

    def poll(
        self,
        session: MotionSession,
        *,
        allow_selected_enabled: bool,
        now: float,
    ) -> None:
        state = session.poll_state()
        if state is not None:
            positions = _state_q20(state)
            _state_velocity_max(state)
            sign = 1.0 if self.delta_rad > 0 else -1.0
            signed_excursion = (
                positions[self.selected_index] - self.baseline.positions_rad[self.selected_index]
            ) * sign
            self.maximum_signed_target_excursion_rad = max(
                self.maximum_signed_target_excursion_rad,
                signed_excursion,
            )
            for index, (observed, initial) in enumerate(
                zip(positions, self.baseline.positions_rad, strict=True)
            ):
                if index == self.selected_index:
                    continue
                excursion = abs(observed - initial)
                self.maximum_non_target_excursion_rad = max(
                    self.maximum_non_target_excursion_rad,
                    excursion,
                )
                if excursion > self.policy.non_target_tolerance_rad:
                    raise MotionRejected(
                        f"q{index} non-target excursion {excursion:.6f} rad exceeds guard"
                    )
            self.last_state = state
            self.last_state_at = now
        diagnostics = session.poll_diagnostics()
        if diagnostics is not None:
            reference = self.safety_reference.diagnostics
            failures = _diagnostic_failures(
                diagnostics,
                selected_index=self.selected_index,
                allow_selected_enabled=allow_selected_enabled,
                temperature_baseline_c=max(joint.mcu_temperature_c for joint in reference.joints),
                max_temperature_c=self.policy.max_temperature_c,
                max_temperature_rise_c=self.policy.max_temperature_rise_c,
                minimum_response_rate_pct=self.policy.minimum_response_rate_pct,
            )
            if failures:
                raise MotionRejected("; ".join(dict.fromkeys(failures)))
            self.maximum_temperature_c = max(
                self.maximum_temperature_c,
                *(joint.mcu_temperature_c for joint in diagnostics.joints),
            )
            self.last_diagnostics = diagnostics
            self.last_diagnostics_at = now
        if now - self.last_state_at > self.policy.stale_timeout_s:
            raise MotionRejected("joint state watchdog expired")
        if now - self.last_diagnostics_at > self.policy.stale_timeout_s:
            raise MotionRejected("joint diagnostics watchdog expired")


def _command_values(positions: tuple[float, ...]) -> tuple[JointCommandValue, ...]:
    if len(positions) != 20 or not all(math.isfinite(value) for value in positions):
        raise MotionRejected("command must contain 20 finite positions")
    return tuple(JointCommandValue(position_rad=value) for value in positions)


def _send(
    session: MotionSession,
    artifacts: MotionArtifacts,
    lifecycle: MotionLifecycle,
    target: DeviceTarget,
    correlation_id: str,
    command_sequence: int,
    positions: tuple[float, ...],
    *,
    step_number: int,
    joint_label: str,
    joint_index: int,
) -> None:
    details = {
        "step_number": step_number,
        "joint_label": joint_label,
        "joint_index": joint_index,
    }
    artifacts.command(
        "COMMAND_ATTEMPTED",
        lifecycle.state,
        serial=target.serial,
        side=target.side.value,
        correlation_id=correlation_id,
        sequence=command_sequence,
        positions_rad=positions,
        details=details,
    )
    try:
        session.send_command(_command_values(positions))
    except Exception as error:
        artifacts.command(
            "COMMAND_FAILED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
            sequence=command_sequence,
            positions_rad=positions,
            details=details,
            error=str(error),
        )
        raise
    artifacts.command(
        "COMMAND_ACCEPTED",
        lifecycle.state,
        serial=target.serial,
        side=target.side.value,
        correlation_id=correlation_id,
        sequence=command_sequence,
        positions_rad=positions,
        details=details,
    )


def _desired_position(
    elapsed_s: float,
    baseline: float,
    delta: float,
    policy: JointSequencePolicy,
) -> float:
    if elapsed_s < policy.ramp_s:
        return baseline + delta * elapsed_s / policy.ramp_s
    elapsed_s -= policy.ramp_s
    if elapsed_s < policy.hold_s:
        return baseline + delta
    elapsed_s -= policy.hold_s
    if elapsed_s < policy.return_s:
        return baseline + delta * (1.0 - elapsed_s / policy.return_s)
    return baseline


def _run_command_window(
    session: MotionSession,
    artifacts: MotionArtifacts,
    lifecycle: MotionLifecycle,
    target: DeviceTarget,
    policy: JointSequencePolicy,
    tracker: _MotionTracker,
    correlation_id: str,
    *,
    step_number: int,
    joint_label: str,
    duration_s: float,
    enabled: bool,
    profile: bool,
    command_sequence: int,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> int:
    started = monotonic()
    next_command = started
    period_s = 1.0 / policy.command_rate_hz
    while monotonic() - started < duration_s:
        now = monotonic()
        tracker.poll(session, allow_selected_enabled=enabled, now=now)
        if now >= next_command:
            positions = list(tracker.baseline.positions_rad)
            if profile:
                positions[tracker.selected_index] = _desired_position(
                    now - started,
                    tracker.baseline.positions_rad[tracker.selected_index],
                    tracker.delta_rad,
                    policy,
                )
            command_sequence += 1
            _send(
                session,
                artifacts,
                lifecycle,
                target,
                correlation_id,
                command_sequence,
                tuple(positions),
                step_number=step_number,
                joint_label=joint_label,
                joint_index=tracker.selected_index,
            )
            if tracker.recorded_state_sequence != tracker.last_state.header.sequence:
                artifacts.state(
                    serial=target.serial,
                    side=target.side.value,
                    correlation_id=correlation_id,
                    sequence=tracker.last_state.header.sequence,
                    device_timestamp_us=tracker.last_state.header.device_timestamp_us,
                    positions_rad=_state_q20(tracker.last_state),
                    details={
                        "step_number": step_number,
                        "joint_label": joint_label,
                        "joint_index": tracker.selected_index,
                    },
                )
                tracker.recorded_state_sequence = tracker.last_state.header.sequence
            while next_command <= now:
                next_command += period_s
        sleep(policy.idle_sleep_s)
    return command_sequence


def _safe_stop(
    session: MotionSession,
    artifacts: MotionArtifacts,
    lifecycle: MotionLifecycle,
    target: DeviceTarget,
    correlation_id: str,
    mask: tuple[int, ...] | None,
) -> tuple[str, ...]:
    failures: list[str] = []
    try:
        session.disable_selected(mask)
        artifacts.event(
            "DISABLED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
            payload={"scope": "whole_hand" if mask is None else "selected_joint"},
        )
        if lifecycle.state in {SafetyState.ARMED, SafetyState.ENABLED}:
            lifecycle.disabled()
    except Exception as disable_error:  # noqa: BLE001 - fail closed around the SDK action.
        failures.append(f"disable failed: {disable_error}")
        artifacts.event(
            "DISABLE_FAILED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
            payload={"error": str(disable_error)},
        )
        try:
            session.emergency_stop_all()
            lifecycle.estopped(str(disable_error))
            artifacts.event(
                "EMERGENCY_STOPPED",
                lifecycle.state,
                serial=target.serial,
                side=target.side.value,
                correlation_id=correlation_id,
            )
        except Exception as estop_error:  # noqa: BLE001 - preserve both stop failures.
            failures.append(f"emergency stop failed: {estop_error}")
            artifacts.event(
                "EMERGENCY_STOP_FAILED",
                lifecycle.state,
                serial=target.serial,
                side=target.side.value,
                correlation_id=correlation_id,
                payload={"error": str(estop_error)},
            )
    return tuple(failures)


def _post_disable_result(
    session: MotionSession,
    policy: JointSequencePolicy,
    tracker: _MotionTracker,
    *,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> dict[str, JsonValue]:
    started = monotonic()
    last_state: StateFrame | None = None
    last_diagnostics: DiagnosticsFrame | None = None
    last_state_at = started
    last_diagnostics_at = started
    while monotonic() - started < policy.post_disable_s:
        now = monotonic()
        state = session.poll_state()
        if state is not None:
            _state_q20(state)
            last_state = state
            last_state_at = now
        diagnostics = session.poll_diagnostics()
        if diagnostics is not None:
            last_diagnostics = diagnostics
            last_diagnostics_at = now
        if now - last_state_at > policy.stale_timeout_s:
            raise MotionRejected("joint state stream became stale after disable")
        if now - last_diagnostics_at > policy.stale_timeout_s:
            raise MotionRejected("joint diagnostics stream became stale after disable")
        sleep(policy.idle_sleep_s)
    if last_state is None or last_diagnostics is None:
        raise MotionRejected("no post-disable state/diagnostics received")
    reference = tracker.safety_reference.diagnostics
    failures = _diagnostic_failures(
        last_diagnostics,
        selected_index=tracker.selected_index,
        allow_selected_enabled=False,
        temperature_baseline_c=max(joint.mcu_temperature_c for joint in reference.joints),
        max_temperature_c=policy.max_temperature_c,
        max_temperature_rise_c=policy.max_temperature_rise_c,
        minimum_response_rate_pct=policy.minimum_response_rate_pct,
    )
    if failures:
        raise MotionRejected("post-disable check failed: " + "; ".join(failures))
    final_velocity = _state_velocity_max(last_state)
    if final_velocity > policy.max_baseline_velocity_rad_s:
        raise MotionRejected(f"post-disable velocity {final_velocity:.6f} rad/s exceeds guard")
    final_positions = _state_q20(last_state)
    return_error = abs(
        final_positions[tracker.selected_index]
        - tracker.baseline.positions_rad[tracker.selected_index]
    )
    minimum_excursion = abs(tracker.delta_rad) * policy.minimum_target_fraction
    if tracker.maximum_signed_target_excursion_rad < minimum_excursion:
        raise MotionRejected(
            "target joint excursion was too small: "
            f"observed={tracker.maximum_signed_target_excursion_rad:.6f}, "
            f"required={minimum_excursion:.6f} rad"
        )
    if return_error > policy.return_tolerance_rad:
        raise MotionRejected(f"target joint return error {return_error:.6f} rad exceeds guard")
    return {
        "maximum_signed_target_excursion_rad": tracker.maximum_signed_target_excursion_rad,
        "maximum_non_target_excursion_rad": tracker.maximum_non_target_excursion_rad,
        "return_error_rad": return_error,
        "maximum_temperature_c": tracker.maximum_temperature_c,
        "post_disable_statuses": sorted({joint.status for joint in last_diagnostics.joints}),
    }


def _validate_scope(target: DeviceTarget, policy: JointSequencePolicy, waiver_id: str) -> None:
    if target.side is not Side.RIGHT:
        raise MotionRejected("the H3 sequence authorizes the right hand only")
    if waiver_id != H2_SEQUENCE_WAIVER_ID:
        raise MotionRejected("limited H2 sequence waiver id does not match")
    if policy.profile_name != H3_SEQUENCE_PROFILE:
        raise MotionRejected(f"the H3 sequence authorizes profile {H3_SEQUENCE_PROFILE} only")
    labels = tuple(step.joint_label for step in policy.steps)
    if labels != H3_S1_SEQUENCE_LABELS:
        raise MotionRejected(f"the H3 sequence requires ordered joints {H3_S1_SEQUENCE_LABELS}")
    for step in policy.steps:
        if step.delta_rad <= 0 or step.delta_rad > H3_MAX_DELTA_RAD:
            raise MotionRejected(
                f"{step.joint_label} delta {step.delta_rad:.6f} rad is outside (0, "
                f"{H3_MAX_DELTA_RAD:.6f}]"
            )


def run_joint_sequence(
    target: DeviceTarget,
    policy: JointSequencePolicy,
    output_dir: Path,
    *,
    waiver_id: str,
    confirm: Confirmation,
    on_ready: ReadyCallback | None = None,
    on_step: StepCallback | None = None,
    client: MotionClient | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> MotionReport:
    started_at = datetime.now(UTC).isoformat()
    artifacts = MotionArtifacts(output_dir)
    lifecycle = MotionLifecycle()
    correlation_id = str(uuid.uuid4())
    active_client: MotionClient = client or WujiSdkMotionClient()
    failures: list[str] = []
    summary: dict[str, JsonValue] = {}
    artifacts.manifest(
        {
            "schema_revision": "hand2_h3_joint_sequence_v1",
            "mode": "bounded_sequential_motion",
            "purpose": "right_s1_mapping_sequence",
            "target": {
                "serial": target.serial,
                "address": target.address,
                "side": target.side.value,
                "expected_firmware": target.expected_firmware,
                "expected_hardware": target.expected_hardware,
                "expected_sdk": target.expected_sdk,
            },
            "policy": asdict(policy),
            "limited_h2_waiver_id": waiver_id,
            "command_capability": True,
            "operator_confirmation": "single_empty_line_before_connect",
        }
    )
    try:
        _validate_scope(target, policy, waiver_id)
        plan = _make_plan(target, policy)
        summary["plan"] = plan.as_json()
        artifacts.event(
            "SEQUENCE_PLANNED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
            payload=plan.as_json(),
        )
        if not confirm(plan):
            raise MotionRejected("operator did not confirm the sequence")
        artifacts.event(
            "OPERATOR_CONFIRMED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
            payload={"confirmation": "empty_line"},
        )

        writes_started = False
        with active_client.open(target) as opened_session:
            try:
                identity = opened_session.identity()
                lifecycle.connected(identity)
                artifacts.event(
                    "CONNECTED",
                    lifecycle.state,
                    serial=target.serial,
                    side=target.side.value,
                    correlation_id=correlation_id,
                    payload={
                        "firmware": identity.firmware,
                        "hardware": identity.hardware,
                        "sdk": identity.sdk,
                    },
                )
                preflight = run_readonly_qualification(
                    opened_session,
                    target,
                    QualificationPolicy(
                        duration_s=policy.preflight_duration_s,
                        warmup_s=policy.warmup_s,
                        stale_timeout_s=policy.stale_timeout_s,
                        idle_sleep_s=policy.idle_sleep_s,
                        temperature_sample_period_s=1.0,
                        max_temperature_rise_c=policy.max_temperature_rise_c,
                        max_temperature_c=policy.max_temperature_c,
                        minimum_response_rate_pct=policy.minimum_response_rate_pct,
                    ),
                    monotonic=monotonic,
                    sleep=sleep,
                )
                summary["preflight"] = preflight.as_json()
                if not preflight.passed:
                    raise MotionRejected("fresh preflight failed: " + "; ".join(preflight.failures))
                readback = opened_session.control_readback()
                _validate_control_readback(readback)
                safety_reference = _collect_baseline(
                    opened_session,
                    policy,
                    duration_s=policy.baseline_s,
                    diagnostic_reference=None,
                    monotonic=monotonic,
                    sleep=sleep,
                )
                previews = tuple(
                    _make_preview(
                        target,
                        policy,
                        safety_reference,
                        readback,
                        step_number=number,
                    )
                    for number in range(1, len(policy.steps) + 1)
                )
                summary["initial_previews"] = [preview.as_json() for preview in previews]
                artifacts.event(
                    "SEQUENCE_READY",
                    lifecycle.state,
                    serial=target.serial,
                    side=target.side.value,
                    correlation_id=correlation_id,
                    payload={"previews": [preview.as_json() for preview in previews]},
                    device_sequence=safety_reference.state.header.sequence,
                    device_timestamp_us=safety_reference.state.header.device_timestamp_us,
                )
                if on_ready is not None:
                    on_ready(previews)
                _collect_baseline(
                    opened_session,
                    policy,
                    duration_s=policy.ready_hold_s,
                    diagnostic_reference=safety_reference.diagnostics,
                    monotonic=monotonic,
                    sleep=sleep,
                )

                opened_session.open_command_stream()
                writes_started = True
                command_sequence = 0
                step_results: list[dict[str, JsonValue]] = []
                for step_number, step in enumerate(policy.steps, start=1):
                    baseline = _collect_baseline(
                        opened_session,
                        policy,
                        duration_s=policy.baseline_s,
                        diagnostic_reference=safety_reference.diagnostics,
                        monotonic=monotonic,
                        sleep=sleep,
                    )
                    preview = _make_preview(
                        target,
                        policy,
                        baseline,
                        readback,
                        step_number=step_number,
                    )
                    if on_step is not None:
                        on_step(preview)
                    selected_index = preview.joint_index
                    mask = tuple(1 if index == selected_index else 0 for index in range(20))
                    lifecycle.armed()
                    artifacts.event(
                        "STEP_ARMED",
                        lifecycle.state,
                        serial=target.serial,
                        side=target.side.value,
                        correlation_id=correlation_id,
                        payload={"preview": preview.as_json(), "mask": list(mask)},
                        device_sequence=baseline.state.header.sequence,
                        device_timestamp_us=baseline.state.header.device_timestamp_us,
                    )
                    now = monotonic()
                    tracker = _MotionTracker(
                        baseline=baseline,
                        safety_reference=safety_reference,
                        selected_index=selected_index,
                        delta_rad=step.delta_rad,
                        policy=policy,
                        last_state=baseline.state,
                        last_diagnostics=baseline.diagnostics,
                        last_state_at=now,
                        last_diagnostics_at=now,
                    )
                    command_sequence = _run_command_window(
                        opened_session,
                        artifacts,
                        lifecycle,
                        target,
                        policy,
                        tracker,
                        correlation_id,
                        step_number=step_number,
                        joint_label=step.joint_label,
                        duration_s=policy.pre_enable_hold_s,
                        enabled=False,
                        profile=False,
                        command_sequence=command_sequence,
                        monotonic=monotonic,
                        sleep=sleep,
                    )
                    opened_session.enable_selected(mask)
                    lifecycle.enabled()
                    artifacts.event(
                        "STEP_ENABLED",
                        lifecycle.state,
                        serial=target.serial,
                        side=target.side.value,
                        correlation_id=correlation_id,
                        payload={
                            "step_number": step_number,
                            "joint_label": step.joint_label,
                            "mask": list(mask),
                        },
                    )
                    command_sequence = _run_command_window(
                        opened_session,
                        artifacts,
                        lifecycle,
                        target,
                        policy,
                        tracker,
                        correlation_id,
                        step_number=step_number,
                        joint_label=step.joint_label,
                        duration_s=(
                            policy.ramp_s + policy.hold_s + policy.return_s + policy.settle_s
                        ),
                        enabled=True,
                        profile=True,
                        command_sequence=command_sequence,
                        monotonic=monotonic,
                        sleep=sleep,
                    )
                    command_sequence += 1
                    _send(
                        opened_session,
                        artifacts,
                        lifecycle,
                        target,
                        correlation_id,
                        command_sequence,
                        baseline.positions_rad,
                        step_number=step_number,
                        joint_label=step.joint_label,
                        joint_index=selected_index,
                    )
                    stop_failures = _safe_stop(
                        opened_session,
                        artifacts,
                        lifecycle,
                        target,
                        correlation_id,
                        mask,
                    )
                    if stop_failures:
                        raise MotionRejected("; ".join(stop_failures))
                    result = {
                        "preview": preview.as_json(),
                        **_post_disable_result(
                            opened_session,
                            policy,
                            tracker,
                            monotonic=monotonic,
                            sleep=sleep,
                        ),
                    }
                    step_results.append(result)
                    artifacts.event(
                        "STEP_COMPLETED",
                        lifecycle.state,
                        serial=target.serial,
                        side=target.side.value,
                        correlation_id=correlation_id,
                        payload=result,
                    )
                    if step_number < len(policy.steps):
                        _collect_baseline(
                            opened_session,
                            policy,
                            duration_s=policy.inter_step_hold_s,
                            diagnostic_reference=safety_reference.diagnostics,
                            monotonic=monotonic,
                            sleep=sleep,
                        )
                opened_session.close_command_stream()
                writes_started = False
                summary.update(
                    {
                        "command_correlation_id": correlation_id,
                        "command_frames": command_sequence,
                        "step_results": step_results,
                        "operator_observation": "pending",
                    }
                )
            except (Exception, KeyboardInterrupt):
                if writes_started:
                    _safe_stop(
                        opened_session,
                        artifacts,
                        lifecycle,
                        target,
                        correlation_id,
                        None,
                    )
                    try:
                        opened_session.close_command_stream()
                    except Exception as close_error:  # noqa: BLE001 - preserve primary failure.
                        artifacts.event(
                            "COMMAND_STREAM_CLOSE_FAILED",
                            lifecycle.state,
                            serial=target.serial,
                            side=target.side.value,
                            correlation_id=correlation_id,
                            payload={"error": str(close_error)},
                        )
                raise
        lifecycle.disconnected()
        artifacts.event(
            "DISCONNECTED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
        )
    except (Exception, KeyboardInterrupt) as error:  # noqa: BLE001 - CLI report boundary.
        failures.append(f"{type(error).__name__}: {error}")
        if lifecycle.state is not SafetyState.ESTOPPED:
            lifecycle.fault(str(error))
        artifacts.event(
            "MOTION_FAILED",
            lifecycle.state,
            serial=target.serial,
            side=target.side.value,
            correlation_id=correlation_id,
            payload={"error": str(error)},
        )
    ended_at = datetime.now(UTC).isoformat()
    unique_failures = tuple(dict.fromkeys(failures))
    report = MotionReport(
        automatic_checks_passed=not unique_failures,
        operator_observation_required=not unique_failures,
        failures=unique_failures,
        started_at=started_at,
        ended_at=ended_at,
        summary=summary,
    )
    artifacts.report(report)
    artifacts.close()
    return report
