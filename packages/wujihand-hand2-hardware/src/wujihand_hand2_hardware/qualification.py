from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from datetime import UTC, datetime

from .mapping import Q20_LABEL_BY_NID, Q20_NIDS, validate_q20_layout
from .sdk_client import ReadOnlySession
from .types import (
    CommunicationSample,
    CommunicationSnapshot,
    DeviceIdentity,
    DeviceTarget,
    JsonValue,
    QualificationPolicy,
    QualificationReport,
    TemperatureSample,
    TransportDiagnostics,
)


@dataclass(slots=True)
class _SequenceTracker:
    first: int | None = None
    last: int | None = None
    gaps: int = 0
    nonmonotonic: int = 0

    def observe(self, sequence: int) -> None:
        if self.first is None:
            self.first = sequence
        if self.last is not None:
            if sequence <= self.last:
                self.nonmonotonic += 1
            elif sequence > self.last + 1:
                self.gaps += sequence - self.last - 1
        self.last = sequence

    def as_json(self) -> dict[str, JsonValue]:
        return asdict(self)


def _identity_failures(identity: DeviceIdentity, target: DeviceTarget) -> list[str]:
    expected = {
        "serial": target.serial,
        "address": target.address,
        "side": target.side,
        "firmware": target.expected_firmware,
        "sdk": target.expected_sdk,
        "online_joints": 20,
        "device_type": "WujiHand2",
    }
    if target.expected_hardware is not None:
        expected["hardware"] = target.expected_hardware
    return [
        f"identity mismatch for {name}: expected={value!r}, observed={getattr(identity, name)!r}"
        for name, value in expected.items()
        if getattr(identity, name) != value
    ]


def _transport_failures(
    transport: TransportDiagnostics | None, *, require_current_snapshot: bool = True
) -> list[str]:
    if transport is None:
        return ["no frame-level transport diagnostics observed"]
    failures: list[str] = []
    if require_current_snapshot and transport.age_ms >= 65534:
        failures.append(f"device communication snapshot is stale: age_ms={transport.age_ms}")
    if require_current_snapshot and transport.e2e_window_loss_x100:
        failures.append(
            f"transport one-second loss window is {transport.e2e_window_loss_x100 / 100:.2f}%"
        )
    return failures


def _transport_delta_failures(
    before: TransportDiagnostics, after: TransportDiagnostics
) -> list[str]:
    failures: list[str] = []
    for name in (
        "e2e_lost",
        "e2e_reordered",
        "e2e_duplicates",
        "rpc_retries",
        "rpc_timeouts",
        "comm_get_failures",
        "sdk_dropped",
    ):
        delta = getattr(after, name) - getattr(before, name)
        if delta:
            failures.append(f"transport counter {name} changed by {delta}")
    if after.age_ms >= 65534:
        failures.append(f"device communication snapshot is stale: age_ms={after.age_ms}")
    if after.e2e_window_loss_x100:
        failures.append(
            f"transport one-second loss window is {after.e2e_window_loss_x100 / 100:.2f}%"
        )
    return failures


def _communication_assessment(
    before: CommunicationSnapshot,
    after: CommunicationSnapshot,
) -> tuple[list[str], dict[str, JsonValue]]:
    issues: list[str] = []
    fingers: list[JsonValue] = []

    def add(issue: str) -> None:
        issues.append(issue)

    if len(before.fingers) != 5 or len(after.fingers) != 5:
        add("communication diagnostics must contain five fingers")
    for first, last in zip(before.fingers, after.fingers, strict=False):
        crc_delta = last.crc_errors - first.crc_errors
        format_delta = last.format_errors - first.format_errors
        uart_delta = last.uart_errors - first.uart_errors
        if crc_delta or format_delta or uart_delta or last.error_per_second:
            add(f"finger {last.finger_index} communication errors increased")
        nodes: list[JsonValue] = []
        for initial_node, final_node in zip(first.nodes, last.nodes, strict=False):
            timeout_delta = final_node.timeout_total - initial_node.timeout_total
            request_delta = final_node.request_total - initial_node.request_total
            response_delta = final_node.response_total - initial_node.response_total
            if final_node.node_type == 0:
                if not final_node.online:
                    add(f"finger {last.finger_index} motor slot {final_node.slot} offline")
                if request_delta <= 0:
                    add(f"finger {last.finger_index} motor slot {final_node.slot} had no requests")
                if timeout_delta:
                    add(
                        f"finger {last.finger_index} motor slot {final_node.slot} "
                        f"timeout counter changed by {timeout_delta}"
                    )
                if final_node.response_rate_pct < 100.0:
                    add(
                        f"finger {last.finger_index} motor slot {final_node.slot} "
                        f"response rate is {final_node.response_rate_pct:.3f}%"
                    )
            nodes.append(
                {
                    "slot": final_node.slot,
                    "node_type": final_node.node_type,
                    "online": final_node.online,
                    "request_delta": request_delta,
                    "response_delta": response_delta,
                    "timeout_delta": timeout_delta,
                    "timeout_total": final_node.timeout_total,
                    "response_rate_pct": final_node.response_rate_pct,
                    "age_ms": final_node.age_ms,
                }
            )
        fingers.append(
            {
                "finger_index": last.finger_index,
                "crc_error_delta": crc_delta,
                "format_error_delta": format_delta,
                "uart_error_delta": uart_delta,
                "error_per_second": last.error_per_second,
                "nodes": nodes,
            }
        )
    return issues, {"fingers": fingers}


def _warm_up_readonly(
    session: ReadOnlySession,
    policy: QualificationPolicy,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> tuple[list[str], list[str], list[str], float]:
    if policy.warmup_s == 0:
        return [], [], [], 0.0

    failures: list[str] = []
    communication_issues: list[str] = []
    communication_gate_issues: list[str] = []
    state_sequence = _SequenceTracker()
    diagnostics_sequence = _SequenceTracker()
    expected_nids = set(Q20_NIDS)
    started = monotonic()
    last_state = started
    last_diagnostics = started
    latest_response_rate_min: float | None = None
    latest_transport: TransportDiagnostics | None = None

    while monotonic() - started < policy.warmup_s and not failures:
        now = monotonic()
        state = session.poll_state()
        if state is not None:
            last_state = now
            state_sequence.observe(state.header.sequence)
            if state_sequence.gaps or state_sequence.nonmonotonic:
                failures.append("warm-up state sequence is not contiguous and monotonic")
            elif {joint.nid for joint in state.joints} != expected_nids:
                failures.append("warm-up state frame has unexpected NIDs")
            elif any(
                not math.isfinite(value)
                for joint in state.joints
                for value in (joint.position_rad, joint.velocity_rad_s, joint.effort_a)
            ):
                failures.append("warm-up state frame has non-finite values")

        diagnostics = session.poll_diagnostics()
        if diagnostics is not None:
            last_diagnostics = now
            latest_transport = diagnostics.transport
            diagnostics_sequence.observe(diagnostics.header.sequence)
            if diagnostics_sequence.gaps or diagnostics_sequence.nonmonotonic:
                failures.append("warm-up diagnostics sequence is not contiguous and monotonic")
            elif {joint.nid for joint in diagnostics.joints} != expected_nids:
                failures.append("warm-up diagnostics frame has unexpected NIDs")
            else:
                transport_issues = _transport_failures(
                    diagnostics.transport,
                    require_current_snapshot=False,
                )
                communication_issues.extend(transport_issues)
                latest_response_rate_min = min(
                    joint.response_rate_pct for joint in diagnostics.joints
                )
                for joint in diagnostics.joints:
                    values = (joint.current_a, joint.bus_voltage_v, joint.mcu_temperature_c)
                    if not all(math.isfinite(value) for value in values):
                        failures.append(f"warm-up NID {joint.nid} has non-finite diagnostics")
                        break
                    if joint.error_code:
                        failures.append(
                            f"warm-up NID {joint.nid} reported error {joint.error_code}"
                        )
                        break
                    if joint.status != "Ready":
                        failures.append(
                            f"warm-up NID {joint.nid} is {joint.status}, expected Ready"
                        )
                        break
                    if any(
                        (
                            joint.position_limit_active,
                            joint.velocity_limit_active,
                            joint.current_limit_active,
                        )
                    ):
                        failures.append(f"warm-up NID {joint.nid} has an active limit")
                        break
                    if (
                        policy.max_temperature_c is not None
                        and joint.mcu_temperature_c >= policy.max_temperature_c
                    ):
                        failures.append(
                            "warm-up temperature reached configured maximum "
                            f"{policy.max_temperature_c:.3f} C"
                        )
                        break

        if now - last_state > policy.stale_timeout_s:
            failures.append("joint state stream became stale during warm-up")
        if now - last_diagnostics > policy.stale_timeout_s:
            failures.append("joint diagnostics stream became stale during warm-up")
        sleep(policy.idle_sleep_s)

    if state_sequence.first is None:
        failures.append("no joint state frames received during warm-up")
    if diagnostics_sequence.first is None:
        failures.append("no joint diagnostics frames received during warm-up")
    else:
        transport_issues = _transport_failures(latest_transport)
        communication_issues.extend(transport_issues)
        if latest_response_rate_min is not None and latest_response_rate_min < 100.0:
            issue = (
                f"warm-up ended with minimum motor response rate {latest_response_rate_min:.3f}%"
            )
            communication_issues.append(issue)
            if latest_response_rate_min < policy.minimum_response_rate_pct:
                communication_gate_issues.append(issue)
    return (
        list(dict.fromkeys(failures)),
        list(dict.fromkeys(communication_issues)),
        list(dict.fromkeys(communication_gate_issues)),
        monotonic() - started,
    )


def run_readonly_qualification(
    session: ReadOnlySession,
    target: DeviceTarget,
    policy: QualificationPolicy,
    *,
    on_temperature: Callable[[TemperatureSample], None] | None = None,
    on_communication: Callable[[CommunicationSample], None] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> QualificationReport:
    started_at = datetime.now(UTC).isoformat()
    identity = session.identity()
    failures = _identity_failures(identity, target)
    labels = session.joint_labels()
    communication_issues: list[str] = []
    communication_gate_issues: list[str] = []
    communication_observations: list[JsonValue] = []
    communication_samples = 0

    def record_communication(
        elapsed_s: float,
        source: str,
        issues: list[str],
        gate_issues: list[str],
        details: dict[str, JsonValue],
    ) -> None:
        nonlocal communication_samples
        sample = CommunicationSample(
            elapsed_s=round(elapsed_s, 3),
            source=source,
            issues=tuple(dict.fromkeys(issues)),
            gate_issues=tuple(dict.fromkeys(gate_issues)),
            details=details,
        )
        communication_samples += 1
        communication_issues.extend(sample.issues)
        communication_gate_issues.extend(sample.gate_issues)
        if sample.issues:
            communication_observations.append(sample.as_json())
        if on_communication is not None:
            on_communication(sample)

    (
        warmup_failures,
        warmup_communication_issues,
        warmup_communication_gate_issues,
        warmup_elapsed_s,
    ) = _warm_up_readonly(session, policy, monotonic, sleep)
    failures.extend(warmup_failures)
    record_communication(
        0.0,
        "warmup_final",
        warmup_communication_issues,
        warmup_communication_gate_issues,
        {"warmup_elapsed_s": round(warmup_elapsed_s, 6)},
    )
    failures.extend(warmup_communication_gate_issues)
    communication_before = session.communication()
    communication_checkpoint = communication_before

    state_sequence = _SequenceTracker()
    diagnostics_sequence = _SequenceTracker()
    state_frames = 0
    diagnostics_frames = 0
    state_nids: set[int] = set()
    diagnostics_nids: set[int] = set()
    error_codes: set[int] = set()
    error_names: dict[str, JsonValue] = {}
    statuses: set[str] = set()
    nonfinite_values = 0
    limit_samples = 0
    max_abs_velocity = 0.0
    max_abs_effort = 0.0
    voltage_min = math.inf
    voltage_max = -math.inf
    temperature_min = math.inf
    temperature_max = -math.inf
    temperature_baseline: float | None = None
    temperature_by_nid: dict[int, float] = {}
    interval_temperature_by_nid: dict[int, float] = {}
    response_rate_min = 100.0
    timeout_first: dict[int, int] = {}
    timeout_last: dict[int, int] = {}
    timeout_checkpoint: dict[int, int] = {}
    response_windows: list[JsonValue] = []
    previous_transport_age_ms: int | None = None
    transport_first: TransportDiagnostics | None = None
    transport_checkpoint: TransportDiagnostics | None = None
    transport_last: TransportDiagnostics | None = None

    started = monotonic()
    last_state = started
    last_diagnostics = started
    next_temperature_sample = (
        started + policy.temperature_sample_period_s
        if policy.temperature_sample_period_s is not None
        else None
    )

    while monotonic() - started < policy.duration_s and not failures:
        now = monotonic()
        state = session.poll_state()
        if state is not None:
            last_state = now
            state_frames += 1
            state_sequence.observe(state.header.sequence)
            if state_sequence.gaps or state_sequence.nonmonotonic:
                failures.append("joint state sequence is not contiguous and monotonic")
                break
            frame_nids = {joint.nid for joint in state.joints}
            state_nids.update(frame_nids)
            if frame_nids != set(Q20_NIDS):
                failures.append(f"state frame has unexpected NIDs: {sorted(frame_nids)}")
                break
            for state_joint in state.joints:
                values = (
                    state_joint.position_rad,
                    state_joint.velocity_rad_s,
                    state_joint.effort_a,
                )
                nonfinite_values += sum(not math.isfinite(value) for value in values)
                if not all(math.isfinite(value) for value in values):
                    failures.append(f"non-finite state value on NID {state_joint.nid}")
                    break
                max_abs_velocity = max(max_abs_velocity, abs(state_joint.velocity_rad_s))
                max_abs_effort = max(max_abs_effort, abs(state_joint.effort_a))

        diagnostics = session.poll_diagnostics()
        if diagnostics is not None:
            last_diagnostics = now
            diagnostics_frames += 1
            diagnostics_sequence.observe(diagnostics.header.sequence)
            if diagnostics_sequence.gaps or diagnostics_sequence.nonmonotonic:
                failures.append("joint diagnostics sequence is not contiguous and monotonic")
                break
            transport_last = diagnostics.transport
            if transport_first is None:
                transport_first = transport_last
            frame_nids = {joint.nid for joint in diagnostics.joints}
            diagnostics_nids.update(frame_nids)
            if frame_nids != set(Q20_NIDS):
                failures.append(f"diagnostics frame has unexpected NIDs: {sorted(frame_nids)}")
                break
            for joint in diagnostics.joints:
                timeout_first.setdefault(joint.nid, joint.timeout_total)
                timeout_checkpoint.setdefault(joint.nid, joint.timeout_total)
                timeout_last[joint.nid] = joint.timeout_total
            is_new_response_window = (
                previous_transport_age_ms is None
                or transport_last.age_ms < previous_transport_age_ms
            )
            previous_transport_age_ms = transport_last.age_ms
            if is_new_response_window:
                rates = {joint.nid: joint.response_rate_pct for joint in diagnostics.joints}
                below_100 = {nid: rate for nid, rate in rates.items() if rate < 100.0}
                below_threshold = {
                    nid: rate
                    for nid, rate in rates.items()
                    if rate < policy.minimum_response_rate_pct
                }
                timeout_deltas_now = {
                    nid: timeout_last[nid] - timeout_checkpoint[nid]
                    for nid in timeout_last
                    if timeout_last[nid] > timeout_checkpoint[nid]
                }
                timeout_checkpoint.update(timeout_last)
                transport_issues = (
                    _transport_failures(transport_last)
                    if transport_checkpoint is None
                    else _transport_delta_failures(transport_checkpoint, transport_last)
                )
                transport_checkpoint = transport_last
                window_issues = list(transport_issues)
                window_gate_issues: list[str] = []
                for nid, rate in below_100.items():
                    issue = f"NID {nid} ({Q20_LABEL_BY_NID[nid]}) response rate is {rate:.3f}%"
                    window_issues.append(issue)
                    if nid in below_threshold:
                        window_gate_issues.append(issue)
                for nid, delta in timeout_deltas_now.items():
                    issue = (
                        f"NID {nid} ({Q20_LABEL_BY_NID[nid]}) timeout counter changed by {delta}"
                    )
                    window_issues.append(issue)
                window: dict[str, JsonValue] = {
                    "sequence": diagnostics.header.sequence,
                    "age_ms": transport_last.age_ms,
                    "minimum_response_rate_pct": min(rates.values()),
                    "response_rate_pct_by_nid": {
                        str(nid): rate for nid, rate in sorted(rates.items())
                    },
                    "below_100_nids": sorted(below_100),
                    "below_threshold_nids": sorted(below_threshold),
                    "timeout_deltas_by_nid": {
                        str(nid): delta for nid, delta in sorted(timeout_deltas_now.items())
                    },
                    "transport": asdict(transport_last),
                }
                response_windows.append(window)
                record_communication(
                    now - started,
                    "joint_diagnostics_1hz",
                    window_issues,
                    window_gate_issues,
                    window,
                )
            for diagnostic_joint in diagnostics.joints:
                values = (
                    diagnostic_joint.current_a,
                    diagnostic_joint.bus_voltage_v,
                    diagnostic_joint.mcu_temperature_c,
                )
                nonfinite_values += sum(not math.isfinite(value) for value in values)
                if not all(math.isfinite(value) for value in values):
                    failures.append(f"non-finite diagnostics value on NID {diagnostic_joint.nid}")
                    break
                error_codes.add(diagnostic_joint.error_code)
                if (
                    diagnostic_joint.error_code
                    and str(diagnostic_joint.error_code) not in error_names
                ):
                    error_names[str(diagnostic_joint.error_code)] = session.describe_error(
                        diagnostic_joint.error_code
                    )
                if diagnostic_joint.error_code:
                    failures.append(
                        f"NID {diagnostic_joint.nid} reported error {diagnostic_joint.error_code}"
                    )
                    break
                statuses.add(diagnostic_joint.status)
                if diagnostic_joint.status != "Ready":
                    failures.append(
                        f"NID {diagnostic_joint.nid} is {diagnostic_joint.status}, expected Ready"
                    )
                    break
                limit_samples += sum(
                    (
                        diagnostic_joint.position_limit_active,
                        diagnostic_joint.velocity_limit_active,
                        diagnostic_joint.current_limit_active,
                    )
                )
                if any(
                    (
                        diagnostic_joint.position_limit_active,
                        diagnostic_joint.velocity_limit_active,
                        diagnostic_joint.current_limit_active,
                    )
                ):
                    failures.append(f"NID {diagnostic_joint.nid} has an active limit")
                    break
                voltage_min = min(voltage_min, diagnostic_joint.bus_voltage_v)
                voltage_max = max(voltage_max, diagnostic_joint.bus_voltage_v)
                temperature_min = min(temperature_min, diagnostic_joint.mcu_temperature_c)
                temperature_max = max(temperature_max, diagnostic_joint.mcu_temperature_c)
                temperature_by_nid[diagnostic_joint.nid] = diagnostic_joint.mcu_temperature_c
                interval_temperature_by_nid[diagnostic_joint.nid] = max(
                    interval_temperature_by_nid.get(diagnostic_joint.nid, -math.inf),
                    diagnostic_joint.mcu_temperature_c,
                )
                response_rate_min = min(response_rate_min, diagnostic_joint.response_rate_pct)

            if temperature_baseline is None and temperature_by_nid:
                temperature_baseline = max(temperature_by_nid.values())
            if policy.max_temperature_c is not None and temperature_max >= policy.max_temperature_c:
                failures.append(
                    f"temperature reached configured maximum {policy.max_temperature_c:.3f} C"
                )
            if (
                policy.max_temperature_rise_c is not None
                and temperature_baseline is not None
                and temperature_max - temperature_baseline >= policy.max_temperature_rise_c
            ):
                failures.append(
                    "temperature rise reached configured maximum "
                    f"{policy.max_temperature_rise_c:.3f} C"
                )

        if now - last_state > policy.stale_timeout_s:
            failures.append("joint state stream became stale")
        if now - last_diagnostics > policy.stale_timeout_s:
            failures.append("joint diagnostics stream became stale")

        if next_temperature_sample is not None and now >= next_temperature_sample:
            current_communication = session.communication()
            interval_comm_issues, interval_comm_summary = _communication_assessment(
                communication_checkpoint,
                current_communication,
            )
            communication_checkpoint = current_communication
            record_communication(
                now - started,
                "comm_diag_1hz",
                interval_comm_issues,
                [],
                interval_comm_summary,
            )
            if interval_temperature_by_nid and temperature_baseline is not None:
                sample = TemperatureSample(
                    elapsed_s=round(now - started, 3),
                    minimum_c=min(interval_temperature_by_nid.values()),
                    maximum_c=max(interval_temperature_by_nid.values()),
                    rise_from_baseline_c=max(interval_temperature_by_nid.values())
                    - temperature_baseline,
                    maximum_by_nid_c={
                        str(nid): value
                        for nid, value in sorted(interval_temperature_by_nid.items())
                    },
                )
                if on_temperature is not None:
                    on_temperature(sample)
            interval_temperature_by_nid.clear()
            next_temperature_sample += policy.temperature_sample_period_s or 0.0

        sleep(policy.idle_sleep_s)

    elapsed_s = monotonic() - started
    communication_after = session.communication()
    final_comm_issues, communication_summary = _communication_assessment(
        communication_before,
        communication_after,
    )
    record_communication(
        elapsed_s,
        "comm_diag_total",
        final_comm_issues,
        [],
        communication_summary,
    )
    failures.extend(validate_q20_layout(labels, state_nids))
    failures.extend(validate_q20_layout(labels, diagnostics_nids))

    if not state_frames:
        failures.append("no joint state frames received")
    if not diagnostics_frames:
        failures.append("no joint diagnostics frames received")
    if state_sequence.gaps or state_sequence.nonmonotonic:
        failures.append("joint state sequence is not contiguous and monotonic")
    if diagnostics_sequence.gaps or diagnostics_sequence.nonmonotonic:
        failures.append("joint diagnostics sequence is not contiguous and monotonic")
    if nonfinite_values:
        failures.append(f"observed {nonfinite_values} non-finite values")
    if error_codes != {0}:
        failures.append(f"nonzero or missing error-code baseline: {sorted(error_codes)}")
    if statuses != {"Ready"}:
        failures.append(f"unexpected joint states: {sorted(statuses)}")
    if limit_samples:
        failures.append(f"observed {limit_samples} active limit samples")
    timeout_deltas = {
        str(nid): timeout_last[nid] - first
        for nid, first in timeout_first.items()
        if timeout_last.get(nid, first) - first
    }
    total_transport_issues: list[str] = []
    if transport_first is not None and transport_last is not None:
        total_transport_issues = _transport_delta_failures(transport_first, transport_last)
    elif transport_last is None:
        total_transport_issues = _transport_failures(None)
    record_communication(
        elapsed_s,
        "transport_total",
        total_transport_issues,
        [],
        {
            "first": None if transport_first is None else asdict(transport_first),
            "last": None if transport_last is None else asdict(transport_last),
        },
    )
    if timeout_deltas:
        issue = f"joint timeout counters increased: {timeout_deltas}"
        record_communication(
            elapsed_s,
            "joint_timeout_total",
            [issue],
            [],
            {"timeout_deltas_by_nid": timeout_deltas},
        )
    if response_rate_min < 100.0:
        issue = f"minimum motor response rate was {response_rate_min:.3f}%"
        record_communication(
            elapsed_s,
            "response_rate_total",
            [issue],
            [issue] if response_rate_min < policy.minimum_response_rate_pct else [],
            {"minimum_response_rate_pct": response_rate_min},
        )
    unique_communication_issues = tuple(dict.fromkeys(communication_issues))
    unique_communication_gate_issues = tuple(dict.fromkeys(communication_gate_issues))
    failures.extend(unique_communication_gate_issues)

    summary: dict[str, JsonValue] = {
        "elapsed_s": round(elapsed_s, 6),
        "warmup_elapsed_s": round(warmup_elapsed_s, 6),
        "identity": {
            "serial": identity.serial,
            "address": identity.address,
            "side": identity.side.value,
            "firmware": identity.firmware,
            "hardware": identity.hardware,
            "sdk": identity.sdk,
            "online_joints": identity.online_joints,
            "device_type": identity.device_type,
        },
        "joint_labels": list(labels),
        "state_frames": state_frames,
        "diagnostics_frames": diagnostics_frames,
        "state_sequence": state_sequence.as_json(),
        "diagnostics_sequence": diagnostics_sequence.as_json(),
        "state_nids": sorted(state_nids),
        "diagnostics_nids": sorted(diagnostics_nids),
        "nonfinite_values": nonfinite_values,
        "observed_error_codes": sorted(error_codes),
        "error_names": error_names,
        "observed_statuses": sorted(statuses),
        "active_limit_samples": limit_samples,
        "max_abs_velocity_rad_s": max_abs_velocity,
        "max_abs_effort_a": max_abs_effort,
        "bus_voltage_v": {
            "minimum": None if math.isinf(voltage_min) else voltage_min,
            "maximum": None if math.isinf(voltage_max) else voltage_max,
        },
        "mcu_temperature_c": {
            "minimum": None if math.isinf(temperature_min) else temperature_min,
            "maximum": None if math.isinf(temperature_max) else temperature_max,
            "baseline_maximum": temperature_baseline,
            "final_by_nid": {str(nid): value for nid, value in sorted(temperature_by_nid.items())},
        },
        "minimum_response_rate_pct": response_rate_min,
        "response_windows": response_windows,
        "response_windows_below_100_pct": sum(
            bool(window["below_100_nids"]) for window in response_windows
        ),
        "communication_policy": "joint_diagnostics_response_rate_floor",
        "minimum_response_rate_threshold_pct": policy.minimum_response_rate_pct,
        "communication_samples": communication_samples,
        "communication_issues": list(unique_communication_issues),
        "communication_gate_issues": list(unique_communication_gate_issues),
        "communication_gate_passed": not unique_communication_gate_issues,
        "communication_observations": communication_observations,
        "joint_timeout_totals": {str(nid): value for nid, value in sorted(timeout_last.items())},
        "joint_timeout_deltas": timeout_deltas,
        "transport": None if transport_last is None else asdict(transport_last),
        "communication_delta": communication_summary,
    }
    ended_at = datetime.now(UTC).isoformat()
    unique_failures = tuple(dict.fromkeys(failures))
    return QualificationReport(
        passed=not unique_failures,
        failures=unique_failures,
        started_at=started_at,
        ended_at=ended_at,
        summary=summary,
    )
