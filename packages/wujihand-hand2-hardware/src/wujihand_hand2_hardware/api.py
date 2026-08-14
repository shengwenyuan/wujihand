from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from .executor import (
    H2_SEQUENCE_WAIVER_ID,
    H4_SEQUENCE_SCOPE_ID,
    Confirmation,
    MotionClient,
    ReadyCallback,
    StepCallback,
    run_joint_sequence,
)
from .journal import RunArtifacts
from .qualification import run_readonly_qualification
from .safety import ReadOnlyLifecycle
from .sdk_client import ReadOnlyClient, WujiSdkReadOnlyClient
from .types import (
    CommunicationSample,
    DeviceTarget,
    JointSequencePolicy,
    MotionReport,
    QualificationPolicy,
    QualificationReport,
    TemperatureSample,
)


def qualify_readonly(
    target: DeviceTarget,
    policy: QualificationPolicy,
    output_dir: Path,
    *,
    client: ReadOnlyClient | None = None,
    on_temperature: Callable[[TemperatureSample], None] | None = None,
) -> QualificationReport:
    artifacts = RunArtifacts(output_dir)
    lifecycle = ReadOnlyLifecycle()
    active_client = client or WujiSdkReadOnlyClient()
    artifacts.manifest(
        {
            "schema_revision": "hand2_hardware_qualification_v3",
            "mode": "read_only",
            "purpose": (
                "temperature_observation"
                if policy.temperature_sample_period_s is not None
                else "read_only_qualification"
            ),
            "target": {
                "serial": target.serial,
                "address": target.address,
                "side": target.side.value,
                "expected_firmware": target.expected_firmware,
                "expected_hardware": target.expected_hardware,
                "expected_sdk": target.expected_sdk,
            },
            "policy": {
                "duration_s": policy.duration_s,
                "warmup_s": policy.warmup_s,
                "stale_timeout_s": policy.stale_timeout_s,
                "temperature_sample_period_s": policy.temperature_sample_period_s,
                "max_temperature_rise_c": policy.max_temperature_rise_c,
                "max_temperature_c": policy.max_temperature_c,
                "minimum_response_rate_pct": policy.minimum_response_rate_pct,
            },
            "command_capability": False,
        }
    )
    try:
        with active_client.open(target) as session:
            identity = session.identity()
            lifecycle.connected(identity)
            artifacts.event(
                "CONNECTED",
                lifecycle.state,
                {
                    "serial": identity.serial,
                    "address": identity.address,
                    "side": identity.side.value,
                    "firmware": identity.firmware,
                    "hardware": identity.hardware,
                    "sdk": identity.sdk,
                },
            )

            def record_temperature(sample: TemperatureSample) -> None:
                artifacts.temperature(sample)
                if on_temperature is not None:
                    on_temperature(sample)

            def record_communication(sample: CommunicationSample) -> None:
                artifacts.communication(sample)

            report = run_readonly_qualification(
                session,
                target,
                policy,
                on_temperature=record_temperature,
                on_communication=record_communication,
            )
            if not report.passed:
                lifecycle.fault("; ".join(report.failures))
            artifacts.report(report)
            artifacts.event(
                "DIAGNOSTIC_OBSERVED",
                lifecycle.state,
                {
                    "passed": report.passed,
                    "failures": list(report.failures),
                    "communication_gate_passed": report.summary["communication_gate_passed"],
                },
            )
        lifecycle.disconnected()
        artifacts.event("DISCONNECTED", lifecycle.state)
        return report
    except Exception as error:
        lifecycle.fault(str(error))
        artifacts.event("QUALIFICATION_FAILED", lifecycle.state, {"error": str(error)})
        raise
    finally:
        artifacts.close()


def monitor_temperature(
    target: DeviceTarget,
    policy: QualificationPolicy,
    output_dir: Path,
    *,
    client: ReadOnlyClient | None = None,
    on_temperature: Callable[[TemperatureSample], None] | None = None,
) -> QualificationReport:
    if policy.temperature_sample_period_s is None:
        raise ValueError("temperature monitor requires temperature_sample_period_s")
    return qualify_readonly(
        target,
        policy,
        output_dir,
        client=client,
        on_temperature=on_temperature,
    )


def bench_joint_sequence(
    target: DeviceTarget,
    policy: JointSequencePolicy,
    output_dir: Path,
    *,
    scope_id: str,
    confirm: Confirmation,
    on_ready: ReadyCallback | None = None,
    on_step: StepCallback | None = None,
    client: MotionClient | None = None,
) -> MotionReport:
    return run_joint_sequence(
        target,
        policy,
        output_dir,
        scope_id=scope_id,
        confirm=confirm,
        on_ready=on_ready,
        on_step=on_step,
        client=client,
    )


__all__ = [
    "H2_SEQUENCE_WAIVER_ID",
    "H4_SEQUENCE_SCOPE_ID",
    "bench_joint_sequence",
    "monitor_temperature",
    "qualify_readonly",
]
