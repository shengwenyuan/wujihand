from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from .api import (
    H2_SEQUENCE_WAIVER_ID,
    H4_SEQUENCE_SCOPE_ID,
    bench_joint_sequence,
    monitor_temperature,
    qualify_readonly,
)
from .executor import H3_SEQUENCE_PROFILE, H4_SEQUENCE_PROFILE
from .mapping import (
    H3_MINIMUM_TARGET_FRACTION,
    H3_S1_SEQUENCE_LABELS,
    H3_SEQUENCE_DEFAULT_DELTA_RAD,
    H4_MINIMUM_TARGET_FRACTION,
    H4_Q20_SEQUENCE_LABELS,
)
from .types import (
    PROJECT_MINIMUM_RESPONSE_RATE_PCT,
    DeviceTarget,
    JointMotionStep,
    JointSequencePolicy,
    MotionPlan,
    MotionPreview,
    QualificationPolicy,
    Side,
    TemperatureSample,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="wujihand-hand2-hardware")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def common(name: str) -> argparse.ArgumentParser:
        command = subparsers.add_parser(name)
        command.add_argument("--serial", required=True)
        command.add_argument("--address", required=True)
        command.add_argument("--side", choices=[side.value for side in Side], required=True)
        command.add_argument("--firmware", required=True)
        command.add_argument("--hardware")
        command.add_argument("--output-dir", type=Path, required=True)
        command.add_argument("--warmup-s", type=float, default=3.0)
        command.add_argument("--stale-timeout-s", type=float, default=0.5)
        command.add_argument(
            "--minimum-response-rate-pct",
            type=float,
            default=PROJECT_MINIMUM_RESPONSE_RATE_PCT,
        )
        return command

    qualify = common("qualify-readonly")
    qualify.add_argument("--duration-s", type=float, default=60.0)

    monitor = common("monitor-temperature")
    monitor.add_argument("--duration-s", type=float, default=600.0)
    monitor.add_argument("--sample-period-s", type=float, default=1.0)
    monitor.add_argument("--max-rise-c", type=float, default=5.0)
    monitor.add_argument("--max-temperature-c", type=float)

    motion = common("bench-joint-sequence")
    motion.add_argument(
        "--profile",
        choices=[H3_SEQUENCE_PROFILE, H4_SEQUENCE_PROFILE],
        required=True,
    )
    motion.add_argument("--delta-rad", type=float, default=H3_SEQUENCE_DEFAULT_DELTA_RAD)
    scope = motion.add_mutually_exclusive_group(required=True)
    scope.add_argument(
        "--scope-id",
        choices=[H2_SEQUENCE_WAIVER_ID, H4_SEQUENCE_SCOPE_ID],
    )
    scope.add_argument(
        "--waiver-id",
        dest="scope_id",
        choices=[H2_SEQUENCE_WAIVER_ID],
        help="compatibility alias for the H3 limited waiver",
    )
    motion.add_argument("--preflight-duration-s", type=float, default=30.0)
    motion.set_defaults(warmup_s=4.0, stale_timeout_s=0.1)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    target = DeviceTarget(
        serial=args.serial,
        address=args.address,
        side=Side(args.side),
        expected_firmware=args.firmware,
        expected_hardware=args.hardware,
    )
    if args.command == "bench-joint-sequence":
        if not sys.stdin.isatty():
            print("FAIL: bench-joint-sequence requires an interactive terminal")
            return 1
        labels = (
            H3_S1_SEQUENCE_LABELS
            if args.profile == H3_SEQUENCE_PROFILE
            else H4_Q20_SEQUENCE_LABELS
        )
        motion_policy = JointSequencePolicy(
            profile_name=args.profile,
            steps=tuple(
                JointMotionStep(joint_label=label, delta_rad=args.delta_rad)
                for label in labels
            ),
            preflight_duration_s=args.preflight_duration_s,
            warmup_s=args.warmup_s,
            stale_timeout_s=args.stale_timeout_s,
            max_temperature_rise_c=5.0,
            max_baseline_span_rad=0.02,
            max_baseline_velocity_rad_s=0.1,
            non_target_tolerance_rad=0.1,
            return_tolerance_rad=0.12,
            minimum_target_fraction=(
                H3_MINIMUM_TARGET_FRACTION
                if args.profile == H3_SEQUENCE_PROFILE
                else H4_MINIMUM_TARGET_FRACTION
            ),
            minimum_response_rate_pct=args.minimum_response_rate_pct,
        )

        def confirm(plan: MotionPlan) -> bool:
            print("\nPLANNED SEQUENTIAL MOTION; THE DEVICE HAS NOT BEEN CONNECTED.\n")
            print(json.dumps(plan.as_json(), indent=2, sort_keys=True))
            print("\nStudio must be closed and the right hand must be the only command target.")
            print("Keep the bench clear and a hand on the physical power disconnect.")
            print(
                f"Response rate below {motion_policy.minimum_response_rate_pct:.0f}% blocks motion; "
                "other communication counters are recorded only."
            )
            print(
                f"MCU temperature >= {motion_policy.max_temperature_c:.0f} C or a "
                f"+{motion_policy.max_temperature_rise_c:.0f} C rise blocks motion."
            )
            print("Motion-quality thresholds use the relaxed bring-up acceptance profile.")
            print("After Enter, a >=34 s read-only gate runs before motion can start.")
            return (
                input("Press Enter once to confirm the entire sequence; type anything to cancel: ")
                == ""
            )

        def ready(previews: tuple[MotionPreview, ...]) -> None:
            print("\nREAD-ONLY PREFLIGHT PASSED; NO JOINT HAS BEEN ENABLED.\n")
            print(
                json.dumps(
                    [preview.as_json() for preview in previews],
                    indent=2,
                    sort_keys=True,
                )
            )
            print(
                f"\nThe first step starts after a {motion_policy.ready_hold_s:.1f} s guarded hold."
            )

        def show_step(preview: MotionPreview) -> None:
            print(
                f"STEP {preview.step_number}/{len(motion_policy.steps)}: "
                f"{preview.joint_label} (q{preview.joint_index}, NID {preview.nid}) "
                f"{preview.baseline_position_rad:.6f} -> {preview.target_position_rad:.6f} rad",
                flush=True,
            )

        try:
            motion_report = bench_joint_sequence(
                target,
                motion_policy,
                args.output_dir,
                scope_id=args.scope_id,
                confirm=confirm,
                on_ready=ready,
                on_step=show_step,
            )
        except Exception as error:  # noqa: BLE001 - fail closed at the CLI boundary.
            print(f"FAIL: {error}")
            return 1
        print(json.dumps(motion_report.as_json(), indent=2, sort_keys=True))
        print(f"artifacts: {args.output_dir}")
        return 0 if motion_report.automatic_checks_passed else 2

    if args.command == "monitor-temperature":
        qualification_policy = QualificationPolicy(
            duration_s=args.duration_s,
            warmup_s=args.warmup_s,
            stale_timeout_s=args.stale_timeout_s,
            temperature_sample_period_s=args.sample_period_s,
            max_temperature_rise_c=args.max_rise_c,
            max_temperature_c=args.max_temperature_c,
            minimum_response_rate_pct=args.minimum_response_rate_pct,
        )

        def show(sample: TemperatureSample) -> None:
            print(json.dumps(sample.as_json(), sort_keys=True), flush=True)

        operation = monitor_temperature
        callback = show
    else:
        qualification_policy = QualificationPolicy(
            duration_s=args.duration_s,
            warmup_s=args.warmup_s,
            stale_timeout_s=args.stale_timeout_s,
            minimum_response_rate_pct=args.minimum_response_rate_pct,
        )
        operation = qualify_readonly
        callback = None

    try:
        qualification_report = operation(
            target,
            qualification_policy,
            args.output_dir,
            on_temperature=callback,
        )
    except Exception as error:  # noqa: BLE001 - SDK failures terminate at the CLI boundary.
        print(f"FAIL: {error}")
        return 1

    print(json.dumps(qualification_report.as_json(), indent=2, sort_keys=True))
    print(f"artifacts: {args.output_dir}")
    return 0 if qualification_report.passed else 2
