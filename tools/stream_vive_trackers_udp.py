#!/usr/bin/env python3
"""Stream configured serial-addressed VIVE Trackers from one OpenVR owner.

This managed producer owns OpenVR and loopback UDP only. It never imports
Isaac, ROS, a robot SDK, or hardware command code.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time
from typing import Callable


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.input import (  # noqa: E402
    OpenVrMultiTrackerAdapter,
    OpenVrTrackerStreamConfig,
)
from wujihand.adapters.storage import (  # noqa: E402
    encode_tracking_lifecycle_event_json,
)
from wujihand.adapters.transport import UdpTrackingSampleSender  # noqa: E402
from wujihand.domain import (  # noqa: E402
    TrackingLifecycleEvent,
    TrackingLifecycleKind,
)


def _bounded_float(
    value: str,
    *,
    option: str,
    minimum: float,
    maximum: float,
) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{option} must be numeric") from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise argparse.ArgumentTypeError(
            f"{option} must be finite and in [{minimum:g}, {maximum:g}]"
        )
    return result


def _poll_hz(value: str) -> float:
    return _bounded_float(
        value,
        option="--poll-hz",
        minimum=1.0,
        maximum=500.0,
    )


def _duration_s(value: str) -> float:
    return _bounded_float(
        value,
        option="--duration-s",
        minimum=0.1,
        maximum=3600.0,
    )


def _non_negative_int(value: str) -> int:
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be an integer") from exc
    if result < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return result


def _udp_port(value: str) -> int:
    result = _non_negative_int(value)
    if not 1 <= result <= 65535:
        raise argparse.ArgumentTypeError("UDP port must be in [1, 65535]")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-serial")
    parser.add_argument("--left-udp-port", type=_udp_port)
    parser.add_argument("--right-serial")
    parser.add_argument("--right-udp-port", type=_udp_port)
    parser.add_argument("--producer-instance", required=True)
    parser.add_argument("--transport-epoch", required=True, type=_non_negative_int)
    parser.add_argument(
        "--previous-transport-epoch",
        type=_non_negative_int,
    )
    parser.add_argument("--tracking-setup-revision", required=True)
    parser.add_argument("--tracking-frame", default="vive_tracking")
    parser.add_argument("--poll-hz", type=_poll_hz, default=90.0)
    parser.add_argument(
        "--duration-s",
        type=_duration_s,
        help="Optional bounded qualification duration; managed runs omit it.",
    )
    return parser


def _lifecycle_event(
    *,
    args: argparse.Namespace,
    kind: TrackingLifecycleKind,
    reason: str,
    sequence: int,
    host_time_ns: int,
) -> TrackingLifecycleEvent:
    if kind is TrackingLifecycleKind.STARTED:
        old_epoch = None
        new_epoch = args.transport_epoch
    elif kind is TrackingLifecycleKind.REBOUND:
        old_epoch = args.previous_transport_epoch
        new_epoch = args.transport_epoch
    else:
        old_epoch = args.transport_epoch
        new_epoch = None
    stream_ids = tuple(
        stream_id
        for side, stream_id in (
            ("left", "vive.left"),
            ("right", "vive.right"),
        )
        if getattr(args, f"{side}_serial") is not None
    )
    return TrackingLifecycleEvent(
        producer_instance=args.producer_instance,
        tracking_setup_revision=args.tracking_setup_revision,
        stream_ids=stream_ids,
        kind=kind,
        reason=reason,
        sequence=sequence,
        old_transport_epoch=old_epoch,
        new_transport_epoch=new_epoch,
        host_time_ns=host_time_ns,
    )


def run(
    args: argparse.Namespace,
    *,
    adapter_factory: Callable[..., OpenVrMultiTrackerAdapter] = (
        OpenVrMultiTrackerAdapter
    ),
    sender_factory: Callable[[int], UdpTrackingSampleSender] = (
        UdpTrackingSampleSender
    ),
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleeper: Callable[[float], None] = time.sleep,
) -> int:
    configured_sides = tuple(
        side
        for side in ("left", "right")
        if getattr(args, f"{side}_serial") is not None
        or getattr(args, f"{side}_udp_port") is not None
    )
    if not configured_sides:
        raise ValueError("at least one Tracker side must be configured")
    if (
        args.previous_transport_epoch is not None
        and args.previous_transport_epoch == args.transport_epoch
    ):
        raise ValueError("previous and current transport epochs must differ")
    for side in configured_sides:
        if (
            getattr(args, f"{side}_serial") is None
            or getattr(args, f"{side}_udp_port") is None
        ):
            raise ValueError(
                f"{side} Tracker serial and UDP port must be provided together"
            )
    if len(configured_sides) == 2 and args.left_serial == args.right_serial:
        raise ValueError("left and right Tracker serials must differ")
    if len(configured_sides) == 2 and args.left_udp_port == args.right_udp_port:
        raise ValueError("left and right UDP ports must differ")

    stream_configs = tuple(
        OpenVrTrackerStreamConfig(
            tracker_serial=getattr(args, f"{side}_serial"),
            stream_id=f"vive.{side}",
            logical_role=f"operator_{side}",
            tracking_frame=args.tracking_frame,
        )
        for side in configured_sides
    )
    owner = adapter_factory(
        stream_configs,
        producer_instance=args.producer_instance,
        transport_epoch=args.transport_epoch,
        tracking_setup_revision=args.tracking_setup_revision,
    )
    senders = tuple(
        sender_factory(getattr(args, f"{side}_udp_port"))
        for side in configured_sides
    )
    state_counts = {
        config.stream_id: {} for config in stream_configs
    }
    last_states: dict[str, str] = {}
    sent = 0
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, request_stop)
    started_ns = monotonic_ns()
    deadline_ns = (
        None
        if args.duration_s is None
        else started_ns + round(args.duration_s * 1_000_000_000)
    )
    period_ns = max(1, round(1_000_000_000 / args.poll_hz))
    next_poll_ns = started_ns
    lifecycle_started = False
    try:
        selected = owner.start()
        disconnected = [item.serial for item in selected if not item.connected]
        if disconnected:
            raise RuntimeError(
                f"configured Trackers are disconnected: {disconnected}"
            )
        print(
            encode_tracking_lifecycle_event_json(
                _lifecycle_event(
                    args=args,
                    kind=(
                        TrackingLifecycleKind.STARTED
                        if args.previous_transport_epoch is None
                        else TrackingLifecycleKind.REBOUND
                    ),
                    reason=(
                        "launcher_start"
                        if args.previous_transport_epoch is None
                        else "managed_restart"
                    ),
                    sequence=0,
                    host_time_ns=monotonic_ns(),
                )
            ),
            flush=True,
        )
        lifecycle_started = True
        while not stop_requested:
            now_ns = monotonic_ns()
            if deadline_ns is not None and now_ns >= deadline_ns:
                break
            if now_ns < next_poll_ns:
                sleeper((next_poll_ns - now_ns) / 1_000_000_000)
            poll_time_ns = monotonic_ns()
            if deadline_ns is not None and poll_time_ns >= deadline_ns:
                break
            polls = owner.poll(host_time_ns=poll_time_ns)
            for poll, sender in zip(polls, senders, strict=True):
                sender.send(poll.sample)
                sent += 1
                stream_id = poll.sample.stream_id
                state = poll.sample.tracking_state.value
                counts = state_counts[stream_id]
                counts[state] = counts.get(state, 0) + 1
                if state != last_states.get(stream_id):
                    print(
                        f"stream={stream_id} tracker_state={state} "
                        f"connected={poll.sample.connected} "
                        f"actionable={str(poll.sample.pose_valid).lower()}",
                        file=sys.stderr,
                        flush=True,
                    )
                    last_states[stream_id] = state
            next_poll_ns += period_ns
            if next_poll_ns <= poll_time_ns:
                skipped = ((poll_time_ns - next_poll_ns) // period_ns) + 1
                next_poll_ns += skipped * period_ns
    except KeyboardInterrupt:
        stop_requested = True
    finally:
        finished_ns = monotonic_ns()
        owner.close()
        for sender in senders:
            sender.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
        if lifecycle_started:
            print(
                encode_tracking_lifecycle_event_json(
                    _lifecycle_event(
                        args=args,
                        kind=TrackingLifecycleKind.STOPPED,
                        reason=(
                            "signal_stop"
                            if stop_requested
                            else "bounded_duration_complete"
                        ),
                        sequence=1,
                        host_time_ns=finished_ns,
                    )
                ),
                flush=True,
            )

    print(
        json.dumps(
            {
                "status": "STOPPED",
                "sent_datagrams": sent,
                "state_counts": state_counts,
                "wall_duration_s": (monotonic_ns() - started_ns)
                / 1_000_000_000,
            },
            sort_keys=True,
        ),
        file=sys.stderr,
        flush=True,
    )
    return 0


def main() -> int:
    args = build_parser().parse_args()
    try:
        return run(args)
    except Exception as exc:
        print(
            f"error: {str(exc).strip() or type(exc).__name__}",
            file=sys.stderr,
            flush=True,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
