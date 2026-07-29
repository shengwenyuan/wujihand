#!/usr/bin/env python3
"""Stream one serial-addressed VIVE Tracker as canonical loopback UDP samples.

This process owns OpenVR only.  It never imports Isaac, ROS, a robot SDK, or a
five-layer Session, and it cannot command real or simulated joints.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.input.openvr_tracker import OpenVrTrackerAdapter  # noqa: E402
from wujihand.adapters.transport import UdpTrackingSampleSender  # noqa: E402


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


def _duration_s(value: str) -> float:
    return _bounded_float(
        value,
        option="--duration-s",
        minimum=1.0,
        maximum=3600.0,
    )


def _poll_hz(value: str) -> float:
    return _bounded_float(
        value,
        option="--poll-hz",
        minimum=1.0,
        maximum=500.0,
    )


def _udp_port(value: str) -> int:
    try:
        result = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--udp-port must be an integer") from exc
    if not 1 <= result <= 65535:
        raise argparse.ArgumentTypeError("--udp-port must be in [1, 65535]")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--udp-port", type=_udp_port, default=49154)
    parser.add_argument("--stream-id", default="vive.right")
    parser.add_argument("--logical-role", default="operator_right")
    parser.add_argument("--tracking-frame", default="vive_tracking")
    parser.add_argument("--poll-hz", type=_poll_hz, default=90.0)
    parser.add_argument("--duration-s", type=_duration_s, default=300.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    adapter = OpenVrTrackerAdapter(
        tracker_serial=args.serial,
        stream_id=args.stream_id,
        logical_role=args.logical_role,
        tracking_frame=args.tracking_frame,
    )
    sender = UdpTrackingSampleSender(args.udp_port)
    sample_counts: dict[str, int] = {}
    last_state: str | None = None
    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + round(args.duration_s * 1_000_000_000)
    period_ns = max(1, round(1_000_000_000 / args.poll_hz))
    next_poll_ns = started_ns
    sent = 0
    try:
        selected = adapter.start()
        if not selected.connected:
            raise RuntimeError(
                f"Tracker {selected.serial} is known but disconnected; "
                "briefly press its power button and wait for a green LED"
            )
        print(
            json.dumps(
                {
                    "status": "STREAMING",
                    "serial": selected.serial,
                    "model": selected.model,
                    "stream_id": args.stream_id,
                    "logical_role": args.logical_role,
                    "tracking_frame": args.tracking_frame,
                    "udp_endpoint": f"127.0.0.1:{args.udp_port}",
                    "poll_hz": args.poll_hz,
                    "duration_s": args.duration_s,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        while True:
            now_ns = time.monotonic_ns()
            if now_ns >= deadline_ns:
                break
            if now_ns < next_poll_ns:
                time.sleep((next_poll_ns - now_ns) / 1_000_000_000)
            poll_time_ns = time.monotonic_ns()
            if poll_time_ns >= deadline_ns:
                break
            poll = adapter.poll(host_time_ns=poll_time_ns)
            sender.send(poll.sample)
            sent += 1
            state = poll.sample.tracking_state.value
            sample_counts[state] = sample_counts.get(state, 0) + 1
            if state != last_state:
                actionable = str(poll.sample.pose_valid).lower()
                print(
                    f"tracker_state={state} connected={poll.sample.connected} "
                    f"pose_valid={poll.sample.pose_valid} udp=sent "
                    f"actionable={actionable}",
                    file=sys.stderr,
                    flush=True,
                )
                last_state = state
            next_poll_ns += period_ns
            if next_poll_ns <= poll_time_ns:
                skipped = ((poll_time_ns - next_poll_ns) // period_ns) + 1
                next_poll_ns += skipped * period_ns
    except KeyboardInterrupt:
        print("Tracker stream stopped by operator.", file=sys.stderr)
    except Exception as exc:
        print(f"error: {str(exc).strip() or type(exc).__name__}", file=sys.stderr)
        return 1
    finally:
        adapter.close()
        sender.close()

    print(
        json.dumps(
            {
                "status": "STOPPED",
                "sent_samples": sent,
                "sample_counts": sample_counts,
                "wall_duration_s": (
                    time.monotonic_ns() - started_ns
                )
                / 1_000_000_000,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
