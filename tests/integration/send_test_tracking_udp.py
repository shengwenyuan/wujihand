#!/usr/bin/env python3
"""Publish a bounded synthetic canonical Tracker stream for Isaac integration."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.transport import UdpTrackingSampleSender  # noqa: E402
from wujihand.domain import TrackedRigidBodySample, TrackingState  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=49154)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--poll-hz", type=float, default=90.0)
    parser.add_argument("--serial", default="LHR-SYNTHETIC")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be in [1, 65535]")
    if not 1.0 <= args.duration_s <= 300.0:
        raise SystemExit("--duration-s must be in [1, 300]")
    if not 1.0 <= args.poll_hz <= 500.0:
        raise SystemExit("--poll-hz must be in [1, 500]")

    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + round(args.duration_s * 1_000_000_000)
    period_ns = round(1_000_000_000 / args.poll_hz)
    sequence = 0
    with UdpTrackingSampleSender(args.port) as sender:
        while True:
            now_ns = time.monotonic_ns()
            if now_ns >= deadline_ns:
                break
            elapsed_s = (now_ns - started_ns) / 1_000_000_000
            sender.send(
                TrackedRigidBodySample(
                    stream_id="vive.right",
                    device_serial=args.serial,
                    logical_role="operator_right",
                    sequence=sequence,
                    tracking_frame="vive_tracking",
                    position_m=(
                        0.20 * math.sin(2.0 * math.pi * elapsed_s / 4.0),
                        1.0,
                        0.0,
                    ),
                    quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                    connected=True,
                    pose_valid=True,
                    tracking_state=TrackingState.RUNNING,
                    quality=1.0,
                    host_time_ns=now_ns,
                    device_time_ns=None,
                )
            )
            sequence += 1
            next_ns = started_ns + sequence * period_ns
            remaining_s = (next_ns - time.monotonic_ns()) / 1_000_000_000
            if remaining_s > 0.0:
                time.sleep(remaining_s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
