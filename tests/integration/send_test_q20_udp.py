#!/usr/bin/env python3
"""Integration helper: send a safe synthetic q20 trajectory over loopback UDP."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.transport import UdpJointCommandSender  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=49152)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--hz", type=float, default=60.0)
    args = parser.parse_args()
    if args.seconds <= 0 or args.hz <= 0:
        raise SystemExit("--seconds and --hz must be positive")

    close = np.array(
        [
            0.15,
            -0.35,
            0.75,
            0.75,
            0.65,
            0.0,
            1.1,
            0.85,
            0.72,
            0.0,
            1.2,
            0.9,
            0.72,
            0.0,
            1.2,
            0.9,
            0.68,
            0.0,
            1.1,
            0.85,
        ],
        dtype=np.float64,
    )
    period = 1.0 / args.hz
    started = time.monotonic()
    sent = 0
    with UdpJointCommandSender(args.port) as sender:
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= args.seconds:
                break
            phase = elapsed / args.seconds
            alpha = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
            sender.send(close * np.clip(alpha, 0.0, 1.0), host_time_ns=time.monotonic_ns())
            sent += 1
            deadline = started + sent * period
            time.sleep(max(deadline - time.monotonic(), 0.0))
    print(f"sent={sent} port={args.port} hz={args.hz:.1f} seconds={args.seconds:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
