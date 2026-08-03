#!/usr/bin/env python3
"""Run passive rosbag2 MCAP recording and finalize one run artifact."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime import (  # noqa: E402
    consumer_receipt_is_terminal,
    finalize_rosbag_recording,
)


CONSUMER_RECEIPT_WAIT_S = 8.0
RECORDER_INTERRUPT_WAIT_S = 12.0
RECORDER_TERMINATE_WAIT_S = 3.0
POLL_INTERVAL_S = 0.05


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--qos-profile", type=Path, required=True)
    parser.add_argument("--topic", action="append", dest="topics", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    topics = tuple(args.topics)
    if len(set(topics)) != len(topics):
        raise SystemExit("recording topic allowlist contains duplicates")
    if any(not topic.startswith("/") for topic in topics):
        raise SystemExit("recording topics must be absolute")
    run_root = args.run_root.resolve()
    output = run_root / "raw" / "rosbag2"
    if output.exists():
        raise SystemExit(f"rosbag output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    started_utc = datetime.now(timezone.utc).isoformat()
    started_monotonic_ns = time.monotonic_ns()
    _write_recorder_metadata(
        run_root,
        {
            "schema": "wujihand.rosbag2_recorder.v1",
            "run_id": args.run_id,
            "state": "starting",
            "started_utc": started_utc,
            "started_monotonic_ns": started_monotonic_ns,
            "storage": "mcap",
            "qos_profile": str(args.qos_profile),
            "topics": list(topics),
        },
    )
    command = [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        str(output),
        "--qos-profile-overrides-path",
        str(args.qos_profile),
        *topics,
    ]
    process: subprocess.Popen[bytes] | None = None
    stop_signal: int | None = None
    consumer_terminal_observed = False

    def request_stop(signum: int, frame: object) -> None:
        del frame
        nonlocal stop_signal
        if stop_signal is None:
            stop_signal = signum

    previous_handlers = {
        current: signal.signal(current, request_stop)
        for current in (signal.SIGINT, signal.SIGTERM)
    }
    exit_code = 127
    try:
        # Keep rosbag outside the launch process group. The wrapper alone owns
        # its shutdown so Ctrl+C cannot signal rosbag and the consumer at once.
        process = subprocess.Popen(command, start_new_session=True)
        _write_recorder_metadata(
            run_root,
            {
                "schema": "wujihand.rosbag2_recorder.v1",
                "run_id": args.run_id,
                "state": "running",
                "started_utc": started_utc,
                "started_monotonic_ns": started_monotonic_ns,
                "process_id": process.pid,
                "process_group_id": process.pid,
                "shutdown_owner": "recorder_wrapper",
                "storage": "mcap",
                "qos_profile": str(args.qos_profile),
                "topics": list(topics),
            },
        )
        while process.poll() is None and stop_signal is None:
            time.sleep(POLL_INTERVAL_S)
        if process.poll() is None:
            assert stop_signal is not None
            consumer_terminal_observed = _wait_for_consumer_terminal(
                run_root,
                run_id=args.run_id,
                timeout_s=CONSUMER_RECEIPT_WAIT_S,
            )
            if not consumer_terminal_observed:
                print(
                    "ROS RECORDING WARNING: consumer terminal receipt was "
                    "not observed before recorder shutdown",
                    file=sys.stderr,
                    flush=True,
                )
            exit_code = _stop_recorder(process)
        else:
            exit_code = process.wait()
    finally:
        for current, previous in previous_handlers.items():
            signal.signal(current, previous)
        _write_recorder_metadata(
            run_root,
            {
                "schema": "wujihand.rosbag2_recorder.v1",
                "run_id": args.run_id,
                "state": "exited" if process is not None else "failed",
                "started_utc": started_utc,
                "started_monotonic_ns": started_monotonic_ns,
                "closed_utc": datetime.now(timezone.utc).isoformat(),
                "closed_monotonic_ns": time.monotonic_ns(),
                "process_id": None if process is None else process.pid,
                "process_group_id": (
                    None if process is None else process.pid
                ),
                "shutdown_owner": "recorder_wrapper",
                "exit_code": exit_code,
                "stop_signal": stop_signal,
                "consumer_terminal_observed": (
                    consumer_terminal_observed
                ),
                "storage": "mcap",
                "qos_profile": str(args.qos_profile),
                "topics": list(topics),
            },
        )
        receipt = finalize_rosbag_recording(
            run_root,
            run_id=args.run_id,
            recorder_exit_code=exit_code,
        )
        print(
            f"ROS RECORDING CLOSED: run_id={args.run_id} state={receipt['state']} root={run_root}",
            flush=True,
        )
    return 0 if receipt["state"] == "complete" else 1


def _wait_for_consumer_terminal(
    run_root: Path,
    *,
    run_id: str,
    timeout_s: float,
) -> bool:
    deadline = time.monotonic() + timeout_s
    while True:
        if consumer_receipt_is_terminal(run_root, run_id=run_id):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(POLL_INTERVAL_S)


def _stop_recorder(process: subprocess.Popen[bytes]) -> int:
    """Stop rosbag once, escalating only after bounded graceful waits."""

    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=RECORDER_INTERRUPT_WAIT_S)
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        return process.wait(timeout=RECORDER_TERMINATE_WAIT_S)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait()


def _write_recorder_metadata(
    run_root: Path,
    value: dict[str, object],
) -> None:
    path = run_root / "recorder.json"
    temporary = run_root / ".recorder.json.tmp"
    run_root.mkdir(parents=True, exist_ok=True)
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


if __name__ == "__main__":
    raise SystemExit(main())
