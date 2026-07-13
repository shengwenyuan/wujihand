#!/usr/bin/env python3
"""Stream a visible right hand from RealSense to Wuji Hand 2 retargeting.

MediaPipe VIDEO mode keeps each preview frame paired with its inference result.
Valid right-hand q20 commands can be published to the local Isaac process over UDP.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs
from wuji_sdk import Handedness, RetargetSession
from wuji_sdk.retargeting import HandModel


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.transport import UdpJointCommandSender  # noqa: E402


DEFAULT_MODEL = ROOT / "artifacts/models/mediapipe/hand_landmarker.task"
CONNECTIONS = tuple(mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run D435 RGB -> MediaPipe right hand -> Wuji Hand 2 retargeting."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--frames", type=int, default=0, help="Stop after N frames; 0 runs forever."
    )
    parser.add_argument("--headless", action="store_true", help="Do not open an OpenCV window.")
    parser.add_argument(
        "--mirror-display",
        action="store_true",
        help="Mirror only the preview; inference always receives the original RGB frame.",
    )
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument(
        "--publish-udp-port",
        type=int,
        default=0,
        help="Publish valid right-hand q20 to 127.0.0.1:PORT; 0 disables publishing.",
    )
    return parser.parse_args()


def draw_hand(frame_bgr: np.ndarray, landmarks: list[object]) -> None:
    height, width = frame_bgr.shape[:2]
    points = [
        (
            int(np.clip(point.x, 0.0, 1.0) * (width - 1)),
            int(np.clip(point.y, 0.0, 1.0) * (height - 1)),
        )
        for point in landmarks
    ]
    for connection in CONNECTIONS:
        cv2.line(frame_bgr, points[connection.start], points[connection.end], (80, 220, 80), 2)
    for index, point in enumerate(points):
        color = (0, 180, 255) if index in (4, 8, 12, 16, 20) else (255, 120, 40)
        cv2.circle(frame_bgr, point, 4, color, -1)


def put_status(frame_bgr: np.ndarray, lines: list[str]) -> None:
    for row, line in enumerate(lines):
        origin = (12, 28 + row * 25)
        cv2.putText(frame_bgr, line, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4)
        cv2.putText(frame_bgr, line, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.62, (240, 240, 240), 1)


def main() -> int:
    args = parse_args()
    if args.width <= 0 or args.height <= 0 or args.fps <= 0:
        raise SystemExit("--width, --height, and --fps must be positive")
    if not 0.0 <= args.confidence <= 1.0:
        raise SystemExit("--confidence must be in [0, 1]")
    if not args.model.is_file():
        print(f"model not found: {args.model}", file=sys.stderr)
        return 2

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.rgb8, args.fps)

    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(args.model.resolve())),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=args.confidence,
        min_hand_presence_confidence=args.confidence,
        min_tracking_confidence=args.confidence,
    )
    retarget = RetargetSession.for_hand(HandModel.WujiHand2, side=Handedness.Right)
    publisher = UdpJointCommandSender(args.publish_udp_port) if args.publish_udp_port else None
    latency_ms: deque[float] = deque(maxlen=120)
    frame_count = 0
    detection_count = 0
    wrong_hand_count = 0
    consecutive_missing = 0
    started_ns = time.monotonic_ns()
    last_timestamp_ms = -1

    profile = pipeline.start(config)
    device_name = profile.get_device().get_info(rs.camera_info.name)
    print(f"camera={device_name} color={args.width}x{args.height}@{args.fps}")
    print("expected_hand=Right keys: q/esc quit, r reset retargeter, space print q20")

    last_qpos: np.ndarray | None = None
    try:
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as detector:
            while args.frames <= 0 or frame_count < args.frames:
                frames = pipeline.wait_for_frames(5000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                rgb = np.asanyarray(color_frame.get_data()).copy()
                timestamp_ms = max(last_timestamp_ms + 1, time.monotonic_ns() // 1_000_000)
                last_timestamp_ms = timestamp_ms
                before_ns = time.monotonic_ns()
                result = detector.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp_ms
                )
                latency_ms.append((time.monotonic_ns() - before_ns) / 1_000_000.0)
                frame_count += 1

                label = "none"
                score = 0.0
                if result.hand_landmarks:
                    category = result.handedness[0][0]
                    label = category.category_name or "unknown"
                    score = float(category.score or 0.0)
                    draw_hand(rgb, result.hand_landmarks[0])
                    if label == "Right":
                        world = np.asarray(
                            [
                                [point.x, point.y, point.z]
                                for point in result.hand_world_landmarks[0]
                            ],
                            dtype=np.float32,
                        )
                        last_qpos = np.asarray(retarget.step(world), dtype=np.float32)
                        if last_qpos.shape != (20,) or not np.isfinite(last_qpos).all():
                            raise RuntimeError("retargeter returned an invalid 20-joint command")
                        if publisher is not None:
                            publisher.send(last_qpos, host_time_ns=time.monotonic_ns())
                        detection_count += 1
                        consecutive_missing = 0
                    else:
                        wrong_hand_count += 1
                        consecutive_missing += 1
                else:
                    consecutive_missing += 1

                if consecutive_missing == args.fps // 2:
                    retarget.reset()
                    last_qpos = None

                elapsed_s = max((time.monotonic_ns() - started_ns) / 1e9, 1e-6)
                mean_latency = float(np.mean(latency_ms)) if latency_ms else 0.0
                status = [
                    f"hand={label} score={score:.2f} expected=Right",
                    f"capture={frame_count / elapsed_s:.1f} fps  inference={mean_latency:.1f} ms",
                    f"valid={detection_count} wrong={wrong_hand_count} missing_run={consecutive_missing}",
                    "q/esc quit | r reset | space print q20",
                ]
                put_status(rgb, status)

                if not args.headless:
                    preview = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    if args.mirror_display:
                        preview = cv2.flip(preview, 1)
                    cv2.imshow("D435 -> MediaPipe -> Wuji Hand 2", preview)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break
                    if key == ord("r"):
                        retarget.reset()
                        last_qpos = None
                        print("retargeter reset")
                    if key == ord(" "):
                        print("q20=", None if last_qpos is None else last_qpos.tolist())

                if frame_count % args.fps == 0:
                    print(" | ".join(status[:3]))
    finally:
        if publisher is not None:
            publisher.close()
        pipeline.stop()
        cv2.destroyAllWindows()

    print(
        f"summary frames={frame_count} valid_right={detection_count} "
        f"wrong_hand={wrong_hand_count} mean_inference_ms="
        f"{(float(np.mean(latency_ms)) if latency_ms else 0.0):.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
