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

from wujihand.adapters.input import MediaPipePalmOrientationEstimator  # noqa: E402
from wujihand.adapters.transport import (  # noqa: E402
    UdpHandCommandSender,
    UdpJointCommandSender,
)
from wujihand.application.calibration import (  # noqa: E402
    PalmOrientationCalibrator,
    StablePalmOrientationWindow,
)
from wujihand.domain import IDENTITY_QUATERNION_WXYZ, PoseIntent  # noqa: E402


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
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument(
        "--publish-udp-port",
        type=int,
        default=0,
        help="Publish legacy q20 v1 to 127.0.0.1:PORT; 0 disables publishing.",
    )
    transport.add_argument(
        "--publish-hand-command-port",
        type=int,
        default=0,
        help="Publish atomic q20 + rotation v2 to 127.0.0.1:PORT; 0 disables it.",
    )
    parser.add_argument(
        "--calibration-frames",
        type=int,
        default=15,
        help="Consecutive stable right-hand frames required for neutral calibration.",
    )
    parser.add_argument(
        "--calibration-max-spread-deg",
        type=float,
        default=8.0,
        help="Maximum SO(3) spread inside the neutral-calibration window.",
    )
    parser.add_argument("--pose-min-quality", type=float, default=0.5)
    parser.add_argument("--calibration-max-gap-ms", type=float, default=100.0)
    parser.add_argument("--pose-disarm-after-ms", type=float, default=500.0)
    parser.add_argument(
        "--clutch-repeat-frames",
        type=int,
        default=3,
        help="Repeat each identity clutch event to survive UDP drain/loss races.",
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
    if args.calibration_frames < 2:
        raise SystemExit("--calibration-frames must be at least 2")
    if not 0.0 < args.calibration_max_spread_deg < 180.0:
        raise SystemExit("--calibration-max-spread-deg must be in (0, 180)")
    if not 0.0 <= args.pose_min_quality <= 1.0:
        raise SystemExit("--pose-min-quality must be in [0, 1]")
    if args.calibration_max_gap_ms <= 0.0 or args.pose_disarm_after_ms <= 0.0:
        raise SystemExit("pose timing thresholds must be positive")
    if args.clutch_repeat_frames < 1:
        raise SystemExit("--clutch-repeat-frames must be positive")
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
    q20_publisher = (
        UdpJointCommandSender(args.publish_udp_port) if args.publish_udp_port else None
    )
    hand_publisher = (
        UdpHandCommandSender(args.publish_hand_command_port)
        if args.publish_hand_command_port
        else None
    )
    orientation_estimator = MediaPipePalmOrientationEstimator()
    orientation_calibrator = PalmOrientationCalibrator()
    calibration_window = StablePalmOrientationWindow(
        required_samples=args.calibration_frames,
        max_spread_rad=np.deg2rad(args.calibration_max_spread_deg),
        min_quality=args.pose_min_quality,
        max_sample_gap_s=args.calibration_max_gap_ms / 1000.0,
    )
    pose_disarm_after_ns = int(args.pose_disarm_after_ms * 1_000_000)
    latency_ms: deque[float] = deque(maxlen=120)
    frame_count = 0
    detection_count = 0
    wrong_hand_count = 0
    consecutive_missing = 0
    consecutive_pose_missing = 0
    pose_rejection_count = 0
    started_ns = time.monotonic_ns()
    last_timestamp_ms = -1

    profile = pipeline.start(config)
    device_name = profile.get_device().get_info(rs.camera_info.name)
    print(f"camera={device_name} color={args.width}x{args.height}@{args.fps}")
    print(
        "expected_hand=Right keys: q/esc quit, r reset all, "
        "c clutch/recenter, space print command"
    )

    last_qpos: np.ndarray | None = None
    last_pose_quat: tuple[float, float, float, float] | None = None
    last_calibration_id: str | None = None
    last_pose_error: str | None = None
    manual_clutch_requested = False
    clutch_repeat_remaining = 0
    last_valid_pose_time_ns: int | None = None

    def invalidate_pose_calibration(reason: str) -> None:
        nonlocal clutch_repeat_remaining
        nonlocal last_calibration_id
        nonlocal last_pose_error
        nonlocal last_pose_quat
        nonlocal last_valid_pose_time_ns
        nonlocal manual_clutch_requested
        orientation_estimator.reset()
        orientation_calibrator.reset()
        calibration_window.reset()
        clutch_repeat_remaining = 0
        last_calibration_id = None
        last_pose_quat = None
        last_valid_pose_time_ns = None
        manual_clutch_requested = False
        last_pose_error = reason
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
                pose_valid_this_frame = False
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
                        command_time_ns = time.monotonic_ns()
                        if q20_publisher is not None:
                            q20_publisher.send(last_qpos, host_time_ns=command_time_ns)
                        if hand_publisher is not None:
                            try:
                                if (
                                    last_valid_pose_time_ns is not None
                                    and command_time_ns - last_valid_pose_time_ns
                                    >= pose_disarm_after_ns
                                ):
                                    invalidate_pose_calibration(
                                        "pose wall-clock gap reached disarm threshold"
                                    )
                                orientation_sample = orientation_estimator.estimate(
                                    world,
                                    host_time_ns=command_time_ns,
                                    quality=score,
                                )
                                pose_valid_this_frame = (
                                    orientation_sample.quality >= args.pose_min_quality
                                )
                                if pose_valid_this_frame:
                                    last_valid_pose_time_ns = command_time_ns
                                last_pose_error = None
                                pose_intent = None
                                if manual_clutch_requested and pose_valid_this_frame:
                                    pose_intent = orientation_calibrator.clutch(
                                        orientation_sample
                                    )
                                    clutch_repeat_remaining = args.clutch_repeat_frames - 1
                                    calibration_window.reset()
                                    manual_clutch_requested = False
                                    print(
                                        "pose clutch captured "
                                        f"calibration_id={pose_intent.calibration_id}"
                                    )
                                elif (
                                    orientation_calibrator.is_calibrated
                                    and clutch_repeat_remaining > 0
                                ):
                                    calibration_id = orientation_calibrator.calibration_id
                                    if calibration_id is None:
                                        raise RuntimeError("calibrated pose has no calibration_id")
                                    pose_intent = PoseIntent(
                                        quat_wxyz=IDENTITY_QUATERNION_WXYZ,
                                        frame_id=orientation_calibrator.output_frame_id,
                                        host_time_ns=orientation_sample.host_time_ns,
                                        quality=orientation_sample.quality,
                                        calibration_id=calibration_id,
                                    )
                                    if pose_valid_this_frame:
                                        clutch_repeat_remaining -= 1
                                elif orientation_calibrator.is_calibrated:
                                    pose_intent = orientation_calibrator.apply(
                                        orientation_sample
                                    )
                                else:
                                    neutral_sample = calibration_window.add(
                                        orientation_sample
                                    )
                                    if neutral_sample is not None:
                                        pose_intent = orientation_calibrator.capture_neutral(
                                            neutral_sample
                                        )
                                        clutch_repeat_remaining = args.clutch_repeat_frames - 1
                                        print(
                                            "pose neutral calibrated "
                                            f"calibration_id={pose_intent.calibration_id}"
                                        )
                                if pose_intent is not None:
                                    hand_publisher.send(
                                        last_qpos,
                                        pose_intent.quat_wxyz,
                                        host_time_ns=pose_intent.host_time_ns,
                                        quality=pose_intent.quality,
                                        calibration_id=pose_intent.calibration_id,
                                    )
                                    last_pose_quat = pose_intent.quat_wxyz
                                    last_calibration_id = pose_intent.calibration_id
                            except ValueError as exc:
                                pose_rejection_count += 1
                                last_pose_error = str(exc)
                        detection_count += 1
                        consecutive_missing = 0
                    else:
                        wrong_hand_count += 1
                        consecutive_missing += 1
                else:
                    consecutive_missing += 1

                reset_after_frames = max(1, args.fps // 2)
                if consecutive_missing == reset_after_frames:
                    retarget.reset()
                    last_qpos = None
                if hand_publisher is not None:
                    if pose_valid_this_frame:
                        consecutive_pose_missing = 0
                    else:
                        consecutive_pose_missing += 1
                        if not orientation_calibrator.is_calibrated:
                            # The neutral window represents consecutive camera
                            # frames, not merely fifteen eventual detections.
                            calibration_window.reset()
                    if (
                        last_valid_pose_time_ns is not None
                        and time.monotonic_ns() - last_valid_pose_time_ns
                        >= pose_disarm_after_ns
                    ):
                        invalidate_pose_calibration(
                            "pose input missing for wall-clock disarm interval"
                        )
                    elif consecutive_pose_missing == reset_after_frames:
                        calibration_window.reset()
                elapsed_s = max((time.monotonic_ns() - started_ns) / 1e9, 1e-6)
                mean_latency = float(np.mean(latency_ms)) if latency_ms else 0.0
                if hand_publisher is None:
                    pose_status = "pose=disabled (q20-only)"
                elif orientation_calibrator.is_calibrated:
                    suffix = " clutch=pending" if manual_clutch_requested else ""
                    if clutch_repeat_remaining:
                        suffix += f" clutch_repeat={clutch_repeat_remaining}"
                    pose_status = f"pose=tracking cal={last_calibration_id[:8]}{suffix}"
                else:
                    pose_status = (
                        f"pose=calibrating {calibration_window.sample_count}/"
                        f"{args.calibration_frames}"
                    )
                if last_pose_error is not None:
                    pose_status += f" rejected={pose_rejection_count}"
                status = [
                    f"hand={label} score={score:.2f} expected=Right",
                    f"capture={frame_count / elapsed_s:.1f} fps  inference={mean_latency:.1f} ms",
                    f"valid={detection_count} wrong={wrong_hand_count} missing_run={consecutive_missing}",
                    pose_status,
                    "q/esc quit | r reset | c clutch | space print",
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
                        invalidate_pose_calibration("manual reset")
                        print("retargeter and pose calibration reset")
                    if key == ord("c") and hand_publisher is not None:
                        manual_clutch_requested = True
                        print("pose clutch requested; hold the desired neutral for one frame")
                    if key == ord(" "):
                        print("q20=", None if last_qpos is None else last_qpos.tolist())
                        print("root_delta_quat_wxyz=", last_pose_quat)

                if frame_count % args.fps == 0:
                    print(" | ".join(status[:3]))
    finally:
        if q20_publisher is not None:
            q20_publisher.close()
        if hand_publisher is not None:
            hand_publisher.close()
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
