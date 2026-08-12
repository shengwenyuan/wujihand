#!/usr/bin/env python3
# ruff: noqa: E402
"""Publish deterministic four-route ROS fixtures without opening devices."""

from __future__ import annotations

import argparse
import gc
import json
from pathlib import Path
import signal
import sys
import time


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

import rclpy  # type: ignore[import-not-found]
from rclpy.node import Node  # type: ignore[import-not-found]
from rclpy.signals import SignalHandlerOptions  # type: ignore[import-not-found]
from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
    HandObservationEnvelope,
    RunRecordingStatus,
    TrackedRigidBodySample,
    TrackingLifecycleEvent,
)

from wujihand.domain import (
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    RunRecordingState,
    TrackedRigidBodySample as CanonicalTrackedSample,
    TrackingLifecycleEvent as CanonicalLifecycleEvent,
    TrackingLifecycleKind,
    TrackingState,
)
from wujihand.application.qualification.dataset_preview_fixture import (
    FIXTURE_PRODUCER,
    FIXTURE_PROFILE_ID,
    REQUIRED_FRAMES,
    SELF_COLLISION_FIXTURE_PRODUCER,
    SELF_COLLISION_FIXTURE_PROFILE_ID,
    fixture_profile_mapping,
    fixture_profile_sha256,
    input_state,
    phase_for_sequence,
    self_collision_fixture_profile_mapping,
    self_collision_fixture_profile_sha256,
    self_collision_input_state,
)
from wujihand.runtime import FixedRateScheduler, RosDeploymentResolver
from wujihand_ros2.conversion import (
    HandObservationTransportEnvelope,
    hand_envelope_to_message,
    lifecycle_event_to_message,
    run_recording_status_from_message,
    tracked_sample_to_message,
)
from wujihand_ros2.qos import qos_profile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deployment", type=Path, required=True)
    parser.add_argument(
        "--local-runtime-binding",
        type=Path,
        required=True,
    )
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument(
        "--profile",
        choices=("static_v1", FIXTURE_PROFILE_ID, SELF_COLLISION_FIXTURE_PROFILE_ID),
        default="static_v1",
    )
    parser.add_argument("--receipt", type=Path)
    parser.add_argument(
        "--run-id",
        help="Qualification run whose STARTED status releases the deterministic fixture.",
    )
    parser.add_argument(
        "--minimum-subscribers",
        type=int,
        default=0,
        help="Wait for this many subscribers on every fixture topic before publishing.",
    )
    parser.add_argument(
        "--discovery-timeout-s",
        type=float,
        default=180.0,
    )
    return parser.parse_args()


def _wait_for_subscribers(
    publishers: tuple[object, ...],
    *,
    minimum_subscribers: int,
    timeout_s: float,
) -> None:
    if minimum_subscribers == 0:
        return
    deadline = time.monotonic() + timeout_s
    while True:
        pending = [
            publisher
            for publisher in publishers
            if publisher.get_subscription_count() < minimum_subscribers
        ]
        if not pending:
            return
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "fixture subscriber discovery timed out: "
                f"{len(pending)} topic(s) below {minimum_subscribers}"
            )
        time.sleep(0.05)


def _wait_for_recording_start(
    node: Node,
    *,
    run_id: str,
    status_qos: object,
    timeout_s: float,
) -> int:
    """Release the A/B/A clock only after the matching control owner is ready."""

    started_host_time_ns: int | None = None

    def receive(message: RunRecordingStatus) -> None:
        nonlocal started_host_time_ns
        status = run_recording_status_from_message(message)
        if status.run_id == run_id and status.state is RunRecordingState.STARTED:
            started_host_time_ns = status.host_time_ns

    subscription = node.create_subscription(
        RunRecordingStatus,
        "recording/status",
        receive,
        status_qos,
    )
    deadline = time.monotonic() + timeout_s
    try:
        while started_host_time_ns is None:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"fixture recording STARTED barrier timed out for run_id={run_id}"
                )
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(subscription)
    return started_host_time_ns


def _hand_observation(
    *,
    side: HandSide,
    source_id: str,
    calibration_id: str,
    sequence: int,
    host_time_ns: int,
    positions_m: tuple[tuple[float, float, float], ...] | None = None,
) -> CanonicalHandObservation:
    side_sign = -1.0 if side is HandSide.LEFT else 1.0
    positions = (
        positions_m
        if positions_m is not None
        else tuple(
            (
                side_sign * index / 1000.0,
                index / 500.0,
                index / 750.0,
            )
            for index in range(len(MEDIAPIPE_HAND_LANDMARK_NAMES))
        )
    )
    return CanonicalHandObservation(
        side=side,
        sequence=sequence,
        source_id=source_id,
        calibration_id=calibration_id,
        transform_id="wuji_glove.hand_skeleton.v1",
        source_time_ns=host_time_ns,
        receive_time_ns=host_time_ns,
        device_time_ns=None,
        device_clock_domain=None,
        frame_id=f"{side.value}_fixture_wrist",
        landmarks=tuple(
            HandLandmark(
                name=name,
                position_m=positions[index],
                confidence=1.0,
            )
            for index, name in enumerate(MEDIAPIPE_HAND_LANDMARK_NAMES)
        ),
    )


def main() -> int:
    args = parse_args()
    if args.frames < 1:
        raise SystemExit("--frames must be positive")
    bounded_profile = args.profile in {
        FIXTURE_PROFILE_ID,
        SELF_COLLISION_FIXTURE_PROFILE_ID,
    }
    if bounded_profile and args.frames < REQUIRED_FRAMES:
        raise SystemExit(f"{args.profile} requires at least {REQUIRED_FRAMES} source frames")
    if args.profile == FIXTURE_PROFILE_ID and not args.run_id:
        raise SystemExit(f"{FIXTURE_PROFILE_ID} requires --run-id")
    if args.minimum_subscribers < 0:
        raise SystemExit("--minimum-subscribers must be non-negative")
    if args.discovery_timeout_s <= 0.0:
        raise SystemExit("--discovery-timeout-s must be positive")
    resolved = RosDeploymentResolver(ROOT).resolve(
        args.deployment,
        local_binding=args.local_runtime_binding,
        verify_artifacts=False,
    )
    namespace = f"/{resolved.deployment.root_namespace}"
    if args.receipt is not None and args.receipt.exists():
        raise SystemExit("fixture receipt already exists")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("fixture_sources", namespace=namespace)
    tracker_publishers = {}
    hand_publishers = {}
    for side in ("left", "right"):
        tracker_publishers[side] = node.create_publisher(
            TrackedRigidBodySample,
            f"input/tracker/{side}/sample",
            qos_profile(resolved.qos_profile.policy("tracker_sample")),
        )
        hand_publishers[side] = node.create_publisher(
            HandObservationEnvelope,
            f"input/glove/{side}/observation",
            qos_profile(resolved.qos_profile.policy("glove_observation")),
        )
    lifecycle_publisher = node.create_publisher(
        TrackingLifecycleEvent,
        "input/tracker/lifecycle",
        qos_profile(resolved.qos_profile.policy("tracking_lifecycle")),
    )
    _wait_for_subscribers(
        (
            *tracker_publishers.values(),
            *hand_publishers.values(),
            lifecycle_publisher,
        ),
        minimum_subscribers=args.minimum_subscribers,
        timeout_s=args.discovery_timeout_s,
    )
    recording_started_host_time_ns = None
    if args.profile == FIXTURE_PROFILE_ID:
        assert args.run_id is not None
        recording_started_host_time_ns = _wait_for_recording_start(
            node,
            run_id=args.run_id,
            status_qos=qos_profile(resolved.qos_profile.policy("run_status")),
            timeout_s=args.discovery_timeout_s,
        )
    producer = {
        FIXTURE_PROFILE_ID: FIXTURE_PRODUCER,
        SELF_COLLISION_FIXTURE_PROFILE_ID: SELF_COLLISION_FIXTURE_PRODUCER,
    }.get(args.profile, "nv5-ros-fixture")
    epoch = 1
    lifecycle_sequence = 0
    stream_ids = tuple(
        resolved.route_plan.route(
            f"nero_{side}",
            "arm_joints",
        ).source.source_id
        for side in ("left", "right")
    )
    lifecycle_started_ns = time.monotonic_ns()
    lifecycle_publisher.publish(
        lifecycle_event_to_message(
            CanonicalLifecycleEvent(
                producer_instance=producer,
                tracking_setup_revision=(resolved.deployment.tracking_setup.setup_revision),
                stream_ids=stream_ids,
                kind=TrackingLifecycleKind.STARTED,
                reason="fixture_started",
                sequence=lifecycle_sequence,
                old_transport_epoch=None,
                new_transport_epoch=epoch,
                host_time_ns=lifecycle_started_ns,
            )
        )
    )
    lifecycle_sequence += 1
    started_ns = 0
    completed_frames = 0
    missed_periods = 0
    phase_counts = {"a_reference": 0, "b_motion": 0, "a_return": 0}
    requested_signal: int | None = None
    python_gc_frozen_during_run = False
    python_gc_frozen_object_count = 0

    def request_stop(signum: int, frame: object) -> None:
        del frame
        nonlocal requested_signal
        if requested_signal is None:
            requested_signal = signum

    previous_handlers = {
        current: signal.signal(current, request_stop) for current in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        gc.collect()
        gc.freeze()
        python_gc_frozen_during_run = True
        python_gc_frozen_object_count = gc.get_freeze_count()
        started_ns = time.monotonic_ns()
        scheduler = FixedRateScheduler(
            rate_hz=resolved.control_profile.physics_hz,
            start_ns=started_ns,
            maximum_catch_up_ticks=0,
        )
        for sequence in range(args.frames):
            if requested_signal is not None:
                break
            scheduled = scheduler.wait_next()
            missed_periods += scheduled.missed_periods_before_tick
            host_time_ns = time.monotonic_ns()
            for side in ("left", "right"):
                tracker_route = resolved.route_plan.route(
                    f"nero_{side}",
                    "arm_joints",
                )
                tracker_local = tracker_route.local_binding
                hand_route = resolved.route_plan.route(
                    f"hand_{side}",
                    "finger_joints",
                )
                hand_local = hand_route.local_binding
                if tracker_local is None or hand_local is None:
                    raise RuntimeError("fixture requires four live route bindings")
                hand_side = HandSide(side)
                qualification_state = (
                    input_state(hand_side, sequence)
                    if args.profile == FIXTURE_PROFILE_ID
                    else (
                        self_collision_input_state(hand_side, sequence)
                        if args.profile == SELF_COLLISION_FIXTURE_PROFILE_ID
                        else None
                    )
                )
                tracker_publishers[side].publish(
                    tracked_sample_to_message(
                        CanonicalTrackedSample(
                            stream_id=tracker_route.source.source_id,
                            device_serial=(tracker_local.device_identity),
                            logical_role=(tracker_route.source.logical_role),
                            producer_instance=producer,
                            transport_epoch=epoch,
                            tracking_setup_revision=(
                                resolved.deployment.tracking_setup.setup_revision
                            ),
                            sequence=sequence,
                            tracking_frame=resolved.mapping.tracking_frame,
                            position_m=(
                                (0.0, 0.0, 1.0)
                                if qualification_state is None
                                else qualification_state.tracker_position_m
                            ),
                            quat_wxyz=(
                                (1.0, 0.0, 0.0, 0.0)
                                if qualification_state is None
                                else qualification_state.tracker_quat_wxyz
                            ),
                            connected=True,
                            pose_valid=True,
                            tracking_state=TrackingState.RUNNING,
                            quality=1.0,
                            host_time_ns=host_time_ns,
                            device_time_ns=None,
                        )
                    )
                )
                hand_publishers[side].publish(
                    hand_envelope_to_message(
                        HandObservationTransportEnvelope(
                            producer_instance=producer,
                            transport_epoch=epoch,
                            observation=_hand_observation(
                                side=hand_side,
                                source_id=hand_route.source.source_id,
                                calibration_id=hand_local.calibration_id,
                                sequence=sequence,
                                host_time_ns=host_time_ns,
                                positions_m=(
                                    None
                                    if qualification_state is None
                                    else qualification_state.hand_landmarks_m
                                ),
                            ),
                        )
                    )
                )
            rclpy.spin_once(node, timeout_sec=0.0)
            completed_frames += 1
            if bounded_profile:
                phase_counts[phase_for_sequence(sequence)] += 1
            scheduler.complete(completed_ns=time.monotonic_ns())
    finally:
        if python_gc_frozen_during_run:
            gc.unfreeze()
        lifecycle_publisher.publish(
            lifecycle_event_to_message(
                CanonicalLifecycleEvent(
                    producer_instance=producer,
                    tracking_setup_revision=(resolved.deployment.tracking_setup.setup_revision),
                    stream_ids=stream_ids,
                    kind=TrackingLifecycleKind.STOPPED,
                    reason="fixture_stopped",
                    sequence=lifecycle_sequence,
                    old_transport_epoch=epoch,
                    new_transport_epoch=None,
                    host_time_ns=time.monotonic_ns(),
                )
            )
        )
        stopped_ns = time.monotonic_ns()
        node.destroy_node()
        rclpy.try_shutdown()
        for current, previous in previous_handlers.items():
            signal.signal(current, previous)
        if args.receipt is not None:
            args.receipt.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "schema": "wujihand.dataset_preview_fixture_receipt.v1",
                "passed": (
                    not bounded_profile
                    or (
                        completed_frames >= REQUIRED_FRAMES
                        and missed_periods == 0
                        and all(phase_counts.values())
                    )
                ),
                "profile_id": args.profile,
                "run_id": args.run_id,
                "recording_started_barrier_host_time_ns": recording_started_host_time_ns,
                "profile_sha256": (
                    fixture_profile_sha256()
                    if args.profile == FIXTURE_PROFILE_ID
                    else (
                        self_collision_fixture_profile_sha256()
                        if args.profile == SELF_COLLISION_FIXTURE_PROFILE_ID
                        else None
                    )
                ),
                "profile": (
                    fixture_profile_mapping()
                    if args.profile == FIXTURE_PROFILE_ID
                    else (
                        self_collision_fixture_profile_mapping()
                        if args.profile == SELF_COLLISION_FIXTURE_PROFILE_ID
                        else None
                    )
                ),
                "producer_instance": producer,
                "transport_epoch": epoch,
                "configured_rate_hz": resolved.control_profile.physics_hz,
                "requested_frames": args.frames,
                "required_frames": (
                    REQUIRED_FRAMES if args.profile == FIXTURE_PROFILE_ID else None
                ),
                "completed_frames": completed_frames,
                "phase_counts": phase_counts,
                "missed_periods": missed_periods,
                "python_gc_frozen_during_run": python_gc_frozen_during_run,
                "python_gc_frozen_object_count": python_gc_frozen_object_count,
                "started_monotonic_ns": started_ns,
                "stopped_monotonic_ns": stopped_ns,
                "effective_hz": (
                    (completed_frames - 1) / ((stopped_ns - started_ns) / 1_000_000_000)
                    if completed_frames >= 2 and stopped_ns > started_ns
                    else 0.0
                ),
                "requested_signal": requested_signal,
            }
            temporary = args.receipt.with_suffix(args.receipt.suffix + ".tmp")
            temporary.write_text(
                json.dumps(payload, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            temporary.replace(args.receipt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
