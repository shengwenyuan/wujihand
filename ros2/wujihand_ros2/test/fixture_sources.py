#!/usr/bin/env python3
# ruff: noqa: E402
"""Publish deterministic four-route ROS fixtures without opening devices."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

import rclpy  # type: ignore[import-not-found]
from rclpy.node import Node  # type: ignore[import-not-found]
from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
    HandObservationEnvelope,
    TrackedRigidBodySample,
    TrackingLifecycleEvent,
)

from wujihand.domain import (
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    MEDIAPIPE_HAND_LANDMARK_NAMES,
    TrackedRigidBodySample as CanonicalTrackedSample,
    TrackingLifecycleEvent as CanonicalLifecycleEvent,
    TrackingLifecycleKind,
    TrackingState,
)
from wujihand.runtime import RosDeploymentResolver
from wujihand_ros2.conversion import (
    HandObservationTransportEnvelope,
    hand_envelope_to_message,
    lifecycle_event_to_message,
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


def _hand_observation(
    *,
    side: HandSide,
    source_id: str,
    calibration_id: str,
    sequence: int,
    host_time_ns: int,
) -> CanonicalHandObservation:
    side_sign = -1.0 if side is HandSide.LEFT else 1.0
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
                position_m=(
                    side_sign * index / 1000.0,
                    index / 500.0,
                    index / 750.0,
                ),
                confidence=1.0,
            )
            for index, name in enumerate(
                MEDIAPIPE_HAND_LANDMARK_NAMES
            )
        ),
    )


def main() -> int:
    args = parse_args()
    if args.frames < 1:
        raise SystemExit("--frames must be positive")
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
    rclpy.init()
    node = Node("fixture_sources", namespace=namespace)
    tracker_publishers = {}
    hand_publishers = {}
    for side in ("left", "right"):
        tracker_publishers[side] = node.create_publisher(
            TrackedRigidBodySample,
            f"input/tracker/{side}/sample",
            qos_profile(
                resolved.qos_profile.policy("tracker_sample")
            ),
        )
        hand_publishers[side] = node.create_publisher(
            HandObservationEnvelope,
            f"input/glove/{side}/observation",
            qos_profile(
                resolved.qos_profile.policy("glove_observation")
            ),
        )
    lifecycle_publisher = node.create_publisher(
        TrackingLifecycleEvent,
        "input/tracker/lifecycle",
        qos_profile(
            resolved.qos_profile.policy("tracking_lifecycle")
        ),
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
    producer = "nv5-ros-fixture"
    epoch = 1
    lifecycle_sequence = 0
    stream_ids = tuple(
        resolved.route_plan.route(
            f"nero_{side}",
            "arm_joints",
        ).source.source_id
        for side in ("left", "right")
    )
    started_ns = time.monotonic_ns()
    lifecycle_publisher.publish(
        lifecycle_event_to_message(
            CanonicalLifecycleEvent(
                producer_instance=producer,
                tracking_setup_revision=(
                    resolved.deployment.tracking_setup.setup_revision
                ),
                stream_ids=stream_ids,
                kind=TrackingLifecycleKind.STARTED,
                reason="fixture_started",
                sequence=lifecycle_sequence,
                old_transport_epoch=None,
                new_transport_epoch=epoch,
                host_time_ns=started_ns,
            )
        )
    )
    lifecycle_sequence += 1
    period_s = 1.0 / resolved.control_profile.physics_hz
    try:
        for sequence in range(args.frames):
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
                    raise RuntimeError(
                        "fixture requires four live route bindings"
                    )
                tracker_publishers[side].publish(
                    tracked_sample_to_message(
                        CanonicalTrackedSample(
                            stream_id=tracker_route.source.source_id,
                            device_serial=(
                                tracker_local.device_identity
                            ),
                            logical_role=(
                                tracker_route.source.logical_role
                            ),
                            producer_instance=producer,
                            transport_epoch=epoch,
                            tracking_setup_revision=(
                                resolved.deployment.tracking_setup.setup_revision
                            ),
                            sequence=sequence,
                            tracking_frame=resolved.mapping.tracking_frame,
                            position_m=(0.0, 0.0, 1.0),
                            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
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
                                side=HandSide(side),
                                source_id=hand_route.source.source_id,
                                calibration_id=hand_local.calibration_id,
                                sequence=sequence,
                                host_time_ns=host_time_ns,
                            ),
                        )
                    )
                )
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    finally:
        lifecycle_publisher.publish(
            lifecycle_event_to_message(
                CanonicalLifecycleEvent(
                    producer_instance=producer,
                    tracking_setup_revision=(
                        resolved.deployment.tracking_setup.setup_revision
                    ),
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
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
