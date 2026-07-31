#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run the ROS 2 Jazzy dual NERO + Hand 2 simulation consumer."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

from wujihand.application.qualification import (
    GLOVE_LIVE_Q27_READINESS_POLICY,
    q27_window_max_delta_rad,
)
from wujihand.domain import HandSide
from wujihand.integrity import sha256_file
from wujihand.runtime import RosDeploymentResolver
from wujihand.runtime.isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    resolve_dual_side_runtimes,
    workcell_frame_position,
)
from wujihand.runtime.isaac_dual_teleoperation import (
    build_dual_teleoperation_application,
)
from wujihand.adapters.simulation import (
    load_nero_dual_tabletop_qualification_profile,
    load_nero_link_geometry_alignment,
)


DEFAULT_DEPLOYMENT = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_live_v2.yaml"
)
DEFAULT_LOCAL_BINDING = (
    ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"
)
NERO_LULA_DESCRIPTION = (
    ROOT / "configs/profiles/agilex_nero_lula_kinematics_v1.yaml"
)
OBLIQUE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
OBLIQUE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
SCREENSHOT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--deployment",
        type=Path,
        default=DEFAULT_DEPLOYMENT,
    )
    parser.add_argument(
        "--local-runtime-binding",
        type=Path,
        default=DEFAULT_LOCAL_BINDING,
    )
    parser.add_argument(
        "--gui",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Bounded headless frames; zero runs until the app closes.",
    )
    parser.add_argument("--report", type=Path)
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args = parser.parse_args()
    if args.frames < 0:
        parser.error("--frames must be non-negative")
    if not args.gui and args.frames == 0:
        parser.error("--no-gui requires a positive --frames bound")
    return args


ARGS = parse_args()
try:
    RESOLVED = RosDeploymentResolver(ROOT).resolve(
        ARGS.deployment,
        local_binding=ARGS.local_runtime_binding,
        verify_artifacts=ARGS.verify_artifacts,
    )
except (FileNotFoundError, ValueError) as exc:
    raise SystemExit(f"NV-5 ROS deployment preflight failed: {exc}") from exc

if RESOLVED.deployment.execution_owner_process_id != "isaac_consumer":
    raise SystemExit("NV-5 requires isaac_consumer as the unique owner")
if RESOLVED.session.session.backend != "isaac":
    raise SystemExit("NV-5 ROS consumer requires an Isaac Session")

SIDES = resolve_dual_side_runtimes(ROOT, RESOLVED.session)
alignment_references = {
    RESOLVED.session.instance(
        runtime.arm_instance_id
    ).binding.compatibility_profile
    for runtime in SIDES
}
if None in alignment_references or len(alignment_references) != 1:
    raise SystemExit(
        "both NERO Bindings must use one geometry alignment profile"
    )
ALIGNMENT_PATH = ROOT / str(next(iter(alignment_references)))
ALIGNMENT = load_nero_link_geometry_alignment(ALIGNMENT_PATH)
NERO_LULA_URDF = (ROOT / ALIGNMENT.source_urdf_path).resolve()
QUALIFICATION_PATH = ROOT / RESOLVED.control_profile.base_qualification.path
QUALIFICATION = load_nero_dual_tabletop_qualification_profile(
    QUALIFICATION_PATH
)
if not NERO_LULA_DESCRIPTION.is_file():
    raise SystemExit(
        f"NERO Lula descriptor not found: {NERO_LULA_DESCRIPTION}"
    )
if sha256_file(NERO_LULA_URDF) != ALIGNMENT.source_urdf_sha256:
    raise SystemExit("source-locked NERO URDF hash drifted")

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp(
    {
        "headless": not ARGS.gui,
        "width": 1280,
        "height": 800,
        "anti_aliasing": 0,
    }
)

import rclpy  # type: ignore[import-not-found]
from rclpy.executors import (  # type: ignore[import-not-found]
    SingleThreadedExecutor,
)
from rclpy.node import Node  # type: ignore[import-not-found]
from sensor_msgs.msg import JointState  # type: ignore[import-not-found]
from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
    HandObservationEnvelope,
    RouteCommand,
    SafetyEvent,
    TrackedRigidBodySample,
    TrackingLifecycleEvent,
)

from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
    set_camera_view,
)
from wujihand_ros2.conversion import (
    SafetyEventObservation,
    route_command_from_decision,
    route_command_to_message,
    safety_event_to_message,
)
from wujihand_ros2.input_adapters import (
    RosHandObservationInputAdapter,
    RosTrackerInputAdapter,
    TrackerInputIdentity,
)
from wujihand_ros2.qos import qos_profile


def _node_binding_name() -> str:
    for binding in RESOLVED.deployment.node_bindings:
        if binding.process_id == "isaac_consumer":
            return binding.node_name
    raise RuntimeError("isaac_consumer node binding is missing")


def _settle(scene: DualNeroHand2IsaacScene) -> dict[str, object]:
    policy = GLOVE_LIVE_Q27_READINESS_POLICY
    scene.apply_targets()
    previous: dict[str, list[float]] | None = None
    deltas: list[float] = []
    for window in range(1, policy.maximum_windows + 1):
        for _ in range(policy.window_frames):
            scene.world.step(render=ARGS.gui)
        current = {
            side: scene.feedback_q27(side).tolist()
            for side in ("left", "right")
        }
        if previous is not None:
            delta = q27_window_max_delta_rad(previous, current)
            deltas.append(delta)
            if (
                window >= policy.minimum_windows
                and delta <= policy.max_window_delta_rad
            ):
                return {
                    "converged": True,
                    "windows": window,
                    "final_max_delta_rad": delta,
                }
        previous = current
    if policy.require_convergence:
        raise RuntimeError("NV-5 scene readiness did not converge")
    return {
        "converged": False,
        "windows": policy.maximum_windows,
        "final_max_delta_rad": deltas[-1],
    }


def _route_topic(side: str, group_id: str, leaf: str) -> str:
    kind = "arm" if group_id == "arm_joints" else "hand"
    return f"{side}/{kind}/{leaf}"


def main() -> int:
    scene = DualNeroHand2IsaacScene(
        project_root=ROOT,
        resolved=RESOLVED.session,
        sides=SIDES,
        alignment_profile=ALIGNMENT,
        qualification_profile=QUALIFICATION,
        physics_hz=RESOLVED.control_profile.physics_hz,
    )
    readiness = _settle(scene)
    rclpy.init()
    node = Node(
        _node_binding_name(),
        namespace=f"/{RESOLVED.deployment.root_namespace}",
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    tracker_inputs: dict[str, RosTrackerInputAdapter] = {}
    hand_inputs: dict[HandSide, RosHandObservationInputAdapter] = {}
    subscriptions = []
    for side in ("left", "right"):
        arm_route = RESOLVED.route_plan.route(
            f"nero_{side}",
            "arm_joints",
        )
        if arm_route.source.kind == "vive_tracker":
            local = arm_route.local_binding
            if local is None:
                raise RuntimeError(f"{side} Tracker binding is missing")
            adapter = RosTrackerInputAdapter(
                TrackerInputIdentity(
                    stream_id=arm_route.source.source_id,
                    device_serial=local.device_identity,
                    logical_role=arm_route.source.logical_role,
                    tracking_setup_revision=(
                        RESOLVED.deployment.tracking_setup.setup_revision
                    ),
                    tracking_frame=RESOLVED.mapping.tracking_frame,
                )
            )
            tracker_inputs[side] = adapter
            subscriptions.append(
                node.create_subscription(
                    TrackedRigidBodySample,
                    f"input/tracker/{side}/sample",
                    adapter.offer_message,
                    qos_profile(
                        RESOLVED.qos_profile.policy("tracker_sample")
                    ),
                )
            )

        hand_route = RESOLVED.route_plan.route(
            f"hand_{side}",
            "finger_joints",
        )
        if hand_route.source.kind == "wuji_glove":
            local = hand_route.local_binding
            if local is None:
                raise RuntimeError(f"{side} Glove binding is missing")
            hand_side = HandSide(side)
            hand_adapter = RosHandObservationInputAdapter(
                side=hand_side,
                source_id=hand_route.source.source_id,
                calibration_id=local.calibration_id,
                transform_id="wuji_glove.hand_skeleton.v1",
            )
            hand_inputs[hand_side] = hand_adapter
            subscriptions.append(
                node.create_subscription(
                    HandObservationEnvelope,
                    f"input/glove/{side}/observation",
                    hand_adapter.offer_message,
                    qos_profile(
                        RESOLVED.qos_profile.policy(
                            "glove_observation"
                        )
                    ),
                )
            )

    def observe_lifecycle(message: TrackingLifecycleEvent) -> None:
        for adapter in tracker_inputs.values():
            adapter.offer_lifecycle_message(message)

    subscriptions.append(
        node.create_subscription(
            TrackingLifecycleEvent,
            "input/tracker/lifecycle",
            observe_lifecycle,
            qos_profile(
                RESOLVED.qos_profile.policy("tracking_lifecycle")
            ),
        )
    )
    del subscriptions

    application = build_dual_teleoperation_application(
        scene=scene,
        route_plan=RESOLVED.route_plan,
        profile=RESOLVED.control_profile,
        mapping=RESOLVED.mapping,
        tracker_inputs=tracker_inputs,
        hand_inputs=hand_inputs,
        lula_description=NERO_LULA_DESCRIPTION,
        lula_urdf=NERO_LULA_URDF,
    )
    command_publishers = {}
    feedback_publishers = {}
    safety_publishers = {}
    for route in RESOLVED.route_plan.routes:
        if route.source.kind not in {"vive_tracker", "wuji_glove"}:
            continue
        key = (route.instance_id, route.group_id)
        command_publishers[key] = node.create_publisher(
            RouteCommand,
            _route_topic(route.side, route.group_id, "command"),
            qos_profile(RESOLVED.qos_profile.policy("route_command")),
        )
        feedback_publishers[key] = node.create_publisher(
            JointState,
            _route_topic(route.side, route.group_id, "feedback"),
            qos_profile(RESOLVED.qos_profile.policy("route_feedback")),
        )
        safety_publishers[key] = node.create_publisher(
            SafetyEvent,
            _route_topic(route.side, route.group_id, "safety"),
            qos_profile(RESOLVED.qos_profile.policy("safety_event")),
        )

    set_camera_view(
        eye=np.asarray(
            workcell_frame_position(
                RESOLVED.session,
                OBLIQUE_CAMERA_EYE_FRAME,
            ),
            dtype=np.float64,
        ),
        target=np.asarray(
            workcell_frame_position(
                RESOLVED.session,
                OBLIQUE_CAMERA_TARGET_FRAME,
            ),
            dtype=np.float64,
        ),
        camera_prim_path=SCREENSHOT_CAMERA_PRIM_PATH,
    )
    started_ns = time.monotonic_ns()
    application.start(now_ns=started_ns)
    scene.apply_targets()
    counters: Counter[str] = Counter()
    safety_state: dict[tuple[str, str], tuple[object, ...]] = {}
    completed_frames = 0
    last_tick_ns = started_ns
    print(
        "NV5 ROS CONSUMER READY: "
        f"deployment={RESOLVED.deployment.deployment_id} "
        f"trackers={sorted(tracker_inputs)} "
        f"gloves={sorted(side.value for side in hand_inputs)}",
        flush=True,
    )
    try:
        while simulation_app.is_running() and (
            ARGS.frames == 0 or completed_frames < ARGS.frames
        ):
            executor.spin_once(timeout_sec=0.0)
            tick_ns = max(time.monotonic_ns(), last_tick_ns + 1)
            for side, adapter in tracker_inputs.items():
                if adapter.take_reference_invalidation():
                    application.arm_controllers[
                        side
                    ].invalidate_reference()
                    counters[f"{side}.tracker_epoch_changes"] += 1
            for side, adapter in hand_inputs.items():
                if adapter.take_epoch_change():
                    application.hand_controllers.invalidate_input_epoch(
                        side,
                    )
                    counters[f"{side.value}.glove_epoch_changes"] += 1

            result = application.cycle.step(
                feedback_q7_rad={
                    side: scene.feedback_q27(side)[
                        application.arm_indices[side]
                    ]
                    for side in application.arm_controllers
                },
                now_ns=tick_ns,
            )
            for labelled in result.arm_steps:
                side = labelled.side
                step = labelled.step
                route = RESOLVED.route_plan.route(
                    f"nero_{side}",
                    "arm_joints",
                )
                scene.arm_targets[side] = step.safety.command.copy()
                _publish_route(
                    node=node,
                    route=route,
                    layout_id=scene.arm_profiles[side].layout_id,
                    decision=step.safety,
                    feedback=scene.feedback_q27(side)[
                        application.arm_indices[side]
                    ],
                    joint_names=scene.arm_profiles[side].layout.names,
                    tick_ns=tick_ns,
                    command_publishers=command_publishers,
                    feedback_publishers=feedback_publishers,
                    safety_publishers=safety_publishers,
                    safety_state=safety_state,
                )
                counters[f"{side}.arm.{step.reason}"] += 1
            for labelled in result.hand_steps:
                side = labelled.side.value
                step = labelled.step
                route = RESOLVED.route_plan.route(
                    f"hand_{side}",
                    "finger_joints",
                )
                scene.hand_targets[side] = step.decision.command.copy()
                _publish_route(
                    node=node,
                    route=route,
                    layout_id=scene.hand_profiles[side].layout_id,
                    decision=step.decision,
                    feedback=scene.feedback_q27(side)[
                        np.asarray(
                            scene.partitions[side].hand_indices_q20,
                            dtype=np.int64,
                        )
                    ],
                    joint_names=scene.hand_profiles[side].layout.names,
                    tick_ns=tick_ns,
                    command_publishers=command_publishers,
                    feedback_publishers=feedback_publishers,
                    safety_publishers=safety_publishers,
                    safety_state=safety_state,
                )
                counters[
                    f"{side}.hand."
                    f"{step.rejection_reason or step.decision.reason}"
                ] += 1
            scene.apply_targets()
            scene.world.step(render=ARGS.gui)
            completed_frames += 1
            last_tick_ns = tick_ns
    finally:
        application.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    report = {
        "schema": "wujihand.isaac_ros_dual_teleoperation_run.v1",
        "scope": (
            "simulation-only dual NERO + Hand2; no UDP, CAN, NERO "
            "hardware, or Hand2 hardware commands"
        ),
        "deployment_id": RESOLVED.deployment.deployment_id,
        "deployment_hash": RESOLVED.deployment_hash,
        "local_binding_hash": RESOLVED.local_binding_hash,
        "session_id": RESOLVED.session.session.session_id,
        "session_hash": RESOLVED.session.session_hash,
        "mapping_sha256": RESOLVED.mapping_sha256,
        "completed_frames": completed_frames,
        "readiness": readiness,
        "counters": dict(counters),
        "input_metrics": {
            **{
                f"tracker_{side}": {
                    **asdict(adapter.metrics),
                    "inbox": asdict(adapter.inbox_metrics),
                }
                for side, adapter in tracker_inputs.items()
            },
            **{
                f"glove_{side.value}": {
                    **asdict(adapter.metrics),
                    "inbox": asdict(adapter.inbox_metrics),
                }
                for side, adapter in hand_inputs.items()
            },
        },
        "passed": True,
    }
    report_path = ARGS.report
    if report_path is None:
        timestamp = datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%SZ"
        )
        report_path = (
            ROOT
            / RESOLVED.deployment.report_root
            / f"{RESOLVED.deployment.deployment_id}-{timestamp}.json"
        )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"NV5 ROS CONSUMER CLOSED: report={report_path}", flush=True)
    return 0


def _publish_route(
    *,
    node: Node,
    route: object,
    layout_id: str,
    decision: object,
    feedback: np.ndarray,
    joint_names: tuple[str, ...],
    tick_ns: int,
    command_publishers: dict[tuple[str, str], object],
    feedback_publishers: dict[tuple[str, str], object],
    safety_publishers: dict[tuple[str, str], object],
    safety_state: dict[tuple[str, str], tuple[object, ...]],
) -> None:
    from wujihand.application.supervision import SafetyDecision
    from wujihand.runtime import DualTeleoperationRoute

    if not isinstance(route, DualTeleoperationRoute):
        raise TypeError("route must be a DualTeleoperationRoute")
    if not isinstance(decision, SafetyDecision):
        raise TypeError("decision must be a SafetyDecision")
    key = (route.instance_id, route.group_id)
    command_publishers[key].publish(
        route_command_to_message(
            route_command_from_decision(
                instance_id=route.instance_id,
                group_id=route.group_id,
                layout_id=layout_id,
                decision=decision,
                produced_time_ns=tick_ns,
            )
        )
    )
    feedback_message = JointState()
    feedback_message.header.stamp = node.get_clock().now().to_msg()
    feedback_message.name = list(joint_names)
    feedback_message.position = [float(value) for value in feedback]
    feedback_publishers[key].publish(feedback_message)
    current_safety = (
        decision.state,
        decision.reason,
        decision.position_clamped,
        decision.rate_limited,
    )
    if safety_state.get(key) != current_safety:
        safety_publishers[key].publish(
            safety_event_to_message(
                SafetyEventObservation(
                    instance_id=route.instance_id,
                    group_id=route.group_id,
                    state=decision.state,
                    reason=decision.reason,
                    position_clamped=decision.position_clamped,
                    rate_limited=decision.rate_limited,
                    host_time_ns=tick_ns,
                )
            )
        )
        safety_state[key] = current_safety


try:
    exit_code = main()
except BaseException:
    traceback.print_exc()
    sys.stderr.flush()
    simulation_app.close(exit_code=1)
    raise
simulation_app.close(exit_code=exit_code)
raise SystemExit(exit_code)
