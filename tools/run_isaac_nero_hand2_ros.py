#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run the ROS 2 Jazzy dual NERO + Hand 2 simulation consumer."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import gc
import json
import math
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback
from typing import Any

import numpy as np
from numpy.typing import NDArray


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

from wujihand.application.qualification import (
    GLOVE_LIVE_Q27_READINESS_POLICY,
    q27_window_max_delta_rad,
)
from wujihand.application.teleoperation import (
    Hand2SimulationStep,
    TrackerArmSimulationStep,
)
from wujihand.domain import (
    ArmControlTrace,
    ArmKinematicsTrace,
    ArmMappingTrace,
    HandControlTrace,
    HandIntentTrace,
    HandSide,
    RouteDecisionTrace,
    RunRecordingState,
    RunRecordingStatus,
    SceneRigidBodyState,
    SourceSelectionTrace,
    TeleoperationTickTrace,
    TickExecutionTrace,
    TickStageTimes,
)
from wujihand.integrity import sha256_file
from wujihand.runtime import (
    FixedRateScheduler,
    RosDeploymentResolver,
    SignalStopRequest,
    configure_current_process_cpu_affinity,
    write_consumer_receipt,
    write_manifest,
)
from wujihand.runtime.isaac_dual_scene import (
    DualNeroHand2IsaacScene,
    SceneReplaySnapshot,
    resolve_dual_side_runtimes,
    workcell_frame_position,
)
from wujihand.runtime.isaac_d405_camera_capture import (
    DualD405CameraCapture,
    POSE_HISTORY_JOIN_TOLERANCE_NS,
    SIMULATION_CAMERA_CAPTURE_ADAPTER,
    SimulationCameraFrame,
    SimulationCameraStaticInventory,
    simulation_seconds_to_stamp_ns,
)
from wujihand.runtime.isaac_dual_teleoperation import (
    build_dual_teleoperation_application,
)
from wujihand.adapters.simulation import (
    load_nero_dual_tabletop_qualification_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_hand2_self_collision import (
    load_nero_hand2_self_collision_filter_profile,
)


DEFAULT_DEPLOYMENT = ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_live_v2.yaml"
DEFAULT_LOCAL_BINDING = ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"
NERO_LULA_DESCRIPTION = ROOT / "configs/profiles/agilex_nero_lula_kinematics_v1.yaml"
SELF_COLLISION_FILTER_PATH = (
    ROOT / "configs/profiles/isaac_nero_hand2_self_collision_filtered_pairs_v1.yaml"
)
OBLIQUE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
OBLIQUE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
SCREENSHOT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"
CONTROL_HZ = 60
RENDER_HZ = 20
CONTROL_TICKS_PER_CAPTURE = 2
GUI_MAXIMUM_CATCH_UP_TICKS = 2
GUI_BLOCK_ON_RENDER = False
VIEWPORT_WIDTH = 800
VIEWPORT_HEIGHT = 500
ISAAC_RENDERER = "MinimalRendering"
ISAAC_RECORDING_RENDERER = "RayTracedLighting"
ISAAC_MINIMAL_SHADING_MODE = 2
ISAAC_CPU_THREAD_LIMIT = 32
PYTHON_GC_POLICY = "collect_and_freeze_during_control_v1"
CAMERA_CAPTURE_EXECUTION = "paused_post_control_replay_v1"


@dataclass(frozen=True, slots=True)
class _CameraReplayState:
    control_tick_id: int
    physics_substep_index: int
    simulation_time_s: float
    scene: SceneReplaySnapshot


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
        help="Bounded 60 Hz control ticks; zero runs until the app closes.",
    )
    parser.add_argument(
        "--cpu-affinity",
        help="Linux CPU list for the Isaac consumer, for example 0-15.",
    )
    parser.add_argument("--report", type=Path)
    parser.add_argument(
        "--recording-enabled",
        action="store_true",
        help="Publish full raw-fact trace and close a run artifact.",
    )
    parser.add_argument("--run-id")
    parser.add_argument("--run-root", type=Path)
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
    if args.recording_enabled and (args.run_id is None or args.run_root is None):
        parser.error("--recording-enabled requires --run-id and --run-root")
    if args.recording_enabled and args.report is not None:
        parser.error("--report cannot be combined with recording mode")
    if not args.recording_enabled and (args.run_id is not None or args.run_root is not None):
        parser.error("--run-id/--run-root require --recording-enabled")
    return args


ARGS = parse_args()
ACTIVE_ISAAC_RENDERER = ISAAC_RECORDING_RENDERER if ARGS.recording_enabled else ISAAC_RENDERER
try:
    PROCESS_CPU_AFFINITY = configure_current_process_cpu_affinity(ARGS.cpu_affinity)
except (RuntimeError, ValueError) as exc:
    raise SystemExit(f"NV-5 ROS CPU affinity preflight failed: {exc}") from exc
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
if RESOLVED.control_profile.physics_hz != 120:
    raise SystemExit("NV-5.1 requires exactly 120 Hz physics")
if RESOLVED.control_profile.physics_hz % CONTROL_HZ != 0:
    raise SystemExit("physics_hz must be divisible by control_hz")
if CONTROL_HZ % RENDER_HZ != 0:
    raise SystemExit("control_hz must be divisible by render_hz")

PHYSICS_SUBSTEPS_PER_CONTROL = RESOLVED.control_profile.physics_hz // CONTROL_HZ
CONTROL_TICKS_PER_RENDER = CONTROL_HZ // RENDER_HZ
if PHYSICS_SUBSTEPS_PER_CONTROL != 2 or CONTROL_TICKS_PER_RENDER != 3:
    raise SystemExit("NV-5.1 requires 120/60/20 physics-control-render scheduling")

SIDES = resolve_dual_side_runtimes(ROOT, RESOLVED.session)
alignment_references = {
    RESOLVED.session.instance(runtime.arm_instance_id).binding.compatibility_profile
    for runtime in SIDES
}
if None in alignment_references or len(alignment_references) != 1:
    raise SystemExit("both NERO Bindings must use one geometry alignment profile")
ALIGNMENT_PATH = ROOT / str(next(iter(alignment_references)))
ALIGNMENT = load_nero_link_geometry_alignment(ALIGNMENT_PATH)
NERO_LULA_URDF = (ROOT / ALIGNMENT.source_urdf_path).resolve()
QUALIFICATION_PATH = ROOT / RESOLVED.control_profile.base_qualification.path
QUALIFICATION = load_nero_dual_tabletop_qualification_profile(QUALIFICATION_PATH)
SELF_COLLISION_FILTER = load_nero_hand2_self_collision_filter_profile(SELF_COLLISION_FILTER_PATH)
if not NERO_LULA_DESCRIPTION.is_file():
    raise SystemExit(f"NERO Lula descriptor not found: {NERO_LULA_DESCRIPTION}")
if sha256_file(NERO_LULA_URDF) != ALIGNMENT.source_urdf_sha256:
    raise SystemExit("source-locked NERO URDF hash drifted")

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp(
    {
        "headless": not ARGS.gui,
        "width": VIEWPORT_WIDTH,
        "height": VIEWPORT_HEIGHT,
        "anti_aliasing": 0,
        "renderer": ACTIVE_ISAAC_RENDERER,
        "minimal_shading_mode": ISAAC_MINIMAL_SHADING_MODE,
        "multi_gpu": False,
        "limit_cpu_threads": ISAAC_CPU_THREAD_LIMIT,
        "disable_viewport_updates": not ARGS.gui,
    }
)

import rclpy  # type: ignore[import-not-found]
import isaacsim.core.experimental.utils.app as app_utils  # type: ignore[import-not-found]
from rclpy.duration import Duration  # type: ignore[import-not-found]
from rclpy.executors import (  # type: ignore[import-not-found]
    SingleThreadedExecutor,
)
from rclpy.node import Node  # type: ignore[import-not-found]
from rclpy.signals import SignalHandlerOptions  # type: ignore[import-not-found]
from sensor_msgs.msg import CameraInfo, Image, JointState  # type: ignore[import-not-found]
from tf2_ros import (  # type: ignore[import-not-found]
    StaticTransformBroadcaster,
    TransformBroadcaster,
)
from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
    HandObservationEnvelope,
    RunRecordingStatus as RunRecordingStatusMessage,
    RouteCommand,
    SafetyEvent,
    SceneRigidBodyState as SceneRigidBodyStateMessage,
    SimulationCameraFrameTruth,
    TeleoperationTickTraceV2 as TeleoperationTickTraceMessage,
    TrackedRigidBodySample,
    TrackingLifecycleEvent,
)

from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
    set_camera_view,
)
from wujihand_ros2.conversion import (
    SafetyEventObservation,
    camera_dynamic_transform,
    camera_static_transform,
    route_command_from_decision,
    route_command_to_message,
    run_recording_status_to_message,
    safety_event_to_message,
    simulation_camera_frame_to_messages,
    scene_rigid_body_state_to_message,
    teleoperation_tick_trace_to_message,
)
from wujihand_ros2.executor_thread import RosExecutorThread
from wujihand_ros2.input_adapters import (
    RosHandSelection,
    RosHandObservationInputAdapter,
    RosInputSynchronization,
    RosTrackerSelection,
    RosTrackerInputAdapter,
    TrackerInputIdentity,
)
from wujihand_ros2.qos import qos_profile
from wujihand_ros2.recording import recording_topics


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
    completed_physics_steps = 0
    physics_steps_per_render = PHYSICS_SUBSTEPS_PER_CONTROL * CONTROL_TICKS_PER_RENDER
    for window in range(1, policy.maximum_windows + 1):
        for _ in range(policy.window_frames):
            scene.world.step(render=False)
            completed_physics_steps += 1
            if ARGS.gui and completed_physics_steps % physics_steps_per_render == 0:
                scene.world.render()
        current = {side: scene.feedback_q27(side).tolist() for side in ("left", "right")}
        if previous is not None:
            delta = q27_window_max_delta_rad(previous, current)
            deltas.append(delta)
            if window >= policy.minimum_windows and delta <= policy.max_window_delta_rad:
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


def _simulation_time_s(scene: DualNeroHand2IsaacScene) -> float:
    value = float(scene.world.current_time)
    if not np.isfinite(value) or value < 0.0:
        raise RuntimeError("Isaac simulation time must be finite and non-negative")
    return value


def _wait_for_recording_graph(
    node: Node,
    topics: tuple[str, ...],
    *,
    timeout_s: float = 180.0,
) -> None:
    """Do not begin a recorded control run before rosbag discovery closes."""

    deadline = time.monotonic() + timeout_s
    while True:
        pending = {
            topic: (2 if "/input/" in topic else 1)
            for topic in topics
            if node.count_subscribers(topic) < (2 if "/input/" in topic else 1)
        }
        if not pending:
            return
        if time.monotonic() >= deadline:
            detail = ", ".join(f"{topic}>={count}" for topic, count in sorted(pending.items()))
            raise RuntimeError(f"recording subscribers did not become ready: {detail}")
        time.sleep(0.05)


def _publish_camera_frames(
    frames: tuple[SimulationCameraFrame, ...],
    *,
    inventories: dict[str, SimulationCameraStaticInventory],
    publishers: dict[str, dict[str, Any]],
    transform_broadcaster: TransformBroadcaster,
    counters: Counter[str],
) -> None:
    """Publish only identity-joined completed frames on the control thread."""

    for frame in frames:
        inventory = inventories[frame.side]
        messages = simulation_camera_frame_to_messages(frame, inventory)
        transform_broadcaster.sendTransform(camera_dynamic_transform(frame, inventory))
        side_publishers = publishers[frame.side]
        side_publishers["color"].publish(messages.color)
        side_publishers["depth"].publish(messages.depth)
        side_publishers["camera_info"].publish(messages.camera_info)
        side_publishers["truth"].publish(messages.truth)
        counters[f"camera.{frame.side}.published_frames"] += 1


def _render_without_simulation_advance(
    scene: DualNeroHand2IsaacScene,
    *,
    camera_capture: DualD405CameraCapture | None,
    camera_render_due: bool,
    counters: Counter[str],
) -> tuple[SimulationCameraFrame, ...]:
    """Service rendering without charging it to a 60 Hz control tick."""

    simulation_time_s = _simulation_time_s(scene)
    started_ns = time.monotonic_ns()
    scene.world.render()
    duration_ns = time.monotonic_ns() - started_ns
    counters["camera.render_update_total_ns"] += duration_ns
    counters["camera.render_update_max_ns"] = max(
        counters["camera.render_update_max_ns"],
        duration_ns,
    )
    if not math.isclose(
        _simulation_time_s(scene),
        simulation_time_s,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise RuntimeError("rendering changed simulation time")
    if camera_capture is None:
        return ()
    if camera_render_due:
        counters["camera.render_updates"] += 1
    return camera_capture.drain_completed()


def _publish_camera_frames_measured(
    frames: tuple[SimulationCameraFrame, ...],
    *,
    inventories: dict[str, SimulationCameraStaticInventory],
    publishers: dict[str, dict[str, Any]],
    transform_broadcaster: TransformBroadcaster,
    counters: Counter[str],
) -> None:
    if not frames:
        return
    started_ns = time.monotonic_ns()
    _publish_camera_frames(
        frames,
        inventories=inventories,
        publishers=publishers,
        transform_broadcaster=transform_broadcaster,
        counters=counters,
    )
    duration_ns = time.monotonic_ns() - started_ns
    counters["camera.publish_total_ns"] += duration_ns
    counters["camera.publish_max_ns"] = max(
        counters["camera.publish_max_ns"],
        duration_ns,
    )


def _replay_camera_frames(
    states: tuple[_CameraReplayState, ...],
    *,
    scene: DualNeroHand2IsaacScene,
    camera_capture: DualD405CameraCapture,
    inventories: dict[str, SimulationCameraStaticInventory],
    publishers: dict[str, dict[str, Any]],
    transform_broadcaster: TransformBroadcaster,
    counters: Counter[str],
) -> tuple[SimulationCameraFrame, ...]:
    """Render recorded 30 Hz states only after the real-time control segment."""

    if not states:
        return ()
    scene.world.reset()
    pending: list[SimulationCameraFrame] = []
    camera_rate_hz = inventories["left"].profile.capture.rate_hz
    if inventories["right"].profile.capture.rate_hz != camera_rate_hz:
        raise RuntimeError("dual D405 replay rates differ")
    first_target_step_count = round(
        states[0].simulation_time_s * RESOLVED.control_profile.physics_hz
    )
    capture_period_steps = PHYSICS_SUBSTEPS_PER_CONTROL * CONTROL_TICKS_PER_CAPTURE
    prime_step_count = first_target_step_count - capture_period_steps
    if prime_step_count < 0:
        raise RuntimeError("first D405 replay state has no preceding priming deadline")
    for _ in range(prime_step_count):
        scene.world.step(render=False)
        counters["camera.replay_physics_steps"] += 1
    scene.restore_camera_replay_snapshot(states[0].scene)
    priming = camera_capture.prime_after_timeline_reset(
        render_update=scene.world.render,
        simulation_time_s=_simulation_time_s(scene),
    )
    counters["camera.replay_priming_updates"] += int(priming["render_updates"])
    discarded_frames = priming["discarded_frames"]
    if not isinstance(discarded_frames, dict):
        raise RuntimeError("D405 replay priming receipt is invalid")
    counters["camera.replay_priming_discarded_frames"] += sum(
        int(value) for value in discarded_frames.values()
    )
    for state in states:
        target_stamp_ns = simulation_seconds_to_stamp_ns(state.simulation_time_s)
        current_stamp_ns = simulation_seconds_to_stamp_ns(_simulation_time_s(scene))
        while current_stamp_ns < target_stamp_ns:
            scene.world.step(render=False)
            counters["camera.replay_physics_steps"] += 1
            current_stamp_ns = simulation_seconds_to_stamp_ns(_simulation_time_s(scene))
        if current_stamp_ns != target_stamp_ns:
            raise RuntimeError(
                "camera replay could not reproduce capture simulation time: "
                f"target={target_stamp_ns}, current={current_stamp_ns}"
            )
        scene.restore_camera_replay_snapshot(state.scene)
        pending.extend(
            camera_capture.observe_completed_substep(
                control_tick_id=state.control_tick_id,
                physics_substep_index=state.physics_substep_index,
                physics_substep_ordinal=1,
                simulation_time_s=state.simulation_time_s,
            )
        )
        pending.extend(
            _render_without_simulation_advance(
                scene,
                camera_capture=camera_capture,
                camera_render_due=True,
                counters=counters,
            )
        )
        if pending:
            _publish_camera_frames_measured(
                tuple(pending),
                inventories=inventories,
                publishers=publishers,
                transform_broadcaster=transform_broadcaster,
                counters=counters,
            )
            pending.clear()
    # RTX completion can trail replay submission after World.reset(). Service
    # bounded renders at the fixed final simulation time, and detach as soon as
    # the exact requested count closes.
    expected_count = len(states)
    replay_simulation_time_s = _simulation_time_s(scene)
    pending.extend(
        camera_capture.stop_and_drain(
            update_app=scene.world.render,
            expected_frames_per_side=expected_count,
        )
    )
    if not math.isclose(
        _simulation_time_s(scene),
        replay_simulation_time_s,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise RuntimeError("paused Kit updates changed D405 replay simulation time")
    if pending:
        _publish_camera_frames_measured(
            tuple(pending),
            inventories=inventories,
            publishers=publishers,
            transform_broadcaster=transform_broadcaster,
            counters=counters,
        )
        pending.clear()
    return tuple(pending)


def main() -> int:
    scene = DualNeroHand2IsaacScene(
        project_root=ROOT,
        resolved=RESOLVED.session,
        sides=SIDES,
        alignment_profile=ALIGNMENT,
        qualification_profile=QUALIFICATION,
        physics_hz=RESOLVED.control_profile.physics_hz,
        self_collision_sides=frozenset({"left", "right"}),
        self_collision_filter_profile=SELF_COLLISION_FILTER,
        wrist_rig_collision_mode="all",
    )
    scene.world.set_block_on_render(GUI_BLOCK_ON_RENDER)
    if bool(scene.world.get_block_on_render()) is not GUI_BLOCK_ON_RENDER:
        raise RuntimeError("Isaac render blocking policy was not applied")
    readiness = _settle(scene)
    camera_capture: DualD405CameraCapture | None = None
    camera_warmup: dict[str, object] | None = None
    if ARGS.recording_enabled:
        assert ARGS.run_id is not None
        camera_capture = DualD405CameraCapture(
            project_root=ROOT,
            scene=scene,
            run_id=ARGS.run_id,
        )
        camera_warmup = camera_capture.warm_up(
            update_app=app_utils.update_app,
            simulation_time_s=lambda: _simulation_time_s(scene),
        )
        inventories = camera_capture.inventories
        camera_rate_hz = inventories[0].profile.capture.rate_hz
        if any(
            inventory.profile.capture.rate_hz != camera_rate_hz for inventory in inventories[1:]
        ):
            raise RuntimeError("dual D405 capture rates differ")
        alignment_steps = 0
        maximum_alignment_steps = inventories[0].profile.schedule.physics_substeps_per_capture
        for alignment_steps in range(maximum_alignment_steps + 1):
            phase = _simulation_time_s(scene) * camera_rate_hz
            if math.isclose(phase, round(phase), rel_tol=0.0, abs_tol=5e-6):
                break
            if alignment_steps == maximum_alignment_steps:
                raise RuntimeError("unable to align D405 activation to a 30 Hz boundary")
            scene.world.step(render=False)
        aligned_simulation_time_s = _simulation_time_s(scene)
        scene.world.render()
        if not math.isclose(
            _simulation_time_s(scene),
            aligned_simulation_time_s,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise RuntimeError("D405 activation alignment render changed simulation time")
        camera_warmup["activation_alignment"] = {
            "physics_steps": alignment_steps,
            "simulation_time_s": aligned_simulation_time_s,
            "camera_phase": aligned_simulation_time_s * camera_rate_hz,
            "rendered_boundary_before_activation": True,
        }
    # SimulationApp and rclpy both install process-level handlers by default.
    # The consumer must own SIGINT/SIGTERM so launch cannot bypass the terminal
    # recording status and atomic receipt hand-off.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(
        _node_binding_name(),
        namespace=f"/{RESOLVED.deployment.root_namespace}",
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor_worker = RosExecutorThread(executor)
    input_synchronization = RosInputSynchronization()

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
                    tracking_setup_revision=(RESOLVED.deployment.tracking_setup.setup_revision),
                    tracking_frame=RESOLVED.mapping.tracking_frame,
                ),
                synchronization=input_synchronization,
            )
            tracker_inputs[side] = adapter
            subscriptions.append(
                node.create_subscription(
                    TrackedRigidBodySample,
                    f"input/tracker/{side}/sample",
                    adapter.offer_message,
                    qos_profile(RESOLVED.qos_profile.policy("tracker_sample")),
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
                synchronization=input_synchronization,
            )
            hand_inputs[hand_side] = hand_adapter
            subscriptions.append(
                node.create_subscription(
                    HandObservationEnvelope,
                    f"input/glove/{side}/observation",
                    hand_adapter.offer_message,
                    qos_profile(RESOLVED.qos_profile.policy("glove_observation")),
                )
            )

    def observe_lifecycle(message: TrackingLifecycleEvent) -> None:
        with input_synchronization.locked():
            for adapter in tracker_inputs.values():
                adapter.offer_lifecycle_message(message)

    subscriptions.append(
        node.create_subscription(
            TrackingLifecycleEvent,
            "input/tracker/lifecycle",
            observe_lifecycle,
            qos_profile(RESOLVED.qos_profile.policy("tracking_lifecycle")),
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
    command_publishers: dict[tuple[str, str], Any] = {}
    feedback_publishers: dict[tuple[str, str], Any] = {}
    safety_publishers: dict[tuple[str, str], Any] = {}
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

    trace_publisher = None
    scene_state_publisher = None
    recording_status_publisher = None
    camera_publishers: dict[str, dict[str, Any]] = {}
    camera_inventories: dict[str, SimulationCameraStaticInventory] = {}
    camera_transform_broadcaster: TransformBroadcaster | None = None
    camera_static_transform_broadcaster: StaticTransformBroadcaster | None = None
    counters: Counter[str] = Counter()
    current_run_root: Path | None = None
    current_run_id: str | None = None
    if ARGS.recording_enabled:
        assert ARGS.run_root is not None
        assert ARGS.run_id is not None
        current_run_root = ARGS.run_root.resolve()
        current_run_id = ARGS.run_id
        trace_publisher = node.create_publisher(
            TeleoperationTickTraceMessage,
            "runtime/tick",
            qos_profile(RESOLVED.qos_profile.policy("trace_event")),
        )
        scene_state_publisher = node.create_publisher(
            SceneRigidBodyStateMessage,
            "scene/rigid_body_state",
            qos_profile(RESOLVED.qos_profile.policy("scene_state")),
        )
        recording_status_publisher = node.create_publisher(
            RunRecordingStatusMessage,
            "recording/status",
            qos_profile(RESOLVED.qos_profile.policy("run_status")),
        )
        if camera_capture is None:
            raise RuntimeError("recording mode did not create dual D405 capture")
        camera_inventories = {inventory.side: inventory for inventory in camera_capture.inventories}
        for side in ("left", "right"):
            base = f"{side}/wrist_camera"
            camera_publishers[side] = {
                "color": node.create_publisher(
                    Image,
                    f"{base}/color/image_raw",
                    qos_profile(RESOLVED.qos_profile.policy("camera_image")),
                ),
                "depth": node.create_publisher(
                    Image,
                    f"{base}/depth/image_raw",
                    qos_profile(RESOLVED.qos_profile.policy("camera_image")),
                ),
                "camera_info": node.create_publisher(
                    CameraInfo,
                    f"{base}/camera_info",
                    qos_profile(RESOLVED.qos_profile.policy("camera_info")),
                ),
                "truth": node.create_publisher(
                    SimulationCameraFrameTruth,
                    f"{base}/frame_truth",
                    qos_profile(RESOLVED.qos_profile.policy("camera_truth")),
                ),
            }
        camera_transform_broadcaster = TransformBroadcaster(node)
        camera_static_transform_broadcaster = StaticTransformBroadcaster(node)
        camera_static_transform_broadcaster.sendTransform(
            [camera_static_transform(camera_inventories[side]) for side in ("left", "right")]
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
    recording_opened_ns = time.monotonic_ns()
    if current_run_root is not None and current_run_id is not None:
        write_manifest(
            current_run_root,
            run_id=current_run_id,
            payload=_run_manifest_payload(
                scene=scene,
                recording_opened_ns=recording_opened_ns,
                camera_capture=camera_capture,
                camera_warmup=camera_warmup,
            ),
        )
        _wait_for_recording_graph(
            node,
            recording_topics(
                f"/{RESOLVED.deployment.root_namespace}",
                RESOLVED.route_plan,
                include_synthetic_d405=True,
            ),
        )
        assert camera_capture is not None
        camera_capture.activate(simulation_time_s=_simulation_time_s(scene))
    started_ns = time.monotonic_ns()
    application.start(now_ns=started_ns)
    scene.apply_targets()
    safety_state: dict[tuple[str, str], tuple[object, ...]] = {}
    completed_frames = 0
    completed_physics_steps = 0
    completed_renders = 0
    pending_camera_frames: list[SimulationCameraFrame] = []
    camera_replay_states: list[_CameraReplayState] = []
    active_tracker_sources: dict[
        str,
        SourceSelectionTrace | None,
    ] = {"left": None, "right": None}
    active_hand_sources: dict[
        str,
        SourceSelectionTrace | None,
    ] = {"left": None, "right": None}
    print(
        "NV5 ROS CONSUMER READY: "
        f"deployment={RESOLVED.deployment.deployment_id} "
        f"trackers={sorted(tracker_inputs)} "
        f"gloves={sorted(side.value for side in hand_inputs)}",
        flush=True,
    )
    if recording_status_publisher is not None and current_run_id is not None:
        recording_status_publisher.publish(
            run_recording_status_to_message(
                RunRecordingStatus(
                    run_id=current_run_id,
                    state=RunRecordingState.STARTED,
                    reason="consumer_started",
                    host_time_ns=started_ns,
                )
            )
        )
    stop_request = SignalStopRequest()
    previous_signal_handlers = {
        current: signal.signal(current, stop_request) for current in (signal.SIGINT, signal.SIGTERM)
    }
    failure_reason: str | None = None
    recording_failure_reason: str | None = None
    loop_failed = False
    cleanup_error: Exception | None = None
    receipt_error: Exception | None = None
    camera_capture_receipt: dict[str, object] | None = None
    python_gc_frozen = False
    python_gc_frozen_object_count = 0
    python_gc_unfrozen_on_close = False
    try:
        gc.collect()
        gc.freeze()
        python_gc_frozen = True
        python_gc_frozen_object_count = gc.get_freeze_count()
        executor_worker.start()
        scheduler = FixedRateScheduler(
            rate_hz=CONTROL_HZ,
            start_ns=time.monotonic_ns(),
            maximum_catch_up_ticks=(GUI_MAXIMUM_CATCH_UP_TICKS if ARGS.gui else 0),
        )
        while (
            not stop_request.requested
            and simulation_app.is_running()
            and (ARGS.frames == 0 or completed_frames < ARGS.frames)
        ):
            executor_worker.raise_if_failed()
            scheduled_tick = scheduler.wait_next()
            counters["scheduler.missed_control_periods"] += (
                scheduled_tick.missed_periods_before_tick
            )
            with input_synchronization.locked():
                tick_ns = time.monotonic_ns()
                snapshot_start_ns = tick_ns
                tracker_snapshots = {
                    side: adapter.snapshot_for_tick(now_ns=tick_ns)
                    for side, adapter in tracker_inputs.items()
                }
                hand_snapshots = {
                    side: adapter.snapshot_for_tick(receive_time_ns=tick_ns)
                    for side, adapter in hand_inputs.items()
                }
                snapshot_end_ns = time.monotonic_ns()
            for side, snapshot in tracker_snapshots.items():
                if snapshot.reference_invalidated:
                    application.arm_controllers[side].invalidate_reference()
                    active_tracker_sources[side] = None
                    counters[f"{side}.tracker_epoch_changes"] += 1
            for side, snapshot in hand_snapshots.items():
                if snapshot.epoch_changed:
                    application.hand_controllers.invalidate_input_epoch(
                        side,
                    )
                    active_hand_sources[side.value] = None
                    counters[f"{side.value}.glove_epoch_changes"] += 1

            pre_feedback = {side: scene.feedback_q27(side) for side in ("left", "right")}
            control_start_ns = time.monotonic_ns()
            result = application.cycle.step(
                feedback_q7_rad={
                    side: pre_feedback[side][application.arm_indices[side]].tolist()
                    for side in application.arm_controllers
                },
                now_ns=tick_ns,
            )
            control_end_ns = time.monotonic_ns()
            arm_steps = {labelled.side: labelled.step for labelled in result.arm_steps}
            hand_steps = {labelled.side.value: labelled.step for labelled in result.hand_steps}
            for arm_labelled in result.arm_steps:
                side = arm_labelled.side
                arm_step = arm_labelled.step
                route = RESOLVED.route_plan.route(
                    f"nero_{side}",
                    "arm_joints",
                )
                scene.arm_targets[side] = arm_step.safety.command.copy()
                _publish_route_command(
                    route=route,
                    layout_id=scene.arm_profiles[side].layout_id,
                    decision=arm_step.safety,
                    tick_ns=tick_ns,
                    command_publishers=command_publishers,
                    safety_publishers=safety_publishers,
                    safety_state=safety_state,
                )
                counters[f"{side}.arm.{arm_step.reason}"] += 1
            for hand_labelled in result.hand_steps:
                side = hand_labelled.side.value
                hand_step = hand_labelled.step
                route = RESOLVED.route_plan.route(
                    f"hand_{side}",
                    "finger_joints",
                )
                scene.hand_targets[side] = hand_step.decision.command.copy()
                _publish_route_command(
                    route=route,
                    layout_id=scene.hand_profiles[side].layout_id,
                    decision=hand_step.decision,
                    tick_ns=tick_ns,
                    command_publishers=command_publishers,
                    safety_publishers=safety_publishers,
                    safety_state=safety_state,
                )
                counters[
                    f"{side}.hand.{hand_step.rejection_reason or hand_step.decision.reason}"
                ] += 1
            apply_start_ns = time.monotonic_ns()
            applied_targets = scene.apply_targets()
            apply_end_ns = time.monotonic_ns()
            simulation_time_before_s = _simulation_time_s(scene)
            physics_start_ns = time.monotonic_ns()
            physics_substep_indices: list[int] = []
            physics_substep_sim_times_s: list[float] = []
            physics_substep_start_ns: list[int] = []
            physics_substep_end_ns: list[int] = []
            completed_camera_frames: list[SimulationCameraFrame] = []
            render_due = (
                ARGS.gui and (scheduled_tick.control_index + 1) % CONTROL_TICKS_PER_RENDER == 0
            )
            camera_render_due = camera_capture is not None and (
                (scheduled_tick.control_index + 1) % CONTROL_TICKS_PER_CAPTURE == 0
            )
            render_index = completed_renders if render_due else None
            for substep in range(PHYSICS_SUBSTEPS_PER_CONTROL):
                physics_substep_indices.append(completed_physics_steps)
                physics_substep_start_ns.append(time.monotonic_ns())
                world_step_start_ns = time.monotonic_ns()
                scene.world.step(render=False)
                world_step_duration_ns = time.monotonic_ns() - world_step_start_ns
                counters["physics.world_step_total_ns"] += world_step_duration_ns
                counters["physics.world_step_max_ns"] = max(
                    counters["physics.world_step_max_ns"],
                    world_step_duration_ns,
                )
                physics_substep_end_ns.append(time.monotonic_ns())
                physics_substep_sim_time_s = _simulation_time_s(scene)
                physics_substep_sim_times_s.append(physics_substep_sim_time_s)
                if camera_capture is not None and ARGS.gui:
                    camera_observe_start_ns = time.monotonic_ns()
                    completed_camera_frames.extend(
                        camera_capture.observe_completed_substep(
                            control_tick_id=scheduled_tick.control_index,
                            physics_substep_index=completed_physics_steps,
                            physics_substep_ordinal=substep,
                            simulation_time_s=physics_substep_sim_time_s,
                        )
                    )
                    camera_observe_duration_ns = time.monotonic_ns() - camera_observe_start_ns
                    counters["camera.observe_total_ns"] += camera_observe_duration_ns
                    counters["camera.observe_max_ns"] = max(
                        counters["camera.observe_max_ns"],
                        camera_observe_duration_ns,
                    )
                completed_physics_steps += 1
            physics_end_ns = time.monotonic_ns()
            simulation_time_after_s = _simulation_time_s(scene)
            if render_due or (ARGS.gui and camera_render_due):
                # World.step(render=True) advances by rendering_dt and therefore
                # cannot represent one 120 Hz physics substep. Render separately
                # so the UI never changes simulation time.
                completed_camera_frames.extend(
                    _render_without_simulation_advance(
                        scene,
                        camera_capture=camera_capture,
                        camera_render_due=camera_render_due,
                        counters=counters,
                    )
                )
                if render_due:
                    completed_renders += 1
            post_feedback = {side: scene.feedback_q27(side) for side in ("left", "right")}
            if camera_render_due and not ARGS.gui:
                replay_snapshot_start_ns = time.monotonic_ns()
                replay_snapshot = scene.camera_replay_snapshot(
                    q27_by_side=post_feedback,
                )
                replay_snapshot_duration_ns = time.monotonic_ns() - replay_snapshot_start_ns
                counters["camera.replay_snapshot_total_ns"] += replay_snapshot_duration_ns
                counters["camera.replay_snapshot_max_ns"] = max(
                    counters["camera.replay_snapshot_max_ns"],
                    replay_snapshot_duration_ns,
                )
                camera_replay_states.append(
                    _CameraReplayState(
                        control_tick_id=scheduled_tick.control_index,
                        physics_substep_index=physics_substep_indices[-1],
                        simulation_time_s=simulation_time_after_s,
                        scene=replay_snapshot,
                    )
                )
            for arm_labelled in result.arm_steps:
                side = arm_labelled.side
                route = RESOLVED.route_plan.route(
                    f"nero_{side}",
                    "arm_joints",
                )
                _publish_route_feedback(
                    node=node,
                    route=route,
                    feedback=post_feedback[side][application.arm_indices[side]],
                    joint_names=scene.arm_profiles[side].layout.names,
                    feedback_publishers=feedback_publishers,
                )
            for hand_labelled in result.hand_steps:
                side = hand_labelled.side.value
                route = RESOLVED.route_plan.route(
                    f"hand_{side}",
                    "finger_joints",
                )
                _publish_route_feedback(
                    node=node,
                    route=route,
                    feedback=post_feedback[side][
                        np.asarray(
                            scene.partitions[side].hand_indices_q20,
                            dtype=np.int64,
                        )
                    ],
                    joint_names=scene.hand_profiles[side].layout.names,
                    feedback_publishers=feedback_publishers,
                )
            trace_time_ns = time.monotonic_ns()
            if (
                trace_publisher is not None
                and current_run_id is not None
                and recording_failure_reason is None
            ):
                try:
                    _publish_recording_tick(
                        run_id=current_run_id,
                        tick_id=scheduled_tick.control_index,
                        stage_times=TickStageTimes(
                            tick_time_ns=tick_ns,
                            snapshot_start_ns=snapshot_start_ns,
                            snapshot_end_ns=snapshot_end_ns,
                            control_start_ns=control_start_ns,
                            control_end_ns=control_end_ns,
                            apply_start_ns=apply_start_ns,
                            apply_end_ns=apply_end_ns,
                            physics_start_ns=physics_start_ns,
                            physics_end_ns=physics_end_ns,
                            trace_time_ns=trace_time_ns,
                        ),
                        execution=TickExecutionTrace(
                            control_index=scheduled_tick.control_index,
                            schedule_slot=scheduled_tick.schedule_slot,
                            scheduled_control_time_ns=scheduled_tick.deadline_ns,
                            control_lateness_ns=(tick_ns - scheduled_tick.deadline_ns),
                            missed_control_periods_before_tick=(
                                scheduled_tick.missed_periods_before_tick
                            ),
                            simulation_time_before_s=simulation_time_before_s,
                            simulation_time_after_s=simulation_time_after_s,
                            target_effective_start_sim_time_s=(simulation_time_before_s),
                            target_effective_end_sim_time_s=simulation_time_after_s,
                            physics_substep_indices=tuple(physics_substep_indices),
                            physics_substep_sim_times_s=tuple(physics_substep_sim_times_s),
                            physics_substep_start_ns=tuple(physics_substep_start_ns),
                            physics_substep_end_ns=tuple(physics_substep_end_ns),
                            rendered=render_due,
                            render_index=render_index,
                        ),
                        trace_publisher=trace_publisher,
                        scene_state_publisher=scene_state_publisher,
                        scene=scene,
                        pre_feedback=pre_feedback,
                        applied_targets=applied_targets,
                        post_feedback=post_feedback,
                        arm_steps=arm_steps,
                        hand_steps=hand_steps,
                        tracker_inputs=tracker_inputs,
                        hand_inputs=hand_inputs,
                        active_tracker_sources=active_tracker_sources,
                        active_hand_sources=active_hand_sources,
                    )
                except Exception as exc:
                    recording_failure_reason = _bounded_reason(exc)
                    counters["recording.trace_failures"] += 1
                    print(
                        f"NV5 RECORDING DEGRADED: {recording_failure_reason}",
                        file=sys.stderr,
                        flush=True,
                    )
            completed_frames += 1
            scheduler.complete(completed_ns=time.monotonic_ns())
            if completed_camera_frames:
                if camera_transform_broadcaster is None:
                    raise RuntimeError("camera TF broadcaster is missing")
                _publish_camera_frames_measured(
                    tuple(completed_camera_frames),
                    inventories=camera_inventories,
                    publishers=camera_publishers,
                    transform_broadcaster=camera_transform_broadcaster,
                    counters=counters,
                )
        if camera_capture is not None and not ARGS.gui:
            if camera_transform_broadcaster is None:
                raise RuntimeError("camera TF broadcaster is missing during replay")
            pending_camera_frames.extend(
                _replay_camera_frames(
                    tuple(camera_replay_states),
                    scene=scene,
                    camera_capture=camera_capture,
                    inventories=camera_inventories,
                    publishers=camera_publishers,
                    transform_broadcaster=camera_transform_broadcaster,
                    counters=counters,
                )
            )
    except BaseException as exc:
        loop_failed = True
        failure_reason = _bounded_reason(exc)
        raise
    finally:
        try:
            if camera_capture is not None:
                drain_simulation_time_s = _simulation_time_s(scene)
                expected_camera_frames = completed_frames // CONTROL_TICKS_PER_CAPTURE
                if ARGS.gui:
                    final_camera_frames = (
                        *pending_camera_frames,
                        *camera_capture.stop_and_drain(
                            update_app=scene.world.render,
                            expected_frames_per_side=expected_camera_frames,
                        ),
                    )
                else:
                    capture_counts = camera_capture.capture_counts
                    if any(
                        capture_counts[side] != expected_camera_frames for side in ("left", "right")
                    ):
                        raise RuntimeError(
                            "paused D405 replay did not close the 30 Hz schedule: "
                            f"expected={expected_camera_frames}, actual={capture_counts!r}"
                        )
                    final_camera_frames = tuple(pending_camera_frames)
                pending_camera_frames.clear()
                if not math.isclose(
                    _simulation_time_s(scene),
                    drain_simulation_time_s,
                    rel_tol=0.0,
                    abs_tol=1e-9,
                ):
                    raise RuntimeError("draining D405 captures changed simulation time")
                if final_camera_frames:
                    if camera_transform_broadcaster is None:
                        raise RuntimeError("camera TF broadcaster is missing during drain")
                    _publish_camera_frames_measured(
                        final_camera_frames,
                        inventories=camera_inventories,
                        publishers=camera_publishers,
                        transform_broadcaster=camera_transform_broadcaster,
                        counters=counters,
                    )
                camera_capture.close()
                camera_capture_receipt = camera_capture.receipt(
                    publish_counts={
                        side: counters[f"camera.{side}.published_frames"]
                        for side in ("left", "right")
                    }
                )
                camera_capture_receipt["capture_execution"] = (
                    "inline_gui_render_v1" if ARGS.gui else CAMERA_CAPTURE_EXECUTION
                )
                camera_capture_receipt["replay_state_count"] = len(camera_replay_states)
                sides_receipt = camera_capture_receipt["sides"]
                if not isinstance(sides_receipt, dict):
                    raise RuntimeError("camera receipt sides mapping is invalid")
                for side in ("left", "right"):
                    side_receipt = sides_receipt.get(side)
                    if (
                        not isinstance(side_receipt, dict)
                        or side_receipt.get("capture_count") != expected_camera_frames
                        or side_receipt.get("publish_count") != expected_camera_frames
                    ):
                        raise RuntimeError(
                            f"{side} D405 capture/publish count differs from 30 Hz schedule"
                        )
        except Exception as exc:
            cleanup_error = exc
            if failure_reason is None:
                failure_reason = _bounded_reason(exc)
        try:
            executor_worker.stop()
        except Exception as exc:
            cleanup_error = exc
            if failure_reason is None:
                failure_reason = _bounded_reason(exc)
        try:
            if python_gc_frozen:
                gc.unfreeze()
                python_gc_unfrozen_on_close = gc.get_freeze_count() == 0
        except Exception as exc:
            if cleanup_error is None:
                cleanup_error = exc
            if failure_reason is None:
                failure_reason = _bounded_reason(exc)
        try:
            application.close()
        except Exception as exc:
            if cleanup_error is None:
                cleanup_error = exc
            if failure_reason is None:
                failure_reason = _bounded_reason(exc)
        closed_ns = time.monotonic_ns()
        state = (
            RunRecordingState.CONSUMER_COMPLETED
            if failure_reason is None and recording_failure_reason is None
            else RunRecordingState.INCOMPLETE
        )
        if current_run_root is not None and current_run_id is not None:
            if recording_status_publisher is not None:
                try:
                    terminal_reason = (
                        "consumer_completed"
                        if state is RunRecordingState.CONSUMER_COMPLETED
                        else (failure_reason or recording_failure_reason or "recording_incomplete")
                    )
                    recording_status_publisher.publish(
                        run_recording_status_to_message(
                            RunRecordingStatus(
                                run_id=current_run_id,
                                state=state,
                                reason=terminal_reason,
                                host_time_ns=closed_ns,
                            )
                        )
                    )
                    if not recording_status_publisher.wait_for_all_acked(Duration(seconds=2.0)):
                        recording_failure_reason = "recording_status_ack_timeout"
                        state = RunRecordingState.INCOMPLETE
                    else:
                        counters["recording.terminal_status_acked"] += 1
                except Exception as exc:
                    recording_failure_reason = f"recording_status_failed:{type(exc).__name__}"
                    state = RunRecordingState.INCOMPLETE
            try:
                # The recorder wrapper treats this atomic receipt as the final
                # hand-off. Publish and acknowledge terminal ROS status first.
                write_consumer_receipt(
                    current_run_root,
                    run_id=current_run_id,
                    state=state,
                    payload=_run_receipt_payload(
                        completed_frames=completed_frames,
                        completed_physics_steps=completed_physics_steps,
                        completed_renders=completed_renders,
                        started_ns=started_ns,
                        closed_ns=closed_ns,
                        readiness=readiness,
                        counters=counters,
                        tracker_inputs=tracker_inputs,
                        hand_inputs=hand_inputs,
                        executor_metrics=asdict(executor_worker.metrics),
                        python_gc_frozen_object_count=(python_gc_frozen_object_count),
                        python_gc_unfrozen_on_close=(python_gc_unfrozen_on_close),
                        stop_signal=stop_request.requested_signal,
                        failure_reason=failure_reason,
                        recording_failure_reason=(recording_failure_reason),
                        camera_capture_receipt=camera_capture_receipt,
                    ),
                )
            except Exception as exc:
                receipt_error = exc
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        finally:
            for current, previous in previous_signal_handlers.items():
                signal.signal(current, previous)
        if not loop_failed:
            if cleanup_error is not None:
                raise cleanup_error
            if receipt_error is not None:
                raise receipt_error

    if current_run_root is not None:
        print(
            f"NV5 ROS CONSUMER CLOSED: run_id={current_run_id} root={current_run_root}",
            flush=True,
        )
        return 0

    report = {
        "schema": "wujihand.isaac_ros_dual_teleoperation_receipt.v2",
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
        "completed_physics_steps": completed_physics_steps,
        "completed_renders": completed_renders,
        "readiness": readiness,
        "counters": dict(counters),
        "synthetic_d405_wrist_rigs": {
            "materialized_sides": [item.side for item in scene.wrist_rigs],
            "camera_prims": [item.camera_prim_path for item in scene.wrist_rigs],
            "capture_enabled": False,
            "data_render_products_created": 0,
            "camera_publishers_created": 0,
            "simulation_only_140_degree": True,
        },
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
        "executor": asdict(executor_worker.metrics),
        "block_on_render": GUI_BLOCK_ON_RENDER,
        "python_gc": {
            "policy": PYTHON_GC_POLICY,
            "frozen_object_count": python_gc_frozen_object_count,
            "unfrozen_on_close": python_gc_unfrozen_on_close,
        },
        "state": "consumer_completed",
    }
    report_path = ARGS.report
    if report_path is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
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


def _publish_route_command(
    *,
    route: object,
    layout_id: str,
    decision: object,
    tick_ns: int,
    command_publishers: dict[tuple[str, str], Any],
    safety_publishers: dict[tuple[str, str], Any],
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


def _publish_route_feedback(
    *,
    node: Node,
    route: object,
    feedback: NDArray[np.float64],
    joint_names: tuple[str, ...],
    feedback_publishers: dict[tuple[str, str], Any],
) -> None:
    from wujihand.runtime import DualTeleoperationRoute

    if not isinstance(route, DualTeleoperationRoute):
        raise TypeError("route must be a DualTeleoperationRoute")
    key = (route.instance_id, route.group_id)
    feedback_message = JointState()
    feedback_message.header.stamp = node.get_clock().now().to_msg()
    feedback_message.name = list(joint_names)
    feedback_message.position = [float(value) for value in feedback]
    feedback_publishers[key].publish(feedback_message)


def _publish_recording_tick(
    *,
    run_id: str,
    tick_id: int,
    stage_times: TickStageTimes,
    execution: TickExecutionTrace,
    trace_publisher: Any,
    scene_state_publisher: Any | None,
    scene: DualNeroHand2IsaacScene,
    pre_feedback: dict[str, NDArray[np.float64]],
    applied_targets: dict[str, NDArray[np.float64]],
    post_feedback: dict[str, NDArray[np.float64]],
    arm_steps: dict[str, TrackerArmSimulationStep],
    hand_steps: dict[str, Hand2SimulationStep],
    tracker_inputs: dict[str, RosTrackerInputAdapter],
    hand_inputs: dict[HandSide, RosHandObservationInputAdapter],
    active_tracker_sources: dict[
        str,
        SourceSelectionTrace | None,
    ],
    active_hand_sources: dict[
        str,
        SourceSelectionTrace | None,
    ],
) -> None:
    for side in ("left", "right"):
        selected_tracker = _tracker_source_trace(tracker_inputs[side].selected)
        arm_mapping = arm_steps[side].mapping
        if arm_mapping is None or arm_mapping.requires_reference:
            active_tracker_sources[side] = None
        elif (
            selected_tracker is not None
            and arm_mapping.input_host_time_ns == selected_tracker.source_time_ns
        ):
            active_tracker_sources[side] = selected_tracker
        if (
            arm_mapping is not None
            and not arm_mapping.requires_reference
            and active_tracker_sources[side] is None
        ):
            raise RuntimeError(f"{side} arm mapping lost source provenance")

        hand_side = HandSide(side)
        selected_hand = (
            _hand_source_trace(hand_inputs[hand_side].selected)
            if hand_side in hand_inputs
            else None
        )
        current_hand_step = hand_steps.get(side)
        if current_hand_step is None:
            active_hand_sources[side] = None
        elif current_hand_step.active_intent is None:
            active_hand_sources[side] = None
        elif current_hand_step.intent is not None:
            if selected_hand is None:
                raise RuntimeError(f"{side} hand intent lost source provenance")
            active_hand_sources[side] = selected_hand
        if (
            current_hand_step is not None
            and current_hand_step.active_intent is not None
            and active_hand_sources[side] is None
        ):
            raise RuntimeError(f"{side} active hand intent has no source")
        trace_publisher.publish(
            teleoperation_tick_trace_to_message(
                _tick_trace(
                    run_id=run_id,
                    tick_id=tick_id,
                    side=side,
                    times=stage_times,
                    execution=execution,
                    pre_feedback=pre_feedback[side],
                    applied_target=applied_targets[side],
                    post_feedback=post_feedback[side],
                    arm_step=arm_steps[side],
                    hand_step=current_hand_step,
                    tracker_selection=tracker_inputs[side].selected,
                    active_tracker_source=active_tracker_sources[side],
                    hand_selection=(
                        hand_inputs[hand_side].selected if hand_side in hand_inputs else None
                    ),
                    active_hand_source=active_hand_sources[side],
                    scene=scene,
                )
            )
        )
    if scene_state_publisher is None:
        return
    scene_time_ns = time.monotonic_ns()
    for snapshot in scene.rigid_body_snapshots():
        scene_state_publisher.publish(
            scene_rigid_body_state_to_message(
                SceneRigidBodyState(
                    run_id=run_id,
                    tick_id=tick_id,
                    prim_path=snapshot.prim_path,
                    recorded_time_ns=scene_time_ns,
                    position_m=snapshot.position_m,
                    quat_wxyz=snapshot.quat_wxyz,
                    linear_velocity_m_s=snapshot.linear_velocity_m_s,
                    angular_velocity_deg_s=(snapshot.angular_velocity_deg_s),
                    kinematic_enabled=snapshot.kinematic_enabled,
                )
            )
        )


def _tick_trace(
    *,
    run_id: str,
    tick_id: int,
    side: str,
    times: TickStageTimes,
    execution: TickExecutionTrace,
    pre_feedback: NDArray[np.float64],
    applied_target: NDArray[np.float64],
    post_feedback: NDArray[np.float64],
    arm_step: TrackerArmSimulationStep,
    hand_step: Hand2SimulationStep | None,
    tracker_selection: RosTrackerSelection | None,
    active_tracker_source: SourceSelectionTrace | None,
    hand_selection: RosHandSelection | None,
    active_hand_source: SourceSelectionTrace | None,
    scene: DualNeroHand2IsaacScene,
) -> TeleoperationTickTrace:
    arm_route = RESOLVED.route_plan.route(
        f"nero_{side}",
        "arm_joints",
    )
    mapping = arm_step.mapping
    mapping_trace = None
    if mapping is not None:
        mapping_trace = ArmMappingTrace(
            target_position_m=mapping.target_position_m,
            target_orientation_wxyz=mapping.target_orientation_wxyz,
            tracker_delta_m=mapping.tracker_delta_m,
            workcell_delta_m=mapping.workcell_delta_m,
            tracker_delta_rotation_wxyz=(mapping.tracker_delta_rotation_wxyz),
            workcell_delta_rotation_wxyz=(mapping.workcell_delta_rotation_wxyz),
            rotation_delta_rad=mapping.rotation_delta_rad,
            input_host_time_ns=mapping.input_host_time_ns,
            accepted=mapping.accepted,
            translation_clamped=mapping.translation_clamped,
            rotation_clamped=mapping.rotation_clamped,
            requires_reference=mapping.requires_reference,
            reason=mapping.reason,
        )
    kinematics = arm_step.kinematics
    kinematics_trace = None
    if kinematics is not None:
        kinematics_trace = ArmKinematicsTrace(
            succeeded=kinematics.succeeded,
            solver_reported_success=kinematics.solver_reported_success,
            candidate_q7_rad=kinematics.candidate_q7_rad,
            position_residual_m=kinematics.position_residual_m,
            orientation_residual_rad=kinematics.orientation_residual_rad,
            reason=kinematics.reason,
        )
    arm = ArmControlTrace(
        source=_tracker_source_trace(tracker_selection),
        active_source=active_tracker_source,
        controller_state=arm_step.state.value,
        controller_reason=arm_step.reason,
        reference_epoch=arm_step.reference_epoch,
        reference_established=arm_step.reference_established,
        reference_revoked=arm_step.reference_revoked,
        mapping=mapping_trace,
        kinematics=kinematics_trace,
        decision=RouteDecisionTrace(
            instance_id=arm_route.instance_id,
            group_id=arm_route.group_id,
            layout_id=scene.arm_profiles[side].layout_id,
            command_rad=tuple(float(value) for value in arm_step.safety.command),
            safety_state=arm_step.safety.state.value,
            reason=arm_step.safety.reason,
            position_clamped=arm_step.safety.position_clamped,
            rate_limited=arm_step.safety.rate_limited,
        ),
    )
    hand = None
    if hand_step is not None:
        hand_route = RESOLVED.route_plan.route(
            f"hand_{side}",
            "finger_joints",
        )
        intent = hand_step.active_intent
        intent_trace = None
        if intent is not None:
            intent_trace = HandIntentTrace(
                sequence=intent.sequence,
                q20_rad=intent.q20_rad,
                layout_id=intent.layout_id,
                produced_time_ns=intent.produced_time_ns,
                retarget_status=intent.retarget_status.value,
                retarget_confidence=intent.retarget_confidence,
                retarget_model_id=intent.retarget_model_id,
                retarget_config_id=intent.retarget_config_id,
            )
        hand = HandControlTrace(
            source=_hand_source_trace(hand_selection),
            active_source=active_hand_source,
            intent=intent_trace,
            intent_is_new=hand_step.intent is not None,
            rejection_reason=hand_step.rejection_reason,
            decision=RouteDecisionTrace(
                instance_id=hand_route.instance_id,
                group_id=hand_route.group_id,
                layout_id=scene.hand_profiles[side].layout_id,
                command_rad=tuple(float(value) for value in hand_step.decision.command),
                safety_state=hand_step.decision.state.value,
                reason=hand_step.decision.reason,
                position_clamped=hand_step.decision.position_clamped,
                rate_limited=hand_step.decision.rate_limited,
            ),
        )
    return TeleoperationTickTrace(
        run_id=run_id,
        tick_id=tick_id,
        side=side,
        times=times,
        execution=execution,
        pre_feedback_q27_rad=tuple(float(value) for value in pre_feedback),
        applied_target_q27_rad=tuple(float(value) for value in applied_target),
        post_feedback_q27_rad=tuple(float(value) for value in post_feedback),
        arm=arm,
        hand=hand,
    )


def _tracker_source_trace(
    selection: RosTrackerSelection | None,
) -> SourceSelectionTrace | None:
    if selection is None:
        return None
    sample = selection.sample
    return SourceSelectionTrace(
        source_id=sample.stream_id,
        producer_instance=sample.producer_instance,
        transport_epoch=sample.transport_epoch,
        sequence=sample.sequence,
        source_time_ns=sample.host_time_ns,
        receive_time_ns=selection.callback_time_ns,
        callback_time_ns=selection.callback_time_ns,
    )


def _hand_source_trace(
    selection: RosHandSelection | None,
) -> SourceSelectionTrace | None:
    if selection is None:
        return None
    envelope = selection.envelope
    observation = envelope.observation
    return SourceSelectionTrace(
        source_id=observation.source_id,
        producer_instance=envelope.producer_instance,
        transport_epoch=envelope.transport_epoch,
        sequence=observation.sequence,
        source_time_ns=observation.source_time_ns,
        receive_time_ns=observation.receive_time_ns,
        callback_time_ns=selection.callback_time_ns,
    )


def _run_manifest_payload(
    *,
    scene: DualNeroHand2IsaacScene,
    recording_opened_ns: int,
    camera_capture: DualD405CameraCapture | None,
    camera_warmup: dict[str, object] | None,
) -> dict[str, object]:
    if camera_capture is None or camera_warmup is None:
        raise RuntimeError("recording manifest requires active D405 camera inventory")
    namespace = f"/{RESOLVED.deployment.root_namespace}"
    return {
        "state": "started",
        "scope": (
            "simulation-only dual NERO + Hand2; no UDP, CAN, NERO "
            "hardware, or Hand2 hardware commands"
        ),
        "recording_opened_monotonic_ns": recording_opened_ns,
        "clock_domain": "host_monotonic",
        "deployment": {
            "config_path": RESOLVED.config_path,
            "deployment_id": RESOLVED.deployment.deployment_id,
            "deployment_hash": RESOLVED.deployment_hash,
            "local_binding_hash": RESOLVED.local_binding_hash,
            "session_id": RESOLVED.session.session.session_id,
            "session_hash": RESOLVED.session.session_hash,
            "mapping_path": RESOLVED.mapping_path,
            "mapping_sha256": RESOLVED.mapping_sha256,
            "root_namespace": RESOLVED.deployment.root_namespace,
        },
        "software": {
            "python": sys.version,
            "platform": platform.platform(),
            **_git_state(),
        },
        "ros": {
            "domain_id": RESOLVED.local_binding.ros_domain_id,
            "rmw_implementation": (RESOLVED.local_binding.rmw_implementation),
            "qos": RESOLVED.qos_profile.to_mapping(),
        },
        "control": RESOLVED.control_profile.to_mapping(),
        "simulation_timing": {
            "physics_hz": RESOLVED.control_profile.physics_hz,
            "physics_dt_s": 1.0 / RESOLVED.control_profile.physics_hz,
            "control_hz": CONTROL_HZ,
            "control_dt_s": 1.0 / CONTROL_HZ,
            "rendering_hz": RENDER_HZ,
            "rendering_dt_s": 1.0 / RENDER_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS_PER_CONTROL,
            "control_ticks_per_render": CONTROL_TICKS_PER_RENDER,
            "scheduler": (
                "monotonic_fixed_rate_bounded_catch_up_v1"
                if ARGS.gui
                else "monotonic_fixed_rate_skip_missed_v1"
            ),
            "maximum_consecutive_catch_up_ticks": (GUI_MAXIMUM_CATCH_UP_TICKS if ARGS.gui else 0),
            "synthetic_camera_service_phase": (
                "inline_gui_render_v1" if ARGS.gui else CAMERA_CAPTURE_EXECUTION
            ),
            "executor": "background_single_threaded_spin_v1",
            "gui": ARGS.gui,
            "viewport_width": VIEWPORT_WIDTH,
            "viewport_height": VIEWPORT_HEIGHT,
            "anti_aliasing": 0,
            "renderer": ACTIVE_ISAAC_RENDERER,
            "minimal_shading_mode": ISAAC_MINIMAL_SHADING_MODE,
            "multi_gpu": False,
            "cpu_thread_limit": ISAAC_CPU_THREAD_LIMIT,
            "process_cpu_affinity": PROCESS_CPU_AFFINITY,
            "block_on_render": bool(scene.world.get_block_on_render()),
            "viewport_updates_enabled": ARGS.gui,
            "python_gc_policy": PYTHON_GC_POLICY,
        },
        "resolved_control_artifacts": {
            "qualification_path": str(QUALIFICATION_PATH.relative_to(ROOT)),
            "qualification_sha256": sha256_file(QUALIFICATION_PATH),
            "geometry_alignment_path": str(ALIGNMENT_PATH.relative_to(ROOT)),
            "geometry_alignment_sha256": sha256_file(ALIGNMENT_PATH),
            "lula_description_path": str(NERO_LULA_DESCRIPTION.relative_to(ROOT)),
            "lula_description_sha256": sha256_file(NERO_LULA_DESCRIPTION),
            "lula_urdf_path": str(NERO_LULA_URDF.relative_to(ROOT)),
            "lula_urdf_sha256": sha256_file(NERO_LULA_URDF),
            "self_collision_filter_path": str(SELF_COLLISION_FILTER_PATH.relative_to(ROOT)),
            "self_collision_filter_sha256": sha256_file(SELF_COLLISION_FILTER_PATH),
        },
        "recording_inventory": {
            "topics": list(
                recording_topics(
                    namespace,
                    RESOLVED.route_plan,
                    include_synthetic_d405=True,
                )
            ),
            "raw_inputs": (
                "Tracker SE3 and Glove canonical 21x3 landmarks remain in typed input topics"
            ),
            "per_tick": [
                "selected source sequence/epoch/callback time",
                "arm mapping and IK result",
                "hand q20 retarget intent",
                "q7/q20 safety decision",
                "atomic applied q27 target",
                "pre-apply and post-step q27 feedback",
                "raw stage timestamps",
                "control deadline, slot and missed-period count",
                "two physics substep indices, host times and simulation times",
                "target-effective simulation interval and render index",
                "Workcell dynamic rigid-body state",
                "dual completed-frame synthetic wrist-camera transactions",
            ],
        },
        "synthetic_d405_wrist_cameras": {
            "simulation_only": True,
            "warning": (
                "SIMULATION ONLY: synthetic 140-degree HFOV; not a physical "
                "RealSense D405 specification or calibration."
            ),
            "adapter": SIMULATION_CAMERA_CAPTURE_ADAPTER,
            "capture_execution": ("inline_gui_render_v1" if ARGS.gui else CAMERA_CAPTURE_EXECUTION),
            "capture_execution_warning": (
                "Headless recording replays exact post-physics simulation states only after "
                "the real-time control segment; it is not a live physical-camera model."
            ),
            "writer_callback_threading": "gpu_clone_host_copy_worker_v3",
            "completed_frame_join": {
                "method": "nearest_reference_time_to_post_substep_pose_history_v1",
                "rounding_tolerance_ns": POSE_HISTORY_JOIN_TOLERANCE_NS,
                "fail_closed_outside_tolerance": True,
            },
            "warmup": camera_warmup,
            "cameras": [inventory.to_mapping() for inventory in camera_capture.inventories],
            "tf_ownership": {
                "owner": "isaac_consumer",
                "dynamic_edges": [
                    "world->wujihand_left_hand_base",
                    "world->wujihand_right_hand_base",
                ],
                "static_edges": [
                    "wujihand_left_hand_base->wujihand_left_wrist_camera_optical",
                    "wujihand_right_hand_base->wujihand_right_wrist_camera_optical",
                ],
                "world_to_optical_direct_edge": False,
                "authoritative_dataset_join": "frame_truth",
            },
            "storage": {
                "image_compression": "none",
                "mcap_compression": "none",
                "raw_payload_estimate_decimal_mb_s": 129,
            },
        },
        "scene": {
            **scene.workcell_materialization.to_mapping(),
            "fixed_body_states": [
                {
                    "prim_path": snapshot.prim_path,
                    "position_m": list(snapshot.position_m),
                    "quat_wxyz": list(snapshot.quat_wxyz),
                    "mobility": "fixed",
                }
                for snapshot in scene.fixed_body_snapshots()
            ],
        },
        "q27_partitions": {
            side: {
                "arm_indices_q7": list(scene.partitions[side].arm_indices_q7),
                "hand_indices_q20": list(scene.partitions[side].hand_indices_q20),
                "arm_layout_id": scene.arm_profiles[side].layout_id,
                "hand_layout_id": scene.hand_profiles[side].layout_id,
            }
            for side in ("left", "right")
        },
        "capabilities": {
            "post_step_q27": True,
            "joint_velocity_feedback": False,
            "joint_effort_feedback": False,
            "dynamic_rigid_body_pose": True,
            "dynamic_rigid_body_velocity": "when_usd_attribute_available",
            "fixed_body_pose": "manifest",
            "raw_contact": False,
            "link7_palm_fingertip_pose": False,
            "task_truth": False,
            "rosbag_internal_queue_depth": False,
            "rosbag_internal_drop_counter": False,
            "executor_internal_queue_depth": False,
            "executor_internal_drop_counter": False,
            "latest_mailbox_superseded_counter": True,
            "control_schedule_missed_period_counter": True,
            "physics_substep_trace": True,
            "render_trace": True,
            "synthetic_d405_rgb_depth_camera_info_truth": True,
            "camera_completed_frame_identity": ("camera_sensor_writer_reference_time_v1"),
            "camera_pose_history_join": True,
            "sequence_and_join_gap_detection": "offline",
        },
        "privacy": {
            "raw_tracker_topics_contain_device_serial": True,
            "public_outputs_must_pseudonymize_device_identity": True,
        },
    }


def _run_receipt_payload(
    *,
    completed_frames: int,
    completed_physics_steps: int,
    completed_renders: int,
    started_ns: int,
    closed_ns: int,
    readiness: dict[str, object],
    counters: Counter[str],
    tracker_inputs: dict[str, RosTrackerInputAdapter],
    hand_inputs: dict[HandSide, RosHandObservationInputAdapter],
    executor_metrics: dict[str, object],
    python_gc_frozen_object_count: int,
    python_gc_unfrozen_on_close: bool,
    stop_signal: int | None,
    failure_reason: str | None,
    recording_failure_reason: str | None,
    camera_capture_receipt: dict[str, object] | None,
) -> dict[str, object]:
    return {
        "scope": "consumer_and_trace_producer_only",
        "completed_ticks": completed_frames,
        "completed_physics_steps": completed_physics_steps,
        "completed_renders": completed_renders,
        "configured_timing": {
            "physics_hz": RESOLVED.control_profile.physics_hz,
            "control_hz": CONTROL_HZ,
            "render_hz": RENDER_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS_PER_CONTROL,
            "control_ticks_per_render": CONTROL_TICKS_PER_RENDER,
            "maximum_consecutive_catch_up_ticks": (GUI_MAXIMUM_CATCH_UP_TICKS if ARGS.gui else 0),
            "synthetic_camera_service_phase": (
                "inline_gui_render_v1" if ARGS.gui else CAMERA_CAPTURE_EXECUTION
            ),
            "process_cpu_affinity": PROCESS_CPU_AFFINITY,
            "block_on_render": GUI_BLOCK_ON_RENDER,
            "python_gc_policy": PYTHON_GC_POLICY,
        },
        "python_gc": {
            "policy": PYTHON_GC_POLICY,
            "frozen_object_count": python_gc_frozen_object_count,
            "unfrozen_on_close": python_gc_unfrozen_on_close,
        },
        "control_started_monotonic_ns": started_ns,
        "closed_monotonic_ns": closed_ns,
        "stop_signal": stop_signal,
        "failure_reason": failure_reason,
        "recording_failure_reason": recording_failure_reason,
        "readiness": readiness,
        "controller_health": dict(counters),
        "input_health": {
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
        "executor": executor_metrics,
        "synthetic_d405_wrist_cameras": camera_capture_receipt,
        "quality_metrics_computed": False,
    }


def _git_state() -> dict[str, object]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()
    except (OSError, subprocess.CalledProcessError):
        return {"git_commit": None, "working_tree_state": "unknown"}
    return {
        "git_commit": commit,
        "working_tree_dirty": bool(status),
        "dirty_paths": [line[3:] for line in status],
    }


def _bounded_reason(exc: BaseException) -> str:
    value = f"{type(exc).__name__}:{str(exc)}".replace("\n", " ")
    return value[:128] or type(exc).__name__


try:
    exit_code = main()
except BaseException:
    traceback.print_exc()
    sys.stderr.flush()
    simulation_app.close(exit_code=1)
    raise
simulation_app.close(exit_code=exit_code)
raise SystemExit(exit_code)
