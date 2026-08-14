#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Run the ROS 2 Jazzy dual NERO + Hand 2 simulation consumer."""

from __future__ import annotations

import argparse
from collections import Counter
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import gc
import json
import math
from pathlib import Path
import platform
from queue import Full, Queue
import signal
import subprocess
import sys
from threading import Lock, Thread
import time
import traceback
from typing import Any, cast

import numpy as np
from numpy.typing import NDArray


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

from wujihand.application.qualification import (
    ROS_TELEOP_Q27_SETTLING_POLICY,
    joint_target_max_errors_rad,
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
from wujihand.dataset import (
    DatasetEpisodeLifecycle,
    EpisodeReadiness,
    Q54RuntimeInventory,
    load_mini_dataset_profile,
)
from wujihand.domain.dataset_recording import (
    DatasetEpisodeEvent,
    DatasetSourceMode,
    SimulationFramePhase,
    SimulationStateFrame,
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
    SceneDatasetStateSnapshot,
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
    load_nero_dual_simulation_startup_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.isaac_contact_tracking import (
    IsaacContactTracker,
    author_isaac_contact_reports,
)
from wujihand.adapters.simulation.nero_hand2_self_collision import (
    load_nero_hand2_self_collision_filter_profile,
)
from wujihand.runtime.wuji_hand2_record_chain import (
    load_record_chain_preflight_receipt,
    resolve_record_chain_workcell_plan,
)

DEFAULT_DEPLOYMENT = ROOT / (
    "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)
DEFAULT_LOCAL_BINDING = ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"
NERO_LULA_DESCRIPTION = ROOT / "configs/profiles/agilex_nero_lula_kinematics_v1.yaml"
OBLIQUE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
OBLIQUE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
SCREENSHOT_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"
GUI_MAXIMUM_CATCH_UP_TICKS = 2
GUI_BLOCK_ON_RENDER = False
VIEWPORT_WIDTH = 800
VIEWPORT_HEIGHT = 500
ISAAC_RENDERER = "MinimalRendering"
ISAAC_RECORDING_RENDERER = "RayTracedLighting"
ISAAC_MINIMAL_SHADING_MODE = 2
ISAAC_CPU_THREAD_LIMIT_CAP = 14
PYTHON_GC_POLICY = "collect_and_freeze_during_control_v1"
CAMERA_CAPTURE_EXECUTION = "paused_post_control_replay_v1"
OPERATOR_PREVIEW_STATE_TOPIC = "operator_preview/simulation_state"
OPERATOR_PREVIEW_MAX_KINEMATIC_LINKS = 128


@dataclass(frozen=True, slots=True)
class _CameraReplayState:
    control_tick_id: int
    physics_substep_index: int
    simulation_time_s: float
    scene: SceneReplaySnapshot


@dataclass(frozen=True, slots=True)
class _RecordingPublishBatch:
    tick_traces: tuple[TeleoperationTickTrace, ...]
    scene_states: tuple[SceneRigidBodyState, ...]
    dataset_states: tuple[SimulationStateFrame, ...]
    operator_preview_state: SimulationStateFrame | None


class _StageTimings:
    def __init__(self) -> None:
        self._samples: dict[str, list[int]] = {}

    def observe(self, counters: Counter[str], name: str, duration_ns: int) -> None:
        self._samples.setdefault(name, []).append(duration_ns)
        counters[f"{name}_total_ns"] += duration_ns
        counters[f"{name}_max_ns"] = max(counters[f"{name}_max_ns"], duration_ns)

    def to_mapping(self) -> dict[str, object]:
        return {
            name: {
                "samples": len(values),
                "mean_ms": sum(values) / len(values) / 1_000_000,
                "max_ms": max(values) / 1_000_000,
                "p95_ms": float(np.percentile(values, 95)) / 1_000_000,
                "p99_ms": float(np.percentile(values, 99)) / 1_000_000,
            }
            for name, values in sorted(self._samples.items())
            if values
        }


@dataclass(frozen=True, slots=True)
class _DeferredDatasetState:
    run_id: str
    control_index: int
    phase: SimulationFramePhase
    simulation_time_s: float
    physics_boundary_index: int
    snapshot: SceneDatasetStateSnapshot

    def materialize(self) -> SimulationStateFrame:
        return self.snapshot.to_frame(
            run_id=self.run_id,
            control_index=self.control_index,
            phase=self.phase,
            simulation_time_s=self.simulation_time_s,
            physics_boundary_index=self.physics_boundary_index,
        )

    def as_next_pre_action(
        self,
        *,
        control_index: int,
        simulation_time_s: float,
        physics_boundary_index: int,
        q54_rad: tuple[float, ...],
        qdot54_rad_s: tuple[float, ...],
    ) -> _DeferredDatasetState:
        if self.phase is not SimulationFramePhase.POST_ACTION:
            raise ValueError("only a post-action draft can close the next pre-action draft")
        if control_index != self.control_index + 1:
            raise ValueError("reused pre-action draft must be control-index adjacent")
        if simulation_time_s != self.simulation_time_s:
            raise ValueError("reused pre-action draft observed simulation-time advance")
        if physics_boundary_index != self.physics_boundary_index:
            raise ValueError("reused pre-action draft observed physics-boundary advance")
        if q54_rad != self.snapshot.q54_rad or qdot54_rad_s != self.snapshot.qdot54_rad_s:
            raise ValueError("reused pre-action draft differs from live q54 readback")
        return _DeferredDatasetState(
            run_id=self.run_id,
            control_index=control_index,
            phase=SimulationFramePhase.PRE_ACTION,
            simulation_time_s=simulation_time_s,
            physics_boundary_index=physics_boundary_index,
            snapshot=self.snapshot,
        )


class _SelfCollisionQ27Probe:
    def __init__(self, scene: DualNeroHand2IsaacScene) -> None:
        self.names: dict[str, tuple[str, ...]] = {}
        self.limits: dict[str, NDArray[np.float64]] = {}
        self.minimum: dict[str, NDArray[np.float64]] = {}
        self.maximum: dict[str, NDArray[np.float64]] = {}
        self.maximum_target_error: dict[str, NDArray[np.float64]] = {}
        self.maximum_target_error_rad = 0.0
        self.final_target_error_rad = 0.0
        self.nonfinite_samples = 0
        self.outside_limit_samples = 0
        self.samples = 0
        for side in ("left", "right"):
            names, limits = scene.runtime_joint_inventory(side)
            values = np.asarray(limits, dtype=np.float64)
            self.names[side] = names
            self.limits[side] = values
            self.minimum[side] = np.full(27, np.inf, dtype=np.float64)
            self.maximum[side] = np.full(27, -np.inf, dtype=np.float64)
            self.maximum_target_error[side] = np.zeros(27, dtype=np.float64)

    def observe(
        self,
        feedback: Mapping[str, NDArray[np.float64]],
        targets: Mapping[str, NDArray[np.float64]],
    ) -> None:
        self.final_target_error_rad = 0.0
        for side in ("left", "right"):
            values = feedback[side]
            if not np.isfinite(values).all():
                self.nonfinite_samples += 1
                continue
            self.minimum[side] = np.minimum(self.minimum[side], values)
            self.maximum[side] = np.maximum(self.maximum[side], values)
            target_error = np.abs(values - targets[side])
            self.maximum_target_error[side] = np.maximum(
                self.maximum_target_error[side], target_error
            )
            self.maximum_target_error_rad = max(
                self.maximum_target_error_rad,
                float(np.max(target_error)),
            )
            self.final_target_error_rad = max(
                self.final_target_error_rad,
                float(np.max(target_error)),
            )
            limits = self.limits[side]
            self.outside_limit_samples += int(
                np.any((values < limits[:, 0] - 0.01) | (values > limits[:, 1] + 0.01))
            )
        self.samples += 1

    def to_mapping(self, scene: DualNeroHand2IsaacScene) -> dict[str, object]:
        sides: dict[str, object] = {}
        for side in ("left", "right"):
            joint_range = self.maximum[side] - self.minimum[side]
            target_error = self.maximum_target_error[side]
            arm_indices = np.asarray(scene.partitions[side].arm_indices_q7, dtype=np.int64)
            hand_indices = np.asarray(scene.partitions[side].hand_indices_q20, dtype=np.int64)
            maximum_target_error_index = int(np.argmax(target_error))
            sides[side] = {
                "joint_names": list(self.names[side]),
                "minimum_feedback_rad": self.minimum[side].tolist(),
                "maximum_feedback_rad": self.maximum[side].tolist(),
                "maximum_arm_target_error_rad": float(np.max(target_error[arm_indices])),
                "maximum_hand_target_error_rad": float(np.max(target_error[hand_indices])),
                "maximum_target_error_joint_name": self.names[side][maximum_target_error_index],
                "maximum_arm_joint_range_rad": float(np.max(joint_range[arm_indices])),
                "maximum_hand_joint_range_rad": float(np.max(joint_range[hand_indices])),
                "maximum_abs_feedback_rad": float(
                    np.max(np.abs(np.concatenate((self.minimum[side], self.maximum[side]))))
                ),
            }
        return {
            "samples": self.samples,
            "nonfinite_samples": self.nonfinite_samples,
            "outside_limit_samples": self.outside_limit_samples,
            "maximum_target_error_rad": self.maximum_target_error_rad,
            "final_target_error_rad": self.final_target_error_rad,
            "sides": sides,
        }


def _self_collision_policy_mapping() -> dict[str, object]:
    profile = None
    if SELF_COLLISION_FILTER_PROFILE is not None:
        profile = {
            "path": RESOLVED.self_collision_profile_path,
            "profile_id": RESOLVED.self_collision_profile_id,
            "sha256": RESOLVED.self_collision_profile_sha256,
        }
    return {
        "enabled": profile is not None,
        "mode": ("merged_q27_filtered_pairs_v2" if profile is not None else "merged_q27_disabled"),
        "profile": profile,
        "qualification_probe_enabled": ARGS.self_collision_qualification,
    }


def _self_collision_qualification_mapping(
    *,
    scene: DualNeroHand2IsaacScene,
    tracker: IsaacContactTracker,
    probe: _SelfCollisionQ27Probe,
    readback: Mapping[str, bool],
    contact_api_paths: tuple[str, ...],
    completed_frames: int,
    counters: Counter[str],
) -> dict[str, object]:
    contacts = tracker.to_mapping()
    hand_contact_pairs: dict[str, list[str]] = {"left": [], "right": []}
    maximum_hand_penetration_m = 0.0
    for pair_name, raw in contacts.items():
        record = cast(Mapping[str, object], raw)
        paths = cast(list[str], record["paths"])
        for side in ("left", "right"):
            prefix = f"/World/Robots/Hand2{side.capitalize()}/"
            if all(path.startswith(prefix) for path in paths):
                contact_frames = sum(
                    cast(Mapping[str, int], record["phase_contact_frames"]).values()
                )
                if contact_frames:
                    hand_contact_pairs[side].append(pair_name)
                    separation = record["minimum_separation_m"]
                    if separation is not None:
                        maximum_hand_penetration_m = max(
                            maximum_hand_penetration_m,
                            max(0.0, -float(separation)),
                        )
    probe_result = probe.to_mapping(scene)
    side_metrics = cast(Mapping[str, Mapping[str, object]], probe_result["sides"])
    expected_filtered_pairs = sum(
        side in {"left", "right"}
        for rule in cast(Any, SELF_COLLISION_FILTER_PROFILE).filtered_pairs
        for side in rule.sides
    )
    checks = {
        "self_collision_readback_enabled_both": readback == {"left": True, "right": True},
        "filtered_pairs_match_profile": (
            len(scene.self_collision_filtered_pairs) == expected_filtered_pairs
        ),
        "contact_reporting_enabled": bool(contact_api_paths),
        "both_hands_reach_internal_contact": all(hand_contact_pairs.values()),
        "hand_contact_penetration_bounded": maximum_hand_penetration_m <= 0.002,
        "q27_samples_cover_control_ticks": probe_result["samples"] == completed_frames,
        "q27_feedback_finite": probe_result["nonfinite_samples"] == 0,
        "q27_feedback_stays_inside_limits": probe_result["outside_limit_samples"] == 0,
        "q27_final_target_error_bounded": float(probe_result["final_target_error_rad"]) <= 0.15,
        "both_arms_move": all(
            float(side_metrics[side]["maximum_arm_joint_range_rad"]) > 0.005
            for side in ("left", "right")
        ),
        "both_hands_move": all(
            float(side_metrics[side]["maximum_hand_joint_range_rad"]) > 0.05
            for side in ("left", "right")
        ),
        "control_has_zero_missed_periods": counters["scheduler.missed_control_periods"] == 0,
    }
    return {
        "passed": all(checks.values()),
        "filter_profile": {
            "path": RESOLVED.self_collision_profile_path,
            "sha256": RESOLVED.self_collision_profile_sha256,
            "profile_id": RESOLVED.self_collision_profile_id,
        },
        "readback": dict(readback),
        "authored_filtered_pairs": [
            {
                "pair_id": pair_id,
                "first_rigid_body_path": first,
                "second_rigid_body_path": second,
            }
            for pair_id, first, second in scene.self_collision_filtered_pairs
        ],
        "contact_report_api_count": len(contact_api_paths),
        "contacts": contacts,
        "hand_contact_pairs": hand_contact_pairs,
        "maximum_hand_penetration_m": maximum_hand_penetration_m,
        "q27": probe_result,
        "checks": checks,
    }


@dataclass(frozen=True, slots=True)
class _RecordingPublishContext:
    """Tick-owned immutable references needed to assemble recording facts."""

    run_id: str
    tick_id: int
    stage_times: TickStageTimes
    execution: TickExecutionTrace
    pre_feedback: tuple[tuple[str, NDArray[np.float64]], ...]
    applied_targets: tuple[tuple[str, NDArray[np.float64]], ...]
    post_feedback: tuple[tuple[str, NDArray[np.float64]], ...]
    arm_steps: tuple[tuple[str, TrackerArmSimulationStep], ...]
    hand_steps: tuple[tuple[str, Hand2SimulationStep], ...]
    tracker_selections: tuple[tuple[str, RosTrackerSelection | None], ...]
    hand_selections: tuple[tuple[str, RosHandSelection | None], ...]
    active_tracker_sources: tuple[tuple[str, SourceSelectionTrace | None], ...]
    active_hand_sources: tuple[tuple[str, SourceSelectionTrace | None], ...]
    arm_layout_ids: tuple[tuple[str, str], ...]
    hand_layout_ids: tuple[tuple[str, str], ...]
    scene_states: tuple[SceneRigidBodyState, ...]
    dataset_pre_state: SimulationStateFrame | _DeferredDatasetState | None
    dataset_post_state: SimulationStateFrame | _DeferredDatasetState | None
    operator_preview_state: SimulationStateFrame | _DeferredDatasetState | None


class _RecordingPublisherWorker:
    """Publish immutable recording batches outside the control thread."""

    _STOP = object()

    def __init__(
        self,
        *,
        trace_publisher: Any,
        scene_state_publisher: Any | None,
        dataset_state_publisher: Any | None,
        operator_preview_state_publisher: Any | None,
    ) -> None:
        self._trace_publisher = trace_publisher
        self._scene_state_publisher = scene_state_publisher
        self._dataset_state_publisher = dataset_state_publisher
        self._operator_preview_state_publisher = operator_preview_state_publisher
        self._queue: Queue[_RecordingPublishContext | object] = Queue(maxsize=4)
        self._lock = Lock()
        self._failure: BaseException | None = None
        self._thread = Thread(
            target=self._run,
            name="wujihand-recording-publisher",
            daemon=False,
        )
        self._started = False
        self._closed = False
        self._enqueued_batches = 0
        self._published_batches = 0
        self._maximum_queue_depth = 0
        self._batch_build_total_ns = 0
        self._batch_build_max_ns = 0
        self._tick_publish_total_ns = 0
        self._tick_publish_max_ns = 0
        self._dataset_publish_total_ns = 0
        self._dataset_publish_max_ns = 0
        self._dataset_state_frames_published = 0
        self._operator_preview_states_published = 0

    def start(self) -> None:
        if self._started:
            raise RuntimeError("recording publisher worker already started")
        self._started = True
        self._thread.start()

    def submit(self, context: _RecordingPublishContext) -> None:
        if not self._started or self._closed:
            raise RuntimeError("recording publisher worker is not active")
        self.raise_if_failed()
        try:
            self._queue.put_nowait(context)
        except Full as exc:
            raise RuntimeError("recording publisher queue exceeded four control ticks") from exc
        self._enqueued_batches += 1
        self._maximum_queue_depth = max(self._maximum_queue_depth, self._queue.qsize())

    def raise_if_failed(self) -> None:
        with self._lock:
            failure = self._failure
        if failure is not None:
            raise RuntimeError("recording publisher worker failed") from failure

    def close(self) -> dict[str, int]:
        if not self._started:
            raise RuntimeError("recording publisher worker was not started")
        if not self._closed:
            self._closed = True
            try:
                self._queue.put(self._STOP, timeout=2.0)
            except Full as exc:
                raise RuntimeError("recording publisher queue did not drain") from exc
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                raise RuntimeError("recording publisher worker did not stop")
        self.raise_if_failed()
        if self._published_batches != self._enqueued_batches:
            raise RuntimeError("recording publisher batch accounting did not close")
        return {
            "recording.publisher_batches": self._published_batches,
            "recording.publisher_queue_max_depth": self._maximum_queue_depth,
            "recording.batch_build_total_ns": self._batch_build_total_ns,
            "recording.batch_build_max_ns": self._batch_build_max_ns,
            "recording.tick_publish_total_ns": self._tick_publish_total_ns,
            "recording.tick_publish_max_ns": self._tick_publish_max_ns,
            "dataset.publish_total_ns": self._dataset_publish_total_ns,
            "dataset.publish_max_ns": self._dataset_publish_max_ns,
            "dataset.state_frames_published": self._dataset_state_frames_published,
            "operator_preview.state_frames_published": (self._operator_preview_states_published),
        }

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                if not isinstance(item, _RecordingPublishContext):
                    raise RuntimeError("recording publisher received an invalid context")
                with self._lock:
                    failed = self._failure is not None
                if failed:
                    continue
                build_started_ns = time.monotonic_ns()
                batch = _build_recording_publish_batch(item)
                build_duration_ns = time.monotonic_ns() - build_started_ns
                self._batch_build_total_ns += build_duration_ns
                self._batch_build_max_ns = max(
                    self._batch_build_max_ns,
                    build_duration_ns,
                )
                self._publish(batch)
            except BaseException as exc:
                with self._lock:
                    if self._failure is None:
                        self._failure = exc
            finally:
                self._queue.task_done()

    def _publish(self, batch: _RecordingPublishBatch) -> None:
        tick_started_ns = time.monotonic_ns()
        for trace in batch.tick_traces:
            self._trace_publisher.publish(teleoperation_tick_trace_to_message(trace))
        if batch.scene_states:
            if self._scene_state_publisher is None:
                raise RuntimeError("recording scene-state publisher is missing")
            for scene_state in batch.scene_states:
                self._scene_state_publisher.publish(scene_rigid_body_state_to_message(scene_state))
        tick_duration_ns = time.monotonic_ns() - tick_started_ns
        self._tick_publish_total_ns += tick_duration_ns
        self._tick_publish_max_ns = max(self._tick_publish_max_ns, tick_duration_ns)

        if batch.dataset_states:
            if self._dataset_state_publisher is None:
                raise RuntimeError("recording dataset-state publisher is missing")
            dataset_started_ns = time.monotonic_ns()
            for dataset_state in batch.dataset_states:
                self._dataset_state_publisher.publish(
                    simulation_state_frame_to_message(dataset_state)
                )
            dataset_duration_ns = time.monotonic_ns() - dataset_started_ns
            self._dataset_publish_total_ns += dataset_duration_ns
            self._dataset_publish_max_ns = max(
                self._dataset_publish_max_ns,
                dataset_duration_ns,
            )
            self._dataset_state_frames_published += len(batch.dataset_states)
        if batch.operator_preview_state is not None:
            if self._operator_preview_state_publisher is None:
                raise RuntimeError("operator-preview state publisher is missing")
            if (
                len(batch.operator_preview_state.kinematic_links)
                > OPERATOR_PREVIEW_MAX_KINEMATIC_LINKS
            ):
                raise RuntimeError(
                    "operator-preview kinematic-link count exceeds transport bound: "
                    f"{len(batch.operator_preview_state.kinematic_links)} > "
                    f"{OPERATOR_PREVIEW_MAX_KINEMATIC_LINKS}"
                )
            self._operator_preview_state_publisher.publish(
                simulation_state_frame_to_message(
                    batch.operator_preview_state,
                    factory=OperatorPreviewStateFrameMessage,
                )
            )
            self._operator_preview_states_published += 1
        self._published_batches += 1


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
        help="Bounded 30 Hz control ticks; zero runs until the app closes.",
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
    parser.add_argument(
        "--external-preview-required",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--dataset-source-mode",
        choices=tuple(item.value for item in DatasetSourceMode),
        default=DatasetSourceMode.LIVE_TELEOPERATION.value,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--run-id")
    parser.add_argument("--run-root", type=Path)
    parser.add_argument(
        "--chain-preflight",
        type=Path,
        help="Passed Hand2 8.3 record-chain preflight receipt.",
    )
    parser.add_argument(
        "--self-collision-qualification",
        action="store_true",
        help="Enable bounded contact and q27 probes for a Session self-collision policy.",
    )
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args = parser.parse_args()
    if args.frames < 0:
        parser.error("--frames must be non-negative")
    unbounded_external_preview_recording = (
        not args.gui
        and args.frames == 0
        and args.recording_enabled
        and args.external_preview_required
    )
    if not args.gui and args.frames == 0 and not unbounded_external_preview_recording:
        parser.error("--no-gui requires a positive --frames bound")
    if args.recording_enabled and (args.run_id is None or args.run_root is None):
        parser.error("--recording-enabled requires --run-id and --run-root")
    if args.recording_enabled and args.report is not None:
        parser.error("--report cannot be combined with recording mode")
    if not args.recording_enabled and (args.run_id is not None or args.run_root is not None):
        parser.error("--run-id/--run-root require --recording-enabled")
    if args.self_collision_qualification and (args.recording_enabled or args.frames == 0):
        parser.error("self-collision qualification requires bounded non-recording mode")
    return args


ARGS = parse_args()
try:
    PROCESS_CPU_AFFINITY = configure_current_process_cpu_affinity(ARGS.cpu_affinity)
except (RuntimeError, ValueError) as exc:
    raise SystemExit(f"NV-5 ROS CPU affinity preflight failed: {exc}") from exc
# SimulationApp does not derive its tasking pool from sched_setaffinity. Keep
# the tasking pool inside the verified process cpuset.
ISAAC_CPU_THREAD_LIMIT = min(
    ISAAC_CPU_THREAD_LIMIT_CAP,
    (len(PROCESS_CPU_AFFINITY) if PROCESS_CPU_AFFINITY is not None else ISAAC_CPU_THREAD_LIMIT_CAP),
)
try:
    RESOLVED = RosDeploymentResolver(ROOT).resolve(
        ARGS.deployment,
        local_binding=ARGS.local_runtime_binding,
        verify_artifacts=ARGS.verify_artifacts,
    )
except (FileNotFoundError, ValueError) as exc:
    raise SystemExit(f"NV-5 ROS deployment preflight failed: {exc}") from exc

SELF_COLLISION_FILTER_PROFILE = None
if RESOLVED.self_collision_profile_path is not None:
    self_collision_profile_path = ROOT / RESOLVED.self_collision_profile_path
    SELF_COLLISION_FILTER_PROFILE = load_nero_hand2_self_collision_filter_profile(
        self_collision_profile_path
    )
    if SELF_COLLISION_FILTER_PROFILE.profile_id != RESOLVED.self_collision_profile_id:
        raise SystemExit("resolved self-collision profile identity differs")
    if sha256_file(self_collision_profile_path) != RESOLVED.self_collision_profile_sha256:
        raise SystemExit("resolved self-collision profile content differs")
if ARGS.self_collision_qualification and SELF_COLLISION_FILTER_PROFILE is None:
    raise SystemExit("self-collision qualification requires a Session runtime profile")

if RESOLVED.deployment.execution_owner_process_id != "isaac_consumer":
    raise SystemExit("NV-5 requires isaac_consumer as the unique owner")
if RESOLVED.session.session.backend != "isaac":
    raise SystemExit("NV-5 ROS consumer requires an Isaac Session")
if RESOLVED.control_profile.physics_hz != 120:
    raise SystemExit("NV-5.1 requires exactly 120 Hz physics")
HAND_REVISIONS = {
    instance.asset.revision
    for instance in RESOLVED.session.instances
    if instance.asset.product == "wuji_hand_2"
}
REQUIRES_MATCHED_CHAIN = HAND_REVISIONS == {"beta1_description_v2026_8_3"}
if REQUIRES_MATCHED_CHAIN != (ARGS.chain_preflight is not None):
    raise SystemExit("the formal Description 8.3 entry requires --chain-preflight")
CHAIN_PREFLIGHT: Mapping[str, object] | None = None
WORKCELL_PLAN = None
if ARGS.chain_preflight is not None:
    try:
        CHAIN_PREFLIGHT = load_record_chain_preflight_receipt(ARGS.chain_preflight)
        WORKCELL_PLAN = resolve_record_chain_workcell_plan(
            ROOT,
            RESOLVED,
            CHAIN_PREFLIGHT,
            verify_content=ARGS.verify_artifacts,
        )
    except (FileNotFoundError, ValueError) as exc:
        raise SystemExit(f"record-chain runtime preflight failed: {exc}") from exc

dataset_profile_ref = RESOLVED.session.session.dataset_profile
if dataset_profile_ref is None:
    raise SystemExit("the formal ROS consumer requires a dataset timing profile")
DATASET_PROFILE = load_mini_dataset_profile(ROOT, dataset_profile_ref.path)
if DATASET_PROFILE.profile_id != dataset_profile_ref.expected_id:
    raise SystemExit("resolved dataset timing profile identity differs")
DATASET_MODE = True
CONTROL_HZ = DATASET_PROFILE.control_hz
RENDER_HZ = DATASET_PROFILE.gui_preview_hz
CONTROL_TICKS_PER_CAPTURE = CONTROL_HZ // DATASET_PROFILE.policy_fps
OPERATOR_PREVIEW_CONTROL_INTERVAL = CONTROL_HZ // RENDER_HZ
DATASET_RECORDING = DATASET_MODE and ARGS.recording_enabled
DATASET_SOURCE_MODE = DatasetSourceMode(ARGS.dataset_source_mode)
DATASET_ELIGIBLE = DATASET_SOURCE_MODE is DatasetSourceMode.LIVE_TELEOPERATION
if ARGS.external_preview_required and (not DATASET_RECORDING or ARGS.gui):
    raise SystemExit("external dataset preview requires headless dataset recording control")
if DATASET_SOURCE_MODE is DatasetSourceMode.SYNTHETIC_FIXTURE:
    if not DATASET_RECORDING or not ARGS.external_preview_required or ARGS.frames <= 0:
        raise SystemExit(
            "synthetic fixture requires bounded headless dataset recording with external preview"
        )
elif DATASET_SOURCE_MODE not in {
    DatasetSourceMode.LIVE_TELEOPERATION,
    DatasetSourceMode.LIVE_QUALIFICATION,
}:
    raise SystemExit(
        "this runner accepts only live teleoperation, live qualification, "
        "or synthetic fixture input"
    )
if REQUIRES_MATCHED_CHAIN:
    assert CHAIN_PREFLIGHT is not None
    preflight_input = CHAIN_PREFLIGHT.get("input_mode")
    expected_input = (
        "stub" if DATASET_SOURCE_MODE is DatasetSourceMode.SYNTHETIC_FIXTURE else "glove"
    )
    if preflight_input != expected_input:
        raise SystemExit("record-chain preflight input mode differs from recording mode")
    preflight_dataset = CHAIN_PREFLIGHT.get("dataset")
    expected_eligible = DATASET_SOURCE_MODE is DatasetSourceMode.LIVE_TELEOPERATION
    if (
        not isinstance(preflight_dataset, Mapping)
        or preflight_dataset.get("source_mode") != DATASET_SOURCE_MODE.value
        or preflight_dataset.get("qualification_only") is not (not expected_eligible)
        or preflight_dataset.get("dataset_eligible") is not expected_eligible
    ):
        raise SystemExit("record-chain preflight dataset policy differs from runtime")
CONTROL_MAXIMUM_CATCH_UP_TICKS = (
    GUI_MAXIMUM_CATCH_UP_TICKS if ARGS.gui or ARGS.external_preview_required else 0
)
# 008 renders all three RGB data views offline from pre-action state.  The
# online 007 RGB+depth capture remains available only to legacy non-dataset runs.
ACTIVE_ISAAC_RENDERER = (
    ISAAC_RENDERER
    if DATASET_MODE
    else (ISAAC_RECORDING_RENDERER if ARGS.recording_enabled else ISAAC_RENDERER)
)

PHYSICS_SUBSTEPS_PER_CONTROL = RESOLVED.control_profile.physics_hz // CONTROL_HZ
CONTROL_TICKS_PER_RENDER = CONTROL_HZ // RENDER_HZ
if (
    RESOLVED.control_profile.physics_hz % CONTROL_HZ != 0
    or CONTROL_HZ % RENDER_HZ != 0
    or CONTROL_HZ % DATASET_PROFILE.policy_fps != 0
    or PHYSICS_SUBSTEPS_PER_CONTROL != 4
    or CONTROL_TICKS_PER_RENDER != 2
    or CONTROL_TICKS_PER_CAPTURE != 1
):
    raise SystemExit("NV-5.1 requires 120/30/15/30 scheduling")

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
QUALIFICATION = load_nero_dual_simulation_startup_profile(QUALIFICATION_PATH)
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
import omni.physx  # type: ignore[import-not-found]
import isaacsim.core.experimental.utils.app as app_utils  # type: ignore[import-not-found]
from pxr import PhysxSchema  # type: ignore[import-not-found]
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
    DatasetEpisodeBoundary as DatasetEpisodeBoundaryMessage,
    HandObservationEnvelope,
    OperatorPreviewStateFrame as OperatorPreviewStateFrameMessage,
    RunRecordingStatus as RunRecordingStatusMessage,
    RouteCommand,
    SafetyEvent,
    SceneRigidBodyState as SceneRigidBodyStateMessage,
    SimulationCameraFrameTruth,
    SimulationStateFrame as SimulationStateFrameMessage,
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
    dataset_episode_boundary_to_message,
    route_command_from_decision,
    route_command_to_message,
    run_recording_status_to_message,
    safety_event_to_message,
    simulation_camera_frame_to_messages,
    simulation_state_frame_to_message,
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
    policy = ROS_TELEOP_Q27_SETTLING_POLICY
    if QUALIFICATION.teleport_to_initial_position:
        scene.teleport_to_targets()
    scene.apply_targets()
    previous: dict[str, list[float]] | None = None
    deltas: list[float] = []
    target_errors_rad: dict[str, float] = {}
    completed_physics_steps = 0
    physics_steps_per_render = PHYSICS_SUBSTEPS_PER_CONTROL * CONTROL_TICKS_PER_RENDER
    for window in range(1, policy.maximum_windows + 1):
        for _ in range(policy.window_frames):
            scene.world.step(render=False)
            completed_physics_steps += 1
            if ARGS.gui and completed_physics_steps % physics_steps_per_render == 0:
                scene.world.render()
        current = {side: scene.feedback_q27(side).tolist() for side in ("left", "right")}
        target_errors_rad = joint_target_max_errors_rad(
            {
                side: np.asarray(current[side], dtype=np.float64)[
                    np.asarray(
                        scene.partitions[side].arm_indices_q7,
                        dtype=np.int64,
                    )
                ]
                for side in ("left", "right")
            },
            scene.initial_arm_targets,
        )
        if previous is not None:
            delta = q27_window_max_delta_rad(previous, current)
            deltas.append(delta)
            if (
                window >= policy.minimum_windows
                and delta <= policy.max_window_delta_rad
                and max(target_errors_rad.values()) <= QUALIFICATION.initial_q7_max_error_rad
            ):
                return {
                    "converged": True,
                    "policy_id": policy.policy_id,
                    "windows": window,
                    "physics_steps": completed_physics_steps,
                    "final_max_delta_rad": delta,
                    "arm_target_errors_rad": target_errors_rad,
                    "arm_target_error_limit_rad": (QUALIFICATION.initial_q7_max_error_rad),
                }
        previous = current
    raise RuntimeError(
        "NV-5 ROS teleoperation scene readiness did not converge: "
        f"windows={policy.maximum_windows}, "
        f"final_max_delta_rad={deltas[-1]:.9f}, "
        f"window_limit_rad={policy.max_window_delta_rad:.9f}, "
        f"arm_target_errors_rad={target_errors_rad}, "
        "arm_target_error_limit_rad="
        f"{QUALIFICATION.initial_q7_max_error_rad:.9f}"
    )


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
    external_preview_state_topic: str | None = None,
    timeout_s: float = 180.0,
) -> None:
    """Do not begin a recorded control run before rosbag discovery closes."""

    deadline = time.monotonic() + timeout_s
    while True:
        minimums = {
            topic: (
                2
                if "/input/" in topic
                or (
                    external_preview_state_topic is not None and topic.endswith("/recording/status")
                )
                else 1
            )
            for topic in topics
        }
        if external_preview_state_topic is not None:
            minimums[external_preview_state_topic] = 1
        pending = {
            topic: minimum
            for topic, minimum in minimums.items()
            if node.count_subscribers(topic) < minimum
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
    """Service rendering without charging it to a control tick."""

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
    render_updates = priming["render_updates"]
    if isinstance(render_updates, bool) or not isinstance(render_updates, int):
        raise RuntimeError("D405 replay priming update count is invalid")
    counters["camera.replay_priming_updates"] += render_updates
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
                physics_substep_ordinal=PHYSICS_SUBSTEPS_PER_CONTROL - 1,
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
    self_collision_sides = (
        frozenset({"left", "right"}) if SELF_COLLISION_FILTER_PROFILE is not None else frozenset()
    )
    scene = DualNeroHand2IsaacScene(
        project_root=ROOT,
        resolved=RESOLVED.session,
        sides=SIDES,
        alignment_profile=ALIGNMENT,
        qualification_profile=QUALIFICATION,
        physics_hz=RESOLVED.control_profile.physics_hz,
        self_collision_sides=self_collision_sides,
        self_collision_filter_profile=SELF_COLLISION_FILTER_PROFILE,
        wrist_rig_collision_mode="all",
        workcell_plan=WORKCELL_PLAN,
    )
    contact_api_paths = (
        author_isaac_contact_reports(
            scene.stage,
            prim_path_prefix="/World/Robots",
            threshold_n=0.0,
        )
        if ARGS.self_collision_qualification
        else ()
    )
    self_collision_readback = {
        side: bool(
            PhysxSchema.PhysxArticulationAPI(
                scene.stage.GetPrimAtPath(scene.authored[side].articulation_root_path)
            )
            .GetEnabledSelfCollisionsAttr()
            .Get()
        )
        for side in ("left", "right")
    }
    if self_collision_readback != {
        side: side in self_collision_sides for side in ("left", "right")
    }:
        raise RuntimeError("merged-q27 self-collision readback differs from request")
    scene.world.set_block_on_render(GUI_BLOCK_ON_RENDER)
    if bool(scene.world.get_block_on_render()) is not GUI_BLOCK_ON_RENDER:
        raise RuntimeError("Isaac render blocking policy was not applied")
    readiness = _settle(scene)
    contact_tracker = (
        IsaacContactTracker(separation_epsilon_m=0.00005)
        if ARGS.self_collision_qualification
        else None
    )
    contact_subscription = (
        omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(
            contact_tracker.callback
        )
        if contact_tracker is not None
        else None
    )
    self_collision_q27_probe = (
        _SelfCollisionQ27Probe(scene) if ARGS.self_collision_qualification else None
    )
    q54_runtime_inventory: Q54RuntimeInventory | None = None
    if DATASET_PROFILE is not None:
        left_names, left_limits = scene.runtime_joint_inventory("left")
        right_names, right_limits = scene.runtime_joint_inventory("right")
        q54_runtime_inventory = DATASET_PROFILE.q54.validate_runtime_inventory(
            left_names=left_names,
            left_limits_rad=left_limits,
            right_names=right_names,
            right_limits_rad=right_limits,
        )
    camera_capture: DualD405CameraCapture | None = None
    camera_warmup: dict[str, object] | None = None
    if ARGS.recording_enabled and not DATASET_MODE:
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
    dataset_boundary_publisher = None
    dataset_state_publisher = None
    operator_preview_state_publisher = None
    camera_publishers: dict[str, dict[str, Any]] = {}
    camera_inventories: dict[str, SimulationCameraStaticInventory] = {}
    camera_transform_broadcaster: TransformBroadcaster | None = None
    camera_static_transform_broadcaster: StaticTransformBroadcaster | None = None
    counters: Counter[str] = Counter()
    stage_timings = _StageTimings()
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
        if DATASET_MODE:
            dataset_boundary_publisher = node.create_publisher(
                DatasetEpisodeBoundaryMessage,
                "dataset/episode_boundary",
                qos_profile(RESOLVED.qos_profile.policy("dataset_boundary")),
            )
            dataset_state_publisher = node.create_publisher(
                SimulationStateFrameMessage,
                "dataset/simulation_state",
                qos_profile(RESOLVED.qos_profile.policy("dataset_state")),
            )
            if ARGS.external_preview_required:
                operator_preview_state_publisher = node.create_publisher(
                    OperatorPreviewStateFrameMessage,
                    OPERATOR_PREVIEW_STATE_TOPIC,
                    qos_profile(RESOLVED.qos_profile.policy("dataset_state")),
                )
        else:
            if camera_capture is None:
                raise RuntimeError("recording mode did not create dual D405 capture")
            camera_inventories = {
                inventory.side: inventory for inventory in camera_capture.inventories
            }
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
    recording_publisher_worker = (
        None
        if trace_publisher is None
        else _RecordingPublisherWorker(
            trace_publisher=trace_publisher,
            scene_state_publisher=scene_state_publisher,
            dataset_state_publisher=dataset_state_publisher,
            operator_preview_state_publisher=operator_preview_state_publisher,
        )
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
    dataset_lifecycle: DatasetEpisodeLifecycle | None = None
    if DATASET_RECORDING:
        assert current_run_id is not None
        assert DATASET_PROFILE is not None
        dataset_lifecycle = DatasetEpisodeLifecycle(
            run_id=current_run_id,
            collection_id=DATASET_PROFILE.profile_id,
            source_mode=DATASET_SOURCE_MODE,
            dataset_eligible=DATASET_ELIGIBLE,
        )
    if current_run_root is not None and current_run_id is not None:
        write_manifest(
            current_run_root,
            run_id=current_run_id,
            payload=_run_manifest_payload(
                scene=scene,
                recording_opened_ns=recording_opened_ns,
                camera_capture=camera_capture,
                camera_warmup=camera_warmup,
                q54_runtime_inventory=q54_runtime_inventory,
            ),
        )
        _wait_for_recording_graph(
            node,
            recording_topics(
                f"/{RESOLVED.deployment.root_namespace}",
                RESOLVED.route_plan,
                include_synthetic_d405=not DATASET_MODE,
                include_dataset_facts=DATASET_MODE,
            ),
            external_preview_state_topic=(
                f"/{RESOLVED.deployment.root_namespace}/{OPERATOR_PREVIEW_STATE_TOPIC}"
                if ARGS.external_preview_required
                else None
            ),
        )
        if camera_capture is not None:
            camera_capture.activate(simulation_time_s=_simulation_time_s(scene))
        if dataset_lifecycle is not None:
            if dataset_boundary_publisher is None:
                raise RuntimeError("dataset boundary publisher is missing")
            dataset_boundary_publisher.publish(
                dataset_episode_boundary_to_message(
                    dataset_lifecycle.opened(host_time_ns=time.monotonic_ns())
                )
            )
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
    effective_dataset_stop_signal: int | None = None
    previous_post_dataset_state: _DeferredDatasetState | None = None
    try:
        gc.collect()
        gc.freeze()
        python_gc_frozen = True
        python_gc_frozen_object_count = gc.get_freeze_count()
        executor_worker.start()
        if recording_publisher_worker is not None:
            recording_publisher_worker.start()
        scheduler = FixedRateScheduler(
            rate_hz=CONTROL_HZ,
            start_ns=time.monotonic_ns(),
            maximum_catch_up_ticks=CONTROL_MAXIMUM_CATCH_UP_TICKS,
        )
        while (
            (DATASET_RECORDING or not stop_request.requested)
            and simulation_app.is_running()
            and (ARGS.frames == 0 or completed_frames < ARGS.frames)
        ):
            executor_worker.raise_if_failed()
            if recording_publisher_worker is not None:
                recording_publisher_worker.raise_if_failed()
            if ARGS.external_preview_required:
                namespace = f"/{RESOLVED.deployment.root_namespace}"
                preview_requirements = (
                    (f"{namespace}/{OPERATOR_PREVIEW_STATE_TOPIC}", 1),
                    (f"{namespace}/recording/status", 2),
                )
                if any(
                    node.count_subscribers(topic) < minimum
                    for topic, minimum in preview_requirements
                ):
                    failure_reason = "external_dataset_preview_subscriber_lost"
                    break
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
            for side, tracker_snapshot in tracker_snapshots.items():
                if tracker_snapshot.reference_invalidated:
                    application.arm_controllers[side].invalidate_reference()
                    active_tracker_sources[side] = None
                    counters[f"{side}.tracker_epoch_changes"] += 1
            for side, hand_snapshot in hand_snapshots.items():
                if hand_snapshot.epoch_changed:
                    application.hand_controllers.invalidate_input_epoch(
                        side,
                    )
                    active_hand_sources[side.value] = None
                    counters[f"{side.value}.glove_epoch_changes"] += 1

            dataset_candidate_tick = bool(
                dataset_lifecycle is not None
                and dataset_lifecycle.boundaries
                and dataset_lifecycle.boundaries[-1].event
                in {DatasetEpisodeEvent.READY, DatasetEpisodeEvent.RECORDING}
            )
            pre_feedback = {side: scene.feedback_q27(side) for side in ("left", "right")}
            pre_dataset_frame: _DeferredDatasetState | None = None
            if dataset_candidate_tick:
                if DATASET_PROFILE is None:
                    raise RuntimeError("dataset profile is missing")
                dataset_state_start_ns = time.monotonic_ns()
                pre_simulation_time_s = _simulation_time_s(scene)
                pre_qdot = {side: scene.feedback_qdot27(side) for side in ("left", "right")}
                if previous_post_dataset_state is None:
                    pre_dataset_snapshot = scene.dataset_state_snapshot(
                        q54_profile=DATASET_PROFILE.q54,
                        q27_by_side=pre_feedback,
                        qdot27_by_side=pre_qdot,
                    )
                    pre_dataset_frame = _DeferredDatasetState(
                        run_id=current_run_id or "",
                        control_index=scheduled_tick.control_index,
                        phase=SimulationFramePhase.PRE_ACTION,
                        simulation_time_s=pre_simulation_time_s,
                        physics_boundary_index=completed_physics_steps,
                        snapshot=pre_dataset_snapshot,
                    )
                    counters["dataset.pre_state_full_reads"] += 1
                else:
                    q54_rad = DATASET_PROFILE.q54.assemble_from_q27(
                        left_q27_rad=tuple(float(value) for value in pre_feedback["left"]),
                        right_q27_rad=tuple(float(value) for value in pre_feedback["right"]),
                    )
                    qdot54_rad_s = DATASET_PROFILE.q54.assemble_velocity_from_q27(
                        left_qdot27_rad_s=tuple(float(value) for value in pre_qdot["left"]),
                        right_qdot27_rad_s=tuple(float(value) for value in pre_qdot["right"]),
                    )
                    pre_dataset_frame = previous_post_dataset_state.as_next_pre_action(
                        control_index=scheduled_tick.control_index,
                        simulation_time_s=pre_simulation_time_s,
                        physics_boundary_index=completed_physics_steps,
                        q54_rad=q54_rad,
                        qdot54_rad_s=qdot54_rad_s,
                    )
                    counters["dataset.pre_state_reused"] += 1
                duration_ns = time.monotonic_ns() - dataset_state_start_ns
                stage_timings.observe(counters, "dataset.pre_state", duration_ns)
            control_start_ns = time.monotonic_ns()
            result = application.cycle.step(
                feedback_q7_rad={
                    side: pre_feedback[side][application.arm_indices[side]].tolist()
                    for side in application.arm_controllers
                },
                now_ns=tick_ns,
            )
            control_end_ns = time.monotonic_ns()
            stage_timings.observe(counters, "control.snapshot", snapshot_end_ns - snapshot_start_ns)
            stage_timings.observe(counters, "control.solve", control_end_ns - control_start_ns)
            arm_steps = {labelled.side: labelled.step for labelled in result.arm_steps}
            hand_steps = {labelled.side.value: labelled.step for labelled in result.hand_steps}
            route_command_start_ns = time.monotonic_ns()
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
            route_command_duration_ns = time.monotonic_ns() - route_command_start_ns
            stage_timings.observe(counters, "ros.route_command", route_command_duration_ns)
            apply_start_ns = time.monotonic_ns()
            applied_targets = scene.apply_targets()
            apply_end_ns = time.monotonic_ns()
            stage_timings.observe(counters, "control.apply", apply_end_ns - apply_start_ns)
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
                if contact_tracker is not None:
                    contact_tracker.set_frame("ros_fixture", completed_physics_steps)
                physics_substep_start_ns.append(time.monotonic_ns())
                world_step_start_ns = time.monotonic_ns()
                scene.world.step(render=False)
                world_step_duration_ns = time.monotonic_ns() - world_step_start_ns
                stage_timings.observe(counters, "physics.world_step", world_step_duration_ns)
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
                    stage_timings.observe(counters, "camera.observe", camera_observe_duration_ns)
                completed_physics_steps += 1
            physics_end_ns = time.monotonic_ns()
            stage_timings.observe(counters, "physics.tick", physics_end_ns - physics_start_ns)
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
            post_dataset_frame: _DeferredDatasetState | None = None
            operator_preview_state: _DeferredDatasetState | None = None
            operator_preview_due = bool(
                ARGS.external_preview_required
                and (scheduled_tick.control_index + 1) % OPERATOR_PREVIEW_CONTROL_INTERVAL == 0
            )
            trace_time_ns = time.monotonic_ns()
            completed_frames += 1
            stage_timings.observe(counters, "control.tick", trace_time_ns - tick_ns)
            scheduler.complete(completed_ns=trace_time_ns)

            post_control_start_ns = time.monotonic_ns()
            post_feedback = {side: scene.feedback_q27(side) for side in ("left", "right")}
            if self_collision_q27_probe is not None:
                self_collision_q27_probe.observe(post_feedback, applied_targets)
            route_feedback_start_ns = time.monotonic_ns()
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
            stage_timings.observe(
                counters,
                "ros.route_feedback",
                time.monotonic_ns() - route_feedback_start_ns,
            )
            if dataset_candidate_tick or operator_preview_due:
                if DATASET_PROFILE is None:
                    raise RuntimeError("dataset profile is missing")
                dataset_state_start_ns = time.monotonic_ns()
                link_snapshots = (
                    scene.operator_preview_link_snapshots()
                    if operator_preview_due
                    else None
                )
                post_dataset_snapshot = scene.dataset_state_snapshot(
                    q54_profile=DATASET_PROFILE.q54,
                    q27_by_side=post_feedback,
                    qdot27_by_side={
                        side: scene.feedback_qdot27(side) for side in ("left", "right")
                    },
                    link_snapshots=link_snapshots,
                )
                post_state = _DeferredDatasetState(
                    run_id=current_run_id or "",
                    control_index=scheduled_tick.control_index,
                    phase=SimulationFramePhase.POST_ACTION,
                    simulation_time_s=simulation_time_after_s,
                    physics_boundary_index=completed_physics_steps,
                    snapshot=post_dataset_snapshot,
                )
                duration_ns = time.monotonic_ns() - dataset_state_start_ns
                if dataset_candidate_tick:
                    post_dataset_frame = post_state
                    stage_timings.observe(counters, "dataset.post_state", duration_ns)
                    previous_post_dataset_state = post_dataset_frame
                if operator_preview_due:
                    if link_snapshots is None:
                        raise RuntimeError("operator preview link snapshot is missing")
                    preview_snapshot_start_ns = time.monotonic_ns()
                    preview_snapshot = scene.operator_preview_state_snapshot(
                        dataset_snapshot=post_dataset_snapshot,
                        link_snapshots=link_snapshots,
                    )
                    operator_preview_state = _DeferredDatasetState(
                        run_id=post_state.run_id,
                        control_index=post_state.control_index,
                        phase=post_state.phase,
                        simulation_time_s=post_state.simulation_time_s,
                        physics_boundary_index=post_state.physics_boundary_index,
                        snapshot=preview_snapshot,
                    )
                    stage_timings.observe(
                        counters,
                        "operator_preview.snapshot",
                        time.monotonic_ns() - preview_snapshot_start_ns,
                    )
            if camera_render_due and not ARGS.gui:
                replay_snapshot_start_ns = time.monotonic_ns()
                replay_snapshot = scene.camera_replay_snapshot(
                    q27_by_side=post_feedback,
                )
                replay_snapshot_duration_ns = time.monotonic_ns() - replay_snapshot_start_ns
                stage_timings.observe(
                    counters,
                    "camera.replay_snapshot",
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

            publish_context: _RecordingPublishContext | None = None
            if (
                recording_publisher_worker is not None
                and current_run_id is not None
                and recording_failure_reason is None
            ):
                try:
                    context_build_start_ns = time.monotonic_ns()
                    publish_context = _build_recording_publish_context(
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
                        include_scene_state=scene_state_publisher is not None,
                        dataset_pre_state=pre_dataset_frame,
                        dataset_post_state=post_dataset_frame,
                        operator_preview_state=operator_preview_state,
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
                    context_build_duration_ns = time.monotonic_ns() - context_build_start_ns
                    stage_timings.observe(
                        counters,
                        "recording.context_build",
                        context_build_duration_ns,
                    )
                except Exception as exc:
                    recording_failure_reason = _bounded_reason(exc)
                    counters["recording.trace_failures"] += 1
                    print(
                        f"NV5 RECORDING DEGRADED: {recording_failure_reason}",
                        file=sys.stderr,
                        flush=True,
                    )
            if publish_context is not None:
                try:
                    enqueue_start_ns = time.monotonic_ns()
                    recording_publisher_worker.submit(publish_context)
                    stage_timings.observe(
                        counters,
                        "recording.enqueue",
                        time.monotonic_ns() - enqueue_start_ns,
                    )
                except Exception as exc:
                    recording_failure_reason = _bounded_reason(exc)
                    counters["recording.trace_failures"] += 1
            stage_timings.observe(
                counters,
                "post_control.bookkeeping",
                time.monotonic_ns() - post_control_start_ns,
            )
            if dataset_lifecycle is not None and recording_failure_reason is not None:
                failure_reason = f"dataset_fact_publish_failed:{recording_failure_reason}"
                break
            if dataset_lifecycle is not None and recording_failure_reason is None:
                if dataset_boundary_publisher is None:
                    raise RuntimeError("dataset boundary publisher is missing")
                last_event = dataset_lifecycle.boundaries[-1].event
                if last_event is DatasetEpisodeEvent.OPENED:
                    input_ready = all(
                        adapter.selected is not None for adapter in tracker_inputs.values()
                    ) and all(adapter.selected is not None for adapter in hand_inputs.values())
                    references_ready = all(
                        active_tracker_sources[side] is not None
                        and active_hand_sources[side] is not None
                        for side in ("left", "right")
                    )
                    episode_readiness = EpisodeReadiness(
                        recorder_ready=True,
                        inputs_ready=input_ready,
                        references_ready=references_ready,
                        scene_settled=bool(readiness.get("converged")),
                    )
                    if episode_readiness.ready:
                        dataset_boundary_publisher.publish(
                            dataset_episode_boundary_to_message(
                                dataset_lifecycle.ready(
                                    host_time_ns=time.monotonic_ns(),
                                    simulation_time_s=simulation_time_after_s,
                                    readiness=episode_readiness,
                                )
                            )
                        )
                        print(
                            "DATASET EPISODE READY: next complete 30 Hz tick is candidate k0",
                            flush=True,
                        )
                elif last_event is DatasetEpisodeEvent.READY:
                    dataset_boundary_publisher.publish(
                        dataset_episode_boundary_to_message(
                            dataset_lifecycle.recording(
                                host_time_ns=time.monotonic_ns(),
                                control_index=scheduled_tick.control_index,
                                simulation_time_s=simulation_time_before_s,
                            )
                        )
                    )
                    counters["dataset.candidate_ticks"] += 1
                elif last_event is DatasetEpisodeEvent.RECORDING:
                    counters["dataset.candidate_ticks"] += 1

                bounded_stop = ARGS.frames > 0 and completed_frames >= ARGS.frames
                if stop_request.requested or bounded_stop:
                    if dataset_lifecycle.boundaries[-1].event is not DatasetEpisodeEvent.RECORDING:
                        failure_reason = "dataset_stop_before_ready_or_first_candidate"
                        break
                    effective_dataset_stop_signal = stop_request.requested_signal or int(
                        signal.SIGTERM
                    )
                    dataset_lifecycle.request_stop(effective_dataset_stop_signal)
                    dataset_boundary_publisher.publish(
                        dataset_episode_boundary_to_message(
                            dataset_lifecycle.complete_final_tick(
                                host_time_ns=time.monotonic_ns(),
                                control_index=scheduled_tick.control_index,
                                simulation_time_s=simulation_time_after_s,
                            )
                        )
                    )
                    break
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
            if recording_publisher_worker is not None:
                for metric_name, metric_value in recording_publisher_worker.close().items():
                    counters[metric_name] = metric_value
        except Exception as exc:
            cleanup_error = exc
            if failure_reason is None:
                failure_reason = _bounded_reason(exc)
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
            if dataset_lifecycle is not None:
                last_event = dataset_lifecycle.boundaries[-1].event
                if (
                    last_event is DatasetEpisodeEvent.STOP_REQUESTED
                    and failure_reason is None
                    and recording_failure_reason is None
                ):
                    if dataset_boundary_publisher is None:
                        raise RuntimeError("dataset boundary publisher is missing during close")
                    dataset_boundary_publisher.publish(
                        dataset_episode_boundary_to_message(
                            dataset_lifecycle.closed(
                                host_time_ns=time.monotonic_ns(),
                                simulation_time_s=_simulation_time_s(scene),
                            )
                        )
                    )
                    if not dataset_boundary_publisher.wait_for_all_acked(Duration(seconds=2.0)):
                        raise RuntimeError("dataset closed boundary acknowledgement timed out")
                    counters["dataset.closed_boundary_acked"] += 1
                elif failure_reason is None and recording_failure_reason is None:
                    failure_reason = "dataset_lifecycle_did_not_reach_stop_requested"
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
                        stage_timings=stage_timings,
                        tracker_inputs=tracker_inputs,
                        hand_inputs=hand_inputs,
                        executor_metrics=asdict(executor_worker.metrics),
                        python_gc_frozen_object_count=(python_gc_frozen_object_count),
                        python_gc_unfrozen_on_close=(python_gc_unfrozen_on_close),
                        stop_signal=(
                            effective_dataset_stop_signal or stop_request.requested_signal
                        ),
                        failure_reason=failure_reason,
                        recording_failure_reason=(recording_failure_reason),
                        camera_capture_receipt=camera_capture_receipt,
                        dataset_lifecycle=dataset_lifecycle,
                        q54_runtime_inventory=q54_runtime_inventory,
                        final_fixed_body_states=[
                            {
                                "prim_path": snapshot.prim_path,
                                "position_m": list(snapshot.position_m),
                                "quat_wxyz": list(snapshot.quat_wxyz),
                            }
                            for snapshot in scene.fixed_body_snapshots()
                        ],
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

    self_collision_qualification = None
    if ARGS.self_collision_qualification:
        if contact_tracker is None or self_collision_q27_probe is None:
            raise RuntimeError("self-collision qualification probes are missing")
        self_collision_qualification = _self_collision_qualification_mapping(
            scene=scene,
            tracker=contact_tracker,
            probe=self_collision_q27_probe,
            readback=self_collision_readback,
            contact_api_paths=contact_api_paths,
            completed_frames=completed_frames,
            counters=counters,
        )
    del contact_subscription

    report = {
        "schema": "wujihand.isaac_ros_dual_teleoperation_receipt.v2",
        "passed": (
            True if self_collision_qualification is None else self_collision_qualification["passed"]
        ),
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
        "stage_timing": stage_timings.to_mapping(),
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
        "self_collision_qualification": self_collision_qualification,
        "self_collision_policy": _self_collision_policy_mapping(),
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
    return 0 if report["passed"] else 2


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


def _recording_owned_q27(
    values: dict[str, NDArray[np.float64]],
) -> tuple[tuple[str, NDArray[np.float64]], ...]:
    if set(values) != {"left", "right"}:
        raise RuntimeError("recording q27 ownership must cover left and right")
    result: list[tuple[str, NDArray[np.float64]]] = []
    for side in ("left", "right"):
        vector = values[side]
        if vector.shape != (27,) or not np.isfinite(vector).all():
            raise RuntimeError(f"recording {side} q27 ownership is invalid")
        vector.setflags(write=False)
        result.append((side, vector))
    return tuple(result)


def _build_recording_publish_context(
    *,
    run_id: str,
    tick_id: int,
    stage_times: TickStageTimes,
    execution: TickExecutionTrace,
    include_scene_state: bool,
    dataset_pre_state: SimulationStateFrame | _DeferredDatasetState | None,
    dataset_post_state: SimulationStateFrame | _DeferredDatasetState | None,
    operator_preview_state: SimulationStateFrame | _DeferredDatasetState | None,
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
) -> _RecordingPublishContext:
    tracker_selections: dict[str, RosTrackerSelection | None] = {}
    hand_selections: dict[str, RosHandSelection | None] = {}
    for side in ("left", "right"):
        tracker_selection = tracker_inputs[side].selected
        tracker_selections[side] = tracker_selection
        selected_tracker = _tracker_source_trace(tracker_selection)
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
        hand_selection = hand_inputs[hand_side].selected if hand_side in hand_inputs else None
        hand_selections[side] = hand_selection
        selected_hand = _hand_source_trace(hand_selection)
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

    scene_states: list[SceneRigidBodyState] = []
    scene_time_ns = time.monotonic_ns()
    reusable_post_state = dataset_post_state or operator_preview_state
    if include_scene_state and reusable_post_state is not None:
        if reusable_post_state.phase is not SimulationFramePhase.POST_ACTION:
            raise RuntimeError("scene-state reuse requires a post-action dataset frame")
        post_rigid_bodies = (
            reusable_post_state.snapshot.rigid_bodies
            if isinstance(reusable_post_state, _DeferredDatasetState)
            else reusable_post_state.rigid_bodies
        )
        for item in post_rigid_bodies:
            scene_states.append(
                SceneRigidBodyState(
                    run_id=run_id,
                    tick_id=tick_id,
                    prim_path=item.prim_path,
                    recorded_time_ns=scene_time_ns,
                    position_m=item.position_m,
                    quat_wxyz=item.quat_wxyz,
                    linear_velocity_m_s=item.linear_velocity_m_s,
                    angular_velocity_deg_s=tuple(
                        math.degrees(value) for value in item.angular_velocity_rad_s
                    ),
                    kinematic_enabled=item.kinematic,
                )
            )
    elif include_scene_state:
        for snapshot in scene.rigid_body_snapshots():
            scene_states.append(
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
    if (dataset_pre_state is None) != (dataset_post_state is None):
        raise RuntimeError("recording publish batch has an incomplete dataset state pair")
    if (
        operator_preview_state is not None
        and operator_preview_state.phase is not SimulationFramePhase.POST_ACTION
    ):
        raise RuntimeError("operator preview requires a post-action state")
    return _RecordingPublishContext(
        run_id=run_id,
        tick_id=tick_id,
        stage_times=stage_times,
        execution=execution,
        pre_feedback=_recording_owned_q27(pre_feedback),
        applied_targets=_recording_owned_q27(applied_targets),
        post_feedback=_recording_owned_q27(post_feedback),
        arm_steps=tuple((side, arm_steps[side]) for side in ("left", "right")),
        hand_steps=tuple(sorted(hand_steps.items())),
        tracker_selections=tuple((side, tracker_selections[side]) for side in ("left", "right")),
        hand_selections=tuple((side, hand_selections[side]) for side in ("left", "right")),
        active_tracker_sources=tuple(
            (side, active_tracker_sources[side]) for side in ("left", "right")
        ),
        active_hand_sources=tuple((side, active_hand_sources[side]) for side in ("left", "right")),
        arm_layout_ids=tuple(
            (side, scene.arm_profiles[side].layout_id) for side in ("left", "right")
        ),
        hand_layout_ids=tuple(
            (side, scene.hand_profiles[side].layout_id) for side in ("left", "right")
        ),
        scene_states=tuple(scene_states),
        dataset_pre_state=dataset_pre_state,
        dataset_post_state=dataset_post_state,
        operator_preview_state=operator_preview_state,
    )


def _build_recording_publish_batch(
    context: _RecordingPublishContext,
) -> _RecordingPublishBatch:
    pre_feedback = dict(context.pre_feedback)
    applied_targets = dict(context.applied_targets)
    post_feedback = dict(context.post_feedback)
    arm_steps = dict(context.arm_steps)
    hand_steps = dict(context.hand_steps)
    tracker_selections = dict(context.tracker_selections)
    hand_selections = dict(context.hand_selections)
    active_tracker_sources = dict(context.active_tracker_sources)
    active_hand_sources = dict(context.active_hand_sources)
    arm_layout_ids = dict(context.arm_layout_ids)
    hand_layout_ids = dict(context.hand_layout_ids)
    tick_traces = tuple(
        _tick_trace(
            run_id=context.run_id,
            tick_id=context.tick_id,
            side=side,
            times=context.stage_times,
            execution=context.execution,
            pre_feedback=pre_feedback[side],
            applied_target=applied_targets[side],
            post_feedback=post_feedback[side],
            arm_step=arm_steps[side],
            hand_step=hand_steps.get(side),
            tracker_selection=tracker_selections[side],
            active_tracker_source=active_tracker_sources[side],
            hand_selection=hand_selections[side],
            active_hand_source=active_hand_sources[side],
            arm_layout_id=arm_layout_ids[side],
            hand_layout_id=hand_layout_ids[side],
        )
        for side in ("left", "right")
    )
    dataset_pre_state = context.dataset_pre_state
    dataset_post_state = context.dataset_post_state
    if isinstance(dataset_pre_state, _DeferredDatasetState):
        dataset_pre_state = dataset_pre_state.materialize()
    if isinstance(dataset_post_state, _DeferredDatasetState):
        dataset_post_state = dataset_post_state.materialize()
    dataset_states = (
        ()
        if dataset_pre_state is None or dataset_post_state is None
        else (dataset_pre_state, dataset_post_state)
    )
    operator_preview_state = context.operator_preview_state
    if context.operator_preview_state is context.dataset_post_state:
        operator_preview_state = dataset_post_state
    elif isinstance(operator_preview_state, _DeferredDatasetState):
        operator_preview_state = operator_preview_state.materialize()
    return _RecordingPublishBatch(
        tick_traces=tick_traces,
        scene_states=context.scene_states,
        dataset_states=dataset_states,
        operator_preview_state=operator_preview_state,
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
    arm_layout_id: str,
    hand_layout_id: str,
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
            layout_id=arm_layout_id,
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
                layout_id=hand_layout_id,
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


def _chain_preflight_manifest() -> dict[str, object] | None:
    if CHAIN_PREFLIGHT is None:
        return None
    if ARGS.chain_preflight is None:
        raise RuntimeError("record-chain preflight globals are incomplete")
    if ARGS.run_root is None:
        raise RuntimeError("record-chain preflight provenance requires a recording run root")
    receipt_path = ARGS.chain_preflight.resolve()
    run_root_path = ARGS.run_root.resolve()
    try:
        relative = receipt_path.relative_to(run_root_path)
    except ValueError as exc:
        raise RuntimeError("record-chain preflight receipt must stay inside the run root") from exc
    return {
        "path": relative.as_posix(),
        "sha256": sha256_file(receipt_path),
        "qualification_id": CHAIN_PREFLIGHT.get("qualification_id"),
        "input_mode": CHAIN_PREFLIGHT.get("input_mode"),
        "task_scene": CHAIN_PREFLIGHT.get("task_scene"),
    }


def _run_manifest_payload(
    *,
    scene: DualNeroHand2IsaacScene,
    recording_opened_ns: int,
    camera_capture: DualD405CameraCapture | None,
    camera_warmup: dict[str, object] | None,
    q54_runtime_inventory: Q54RuntimeInventory | None,
) -> dict[str, object]:
    if DATASET_MODE:
        if DATASET_PROFILE is None or dataset_profile_ref is None or q54_runtime_inventory is None:
            raise RuntimeError("dataset recording manifest requires q54 runtime closure")
        camera_manifest: dict[str, object] = {
            "simulation_only": True,
            "online_capture_enabled": False,
            "payloads_in_control_mcap": [],
            "data_phase": "offline_fixed_pre_action_state_v1",
            "rgb_only": True,
            "depth": "omitted",
            "warning": (
                "The two wrist views use a synthetic 140-degree projection; this is not "
                "a physical RealSense D405 specification or calibration."
            ),
            "cameras": [
                {
                    "logical_id": camera.logical_id,
                    "feature_key": camera.feature_key,
                    "carrier_identity": camera.carrier_identity,
                    "profile_path": camera.profile.path,
                    "profile_sha256": camera.profile.sha256,
                    "payload_whitelist": list(camera.payload_whitelist),
                    "physical_calibration_compatible": (camera.physical_calibration_compatible),
                }
                for camera in DATASET_PROFILE.cameras
            ],
        }
        dataset_manifest: dict[str, object] | None = {
            "profile_id": DATASET_PROFILE.profile_id,
            "profile_path": dataset_profile_ref.path,
            "profile_sha256": DATASET_PROFILE.file_sha256,
            "episode_id_rule": "run_id_equals_episode_id",
            "source_mode": DATASET_SOURCE_MODE.value,
            "dataset_eligible": DATASET_ELIGIBLE,
            "policy_fps": DATASET_PROFILE.policy_fps,
            "selection": "relative_all_control_index_no_interpolation_v1",
            "observation_phase": "pre_action",
            "q54_runtime_inventory": q54_runtime_inventory.to_mapping(),
            "dynamic_object_inventory": dict(scene.dataset_dynamic_object_paths),
            "kinematic_link_inventory": [
                {"side": side, "logical_link_id": logical_id, "prim_path": path}
                for (side, logical_id), path in sorted(scene.dataset_kinematic_link_paths.items())
            ],
            "raw_contact": {
                "available": False,
                "reason": "unsupported_in_dataset_profile",
            },
        }
    else:
        if camera_capture is None or camera_warmup is None:
            raise RuntimeError("recording manifest requires active D405 camera inventory")
        camera_manifest = {
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
        }
        dataset_manifest = None
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
            "assembly_path": RESOLVED.session.assembly_path,
            "assembly_sha256": sha256_file(ROOT / RESOLVED.session.assembly_path),
            "workcell_path": RESOLVED.session.workcell_path,
            "workcell_sha256": sha256_file(ROOT / RESOLVED.session.workcell_path),
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
        "self_collision_policy": _self_collision_policy_mapping(),
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
                if CONTROL_MAXIMUM_CATCH_UP_TICKS
                else "monotonic_fixed_rate_skip_missed_v1"
            ),
            "maximum_consecutive_catch_up_ticks": CONTROL_MAXIMUM_CATCH_UP_TICKS,
            "synthetic_camera_service_phase": (
                "offline_fixed_pre_action_state_v1"
                if DATASET_MODE
                else ("inline_gui_render_v1" if ARGS.gui else CAMERA_CAPTURE_EXECUTION)
            ),
            "executor": "background_single_threaded_spin_v1",
            "gui": ARGS.gui,
            "external_gui_preview_required": ARGS.external_preview_required,
            "external_gui_preview_hz": (RENDER_HZ if ARGS.external_preview_required else None),
            "external_gui_preview_state_topic": (
                f"{namespace}/{OPERATOR_PREVIEW_STATE_TOPIC}"
                if ARGS.external_preview_required
                else None
            ),
            "external_gui_preview_state_recorded": False,
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
            "record_chain_preflight": _chain_preflight_manifest(),
        },
        "recording_inventory": {
            "topics": list(
                recording_topics(
                    namespace,
                    RESOLVED.route_plan,
                    include_synthetic_d405=not DATASET_MODE,
                    include_dataset_facts=DATASET_MODE,
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
                "four physics substep indices, host times and simulation times",
                "target-effective simulation interval and render index",
                "Workcell dynamic rigid-body state",
                (
                    "pre/post q54/qdot54, dynamic-object and kinematic-link closure"
                    if DATASET_MODE
                    else "dual completed-frame synthetic wrist-camera transactions"
                ),
            ],
        },
        "dataset": dataset_manifest,
        "synthetic_d405_wrist_cameras": camera_manifest,
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
            "joint_velocity_feedback": DATASET_MODE,
            "joint_effort_feedback": False,
            "dynamic_rigid_body_pose": True,
            "dynamic_rigid_body_velocity": "when_usd_attribute_available",
            "fixed_body_pose": "manifest",
            "raw_contact": False,
            "link7_palm_fingertip_pose": DATASET_MODE,
            "task_truth": False,
            "rosbag_internal_queue_depth": False,
            "rosbag_internal_drop_counter": False,
            "executor_internal_queue_depth": False,
            "executor_internal_drop_counter": False,
            "latest_mailbox_superseded_counter": True,
            "control_schedule_missed_period_counter": True,
            "physics_substep_trace": True,
            "render_trace": True,
            "synthetic_d405_rgb_depth_camera_info_truth": not DATASET_MODE,
            "camera_completed_frame_identity": (
                "offline_renderer_completed_identity_v1"
                if DATASET_MODE
                else "camera_sensor_writer_reference_time_v1"
            ),
            "camera_pose_history_join": not DATASET_MODE,
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
    stage_timings: _StageTimings,
    tracker_inputs: dict[str, RosTrackerInputAdapter],
    hand_inputs: dict[HandSide, RosHandObservationInputAdapter],
    executor_metrics: dict[str, object],
    python_gc_frozen_object_count: int,
    python_gc_unfrozen_on_close: bool,
    stop_signal: int | None,
    failure_reason: str | None,
    recording_failure_reason: str | None,
    camera_capture_receipt: dict[str, object] | None,
    dataset_lifecycle: DatasetEpisodeLifecycle | None,
    q54_runtime_inventory: Q54RuntimeInventory | None,
    final_fixed_body_states: list[dict[str, object]],
) -> dict[str, object]:
    return {
        "scope": "consumer_and_trace_producer_only",
        "self_collision_policy": _self_collision_policy_mapping(),
        "completed_ticks": completed_frames,
        "completed_physics_steps": completed_physics_steps,
        "completed_renders": completed_renders,
        "configured_timing": {
            "physics_hz": RESOLVED.control_profile.physics_hz,
            "control_hz": CONTROL_HZ,
            "render_hz": RENDER_HZ,
            "physics_substeps_per_control": PHYSICS_SUBSTEPS_PER_CONTROL,
            "control_ticks_per_render": CONTROL_TICKS_PER_RENDER,
            "maximum_consecutive_catch_up_ticks": CONTROL_MAXIMUM_CATCH_UP_TICKS,
            "synthetic_camera_service_phase": (
                "inline_gui_render_v1" if ARGS.gui else CAMERA_CAPTURE_EXECUTION
            ),
            "process_cpu_affinity": PROCESS_CPU_AFFINITY,
            "block_on_render": GUI_BLOCK_ON_RENDER,
            "python_gc_policy": PYTHON_GC_POLICY,
            "external_gui_preview_required": ARGS.external_preview_required,
            "external_gui_preview_hz": (RENDER_HZ if ARGS.external_preview_required else None),
            "external_gui_preview_state_topic": (
                f"/{RESOLVED.deployment.root_namespace}/{OPERATOR_PREVIEW_STATE_TOPIC}"
                if ARGS.external_preview_required
                else None
            ),
            "external_gui_preview_state_recorded": False,
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
        "stage_timing": stage_timings.to_mapping(),
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
        "final_fixed_body_states": final_fixed_body_states,
        "synthetic_d405_wrist_cameras": camera_capture_receipt,
        "dataset": (
            None
            if dataset_lifecycle is None
            else {
                "profile_id": DATASET_PROFILE.profile_id,
                "profile_sha256": DATASET_PROFILE.file_sha256,
                "events": [boundary.to_mapping() for boundary in dataset_lifecycle.boundaries],
                "lifecycle_closed": bool(
                    dataset_lifecycle.boundaries
                    and dataset_lifecycle.boundaries[-1].event is DatasetEpisodeEvent.CLOSED
                ),
                "q54_runtime_inventory": (
                    None if q54_runtime_inventory is None else q54_runtime_inventory.to_mapping()
                ),
            }
        ),
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
