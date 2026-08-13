#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported only after SimulationApp starts.
"""Render a passive 20 Hz GUI replica for one headless dataset recording."""

from __future__ import annotations

import argparse
from collections.abc import Mapping
import gc
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
from threading import Lock
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

from wujihand.adapters.simulation import (
    load_nero_dual_simulation_startup_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.dataset import load_mini_dataset_profile
from wujihand.domain import RunRecordingState
from wujihand.domain.dataset_recording import (
    SimulationFramePhase,
    SimulationStateFrame,
)
from wujihand.integrity import sha256_file
from wujihand.runtime import (
    FixedRateScheduler,
    RosDeploymentResolver,
    configure_current_process_cpu_affinity,
)
from wujihand.runtime.isaac_dual_scene import (
    workcell_frame_position,
    resolve_dual_side_runtimes,
)
from wujihand.runtime.wuji_hand2_record_chain import (
    load_record_chain_preflight_receipt,
    resolve_record_chain_workcell_plan,
)


DEFAULT_DEPLOYMENT = (
    ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml"
)
DEFAULT_LOCAL_BINDING = ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"
NERO_LULA_DESCRIPTION = ROOT / "configs/profiles/agilex_nero_lula_kinematics_v1.yaml"
PREVIEW_HZ = 20
PREVIEW_MAXIMUM_CATCH_UP_TICKS = 2
PREVIEW_WIDTH = 480
PREVIEW_HEIGHT = 300
PREVIEW_RENDERER = "MinimalRendering"
PREVIEW_MINIMAL_SHADING_MODE = 3
PREVIEW_THREAD_LIMIT_CAP = 32
PREVIEW_CONSUMER_WAIT_TIMEOUT_S = 180.0
PREVIEW_SHUTDOWN_GRACE_S = 8.0
PREVIEW_WARMUP_MIN_RENDERS = 30
PREVIEW_WARMUP_MAX_RENDERS = 120
PREVIEW_WARMUP_STABLE_RENDERS = 10
PREVIEW_WARMUP_RENDER_BUDGET_NS = 40_000_000
PREVIEW_RENDER_BUDGET_NS = 50_000_000
PREVIEW_BACKGROUND_COLOR_RGB = (0.30, 0.30, 0.30)
STATE_CLOSURE_LIMIT = 2e-5
OPERATOR_PREVIEW_STATE_TOPIC = "operator_preview/simulation_state"
PREVIEW_TRANSFORM_SYNC = "full_link_truth_to_official_episode_replay_usd_v3"
PREVIEW_POSE_BACKEND = "usd"
PREVIEW_POSE_READBACK_BACKEND = "usd"
PREVIEW_POSE_REPLAY_BACKEND = "usd_parent_first"
PREVIEW_VIEWPORT_CAPTURE_PROFILE = "active_viewport_ldr_rgb_replicator_single_step_v4"
PREVIEW_RENDER_TRANSACTION = (
    "reference_time_annotated_single_tick_replicator_step_zero_subframes_delta_time_zero"
)
PREVIEW_SYNC_ANNOTATOR = "ReferenceTime"
PREVIEW_MULTI_TICK_RENDERING = False
PREVIEW_VISIBLE_MOTION_Q54_THRESHOLD_RAD = 0.1
PREVIEW_VISIBLE_MOTION_TRIGGER_RAD = 0.05
PREVIEW_VISIBLE_MOTION_PIXEL_DELTA = 8
PREVIEW_VISIBLE_MOTION_MIN_CHANGED_FRACTION = 0.0005
PREVIEW_RENDERABLE_GEOMETRY_MATRIX_DELTA = 1e-4
PREVIEW_REFERENCE_WARMUP_APPLIED_FRAMES = 30
PREVIEW_COMPONENT_SOURCE_POSITION_DELTA_M = 1e-4
PREVIEW_COMPONENT_SOURCE_ORIENTATION_DELTA_RAD = 1e-3
PREVIEW_COMPONENTS = ("left_arm", "left_hand", "right_arm", "right_hand")
SCENE_CAMERA_PRIM_PATH = "/OmniverseKit_Persp"
SCENE_CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
SCENE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deployment", type=Path, default=DEFAULT_DEPLOYMENT)
    parser.add_argument(
        "--local-runtime-binding",
        type=Path,
        default=DEFAULT_LOCAL_BINDING,
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--cpu-affinity")
    parser.add_argument("--chain-preflight", type=Path)
    parser.add_argument("--wait-for-node")
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def _wait_for_ros_node(node_path: str, *, timeout_s: float) -> None:
    if not node_path.startswith("/") or node_path.endswith("/"):
        raise ValueError("ROS node path must be absolute")

    import rclpy  # type: ignore[import-not-found]
    from rclpy.context import Context  # type: ignore[import-not-found]
    from rclpy.executors import SingleThreadedExecutor  # type: ignore[import-not-found]
    from rclpy.node import Node  # type: ignore[import-not-found]

    context = Context()
    rclpy.init(context=context)
    node = Node("dataset_preview_startup_gate", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            names = {
                f"{namespace.rstrip('/')}/{name}"
                for name, namespace in node.get_node_names_and_namespaces()
            }
            if node_path in names:
                return
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    raise RuntimeError(f"ROS startup node did not appear: {node_path}")


def _maximum_abs_error(left: object, right: object) -> float:
    left_array = np.asarray(left, dtype=np.float64)
    right_array = np.asarray(right, dtype=np.float64)
    if left_array.shape != right_array.shape or not (
        np.isfinite(left_array).all() and np.isfinite(right_array).all()
    ):
        return math.inf
    return float(np.max(np.abs(left_array - right_array), initial=0.0))


def _preview_pose_paths(frame: SimulationStateFrame) -> tuple[str, ...]:
    paths = tuple(item.prim_path for item in frame.kinematic_links) + tuple(
        item.prim_path for item in frame.rigid_bodies
    )
    if len(paths) != len(set(paths)):
        raise ValueError("operator preview pose inventory contains duplicate prim paths")
    if len(frame.kinematic_links) <= 14:
        raise ValueError("operator preview requires the complete articulation-link inventory")
    return paths


def _ancestry_tiers(paths: tuple[str, ...]) -> tuple[tuple[int, ...], ...]:
    """Assign absolute prim paths to parent-before-child replay tiers."""

    if not paths:
        raise ValueError("operator preview pose inventory is empty")
    by_path = {path: index for index, path in enumerate(paths)}
    if len(by_path) != len(paths):
        raise ValueError("operator preview pose inventory contains duplicates")
    depths: list[int] = []
    for path in paths:
        if not path.startswith("/") or path == "/" or ".." in path.split("/"):
            raise ValueError(f"invalid operator preview prim path: {path}")
        parent = path.rsplit("/", 1)[0]
        depth = 0
        while parent:
            if parent in by_path:
                depth += 1
            parent = parent.rsplit("/", 1)[0]
        depths.append(depth)
    return tuple(
        tuple(index for index, depth in enumerate(depths) if depth == tier)
        for tier in range(max(depths) + 1)
    )


def _renderable_geometry_bindings(
    *,
    stage: Any,
    pose_paths: tuple[str, ...],
    usd: Any,
    usd_geom: Any,
) -> tuple[tuple[str, str], ...]:
    """Select one visible render-purpose Gprim owned by each replayed pose prim."""

    owners = set(pose_paths)
    selected: dict[str, str] = {}
    # NERO visuals are instanceable USD references.  Stage.Traverse() skips
    # their instance-proxy Gprims, which previously left both arms at the
    # authored vertical rest pose while non-instanced Hand2 visuals moved.
    for prim in usd.PrimRange.Stage(stage, usd.TraverseInstanceProxies()):
        if not prim.IsA(usd_geom.Gprim):
            continue
        imageable = usd_geom.Imageable(prim)
        if str(imageable.ComputeVisibility()) == "invisible":
            continue
        if str(imageable.ComputePurpose()) not in {"default", "render"}:
            continue
        current = prim
        while current and current.IsValid() and not current.IsPseudoRoot():
            owner = str(current.GetPath())
            if owner in owners:
                selected.setdefault(owner, str(prim.GetPath()))
                break
            current = current.GetParent()
    return tuple((path, selected[path]) for path in pose_paths if path in selected)


def _component_path_inventory(
    *,
    pose_paths: tuple[str, ...],
    replay_paths: tuple[str, ...],
    component_prefixes: dict[str, str],
    expected_source_pose_counts: dict[str, int],
) -> tuple[dict[str, tuple[str, ...]], dict[str, tuple[str, ...]]]:
    if tuple(component_prefixes) != PREVIEW_COMPONENTS:
        raise ValueError("operator-preview component prefixes are incomplete or unordered")
    if tuple(expected_source_pose_counts) != PREVIEW_COMPONENTS:
        raise ValueError("operator-preview expected component counts are incomplete or unordered")
    if len(set(component_prefixes.values())) != len(component_prefixes):
        raise ValueError("operator-preview component prefixes are not unique")

    def owned(paths: tuple[str, ...], prefix: str) -> tuple[str, ...]:
        root = prefix.rstrip("/")
        return tuple(path for path in paths if path == root or path.startswith(root + "/"))

    source = {
        component: owned(pose_paths, prefix)
        for component, prefix in component_prefixes.items()
    }
    replay = {
        component: owned(replay_paths, prefix)
        for component, prefix in component_prefixes.items()
    }
    for component in PREVIEW_COMPONENTS:
        expected_count = expected_source_pose_counts[component]
        if len(source[component]) != expected_count:
            raise RuntimeError(
                f"operator-preview {component} source inventory is incomplete: "
                f"{len(source[component])} != {expected_count}"
            )
        if not replay[component]:
            raise RuntimeError(f"operator-preview {component} has no renderable replay pose")
        if component.endswith("_arm") and replay[component] != source[component]:
            missing = tuple(path for path in source[component] if path not in replay[component])
            raise RuntimeError(
                f"operator-preview {component} renderable replay inventory is incomplete: "
                f"missing={missing}"
            )
    return source, replay


def _expected_hand_source_pose_count(
    *,
    asset_revision: str,
    side: str,
    backend_base_frame: str,
) -> int:
    known = {
        ("beta1_description_v2026_6_27", "left", "l_base_link"): 27,
        ("beta1_description_v2026_6_27", "right", "r_base_link"): 27,
        ("beta1_description_v2026_8_3", "left", "l_wrist"): 26,
        ("beta1_description_v2026_8_3", "right", "r_wrist"): 26,
    }
    key = (asset_revision, side, backend_base_frame)
    try:
        return known[key]
    except KeyError as exc:
        raise ValueError(
            "operator-preview Hand2 pose inventory is not qualified for "
            f"revision={asset_revision!r}, side={side!r}, base={backend_base_frame!r}"
        ) from exc


def _expected_arm_source_pose_count(
    *,
    asset_revision: str,
    backend_tool_frame: str,
) -> int:
    known = {
        ("model_f6642ce0_v1", "link7"): 8,
        ("model_f6642ce0_gripper_flange_v1", "gripper_flange"): 9,
        (
            "model_f6642ce0_gripper_flange_collision_proxy_v1",
            "gripper_flange",
        ): 9,
    }
    key = (asset_revision, backend_tool_frame)
    try:
        return known[key]
    except KeyError as exc:
        raise ValueError(
            "operator-preview NERO pose inventory is not qualified for "
            f"revision={asset_revision!r}, tool_frame={backend_tool_frame!r}"
        ) from exc


def _pose_group_delta(
    reference: tuple[NDArray[np.float64], NDArray[np.float64]],
    current: tuple[NDArray[np.float64], NDArray[np.float64]],
) -> tuple[float, float]:
    reference_positions, reference_orientations = reference
    current_positions, current_orientations = current
    if (
        reference_positions.shape != current_positions.shape
        or reference_orientations.shape != current_orientations.shape
        or reference_positions.ndim != 2
        or reference_positions.shape[1:] != (3,)
        or reference_orientations.ndim != 2
        or reference_orientations.shape[1:] != (4,)
    ):
        raise ValueError("operator-preview component pose arrays are incompatible")
    position_delta = _maximum_abs_error(reference_positions, current_positions)
    dots = np.abs(np.sum(reference_orientations * current_orientations, axis=1))
    angles = 2.0 * np.arccos(np.clip(dots, -1.0, 1.0))
    return position_delta, float(np.max(angles, initial=0.0))


class _PoseReplayWriter:
    """Parent-first pose playback using an official EpisodeReplayer backend."""

    def __init__(
        self,
        *,
        stage: Any,
        paths: tuple[str, ...],
        component_prefixes: dict[str, str],
        expected_source_pose_counts: dict[str, int],
        sdf: Any,
        usd: Any,
        usd_geom: Any,
        xform_prim_type: Any,
        backend_context: Any,
    ) -> None:
        self._stage = stage
        self.paths = paths
        renderable_bindings = _renderable_geometry_bindings(
            stage=stage,
            pose_paths=paths,
            usd=usd,
            usd_geom=usd_geom,
        )
        if not renderable_bindings:
            raise RuntimeError("operator preview contains no visible renderable geometry")
        self.replay_paths = tuple(owner for owner, _ in renderable_bindings)
        self.renderable_paths = tuple(geometry for _, geometry in renderable_bindings)
        (
            self.component_source_paths,
            self.component_replay_paths,
        ) = _component_path_inventory(
            pose_paths=paths,
            replay_paths=self.replay_paths,
            component_prefixes=component_prefixes,
            expected_source_pose_counts=expected_source_pose_counts,
        )
        geometry_by_owner = dict(renderable_bindings)
        self.component_renderable_paths = {
            component: tuple(geometry_by_owner[path] for path in component_paths)
            for component, component_paths in self.component_replay_paths.items()
        }
        self.tiers = _ancestry_tiers(self.replay_paths)
        self._previous_edit_target = stage.GetEditTarget()
        self._layer = sdf.Layer.CreateAnonymous("wujihand_operator_preview")
        session_layer = stage.GetSessionLayer()
        session_layer.subLayerPaths.insert(0, self._layer.identifier)
        stage.SetEditTarget(usd.EditTarget(self._layer))
        self._batches = tuple(
            xform_prim_type([self.replay_paths[index] for index in tier])
            for tier in self.tiers
        )
        self._readback_batch = xform_prim_type(self.replay_paths)
        self._backend_context = backend_context
        self._sdf = sdf
        self._usd = usd
        self._usd_geom = usd_geom
        self._reset_tiers: set[int] = set()
        self._apply_count = 0

    def apply(self, frame: SimulationStateFrame) -> None:
        items = tuple(frame.kinematic_links) + tuple(frame.rigid_bodies)
        by_path = {item.prim_path: item for item in items}
        if tuple(by_path) != self.paths:
            raise RuntimeError("operator preview pose inventory changed within one run")
        if not all(item.valid for item in frame.kinematic_links):
            raise RuntimeError("operator preview received an invalid articulation link")
        if not all(item.valid for item in frame.rigid_bodies):
            raise RuntimeError("operator preview received an invalid rigid body")
        positions = np.asarray(
            [by_path[path].position_m for path in self.replay_paths],
            dtype=np.float32,
        )
        orientations = np.asarray(
            [by_path[path].quat_wxyz for path in self.replay_paths],
            dtype=np.float32,
        )
        with self._backend_context(PREVIEW_POSE_BACKEND):
            for tier_index, (tier, batch) in enumerate(
                zip(self.tiers, self._batches, strict=True)
            ):
                selector = np.asarray(tier, dtype=np.int64)

                def write() -> None:
                    with self._sdf.ChangeBlock():
                        batch.set_world_poses(
                            positions=positions[selector],
                            orientations=orientations[selector],
                        )

                try:
                    write()
                except Exception as exc:
                    message = str(exc).lower()
                    if (
                        tier_index in self._reset_tiers
                        or "xformop" not in message
                        or "missing" not in message
                    ):
                        raise
                    batch.reset_xform_op_properties()
                    self._reset_tiers.add(tier_index)
                    write()
        self._apply_count += 1

    def position_max_error(self, frame: SimulationStateFrame) -> float:
        expected = {
            item.prim_path: item
            for item in (*frame.kinematic_links, *frame.rigid_bodies)
        }
        if tuple(expected) != self.paths:
            return math.inf
        with self._backend_context(PREVIEW_POSE_READBACK_BACKEND):
            actual_positions, _ = self._readback_batch.get_world_poses()
        if hasattr(actual_positions, "numpy"):
            actual_positions = actual_positions.numpy()
        expected_positions = np.asarray(
            [expected[path].position_m for path in self.replay_paths],
            dtype=np.float64,
        )
        return _maximum_abs_error(actual_positions, expected_positions)

    @property
    def inventory_sha256(self) -> str:
        payload = json.dumps(self.paths, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    @property
    def renderable_inventory_sha256(self) -> str:
        payload = json.dumps(self.renderable_paths, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    @property
    def replay_inventory_sha256(self) -> str:
        payload = json.dumps(self.replay_paths, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(payload).hexdigest()

    def renderable_world_matrices(self) -> NDArray[np.float64]:
        return self._world_matrices(self.renderable_paths)

    def _world_matrices(self, paths: tuple[str, ...]) -> NDArray[np.float64]:
        matrices = tuple(
            np.asarray(
                self._usd_geom.Xformable(self._stage.GetPrimAtPath(path))
                .ComputeLocalToWorldTransform(self._usd.TimeCode.Default()),
                dtype=np.float64,
            )
            for path in paths
        )
        result = np.stack(matrices)
        if result.shape != (len(paths), 4, 4) or not np.isfinite(result).all():
            raise RuntimeError("operator-preview renderable world matrices are invalid")
        return result

    def renderable_component_world_matrices(self) -> dict[str, NDArray[np.float64]]:
        return {
            component: self._world_matrices(paths)
            for component, paths in self.component_renderable_paths.items()
        }

    def component_source_poses(
        self,
        frame: SimulationStateFrame,
    ) -> dict[str, tuple[NDArray[np.float64], NDArray[np.float64]]]:
        items = tuple(frame.kinematic_links) + tuple(frame.rigid_bodies)
        by_path = {item.prim_path: item for item in items}
        if tuple(by_path) != self.paths:
            raise RuntimeError("operator-preview pose inventory changed within one run")
        return {
            component: (
                np.asarray([by_path[path].position_m for path in paths], dtype=np.float64),
                np.asarray([by_path[path].quat_wxyz for path in paths], dtype=np.float64),
            )
            for component, paths in self.component_source_paths.items()
        }

    @property
    def component_source_pose_counts(self) -> dict[str, int]:
        return {component: len(paths) for component, paths in self.component_source_paths.items()}

    @property
    def component_replay_pose_counts(self) -> dict[str, int]:
        return {component: len(paths) for component, paths in self.component_replay_paths.items()}

    @property
    def component_renderable_geometry_counts(self) -> dict[str, int]:
        return {
            component: len(paths)
            for component, paths in self.component_renderable_paths.items()
        }

    @property
    def apply_count(self) -> int:
        return self._apply_count

    def close(self) -> None:
        self._stage.SetEditTarget(self._previous_edit_target)
        session_layer = self._stage.GetSessionLayer()
        paths = list(session_layer.subLayerPaths)
        if self._layer.identifier in paths:
            paths.remove(self._layer.identifier)
            session_layer.subLayerPaths = paths


def _viewport_pixel_difference(
    baseline_rgb: NDArray[np.uint8],
    motion_rgb: NDArray[np.uint8],
) -> tuple[float, int, float]:
    """Measure material RGB change without treating sub-LSB renderer noise as motion."""

    if baseline_rgb.shape != motion_rgb.shape or baseline_rgb.ndim != 3:
        raise ValueError("live preview viewport captures have incompatible shapes")
    if baseline_rgb.shape[2] != 3:
        raise ValueError("live preview viewport captures must contain RGB pixels")
    delta = np.abs(baseline_rgb.astype(np.int16) - motion_rgb.astype(np.int16))
    changed = np.max(delta, axis=2) >= PREVIEW_VISIBLE_MOTION_PIXEL_DELTA
    return (
        float(np.mean(delta)),
        int(np.max(delta, initial=0)),
        float(np.mean(changed)),
    )


def _q54_group_ranges(
    minimum: NDArray[np.float64] | None,
    maximum: NDArray[np.float64] | None,
) -> dict[str, float]:
    if minimum is None or maximum is None:
        return {
            "left_arm_q7": 0.0,
            "left_hand_q20": 0.0,
            "right_arm_q7": 0.0,
            "right_hand_q20": 0.0,
        }
    ranges = maximum - minimum
    return {
        "left_arm_q7": float(np.max(ranges[0:7], initial=0.0)),
        "left_hand_q20": float(np.max(ranges[7:27], initial=0.0)),
        "right_arm_q7": float(np.max(ranges[27:34], initial=0.0)),
        "right_hand_q20": float(np.max(ranges[34:54], initial=0.0)),
    }


def _write_receipt(run_root: Path, payload: dict[str, object]) -> Path:
    output = run_root / "derived" / "live_preview"
    output.mkdir(parents=True, exist_ok=True)
    if output.is_symlink():
        raise ValueError("live preview output must not be a symbolic link")
    destination = output / "receipt.json"
    temporary = output / ".receipt.json.tmp"
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    return destination


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.run_id.strip() or args.run_id != args.run_id.strip():
        raise SystemExit("--run-id must be non-blank and trimmed")
    process_affinity = configure_current_process_cpu_affinity(args.cpu_affinity)
    thread_limit = min(
        PREVIEW_THREAD_LIMIT_CAP,
        len(process_affinity) if process_affinity is not None else PREVIEW_THREAD_LIMIT_CAP,
    )
    resolved = RosDeploymentResolver(ROOT).resolve(
        args.deployment,
        local_binding=args.local_runtime_binding,
        verify_artifacts=args.verify_artifacts,
    )
    hand_revisions = {
        instance.asset.revision
        for instance in resolved.session.instances
        if instance.asset.product == "wuji_hand_2"
    }
    requires_matched_chain = hand_revisions == {"beta1_description_v2026_8_3"}
    if requires_matched_chain != (args.chain_preflight is not None):
        raise SystemExit(
            "Description 8.3 preview requires --chain-preflight and historical entries forbid it"
        )
    chain_preflight: Mapping[str, object] | None = None
    workcell_plan = None
    if args.chain_preflight is not None:
        try:
            chain_preflight = load_record_chain_preflight_receipt(
                args.chain_preflight
            )
            workcell_plan = resolve_record_chain_workcell_plan(
                ROOT,
                resolved,
                chain_preflight,
                verify_content=args.verify_artifacts,
            )
        except (FileNotFoundError, ValueError) as exc:
            raise SystemExit(f"record-chain preview preflight failed: {exc}") from exc
    profile_ref = resolved.session.session.dataset_profile
    if profile_ref is None:
        raise SystemExit("live preview requires a dataset Session")
    dataset_profile = load_mini_dataset_profile(ROOT, profile_ref.path)
    if dataset_profile.profile_id != profile_ref.expected_id:
        raise SystemExit("live preview dataset profile identity differs")
    sides = resolve_dual_side_runtimes(ROOT, resolved.session)
    sides_by_name = {runtime.side: runtime for runtime in sides}
    component_prefixes = {
        "left_arm": sides_by_name["left"].arm_prim_path,
        "left_hand": sides_by_name["left"].hand_prim_path,
        "right_arm": sides_by_name["right"].arm_prim_path,
        "right_hand": sides_by_name["right"].hand_prim_path,
    }
    hand_instances = {
        side: resolved.session.instance(sides_by_name[side].hand_instance_id)
        for side in ("left", "right")
    }
    arm_instances = {
        side: resolved.session.instance(sides_by_name[side].arm_instance_id)
        for side in ("left", "right")
    }
    expected_component_source_pose_counts = {
        "left_arm": _expected_arm_source_pose_count(
            asset_revision=arm_instances["left"].asset.revision,
            backend_tool_frame=arm_instances["left"].binding.backend_frame(
                arm_instances["left"].asset.frame_name("tool_flange")
            ),
        ),
        "left_hand": _expected_hand_source_pose_count(
            asset_revision=hand_instances["left"].asset.revision,
            side="left",
            backend_base_frame=hand_instances["left"].binding.backend_frame(
                hand_instances["left"].asset.frame_name("base")
            ),
        ),
        "right_arm": _expected_arm_source_pose_count(
            asset_revision=arm_instances["right"].asset.revision,
            backend_tool_frame=arm_instances["right"].binding.backend_frame(
                arm_instances["right"].asset.frame_name("tool_flange")
            ),
        ),
        "right_hand": _expected_hand_source_pose_count(
            asset_revision=hand_instances["right"].asset.revision,
            side="right",
            backend_base_frame=hand_instances["right"].binding.backend_frame(
                hand_instances["right"].asset.frame_name("base")
            ),
        ),
    }
    alignment_references = {
        resolved.session.instance(runtime.arm_instance_id).binding.compatibility_profile
        for runtime in sides
    }
    if None in alignment_references or len(alignment_references) != 1:
        raise SystemExit("live preview requires one NERO geometry profile")
    alignment_path = ROOT / str(next(iter(alignment_references)))
    alignment = load_nero_link_geometry_alignment(alignment_path)
    source_urdf = (ROOT / alignment.source_urdf_path).resolve()
    if sha256_file(source_urdf) != alignment.source_urdf_sha256:
        raise SystemExit("live preview source-locked NERO URDF hash drifted")
    qualification = load_nero_dual_simulation_startup_profile(
        ROOT / resolved.control_profile.base_qualification.path
    )

    if args.wait_for_node:
        print(
            f"DATASET LIVE PREVIEW WAITING: node={args.wait_for_node}",
            flush=True,
        )
        _wait_for_ros_node(
            args.wait_for_node,
            timeout_s=PREVIEW_CONSUMER_WAIT_TIMEOUT_S,
        )
        print(
            f"DATASET LIVE PREVIEW RELEASED: node={args.wait_for_node}",
            flush=True,
        )

    from isaacsim import SimulationApp  # type: ignore[import-not-found]

    simulation_app: Any = SimulationApp(
        {
            "headless": False,
            "width": PREVIEW_WIDTH,
            "height": PREVIEW_HEIGHT,
            "anti_aliasing": 0,
            "renderer": PREVIEW_RENDERER,
            "minimal_shading_mode": PREVIEW_MINIMAL_SHADING_MODE,
            "multi_gpu": False,
            "limit_cpu_threads": thread_limit,
            "disable_viewport_updates": False,
            "extra_args": [
                "--/rtx/hydra/supportMultiTickRate="
                f"{'true' if PREVIEW_MULTI_TICK_RENDERING else 'false'}"
            ],
        }
    )

    import carb  # type: ignore[import-not-found]
    import omni.kit.renderer_capture  # type: ignore[import-not-found]
    import omni.replicator.core as rep  # type: ignore[import-not-found]
    import omni.timeline  # type: ignore[import-not-found]
    import rclpy  # type: ignore[import-not-found]
    from rclpy.executors import SingleThreadedExecutor  # type: ignore[import-not-found]
    from rclpy.node import Node  # type: ignore[import-not-found]
    from rclpy.signals import SignalHandlerOptions  # type: ignore[import-not-found]
    from isaacsim.core.experimental.prims import (  # type: ignore[import-not-found]
        XformPrim,
    )
    from isaacsim.core.experimental.utils.backend import (  # type: ignore[import-not-found]
        use_backend,
    )
    from isaacsim.core.rendering_manager import (  # type: ignore[import-not-found]
        RenderingManager,
        ViewportManager,
    )
    from isaacsim.core.simulation_manager import (  # type: ignore[import-not-found]
        SimulationManager,
    )
    from isaacsim.core.utils.viewports import set_camera_view  # type: ignore[import-not-found]
    from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
        capture_viewport_to_buffer,
        get_active_viewport,
    )
    from pxr import Sdf, Usd, UsdGeom  # type: ignore[import-not-found]
    from wujihand_interfaces.msg import (  # type: ignore[import-not-found]
        OperatorPreviewStateFrame as OperatorPreviewStateFrameMessage,
        RunRecordingStatus as RunRecordingStatusMessage,
    )
    from wujihand.runtime.isaac_dual_scene import DualNeroHand2IsaacScene
    from wujihand_ros2.conversion import (
        run_recording_status_from_message,
        simulation_state_frame_from_message,
    )
    from wujihand_ros2.executor_thread import RosExecutorThread
    from wujihand_ros2.qos import qos_profile

    scene: Any | None = None
    node: Any | None = None
    executor_worker: Any | None = None
    primary_error: BaseException | None = None
    receipt_path: Path | None = None
    terminal_state: str | None = None
    foreign_run_messages = 0
    source_frames_received = 0
    source_frames_applied = 0
    source_kinematic_link_count = 0
    rendered_active_frames = 0
    missed_render_periods = 0
    render_total_ns = 0
    render_max_ns = 0
    slow_render_events: list[dict[str, int | bool | None]] = []
    pose_apply_total_ns = 0
    pose_apply_max_ns = 0
    warmup_render_count = 0
    warmup_render_max_ns = 0
    warmup_tail_max_ns = 0
    pose_position_max_error = 0.0
    pose_closure_checks = 0
    source_control_indices: list[int] = []
    render_completion_ns: list[int] = []
    applied_q54_min: NDArray[np.float64] | None = None
    applied_q54_max: NDArray[np.float64] | None = None
    previous_applied_q54: NDArray[np.float64] | None = None
    applied_q54_changed_frames = 0
    viewport_capture_helpers: list[Any] = []
    viewport_reference_frame: SimulationStateFrame | None = None
    viewport_motion_frame: SimulationStateFrame | None = None
    viewport_return_frame: SimulationStateFrame | None = None
    viewport_baseline_q54: NDArray[np.float64] | None = None
    viewport_baseline_control_index: int | None = None
    viewport_motion_control_index: int | None = None
    viewport_return_control_index: int | None = None
    viewport_motion_trigger_rad = 0.0
    viewport_baseline_sha256: str | None = None
    viewport_motion_sha256: str | None = None
    viewport_return_sha256: str | None = None
    viewport_repeat_sha256: str | None = None
    viewport_pixel_mean_abs_delta: float | None = None
    viewport_pixel_max_abs_delta: int | None = None
    viewport_pixel_changed_fraction: float | None = None
    viewport_repeat_pixel_mean_abs_delta: float | None = None
    viewport_repeat_pixel_max_abs_delta: int | None = None
    viewport_repeat_pixel_changed_fraction: float | None = None
    viewport_capture_frame_numbers: dict[str, int] = {}
    viewport_visible_motion_required = False
    viewport_visible_motion_passed = False
    viewport_static_repeat_passed = False
    renderable_geometry_motion_passed = False
    renderable_geometry_matrix_max_delta = 0.0
    component_reference_source_poses: (
        dict[str, tuple[NDArray[np.float64], NDArray[np.float64]]] | None
    ) = None
    component_source_position_max_delta_m = {
        component: 0.0 for component in PREVIEW_COMPONENTS
    }
    component_source_orientation_max_delta_rad = {
        component: 0.0 for component in PREVIEW_COMPONENTS
    }
    component_motion_scores = {component: 0.0 for component in PREVIEW_COMPONENTS}
    component_motion_frames: dict[str, SimulationStateFrame | None] = {
        component: None for component in PREVIEW_COMPONENTS
    }
    component_source_motion_required = {
        component: False for component in PREVIEW_COMPONENTS
    }
    component_renderable_matrix_max_delta = {
        component: 0.0 for component in PREVIEW_COMPONENTS
    }
    component_renderable_motion_passed = {
        component: False for component in PREVIEW_COMPONENTS
    }
    qa_pose_apply_count = 0
    source_pose_position_max_delta_m = 0.0
    source_pose_orientation_max_delta = 0.0
    source_q54_group_max_range_rad = _q54_group_ranges(None, None)
    source_q54_max_range_rad = 0.0
    viewport_capture_root = args.run_root.resolve() / "derived" / "live_preview"
    viewport_baseline_path = viewport_capture_root / "viewport_baseline.png"
    viewport_motion_path = viewport_capture_root / "viewport_motion.png"
    viewport_return_path = viewport_capture_root / "viewport_return.png"
    viewport_repeat_path = viewport_capture_root / "viewport_repeat.png"
    baseline_capture_complete = False
    motion_capture_complete = False
    return_capture_complete = False
    repeat_capture_complete = False
    latest_lock = Lock()
    latest_frame: SimulationStateFrame | None = None
    callback_failure: BaseException | None = None
    shutdown_signal: int | None = None
    shutdown_requested_ns: int | None = None
    physics_step_anchor: int | None = None
    simulation_time_anchor: float | None = None
    python_gc_frozen = False
    python_gc_frozen_object_count = 0
    python_gc_unfrozen_on_close = False
    timeline_playing_before_replay_stop: bool | None = None
    timeline_playing_after_replay_stop: bool | None = None
    timeline_stopped_for_replay: bool | None = None
    simulation_manager_physics_view_active: bool | None = None
    world_physics_view_active: bool | None = None
    local_physics_simulating: bool | None = None
    pose_writer: _PoseReplayWriter | None = None
    viewport_sync_annotator: Any | None = None
    viewport_render_product_path: str | None = None
    multi_tick_rendering_enabled = carb.settings.get_settings().get_as_bool(
        "/rtx/hydra/supportMultiTickRate"
    )
    background_color_readback: tuple[float, float, float] | None = None
    passed = False
    acceptance_failures: list[str] = []

    def request_stop(signum: int, frame: object) -> None:
        del frame
        nonlocal shutdown_signal, shutdown_requested_ns
        if shutdown_signal is None:
            shutdown_signal = signum
            shutdown_requested_ns = time.monotonic_ns()

    previous_handlers = {
        current: signal.signal(current, request_stop) for current in (signal.SIGINT, signal.SIGTERM)
    }

    try:
        scene = DualNeroHand2IsaacScene(
            project_root=ROOT,
            resolved=resolved.session,
            sides=sides,
            alignment_profile=alignment,
            qualification_profile=qualification,
            physics_hz=resolved.control_profile.physics_hz,
            self_collision_sides=frozenset(),
            wrist_rig_collision_mode="all",
            visual_replay_only=True,
            workcell_plan=workcell_plan,
        )
        preview_settings = carb.settings.get_settings()
        preview_settings.set_float_array(
            "/rtx/background/source/color",
            list(PREVIEW_BACKGROUND_COLOR_RGB),
        )
        background_color_readback = tuple(
            float(value)
            for value in preview_settings.get("/rtx/background/source/color")
        )
        if not np.allclose(
            background_color_readback,
            PREVIEW_BACKGROUND_COLOR_RGB,
            rtol=0.0,
            atol=1e-6,
        ):
            raise RuntimeError("operator preview background override was not applied")
        if not scene.visual_replay_only or scene.articulations:
            raise RuntimeError("live preview materialized an active articulation runtime")
        simulation_manager_physics_view_active = (
            SimulationManager.get_physics_simulation_view() is not None
        )
        world_physics_view_active = getattr(scene.world, "_physics_sim_view", None) is not None
        local_physics_simulating = bool(SimulationManager.is_simulating())
        if (
            simulation_manager_physics_view_active
            or world_physics_view_active
            or local_physics_simulating
        ):
            raise RuntimeError("live preview visual scene initialized a local physics runtime")
        set_camera_view(
            eye=np.asarray(
                workcell_frame_position(resolved.session, SCENE_CAMERA_EYE_FRAME),
                dtype=np.float64,
            ),
            target=np.asarray(
                workcell_frame_position(resolved.session, SCENE_CAMERA_TARGET_FRAME),
                dtype=np.float64,
            ),
            camera_prim_path=SCENE_CAMERA_PRIM_PATH,
        )
        timeline = omni.timeline.get_timeline_interface()
        timeline_playing_before_replay_stop = bool(timeline.is_playing())
        timeline.stop()
        RenderingManager.render()
        timeline_playing_after_replay_stop = bool(timeline.is_playing())
        timeline_stopped_for_replay = bool(timeline.is_stopped())
        if timeline_playing_after_replay_stop or not timeline_stopped_for_replay:
            raise RuntimeError("live preview replay requires a stopped Kit timeline")
        physics_step_anchor = int(scene.world.current_time_step_index)
        simulation_time_anchor = float(scene.world.current_time)

        viewport_api = get_active_viewport()
        if viewport_api is None:
            raise RuntimeError("live preview active viewport is unavailable")
        viewport_render_product = ViewportManager.get_render_product(viewport_api)
        if not viewport_render_product or not viewport_render_product.GetPrim().IsValid():
            raise RuntimeError("live preview active viewport render product is unavailable")
        viewport_render_product_path = str(viewport_render_product.GetPath())
        viewport_sync_annotator = rep.AnnotatorRegistry.get_annotator(
            PREVIEW_SYNC_ANNOTATOR,
        )
        viewport_sync_annotator.attach(viewport_render_product_path)

        def render_preview_transaction() -> None:
            rep.orchestrator.step(
                rt_subframes=0,
                pause_timeline=False,
                delta_time=0.0,
                wait_for_render=True,
            )

        warmup_tail_ns: list[int] = []
        for _ in range(PREVIEW_WARMUP_MAX_RENDERS):
            warmup_started_ns = time.monotonic_ns()
            render_preview_transaction()
            warmup_duration_ns = time.monotonic_ns() - warmup_started_ns
            warmup_render_count += 1
            warmup_render_max_ns = max(warmup_render_max_ns, warmup_duration_ns)
            warmup_tail_ns.append(warmup_duration_ns)
            del warmup_tail_ns[:-PREVIEW_WARMUP_STABLE_RENDERS]
            warmup_tail_max_ns = max(warmup_tail_ns)
            if int(scene.world.current_time_step_index) != physics_step_anchor:
                raise RuntimeError("live preview warm-up advanced the physics step index")
            if not math.isclose(
                float(scene.world.current_time),
                simulation_time_anchor,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise RuntimeError("live preview warm-up advanced simulation time")
            if (
                warmup_render_count >= PREVIEW_WARMUP_MIN_RENDERS
                and len(warmup_tail_ns) == PREVIEW_WARMUP_STABLE_RENDERS
                and warmup_tail_max_ns <= PREVIEW_WARMUP_RENDER_BUDGET_NS
            ):
                break
        else:
            raise RuntimeError("live preview render did not stabilize below the warm-up budget")

        viewport_capture_root.mkdir(parents=True, exist_ok=True)

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node(
            "dataset_live_preview",
            namespace=f"/{resolved.deployment.root_namespace}",
        )

        def receive_state(message: Any) -> None:
            nonlocal latest_frame, callback_failure, foreign_run_messages
            nonlocal source_frames_received
            try:
                frame = simulation_state_frame_from_message(message)
                if frame.run_id != args.run_id:
                    foreign_run_messages += 1
                    return
                if frame.phase is not SimulationFramePhase.POST_ACTION:
                    return
                with latest_lock:
                    if latest_frame is not None and (
                        frame.control_index <= latest_frame.control_index
                    ):
                        raise RuntimeError(
                            "live preview post-state control index is not increasing"
                        )
                    latest_frame = frame
                    source_frames_received += 1
            except BaseException as exc:
                callback_failure = exc
                raise

        def receive_status(message: Any) -> None:
            nonlocal terminal_state, callback_failure, foreign_run_messages
            try:
                status = run_recording_status_from_message(message)
                if status.run_id != args.run_id:
                    foreign_run_messages += 1
                    return
                if status.state in {
                    RunRecordingState.CONSUMER_COMPLETED,
                    RunRecordingState.INCOMPLETE,
                }:
                    terminal_state = status.state.value
            except BaseException as exc:
                callback_failure = exc
                raise

        subscriptions = (
            node.create_subscription(
                OperatorPreviewStateFrameMessage,
                OPERATOR_PREVIEW_STATE_TOPIC,
                receive_state,
                qos_profile(resolved.qos_profile.policy("dataset_state")),
            ),
            node.create_subscription(
                RunRecordingStatusMessage,
                "recording/status",
                receive_status,
                qos_profile(resolved.qos_profile.policy("run_status")),
            ),
        )
        del subscriptions
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor_worker = RosExecutorThread(
            executor,
            name="wujihand-dataset-live-preview-ros",
        )
        gc.collect()
        gc.freeze()
        python_gc_frozen = True
        python_gc_frozen_object_count = gc.get_freeze_count()
        executor_worker.start()
        print(
            f"DATASET LIVE PREVIEW READY: run_id={args.run_id} rate_hz={PREVIEW_HZ} passive=true",
            flush=True,
        )

        scheduler = FixedRateScheduler(
            rate_hz=PREVIEW_HZ,
            start_ns=time.monotonic_ns(),
            maximum_catch_up_ticks=PREVIEW_MAXIMUM_CATCH_UP_TICKS,
        )
        applied_control_index: int | None = None
        while simulation_app.is_running():
            executor_worker.raise_if_failed()
            if callback_failure is not None:
                raise RuntimeError("live preview ROS callback failed") from callback_failure
            if terminal_state is not None:
                break
            if shutdown_requested_ns is not None and (
                time.monotonic_ns() - shutdown_requested_ns
                >= round(PREVIEW_SHUTDOWN_GRACE_S * 1_000_000_000)
            ):
                break
            scheduled = scheduler.wait_next()
            missed_render_periods += scheduled.missed_periods_before_tick
            with latest_lock:
                current = latest_frame
            queued_frame: SimulationStateFrame | None = None
            if current is not None and current.control_index != applied_control_index:
                current_q54 = np.asarray(current.q54_rad, dtype=np.float64)
                if source_kinematic_link_count == 0:
                    source_kinematic_link_count = len(current.kinematic_links)
                elif len(current.kinematic_links) != source_kinematic_link_count:
                    raise RuntimeError("operator preview link inventory count changed")
                if pose_writer is None:
                    pose_writer = _PoseReplayWriter(
                        stage=scene.stage,
                        paths=_preview_pose_paths(current),
                        component_prefixes=component_prefixes,
                        expected_source_pose_counts=(
                            expected_component_source_pose_counts
                        ),
                        sdf=Sdf,
                        usd=Usd,
                        usd_geom=UsdGeom,
                        xform_prim_type=XformPrim,
                        backend_context=use_backend,
                    )
                pose_apply_started_ns = time.monotonic_ns()
                pose_writer.apply(current)
                pose_apply_duration_ns = time.monotonic_ns() - pose_apply_started_ns
                pose_apply_total_ns += pose_apply_duration_ns
                pose_apply_max_ns = max(pose_apply_max_ns, pose_apply_duration_ns)
                queued_frame = current

            render_started_ns = time.monotonic_ns()
            render_preview_transaction()
            render_duration_ns = time.monotonic_ns() - render_started_ns
            if render_duration_ns > PREVIEW_WARMUP_RENDER_BUDGET_NS and len(
                slow_render_events
            ) < 16:
                slow_render_events.append(
                    {
                        "schedule_slot": scheduled.schedule_slot,
                        "source_control_index": (
                            None if current is None else current.control_index
                        ),
                        "queued_new_source_frame": queued_frame is not None,
                        "render_duration_ns": render_duration_ns,
                    }
                )
            if queued_frame is not None:
                assert pose_writer is not None
                current_q54 = np.asarray(queued_frame.q54_rad, dtype=np.float64)
                if source_frames_applied == 0:
                    current_pose_error = pose_writer.position_max_error(queued_frame)
                    pose_closure_checks += 1
                    pose_position_max_error = max(
                        pose_position_max_error,
                        current_pose_error,
                    )
                    if current_pose_error >= STATE_CLOSURE_LIMIT:
                        raise RuntimeError(
                            "live preview initial pose replay did not close below 2e-5"
                        )
                applied_control_index = queued_frame.control_index
                source_control_indices.append(queued_frame.control_index)
                source_frames_applied += 1
                if applied_q54_min is None:
                    applied_q54_min = current_q54.copy()
                    applied_q54_max = current_q54.copy()
                else:
                    assert applied_q54_max is not None
                    applied_q54_min = np.minimum(applied_q54_min, current_q54)
                    applied_q54_max = np.maximum(applied_q54_max, current_q54)
                if previous_applied_q54 is not None and not np.array_equal(
                    previous_applied_q54,
                    current_q54,
                ):
                    applied_q54_changed_frames += 1
                previous_applied_q54 = current_q54.copy()
                if (
                    viewport_reference_frame is None
                    and source_frames_applied >= PREVIEW_REFERENCE_WARMUP_APPLIED_FRAMES
                ):
                    viewport_reference_frame = queued_frame
                    viewport_baseline_q54 = current_q54.copy()
                    viewport_baseline_control_index = queued_frame.control_index
                    component_reference_source_poses = (
                        pose_writer.component_source_poses(queued_frame)
                    )
                elif viewport_baseline_q54 is not None:
                    assert component_reference_source_poses is not None
                    current_component_poses = pose_writer.component_source_poses(
                        queued_frame
                    )
                    for component in PREVIEW_COMPONENTS:
                        position_delta, orientation_delta = _pose_group_delta(
                            component_reference_source_poses[component],
                            current_component_poses[component],
                        )
                        component_source_position_max_delta_m[component] = max(
                            component_source_position_max_delta_m[component],
                            position_delta,
                        )
                        component_source_orientation_max_delta_rad[component] = max(
                            component_source_orientation_max_delta_rad[component],
                            orientation_delta,
                        )
                        score = max(
                            position_delta / PREVIEW_COMPONENT_SOURCE_POSITION_DELTA_M,
                            orientation_delta
                            / PREVIEW_COMPONENT_SOURCE_ORIENTATION_DELTA_RAD,
                        )
                        if score > component_motion_scores[component]:
                            component_motion_scores[component] = score
                            component_motion_frames[component] = queued_frame
                    visible_delta = float(
                        np.max(np.abs(current_q54 - viewport_baseline_q54), initial=0.0)
                    )
                    if visible_delta > viewport_motion_trigger_rad:
                        viewport_motion_trigger_rad = visible_delta
                        viewport_motion_frame = queued_frame
                        viewport_motion_control_index = queued_frame.control_index
                    viewport_return_frame = queued_frame
                    viewport_return_control_index = queued_frame.control_index
            if applied_control_index is not None:
                rendered_active_frames += 1
                render_total_ns += render_duration_ns
                render_max_ns = max(render_max_ns, render_duration_ns)
                render_completion_ns.append(time.monotonic_ns())
            if int(scene.world.current_time_step_index) != physics_step_anchor:
                raise RuntimeError("live preview advanced the physics step index")
            if not math.isclose(
                float(scene.world.current_time),
                simulation_time_anchor,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise RuntimeError("live preview advanced simulation time")
            scheduler.complete(completed_ns=time.monotonic_ns())

        if viewport_reference_frame is None or viewport_return_frame is None:
            raise RuntimeError("live preview did not retain enough source frames for viewport QA")

        def capture_injected_state(
            frame: SimulationStateFrame,
            paths: tuple[Path, ...],
        ) -> tuple[
            int,
            tuple[NDArray[np.uint8], ...],
            NDArray[np.float64],
            dict[str, NDArray[np.float64]],
        ]:
            nonlocal pose_closure_checks, pose_position_max_error, qa_pose_apply_count
            if not paths or any(path.exists() for path in paths):
                raise RuntimeError(f"invalid live preview capture paths: {paths}")
            assert pose_writer is not None
            pose_writer.apply(frame)
            qa_pose_apply_count += 1

            captured: list[NDArray[np.uint8] | None] = [None] * len(paths)

            def callback_for(index: int) -> Any:
                def receive_pixels(
                    buffer: object,
                    buffer_size: int,
                    width: int,
                    height: int,
                    pixel_format: object,
                ) -> None:
                    raw = omni.kit.renderer_capture.convert_raw_bytes_to_list(
                        buffer,
                        buffer_size,
                        width,
                        height,
                        pixel_format,
                    )
                    rgba = np.asarray(raw, dtype=np.uint8).reshape((height, width, 4))
                    if (width, height) != (PREVIEW_WIDTH, PREVIEW_HEIGHT):
                        raise RuntimeError(f"unexpected viewport capture size: {width}x{height}")
                    captured[index] = rgba[:, :, :3].copy()

                return receive_pixels

            helpers = [
                capture_viewport_to_buffer(viewport_api, callback_for(index), is_hdr=False)
                for index in range(len(paths))
            ]
            viewport_capture_helpers.extend(helpers)
            render_preview_transaction()
            capture_pose_error = pose_writer.position_max_error(frame)
            pose_closure_checks += 1
            if capture_pose_error >= STATE_CLOSURE_LIMIT:
                raise RuntimeError("deterministic viewport pose replay did not close")
            omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
            if any(image is None for image in captured):
                raise RuntimeError("live preview viewport buffer capture did not complete")
            frame_numbers = tuple(int(helper.frame_number) for helper in helpers)
            if len(set(frame_numbers)) != 1:
                raise RuntimeError(
                    f"one capture transaction crossed viewport frames: {frame_numbers}"
                )
            from PIL import Image  # type: ignore[import-not-found]

            completed = tuple(image for image in captured if image is not None)
            for path, image in zip(paths, completed, strict=True):
                Image.fromarray(image, mode="RGB").save(path)
            pose_position_max_error = max(
                pose_position_max_error,
                capture_pose_error,
            )
            if int(scene.world.current_time_step_index) != physics_step_anchor:
                raise RuntimeError("viewport capture advanced the physics step index")
            if not math.isclose(
                float(scene.world.current_time),
                simulation_time_anchor,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise RuntimeError("viewport capture advanced simulation time")
            return (
                frame_numbers[0],
                completed,
                pose_writer.renderable_world_matrices(),
                pose_writer.renderable_component_world_matrices(),
            )

        def measure_injected_component_matrices(
            frame: SimulationStateFrame,
        ) -> dict[str, NDArray[np.float64]]:
            nonlocal pose_closure_checks, pose_position_max_error, qa_pose_apply_count
            assert pose_writer is not None
            pose_writer.apply(frame)
            qa_pose_apply_count += 1
            render_preview_transaction()
            current_error = pose_writer.position_max_error(frame)
            pose_closure_checks += 1
            pose_position_max_error = max(pose_position_max_error, current_error)
            if current_error >= STATE_CLOSURE_LIMIT:
                raise RuntimeError("component pose replay did not close")
            if int(scene.world.current_time_step_index) != physics_step_anchor:
                raise RuntimeError("component pose QA advanced the physics step index")
            if not math.isclose(
                float(scene.world.current_time),
                simulation_time_anchor,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise RuntimeError("component pose QA advanced simulation time")
            return pose_writer.renderable_component_world_matrices()

        (
            baseline_frame_number,
            _,
            baseline_renderable_matrices,
            baseline_component_matrices,
        ) = capture_injected_state(viewport_reference_frame, (viewport_baseline_path,))
        viewport_capture_frame_numbers["baseline"] = baseline_frame_number
        if (
            viewport_motion_frame is not None
            and viewport_motion_trigger_rad >= PREVIEW_VISIBLE_MOTION_TRIGGER_RAD
        ):
            (
                motion_frame_number,
                _,
                motion_renderable_matrices,
                motion_component_matrices,
            ) = capture_injected_state(viewport_motion_frame, (viewport_motion_path,))
            viewport_capture_frame_numbers["motion"] = motion_frame_number
        else:
            motion_renderable_matrices = baseline_renderable_matrices.copy()
            motion_component_matrices = {
                component: matrices.copy()
                for component, matrices in baseline_component_matrices.items()
            }

        component_matrix_cache: dict[int, dict[str, NDArray[np.float64]]] = {}
        if viewport_motion_frame is not None:
            component_matrix_cache[viewport_motion_frame.control_index] = (
                motion_component_matrices
            )
        for component in PREVIEW_COMPONENTS:
            required = bool(
                component_source_position_max_delta_m[component]
                >= PREVIEW_COMPONENT_SOURCE_POSITION_DELTA_M
                or component_source_orientation_max_delta_rad[component]
                >= PREVIEW_COMPONENT_SOURCE_ORIENTATION_DELTA_RAD
            )
            component_source_motion_required[component] = required
            motion_frame = component_motion_frames[component]
            if not required:
                component_renderable_motion_passed[component] = True
                continue
            if motion_frame is None:
                raise RuntimeError(f"missing {component} source-motion QA frame")
            matrices_by_component = component_matrix_cache.get(motion_frame.control_index)
            if matrices_by_component is None:
                matrices_by_component = measure_injected_component_matrices(motion_frame)
                component_matrix_cache[motion_frame.control_index] = matrices_by_component
            matrix_delta = _maximum_abs_error(
                baseline_component_matrices[component],
                matrices_by_component[component],
            )
            component_renderable_matrix_max_delta[component] = matrix_delta
            component_renderable_motion_passed[component] = bool(
                matrix_delta >= PREVIEW_RENDERABLE_GEOMETRY_MATRIX_DELTA
            )

        return_frame_number, _, _, _ = capture_injected_state(
            viewport_return_frame,
            (viewport_return_path, viewport_repeat_path),
        )
        viewport_capture_frame_numbers["return"] = return_frame_number
        viewport_capture_frame_numbers["repeat"] = return_frame_number
        capture_reference_order_closed = bool(
            set(viewport_capture_frame_numbers) == {"baseline", "motion", "return", "repeat"}
            and viewport_capture_frame_numbers["baseline"]
            < viewport_capture_frame_numbers["motion"]
            < viewport_capture_frame_numbers["return"]
            == viewport_capture_frame_numbers["repeat"]
        )

        source_q54_group_max_range_rad = _q54_group_ranges(
            applied_q54_min,
            applied_q54_max,
        )
        source_q54_max_range_rad = max(source_q54_group_max_range_rad.values(), default=0.0)
        viewport_visible_motion_required = (
            source_q54_max_range_rad >= PREVIEW_VISIBLE_MOTION_Q54_THRESHOLD_RAD
        )
        if viewport_motion_frame is not None:
            baseline_items = tuple(viewport_reference_frame.kinematic_links) + tuple(
                viewport_reference_frame.rigid_bodies
            )
            motion_items = tuple(viewport_motion_frame.kinematic_links) + tuple(
                viewport_motion_frame.rigid_bodies
            )
            source_pose_position_max_delta_m = _maximum_abs_error(
                [item.position_m for item in baseline_items],
                [item.position_m for item in motion_items],
            )
            source_pose_orientation_max_delta = _maximum_abs_error(
                [item.quat_wxyz for item in baseline_items],
                [item.quat_wxyz for item in motion_items],
            )
        renderable_geometry_matrix_max_delta = _maximum_abs_error(
            baseline_renderable_matrices,
            motion_renderable_matrices,
        )
        renderable_geometry_motion_passed = bool(
            not viewport_visible_motion_required
            or renderable_geometry_matrix_max_delta
            >= PREVIEW_RENDERABLE_GEOMETRY_MATRIX_DELTA
        )
        baseline_capture_complete = (
            viewport_baseline_path.is_file() and viewport_baseline_path.stat().st_size > 0
        )
        motion_capture_complete = (
            viewport_motion_path.is_file() and viewport_motion_path.stat().st_size > 0
        )
        return_capture_complete = (
            viewport_return_path.is_file() and viewport_return_path.stat().st_size > 0
        )
        repeat_capture_complete = (
            viewport_repeat_path.is_file() and viewport_repeat_path.stat().st_size > 0
        )
        if baseline_capture_complete:
            viewport_baseline_sha256 = sha256_file(viewport_baseline_path)
        if motion_capture_complete:
            viewport_motion_sha256 = sha256_file(viewport_motion_path)
        if return_capture_complete:
            viewport_return_sha256 = sha256_file(viewport_return_path)
        if repeat_capture_complete:
            viewport_repeat_sha256 = sha256_file(viewport_repeat_path)
        if baseline_capture_complete and motion_capture_complete:
            from PIL import Image  # type: ignore[import-not-found]

            with Image.open(viewport_baseline_path) as baseline_image:
                baseline_rgb = np.asarray(baseline_image.convert("RGB"), dtype=np.uint8)
            with Image.open(viewport_motion_path) as motion_image:
                motion_rgb = np.asarray(motion_image.convert("RGB"), dtype=np.uint8)
            (
                viewport_pixel_mean_abs_delta,
                viewport_pixel_max_abs_delta,
                viewport_pixel_changed_fraction,
            ) = _viewport_pixel_difference(baseline_rgb, motion_rgb)
        if return_capture_complete and repeat_capture_complete:
            from PIL import Image  # type: ignore[import-not-found]

            with Image.open(viewport_return_path) as return_image:
                return_rgb = np.asarray(return_image.convert("RGB"), dtype=np.uint8)
            with Image.open(viewport_repeat_path) as repeat_image:
                repeat_rgb = np.asarray(repeat_image.convert("RGB"), dtype=np.uint8)
            (
                viewport_repeat_pixel_mean_abs_delta,
                viewport_repeat_pixel_max_abs_delta,
                viewport_repeat_pixel_changed_fraction,
            ) = _viewport_pixel_difference(return_rgb, repeat_rgb)
        viewport_visible_motion_passed = bool(
            not viewport_visible_motion_required
            or (
                viewport_pixel_changed_fraction is not None
                and viewport_pixel_changed_fraction >= PREVIEW_VISIBLE_MOTION_MIN_CHANGED_FRACTION
            )
        )
        viewport_static_repeat_passed = bool(
            return_capture_complete
            and repeat_capture_complete
            and viewport_return_sha256 == viewport_repeat_sha256
            and viewport_repeat_pixel_max_abs_delta == 0
        )

        if len(render_completion_ns) >= 2:
            render_effective_hz = (len(render_completion_ns) - 1) / (
                (render_completion_ns[-1] - render_completion_ns[0]) / 1_000_000_000
            )
        else:
            render_effective_hz = 0.0
        simulation_manager_physics_view_active = bool(
            simulation_manager_physics_view_active
            or SimulationManager.get_physics_simulation_view() is not None
        )
        world_physics_view_active = bool(
            world_physics_view_active
            or getattr(scene.world, "_physics_sim_view", None) is not None
        )
        local_physics_simulating = bool(
            local_physics_simulating or SimulationManager.is_simulating()
        )
        acceptance = {
            "consumer_terminal_completed": (
                terminal_state == RunRecordingState.CONSUMER_COMPLETED.value
            ),
            "source_state_applied": source_frames_applied > 0,
            "active_frames_rendered": rendered_active_frames >= 2,
            "render_schedule_closed": missed_render_periods == 0,
            "render_under_50_ms": render_max_ns < PREVIEW_RENDER_BUDGET_NS,
            "effective_render_rate": (abs(render_effective_hz - PREVIEW_HZ) / PREVIEW_HZ <= 0.05),
            "complete_link_inventory": source_kinematic_link_count > 14,
            "visual_only_scene": (
                not simulation_manager_physics_view_active
                and not world_physics_view_active
                and not local_physics_simulating
            ),
            "synchronous_pre_render_pose_application": (
                pose_writer is not None
                and pose_writer.apply_count == source_frames_applied + qa_pose_apply_count
            ),
            "pose_replay_closed": pose_position_max_error < STATE_CLOSURE_LIMIT,
            "component_replay_inventory": (
                pose_writer is not None
                and pose_writer.component_source_pose_counts
                == expected_component_source_pose_counts
                and all(
                    pose_writer.component_replay_pose_counts[component]
                    == pose_writer.component_source_pose_counts[component]
                    for component in ("left_arm", "right_arm")
                )
                and all(
                    pose_writer.component_renderable_geometry_counts[component]
                    == pose_writer.component_replay_pose_counts[component]
                    > 0
                    for component in PREVIEW_COMPONENTS
                )
            ),
            "component_renderable_motion": all(
                component_renderable_motion_passed.values()
            ),
            "renderable_geometry_motion": renderable_geometry_motion_passed,
            "viewport_baseline_captured": baseline_capture_complete,
            "viewport_capture_reference_order": capture_reference_order_closed,
            "viewport_visible_motion": viewport_visible_motion_passed,
            "viewport_static_repeat": viewport_static_repeat_passed,
            "single_run_only": foreign_run_messages == 0,
        }
        acceptance_failures = [name for name, closed in acceptance.items() if not closed]
        passed = not acceptance_failures
        if not passed:
            raise RuntimeError(
                "live preview acceptance gates did not pass: " + ",".join(acceptance_failures)
            )
    except BaseException as exc:
        primary_error = exc
    finally:
        if viewport_sync_annotator is not None:
            try:
                viewport_sync_annotator.detach(viewport_render_product_path)
            except BaseException as exc:
                if primary_error is None:
                    primary_error = exc
        if pose_writer is not None:
            try:
                pose_writer.close()
            except BaseException as exc:
                if primary_error is None:
                    primary_error = exc
        if executor_worker is not None:
            try:
                executor_worker.stop()
            except BaseException as exc:
                if primary_error is None:
                    primary_error = exc
        if node is not None:
            try:
                node.destroy_node()
                rclpy.try_shutdown()
            except BaseException as exc:
                if primary_error is None:
                    primary_error = exc
        try:
            if python_gc_frozen:
                gc.unfreeze()
                python_gc_unfrozen_on_close = gc.get_freeze_count() == 0
        except BaseException as exc:
            if primary_error is None:
                primary_error = exc
        for current_signal, previous in previous_handlers.items():
            signal.signal(current_signal, previous)

        if len(render_completion_ns) >= 2:
            effective_hz = (len(render_completion_ns) - 1) / (
                (render_completion_ns[-1] - render_completion_ns[0]) / 1_000_000_000
            )
        else:
            effective_hz = 0.0
        receipt = {
            "schema": "wujihand.dataset_live_preview_receipt.v1",
            "passed": passed and primary_error is None,
            "run_id": args.run_id,
            "role": "passive_external_gui_preview",
            "control_authority": False,
            "recorded_to_mcap": False,
            "record_chain_task_scene": (
                None
                if chain_preflight is None
                else chain_preflight.get("task_scene")
            ),
            "scene": (
                None
                if scene is None
                else scene.workcell_materialization.to_mapping()
            ),
            "background_color_rgb_requested": list(
                PREVIEW_BACKGROUND_COLOR_RGB
            ),
            "background_color_rgb_readback": (
                None
                if background_color_readback is None
                else list(background_color_readback)
            ),
            "transform_sync": PREVIEW_TRANSFORM_SYNC,
            "pose_replay_backend": PREVIEW_POSE_REPLAY_BACKEND,
            "pose_write_backend": PREVIEW_POSE_BACKEND,
            "pose_readback_backend": PREVIEW_POSE_READBACK_BACKEND,
            "pose_replay_reference": "isaacsim.replicator.episode_recorder.EpisodeReplayer",
            "pose_application_phase": "synchronous_pre_render_transaction",
            "pose_apply_count": (
                None if pose_writer is None else pose_writer.apply_count
            ),
            "qa_pose_apply_count": qa_pose_apply_count,
            "local_physics_replay_enabled": False,
            "scene_materialization_mode": "visual_replay_only",
            "simulation_manager_physics_view_active": (
                simulation_manager_physics_view_active
            ),
            "world_physics_view_active": world_physics_view_active,
            "local_physics_simulating": local_physics_simulating,
            "timeline_playing_before_replay_stop": timeline_playing_before_replay_stop,
            "timeline_playing_after_replay_stop": timeline_playing_after_replay_stop,
            "timeline_stopped_for_replay": timeline_stopped_for_replay,
            "viewport_capture_profile": PREVIEW_VIEWPORT_CAPTURE_PROFILE,
            "render_transaction": PREVIEW_RENDER_TRANSACTION,
            "multi_tick_rendering_enabled": multi_tick_rendering_enabled,
            "viewport_render_product_path": viewport_render_product_path,
            "viewport_sync_annotator": PREVIEW_SYNC_ANNOTATOR,
            "viewport_sync_annotator_attached": viewport_sync_annotator is not None,
            "viewport_capture_frame_numbers": viewport_capture_frame_numbers,
            "source_topic": (
                f"/{resolved.deployment.root_namespace}/{OPERATOR_PREVIEW_STATE_TOPIC}"
            ),
            "terminal_state": terminal_state,
            "shutdown_signal": shutdown_signal,
            "configured_render_hz": PREVIEW_HZ,
            "maximum_consecutive_catch_up_ticks": PREVIEW_MAXIMUM_CATCH_UP_TICKS,
            "viewport_width": PREVIEW_WIDTH,
            "viewport_height": PREVIEW_HEIGHT,
            "renderer": PREVIEW_RENDERER,
            "minimal_shading_mode": PREVIEW_MINIMAL_SHADING_MODE,
            "effective_render_hz": effective_hz,
            "rendered_active_frames": rendered_active_frames,
            "missed_render_periods": missed_render_periods,
            "render_mean_ms": (
                render_total_ns / rendered_active_frames / 1_000_000
                if rendered_active_frames
                else None
            ),
            "render_max_ms": render_max_ns / 1_000_000,
            "render_budget_ms": PREVIEW_RENDER_BUDGET_NS / 1_000_000,
            "slow_render_events": slow_render_events,
            "pose_apply_mean_ms": (
                pose_apply_total_ns / source_frames_applied / 1_000_000
                if source_frames_applied
                else None
            ),
            "pose_apply_max_ms": pose_apply_max_ns / 1_000_000,
            "warmup_render_count": warmup_render_count,
            "warmup_render_max_ms": warmup_render_max_ns / 1_000_000,
            "warmup_tail_max_ms": warmup_tail_max_ns / 1_000_000,
            "warmup_render_budget_ms": PREVIEW_WARMUP_RENDER_BUDGET_NS / 1_000_000,
            "source_frames_received": source_frames_received,
            "source_frames_applied": source_frames_applied,
            "source_kinematic_link_count": source_kinematic_link_count,
            "source_pose_prim_count": (
                None if pose_writer is None else len(pose_writer.paths)
            ),
            "source_pose_inventory_sha256": (
                None if pose_writer is None else pose_writer.inventory_sha256
            ),
            "replay_pose_prim_count": (
                None if pose_writer is None else len(pose_writer.replay_paths)
            ),
            "replay_pose_inventory_sha256": (
                None if pose_writer is None else pose_writer.replay_inventory_sha256
            ),
            "non_renderable_pose_prim_count": (
                None
                if pose_writer is None
                else len(pose_writer.paths) - len(pose_writer.replay_paths)
            ),
            "renderable_geometry_count": (
                None if pose_writer is None else len(pose_writer.renderable_paths)
            ),
            "renderable_geometry_inventory_sha256": (
                None if pose_writer is None else pose_writer.renderable_inventory_sha256
            ),
            "component_source_pose_counts": (
                None if pose_writer is None else pose_writer.component_source_pose_counts
            ),
            "component_replay_pose_counts": (
                None if pose_writer is None else pose_writer.component_replay_pose_counts
            ),
            "component_renderable_geometry_counts": (
                None
                if pose_writer is None
                else pose_writer.component_renderable_geometry_counts
            ),
            "component_source_position_max_delta_m": (
                component_source_position_max_delta_m
            ),
            "component_source_orientation_max_delta_rad": (
                component_source_orientation_max_delta_rad
            ),
            "component_source_position_motion_threshold_m": (
                PREVIEW_COMPONENT_SOURCE_POSITION_DELTA_M
            ),
            "component_source_orientation_motion_threshold_rad": (
                PREVIEW_COMPONENT_SOURCE_ORIENTATION_DELTA_RAD
            ),
            "component_source_motion_required": component_source_motion_required,
            "component_renderable_matrix_max_delta": (
                component_renderable_matrix_max_delta
            ),
            "component_renderable_motion_passed": (
                component_renderable_motion_passed
            ),
            "source_pose_position_max_delta_m": source_pose_position_max_delta_m,
            "source_pose_orientation_max_delta": source_pose_orientation_max_delta,
            "renderable_geometry_matrix_max_delta": (
                renderable_geometry_matrix_max_delta
            ),
            "renderable_geometry_matrix_delta_threshold": (
                PREVIEW_RENDERABLE_GEOMETRY_MATRIX_DELTA
            ),
            "renderable_geometry_motion_passed": renderable_geometry_motion_passed,
            "source_control_index_first": (
                source_control_indices[0] if source_control_indices else None
            ),
            "source_control_index_last": (
                source_control_indices[-1] if source_control_indices else None
            ),
            "source_q54_max_range_rad": source_q54_max_range_rad,
            "source_q54_group_max_range_rad": source_q54_group_max_range_rad,
            "source_q54_changed_frames": applied_q54_changed_frames,
            "viewport_baseline_control_index": viewport_baseline_control_index,
            "viewport_motion_control_index": viewport_motion_control_index,
            "viewport_return_control_index": viewport_return_control_index,
            "viewport_motion_trigger_rad": viewport_motion_trigger_rad,
            "viewport_visible_motion_q54_threshold_rad": (PREVIEW_VISIBLE_MOTION_Q54_THRESHOLD_RAD),
            "viewport_visible_motion_required": viewport_visible_motion_required,
            "viewport_visible_motion_passed": viewport_visible_motion_passed,
            "viewport_static_repeat_passed": viewport_static_repeat_passed,
            "viewport_baseline_sha256": viewport_baseline_sha256,
            "viewport_motion_sha256": viewport_motion_sha256,
            "viewport_return_sha256": viewport_return_sha256,
            "viewport_repeat_sha256": viewport_repeat_sha256,
            "viewport_pixel_delta_threshold": PREVIEW_VISIBLE_MOTION_PIXEL_DELTA,
            "viewport_pixel_mean_abs_delta": viewport_pixel_mean_abs_delta,
            "viewport_pixel_max_abs_delta": viewport_pixel_max_abs_delta,
            "viewport_pixel_changed_fraction": viewport_pixel_changed_fraction,
            "viewport_pixel_min_changed_fraction": (PREVIEW_VISIBLE_MOTION_MIN_CHANGED_FRACTION),
            "viewport_repeat_pixel_mean_abs_delta": (viewport_repeat_pixel_mean_abs_delta),
            "viewport_repeat_pixel_max_abs_delta": viewport_repeat_pixel_max_abs_delta,
            "viewport_repeat_pixel_changed_fraction": (viewport_repeat_pixel_changed_fraction),
            "foreign_run_messages": foreign_run_messages,
            "pose_position_max_abs_error_m": pose_position_max_error,
            "pose_closure_checks": pose_closure_checks,
            "state_closure_limit": STATE_CLOSURE_LIMIT,
            "acceptance_failures": acceptance_failures,
            "physics_step_anchor": physics_step_anchor,
            "simulation_time_anchor": simulation_time_anchor,
            "process_cpu_affinity": process_affinity,
            "cpu_thread_limit": thread_limit,
            "python_gc_frozen_object_count": python_gc_frozen_object_count,
            "python_gc_unfrozen_on_close": python_gc_unfrozen_on_close,
            "error": (
                None if primary_error is None else f"{type(primary_error).__name__}:{primary_error}"
            ),
        }
        try:
            receipt_path = _write_receipt(args.run_root.resolve(), receipt)
        except BaseException as exc:
            if primary_error is None:
                primary_error = exc
        preclose_result = {
            "schema": "wujihand.dataset_live_preview_cli_result.v1",
            "passed": passed and primary_error is None,
            "run_id": args.run_id,
            "receipt": None if receipt_path is None else str(receipt_path),
            "error": (
                None if primary_error is None else f"{type(primary_error).__name__}:{primary_error}"
            ),
        }
        print(json.dumps(preclose_result, sort_keys=True), flush=True)
        try:
            simulation_app.close()
        except BaseException as exc:
            if primary_error is None:
                primary_error = exc

    result = {
        "schema": "wujihand.dataset_live_preview_cli_result.v1",
        "passed": passed and primary_error is None,
        "run_id": args.run_id,
        "receipt": None if receipt_path is None else str(receipt_path),
        "error": (
            None if primary_error is None else f"{type(primary_error).__name__}:{primary_error}"
        ),
    }
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
