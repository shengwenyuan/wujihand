#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported only after SimulationApp starts.
"""Render an explicitly non-canonical diagnostic selection from one raw episode."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation import (
    load_nero_dual_tabletop_qualification_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.dataset import load_mini_dataset_profile, validate_rgb8_png
from wujihand.dataset.vision import CAMERA_IDS
from wujihand.domain.dataset_recording import SimulationFramePhase, SimulationStateFrame
from wujihand.integrity import sha256_file
from wujihand.runtime import RosDeploymentResolver
from wujihand.runtime.isaac_dual_scene import resolve_dual_side_runtimes


DEFAULT_DEPLOYMENT = (
    ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml"
)
DEFAULT_LOCAL_BINDING = ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"
SELECTION_SCHEMA = "wujihand.diagnostic_replay_selection.v1"
OUTPUT_SCHEMA = "wujihand.diagnostic_triview_replay.v1"
LINK_POSITION_LIMIT_M = 2e-5
SOURCE_TIME_TOLERANCE_S = 5e-6


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--deployment", type=Path, default=DEFAULT_DEPLOYMENT)
    parser.add_argument("--local-runtime-binding", type=Path, default=DEFAULT_LOCAL_BINDING)
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def _mapping(value: object, *, field: str) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return value


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_json_mapping(path: Path, *, field: str) -> dict[str, object]:
    try:
        return _mapping(json.loads(path.read_text(encoding="utf-8")), field=field)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"{field} is invalid") from exc


def _validate_selection(
    document: dict[str, object],
    *,
    run_id: str,
) -> tuple[tuple[str, SimulationStateFrame], ...]:
    if document.get("schema") != SELECTION_SCHEMA:
        raise ValueError("diagnostic selection schema differs")
    if document.get("diagnostic_only") is not True:
        raise ValueError("selection is not explicitly diagnostic-only")
    if document.get("formal_release_passed") is not False:
        raise ValueError("diagnostic replay requires an explicitly failed formal release")
    if document.get("run_id") != run_id:
        raise ValueError("diagnostic selection run ID differs")
    blocker = document.get("formal_release_blocker")
    if not isinstance(blocker, str) or not blocker:
        raise ValueError("diagnostic selection must preserve its formal release blocker")
    selected = document.get("selected")
    if not isinstance(selected, list) or not selected:
        raise ValueError("diagnostic selection must contain at least one frame")
    if document.get("selected_frame_count") != len(selected):
        raise ValueError("diagnostic selected-frame count differs")
    result: list[tuple[str, SimulationStateFrame]] = []
    previous_control_index: int | None = None
    for index, value in enumerate(selected):
        item = _mapping(value, field=f"selected[{index}]")
        reason = item.get("reason")
        if not isinstance(reason, str) or not reason:
            raise ValueError(f"selected[{index}].reason is invalid")
        frame = SimulationStateFrame.from_mapping(
            _mapping(item.get("frame"), field=f"selected[{index}].frame")
        )
        if frame.run_id != run_id or frame.phase is not SimulationFramePhase.PRE_ACTION:
            raise ValueError("diagnostic selection must contain same-run pre_action states")
        if previous_control_index is not None and frame.control_index <= previous_control_index:
            raise ValueError("diagnostic source control indices must be strictly increasing")
        previous_control_index = frame.control_index
        result.append((reason, frame))
    return tuple(result)


def _validate_source_clock(
    frames: tuple[tuple[str, SimulationStateFrame], ...],
    *,
    physics_hz: float,
) -> int:
    origins: set[int] = set()
    previous_boundary: int | None = None
    for _, frame in frames:
        source_grid_index = round(frame.simulation_time_s * physics_hz)
        if abs(source_grid_index / physics_hz - frame.simulation_time_s) > SOURCE_TIME_TOLERANCE_S:
            raise ValueError("diagnostic source state differs from the 120 Hz grid")
        if previous_boundary is not None and frame.physics_boundary_index <= previous_boundary:
            raise ValueError("diagnostic physics boundaries must be strictly increasing")
        previous_boundary = frame.physics_boundary_index
        origins.add(source_grid_index - frame.physics_boundary_index)
    if len(origins) != 1:
        raise ValueError("diagnostic source clock does not have one fixed integer origin")
    return next(iter(origins))


def _prepare_paths(args: argparse.Namespace) -> tuple[Path, Path, Path, Path]:
    project_root = args.project_root.resolve()
    run_root = args.run_root.resolve()
    selection = args.selection.resolve()
    output = args.output.absolute()
    if not project_root.is_dir() or not run_root.is_dir() or not selection.is_file():
        raise ValueError("project, run, and selection paths must exist")
    if args.run_root.is_symlink() or args.selection.is_symlink() or args.output.is_symlink():
        raise ValueError("diagnostic replay paths must not be symbolic links")
    if output.exists():
        raise FileExistsError(f"diagnostic output already exists: {output}")
    if output.is_relative_to(run_root):
        raise ValueError("diagnostic output must stay outside the canonical raw run")
    return project_root, run_root, selection, output


def _resolve_scene_inputs(
    project_root: Path,
    args: argparse.Namespace,
) -> tuple[Any, Any, Any, Any, Any]:
    resolved = RosDeploymentResolver(project_root).resolve(
        args.deployment,
        local_binding=args.local_runtime_binding,
        verify_artifacts=args.verify_artifacts,
    )
    profile_ref = resolved.session.session.dataset_profile
    if profile_ref is None:
        raise ValueError("diagnostic Deployment has no dataset profile")
    dataset_profile = load_mini_dataset_profile(project_root, profile_ref.path)
    if dataset_profile.profile_id != profile_ref.expected_id:
        raise ValueError("diagnostic dataset profile ID differs from the Session pin")
    sides = resolve_dual_side_runtimes(project_root, resolved.session)
    alignment_references = {
        resolved.session.instance(runtime.arm_instance_id).binding.compatibility_profile
        for runtime in sides
    }
    if None in alignment_references or len(alignment_references) != 1:
        raise ValueError("both NERO bindings must resolve one geometry profile")
    alignment_path = project_root / str(next(iter(alignment_references)))
    alignment_profile = load_nero_link_geometry_alignment(alignment_path)
    source_urdf = (project_root / alignment_profile.source_urdf_path).resolve()
    if sha256_file(source_urdf) != alignment_profile.source_urdf_sha256:
        raise ValueError("source-locked NERO URDF hash drifted")
    qualification_profile = load_nero_dual_tabletop_qualification_profile(
        project_root / resolved.control_profile.base_qualification.path
    )
    return resolved, dataset_profile, sides, alignment_profile, qualification_profile


def _render(
    *,
    project_root: Path,
    run_root: Path,
    selection_path: Path,
    output: Path,
    selection_document: dict[str, object],
    frames: tuple[tuple[str, SimulationStateFrame], ...],
    source_grid_origin: int,
    resolved: Any,
    dataset_profile: Any,
    sides: Any,
    alignment_profile: Any,
    qualification_profile: Any,
) -> dict[str, object]:
    staging = output.with_name(f".{output.name}.partial-{os.getpid()}")
    if staging.exists() or staging.is_symlink():
        raise FileExistsError(f"diagnostic staging path already exists: {staging}")
    staging.mkdir(parents=True)
    simulation_app: Any | None = None
    backend: Any | None = None
    primary_error: BaseException | None = None
    records: list[dict[str, object]] = []
    inventories: list[dict[str, object]] = []
    maximum_link_error = 0.0
    try:
        from isaacsim import SimulationApp  # type: ignore[import-not-found]

        simulation_app = SimulationApp(
            {
                "headless": True,
                "width": 640,
                "height": 480,
                "anti_aliasing": 0,
                "renderer": "RayTracedLighting",
                "multi_gpu": False,
                "limit_cpu_threads": 32,
                "disable_viewport_updates": True,
            }
        )
        from wujihand.runtime.isaac_dataset_rgb_renderer import (
            IsaacFixedStateRgbBackend,
        )
        from wujihand.runtime.isaac_dual_scene import DualNeroHand2IsaacScene

        scene = DualNeroHand2IsaacScene(
            project_root=project_root,
            resolved=resolved.session,
            sides=sides,
            alignment_profile=alignment_profile,
            qualification_profile=qualification_profile,
            physics_hz=resolved.control_profile.physics_hz,
            self_collision_sides=frozenset(),
            wrist_rig_collision_mode="all",
        )
        left_names, left_limits = scene.runtime_joint_inventory("left")
        right_names, right_limits = scene.runtime_joint_inventory("right")
        dataset_profile.q54.validate_runtime_inventory(
            left_names=left_names,
            left_limits_rad=left_limits,
            right_names=right_names,
            right_limits_rad=right_limits,
        )
        backend = IsaacFixedStateRgbBackend(
            project_root=project_root,
            scene=scene,
            dataset_profile=dataset_profile,
            warmup_update_app=simulation_app.update,
        )
        inventories = [item.to_mapping() for item in backend.inventories]
        for dataset_frame_index, (reason, frame) in enumerate(frames):
            acknowledged = backend.inject_pre_action_state(
                frame,
                dataset_frame_index=dataset_frame_index,
            )
            if acknowledged != frame.payload_digest_sha256:
                raise RuntimeError("diagnostic renderer state acknowledgement differs")
            replay_time_s = backend.simulation_time_s
            expected_replay_time_s = dataset_frame_index / dataset_profile.policy_fps
            if not math.isclose(
                replay_time_s,
                expected_replay_time_s,
                rel_tol=0.0,
                abs_tol=SOURCE_TIME_TOLERANCE_S,
            ):
                raise RuntimeError("diagnostic renderer replay timeline differs")
            rendered = tuple(
                backend.render_rgb(
                    camera_id=camera_id,
                    dataset_frame_index=dataset_frame_index,
                )
                for camera_id in CAMERA_IDS
            )
            closure = dict(backend.closure_metrics)
            link_error = closure.get("kinematic_link_position_max_abs_error_m")
            if link_error is None or not link_error < LINK_POSITION_LIMIT_M:
                raise RuntimeError("diagnostic link position closure did not pass")
            maximum_link_error = max(maximum_link_error, float(link_error))
            reference = backend.completed_reference_time
            payloads: dict[str, dict[str, object]] = {}
            for item in rendered:
                validate_rgb8_png(item.payload_png, field=item.camera_id)
                relative_path = Path(item.camera_id) / f"{dataset_frame_index:06d}.png"
                payload_path = staging / relative_path
                payload_path.parent.mkdir(parents=True, exist_ok=True)
                payload_path.write_bytes(item.payload_png)
                payloads[item.camera_id] = {
                    "path": relative_path.as_posix(),
                    "sha256": _sha256_bytes(item.payload_png),
                    "completed_frame_identity": item.completed_frame_identity,
                    "camera_profile_sha256": item.camera_profile_sha256,
                    "parent_frame_id": item.parent_frame_id,
                    "world_from_parent_row_major": list(item.world_from_parent_row_major),
                    "world_from_camera_optical_row_major": list(
                        item.world_from_camera_optical_row_major
                    ),
                }
            records.append(
                {
                    "dataset_frame_index": dataset_frame_index,
                    "reason": reason,
                    "source_control_index": frame.control_index,
                    "source_tick_id": frame.tick_id,
                    "source_state_digest": frame.payload_digest_sha256,
                    "source_simulation_time_s": frame.simulation_time_s,
                    "source_physics_boundary_index": frame.physics_boundary_index,
                    "source_physics_grid_index": round(
                        frame.simulation_time_s * dataset_profile.physics_hz
                    ),
                    "replay_time_s": replay_time_s,
                    "completed_reference_time": {
                        "numerator": reference[0],
                        "denominator": reference[1],
                    },
                    "closure": closure,
                    "rgb": payloads,
                }
            )
        if backend.source_physics_grid_origin != source_grid_origin:
            raise RuntimeError("renderer and preflight source clock origins differ")
        renderer_identity = backend.renderer_identity

        raw_manifest = _load_json_mapping(run_root / "manifest.json", field="raw manifest")
        raw_deployment = _mapping(raw_manifest.get("deployment"), field="raw deployment")
        current_workcell_sha256 = sha256_file(project_root / resolved.session.workcell_path)
        recorded_workcell_sha256 = raw_deployment.get("workcell_sha256")
        manifest: dict[str, object] = {
            "schema": OUTPUT_SCHEMA,
            "diagnostic_only": True,
            "canonical_dataset_artifact": False,
            "formal_release_passed": False,
            "formal_release_blocker": selection_document["formal_release_blocker"],
            "mainline_release_gates_changed": False,
            "bypass_scope": [
                "formal_release_gate_bypass_for_explicit_diagnostic_selection_only",
            ],
            "state_replay_policy": ("replay recorded q54 exactly; do not undo recorded clamps"),
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "run_id": run_root.name,
            "source": {
                "selection_path": str(selection_path),
                "selection_sha256": sha256_file(selection_path),
                "mcap_path": str(run_root / "raw/rosbag2/rosbag2_0.mcap"),
                "mcap_sha256": selection_document["source_mcap_sha256"],
                "recorded_deployment": raw_deployment,
            },
            "renderer": {
                "identity": renderer_identity,
                "deployment_id": resolved.deployment.deployment_id,
                "deployment_hash": resolved.deployment_hash,
                "session_id": resolved.session.session.session_id,
                "session_hash": resolved.session.session_hash,
                "assembly_sha256": sha256_file(project_root / resolved.session.assembly_path),
                "workcell_sha256": current_workcell_sha256,
                "render_workcell_differs_from_recorded": (
                    recorded_workcell_sha256 != current_workcell_sha256
                ),
                "dataset_profile_id": dataset_profile.profile_id,
                "dataset_profile_sha256": dataset_profile.file_sha256,
                "camera_runtime_inventories": inventories,
            },
            "clock": {
                "source_physics_hz": dataset_profile.physics_hz,
                "source_time_tolerance_s": SOURCE_TIME_TOLERANCE_S,
                "source_physics_grid_origin": source_grid_origin,
                "replay_fps": dataset_profile.policy_fps,
                "replay_formula": "dataset_frame_index / replay_fps",
            },
            "acceptance": {
                "kinematic_link_position_max_abs_error_required": (f"<{LINK_POSITION_LIMIT_M} m"),
                "single_render_transaction_per_state": True,
                "three_camera_reference_equality_required": True,
                "reference_strictly_increasing_required": True,
            },
            "frame_count": len(records),
            "frames": records,
        }
        (staging / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        staging.rename(output)
        print(
            json.dumps(
                {
                    "schema": OUTPUT_SCHEMA,
                    "passed": True,
                    "output": str(output),
                    "frame_count": len(records),
                    "source_physics_grid_origin": source_grid_origin,
                    "kinematic_link_position_max_abs_error_m": maximum_link_error,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return manifest
    except BaseException as exc:
        primary_error = exc
        print(
            json.dumps(
                {
                    "schema": OUTPUT_SCHEMA,
                    "passed": False,
                    "error": str(exc),
                    "error_type": type(exc).__name__,
                },
                sort_keys=True,
            ),
            file=sys.stderr,
            flush=True,
        )
        raise
    finally:
        close_error: BaseException | None = None
        if staging.exists():
            shutil.rmtree(staging)
        if backend is not None:
            try:
                backend.close()
            except BaseException as exc:
                close_error = exc
        if simulation_app is not None:
            try:
                simulation_app.close()
            except BaseException as exc:
                if close_error is None:
                    close_error = exc
        if close_error is not None and primary_error is None:
            raise RuntimeError("diagnostic renderer cleanup failed") from close_error


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        project_root, run_root, selection_path, output = _prepare_paths(args)
        selection_document = _load_json_mapping(
            selection_path,
            field="diagnostic selection",
        )
        frames = _validate_selection(selection_document, run_id=run_root.name)
        expected_mcap_sha256 = selection_document.get("source_mcap_sha256")
        mcap_path = run_root / "raw/rosbag2/rosbag2_0.mcap"
        if (
            not isinstance(expected_mcap_sha256, str)
            or sha256_file(mcap_path) != expected_mcap_sha256
        ):
            raise ValueError("diagnostic source MCAP hash differs")
        resolved_inputs = _resolve_scene_inputs(project_root, args)
        resolved, dataset_profile, sides, alignment_profile, qualification_profile = resolved_inputs
        source_grid_origin = _validate_source_clock(
            frames,
            physics_hz=dataset_profile.physics_hz,
        )
        manifest = _render(
            project_root=project_root,
            run_root=run_root,
            selection_path=selection_path,
            output=output,
            selection_document=selection_document,
            frames=frames,
            source_grid_origin=source_grid_origin,
            resolved=resolved,
            dataset_profile=dataset_profile,
            sides=sides,
            alignment_profile=alignment_profile,
            qualification_profile=qualification_profile,
        )
        manifest_frames = manifest.get("frames")
        if not isinstance(manifest_frames, list):
            raise RuntimeError("diagnostic manifest frame inventory is invalid")
        link_errors: list[float] = []
        for value in manifest_frames:
            frame = _mapping(value, field="manifest frame")
            closure = _mapping(frame.get("closure"), field="manifest closure")
            link_error = closure.get("kinematic_link_position_max_abs_error_m")
            if isinstance(link_error, bool) or not isinstance(link_error, (int, float)):
                raise RuntimeError("diagnostic manifest link error is invalid")
            link_errors.append(float(link_error))
        maximum_link_error = max(link_errors)
        print(
            json.dumps(
                {
                    "schema": OUTPUT_SCHEMA,
                    "passed": True,
                    "output": str(output),
                    "frame_count": manifest["frame_count"],
                    "source_physics_grid_origin": source_grid_origin,
                    "kinematic_link_position_max_abs_error_m": maximum_link_error,
                },
                sort_keys=True,
            )
        )
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(
            json.dumps(
                {"schema": OUTPUT_SCHEMA, "passed": False, "error": str(exc)},
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
