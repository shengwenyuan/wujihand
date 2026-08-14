#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported only after SimulationApp starts.
"""Run one device-free fixed-state three-view renderer qualification on Isaac 6."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation import (
    load_nero_dual_tabletop_qualification_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.dataset import load_mini_dataset_profile, validate_rgb8_png
from wujihand.domain.dataset_recording import SimulationFramePhase
from wujihand.integrity import sha256_file
from wujihand.runtime import RosDeploymentResolver
from wujihand.runtime.isaac_dual_scene import resolve_dual_side_runtimes


DEFAULT_DEPLOYMENT = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)
DEFAULT_LOCAL_BINDING = ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deployment", type=Path, default=DEFAULT_DEPLOYMENT)
    parser.add_argument(
        "--local-runtime-binding",
        type=Path,
        default=DEFAULT_LOCAL_BINDING,
    )
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def _result(*, passed: bool, **values: object) -> str:
    return json.dumps(
        {
            "schema": "wujihand.isaac_mini_dataset_renderer_qualification.v1",
            "passed": passed,
            **values,
        },
        sort_keys=True,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    simulation_app: Any | None = None
    backend: Any | None = None
    primary_error: BaseException | None = None
    try:
        resolved = RosDeploymentResolver(ROOT).resolve(
            args.deployment,
            local_binding=args.local_runtime_binding,
            verify_artifacts=args.verify_artifacts,
        )
        profile_ref = resolved.session.session.dataset_profile
        if profile_ref is None:
            raise ValueError("qualification Deployment has no dataset profile")
        dataset_profile = load_mini_dataset_profile(ROOT, profile_ref.path)
        if dataset_profile.profile_id != profile_ref.expected_id:
            raise ValueError("qualification dataset profile ID differs")
        sides = resolve_dual_side_runtimes(ROOT, resolved.session)
        alignment_references = {
            resolved.session.instance(runtime.arm_instance_id).binding.compatibility_profile
            for runtime in sides
        }
        if None in alignment_references or len(alignment_references) != 1:
            raise ValueError("both NERO bindings must resolve one geometry profile")
        alignment_path = ROOT / str(next(iter(alignment_references)))
        alignment_profile = load_nero_link_geometry_alignment(alignment_path)
        source_urdf = (ROOT / alignment_profile.source_urdf_path).resolve()
        if sha256_file(source_urdf) != alignment_profile.source_urdf_sha256:
            raise ValueError("source-locked NERO URDF hash drifted")
        qualification_profile = load_nero_dual_tabletop_qualification_profile(
            ROOT / resolved.control_profile.base_qualification.path
        )

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
            project_root=ROOT,
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
            project_root=ROOT,
            scene=scene,
            dataset_profile=dataset_profile,
            warmup_update_app=simulation_app.update,
        )
        # Backend startup may drain Isaac's pending World.reset() work and
        # warm RTX cameras.  The qualification boundary begins only after
        # that device-free initialization, immediately before truth capture.
        physics_step_before = int(scene.world.current_time_step_index)
        frame = scene.create_dataset_state_frame(
            run_id="diagnostic-device-free-renderer",
            control_index=0,
            phase=SimulationFramePhase.PRE_ACTION,
            simulation_time_s=0.0,
            physics_boundary_index=physics_step_before,
            q54_profile=dataset_profile.q54,
            q27_by_side={side: scene.feedback_q27(side) for side in ("left", "right")},
            qdot27_by_side={side: scene.feedback_qdot27(side) for side in ("left", "right")},
        )
        acknowledged = backend.inject_pre_action_state(frame, dataset_frame_index=0)
        if acknowledged != frame.payload_digest_sha256:
            raise RuntimeError("qualification renderer state acknowledgement differs")
        rendered = tuple(
            backend.render_rgb(camera_id=camera_id, dataset_frame_index=0)
            for camera_id in ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb")
        )
        payload_hashes = {
            item.camera_id: hashlib.sha256(item.payload_png).hexdigest() for item in rendered
        }
        for item in rendered:
            validate_rgb8_png(item.payload_png, field=item.camera_id)
        if len(set(payload_hashes.values())) != 3:
            raise RuntimeError("qualification renderer returned duplicate camera payloads")
        physics_step_after = int(scene.world.current_time_step_index)
        if physics_step_after != physics_step_before:
            raise RuntimeError("qualification renderer advanced physics")
        inventories = [item.to_mapping() for item in backend.inventories]
        backend.close()
        backend = None
        print(
            _result(
                passed=True,
                deployment_id=resolved.deployment.deployment_id,
                dataset_profile_id=dataset_profile.profile_id,
                q54_profile_sha256=dataset_profile.q54.file_sha256,
                source_state_digest=frame.payload_digest_sha256,
                physics_step_before=physics_step_before,
                physics_step_after=physics_step_after,
                payload_sha256=payload_hashes,
                camera_runtime_inventories=inventories,
            )
        )
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        primary_error = exc
        print(_result(passed=False, error=str(exc)), file=sys.stderr)
        return 2
    finally:
        close_error: BaseException | None = None
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
            raise RuntimeError("renderer qualification cleanup failed") from close_error


if __name__ == "__main__":
    raise SystemExit(main())
