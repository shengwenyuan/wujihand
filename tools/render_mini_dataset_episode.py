#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported only after SimulationApp starts.
"""Render one gated 008 episode into exact pre-action three-view RGB truth."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, cast


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "analysis/teleoperation_quality/src"))

from teleoperation_quality.artifact import load_run_artifact
from wujihand.adapters.simulation import (
    load_nero_dual_simulation_startup_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.dataset import (
    load_alignment_artifact,
    load_mini_dataset_profile,
    load_normalized_episode_artifact,
    load_release_decision_artifact,
    load_visual_domain_variant_profile,
    render_exact_triview,
)
from wujihand.integrity import sha256_file
from wujihand.runtime import RosDeploymentResolver
from wujihand.runtime.isaac_dual_scene import resolve_dual_side_runtimes
from wujihand.runtime.wuji_hand2_record_chain import (
    load_record_chain_preflight_receipt,
    resolve_record_chain_workcell_plan,
)


DEFAULT_DEPLOYMENT = (
    ROOT / "configs/deployments/isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3.yaml"
)
DEFAULT_LOCAL_BINDING = ROOT / "configs/local/workstation2_nv5_ros_v2.yaml"
ISAAC_RENDERER = "RayTracedLighting"
DEFAULT_VARIANT_PROFILE = (
    ROOT / "configs/profiles/isaac_mini_dataset_visual_domain_variants_v1.yaml"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
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
    parser.add_argument("--render-variant", default="nominal")
    parser.add_argument("--variant-profile", type=Path, default=DEFAULT_VARIANT_PROFILE)
    return parser


def _mapping(value: object, *, field: str) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _preflight_derived(run_root: Path, *, artifact_name: str) -> None:
    if run_root.is_symlink() or not run_root.is_dir():
        raise ValueError("run root must be a non-symlink directory")
    load_run_artifact(run_root)
    release = load_release_decision_artifact(
        run_root / "derived" / "release",
        expected_run_id=run_root.name,
    )
    if not release.decision.passed:
        raise ValueError("offline RGB rendering requires a passing release decision")
    normalized = load_normalized_episode_artifact(
        run_root / "derived" / "normalized",
        expected_run_id=run_root.name,
    )
    alignment = load_alignment_artifact(
        run_root / "derived" / "alignment",
        expected_run_id=run_root.name,
    )
    if not normalized.facts.ticks or not alignment.frames:
        raise ValueError("offline RGB rendering requires non-empty normalized/alignment facts")
    vision = run_root / "derived" / artifact_name
    if vision.exists() or vision.is_symlink():
        raise FileExistsError("vision artifact already exists")


def _validate_raw_manifest_identity(
    run_root: Path,
    *,
    resolved: Any,
    dataset_profile_id: str,
    dataset_profile_sha256: str,
) -> None:
    try:
        value = json.loads((run_root / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("raw run manifest is invalid") from exc
    manifest = _mapping(value, field="raw manifest")
    deployment = _mapping(manifest.get("deployment"), field="raw manifest deployment")
    dataset = _mapping(manifest.get("dataset"), field="raw manifest dataset")
    expected = {
        "deployment_id": resolved.deployment.deployment_id,
        "deployment_hash": resolved.deployment_hash,
        "local_binding_hash": resolved.local_binding_hash,
        "session_id": resolved.session.session.session_id,
        "session_hash": resolved.session.session_hash,
        "assembly_path": resolved.session.assembly_path,
        "assembly_sha256": sha256_file(ROOT / resolved.session.assembly_path),
        "workcell_path": resolved.session.workcell_path,
        "workcell_sha256": sha256_file(ROOT / resolved.session.workcell_path),
    }
    if any(deployment.get(key) != wanted for key, wanted in expected.items()):
        raise ValueError("renderer Deployment/Session identity differs from the recorded run")
    if (
        dataset.get("profile_id") != dataset_profile_id
        or dataset.get("profile_sha256") != dataset_profile_sha256
        or dataset.get("episode_id_rule") != "run_id_equals_episode_id"
    ):
        raise ValueError("renderer dataset profile identity differs from the recorded run")


def _result(
    *,
    passed: bool,
    run_id: str | None = None,
    output: str | None = None,
    frame_count: int | None = None,
    inventories: list[dict[str, object]] | None = None,
    render_variant: dict[str, object] | None = None,
    render_variant_profile_sha256: str | None = None,
    error: str | None = None,
) -> str:
    return json.dumps(
        {
            "schema": "wujihand.dataset_renderer_cli_result.v1",
            "passed": passed,
            "run_id": run_id,
            "output": output,
            "frame_count": frame_count,
            "camera_runtime_inventories": inventories,
            "render_variant": render_variant,
            "render_variant_profile_sha256": render_variant_profile_sha256,
            "error": error,
        },
        sort_keys=True,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    simulation_app: Any | None = None
    backend: Any | None = None
    primary_error: BaseException | None = None
    try:
        run_root = args.run_root.resolve()
        variant_profile = load_visual_domain_variant_profile(ROOT, args.variant_profile)
        variant = variant_profile.variant(args.render_variant)
        artifact_name = (
            "vision" if variant.variant_id == "nominal" else f"vision_{variant.variant_id}"
        )
        _preflight_derived(run_root, artifact_name=artifact_name)
        resolved = RosDeploymentResolver(ROOT).resolve(
            args.deployment,
            local_binding=args.local_runtime_binding,
            verify_artifacts=args.verify_artifacts,
        )
        profile_ref = resolved.session.session.dataset_profile
        if profile_ref is None:
            raise ValueError("renderer Deployment does not resolve an 008 dataset profile")
        dataset_profile = load_mini_dataset_profile(ROOT, profile_ref.path)
        if dataset_profile.profile_id != profile_ref.expected_id:
            raise ValueError("renderer dataset profile ID differs from its Session pin")
        _validate_raw_manifest_identity(
            run_root,
            resolved=resolved,
            dataset_profile_id=dataset_profile.profile_id,
            dataset_profile_sha256=dataset_profile.file_sha256,
        )
        preflight_path = run_root / "preflight" / "wuji_hand2_record_chain.json"
        workcell_plan = (
            resolve_record_chain_workcell_plan(
                ROOT,
                resolved,
                load_record_chain_preflight_receipt(preflight_path),
                verify_content=args.verify_artifacts,
            )
            if preflight_path.is_file()
            else None
        )
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
        qualification_path = ROOT / resolved.control_profile.base_qualification.path
        qualification_profile = load_nero_dual_simulation_startup_profile(
            qualification_path
        )

        from isaacsim import SimulationApp  # type: ignore[import-not-found]

        simulation_app = SimulationApp(
            {
                "headless": True,
                "width": 640,
                "height": 480,
                "anti_aliasing": 0,
                "renderer": ISAAC_RENDERER,
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
            workcell_plan=workcell_plan,
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
            # Full Kit updates are allowed only before any recorded state is
            # injected, so all three render products can finish startup warm-up.
            warmup_update_app=simulation_app.update,
            visual_domain_variant=variant,
            visual_domain_variant_profile_sha256=variant_profile.file_sha256,
        )
        artifact = render_exact_triview(
            run_root,
            dataset_profile=dataset_profile,
            backend=backend,
            artifact_name=artifact_name,
        )
        inventories = [item.to_mapping() for item in backend.inventories]
        backend.close()
        backend = None
        print(
            _result(
                passed=True,
                run_id=artifact.run_id,
                output=str(artifact.root),
                frame_count=artifact.frame_count,
                inventories=inventories,
                render_variant=variant.to_mapping(),
                render_variant_profile_sha256=variant_profile.file_sha256,
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
            raise RuntimeError("offline renderer cleanup failed") from close_error


if __name__ == "__main__":
    raise SystemExit(main())
