#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Render the S4 dual D405 wrist-rig appearance and 140-degree rest views."""

from __future__ import annotations

import argparse
from collections.abc import Mapping
import json
from pathlib import Path
import sys
from typing import Any, cast

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SESSION = Path(
    "configs/sessions/"
    "isaac_nero_dual_hand2_d405_wrist_rig_physical_simulation_nominal_v1.yaml"
)
DEFAULT_FILTER_PROFILE = Path(
    "configs/profiles/isaac_nero_hand2_self_collision_filtered_pairs_v1.yaml"
)
PERSPECTIVE_CAMERA_PATH = "/OmniverseKit_Persp"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument("--filter-profile", type=Path, default=DEFAULT_FILTER_PROFILE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--physics-hz", type=int, default=120)
    parser.add_argument("--settle-frames", type=int, default=240)
    parser.add_argument("--render-settle-frames", type=int, default=8)
    return parser.parse_args()


ARGS = _parse_args()
if ARGS.physics_hz < 1 or ARGS.settle_frames < 1 or ARGS.render_settle_frames < 1:
    raise ValueError("physics and settle frame counts must be positive")
if ARGS.render_settle_frames > ARGS.settle_frames:
    raise ValueError("--render-settle-frames cannot exceed --settle-frames")

PROJECT_ROOT = ARGS.project_root.expanduser().resolve()
OUTPUT_DIR = ARGS.output_dir.expanduser().resolve()
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from wujihand.adapters.simulation.nero_hand2_self_collision import (
    load_nero_hand2_self_collision_filter_profile,
)
from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_tabletop import (
    load_nero_dual_tabletop_qualification_profile,
)
from wujihand.integrity import sha256_file
from wujihand.runtime import SessionResolver
from wujihand.runtime.isaac_dual_scene import resolve_dual_side_runtimes


def _project_file(path: Path) -> Path:
    candidate = path if path.is_absolute() else PROJECT_ROOT / path
    resolved = candidate.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return resolved


SESSION_PATH = _project_file(ARGS.session)
FILTER_PROFILE_PATH = _project_file(ARGS.filter_profile)
RESOLVED = SessionResolver(PROJECT_ROOT).resolve(SESSION_PATH, verify_artifacts=True)
SIDES = resolve_dual_side_runtimes(PROJECT_ROOT, RESOLVED)
ALIGNMENT_REFS = {
    RESOLVED.instance(side.arm_instance_id).binding.compatibility_profile for side in SIDES
}
if None in ALIGNMENT_REFS or len(ALIGNMENT_REFS) != 1:
    raise RuntimeError("both NERO bindings must share one geometry alignment profile")
ALIGNMENT_PATH = _project_file(Path(cast(str, ALIGNMENT_REFS.pop())))
QUALIFICATION_REF = RESOLVED.session.runtime.compatibility_profile
if QUALIFICATION_REF is None:
    raise RuntimeError("D405 inspection Session lacks a qualification profile")
QUALIFICATION_PATH = _project_file(Path(QUALIFICATION_REF))
ALIGNMENT = load_nero_link_geometry_alignment(ALIGNMENT_PATH)
QUALIFICATION = load_nero_dual_tabletop_qualification_profile(QUALIFICATION_PATH)
FILTER_PROFILE = load_nero_hand2_self_collision_filter_profile(FILTER_PROFILE_PATH)

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": True,
        "width": 640,
        "height": 480,
        "anti_aliasing": 0,
    }
)

import omni.kit.renderer_capture
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from PIL import Image
from pxr import Gf, Usd, UsdGeom

from wujihand.runtime.isaac_dual_scene import DualNeroHand2IsaacScene


def _world_point(
    stage: Any,
    prim_path: str,
    local_point_m: tuple[float, float, float],
) -> tuple[float, float, float]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"invalid inspection frame: {prim_path}")
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    point = transform.Transform(Gf.Vec3d(*local_point_m))
    return cast(tuple[float, float, float], tuple(float(point[index]) for index in range(3)))


def _world_transform(stage: Any, prim_path: str) -> list[list[float]]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid() or not prim.IsA(UsdGeom.Xformable):
        raise RuntimeError(f"invalid transform prim: {prim_path}")
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    return [[float(matrix[row][column]) for column in range(4)] for row in range(4)]


def _state_snapshot(scene: DualNeroHand2IsaacScene) -> dict[str, object]:
    return {
        "q27": {
            side: scene.feedback_q27(side).tolist() for side in ("left", "right")
        },
        "hand_base_world": {
            side: _world_transform(scene.stage, scene.authored[side].config.child_base_link_path)
            for side in ("left", "right")
        },
        "camera_world": {
            handles.side: _world_transform(scene.stage, handles.camera_prim_path)
            for handles in scene.wrist_rigs
        },
    }


def _capture(
    scene: DualNeroHand2IsaacScene,
    *,
    filename: str,
    camera_path: str,
    eye_m: tuple[float, float, float] | None = None,
    target_m: tuple[float, float, float] | None = None,
) -> dict[str, object]:
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    viewport.camera_path = camera_path
    if (eye_m is None) != (target_m is None):
        raise ValueError("eye and target must be supplied together")
    if eye_m is not None and target_m is not None:
        set_camera_view(
            eye=np.asarray(eye_m, dtype=np.float64),
            target=np.asarray(target_m, dtype=np.float64),
            camera_prim_path=camera_path,
            viewport_api=viewport,
        )
    for _ in range(4):
        simulation_app.update()
    path = OUTPUT_DIR / filename
    path.unlink(missing_ok=True)
    capture = capture_viewport_to_file(viewport, file_path=str(path))
    completed = simulation_app.run_coroutine(capture.wait_for_result(completion_frames=30))
    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not completed or not path.is_file():
        raise RuntimeError(f"Isaac did not complete screenshot {filename}")
    with Image.open(path) as image:
        resolution = [int(image.width), int(image.height)]
        extrema = image.convert("RGB").getextrema()
    if resolution != [640, 480] or all(low == high for low, high in extrema):
        raise RuntimeError(f"invalid screenshot payload: {filename}")
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "resolution": resolution,
        "camera_path": camera_path,
        "eye_m": None if eye_m is None else list(eye_m),
        "target_m": None if target_m is None else list(target_m),
    }


def _closeup_view(
    scene: DualNeroHand2IsaacScene,
    side: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    sign = -1.0 if side == "left" else 1.0
    hand_base = scene.authored[side].config.child_base_link_path
    eye = _world_point(scene.stage, hand_base, (-0.18, sign * 0.23, 0.14))
    target = _world_point(scene.stage, hand_base, (-0.035, sign * 0.065, 0.025))
    return eye, target


def _stable(before: Mapping[str, object], after: Mapping[str, object]) -> bool:
    return json.dumps(before, sort_keys=True) == json.dumps(after, sort_keys=True)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    scene: DualNeroHand2IsaacScene | None = None
    try:
        scene = DualNeroHand2IsaacScene(
            project_root=PROJECT_ROOT,
            resolved=RESOLVED,
            sides=SIDES,
            alignment_profile=ALIGNMENT,
            qualification_profile=QUALIFICATION,
            physics_hz=ARGS.physics_hz,
            self_collision_sides=frozenset({"left", "right"}),
            self_collision_filter_profile=FILTER_PROFILE,
            wrist_rig_collision_mode="all",
        )
        for frame in range(ARGS.settle_frames):
            scene.apply_targets()
            scene.world.step(
                render=frame >= ARGS.settle_frames - ARGS.render_settle_frames
            )
        scene.world.pause()
        before = _state_snapshot(scene)
        screenshots: dict[str, object] = {}
        screenshots["dual_overview"] = _capture(
            scene,
            filename="dual_overview.png",
            camera_path=PERSPECTIVE_CAMERA_PATH,
            eye_m=(1.55, -2.05, 1.75),
            target_m=(0.0, 0.0, 0.85),
        )
        handles_by_side = {handles.side: handles for handles in scene.wrist_rigs}
        for side in ("left", "right"):
            eye, target = _closeup_view(scene, side)
            screenshots[f"{side}_assembly_closeup"] = _capture(
                scene,
                filename=f"{side}_assembly_closeup.png",
                camera_path=PERSPECTIVE_CAMERA_PATH,
                eye_m=eye,
                target_m=target,
            )
            screenshots[f"{side}_optical_rest_140"] = _capture(
                scene,
                filename=f"{side}_optical_rest_140.png",
                camera_path=handles_by_side[side].camera_prim_path,
            )
        after = _state_snapshot(scene)
        report = {
            "schema": "wujihand.isaac_d405_wrist_rig_s4_render_report.v1",
            "passed": _stable(before, after),
            "session": {
                "path": str(SESSION_PATH),
                "sha256": sha256_file(SESSION_PATH),
                "resolved_hash": RESOLVED.session_hash,
            },
            "simulation_camera_boundary": (
                "SIMULATION ONLY: synthetic 140-degree HFOV; not a physical "
                "RealSense D405 specification or calibration."
            ),
            "camera_profiles": {
                runtime.side: {
                    "path": str(runtime.camera_profile_path),
                    "sha256": sha256_file(runtime.camera_profile_path),
                    "profile_id": runtime.camera_profile.profile_id,
                    "horizontal_fov_deg": runtime.camera_profile.optics.horizontal_fov_deg,
                    "resolution": [
                        runtime.camera_profile.capture.width_px,
                        runtime.camera_profile.capture.height_px,
                    ],
                    "simulation_only": runtime.camera_profile.simulation_only,
                }
                for runtime in scene.wrist_rig_runtimes
            },
            "collision_inventory": {
                handles.side: {
                    "mount_shapes": len(handles.mount_collision_paths),
                    "camera_shapes": len(handles.camera_collision_paths),
                }
                for handles in scene.wrist_rigs
            },
            "state_before_render_only_capture": before,
            "state_after_render_only_capture": after,
            "checks": {
                "world_is_paused_during_capture": not scene.world.is_playing(),
                "render_only_capture_preserves_state": _stable(before, after),
                "both_camera_prims_present": len(scene.wrist_rigs) == 2,
                "screenshots_complete": len(screenshots) == 5,
            },
            "screenshots": screenshots,
        }
        report["passed"] = all(cast(Mapping[str, bool], report["checks"]).values())
        report_path = OUTPUT_DIR / "report.json"
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if not report["passed"]:
            raise RuntimeError("S4 D405 wrist-rig render checks failed")
        print(
            "D405 WRIST RIG S4 RENDER PASS: "
            f"report={report_path} overview={OUTPUT_DIR / 'dual_overview.png'}",
            flush=True,
        )
        return 0
    finally:
        if scene is not None:
            scene.world.stop()
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
