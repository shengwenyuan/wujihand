#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Open the formal dual NERO—Hand 2—v2 mount—D405 Isaac inspector."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback
from typing import cast

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SESSION = Path(
    "configs/sessions/isaac_nero_dual_hand2_d405_wrist_rig_physical_simulation_nominal_v1.yaml"
)
DEFAULT_QUALIFICATION_PROFILE = Path(
    "configs/profiles/isaac_nero_hand2_self_collision_qualification_v1.yaml"
)
PERSPECTIVE_CAMERA_PATH = "/OmniverseKit_Persp"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument(
        "--qualification-profile",
        type=Path,
        default=DEFAULT_QUALIFICATION_PROFILE,
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gui", action="store_true", help="Open the Isaac GUI.")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="GUI update count; zero keeps a GUI run open until Isaac closes.",
    )
    parser.add_argument(
        "--hand-pose",
        choices=("rest", "grasp"),
        help="Apply the same versioned q20 pose to both hands; default: rest.",
    )
    parser.add_argument(
        "--right-hand-pose",
        choices=("rest", "grasp"),
        help=(
            "Compatibility alias for the former inspector. It now applies axis-"
            "symmetrically to both hands and never rotates a Hand 2 base."
        ),
    )
    parser.add_argument(
        "--initial-view",
        choices=("assembly", "left-optical", "right-optical"),
        default="assembly",
    )
    parser.add_argument("--render-settle-frames", type=int, default=8)
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> str:
    if args.frames < 0:
        raise ValueError("--frames must be non-negative")
    if not args.gui and args.frames == 0:
        raise ValueError("headless inspection requires a positive --frames value")
    if args.render_settle_frames < 1:
        raise ValueError("--render-settle-frames must be positive")
    if (
        args.hand_pose is not None
        and args.right_hand_pose is not None
        and args.hand_pose != args.right_hand_pose
    ):
        raise ValueError("--hand-pose and --right-hand-pose disagree")
    return cast(str, args.hand_pose or args.right_hand_pose or "rest")


ARGS = _parse_args()
HAND_POSE = _validate_args(ARGS)
PROJECT_ROOT = ARGS.project_root.expanduser().resolve()
OUTPUT_DIR = ARGS.output_dir.expanduser().resolve()
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from wujihand.integrity import sha256_file
from wujihand.runtime.isaac_d405_wrist_rig_inspection import (
    D405WristRigInspection,
    HandPose,
    inspection_state_snapshot,
    materialize_d405_wrist_rig_inspection,
    preflight_d405_wrist_rig_inspection,
)


# Session, Assembly, Binding, source-lock and profile validation completes before Isaac.
PLAN = preflight_d405_wrist_rig_inspection(
    project_root=PROJECT_ROOT,
    session_path=ARGS.session,
    qualification_profile_path=ARGS.qualification_profile,
)

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": not ARGS.gui,
        "width": 640,
        "height": 480,
        "anti_aliasing": 0,
    }
)

import omni.kit.renderer_capture
import omni.timeline
from isaacsim.core.utils.viewports import set_camera_view
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from PIL import Image
from pxr import Gf, Sdf, UsdGeom


def _capture(
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


def _hand_view(
    inspection: D405WristRigInspection,
    side: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    sign = -1.0 if side == "left" else 1.0
    eye_local = (-0.18, sign * 0.23, 0.14)
    target_local = (-0.035, sign * 0.065, 0.025)
    return (
        inspection.hand_base_point_world_m(side, eye_local),
        inspection.hand_base_point_world_m(side, target_local),
    )


def _capture_exploded_interface(
    inspection: D405WristRigInspection,
    side: str,
) -> dict[str, object]:
    """Expose the keyed flange face without changing its accepted assembly pose."""

    handles = next(item for item in inspection.scene.wrist_rigs if item.side == side)
    copy_path = f"{handles.root_path}/InspectionExplodedMount"
    root_layer = inspection.scene.stage.GetRootLayer()
    copied = Sdf.CopySpec(
        root_layer,
        Sdf.Path(handles.mount_visual_path),
        root_layer,
        Sdf.Path(copy_path),
    )
    if not copied:
        raise RuntimeError(f"could not create exploded mount view for {side}")
    try:
        copy_prim = inspection.scene.stage.GetPrimAtPath(copy_path)
        xformable = UsdGeom.Xformable(copy_prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(-0.20, 0.0, 0.0))
        sign = -1.0 if side == "left" else 1.0
        eye = inspection.hand_base_point_world_m(side, (-0.20, sign * 0.03, 0.09))
        target = inspection.hand_base_point_world_m(side, (-0.20, sign * 0.02, 0.0))
        return _capture(
            filename=f"{side}_flange_interface.png",
            camera_path=PERSPECTIVE_CAMERA_PATH,
            eye_m=eye,
            target_m=target,
        )
    finally:
        # Isaac 6.0.1 invalidates every deprecated Articulation physics view
        # when any stage prim is deleted. Keep this inspection-only copy in the
        # transient stage and hide it; the stage is discarded when Kit closes.
        copy_prim = inspection.scene.stage.GetPrimAtPath(copy_path)
        if copy_prim.IsValid():
            UsdGeom.Imageable(copy_prim).MakeInvisible()


def _set_collision_debug(
    inspection: D405WristRigInspection,
    *,
    enabled: bool,
) -> None:
    for handles in inspection.scene.wrist_rigs:
        visual_paths = (handles.mount_visual_path, handles.camera_visual_path)
        for path in visual_paths:
            imageable = UsdGeom.Imageable(inspection.scene.stage.GetPrimAtPath(path))
            imageable.GetVisibilityAttr().Set(
                UsdGeom.Tokens.invisible if enabled else UsdGeom.Tokens.inherited
            )
        groups = (
            (handles.mount_collision_paths, Gf.Vec3f(1.0, 0.32, 0.05), 1.0),
            (handles.camera_collision_paths, Gf.Vec3f(0.05, 0.85, 0.25), 0.3),
        )
        for paths, color, opacity in groups:
            for path in paths:
                gprim = UsdGeom.Gprim(inspection.scene.stage.GetPrimAtPath(path))
                gprim.GetPurposeAttr().Set(
                    UsdGeom.Tokens.default_ if enabled else UsdGeom.Tokens.guide
                )
                if enabled:
                    gprim.GetDisplayColorAttr().Set([color])
                    gprim.GetDisplayOpacityAttr().Set([opacity])


def _select_initial_view(
    inspection: D405WristRigInspection,
) -> str:
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    if ARGS.initial_view == "assembly":
        viewport.camera_path = PERSPECTIVE_CAMERA_PATH
        set_camera_view(
            eye=np.asarray((1.55, -2.05, 1.75), dtype=np.float64),
            target=np.asarray((0.0, 0.0, 0.85), dtype=np.float64),
            camera_prim_path=PERSPECTIVE_CAMERA_PATH,
            viewport_api=viewport,
        )
        return PERSPECTIVE_CAMERA_PATH
    side = ARGS.initial_view.removesuffix("-optical")
    camera_path = next(
        handles.camera_prim_path for handles in inspection.scene.wrist_rigs if handles.side == side
    )
    viewport.camera_path = camera_path
    return camera_path


def _render_acceptance_screenshots(
    inspection: D405WristRigInspection,
) -> dict[str, object]:
    screenshots: dict[str, object] = {}
    handles_by_side = {handles.side: handles for handles in inspection.scene.wrist_rigs}
    if inspection.hand_pose == "grasp":
        for side in ("left", "right"):
            screenshots[f"{side}_optical_grasp_140"] = _capture(
                filename=f"{side}_optical_grasp_140.png",
                camera_path=handles_by_side[side].camera_prim_path,
            )
        return screenshots
    screenshots["dual_overview"] = _capture(
        filename="dual_overview.png",
        camera_path=PERSPECTIVE_CAMERA_PATH,
        eye_m=(1.55, -2.05, 1.75),
        target_m=(0.0, 0.0, 0.85),
    )
    for side in ("left", "right"):
        eye, target = _hand_view(inspection, side)
        screenshots[f"{side}_assembly_closeup"] = _capture(
            filename=f"{side}_assembly_closeup.png",
            camera_path=PERSPECTIVE_CAMERA_PATH,
            eye_m=eye,
            target_m=target,
        )
        screenshots[f"{side}_flange_interface"] = _capture_exploded_interface(
            inspection,
            side,
        )
        screenshots[f"{side}_optical_{inspection.hand_pose}_140"] = _capture(
            filename=f"{side}_optical_{inspection.hand_pose}_140.png",
            camera_path=handles_by_side[side].camera_prim_path,
        )
    _set_collision_debug(inspection, enabled=True)
    try:
        for side in ("left", "right"):
            eye, target = _hand_view(inspection, side)
            screenshots[f"{side}_collision_debug"] = _capture(
                filename=f"{side}_collision_debug.png",
                camera_path=PERSPECTIVE_CAMERA_PATH,
                eye_m=eye,
                target_m=target,
            )
    finally:
        _set_collision_debug(inspection, enabled=False)
    return screenshots


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    inspection: D405WristRigInspection | None = None
    try:
        inspection = materialize_d405_wrist_rig_inspection(
            PLAN,
            hand_pose=cast(HandPose, HAND_POSE),
            render_settle_frames=ARGS.render_settle_frames,
        )
        screenshots = _render_acceptance_screenshots(inspection)
        active_camera = _select_initial_view(inspection)
        for _ in range(4):
            simulation_app.update()
        before_gui_ready = inspection_state_snapshot(inspection.scene)
        timeline = omni.timeline.get_timeline_interface()
        checks = {
            "two_q27_roots_preserved": len(inspection.scene.root_paths_before_reset) == 2,
            "both_wrist_rigs_present": len(inspection.scene.wrist_rigs) == 2,
            "selected_pose_is_axis_symmetric": all(
                inspection.scene.hand_targets[side].shape == (20,) for side in ("left", "right")
            ),
            "settled_to_pause_state_unchanged": (
                inspection.settled_snapshot == inspection.paused_snapshot
            ),
            "render_only_updates_preserve_state": (inspection.paused_snapshot == before_gui_ready),
            "timeline_is_paused_not_stopped": (
                not timeline.is_playing() and not timeline.is_stopped()
            ),
            "acceptance_screenshots_complete": len(screenshots)
            == (9 if inspection.hand_pose == "rest" else 2),
        }
        report = {
            "schema": "wujihand.isaac_d405_wrist_rig_static_inspection_report.v1",
            "passed": all(checks.values()),
            "hand_pose": inspection.hand_pose,
            "self_collision_policy": "merged_q27_disabled",
            "initial_view": ARGS.initial_view,
            "active_camera": active_camera,
            "session": {
                "path": str(PLAN.session_path),
                "sha256": sha256_file(PLAN.session_path),
                "resolved_hash": PLAN.resolved.session_hash,
            },
            "simulation_camera_boundary": (
                "SIMULATION ONLY: synthetic 140-degree HFOV; not a physical "
                "RealSense D405 specification or calibration."
            ),
            "lifecycle": {
                "physics_state": "paused",
                "world_stop_deferred_until_finally": True,
                "settled_snapshot": inspection.settled_snapshot,
                "paused_snapshot": inspection.paused_snapshot,
                "before_gui_ready_snapshot": before_gui_ready,
            },
            "final_target_error_rad": inspection.final_target_error_rad,
            "checks": checks,
            "screenshots": screenshots,
        }
        report_path = OUTPUT_DIR / "report.json"
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if not report["passed"]:
            raise RuntimeError(f"static inspection lifecycle failed: {checks}")
        print(
            "WRIST MOUNT RENDER PASS: "
            f"report={report_path} "
            f"left_optical={OUTPUT_DIR / f'left_optical_{HAND_POSE}_140.png'} "
            f"right_optical={OUTPUT_DIR / f'right_optical_{HAND_POSE}_140.png'}",
            flush=True,
        )
        print(
            "WRIST MOUNT GUI READY: "
            f"pose={HAND_POSE} initial_view={ARGS.initial_view} "
            f"camera={active_camera} timeline=paused",
            flush=True,
        )
        completed = 0
        while simulation_app.is_running() and (ARGS.frames == 0 or completed < ARGS.frames):
            simulation_app.update()
            completed += 1
        return 0
    except BaseException:
        # Isaac's fast shutdown can terminate before Python's default exception
        # hook runs. Emit the original failure while Kit is still alive.
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()
        raise
    finally:
        if inspection is not None:
            inspection.scene.world.stop()
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
