#!/usr/bin/env python3
"""Open a visual-only NERO + Hand 2 + Gemini 305 mount inspection in Isaac."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SESSION = Path("configs/sessions/isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml")
PERSPECTIVE_CAMERA_PATH = "/OmniverseKit_Persp"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument(
        "--mount-stl",
        type=Path,
        required=True,
        help="OpenSCAD-exported v1 mount STL in millimetres.",
    )
    parser.add_argument(
        "--camera-stl",
        type=Path,
        help=(
            "Optional private Gemini 305 STL in millimetres, pre-aligned with "
            "rear face Z=0 and the rear-hole row along local Y."
        ),
    )
    parser.add_argument("--gui", action="store_true", help="Open the Isaac GUI.")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="GUI update count; zero keeps the GUI open until the operator closes it.",
    )
    parser.add_argument(
        "--initial-view",
        choices=("assembly", "color"),
        default="assembly",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--physics-hz", type=int, default=120)
    parser.add_argument("--settle-frames", type=int, default=240)
    parser.add_argument("--render-settle-frames", type=int, default=8)
    parser.add_argument(
        "--verify-session-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--verify-mount-digest",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reject a mount STL whose geometry differs from the frozen v1 SCAD export.",
    )
    parser.add_argument(
        "--verify-camera-digest",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reject an optional camera STL that is not the aligned private inspection mesh.",
    )
    parser.add_argument(
        "--camera-visual-aids",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Default: enabled for the box proxy and disabled for a supplied CAD mesh.",
    )
    parser.add_argument(
        "--extra-lights",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.frames < 0:
        raise ValueError("--frames must be non-negative")
    if not args.gui and args.frames == 0:
        raise ValueError("headless inspection requires a positive --frames value")
    if args.width < 64 or args.height < 64:
        raise ValueError("viewport dimensions must each be at least 64 pixels")
    if args.width * 10 != args.height * 16:
        raise ValueError("viewport resolution must preserve the Gemini Color 16:10 aspect")
    if args.physics_hz < 1 or args.settle_frames < 1 or args.render_settle_frames < 1:
        raise ValueError("physics-hz and settle frame counts must be positive")
    if args.render_settle_frames > args.settle_frames:
        raise ValueError("--render-settle-frames cannot exceed --settle-frames")


def _select_initial_view(inspection: Any, initial_view: str) -> str:
    import numpy as np
    from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
        set_camera_view,
    )
    from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
        get_active_viewport,
    )

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    if initial_view == "color":
        viewport.camera_path = inspection.overlay.color_camera_path
        return str(inspection.overlay.color_camera_path)
    viewport.camera_path = PERSPECTIVE_CAMERA_PATH
    set_camera_view(
        eye=np.asarray(inspection.assembly_eye_world_m, dtype=np.float64),
        target=np.asarray(inspection.assembly_target_world_m, dtype=np.float64),
        camera_prim_path=PERSPECTIVE_CAMERA_PATH,
        viewport_api=viewport,
    )
    return PERSPECTIVE_CAMERA_PATH


def main() -> int:
    args = build_parser().parse_args()
    _validate_args(args)
    project_root = args.project_root.expanduser().resolve()
    sys.path.insert(0, str(project_root / "src"))

    # Complete Session/source-lock/profile/STL validation before Isaac starts.
    from wujihand.runtime.isaac_wrist_mount_inspection import (
        materialize_isaac_wrist_mount_inspection,
        preflight_isaac_wrist_mount_inspection,
    )

    plan = preflight_isaac_wrist_mount_inspection(
        project_root=project_root,
        session_path=args.session,
        mount_stl_path=args.mount_stl,
        camera_stl_path=args.camera_stl,
        verify_session_artifacts=args.verify_session_artifacts,
        verify_mount_digest=args.verify_mount_digest,
        verify_camera_digest=args.verify_camera_digest,
    )

    from isaacsim import SimulationApp  # type: ignore[import-not-found]

    simulation_app = SimulationApp(
        {
            "headless": not args.gui,
            "width": args.width,
            "height": args.height,
            "anti_aliasing": 0,
        }
    )
    inspection = None
    try:
        inspection = materialize_isaac_wrist_mount_inspection(
            plan,
            physics_hz=args.physics_hz,
            settle_frames=args.settle_frames,
            render_settle_frames=args.render_settle_frames,
            show_camera_visual_aids=args.camera_visual_aids,
            extra_lights=args.extra_lights,
        )
        for _ in range(3):
            simulation_app.update()
        active_camera = _select_initial_view(inspection, args.initial_view)
        for _ in range(3):
            simulation_app.update()
        print(
            "WRIST MOUNT INSPECTION READY: "
            f"config={plan.config.config_id} "
            f"overlay={inspection.overlay.overlay_root_path} "
            f"geometry={inspection.overlay.camera_geometry_kind} "
            f"acceptance_hfov_deg={plan.config.acceptance_color_hfov_deg:g} "
            f"active_camera={active_camera} "
            f"color_camera={inspection.overlay.color_camera_path}",
            flush=True,
        )
        print(
            "The stage is paused and visual-only. Use the viewport Camera menu "
            "to switch between Perspective and the synthetic 140-degree "
            "ColorOpticalFrame; close Isaac to exit.",
            flush=True,
        )
        completed = 0
        while simulation_app.is_running() and (args.frames == 0 or completed < args.frames):
            simulation_app.update()
            completed += 1
        return 0
    finally:
        if inspection is not None:
            inspection.scene.world.stop()
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
