#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Open the provisional T-frame layout without starting any input or hardware path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation.nero_link_geometry_alignment import (
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_startup import (
    load_nero_dual_simulation_startup_profile,
)
from wujihand.integrity import sha256_file
from wujihand.runtime import SessionResolver, resolve_isaac_workcell_plan
from wujihand.runtime.isaac_dual_scene import (
    resolve_dual_side_runtimes,
    workcell_frame_position,
)


DEFAULT_SESSION = ROOT / (
    "configs/sessions/isaac_nero_dual_hand2_tframe_inspection_v2026_8_3_v1.yaml"
)
CAMERA_PATH = "/OmniverseKit_Persp"
CAMERA_EYE_FRAME = "simulation_nominal_camera_oblique_eye"
CAMERA_TARGET_FRAME = "simulation_nominal_camera_oblique_target"
INTERFACE_CAMERA_EYE_FRAME = "simulation_nominal_camera_right_interface_eye"
INTERFACE_CAMERA_TARGET_FRAME = "simulation_nominal_camera_right_interface_target"
INITIALIZATION_FRAMES = 240


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, default=DEFAULT_SESSION)
    parser.add_argument(
        "--task-scene",
        type=Path,
        help="Layer an independently selected task scene over the T-frame Workcell.",
    )
    parser.add_argument("--gui", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Positive values close automatically; zero keeps the GUI open.",
    )
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--interface-screenshot", type=Path)
    parser.add_argument(
        "--wrist-screenshot-dir",
        type=Path,
        help="Capture the two synthetic D405 optical views after settling.",
    )
    parser.add_argument(
        "--visual-only",
        action="store_true",
        help="Disable imported T-frame colliders for layout isolation.",
    )
    parser.add_argument(
        "--verify-artifacts",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames < 0 or (not ARGS.gui and ARGS.frames == 0):
    raise SystemExit("headless inspection requires --frames > 0")
if (
    ARGS.screenshot is not None
    or ARGS.interface_screenshot is not None
    or ARGS.wrist_screenshot_dir is not None
) and not ARGS.gui:
    raise SystemExit("viewport capture requires --gui")

RESOLVED = SessionResolver(ROOT).resolve(
    ARGS.session,
    verify_artifacts=ARGS.verify_artifacts,
    overrides=(None if ARGS.task_scene is None else {"task_scene": ARGS.task_scene}),
)
WORKCELL_PLAN = resolve_isaac_workcell_plan(
    ROOT,
    RESOLVED.workcell,
    task_scene=ARGS.task_scene,
    verify_content=ARGS.verify_artifacts,
)
startup_path = RESOLVED.session.runtime.compatibility_profile
if startup_path is None:
    raise SystemExit("T-frame inspection Session has no startup profile")
STARTUP = load_nero_dual_simulation_startup_profile(ROOT / startup_path)
SIDES = resolve_dual_side_runtimes(ROOT, RESOLVED)
alignment_paths = {
    RESOLVED.instance(side.arm_instance_id).binding.compatibility_profile for side in SIDES
}
if len(alignment_paths) != 1:
    raise SystemExit("T-frame inspection requires one shared NERO alignment selection")
alignment_path = next(iter(alignment_paths))
ALIGNMENT = (
    None
    if alignment_path is None
    else load_nero_link_geometry_alignment(ROOT / alignment_path)
)
if ALIGNMENT is not None and (
    sha256_file(ROOT / ALIGNMENT.source_urdf_path) != ALIGNMENT.source_urdf_sha256
):
    raise SystemExit("pinned NERO URDF hash drifted")

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp(
    {
        "headless": not ARGS.gui,
        "width": 1280,
        "height": 800,
        "anti_aliasing": 0,
    }
)

from isaacsim.core.utils.viewports import set_camera_view  # type: ignore[import-not-found]
from wujihand.runtime.isaac_dual_scene import DualNeroHand2IsaacScene


def capture(path: Path, *, camera_path: str = CAMERA_PATH) -> None:
    import omni.kit.renderer_capture  # type: ignore[import-not-found]
    from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
        capture_viewport_to_file,
        get_active_viewport,
    )

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    viewport.camera_path = camera_path
    for _ in range(4):
        simulation_app.update()
    path.parent.mkdir(parents=True, exist_ok=True)
    result = capture_viewport_to_file(viewport, file_path=str(path))
    captured = simulation_app.run_coroutine(result.wait_for_result(completion_frames=30))
    omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
    if not captured or not path.is_file():
        raise RuntimeError("T-frame viewport capture did not complete")


def disable_imported_colliders(scene: DualNeroHand2IsaacScene) -> int:
    from pxr import UsdPhysics

    imported = scene.workcell_materialization.imported_prim_paths
    paths = tuple(
        path
        for path in scene.workcell_materialization.fixed_collider_paths
        if any(path.startswith(f"{root}/") for root in imported)
    )
    for path in paths:
        UsdPhysics.CollisionAPI(scene.stage.GetPrimAtPath(path)).GetCollisionEnabledAttr().Set(
            False
        )
    return len(paths)


def main() -> int:
    scene = DualNeroHand2IsaacScene(
        project_root=ROOT,
        resolved=RESOLVED,
        sides=SIDES,
        alignment_profile=ALIGNMENT,
        qualification_profile=STARTUP,
        physics_hz=120,
        self_collision_sides=frozenset(),
        wrist_rig_collision_mode="all",
        workcell_plan=WORKCELL_PLAN,
    )
    set_camera_view(
        eye=np.asarray(workcell_frame_position(RESOLVED, CAMERA_EYE_FRAME)),
        target=np.asarray(workcell_frame_position(RESOLVED, CAMERA_TARGET_FRAME)),
        camera_prim_path=CAMERA_PATH,
    )
    disabled_collider_count = disable_imported_colliders(scene) if ARGS.visual_only else 0
    scene.teleport_to_targets()

    initialization_frames = (
        min(INITIALIZATION_FRAMES, ARGS.frames) if ARGS.frames > 0 else INITIALIZATION_FRAMES
    )
    for _ in range(initialization_frames):
        scene.apply_targets()
        scene.world.step(render=ARGS.gui)

    remaining = max(0, ARGS.frames - initialization_frames)
    for _ in range(remaining):
        scene.apply_targets()
        scene.world.step(render=ARGS.gui)

    if ARGS.screenshot is not None:
        capture(ARGS.screenshot)

    if ARGS.interface_screenshot is not None:
        set_camera_view(
            eye=np.asarray(workcell_frame_position(RESOLVED, INTERFACE_CAMERA_EYE_FRAME)),
            target=np.asarray(workcell_frame_position(RESOLVED, INTERFACE_CAMERA_TARGET_FRAME)),
            camera_prim_path=CAMERA_PATH,
        )
        capture(ARGS.interface_screenshot)
        set_camera_view(
            eye=np.asarray(workcell_frame_position(RESOLVED, CAMERA_EYE_FRAME)),
            target=np.asarray(workcell_frame_position(RESOLVED, CAMERA_TARGET_FRAME)),
            camera_prim_path=CAMERA_PATH,
        )

    wrist_screenshots: dict[str, str] = {}
    if ARGS.wrist_screenshot_dir is not None:
        for handles in scene.wrist_rigs:
            path = ARGS.wrist_screenshot_dir / f"{handles.side}_optical.png"
            capture(path, camera_path=handles.camera_prim_path)
            wrist_screenshots[handles.side] = str(path.resolve())
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = CAMERA_PATH

    summary = {
        "session_id": RESOLVED.session.session_id,
        "workcell_id": RESOLVED.workcell.workcell_id,
        "task_scene_profile_id": WORKCELL_PLAN.task_scene_profile_id,
        "task_scene_profile_path": WORKCELL_PLAN.task_scene_profile_path,
        "environment_imports": [operation.import_id for operation in WORKCELL_PLAN.imports],
        "startup_profile_id": STARTUP.profile_id,
        "status": STARTUP.status,
        "hardware_access": False,
        "visual_only": ARGS.visual_only,
        "disabled_collider_count": disabled_collider_count,
        "mounts": {
            side.side: {
                "position_m": list(side.mount_pose.position_m),
                "quat_wxyz": list(side.mount_pose.quat_wxyz),
            }
            for side in SIDES
        },
        "fixed_collider_count": len(scene.workcell_materialization.fixed_collider_paths),
        "q7_max_error_rad": {
            side: float(
                np.max(np.abs(scene.feedback_q27(side)[:7] - scene.initial_arm_targets[side]))
            )
            for side in ("left", "right")
        },
        "q7_feedback_rad": {
            side: scene.feedback_q27(side)[:7].tolist() for side in ("left", "right")
        },
        "screenshot": None if ARGS.screenshot is None else str(ARGS.screenshot.resolve()),
        "interface_screenshot": (
            None if ARGS.interface_screenshot is None else str(ARGS.interface_screenshot.resolve())
        ),
        "wrist_screenshots": wrist_screenshots,
    }
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True), flush=True)

    if ARGS.frames == 0:
        print("T-FRAME INSPECTION READY: close Isaac Sim to exit.", flush=True)
        while simulation_app.is_running():
            scene.apply_targets()
            scene.world.step(render=True)
    return 0


try:
    exit_code = main()
finally:
    simulation_app.close()
raise SystemExit(exit_code)
