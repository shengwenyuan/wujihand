#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Validate a resolved Workcell in Isaac Sim without injecting a robot."""

from __future__ import annotations

import argparse
from importlib.metadata import version
import json
from pathlib import Path
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime import SessionResolver, resolve_isaac_workcell_plan
from wujihand.runtime.isaac_dual_scene import workcell_frame_position


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--report", type=Path)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument(
        "--verify-content",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--camera-eye-frame",
        default="simulation_nominal_camera_oblique_eye",
    )
    parser.add_argument(
        "--camera-target-frame",
        default="simulation_nominal_camera_oblique_target",
    )
    return parser.parse_args()


ARGS = parse_args()
if ARGS.frames < 1:
    raise SystemExit("--frames must be positive")
RESOLVED = SessionResolver(ROOT).resolve(ARGS.session)
if RESOLVED.session.backend != "isaac":
    raise SystemExit("Workcell validation requires an Isaac Session")
PLAN = resolve_isaac_workcell_plan(
    ROOT,
    RESOLVED.workcell,
    verify_content=ARGS.verify_content,
)

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp(
    {
        "headless": not ARGS.gui,
        "width": 1280,
        "height": 800,
        "anti_aliasing": 0,
    }
)

import omni.kit.renderer_capture  # type: ignore[import-not-found]
from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
    capture_viewport_to_file,
    get_active_viewport,
)
from pxr import Usd, UsdGeom  # type: ignore[import-not-found]
from isaacsim.core.api import World  # type: ignore[import-not-found]
from isaacsim.core.utils.viewports import (  # type: ignore[import-not-found]
    set_camera_view,
)
from wujihand.runtime.isaac_workcell import materialize_isaac_workcell


def main() -> int:
    started = time.monotonic()
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 30.0,
        backend="numpy",
        device="cpu",
    )
    materialized = materialize_isaac_workcell(world, PLAN)
    materialized_at = time.monotonic()
    eye = workcell_frame_position(RESOLVED, ARGS.camera_eye_frame)
    target = workcell_frame_position(RESOLVED, ARGS.camera_target_frame)
    set_camera_view(
        eye=np.asarray(eye),
        target=np.asarray(target),
        camera_prim_path="/OmniverseKit_Persp",
    )
    world.reset()
    for frame in range(ARGS.frames):
        world.step(
            render=(
                ARGS.gui
                or ARGS.screenshot is not None
                and frame % 4 == 0
            )
        )
    stepped_at = time.monotonic()

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    rigid_body_transforms: dict[str, list[float]] = {}
    for path in materialized.rigid_body_paths:
        matrix = cache.GetLocalToWorldTransform(world.scene.stage.GetPrimAtPath(path))
        translation = np.asarray(tuple(matrix.ExtractTranslation()), dtype=np.float64)
        if not np.isfinite(translation).all():
            raise RuntimeError(f"non-finite rigid body transform: {path}")
        rigid_body_transforms[path] = translation.tolist()

    screenshot: str | None = None
    if ARGS.screenshot is not None:
        path = ARGS.screenshot.resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(8):
            world.step(render=True)
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("active Isaac viewport is unavailable")
        capture = capture_viewport_to_file(viewport, file_path=str(path))
        captured = simulation_app.run_coroutine(
            capture.wait_for_result(completion_frames=30)
        )
        omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
        if not captured or not path.is_file():
            raise RuntimeError("Isaac viewport capture did not complete")
        screenshot = str(path)

    report = {
        "schema": "wujihand.isaac_workcell_smoke.v1",
        "session_id": RESOLVED.session.session_id,
        "session_hash": RESOLVED.session_hash,
        "isaac_distribution": version("isaacsim"),
        "frames": ARGS.frames,
        "camera_eye_m": list(eye),
        "camera_target_m": list(target),
        "materialization": materialized.to_mapping(),
        "rigid_body_transforms_m": rigid_body_transforms,
        "materialization_s": materialized_at - started,
        "stepping_s": stepped_at - materialized_at,
        "screenshot": screenshot,
    }
    if ARGS.report is None:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        report_path = ARGS.report.resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"ISAAC WORKCELL PASS: report={report_path}")
    world.stop()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
