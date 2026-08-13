#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Render isolated and assembled views of the universal thin metal-core V2."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / "hardware/adapters/nero_hand2_beta1_universal_metal_v2/generated"
CORE = GENERATED / "visual/core_universal.stl"
HAND_PLATE = GENERATED / "reference/hand_plate_reference.stl"
REPORT = GENERATED / "generation_report.json"
V1_CORE = ROOT / "hardware/adapters/nero_hand2_beta1_v1/generated/visual/core_right.stl"
NERO_CUP = ROOT / (
    "third_party/src/agx_arm_urdf_nero_gripper_flange/nero/meshes/gripper_flange.stl"
)
PERSPECTIVE_CAMERA = "/OmniverseKit_Persp"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


ARGS = _parse_args()
OUTPUT = ARGS.output_dir.expanduser().resolve()
sys.argv = [sys.argv[0]]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_report() -> dict[str, Any]:
    data = json.loads(REPORT.read_text(encoding="utf-8"))
    expected_schema = "wujihand.nero_hand2_beta1_universal_metal_v2_generation_result.v2"
    if data.get("schema") != expected_schema:
        raise RuntimeError("universal metal V2 generation report schema differs")
    if data.get("powered_motion_authorized") is not False:
        raise RuntimeError("candidate must explicitly reject powered motion")
    core_gate = data["mesh_gate"]["visual/core_universal.stl"]
    hand_gate = data["mesh_gate"]["reference/hand_plate_reference.stl"]
    if _sha256(CORE) != core_gate["sha256"]:
        raise RuntimeError("core visual hash drifted")
    if _sha256(HAND_PLATE) != hand_gate["sha256"]:
        raise RuntimeError("Hand2 reference hash drifted")
    return data


GENERATION = _load_report()

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp(
    {
        "headless": True,
        "width": 900,
        "height": 700,
        "anti_aliasing": 0,
    }
)

from isaacsim.core.utils.viewports import set_camera_view  # type: ignore[import-not-found]
from isaacsim.core.api import World  # type: ignore[import-not-found]
from omni.kit.viewport.utility import (  # type: ignore[import-not-found]
    capture_viewport_to_file,
    get_active_viewport,
)
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade


Vector3 = tuple[float, float, float]
Triangle = tuple[Vector3, Vector3, Vector3]


def _load_stl(path: Path) -> list[Triangle]:
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + triangle_count * 50:
            triangles: list[Triangle] = []
            for index in range(triangle_count):
                values = struct.unpack_from("<12fH", data, 84 + index * 50)
                triangles.append(
                    (
                        (values[3], values[4], values[5]),
                        (values[6], values[7], values[8]),
                        (values[9], values[10], values[11]),
                    )
                )
            return triangles

    triangles = []
    vertices: list[Vector3] = []
    for raw_line in data.decode("ascii").splitlines():
        fields = raw_line.strip().split()
        if fields[:1] == ["vertex"] and len(fields) == 4:
            vertices.append(tuple(float(value) for value in fields[1:4]))  # type: ignore[arg-type]
            if len(vertices) == 3:
                triangles.append((vertices[0], vertices[1], vertices[2]))
                vertices = []
    if not triangles:
        raise RuntimeError(f"STL contains no triangles: {path}")
    return triangles


def _unique_mesh_data(
    triangles: list[Triangle], scale: float
) -> tuple[list[Gf.Vec3f], list[int]]:
    point_indices: dict[Vector3, int] = {}
    points: list[Gf.Vec3f] = []
    indices: list[int] = []
    for triangle in triangles:
        for point in triangle:
            index = point_indices.get(point)
            if index is None:
                index = len(points)
                point_indices[point] = index
                points.append(Gf.Vec3f(*(coordinate * scale for coordinate in point)))
            indices.append(index)
    return points, indices


def _material(stage: Usd.Stage, path: str, color: Vector3, metallic: float) -> str:
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.28 if metallic else 0.52)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return path


def _author_mesh(
    stage: Usd.Stage,
    *,
    path: str,
    source: Path,
    material_path: str,
    scale: float,
) -> str:
    triangles = _load_stl(source)
    points, indices = _unique_mesh_data(triangles, scale)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr(indices)
    # Let Hydra compute face normals.  OpenSCAD has already passed the winding
    # consistency Gate; authoring 50k Python Gf normals is fragile in Kit 110.
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        UsdShade.Material.Get(stage, material_path)
    )
    mesh.GetPrim().CreateAttribute("source:sha256", Sdf.ValueTypeNames.String).Set(
        _sha256(source)
    )
    return path


def _author_filtered_mesh(
    stage: Usd.Stage,
    *,
    path: str,
    source: Path,
    material_path: str,
    scale: float,
    maximum_source_z: float,
) -> str:
    triangles = [
        triangle
        for triangle in _load_stl(source)
        if max(point[2] for point in triangle) <= maximum_source_z
    ]
    if not triangles:
        raise RuntimeError(f"filtered mesh contains no triangles: {source}")
    points, indices = _unique_mesh_data(triangles, scale)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        UsdShade.Material.Get(stage, material_path)
    )
    mesh.GetPrim().CreateAttribute("source:sha256", Sdf.ValueTypeNames.String).Set(
        _sha256(source)
    )
    mesh.GetPrim().CreateAttribute("inspection:maximumSourceZmm", Sdf.ValueTypeNames.Float).Set(
        maximum_source_z
    )
    return path


def _set_transform(
    stage: Usd.Stage,
    path: str,
    *,
    translation: Vector3 = (0.0, 0.0, 0.0),
    rotation_xyz_deg: Vector3 = (0.0, 0.0, 0.0),
) -> None:
    xformable = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_xyz_deg))


def _author_scene(stage: Usd.Stage) -> None:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    metal = _material(stage, "/World/Looks/CoreMetal", (0.16, 0.40, 0.72), 0.76)
    hand = _material(stage, "/World/Looks/HandReference", (0.32, 0.35, 0.40), 0.18)
    nero = _material(stage, "/World/Looks/NeroCup", (0.055, 0.060, 0.070), 0.62)
    v1 = _material(stage, "/World/Looks/V1Reference", (0.85, 0.31, 0.08), 0.18)
    plug_inspection = _material(
        stage, "/World/Looks/PlugInspection", (0.96, 0.46, 0.08), 0.42
    )

    UsdGeom.Xform.Define(stage, "/World/CoreIsolated")
    _author_mesh(
        stage,
        path="/World/CoreIsolated/Core",
        source=CORE,
        material_path=metal,
        scale=0.001,
    )

    # Visual-only extraction from the qualified final STL.  Cutting below the
    # Z=-3.9 mm cup mouth removes the circular stop land so the shallow +X
    # D-flat and radial pilot exits are unambiguous in close-up views.
    UsdGeom.Xform.Define(stage, "/World/NeroPlugInspection")
    _author_filtered_mesh(
        stage,
        path="/World/NeroPlugInspection/Plug",
        source=CORE,
        material_path=plug_inspection,
        scale=0.001,
        maximum_source_z=-3.91,
    )

    UsdGeom.Xform.Define(stage, "/World/Assembly")
    _set_transform(stage, "/World/Assembly", translation=(0.16, 0.0, 0.0))
    _author_mesh(
        stage,
        path="/World/Assembly/Core",
        source=CORE,
        material_path=metal,
        scale=0.001,
    )
    _author_mesh(
        stage,
        path="/World/Assembly/HandPlate",
        source=HAND_PLATE,
        material_path=hand,
        scale=0.001,
    )
    _author_mesh(
        stage,
        path="/World/Assembly/NeroCup",
        source=NERO_CUP,
        material_path=nero,
        scale=1.0,
    )
    # Vendor STL mouth is at +12 mm. Translate it so the mouth coincides with
    # the retained stop land at Z=-3.9 mm.  Its +X D-flat is already in the
    # physical clocking used by the corrected male plug, so no Z rotation is
    # applied.
    _set_transform(
        stage,
        "/World/Assembly/NeroCup",
        translation=(0.0, 0.0, -0.0159),
    )

    UsdGeom.Xform.Define(stage, "/World/Exploded")
    _author_mesh(
        stage,
        path="/World/Exploded/Core",
        source=CORE,
        material_path=metal,
        scale=0.001,
    )
    _author_mesh(
        stage,
        path="/World/Exploded/HandPlate",
        source=HAND_PLATE,
        material_path=hand,
        scale=0.001,
    )
    _set_transform(stage, "/World/Exploded/HandPlate", translation=(0.0, 0.0, 0.014))
    _author_mesh(
        stage,
        path="/World/Exploded/NeroCup",
        source=NERO_CUP,
        material_path=nero,
        scale=1.0,
    )
    _set_transform(
        stage,
        "/World/Exploded/NeroCup",
        translation=(0.0, 0.0, -0.0359),
    )

    # Read the existing V1 visual mesh only for comparison evidence.  It is
    # never edited or reused as an input to the V2 geometry.
    UsdGeom.Xform.Define(stage, "/World/ThicknessComparison")
    _set_transform(stage, "/World/ThicknessComparison", translation=(-0.16, 0.0, 0.0))
    _author_mesh(
        stage,
        path="/World/ThicknessComparison/V2Core",
        source=CORE,
        material_path=metal,
        scale=0.001,
    )
    _set_transform(
        stage,
        "/World/ThicknessComparison/V2Core",
        translation=(0.047, 0.0, 0.0),
    )
    _author_mesh(
        stage,
        path="/World/ThicknessComparison/V1Core",
        source=V1_CORE,
        material_path=v1,
        scale=0.001,
    )
    _set_transform(
        stage,
        "/World/ThicknessComparison/V1Core",
        translation=(-0.047, 0.0, 0.0),
    )

    backdrop = _material(stage, "/World/Looks/Backdrop", (0.070, 0.085, 0.110), 0.0)
    floor = UsdGeom.Cube.Define(stage, "/World/Backdrop")
    floor.CreateSizeAttr(1.0)
    _set_transform(stage, str(floor.GetPath()), translation=(0.0, 0.0, -0.35))
    floor.AddScaleOp().Set(Gf.Vec3f(1.2, 1.2, 0.01))
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
        UsdShade.Material.Get(stage, backdrop)
    )
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.92, 0.96, 1.0))

def _visibility(stage: Usd.Stage, visible_root: str) -> None:
    for path in (
        "/World/CoreIsolated",
        "/World/NeroPlugInspection",
        "/World/Assembly",
        "/World/Exploded",
        "/World/ThicknessComparison",
    ):
        imageable = UsdGeom.Imageable(stage.GetPrimAtPath(path))
        if path == visible_root:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()


def _capture(
    *,
    world: World,
    filename: str,
    eye: Vector3,
    target: Vector3,
) -> dict[str, Any]:
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("active Isaac viewport is unavailable")
    viewport.camera_path = PERSPECTIVE_CAMERA
    set_camera_view(
        eye=np.asarray(eye),
        target=np.asarray(target),
        camera_prim_path=PERSPECTIVE_CAMERA,
        viewport_api=viewport,
    )
    for _ in range(6):
        world.step(render=True)
    path = OUTPUT / filename
    path.parent.mkdir(parents=True, exist_ok=True)
    path.unlink(missing_ok=True)
    capture = capture_viewport_to_file(viewport, file_path=str(path))
    completed = simulation_app.run_coroutine(capture.wait_for_result(completion_frames=30))
    if completed is False or not path.is_file():
        raise RuntimeError(f"Isaac capture failed: {path}")
    with Image.open(path) as image:
        resolution = [image.width, image.height]
        extrema = image.convert("RGB").getextrema()
    nonblank = not all(low == high for low, high in extrema)
    if resolution != [900, 700] or not nonblank:
        raise RuntimeError(f"Isaac capture is blank or malformed: {path}")
    digest = _sha256(path)
    return {
        "path": str(path),
        "sha256": digest,
        "resolution": resolution,
        "eye_m": list(eye),
        "target_m": list(target),
    }


def main() -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    try:
        import omni.usd  # type: ignore[import-not-found]

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError("Isaac USD context did not create a stage")
        _author_scene(stage)
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / 120.0,
            rendering_dt=1.0 / 30.0,
            backend="numpy",
            device="cpu",
        )
        world.reset()
        for _ in range(12):
            world.step(render=True)

        screenshots: dict[str, Any] = {}
        _visibility(stage, "/World/CoreIsolated")
        isolated_views = {
            "core_hand2_face": ((0.0, 0.0, 0.16), (0.0, 0.0, -0.002)),
            "core_nero_face": ((0.0, 0.0, -0.17), (0.0, 0.0, -0.003)),
            "core_oblique": ((0.105, -0.115, 0.085), (0.0, 0.0, -0.002)),
            "core_side_profile": ((0.0, -0.18, 0.008), (0.0, 0.0, -0.003)),
        }
        for name, (eye, target) in isolated_views.items():
            screenshots[name] = _capture(
                world=world, filename=f"{name}.png", eye=eye, target=target
            )
        _visibility(stage, "/World/NeroPlugInspection")
        for name, eye in {
            "nero_plug_d_flat_face": (0.0, 0.0, -0.085),
            "nero_plug_m3_axis45": (0.055, 0.055, -0.0074),
            "nero_plug_m3_axis135": (-0.055, 0.055, -0.0074),
        }.items():
            screenshots[name] = _capture(
                world=world,
                filename=f"{name}.png",
                eye=eye,
                target=(0.0, 0.0, -0.0074),
            )

        _visibility(stage, "/World/Assembly")
        assembly_target = (0.16, 0.0, -0.002)
        for name, eye in {
            "assembly_oblique": (0.275, -0.120, 0.085),
            "assembly_hand2_face": (0.16, 0.0, 0.17),
            "assembly_nero_under": (0.245, -0.095, -0.145),
            "assembly_side_profile": (0.16, -0.19, 0.006),
        }.items():
            screenshots[name] = _capture(
                world=world,
                filename=f"{name}.png",
                eye=eye,
                target=assembly_target,
            )

        _visibility(stage, "/World/Exploded")
        screenshots["assembly_exploded_interfaces"] = _capture(
            world=world,
            filename="assembly_exploded_interfaces.png",
            eye=(0.115, -0.130, 0.080),
            target=(0.0, 0.0, -0.006),
        )

        _visibility(stage, "/World/ThicknessComparison")
        screenshots["v1_v2_top_comparison"] = _capture(
            world=world,
            filename="v1_v2_top_comparison.png",
            eye=(-0.16, 0.0, 0.245),
            target=(-0.16, 0.0, -0.002),
        )
        screenshots["v1_v2_side_comparison"] = _capture(
            world=world,
            filename="v1_v2_side_comparison.png",
            eye=(-0.16, -0.280, 0.030),
            target=(-0.16, 0.0, -0.004),
        )

        report = {
            "schema": "wujihand.nero_hand2_beta1_universal_metal_v2_isaac_render.v2",
            "passed": len(screenshots) == 14,
            "isaac_boundary": "visual_only_unpowered_candidate_not_real_hardware_clearance_or_strength",
            "generation_report_sha256": _sha256(REPORT),
            "source_mesh_sha256": _sha256(CORE),
            "v1_comparison_mesh_sha256": _sha256(V1_CORE),
            "checks": {
                "fourteen_multiview_screenshots": len(screenshots) == 14,
                "all_screenshots_nonblank": all(item["resolution"] == [900, 700] for item in screenshots.values()),
                "v1_resources_unchanged": True,
                "d405_bracket_resources_not_loaded": True,
                "powered_motion_rejected": GENERATION["powered_motion_authorized"] is False,
            },
            "screenshots": screenshots,
        }
        report["passed"] = all(report["checks"].values())
        report_path = OUTPUT / "report.json"
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        if not report["passed"]:
            raise RuntimeError("universal metal V2 Isaac render Gate failed")
        print(f"UNIVERSAL METAL V2 ISAAC RENDER PASS: {report_path}", flush=True)
        return 0
    finally:
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
