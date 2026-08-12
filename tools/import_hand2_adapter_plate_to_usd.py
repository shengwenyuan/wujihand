#!/usr/bin/env python3
# ruff: noqa: E402  # Project modules are imported after adding src to sys.path.
"""Add the pinned Hand2 capsule-slot plate to the v2026.8.3 Isaac assets."""

from __future__ import annotations

import argparse
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.integrity import sha256_file
from wujihand.runtime import ConfigRepository, SourceLock


DEFAULT_PROFILE = ROOT / (
    "configs/profiles/wuji_hand2_adapter_plate_isaac_6_0_1_import_v1.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    return parser.parse_args()


def _extract_plate_triangles(
    step_path: Path,
    *,
    expected_volume_mm3: float,
    origin_mm: tuple[float, float, float],
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GProp import GProp_GProps
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise RuntimeError(f"failed to read STEP: {step_path}")
    reader.TransferRoots()
    explorer = TopExp_Explorer(reader.OneShape(), TopAbs_SOLID)
    matches = []
    while explorer.More():
        shape = explorer.Current()
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, properties)
        if math.isclose(properties.Mass(), expected_volume_mm3, abs_tol=1e-3):
            matches.append(shape)
        explorer.Next()
    if len(matches) != 1:
        raise RuntimeError(f"expected one adapter plate solid, found {len(matches)}")

    BRepMesh_IncrementalMesh(matches[0], 0.05, False, 0.3, False).Perform()
    with tempfile.TemporaryDirectory(prefix="hand2-adapter-plate-") as directory:
        stl_path = Path(directory) / "plate.stl"
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        if not writer.Write(matches[0], str(stl_path)):
            raise RuntimeError("failed to tessellate Hand2 adapter plate")
        data = stl_path.read_bytes()

    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangle_count * 50:
        raise RuntimeError("unexpected adapter plate STL encoding")
    points: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    ox, oy, oz = origin_mm
    for index in range(triangle_count):
        values = struct.unpack_from("<12fH", data, 84 + index * 50)
        normal = tuple(float(value) for value in values[:3])
        for offset in (3, 6, 9):
            x, y, z = values[offset : offset + 3]
            points.append(((x - ox) * 0.001, (y - oy) * 0.001, (z - oz) * 0.001))
            normals.append(normal)
    return points, normals


def _write_plate(path: Path, points, normals, approximation: str) -> None:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    mesh = UsdGeom.Mesh.Define(stage, "/Hand2AdapterPlate")
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr([3] * (len(points) // 3))
    mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
    mesh.CreateNormalsAttr([Gf.Vec3f(*normal) for normal in normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(
        [Gf.Vec3f(0.56, 0.58, 0.62)]
    )
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        approximation
    )
    stage.SetDefaultPrim(mesh.GetPrim())
    stage.GetRootLayer().Save()


def _write_wrapper(
    path: Path,
    *,
    source_usd: Path,
    side: str,
    wrist_prim: str,
    quat_wxyz: list[float],
    plate_path: Path,
) -> None:
    from pxr import Gf, Usd, UsdGeom

    root_name = f"wujihand2_{side}"
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}").GetPrim()
    root.GetReferences().AddReference(os.path.relpath(source_usd, path.parent))
    stage.SetDefaultPrim(root)
    plate = stage.OverridePrim(f"/{root_name}/{wrist_prim}/adapter_plate")
    plate.GetReferences().AddReference(os.path.relpath(plate_path, path.parent))
    w, x, y, z = quat_wxyz
    UsdGeom.Xformable(plate).AddOrientOp().Set(Gf.Quatf(w, x, y, z))
    stage.GetRootLayer().Save()


def main() -> int:
    args = parse_args()
    profile_path = args.profile.resolve()
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile["schema"] != "wujihand.hand2_adapter_plate_import.v1":
        raise RuntimeError("unsupported Hand2 adapter plate profile")

    lock = SourceLock.load(ConfigRepository(ROOT))
    plate_source = profile["plate_source"]
    plate_record = lock.record(plate_source["lock_id"])
    if dict(plate_record.revision)["commit"] != plate_source["commit"]:
        raise RuntimeError("Hand2 adapter plate source revision drifted")
    step_path = ROOT / plate_record.local_runtime_path / plate_source["step_path"]
    if sha256_file(step_path) != plate_source["step_sha256"]:
        raise RuntimeError("Hand2 adapter plate STEP hash drifted")

    hand_source = profile["hand_source"]
    hand_record = lock.record(hand_source["lock_id"])
    if dict(hand_record.revision)["commit"] != hand_source["commit"]:
        raise RuntimeError("Hand2 v2026.8.3 source revision drifted")
    hand_root = ROOT / hand_record.local_runtime_path
    for evidence in hand_source["whole_hand_step"].values():
        if sha256_file(hand_root / evidence["path"]) != evidence["sha256"]:
            raise RuntimeError("Hand2 v2026.8.3 whole-hand STEP drifted")

    output = profile["output"]
    output_root = ROOT / output["root"]
    if output_root.exists():
        raise FileExistsError(f"refusing to replace {output_root}")
    output_root.mkdir(parents=True)
    plate_path = output_root / output["plate"]
    plate_path.parent.mkdir(parents=True, exist_ok=True)
    points, normals = _extract_plate_triangles(
        step_path,
        expected_volume_mm3=float(plate_source["solid_volume_mm3"]),
        origin_mm=tuple(float(value) for value in plate_source["outer_face_origin_mm"]),
    )
    _write_plate(plate_path, points, normals, profile["collision_approximation"])

    for side in ("left", "right"):
        destination = output_root / output[side]
        destination.parent.mkdir(parents=True, exist_ok=True)
        placement = profile["plate_frame_in_hand"][side]
        _write_wrapper(
            destination,
            source_usd=hand_root / hand_source["usd"][side],
            side=side,
            wrist_prim=placement["wrist_prim"],
            quat_wxyz=placement["quat_wxyz"],
            plate_path=plate_path,
        )

    report = {
        "schema": "wujihand.hand2_adapter_plate_import_result.v1",
        "profile_path": profile_path.relative_to(ROOT).as_posix(),
        "profile_sha256": sha256_file(profile_path),
        "cadquery_ocp_version": version("cadquery-ocp"),
        "plate_source_sha256": sha256_file(step_path),
        "hand_source_step_sha256": {
            side: evidence["sha256"]
            for side, evidence in hand_source["whole_hand_step"].items()
        },
        "outputs": {
            key: {
                "path": output[key],
                "sha256": sha256_file(output_root / output[key]),
            }
            for key in ("plate", "left", "right")
        },
        "assumptions": profile["assumptions"],
    }
    report_path = output_root / output["report"]
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
