#!/usr/bin/env python3
"""Build and qualify independent left/right D405 wrist-rig assets."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
from typing import cast

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation.d405_wrist_rig_assets import (  # noqa: E402
    Matrix3,
    Triangle,
    Vector3,
    audit_collision_proxy,
    audit_mesh,
    load_stl_triangles,
    optical_frame_contract,
    rotation_matrix_z_y,
    transform_triangles,
    write_binary_stl,
)
from wujihand.integrity import sha256_file  # noqa: E402
from wujihand.runtime.config_repository import ConfigRepository  # noqa: E402
from wujihand.runtime.source_lock import SourceLock  # noqa: E402


DEFAULT_RECIPE = ROOT / "configs/profiles/isaac_d405_wrist_rig_asset_generation_v1.yaml"
GENERATOR_PATH = Path(__file__).resolve()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recipe", type=Path, default=DEFAULT_RECIPE)
    parser.add_argument("--openscad", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    return cast(Mapping[str, object], value)


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _number(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _vector3(value: object, *, field: str) -> Vector3:
    items = _sequence(value, field=field)
    if len(items) != 3:
        raise ValueError(f"{field} must contain exactly three values")
    return cast(
        Vector3,
        tuple(_number(item, field=f"{field}[{index}]") for index, item in enumerate(items)),
    )


def _load_recipe(path: Path) -> Mapping[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    recipe = _mapping(document, field="recipe")
    if recipe.get("schema") != "wujihand.d405_wrist_rig_asset_generation.v1":
        raise ValueError("unsupported D405 wrist-rig asset recipe schema")
    if recipe.get("status") != "simulation_only":
        raise ValueError("D405 wrist-rig recipe must remain simulation_only")
    warning = _string(recipe.get("camera_warning"), field="camera_warning")
    if "synthetic 140-degree HFOV" not in warning or "not a physical" not in warning:
        raise ValueError("camera_warning must preserve the synthetic 140-degree boundary")
    return recipe


def _openscad_command(explicit: Path | None) -> list[str]:
    executable = explicit or (
        Path(found) if (found := shutil.which("openscad")) is not None else None
    )
    if executable is None:
        raise FileNotFoundError("OpenSCAD is required to generate wrist-mount meshes")
    if platform.system() == "Darwin":
        return ["/usr/bin/arch", "-x86_64", str(executable)]
    return [str(executable)]


def _tool_version(command: Sequence[str]) -> str:
    completed = subprocess.run(
        [*command, "--version"],
        check=True,
        capture_output=True,
        text=True,
    )
    output = (completed.stdout + completed.stderr).strip()
    prefix = "OpenSCAD version "
    if not output.startswith(prefix):
        raise RuntimeError(f"unexpected OpenSCAD version output: {output!r}")
    return output.removeprefix(prefix)


def _export_mount(
    command: Sequence[str],
    *,
    scad_path: Path,
    side: str,
    destination: Path,
) -> tuple[Triangle, ...]:
    raw_path = destination.with_suffix(".openscad.stl")
    destination.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            *command,
            "-D",
            "show_reference_preview=false",
            "-D",
            f'mount_side="{side}"',
            "-o",
            str(raw_path),
            str(scad_path),
        ],
        check=True,
    )
    triangles = load_stl_triangles(raw_path)
    write_binary_stl(
        destination,
        triangles,
        header=f"Wuji Hand2 D405 mount v2 {side}; millimetres",
    )
    raw_path.unlink()
    return load_stl_triangles(destination)


def _aligned_d405(
    source: Sequence[Triangle],
    *,
    side: str,
) -> tuple[Triangle, ...]:
    aligned = transform_triangles(
        source,
        rotation=((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        translation_mm=(0.0, 0.0, 23.0),
    )
    if side == "right":
        return aligned
    if side != "left":
        raise ValueError(f"unsupported side: {side!r}")
    return transform_triangles(
        aligned,
        rotation=((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
        reverse_winding=True,
    )


def _camera_point(
    point: Vector3,
    *,
    rotation: Matrix3,
    translation: Vector3,
) -> Vector3:
    return cast(
        Vector3,
        tuple(
            translation[row] + sum(rotation[row][column] * point[column] for column in range(3))
            for row in range(3)
        ),
    )


def _box_between_xy(
    *,
    name: str,
    start: Vector3,
    end: Vector3,
    width_mm: float,
    thickness_mm: float,
) -> dict[str, object]:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)
    cosine, sine = math.cos(angle), math.sin(angle)
    return {
        "name": name,
        "type": "box",
        "center_mm": [
            (start[0] + end[0]) / 2.0,
            (start[1] + end[1]) / 2.0,
            (start[2] + end[2]) / 2.0,
        ],
        "size_mm": [length, width_mm, thickness_mm],
        "rotation": [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
    }


def _build_right_mount_proxy(recipe: Mapping[str, object]) -> dict[str, object]:
    mount = _mapping(recipe.get("mount"), field="mount")
    placement = _mapping(
        mount.get("camera_interface_in_right_hand"),
        field="mount.camera_interface_in_right_hand",
    )
    translation = _vector3(placement.get("translation_mm"), field="camera translation")
    rotation = rotation_matrix_z_y(
        _number(placement.get("azimuth_deg"), field="camera azimuth"),
        _number(placement.get("tilt_deg"), field="camera tilt"),
    )
    plate = _mapping(mount.get("plate"), field="mount.plate")
    plate_size = _vector3(plate.get("size_mm"), field="mount.plate.size_mm")
    base = _mapping(mount.get("base"), field="mount.base")
    thickness = _number(base.get("thickness_mm"), field="base thickness")
    rail_center = _vector3(base.get("rail_center_mm"), field="base rail center")
    rail_size = _vector3(base.get("rail_size_mm"), field="base rail size")
    flange_centers = tuple(
        _vector3(value, field="flange center")
        for value in _sequence(
            base.get("flange_centers_right_mm"), field="flange centers"
        )
    )
    if len(flange_centers) != 2:
        raise ValueError("the mount requires exactly two flange centres")
    key_size = _vector3(base.get("key_size_mm"), field="base key size")
    key_center_z = _number(base.get("key_center_z_mm"), field="base key center z")
    target_y = _number(base.get("connector_target_y_mm"), field="connector target y")
    connector_width = _number(base.get("connector_width_mm"), field="connector width")
    primitives: list[dict[str, object]] = [
        {
            "name": "base_outer_rail",
            "type": "box",
            "center_mm": list(rail_center),
            "size_mm": list(rail_size),
        }
    ]
    for index, center in enumerate(flange_centers):
        target = (math.copysign(22.0, center[0]), target_y, center[2])
        primitives.append(
            _box_between_xy(
                name=f"base_connector_{index}",
                start=center,
                end=target,
                width_mm=connector_width,
                thickness_mm=thickness,
            )
        )
        key_angle = math.radians(math.copysign(45.0, center[0]))
        cosine, sine = math.cos(key_angle), math.sin(key_angle)
        primitives.append(
            {
                "name": f"flange_key_{index}",
                "type": "box",
                "center_mm": [center[0], center[1], key_center_z],
                "size_mm": list(key_size),
                "rotation": [
                    [cosine, -sine, 0.0],
                    [sine, cosine, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            }
        )

    struts = _mapping(mount.get("struts"), field="mount.struts")
    base_points = tuple(
        _vector3(value, field="strut base point")
        for value in _sequence(struts.get("base_points_right_mm"), field="strut base points")
    )
    anchors = tuple(
        _number(value, field="camera anchor")
        for value in _sequence(struts.get("camera_anchor_y_mm"), field="camera anchors")
    )
    radii = tuple(
        _number(value, field="strut radius")
        for value in _sequence(struts.get("radius_mm"), field="strut radii")
    )
    if not (len(base_points) == len(anchors) == len(radii) == 4):
        raise ValueError("the accepted mount must retain four routed struts")
    route_x = _number(struts.get("route_x_mm"), field="strut route x")
    route_z = _number(struts.get("route_z_mm"), field="strut route z")
    plate_z = _number(struts.get("plate_z_mm"), field="strut plate z")
    for index, (start, anchor, radius) in enumerate(
        zip(base_points, anchors, radii, strict=True)
    ):
        route = _camera_point(
            (route_x, anchor, route_z), rotation=rotation, translation=translation
        )
        plate_point = _camera_point(
            (0.0, anchor, plate_z), rotation=rotation, translation=translation
        )
        primitives.extend(
            [
                {
                    "name": f"strut_{index}_base_to_route",
                    "type": "capsule_segment",
                    "start_mm": list(start),
                    "end_mm": list(route),
                    "radius_mm": radius,
                },
                {
                    "name": f"strut_{index}_route_to_plate",
                    "type": "capsule_segment",
                    "start_mm": list(route),
                    "end_mm": list(plate_point),
                    "radius_mm": radius,
                },
            ]
        )
    plate_center = _camera_point(
        (0.0, 0.0, plate_size[2] / 2.0),
        rotation=rotation,
        translation=translation,
    )
    primitives.append(
        {
            "name": "camera_plate",
            "type": "box",
            "center_mm": list(plate_center),
            "size_mm": list(plate_size),
            "rotation": [list(row) for row in rotation],
        }
    )
    clear_points = [
        list(_vector3(value, field="required clear point"))
        for value in _sequence(
            mount.get("required_clear_points_right_mm"),
            field="required clear points",
        )
    ]
    return {
        "schema": "wujihand.compound_collision_proxy.v1",
        "component": "nero_hand2_beta1_d405_mount_v2",
        "side": "right",
        "canonical_frame": "hand_interface",
        "units": "mm",
        "rigid_body_policy": "child_shapes_only_no_mass_or_rigid_body_api",
        "primitives": primitives,
        "required_clear_points_mm": clear_points,
    }


def _mirror_proxy(proxy: Mapping[str, object], *, side: str) -> dict[str, object]:
    mirrored = dict(proxy)
    mirrored["side"] = side
    primitives: list[dict[str, object]] = []
    for raw_primitive in _sequence(proxy.get("primitives"), field="proxy primitives"):
        primitive = dict(_mapping(raw_primitive, field="proxy primitive"))
        for field in ("center_mm", "start_mm", "end_mm"):
            if field in primitive:
                point = _vector3(primitive[field], field=field)
                primitive[field] = [point[0], -point[1], point[2]]
        if "rotation" in primitive:
            raw_rows = _sequence(primitive["rotation"], field="primitive rotation")
            rotation = cast(
                Matrix3,
                tuple(_vector3(row, field="primitive rotation row") for row in raw_rows),
            )
            signs = (1.0, -1.0, 1.0)
            primitive["rotation"] = [
                [
                    signs[row] * rotation[row][column] * signs[column]
                    for column in range(3)
                ]
                for row in range(3)
            ]
        primitives.append(primitive)
    mirrored["primitives"] = primitives
    mirrored["required_clear_points_mm"] = [
        [point[0], -point[1], point[2]]
        for point in (
            _vector3(value, field="required clear point")
            for value in _sequence(
                proxy.get("required_clear_points_mm"), field="required clear points"
            )
        )
    ]
    return mirrored


def _d405_proxy(*, side: str, optical: Mapping[str, object]) -> dict[str, object]:
    optical_side = _mapping(optical.get(side), field=f"optical.{side}")
    return {
        "schema": "wujihand.compound_collision_proxy.v1",
        "component": "realsense_d405_housing_sim",
        "side": side,
        "canonical_frame": "rear_mount",
        "units": "mm",
        "rigid_body_policy": "child_shapes_only_no_mass_or_rigid_body_api",
        "frames": {
            "rear_mount_to_optical": optical_side["rear_mount_to_optical"],
        },
        "primitives": [
            {
                "name": "housing_box",
                "type": "box",
                "center_mm": [0.0, 0.0, 11.5],
                "size_mm": [42.0, 42.0, 23.0],
            }
        ],
        "required_clear_points_mm": [],
    }


def _dump_yaml(path: Path, document: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(dict(document), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def _assert_mesh_gate(name: str, audit: Mapping[str, object]) -> None:
    if (
        audit["body_count"] != 1
        or audit["shared_edge_component_count"] != 1
        or audit["watertight"] is not True
        or audit["winding_consistent"] is not True
        or audit["degenerate_triangle_count"] != 0
    ):
        raise RuntimeError(f"{name} failed the single watertight body gate: {audit}")


def _mirror_bounds_error(right: Sequence[Sequence[float]], left: Sequence[Sequence[float]]) -> float:
    expected = (
        (right[0][0], -right[1][1], right[0][2]),
        (right[1][0], -right[0][1], right[1][2]),
    )
    return max(
        abs(float(left[bound][axis]) - expected[bound][axis])
        for bound in range(2)
        for axis in range(3)
    )


def main() -> int:
    args = _parse_args()
    recipe_path = args.recipe.resolve()
    recipe = _load_recipe(recipe_path)
    toolchain = _mapping(recipe.get("toolchain"), field="toolchain")
    openscad = _openscad_command(args.openscad)
    openscad_version = _tool_version(openscad)
    expected_version = _string(
        toolchain.get("openscad_version"), field="toolchain.openscad_version"
    )
    if openscad_version != expected_version:
        raise RuntimeError(
            f"OpenSCAD version mismatch: expected {expected_version}, got {openscad_version}"
        )

    source = _mapping(recipe.get("source"), field="source")
    scad_path = ROOT / _string(source.get("mount_scad"), field="source.mount_scad")
    source_lock = SourceLock.load(ConfigRepository(ROOT))
    d405_source = source_lock.record(
        _string(source.get("d405_lock_id"), field="source.d405_lock_id")
    )
    expected_commit = _string(source.get("d405_commit"), field="source.d405_commit")
    if dict(d405_source.revision).get("commit") != expected_commit:
        raise RuntimeError("D405 recipe and source-lock commits differ")
    d405_relative = _string(source.get("d405_mesh"), field="source.d405_mesh")
    d405_path = ROOT / d405_source.local_runtime_path / d405_relative
    expected_d405_hash = _string(
        source.get("d405_mesh_sha256"), field="source.d405_mesh_sha256"
    )
    if sha256_file(d405_path) != expected_d405_hash:
        raise RuntimeError("official D405 source mesh hash mismatch")

    output = _mapping(recipe.get("output"), field="output")
    output_root = ROOT / _string(output.get("root"), field="output.root")
    output_names = {
        key: _string(output.get(key), field=f"output.{key}")
        for key in (
            "mount_visual_right",
            "mount_visual_left",
            "d405_visual_right",
            "d405_visual_left",
            "mount_collision_right",
            "mount_collision_left",
            "d405_collision_right",
            "d405_collision_left",
            "report",
        )
    }
    destinations = {key: output_root / value for key, value in output_names.items()}
    existing = sorted(str(path) for path in destinations.values() if path.exists())
    if existing and not args.overwrite:
        raise FileExistsError(f"refusing to replace generated assets: {existing}")

    geometry = _mapping(recipe.get("geometry"), field="geometry")
    weld_tolerance = _number(
        geometry.get("weld_tolerance_mm"), field="geometry.weld_tolerance_mm"
    )
    coverage_gate = _number(
        geometry.get("minimum_proxy_visual_vertex_coverage"),
        field="geometry.minimum_proxy_visual_vertex_coverage",
    )
    mount = _mapping(recipe.get("mount"), field="mount")
    placement = _mapping(
        mount.get("camera_interface_in_right_hand"),
        field="mount.camera_interface_in_right_hand",
    )
    body_translation = _vector3(placement.get("translation_mm"), field="body translation")
    body_rotation = rotation_matrix_z_y(
        _number(placement.get("azimuth_deg"), field="body azimuth"),
        _number(placement.get("tilt_deg"), field="body tilt"),
    )
    d405 = _mapping(recipe.get("d405"), field="d405")
    optical = optical_frame_contract(
        body_rotation=body_rotation,
        body_translation_mm=body_translation,
        optical_origin_from_rear_mm=_vector3(
            d405.get("optical_origin_from_rear_right_mm"),
            field="D405 optical origin",
        ),
    )

    with tempfile.TemporaryDirectory(prefix="wujihand-d405-assets-") as temporary:
        staging = Path(temporary)
        mount_right_path = staging / output_names["mount_visual_right"]
        mount_left_path = staging / output_names["mount_visual_left"]
        mount_right = _export_mount(
            openscad, scad_path=scad_path, side="right", destination=mount_right_path
        )
        mount_left = _export_mount(
            openscad, scad_path=scad_path, side="left", destination=mount_left_path
        )
        raw_d405 = load_stl_triangles(d405_path)
        d405_right = _aligned_d405(raw_d405, side="right")
        d405_left = _aligned_d405(raw_d405, side="left")
        d405_right_path = staging / output_names["d405_visual_right"]
        d405_left_path = staging / output_names["d405_visual_left"]
        write_binary_stl(
            d405_right_path,
            d405_right,
            header="Official realsense-ros D405 aligned right; millimetres",
        )
        write_binary_stl(
            d405_left_path,
            d405_left,
            header="Official realsense-ros D405 aligned left; millimetres",
        )
        d405_right = load_stl_triangles(d405_right_path)
        d405_left = load_stl_triangles(d405_left_path)

        mesh_triangles = {
            "mount_visual_right": mount_right,
            "mount_visual_left": mount_left,
            "d405_visual_right": d405_right,
            "d405_visual_left": d405_left,
        }
        mesh_audits = {
            name: audit_mesh(triangles, weld_tolerance_mm=weld_tolerance).to_mapping()
            for name, triangles in mesh_triangles.items()
        }
        for name, audit in mesh_audits.items():
            _assert_mesh_gate(name, audit)

        mount_right_proxy = _build_right_mount_proxy(recipe)
        mount_left_proxy = _mirror_proxy(mount_right_proxy, side="left")
        d405_right_proxy = _d405_proxy(side="right", optical=optical)
        d405_left_proxy = _d405_proxy(side="left", optical=optical)
        proxies = {
            "mount_collision_right": mount_right_proxy,
            "mount_collision_left": mount_left_proxy,
            "d405_collision_right": d405_right_proxy,
            "d405_collision_left": d405_left_proxy,
        }
        for name, proxy in proxies.items():
            _dump_yaml(staging / output_names[name], proxy)
        proxy_audits = {
            "mount_collision_right": audit_collision_proxy(
                mount_right, mount_right_proxy
            ).to_mapping(),
            "mount_collision_left": audit_collision_proxy(
                mount_left, mount_left_proxy
            ).to_mapping(),
            "d405_collision_right": audit_collision_proxy(
                d405_right, d405_right_proxy
            ).to_mapping(),
            "d405_collision_left": audit_collision_proxy(
                d405_left, d405_left_proxy
            ).to_mapping(),
        }
        for name, audit in proxy_audits.items():
            if (
                float(cast(float, audit["covered_vertex_fraction"])) < coverage_gate
                or audit["clear_points_preserved"] is not True
            ):
                raise RuntimeError(f"{name} failed collision proxy coverage: {audit}")

        mount_bounds_error = _mirror_bounds_error(
            cast(Sequence[Sequence[float]], mesh_audits["mount_visual_right"]["bounds_mm"]),
            cast(Sequence[Sequence[float]], mesh_audits["mount_visual_left"]["bounds_mm"]),
        )
        d405_bounds_error = _mirror_bounds_error(
            cast(Sequence[Sequence[float]], mesh_audits["d405_visual_right"]["bounds_mm"]),
            cast(Sequence[Sequence[float]], mesh_audits["d405_visual_left"]["bounds_mm"]),
        )
        determinant_error = max(
            abs(
                float(
                    cast(Mapping[str, float],
                         _mapping(_mapping(optical[side], field=side)["determinants"], field="determinants"))[kind]
                )
                - 1.0
            )
            for side in ("right", "left")
            for kind in ("body", "optical")
        )
        if mount_bounds_error > 1e-4 or d405_bounds_error > 1e-4 or determinant_error > 1e-12:
            raise RuntimeError("left/right mirror or proper-rotation gate failed")

        generated_paths = {
            name: staging / relative
            for name, relative in output_names.items()
            if name != "report"
        }
        report: dict[str, object] = {
            "schema": "wujihand.d405_wrist_rig_asset_generation_result.v1",
            "status": "qualified",
            "scope": "simulation_only passive visual and collision assets",
            "camera_warning": recipe["camera_warning"],
            "recipe": {
                "path": recipe_path.relative_to(ROOT).as_posix(),
                "sha256": sha256_file(recipe_path),
            },
            "generator": {
                "path": GENERATOR_PATH.relative_to(ROOT).as_posix(),
                "sha256": sha256_file(GENERATOR_PATH),
                "openscad_version": openscad_version,
                "openscad_command": list(openscad),
            },
            "inputs": {
                "mount_scad": {
                    "path": scad_path.relative_to(ROOT).as_posix(),
                    "sha256": sha256_file(scad_path),
                },
                "d405_mesh": {
                    "source": d405_source.name,
                    "commit": expected_commit,
                    "path": d405_relative,
                    "sha256": expected_d405_hash,
                },
            },
            "coordinate_contract": geometry,
            "optical_frames": optical,
            "mesh_audits": mesh_audits,
            "collision_proxy_audits": proxy_audits,
            "mirror_gate": {
                "mount_bounds_max_error_mm": mount_bounds_error,
                "d405_bounds_max_error_mm": d405_bounds_error,
                "proper_rotation_determinant_max_error": determinant_error,
                "usd_negative_scale_required": False,
            },
            "feature_contract": {
                "circular_flange_holes": "robot_screw_holes",
                "capsule_keys": 2,
                "routed_strut_segments": 8,
                "camera_plate": "34x30x3.2_mm_with_two_round_m3_clearance_holes",
            },
            "outputs": {
                name: {
                    "path": output_names[name],
                    "sha256": sha256_file(path),
                }
                for name, path in generated_paths.items()
            },
        }
        report_path = staging / output_names["report"]
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        for name, destination in destinations.items():
            source_path = staging / output_names[name]
            destination.parent.mkdir(parents=True, exist_ok=True)
            source_path.replace(destination)

    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
