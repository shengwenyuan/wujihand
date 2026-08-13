#!/usr/bin/env python3
"""Build and qualify the NERO—Hand2 Beta1 universal thin metal-core V2."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / (
    "hardware/adapters/nero_hand2_beta1_universal_metal_v2/"
    "nero_hand2_beta1_universal_metal_v2.scad"
)
OUTPUT = ROOT / "hardware/adapters/nero_hand2_beta1_universal_metal_v2/generated"

PARTS = {
    "core_universal": "print/core_universal.stl",
    "hand_plate_reference": "reference/hand_plate_reference.stl",
}

PRE_FIX_CORE_PARAMETERS = {
    "nero_plug_length": 5.80,
    "nero_plug_flat_clocking_deg": -90.0,
    "nero_radial_hole_clocking_deg": 0.0,
    "nero_radial_hole_min_lead_side_material": 0.0,
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


Vector3 = tuple[float, float, float]
Face = tuple[Vector3, Vector3, Vector3]


def _read_binary_stl(path: Path) -> tuple[list[Vector3], list[Face], int]:
    data = path.read_bytes()
    if len(data) < 84:
        raise RuntimeError(f"STL is truncated: {path}")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + triangle_count * 50:
        raise RuntimeError(f"STL is not canonical binary STL: {path}")
    vertices: list[Vector3] = []
    faces: list[Face] = []
    for index in range(triangle_count):
        record = struct.unpack_from("<12fH", data, 84 + index * 50)
        face = (
            (record[3], record[4], record[5]),
            (record[6], record[7], record[8]),
            (record[9], record[10], record[11]),
        )
        faces.append(face)
        vertices.extend(face)
    return vertices, faces, triangle_count


def _topology_gate(faces: list[Face]) -> dict[str, Any]:
    edge_faces: dict[tuple[Vector3, Vector3], list[tuple[int, int]]] = {}
    degenerate_count = 0
    for face_index, face in enumerate(faces):
        if len(set(face)) != 3:
            degenerate_count += 1
        for start, end in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            key = (start, end) if start < end else (end, start)
            direction = 1 if (start, end) == key else -1
            edge_faces.setdefault(key, []).append((face_index, direction))
    nonmanifold_edges = sum(len(uses) != 2 for uses in edge_faces.values())
    inconsistent_edges = sum(
        len(uses) == 2 and uses[0][1] == uses[1][1]
        for uses in edge_faces.values()
    )
    if degenerate_count or nonmanifold_edges or inconsistent_edges:
        raise RuntimeError(
            "mesh topology Gate failed: "
            f"degenerate={degenerate_count}, nonmanifold={nonmanifold_edges}, "
            f"inconsistent={inconsistent_edges}"
        )

    adjacency: list[list[int]] = [[] for _ in faces]
    for uses in edge_faces.values():
        first, second = uses[0][0], uses[1][0]
        adjacency[first].append(second)
        adjacency[second].append(first)
    body_count = 0
    unseen = set(range(len(faces)))
    while unseen:
        body_count += 1
        stack = [unseen.pop()]
        while stack:
            for neighbor in adjacency[stack.pop()]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    stack.append(neighbor)
    if body_count != 1:
        raise RuntimeError(f"mesh must contain one connected body, found {body_count}")
    return {
        "watertight": True,
        "winding_consistent": True,
        "body_count": body_count,
        "degenerate_triangle_count": degenerate_count,
    }


def _mesh_report(path: Path) -> dict[str, Any]:
    vertices, faces, triangle_count = _read_binary_stl(path)
    minimum = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
    if not all(math.isfinite(value) for vertex in vertices for value in vertex):
        raise RuntimeError(f"STL contains a non-finite coordinate: {path}")
    return {
        "sha256": _sha256(path),
        "bytes": path.stat().st_size,
        "triangles": triangle_count,
        "bounds_mm": {"min": minimum, "max": maximum},
        "extents_mm": [maximum[index] - minimum[index] for index in range(3)],
        **_topology_gate(faces),
    }


def _plug_fix_scope_gates(
    v1_path: Path,
    pre_fix_path: Path,
    fixed_path: Path,
) -> dict[str, Any]:
    v1_vertices, _, _ = _read_binary_stl(v1_path)
    pre_fix_vertices, _, _ = _read_binary_stl(pre_fix_path)
    fixed_vertices, _, _ = _read_binary_stl(fixed_path)

    def rounded(
        vertices: list[Vector3],
        *,
        predicate: Any,
        z_shift_mm: float = 0.0,
    ) -> set[Vector3]:
        return {
            (round(x, 4), round(y, 4), round(z + z_shift_mm, 4))
            for x, y, z in vertices
            if predicate(x, y, z)
        }

    def directed_max(source: set[Vector3], target: set[Vector3]) -> float:
        unmatched = source - target
        if not unmatched:
            return 0.0
        return max(
            min(
                math.sqrt(sum((left[axis] - right[axis]) ** 2 for axis in range(3)))
                for right in target
            )
            for left in unmatched
        )

    # The requested edit is confined below the V2 body.  Compare a regenerated
    # pre-fix reference against the corrected mesh: the body, Hand2 keys,
    # accessory frame/holes, cable notch, and their triangulated surfaces must
    # remain identical above the plug/stop-land join.
    pre_fix_frozen = rounded(
        pre_fix_vertices,
        predicate=lambda x, y, z: z >= -3.49,
    )
    fixed_frozen = rounded(
        fixed_vertices,
        predicate=lambda x, y, z: z >= -3.49,
    )
    if not pre_fix_frozen or len(pre_fix_frozen) != len(fixed_frozen):
        raise RuntimeError("non-plug frozen geometry vertex count drifted")
    frozen_hausdorff = max(
        directed_max(pre_fix_frozen, fixed_frozen),
        directed_max(fixed_frozen, pre_fix_frozen),
    )
    if frozen_hausdorff > 0.00011:
        raise RuntimeError(
            "body/Hand2/accessory geometry drifted outside the allowed plug edit: "
            f"Hausdorff={frozen_hausdorff:.7f} mm"
        )

    # At Z>=1.5 only the four capsule lead faces remain, except for four known
    # V1 camera-lug top vertices.  V2 must be an exact subset and may remove
    # only those side-specific lug vertices.
    v1_keys = rounded(v1_vertices, predicate=lambda x, y, z: z >= 1.5)
    v2_keys = rounded(fixed_vertices, predicate=lambda x, y, z: z >= 1.5)
    removed = v1_keys - v2_keys
    added = v2_keys - v1_keys
    expected_removed = {
        (-16.0, -35.0, 3.0),
        (16.0, -35.0, 3.0),
        (-16.0, -28.0, 3.0),
        (16.0, -28.0, 3.0),
    }
    if added or removed != expected_removed:
        raise RuntimeError(
            "Hand2 capsule lead geometry drifted beyond deleting the V1 side lug"
        )

    # Mesh-derived D-flat gate.  At the full-diameter shank start, zero
    # clocking gives a +X chord at X=18 mm while the other extrema remain the
    # Ø39.6 circle.  This also proves that the exported plug is not circular.
    full_shank_start_z = -3.9 - 7.8 + 0.9
    shank_ring = [
        (x, y, z)
        for x, y, z in fixed_vertices
        if abs(z - full_shank_start_z) <= 0.001 and math.hypot(x, y) >= 17.0
    ]
    if not shank_ring:
        raise RuntimeError("could not isolate the corrected NERO D-flat shank ring")
    d_flat_envelope = {
        "min_x": min(item[0] for item in shank_ring),
        "max_x": max(item[0] for item in shank_ring),
        "min_y": min(item[1] for item in shank_ring),
        "max_y": max(item[1] for item in shank_ring),
    }
    expected_d_flat_envelope = {
        "min_x": -19.8,
        "max_x": 18.0,
        "min_y": -19.8,
        "max_y": 19.8,
    }
    d_flat_error = max(
        abs(d_flat_envelope[key] - expected_d_flat_envelope[key])
        for key in expected_d_flat_envelope
    )
    if d_flat_error > 0.001:
        raise RuntimeError(f"zero-clocked NERO D-flat mesh drifted by {d_flat_error:.6f} mm")

    # At the top/bottom tangent of each radial pilot, all outer intersection
    # vertices lie on the four requested 45-degree axes.  Derive the angles
    # from the STL instead of trusting only the SCAD parameter value.
    radial_hole_z = -3.9 - 3.5
    radial_hole_radius = 2.70 / 2
    tangent_vertices = [
        (x, y)
        for x, y, z in fixed_vertices
        if abs(abs(z - radial_hole_z) - radial_hole_radius) <= 0.001
        and math.hypot(x, y) >= 17.0
    ]
    if not tangent_vertices:
        raise RuntimeError("could not isolate NERO radial-pilot tangent vertices")
    tangent_angles = sorted(
        {
            round((math.degrees(math.atan2(y, x)) + 360.0) % 360.0, 3)
            for x, y in tangent_vertices
        }
    )
    expected_angles = [45.0, 135.0, 225.0, 315.0]
    if len(tangent_angles) != 4 or max(
        abs(actual - expected)
        for actual, expected in zip(tangent_angles, expected_angles)
    ) > 0.01:
        raise RuntimeError(
            f"NERO radial-pilot mesh angles are not the requested 45-degree cross: {tangent_angles}"
        )

    return {
        "pre_fix_reference_path": "reference/core_universal_pre_plug_fix_reference.stl",
        "pre_fix_reference_sha256": _sha256(pre_fix_path),
        "edit_scope": "nero_male_plug_only_below_z_minus_3.49_mm",
        "unchanged_body_hand2_accessory_vertex_count": len(fixed_frozen),
        "unchanged_body_hand2_accessory_hausdorff_mm": frozen_hausdorff,
        "nero_plug_length_before_mm": 5.8,
        "nero_plug_length_after_mm": 7.8,
        "nero_radial_hole_lead_side_material_before_mm": 0.95,
        "nero_radial_hole_lead_side_material_after_mm": 2.95,
        "nero_d_flat_envelope_mm": d_flat_envelope,
        "nero_d_flat_envelope_maximum_error_mm": d_flat_error,
        "nero_radial_hole_exit_angles_deg": tangent_angles,
        "v1_core_path": str(v1_path.relative_to(ROOT)),
        "v1_core_sha256": _sha256(v1_path),
        "hand2_capsule_lead_vertices_preserved": len(v2_keys),
        "removed_v1_side_lug_vertices": [list(item) for item in sorted(removed)],
        "added_v2_capsule_lead_vertices": len(added),
    }


def _run(command: list[str]) -> str:
    result = subprocess.run(command, check=True, text=True, capture_output=True)
    return (result.stdout + result.stderr).strip()


def _ascii_stl_to_binary(source: Path, destination: Path) -> None:
    triangles: list[
        tuple[Vector3, Vector3, Vector3, Vector3]
    ] = []
    normal: Vector3 | None = None
    vertices: list[Vector3] = []
    for raw_line in source.read_text(encoding="ascii").splitlines():
        fields = raw_line.strip().split()
        if fields[:2] == ["facet", "normal"] and len(fields) == 5:
            normal = tuple(float(value) for value in fields[2:5])  # type: ignore[assignment]
            vertices = []
        elif fields[:1] == ["vertex"] and len(fields) == 4:
            vertices.append(tuple(float(value) for value in fields[1:4]))  # type: ignore[arg-type]
        elif fields[:1] == ["endfacet"]:
            if normal is None or len(vertices) != 3:
                raise RuntimeError(f"invalid ASCII STL facet in {source}")
            triangles.append((normal, vertices[0], vertices[1], vertices[2]))
            normal = None
            vertices = []
    if not triangles:
        raise RuntimeError(f"ASCII STL has no triangles: {source}")
    triangles = [
        tuple(tuple(0.0 if value == 0.0 else value for value in vector) for vector in triangle)
        for triangle in triangles
    ]  # type: ignore[assignment]
    triangles.sort(key=lambda triangle: (triangle[1], triangle[2], triangle[3], triangle[0]))
    with destination.open("wb") as stream:
        stream.write(b"NERO Hand2 universal metal V2 OpenSCAD export".ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            stream.write(
                struct.pack(
                    "<12fH",
                    *(value for vector in triangle for value in vector),
                    0,
                )
            )


def _export_part(
    openscad: str,
    selector: str,
    destination: Path,
    *,
    parameter_overrides: dict[str, float] | None = None,
) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".ascii.stl")
    command = [
        openscad,
        "--export-format",
        "asciistl",
        "-D",
        f'part="{selector}"',
    ]
    for name, value in (parameter_overrides or {}).items():
        command.extend(["-D", f"{name}={value}"])
    command.extend(["-o", str(temporary), str(SOURCE)])
    _run(command)
    _ascii_stl_to_binary(temporary, destination)
    temporary.unlink()


def _openscad_version(executable: str) -> str:
    return _run([executable, "--version"]).splitlines()[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--openscad", default=shutil.which("openscad") or "openscad")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--verify-repeatability", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    if output.exists() and not args.overwrite:
        raise SystemExit(f"output exists; pass --overwrite: {output}")
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)

    mesh_gate: dict[str, Any] = {}
    repeatability: dict[str, Any] = {}
    for selector, relative in PARTS.items():
        destination = output / relative
        _export_part(args.openscad, selector, destination)
        mesh_gate[relative] = _mesh_report(destination)
        if args.verify_repeatability:
            repeated = destination.with_name(f"{destination.stem}.repeat.stl")
            _export_part(args.openscad, selector, repeated)
            first = _sha256(destination)
            second = _sha256(repeated)
            repeated.unlink()
            if first != second:
                raise RuntimeError(f"repeat OpenSCAD export drifted: {selector}")
            repeatability[relative] = {"identical": True, "sha256": first}

    pre_fix_core = output / "reference/core_universal_pre_plug_fix_reference.stl"
    _export_part(
        args.openscad,
        "core_universal",
        pre_fix_core,
        parameter_overrides=PRE_FIX_CORE_PARAMETERS,
    )
    mesh_gate["reference/core_universal_pre_plug_fix_reference.stl"] = _mesh_report(
        pre_fix_core
    )

    source_core = output / "print/core_universal.stl"
    visual_core = output / "visual/core_universal.stl"
    visual_core.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_core, visual_core)
    mesh_gate["visual/core_universal.stl"] = _mesh_report(visual_core)

    core_audit = mesh_gate["print/core_universal.stl"]
    plug_fix_scope = _plug_fix_scope_gates(
        ROOT / "hardware/adapters/nero_hand2_beta1_v1/generated/visual/core_right.stl",
        pre_fix_core,
        source_core,
    )
    expected_bounds = {
        "min": [-31.0, -31.0, -11.7],
        "max": [31.0, 31.0, 1.8],
    }
    bounds_error = max(
        abs(float(core_audit["bounds_mm"][bound][axis]) - expected_bounds[bound][axis])
        for bound in ("min", "max")
        for axis in range(3)
    )
    if bounds_error > 0.001:
        raise RuntimeError(f"core envelope drifted by {bounds_error:.6f} mm")

    report = {
        "schema": "wujihand.nero_hand2_beta1_universal_metal_v2_generation_result.v2",
        "status": "metal_intent_candidate_print_test_first",
        "source": str(SOURCE.relative_to(ROOT)),
        "source_sha256": _sha256(SOURCE),
        "openscad": _openscad_version(args.openscad),
        "parameters_mm": {
            "core_thickness": 3.5,
            "v1_core_thickness": 10.0,
            "axial_offset_vendor_flange_to_hand2": 15.9,
            "v1_axial_offset_vendor_flange_to_hand2": 22.4,
            "axial_offset_reduction": 6.5,
            "frame_outer_size": 62.0,
            "frame_inner_size": 46.0,
            "frame_rail_width": 8.0,
            "center_hub_d": 42.0,
            "corner_boss_d": 16.0,
            "diagonal_rib_width": 7.0,
            "accessory_interface": "4_sides_x_2_axial_M3_clearance",
            "accessory_hole_centers": {
                "north_south": [[-10.0, -27.0], [10.0, -27.0], [-10.0, 27.0], [10.0, 27.0]],
                "east_west": [[-27.0, -10.0], [-27.0, 10.0], [27.0, -10.0], [27.0, 10.0]],
            },
            "nero_print_test_plug_d": 39.6,
            "nero_plug_length": 7.8,
            "nero_plug_flat_x": 18.0,
            "nero_plug_flat_clocking_deg": 0.0,
            "nero_radial_hole_from_mouth": 3.5,
            "nero_radial_hole_clocking_deg": 45.0,
            "nero_radial_m3_print_pilot_d": 2.7,
            "nero_radial_hole_lead_side_material": 2.95,
            "nero_stop_land_outer_d": 44.5,
            "nero_stop_land_height": 0.4,
            "hand_capsule_center_abs_xy": 20.1525,
            "hand_capsule_key_width": 5.8,
            "hand_capsule_key_center_span": 8.0,
            "hand_capsule_key_depth": 1.8,
            "hand_m3_countersink": "90_deg_d6.4_to_d3.4",
            "hand_m3_countersink_residual_cylindrical_length": 2.0,
        },
        "design_gates": {
            "single_universal_part": True,
            "no_left_right_exports": True,
            "no_d405_bracket_or_side_lug": True,
            "body_and_accessory_rails_same_thickness": True,
            "fourfold_body_and_accessory_pattern": True,
            "nero_d_flat_zero_clocked_to_vendor_cup": True,
            "nero_radial_m3_pattern_clocked_45_deg": True,
            "pre_fix_body_hand2_accessory_geometry_preserved": True,
            "cable_relief_clocking_frozen_from_pre_fix_v2": True,
            "thin_stop_shoulder_land_retained": True,
            "bounds_maximum_error_mm": bounds_error,
        },
        "plug_fix_scope_gate": plug_fix_scope,
        "mesh_gate": mesh_gate,
        "repeatability_gate": {
            "enabled": args.verify_repeatability,
            "artifacts": repeatability,
        },
        "unresolved_physical_gates": [
            "current_machine_cup_inner_diameter_roundness_and_thin_stop_shoulder_metrology",
            "both_hand2_plate_capsules_m3_thread_depth_and_flat_head_fastener_compatibility",
            "metal_material_process_fillet_fea_fatigue_and_fastener_torque",
            "accessory_interface_load_case_and_final_camera_bracket",
            "cable_bend_clearance_on_unpowered_real_hardware",
        ],
        "powered_motion_authorized": False,
    }
    report_path = output / "generation_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
