from __future__ import annotations

import math
from pathlib import Path
import re
import struct

import pytest

from wujihand.adapters.simulation.nero_hand2_gemini305_mount import (
    MOUNT_V1_CONFIG,
    NeroHand2Gemini305MountConfig,
    color_camera_quat_wxyz,
    load_stl_mesh_mm,
    stl_geometry_sha256,
)


ROOT = Path(__file__).parents[2]
SCAD = ROOT / MOUNT_V1_CONFIG.scad_source


def _scad_number(name: str) -> float:
    match = re.search(
        rf"^{re.escape(name)}\s*=\s*([-+]?[0-9]+(?:\.[0-9]+)?)\s*;",
        SCAD.read_text(encoding="utf-8"),
        flags=re.MULTILINE,
    )
    assert match is not None
    return float(match.group(1))


def test_v1_config_matches_the_checked_in_scad_and_acceptance_camera() -> None:
    config = MOUNT_V1_CONFIG

    assert _scad_number("dorsal_sign") == config.dorsal_sign
    assert _scad_number("camera_tilt_deg") == config.camera_tilt_deg
    assert _scad_number("camera_center_dorsal_offset") == config.camera_center_dorsal_offset_mm
    assert _scad_number("camera_rear_plane_z") == config.camera_rear_plane_z_mm
    assert _scad_number("camera_plate_thickness") == config.camera_plate_thickness_mm
    assert config.camera_translation_m == pytest.approx((-0.070, 0.0, 0.042))
    assert config.camera_rotation_y_deg == pytest.approx(16.0)
    assert config.acceptance_projection_origin_m == pytest.approx((0.0, 0.009, 0.0272))
    assert config.official_color_hfov_deg == pytest.approx(94.0)
    assert config.acceptance_color_hfov_deg == pytest.approx(140.0)
    assert config.acceptance_color_vfov_deg == pytest.approx(119.5710, abs=1e-4)
    assert config.mount_file_sha256.startswith("e6db31e7")
    assert config.mount_geometry_sha256.startswith("60dba54d")
    assert config.aligned_camera_mesh_sha256.startswith("11698236")


def test_v1_flange_uses_external_plate_circular_holes_and_capsule_keys() -> None:
    source = SCAD.read_text(encoding="utf-8")

    assert _scad_number("robot_screw_hole_diameter") == pytest.approx(3.4)
    assert _scad_number("flange_key_width") == pytest.approx(5.8)
    assert _scad_number("flange_key_center_span") == pytest.approx(8.0)
    assert _scad_number("flange_key_depth") == pytest.approx(1.8)
    assert "translate([0, 0, -base_thickness])" in source
    assert "module flange_key(" in source
    assert "module flange_screw_hole(" in source
    assert "module flange_slot(" not in source


def test_config_and_color_camera_orientation_fail_closed() -> None:
    with pytest.raises(ValueError, match="right-hand dorsal"):
        NeroHand2Gemini305MountConfig(dorsal_sign=0)
    with pytest.raises(ValueError, match="finite and positive"):
        NeroHand2Gemini305MountConfig(camera_plate_thickness_mm=0.0)
    with pytest.raises(ValueError, match="below 179"):
        NeroHand2Gemini305MountConfig(acceptance_color_hfov_deg=179.0)
    with pytest.raises(ValueError, match="dorsal_sign"):
        color_camera_quat_wxyz(0)

    quaternion = color_camera_quat_wxyz(-1)
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)


def test_loads_ascii_stl_in_millimetres_as_stage_metres(tmp_path: Path) -> None:
    path = tmp_path / "triangle-ascii.stl"
    path.write_text(
        """solid triangle
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 10 0 0
    vertex 0 20 0
  endloop
endfacet
endsolid triangle
""",
        encoding="utf-8",
    )

    mesh = load_stl_mesh_mm(path)

    assert mesh.encoding == "ascii"
    assert mesh.triangle_count == 1
    assert mesh.points_m[0] == pytest.approx((0.0, 0.0, 0.0))
    assert mesh.points_m[1] == pytest.approx((0.01, 0.0, 0.0))
    assert mesh.points_m[2] == pytest.approx((0.0, 0.02, 0.0))
    assert mesh.bounds_mm == ((0.0, 0.0, 0.0), (10.0, 20.0, 0.0))
    assert mesh.normals[0] == pytest.approx((0.0, 0.0, 1.0))


def test_loads_binary_stl_and_rejects_non_finite_vertices(tmp_path: Path) -> None:
    binary = tmp_path / "triangle-binary.stl"
    binary.write_bytes(
        b"wujihand".ljust(80, b"\0")
        + struct.pack("<I", 1)
        + struct.pack(
            "<12fH",
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0,
        )
    )
    mesh = load_stl_mesh_mm(binary)
    assert mesh.encoding == "binary"
    assert mesh.bounds_mm == ((0.0, 0.0, 0.0), (1.0, 1.0, 0.0))

    invalid = tmp_path / "non-finite.stl"
    invalid.write_text(
        "solid x\nvertex nan 0 0\nvertex 1 0 0\nvertex 0 1 0\nendsolid x\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="non-finite"):
        load_stl_mesh_mm(invalid)


def test_geometry_digest_ignores_facet_and_vertex_order(tmp_path: Path) -> None:
    first = tmp_path / "first.stl"
    second = tmp_path / "second.stl"
    first.write_text(
        "solid x\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
        "vertex 0 0 1\nvertex 1 0 1\nvertex 0 1 1\nendsolid x\n",
        encoding="utf-8",
    )
    second.write_text(
        "solid x\nvertex 0 1 1\nvertex 0 0 1\nvertex 1 0 1\n"
        "vertex 0 1 0\nvertex 1 0 0\nvertex 0 0 0\nendsolid x\n",
        encoding="utf-8",
    )

    assert stl_geometry_sha256(load_stl_mesh_mm(first)) == stl_geometry_sha256(
        load_stl_mesh_mm(second)
    )
