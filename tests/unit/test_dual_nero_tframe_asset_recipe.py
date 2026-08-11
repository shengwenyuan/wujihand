from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import numpy as np
import pytest


ROOT = Path(__file__).parents[2]
TOOL_PATH = ROOT / "tools/import_dual_nero_tframe_step_to_usd.py"


def _load_tool() -> ModuleType:
    spec = importlib.util.spec_from_file_location("dual_nero_tframe_import_tool", TOOL_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_selection_classifies_every_step_occurrence_once() -> None:
    tool = _load_tool()
    selection = tool.load_selection_manifest()

    assert len(selection.records) == 39
    assert selection.ids("include") == (
        "NAUO1",
        "NAUO5",
        "NAUO6",
        "NAUO7",
        "NAUO8",
        "NAUO9",
        "NAUO10",
    )
    assert len(selection.ids("exclude")) == 30
    assert selection.ids("mount_witness") == ("NAUO37", "NAUO38")
    assert selection.by_id["NAUO2"].product_name == "D型模块-1"
    assert selection.by_id["NAUO37"].category == "mount_witness"


def test_recipe_keeps_mounts_provisional_and_collision_per_solid() -> None:
    tool = _load_tool()
    recipe = tool.load_import_recipe()

    assert recipe.status == "generated_unqualified_simulation_only"
    assert recipe.coordinate_frame_status == "provisional_inspection_only"
    assert recipe.witness_ids == ("NAUO37", "NAUO38")
    assert recipe.collision["method"] == "per_topods_solid_adaptive_convex_or_static_triangle_mesh"
    assert recipe.collision["concave_fallback"] == "static_triangle_mesh"
    assert recipe.collision["convex_hull_max_volume_ratio"] == 2.0
    assert recipe.collision["qhull_joggle"] is False
    axes = np.asarray(recipe.assembly_to_asset_axis_rows)
    np.testing.assert_allclose(axes @ axes.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(axes) == 1.0


def test_triangle_digest_ignores_triangle_winding_and_face_order() -> None:
    tool = _load_tool()
    vertices = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    faces = np.asarray([[0, 1, 2], [0, 3, 1]], dtype=np.int64)
    reordered = np.asarray([[1, 3, 0], [2, 1, 0]], dtype=np.int64)

    assert tool._triangle_digest(  # noqa: SLF001
        vertices, faces, quantization_m=1e-6
    ) == tool._triangle_digest(  # noqa: SLF001
        vertices, reordered, quantization_m=1e-6
    )


def test_brep_exception_accepts_only_named_non_solid_statuses() -> None:
    tool = _load_tool()
    recipe = tool.load_import_recipe()
    by_shape_type = {
        name: {"checked": 1, "invalid": 0, "statuses": {}}
        for name in ("compound", "solid", "shell", "face", "wire", "edge", "vertex")
    }
    by_shape_type["face"] = {
        "checked": 22,
        "invalid": 22,
        "statuses": {"BRepCheck_UnorientableShape": 22},
    }
    by_shape_type["wire"] = {
        "checked": 4,
        "invalid": 4,
        "statuses": {"BRepCheck_NotConnected": 4},
    }
    summary = {"valid": False, "by_shape_type": by_shape_type}

    assert (
        tool._brep_quality_status(  # noqa: SLF001
            category="include",
            valid=False,
            summary=summary,
            solid_count=34,
            recipe=recipe,
        )
        == "invalid_aggregate_accepted_for_unqualified_mesh_with_valid_solids"
    )

    by_shape_type["solid"] = {
        "checked": 34,
        "invalid": 1,
        "statuses": {"BRepCheck_NotClosed": 1},
    }
    with pytest.raises(RuntimeError, match="invalid solid topology"):
        tool._brep_quality_status(  # noqa: SLF001
            category="include",
            valid=False,
            summary=summary,
            solid_count=34,
            recipe=recipe,
        )
