#!/usr/bin/env python3
"""Build an unqualified static T-frame USD package from the restricted STEP.

The input STEP stays read-only.  The tool resolves its AP203/XCAF occurrence
hierarchy, applies a reviewed occurrence selection, emits a flattened check
STEP, and builds separate visual and static collision USD layers.  NERO
occurrences are report-only mount witnesses and are never emitted as geometry.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import platform
import re
import resource
import struct
import sys
import tempfile
import time
from types import SimpleNamespace
from typing import cast

import yaml


ROOT = Path(__file__).resolve().parents[1]
SELECTION_PATH = ROOT / "configs/profiles/dual_nero_tframe_step_selection_v1.yaml"
RECIPE_PATH = ROOT / "configs/profiles/dual_nero_tframe_isaac_6_0_1_import_v1.yaml"
SELECTION_SCHEMA = "wujihand.step_occurrence_selection.v1"
RECIPE_SCHEMA = "wujihand.dual_nero_tframe_step_import_recipe.v1"
INVENTORY_SCHEMA = "wujihand.step_assembly_inventory.v1"
SEMANTIC_SCHEMA = "wujihand.static_workcell_semantic_manifest.v1"
REPORT_SCHEMA = "wujihand.dual_nero_tframe_generation_report.v1"
_OCCURRENCE_RE = re.compile(r"^NAUO([1-9][0-9]*)$")
_ENTITY_RE = re.compile(r"^#([0-9]+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*;\s*$")
_REFERENCE_RE = re.compile(r"#([0-9]+)")
_STEP_STRING_RE = re.compile(r"'((?:''|[^'])*)'")


@dataclass(frozen=True, slots=True)
class SelectionRecord:
    occurrence_id: str
    product_name: str
    role: str
    rationale: str
    category: str


@dataclass(frozen=True, slots=True)
class SelectionManifest:
    selection_id: str
    status: str
    source_sha256: str
    source_size_bytes: int
    records: tuple[SelectionRecord, ...]
    assumptions: tuple[str, ...]

    @property
    def by_id(self) -> dict[str, SelectionRecord]:
        return {record.occurrence_id: record for record in self.records}

    def ids(self, category: str) -> tuple[str, ...]:
        return tuple(record.occurrence_id for record in self.records if record.category == category)


@dataclass(frozen=True, slots=True)
class ImportRecipe:
    recipe_id: str
    status: str
    isaac_version: str
    openusd_version: str
    source_sha256: str
    source_size_bytes: int
    source_encoding: str
    selection_path: str
    selection_id: str
    output_root: str
    root_prim: str
    visual_root_prim: str
    collision_root_prim: str
    coordinate_frame_id: str
    coordinate_frame_status: str
    witness_ids: tuple[str, ...]
    assembly_to_asset_axis_rows: tuple[tuple[float, float, float], ...]
    visual: Mapping[str, object]
    collision: Mapping[str, object]
    brep_acceptance: Mapping[str, object]
    canonicalization: Mapping[str, object]
    toolchain: Mapping[str, object]
    assumptions: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class RawOccurrence:
    occurrence_id: str
    entity_id: int
    parent_product_definition_id: int
    child_product_definition_id: int
    parent_occurrence_id: str | None
    product_name: str


@dataclass(slots=True)
class OccurrenceGeometry:
    raw: RawOccurrence
    category: str
    role: str
    parent_occurrence_id: str | None
    is_assembly: bool
    local_matrix_mm: list[list[float]]
    assembly_matrix_mm: list[list[float]]
    rotation_determinant: float
    shape: object
    shape_valid: bool
    brep_quality_status: str
    brep_check: Mapping[str, object] | None
    solid_count: int
    shell_count: int
    closed_shell_count: int
    face_count: int
    assembled_aabb_mm: list[list[float]]
    assembled_aabb_method: str
    source_color_rgb: list[float] | None


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must be a sequence")
    return value


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _integer(value: object, *, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    return value


def _float(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _boolean(value: object, *, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value


def _project_path(value: object, *, field: str) -> str:
    text = _string(value, field=field)
    path = Path(text)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != text:
        raise ValueError(f"{field} must be a normalized project-relative path")
    return text


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _natural_occurrence_key(value: str) -> int:
    match = _OCCURRENCE_RE.fullmatch(value)
    if match is None:
        raise ValueError(f"invalid occurrence ID: {value!r}")
    return int(match.group(1))


def _canonicalize(value: object, *, decimals: int) -> object:
    if isinstance(value, Mapping):
        return {
            str(key): _canonicalize(item, decimals=decimals)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (list, tuple)):
        return [_canonicalize(item, decimals=decimals) for item in value]
    if isinstance(value, float):
        rounded = round(value, decimals)
        return 0.0 if rounded == 0.0 else rounded
    return value


def _canonical_json_bytes(value: object, *, decimals: int) -> bytes:
    normalized = _canonicalize(value, decimals=decimals)
    return json.dumps(
        normalized,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _identity_payload(value: Mapping[str, object], *, decimals: int) -> dict[str, object]:
    payload = dict(value)
    payload["identity_sha256"] = hashlib.sha256(
        _canonical_json_bytes(payload, decimals=decimals)
    ).hexdigest()
    return payload


def _write_json(path: Path, value: object, *, decimals: int) -> None:
    normalized = _canonicalize(value, decimals=decimals)
    path.write_text(
        json.dumps(normalized, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _load_yaml(path: Path) -> Mapping[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    return _mapping(value, field=str(path))


def load_selection_manifest(path: str | Path = SELECTION_PATH) -> SelectionManifest:
    document = _load_yaml(Path(path))
    if document.get("schema") != SELECTION_SCHEMA:
        raise ValueError("unsupported T-frame selection schema")
    source = _mapping(document.get("source"), field="selection.source")
    records: list[SelectionRecord] = []
    for category in ("include", "exclude", "mount_witness"):
        for index, item in enumerate(
            _sequence(document.get(category), field=f"selection.{category}")
        ):
            record = _mapping(item, field=f"selection.{category}[{index}]")
            if frozenset(record) != {
                "occurrence_id",
                "product_name",
                "role",
                "rationale",
            }:
                raise ValueError(f"selection.{category}[{index}] keys differ")
            records.append(
                SelectionRecord(
                    occurrence_id=_string(
                        record.get("occurrence_id"),
                        field=f"selection.{category}[{index}].occurrence_id",
                    ),
                    product_name=_string(
                        record.get("product_name"),
                        field=f"selection.{category}[{index}].product_name",
                    ),
                    role=_string(record.get("role"), field="selection role"),
                    rationale=_string(record.get("rationale"), field="selection rationale"),
                    category=category,
                )
            )
    ids = [record.occurrence_id for record in records]
    for occurrence_id in ids:
        _natural_occurrence_key(occurrence_id)
    if len(ids) != len(set(ids)):
        raise ValueError("selection occurrence IDs must be unique")
    expected = {f"NAUO{index}" for index in range(1, 40)}
    if set(ids) != expected:
        raise ValueError(
            "selection must classify NAUO1..NAUO39 exactly: "
            f"missing={sorted(expected - set(ids))}, unexpected={sorted(set(ids) - expected)}"
        )
    records.sort(key=lambda item: _natural_occurrence_key(item.occurrence_id))
    assumptions = tuple(
        _string(value, field="selection.assumptions[]")
        for value in _sequence(document.get("assumptions"), field="selection.assumptions")
    )
    return SelectionManifest(
        selection_id=_string(document.get("selection_id"), field="selection.selection_id"),
        status=_string(document.get("status"), field="selection.status"),
        source_sha256=_string(source.get("sha256"), field="selection.source.sha256"),
        source_size_bytes=_integer(source.get("size_bytes"), field="selection.source.size_bytes"),
        records=tuple(records),
        assumptions=assumptions,
    )


def load_import_recipe(path: str | Path = RECIPE_PATH) -> ImportRecipe:
    document = _load_yaml(Path(path))
    if document.get("schema") != RECIPE_SCHEMA:
        raise ValueError("unsupported T-frame import recipe schema")
    isaac = _mapping(document.get("isaac"), field="recipe.isaac")
    source = _mapping(document.get("source"), field="recipe.source")
    selection = _mapping(document.get("selection"), field="recipe.selection")
    output = _mapping(document.get("output"), field="recipe.output")
    if output.get("up_axis") != "Z":
        raise ValueError("T-frame output must be Z-up")
    if not math.isclose(
        _float(output.get("meters_per_unit"), field="recipe.output.meters_per_unit"),
        1.0,
    ):
        raise ValueError("T-frame output meters_per_unit must be 1.0")
    frame = _mapping(document.get("coordinate_frame"), field="recipe.coordinate_frame")
    rows = tuple(
        tuple(
            _float(component, field="recipe.coordinate_frame.assembly_to_asset_axis_rows")
            for component in _sequence(row, field="recipe coordinate row")
        )
        for row in _sequence(
            frame.get("assembly_to_asset_axis_rows"), field="recipe coordinate rows"
        )
    )
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("assembly_to_asset_axis_rows must be 3x3")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if not math.isclose(determinant, 1.0, abs_tol=1e-12):
        raise ValueError("assembly_to_asset axes must be right-handed")
    witness_ids = tuple(
        _string(value, field="recipe coordinate witness")
        for value in _sequence(
            frame.get("mount_witness_occurrence_ids"), field="recipe coordinate witnesses"
        )
    )
    assumptions = tuple(
        _string(value, field="recipe.assumptions[]")
        for value in _sequence(document.get("assumptions"), field="recipe.assumptions")
    )
    return ImportRecipe(
        recipe_id=_string(document.get("recipe_id"), field="recipe.recipe_id"),
        status=_string(document.get("status"), field="recipe.status"),
        isaac_version=_string(isaac.get("version"), field="recipe.isaac.version"),
        openusd_version=_string(isaac.get("openusd_version"), field="recipe.isaac.openusd_version"),
        source_sha256=_string(source.get("sha256"), field="recipe.source.sha256"),
        source_size_bytes=_integer(source.get("size_bytes"), field="recipe.source.size_bytes"),
        source_encoding=_string(source.get("text_encoding"), field="recipe.source.text_encoding"),
        selection_path=_project_path(selection.get("path"), field="recipe.selection.path"),
        selection_id=_string(selection.get("selection_id"), field="recipe.selection.selection_id"),
        output_root=_project_path(output.get("root"), field="recipe.output.root"),
        root_prim=_string(output.get("root_prim"), field="recipe.output.root_prim"),
        visual_root_prim=_string(
            output.get("visual_root_prim"), field="recipe.output.visual_root_prim"
        ),
        collision_root_prim=_string(
            output.get("collision_root_prim"), field="recipe.output.collision_root_prim"
        ),
        coordinate_frame_id=_string(
            frame.get("frame_id"), field="recipe.coordinate_frame.frame_id"
        ),
        coordinate_frame_status=_string(
            frame.get("status"), field="recipe.coordinate_frame.status"
        ),
        witness_ids=witness_ids,
        assembly_to_asset_axis_rows=rows,
        visual=_mapping(document.get("visual"), field="recipe.visual"),
        collision=_mapping(document.get("collision"), field="recipe.collision"),
        brep_acceptance=_mapping(document.get("brep_acceptance"), field="recipe.brep_acceptance"),
        canonicalization=_mapping(
            document.get("canonicalization"), field="recipe.canonicalization"
        ),
        toolchain=_mapping(document.get("toolchain"), field="recipe.toolchain"),
        assumptions=assumptions,
    )


def _runtime_modules() -> SimpleNamespace:
    try:
        import numpy as np
        import OCP
        from OCP.Bnd import Bnd_Box
        from OCP.BRep import BRep_Builder, BRep_Tool
        from OCP.BRepBndLib import BRepBndLib
        from OCP.BRepCheck import BRepCheck_Analyzer, BRepCheck_NoError
        from OCP.BRepGProp import BRepGProp
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.GProp import GProp_GProps
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.Interface import Interface_Static
        from OCP.Quantity import Quantity_Color
        from OCP.STEPCAFControl import STEPCAFControl_Reader
        from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
        from OCP.TCollection import TCollection_ExtendedString
        from OCP.TDataStd import TDataStd_Name
        from OCP.TDF import TDF_Label, TDF_LabelSequence
        from OCP.TDocStd import TDocStd_Document
        from OCP.TopAbs import (
            TopAbs_FACE,
            TopAbs_COMPOUND,
            TopAbs_EDGE,
            TopAbs_REVERSED,
            TopAbs_SHELL,
            TopAbs_SOLID,
            TopAbs_VERTEX,
            TopAbs_WIRE,
        )
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS, TopoDS_Compound
        from OCP.XCAFApp import XCAFApp_Application
        from OCP.XCAFDoc import (
            XCAFDoc_ColorGen,
            XCAFDoc_ColorSurf,
            XCAFDoc_ColorTool,
            XCAFDoc_DocumentTool,
            XCAFDoc_ShapeTool,
        )
        from pxr import Kind, Sdf, Usd, UsdGeom, UsdPhysics
        from scipy.spatial import ConvexHull
        import trimesh
    except ImportError as error:
        raise RuntimeError(
            "CAD runtime is incomplete; use the pinned isolated OCP/USD toolchain"
        ) from error
    return SimpleNamespace(
        np=np,
        OCP=OCP,
        Bnd_Box=Bnd_Box,
        BRep_Builder=BRep_Builder,
        BRep_Tool=BRep_Tool,
        BRepBndLib=BRepBndLib,
        BRepCheck_Analyzer=BRepCheck_Analyzer,
        BRepCheck_NoError=BRepCheck_NoError,
        BRepGProp=BRepGProp,
        BRepMesh_IncrementalMesh=BRepMesh_IncrementalMesh,
        GProp_GProps=GProp_GProps,
        IFSelect_RetDone=IFSelect_RetDone,
        Interface_Static=Interface_Static,
        Quantity_Color=Quantity_Color,
        STEPCAFControl_Reader=STEPCAFControl_Reader,
        STEPControl_AsIs=STEPControl_AsIs,
        STEPControl_Writer=STEPControl_Writer,
        TCollection_ExtendedString=TCollection_ExtendedString,
        TDataStd_Name=TDataStd_Name,
        TDF_Label=TDF_Label,
        TDF_LabelSequence=TDF_LabelSequence,
        TDocStd_Document=TDocStd_Document,
        TopAbs_FACE=TopAbs_FACE,
        TopAbs_COMPOUND=TopAbs_COMPOUND,
        TopAbs_EDGE=TopAbs_EDGE,
        TopAbs_REVERSED=TopAbs_REVERSED,
        TopAbs_SHELL=TopAbs_SHELL,
        TopAbs_SOLID=TopAbs_SOLID,
        TopAbs_VERTEX=TopAbs_VERTEX,
        TopAbs_WIRE=TopAbs_WIRE,
        TopExp_Explorer=TopExp_Explorer,
        TopLoc_Location=TopLoc_Location,
        TopoDS=TopoDS,
        TopoDS_Compound=TopoDS_Compound,
        XCAFApp_Application=XCAFApp_Application,
        XCAFDoc_ColorGen=XCAFDoc_ColorGen,
        XCAFDoc_ColorSurf=XCAFDoc_ColorSurf,
        XCAFDoc_ColorTool=XCAFDoc_ColorTool,
        XCAFDoc_DocumentTool=XCAFDoc_DocumentTool,
        XCAFDoc_ShapeTool=XCAFDoc_ShapeTool,
        Kind=Kind,
        Sdf=Sdf,
        Usd=Usd,
        UsdGeom=UsdGeom,
        UsdPhysics=UsdPhysics,
        ConvexHull=ConvexHull,
        trimesh=trimesh,
    )


def _verify_toolchain(recipe: ImportRecipe, runtime: SimpleNamespace) -> dict[str, str]:
    actual = {
        "python": f"{sys.version_info.major}.{sys.version_info.minor}",
        "cadquery_ocp": version("cadquery-ocp"),
        "opencascade": str(runtime.OCP.__version__),
        "numpy": version("numpy"),
        "scipy": version("scipy"),
        "trimesh": version("trimesh"),
        "pyyaml": version("PyYAML"),
        "usd_core": version("usd-core"),
    }
    expected = {key: str(value) for key, value in recipe.toolchain.items()}
    if actual != expected:
        raise RuntimeError(f"CAD toolchain differs: expected={expected}, actual={actual}")
    usd_version = ".".join(str(item) for item in runtime.Usd.GetVersion())
    if usd_version != recipe.openusd_version:
        raise RuntimeError(
            f"OpenUSD library differs: recipe={recipe.openusd_version}, actual={usd_version}"
        )
    return actual


def _first_step_string(arguments: str) -> str:
    match = _STEP_STRING_RE.search(arguments)
    if match is None:
        raise ValueError("STEP entity has no string argument")
    return match.group(1).replace("''", "'")


def _scan_raw_step(step_path: Path, *, encoding: str) -> tuple[str, list[RawOccurrence]]:
    products: dict[int, str] = {}
    formations: dict[int, int] = {}
    product_definitions: dict[int, int] = {}
    nauo_entities: list[tuple[int, str, int, int]] = []
    text = step_path.read_bytes().decode(encoding)
    for raw_line in text.splitlines():
        match = _ENTITY_RE.match(raw_line.strip())
        if match is None:
            continue
        entity_id = int(match.group(1))
        entity_type = match.group(2)
        arguments = match.group(3)
        references = [int(value) for value in _REFERENCE_RE.findall(arguments)]
        if entity_type == "PRODUCT":
            products[entity_id] = _first_step_string(arguments)
        elif entity_type.startswith("PRODUCT_DEFINITION_FORMATION"):
            if not references:
                raise ValueError(f"formation #{entity_id} has no PRODUCT reference")
            formations[entity_id] = references[0]
        elif entity_type == "PRODUCT_DEFINITION":
            if not references:
                raise ValueError(f"product definition #{entity_id} has no formation")
            product_definitions[entity_id] = references[0]
        elif entity_type == "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
            if len(references) < 2:
                raise ValueError(f"NAUO entity #{entity_id} has too few references")
            nauo_entities.append(
                (entity_id, _first_step_string(arguments), references[-2], references[-1])
            )
    if len(products) != 17 or len(nauo_entities) != 39:
        raise RuntimeError(
            f"raw STEP inventory differs: products={len(products)}, occurrences={len(nauo_entities)}"
        )

    def product_name(product_definition_id: int) -> str:
        formation_id = product_definitions[product_definition_id]
        return products[formations[formation_id]]

    child_to_occurrence = {child: occurrence for _, occurrence, _, child in nauo_entities}
    occurrences = [
        RawOccurrence(
            occurrence_id=occurrence_id,
            entity_id=entity_id,
            parent_product_definition_id=parent_id,
            child_product_definition_id=child_id,
            parent_occurrence_id=child_to_occurrence.get(parent_id),
            product_name=product_name(child_id),
        )
        for entity_id, occurrence_id, parent_id, child_id in nauo_entities
    ]
    occurrences.sort(key=lambda item: _natural_occurrence_key(item.occurrence_id))
    root_product_definition_ids = {parent for _, _, parent, _ in nauo_entities} - {
        child for _, _, _, child in nauo_entities
    }
    if len(root_product_definition_ids) != 1:
        raise RuntimeError("STEP must have exactly one root product definition")
    root_product_name = product_name(next(iter(root_product_definition_ids)))
    return root_product_name, occurrences


def _label_name(label: object, runtime: SimpleNamespace) -> str | None:
    attribute = runtime.TDataStd_Name()
    if not label.FindAttribute(runtime.TDataStd_Name.GetID_s(), attribute):
        return None
    return attribute.Get().ToExtString()


def _location_matrix(location: object) -> list[list[float]]:
    transform = location.Transformation()
    return [
        [float(transform.Value(row, column)) for column in range(1, 5)] for row in range(1, 4)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def _shape_counts(shape: object, runtime: SimpleNamespace) -> tuple[int, int, int, int]:
    counts: dict[object, int] = {}
    for shape_type in (runtime.TopAbs_SOLID, runtime.TopAbs_SHELL, runtime.TopAbs_FACE):
        explorer = runtime.TopExp_Explorer(shape, shape_type)
        count = 0
        while explorer.More():
            count += 1
            explorer.Next()
        counts[shape_type] = count
    closed_shells = 0
    explorer = runtime.TopExp_Explorer(shape, runtime.TopAbs_SHELL)
    while explorer.More():
        if runtime.TopoDS.Shell_s(explorer.Current()).Closed():
            closed_shells += 1
        explorer.Next()
    return (
        counts[runtime.TopAbs_SOLID],
        counts[runtime.TopAbs_SHELL],
        closed_shells,
        counts[runtime.TopAbs_FACE],
    )


def _shape_aabb(shape: object, runtime: SimpleNamespace) -> list[list[float]]:
    box = runtime.Bnd_Box()
    runtime.BRepBndLib.AddOptimal_s(shape, box, False, False)
    minimum_x, minimum_y, minimum_z, maximum_x, maximum_y, maximum_z = box.Get()
    return [
        [float(minimum_x), float(minimum_y), float(minimum_z)],
        [float(maximum_x), float(maximum_y), float(maximum_z)],
    ]


def _source_color(
    component: object,
    referred: object,
    runtime: SimpleNamespace,
) -> list[float] | None:
    for label in (component, referred):
        for color_type in (runtime.XCAFDoc_ColorSurf, runtime.XCAFDoc_ColorGen):
            color = runtime.Quantity_Color()
            if runtime.XCAFDoc_ColorTool.GetColor_s(label, color_type, color):
                return [float(color.Red()), float(color.Green()), float(color.Blue())]
    return None


def _brep_check_summary(
    shape: object,
    analyzer: object,
    runtime: SimpleNamespace,
) -> dict[str, object]:
    shape_types = (
        ("compound", runtime.TopAbs_COMPOUND),
        ("solid", runtime.TopAbs_SOLID),
        ("shell", runtime.TopAbs_SHELL),
        ("face", runtime.TopAbs_FACE),
        ("wire", runtime.TopAbs_WIRE),
        ("edge", runtime.TopAbs_EDGE),
        ("vertex", runtime.TopAbs_VERTEX),
    )
    by_shape_type: dict[str, object] = {}
    for name, shape_type in shape_types:
        checked = 0
        invalid = 0
        status_counts: dict[str, int] = {}
        explorer = runtime.TopExp_Explorer(shape, shape_type)
        while explorer.More():
            checked += 1
            result = analyzer.Result(explorer.Current())
            statuses = list(result.Status()) if result is not None else []
            non_error = [status for status in statuses if status != runtime.BRepCheck_NoError]
            if non_error:
                invalid += 1
                for status in non_error:
                    status_counts[status.name] = status_counts.get(status.name, 0) + 1
            explorer.Next()
        by_shape_type[name] = {
            "checked": checked,
            "invalid": invalid,
            "statuses": dict(sorted(status_counts.items())),
        }
    return {"valid": bool(analyzer.IsValid()), "by_shape_type": by_shape_type}


def _brep_quality_status(
    *,
    category: str,
    valid: bool,
    summary: Mapping[str, object] | None,
    solid_count: int,
    recipe: ImportRecipe,
) -> str:
    if category == "exclude":
        return "excluded_not_qualified"
    if category == "mount_witness":
        policy = recipe.brep_acceptance.get("mount_witness_policy")
        if policy != "transform_only_allow_invalid_geometry_never_emit_witness_mesh":
            raise RuntimeError("unsupported mount-witness BRep policy")
        return (
            "transform_only_valid_geometry_not_emitted"
            if valid
            else "transform_only_invalid_geometry_not_emitted"
        )
    if valid:
        return "valid"
    if summary is None or solid_count <= 0:
        raise RuntimeError("included invalid BRep has no valid-solid evidence")
    policy = recipe.brep_acceptance.get("included_shape_policy")
    if policy != "allow_invalid_aggregate_only_when_solids_and_shells_are_valid":
        raise RuntimeError("unsupported included-shape BRep policy")
    allowed = {
        _string(value, field="brep_acceptance.allowed_non_solid_statuses[]")
        for value in _sequence(
            recipe.brep_acceptance.get("allowed_non_solid_statuses"),
            field="brep_acceptance.allowed_non_solid_statuses",
        )
    }
    by_shape_type = _mapping(summary.get("by_shape_type"), field="BRep summary")
    for shape_type in ("compound", "solid", "shell", "edge", "vertex"):
        record = _mapping(by_shape_type.get(shape_type), field=f"BRep summary {shape_type}")
        if _integer(record.get("invalid"), field=f"BRep summary {shape_type}.invalid"):
            raise RuntimeError(f"included invalid BRep has invalid {shape_type} topology")
    observed: set[str] = set()
    for shape_type in ("face", "wire"):
        record = _mapping(by_shape_type.get(shape_type), field=f"BRep summary {shape_type}")
        statuses = _mapping(record.get("statuses"), field=f"BRep summary {shape_type}.statuses")
        observed.update(statuses)
    if not observed or not observed.issubset(allowed):
        raise RuntimeError(
            f"included invalid BRep statuses are unsupported: observed={sorted(observed)}, "
            f"allowed={sorted(allowed)}"
        )
    return "invalid_aggregate_accepted_for_unqualified_mesh_with_valid_solids"


def _load_xcaf_occurrences(
    step_path: Path,
    raw_occurrences: Sequence[RawOccurrence],
    selection: SelectionManifest,
    recipe: ImportRecipe,
    runtime: SimpleNamespace,
) -> tuple[object, dict[str, OccurrenceGeometry]]:
    app = runtime.XCAFApp_Application.GetApplication_s()
    document = runtime.TDocStd_Document(runtime.TCollection_ExtendedString("MDTV-XCAF"))
    app.NewDocument(runtime.TCollection_ExtendedString("MDTV-XCAF"), document)
    reader = runtime.STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    reader.SetLayerMode(True)
    if reader.ReadFile(str(step_path)) != runtime.IFSelect_RetDone:
        raise RuntimeError("OCCT failed to read STEP")
    if not reader.Transfer(document):
        raise RuntimeError("OCCT failed to transfer STEP into XCAF")
    shape_tool = runtime.XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = runtime.TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise RuntimeError(f"expected one XCAF root, got {roots.Length()}")
    raw_by_id = {item.occurrence_id: item for item in raw_occurrences}
    selection_by_id = selection.by_id
    occurrences: dict[str, OccurrenceGeometry] = {}
    validity_cache: dict[int, bool] = {}
    detailed_check_cache: dict[int, Mapping[str, object]] = {}

    def walk(definition: object, parent_location: object, parent_id: str | None) -> None:
        components = runtime.TDF_LabelSequence()
        if not runtime.XCAFDoc_ShapeTool.GetComponents_s(definition, components, False):
            return
        for index in range(1, components.Length() + 1):
            component = components.Value(index)
            occurrence_id = _label_name(component, runtime)
            if occurrence_id is None or occurrence_id not in raw_by_id:
                raise RuntimeError(f"XCAF component has unknown occurrence name {occurrence_id!r}")
            referred = runtime.TDF_Label()
            if not runtime.XCAFDoc_ShapeTool.GetReferredShape_s(component, referred):
                raise RuntimeError(f"{occurrence_id} has no referred XCAF shape")
            local_location = runtime.XCAFDoc_ShapeTool.GetLocation_s(component)
            assembly_location = parent_location.Multiplied(local_location)
            definition_shape = runtime.XCAFDoc_ShapeTool.GetShape_s(referred)
            shape = definition_shape.Located(assembly_location)
            local_matrix = _location_matrix(local_location)
            assembly_matrix = _location_matrix(assembly_location)
            determinant = float(
                runtime.np.linalg.det(runtime.np.asarray(assembly_matrix, dtype=float)[:3, :3])
            )
            solid_count, shell_count, closed_shell_count, face_count = _shape_counts(shape, runtime)
            selected = selection_by_id[occurrence_id]
            product_key = raw_by_id[occurrence_id].child_product_definition_id
            analyzer = None
            if product_key not in validity_cache:
                analyzer = runtime.BRepCheck_Analyzer(shape, True)
                validity_cache[product_key] = bool(analyzer.IsValid())
            shape_valid = validity_cache[product_key]
            detailed_check = None
            if selected.category in {"include", "mount_witness"}:
                if product_key not in detailed_check_cache:
                    if analyzer is None:
                        analyzer = runtime.BRepCheck_Analyzer(shape, True)
                    detailed_check_cache[product_key] = _brep_check_summary(
                        shape, analyzer, runtime
                    )
                detailed_check = detailed_check_cache[product_key]
            quality_status = _brep_quality_status(
                category=selected.category,
                valid=shape_valid,
                summary=detailed_check,
                solid_count=solid_count,
                recipe=recipe,
            )
            occurrences[occurrence_id] = OccurrenceGeometry(
                raw=raw_by_id[occurrence_id],
                category=selected.category,
                role=selected.role,
                parent_occurrence_id=parent_id,
                is_assembly=bool(runtime.XCAFDoc_ShapeTool.IsAssembly_s(referred)),
                local_matrix_mm=local_matrix,
                assembly_matrix_mm=assembly_matrix,
                rotation_determinant=determinant,
                shape=shape,
                shape_valid=shape_valid,
                brep_quality_status=quality_status,
                brep_check=detailed_check,
                solid_count=solid_count,
                shell_count=shell_count,
                closed_shell_count=closed_shell_count,
                face_count=face_count,
                assembled_aabb_mm=_shape_aabb(shape, runtime),
                assembled_aabb_method="brep_add_optimal",
                source_color_rgb=_source_color(component, referred, runtime),
            )
            if runtime.XCAFDoc_ShapeTool.IsAssembly_s(referred):
                walk(referred, assembly_location, occurrence_id)

    walk(roots.Value(1), runtime.TopLoc_Location(), None)
    if set(occurrences) != set(raw_by_id):
        raise RuntimeError("XCAF and raw STEP occurrence sets differ")
    for occurrence_id, geometry in occurrences.items():
        raw = geometry.raw
        selected = selection_by_id[occurrence_id]
        if raw.parent_occurrence_id != geometry.parent_occurrence_id:
            raise RuntimeError(f"{occurrence_id} parent differs between AP203 and XCAF")
        if raw.product_name != selected.product_name:
            raise RuntimeError(
                f"{occurrence_id} product differs: STEP={raw.product_name!r}, "
                f"selection={selected.product_name!r}"
            )
    return document, occurrences


def _frame_transform(
    recipe: ImportRecipe,
    occurrences: Mapping[str, OccurrenceGeometry],
    runtime: SimpleNamespace,
) -> tuple[object, object, list[float]]:
    witness_origins = [
        runtime.np.asarray(occurrences[item].assembly_matrix_mm, dtype=float)[:3, 3]
        for item in recipe.witness_ids
    ]
    center_mm = runtime.np.mean(runtime.np.stack(witness_origins), axis=0)
    rotation = runtime.np.asarray(recipe.assembly_to_asset_axis_rows, dtype=float)
    transform = runtime.np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = -(rotation @ center_mm) * 0.001
    transform[:3, :3] *= 0.001
    orientation_transform = runtime.np.eye(4, dtype=float)
    orientation_transform[:3, :3] = rotation
    orientation_transform[:3, 3] = -(rotation @ center_mm)
    return transform, orientation_transform, center_mm.tolist()


def _transform_points(
    points_mm: object, frame_transform: object, runtime: SimpleNamespace
) -> object:
    points = runtime.np.asarray(points_mm, dtype=float)
    return points @ frame_transform[:3, :3].T + frame_transform[:3, 3]


def _tessellate(
    shape: object,
    *,
    linear_deflection_mm: float,
    angular_deflection_rad: float,
    frame_transform: object,
    runtime: SimpleNamespace,
) -> tuple[object, object]:
    mesher = runtime.BRepMesh_IncrementalMesh(
        shape,
        linear_deflection_mm,
        False,
        angular_deflection_rad,
        False,
    )
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError("OCCT tessellation failed")
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    explorer = runtime.TopExp_Explorer(shape, runtime.TopAbs_FACE)
    while explorer.More():
        face = runtime.TopoDS.Face_s(explorer.Current())
        location = runtime.TopLoc_Location()
        triangulation = runtime.BRep_Tool.Triangulation_s(face, location)
        if triangulation is not None and triangulation.NbTriangles() > 0:
            offset = len(vertices)
            transform = location.Transformation()
            for node_index in range(1, triangulation.NbNodes() + 1):
                point = triangulation.Node(node_index).Transformed(transform)
                vertices.append((point.X(), point.Y(), point.Z()))
            reversed_face = face.Orientation() == runtime.TopAbs_REVERSED
            for triangle_index in range(1, triangulation.NbTriangles() + 1):
                first, second, third = triangulation.Triangle(triangle_index).Get()
                triangle = (
                    offset + first - 1,
                    offset + second - 1,
                    offset + third - 1,
                )
                faces.append((triangle[0], triangle[2], triangle[1]) if reversed_face else triangle)
        explorer.Next()
    if not vertices or not faces:
        raise RuntimeError("shape produced no triangles")
    transformed = _transform_points(vertices, frame_transform, runtime)
    return transformed, runtime.np.asarray(faces, dtype=runtime.np.int64)


def _build_inspection_meshes(
    recipe: ImportRecipe,
    occurrences: Mapping[str, OccurrenceGeometry],
    frame_transform: object,
    runtime: SimpleNamespace,
) -> dict[str, tuple[object, object]]:
    identity_mm = runtime.np.eye(4, dtype=float)
    meshes: dict[str, tuple[object, object]] = {}
    for occurrence_id in sorted(occurrences, key=_natural_occurrence_key):
        geometry = occurrences[occurrence_id]
        if geometry.is_assembly:
            continue
        vertices_mm, faces = _tessellate(
            geometry.shape,
            linear_deflection_mm=_float(
                recipe.collision["linear_deflection_mm"],
                field="collision.linear_deflection_mm",
            ),
            angular_deflection_rad=_float(
                recipe.collision["angular_deflection_rad"],
                field="collision.angular_deflection_rad",
            ),
            frame_transform=identity_mm,
            runtime=runtime,
        )
        geometry.assembled_aabb_mm = _mesh_aabb(vertices_mm)
        geometry.assembled_aabb_method = "fixed_coarse_tessellation_applied_transform"
        meshes[occurrence_id] = (
            _transform_points(vertices_mm, frame_transform, runtime),
            faces,
        )

    children_by_parent: dict[str, list[str]] = {}
    for occurrence_id, geometry in occurrences.items():
        if geometry.parent_occurrence_id is not None:
            children_by_parent.setdefault(geometry.parent_occurrence_id, []).append(occurrence_id)

    def update_assembly_bounds(occurrence_id: str) -> list[list[float]]:
        geometry = occurrences[occurrence_id]
        if not geometry.is_assembly:
            return geometry.assembled_aabb_mm
        child_bounds = [
            update_assembly_bounds(child_id)
            for child_id in children_by_parent.get(occurrence_id, [])
        ]
        if not child_bounds:
            raise RuntimeError(f"assembly occurrence {occurrence_id} has no child geometry")
        geometry.assembled_aabb_mm = _aggregate_bounds(child_bounds)
        geometry.assembled_aabb_method = "aggregate_of_child_occurrence_tessellated_bounds"
        return geometry.assembled_aabb_mm

    for occurrence_id, geometry in occurrences.items():
        if geometry.is_assembly:
            update_assembly_bounds(occurrence_id)
    return meshes


def _mesh_aabb(vertices: object) -> list[list[float]]:
    return [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]


def _triangle_digest(
    vertices: object,
    faces: object,
    *,
    quantization_m: float,
) -> str:
    quantized = (vertices / quantization_m).round().astype("int64")
    triangle_hashes: list[bytes] = []
    for face in faces:
        points = sorted(tuple(int(value) for value in quantized[index]) for index in face)
        triangle_hashes.append(
            hashlib.sha256(
                struct.pack("<9q", *(value for point in points for value in point))
            ).digest()
        )
    digest = hashlib.sha256()
    for item in sorted(triangle_hashes):
        digest.update(item)
    return digest.hexdigest()


def _combined_digest(records: Sequence[Mapping[str, object]]) -> str:
    digest = hashlib.sha256()
    for record in sorted(records, key=lambda item: str(item["prim_path"])):
        digest.update(str(record["prim_path"]).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(record["triangle_digest_sha256"]).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _set_custom_string(prim: object, name: str, value: str, runtime: SimpleNamespace) -> None:
    prim.CreateAttribute(name, runtime.Sdf.ValueTypeNames.String, custom=True).Set(value)


def _author_mesh(
    stage: object,
    prim_path: str,
    vertices: object,
    faces: object,
    *,
    double_sided: bool,
    color: tuple[float, float, float],
    runtime: SimpleNamespace,
) -> object:
    mesh = runtime.UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr(vertices.tolist())
    mesh.CreateFaceVertexCountsAttr([3] * int(len(faces)))
    mesh.CreateFaceVertexIndicesAttr(faces.reshape(-1).tolist())
    mesh.CreateSubdivisionSchemeAttr(runtime.UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(double_sided)
    mesh.CreateDisplayColorAttr([color])
    mesh.CreateExtentAttr(_mesh_aabb(vertices))
    return mesh


def _write_visual_stage(
    path: Path,
    recipe: ImportRecipe,
    meshes: Mapping[str, tuple[object, object]],
    occurrences: Mapping[str, OccurrenceGeometry],
    runtime: SimpleNamespace,
) -> None:
    stage = runtime.Usd.Stage.CreateNew(str(path))
    runtime.UsdGeom.SetStageUpAxis(stage, runtime.UsdGeom.Tokens.z)
    runtime.UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = runtime.UsdGeom.Xform.Define(stage, recipe.visual_root_prim)
    root.GetPrim().SetMetadata("kind", runtime.Kind.Tokens.component)
    stage.SetDefaultPrim(root.GetPrim())
    _set_custom_string(root.GetPrim(), "wujihand:status", recipe.status, runtime)
    geometry_scope = runtime.UsdGeom.Scope.Define(stage, f"{recipe.visual_root_prim}/Geometry")
    geometry_scope.CreatePurposeAttr(runtime.UsdGeom.Tokens.render)
    palette = (
        (0.31, 0.57, 0.82),
        (0.92, 0.59, 0.25),
        (0.35, 0.70, 0.47),
        (0.80, 0.35, 0.42),
        (0.57, 0.43, 0.75),
        (0.80, 0.74, 0.29),
        (0.33, 0.72, 0.74),
    )
    for index, occurrence_id in enumerate(sorted(meshes, key=_natural_occurrence_key)):
        vertices, faces = meshes[occurrence_id]
        mesh = _author_mesh(
            stage,
            f"{recipe.visual_root_prim}/Geometry/{occurrence_id}",
            vertices,
            faces,
            double_sided=bool(recipe.visual["double_sided"]),
            color=palette[index % len(palette)],
            runtime=runtime,
        )
        geometry = occurrences[occurrence_id]
        _set_custom_string(mesh.GetPrim(), "wujihand:occurrenceId", occurrence_id, runtime)
        _set_custom_string(
            mesh.GetPrim(), "wujihand:productName", geometry.raw.product_name, runtime
        )
        _set_custom_string(mesh.GetPrim(), "wujihand:role", geometry.role, runtime)
    stage.GetRootLayer().Save()


def _compact_quantized_mesh(
    vertices: object,
    faces: object,
    *,
    quantization_m: float,
    runtime: SimpleNamespace,
) -> tuple[object, object]:
    quantized = runtime.np.round(vertices / quantization_m).astype(runtime.np.int64)
    unique_quantized, inverse = runtime.np.unique(
        quantized,
        axis=0,
        return_inverse=True,
    )
    compact_faces = inverse[faces]
    keep = runtime.np.asarray(
        [len({int(value) for value in face}) == 3 for face in compact_faces],
        dtype=bool,
    )
    compact_faces = compact_faces[keep].astype(runtime.np.int64)
    compact_vertices = unique_quantized.astype(float) * quantization_m
    if not len(compact_faces):
        raise RuntimeError("quantized collision mesh has no non-degenerate triangles")
    return compact_vertices, compact_faces


def _collision_proxies(
    recipe: ImportRecipe,
    occurrences: Mapping[str, OccurrenceGeometry],
    frame_transform: object,
    runtime: SimpleNamespace,
) -> tuple[list[dict[str, object]], dict[str, tuple[object, object]]]:
    quantization = _float(
        recipe.canonicalization["geometry_quantization_m"],
        field="canonicalization.geometry_quantization_m",
    )
    minimum_volume = _float(
        recipe.collision["minimum_source_solid_volume_mm3"],
        field="collision.minimum_source_solid_volume_mm3",
    )
    maximum_hull_ratio = _float(
        recipe.collision["convex_hull_max_volume_ratio"],
        field="collision.convex_hull_max_volume_ratio",
    )
    if recipe.collision.get("concave_fallback") != "static_triangle_mesh":
        raise RuntimeError("collision concave fallback must remain static_triangle_mesh")
    records: list[dict[str, object]] = []
    meshes: dict[str, tuple[object, object]] = {}
    for occurrence_id in sorted(
        (item for item, value in occurrences.items() if value.category == "include"),
        key=_natural_occurrence_key,
    ):
        geometry = occurrences[occurrence_id]
        explorer = runtime.TopExp_Explorer(geometry.shape, runtime.TopAbs_SOLID)
        solid_index = 0
        emitted_count = 0
        while explorer.More():
            solid_index += 1
            solid = runtime.TopoDS.Solid_s(explorer.Current())
            properties = runtime.GProp_GProps()
            runtime.BRepGProp.VolumeProperties_s(solid, properties, True, False, False)
            source_volume_mm3 = abs(float(properties.Mass()))
            if source_volume_mm3 < minimum_volume:
                explorer.Next()
                continue
            vertices, source_faces = _tessellate(
                solid,
                linear_deflection_mm=_float(
                    recipe.collision["linear_deflection_mm"],
                    field="collision.linear_deflection_mm",
                ),
                angular_deflection_rad=_float(
                    recipe.collision["angular_deflection_rad"],
                    field="collision.angular_deflection_rad",
                ),
                frame_transform=frame_transform,
                runtime=runtime,
            )
            source_vertices, source_faces = _compact_quantized_mesh(
                vertices,
                source_faces,
                quantization_m=quantization,
                runtime=runtime,
            )
            if len(source_vertices) < 4:
                raise RuntimeError(f"{occurrence_id} solid {solid_index} is degenerate")
            hull = runtime.ConvexHull(source_vertices)
            used = runtime.np.unique(hull.simplices.reshape(-1))
            hull_vertices = source_vertices[used]
            remap = {int(old): index for index, old in enumerate(used.tolist())}
            hull_faces = runtime.np.asarray(
                [[remap[int(value)] for value in face] for face in hull.simplices],
                dtype=runtime.np.int64,
            )
            center = hull_vertices.mean(axis=0)
            for face_index, face in enumerate(hull_faces):
                first, second, third = hull_vertices[face]
                normal = runtime.np.cross(second - first, third - first)
                if float(runtime.np.dot(normal, (first + second + third) / 3.0 - center)) < 0:
                    hull_faces[face_index, [1, 2]] = hull_faces[face_index, [2, 1]]
            volume_ratio = float(hull.volume / (source_volume_mm3 * 1e-9))
            if volume_ratio <= maximum_hull_ratio:
                proxy_vertices = hull_vertices
                proxy_faces = hull_faces
                approximation = "convexHull"
                representation = "convex_hull"
            else:
                proxy_vertices = source_vertices
                proxy_faces = source_faces
                approximation = "none"
                representation = "static_triangle_mesh"
            emitted_count += 1
            proxy_id = f"{occurrence_id}_solid_{emitted_count:03d}"
            prim_path = f"{recipe.collision_root_prim}/Geometry/{proxy_id}"
            digest = _triangle_digest(
                proxy_vertices,
                proxy_faces,
                quantization_m=quantization,
            )
            records.append(
                {
                    "prim_path": prim_path,
                    "occurrence_id": occurrence_id,
                    "source_solid_index": solid_index,
                    "source_volume_m3": source_volume_mm3 * 1e-9,
                    "source_triangle_count": int(len(source_faces)),
                    "convex_hull_volume_m3": float(hull.volume),
                    "volume_ratio": volume_ratio,
                    "representation": representation,
                    "vertex_count": int(len(proxy_vertices)),
                    "triangle_count": int(len(proxy_faces)),
                    "aabb_m": _mesh_aabb(proxy_vertices),
                    "triangle_digest_sha256": digest,
                    "approximation": approximation,
                }
            )
            meshes[proxy_id] = (proxy_vertices, proxy_faces)
            explorer.Next()
        if emitted_count == 0:
            raise RuntimeError(f"included occurrence {occurrence_id} has no collision solids")
    return records, meshes


def _write_collision_stage(
    path: Path,
    recipe: ImportRecipe,
    records: Sequence[Mapping[str, object]],
    meshes: Mapping[str, tuple[object, object]],
    runtime: SimpleNamespace,
) -> None:
    stage = runtime.Usd.Stage.CreateNew(str(path))
    runtime.UsdGeom.SetStageUpAxis(stage, runtime.UsdGeom.Tokens.z)
    runtime.UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = runtime.UsdGeom.Xform.Define(stage, recipe.collision_root_prim)
    root.GetPrim().SetMetadata("kind", runtime.Kind.Tokens.component)
    stage.SetDefaultPrim(root.GetPrim())
    _set_custom_string(root.GetPrim(), "wujihand:status", recipe.status, runtime)
    geometry_scope = runtime.UsdGeom.Scope.Define(stage, f"{recipe.collision_root_prim}/Geometry")
    geometry_scope.CreatePurposeAttr(runtime.UsdGeom.Tokens.proxy)
    records_by_name = {str(record["prim_path"]).rsplit("/", 1)[-1]: record for record in records}
    for proxy_id in sorted(meshes):
        vertices, faces = meshes[proxy_id]
        mesh = _author_mesh(
            stage,
            str(records_by_name[proxy_id]["prim_path"]),
            vertices,
            faces,
            double_sided=False,
            color=(0.75, 0.25, 0.25),
            runtime=runtime,
        )
        runtime.UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        runtime.UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(
            str(records_by_name[proxy_id]["approximation"])
        )
        _set_custom_string(
            mesh.GetPrim(),
            "wujihand:occurrenceId",
            str(records_by_name[proxy_id]["occurrence_id"]),
            runtime,
        )
        _set_custom_string(mesh.GetPrim(), "wujihand:role", "static_collision_proxy", runtime)
    stage.GetRootLayer().Save()


def _write_wrapper(path: Path, recipe: ImportRecipe, runtime: SimpleNamespace) -> None:
    stage = runtime.Usd.Stage.CreateNew(str(path))
    runtime.UsdGeom.SetStageUpAxis(stage, runtime.UsdGeom.Tokens.z)
    runtime.UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = runtime.UsdGeom.Xform.Define(stage, recipe.root_prim)
    root.GetPrim().SetMetadata("kind", runtime.Kind.Tokens.component)
    stage.SetDefaultPrim(root.GetPrim())
    _set_custom_string(root.GetPrim(), "wujihand:status", recipe.status, runtime)
    _set_custom_string(
        root.GetPrim(), "wujihand:coordinateFrameStatus", recipe.coordinate_frame_status, runtime
    )
    visual = runtime.UsdGeom.Xform.Define(stage, f"{recipe.root_prim}/Visual")
    visual.GetPrim().GetReferences().AddReference("./tframe_visual.usdc", recipe.visual_root_prim)
    runtime.UsdGeom.Imageable(visual.GetPrim()).CreatePurposeAttr(runtime.UsdGeom.Tokens.render)
    collision = runtime.UsdGeom.Xform.Define(stage, f"{recipe.root_prim}/Collision")
    collision.GetPrim().GetReferences().AddReference(
        "./tframe_collision.usdc", recipe.collision_root_prim
    )
    runtime.UsdGeom.Imageable(collision.GetPrim()).CreatePurposeAttr(runtime.UsdGeom.Tokens.proxy)
    stage.GetRootLayer().Save()


def _write_cleaned_step(
    path: Path,
    occurrences: Mapping[str, OccurrenceGeometry],
    runtime: SimpleNamespace,
) -> None:
    compound = runtime.TopoDS_Compound()
    builder = runtime.BRep_Builder()
    builder.MakeCompound(compound)
    for occurrence_id in sorted(
        (item for item, value in occurrences.items() if value.category == "include"),
        key=_natural_occurrence_key,
    ):
        builder.Add(compound, occurrences[occurrence_id].shape)
    if not runtime.Interface_Static.SetCVal_s("write.step.schema", "AP203"):
        raise RuntimeError("OCCT cannot select AP203 STEP output")
    writer = runtime.STEPControl_Writer()
    if writer.Transfer(compound, runtime.STEPControl_AsIs) != runtime.IFSelect_RetDone:
        raise RuntimeError("failed to transfer selected compound to STEP writer")
    if writer.Write(str(path)) != runtime.IFSelect_RetDone:
        raise RuntimeError("failed to write cleaned STEP")


def _write_preview(
    path: Path,
    meshes: Mapping[str, tuple[object, object]],
    categories: Mapping[str, str],
    runtime: SimpleNamespace,
) -> None:
    scene = runtime.trimesh.Scene(base_frame="provisional_tframe_shoulder_center")
    palette = {
        "include": [80, 150, 210, 255],
        "exclude": [150, 150, 150, 180],
        "mount_witness": [230, 125, 55, 220],
    }
    for name in sorted(meshes, key=_natural_occurrence_key):
        vertices, faces = meshes[name]
        mesh = runtime.trimesh.Trimesh(
            vertices=vertices,
            faces=faces,
            process=False,
            validate=False,
        )
        mesh.visual.face_colors = palette[categories[name]]
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    path.write_bytes(scene.export(file_type="glb"))


def _aggregate_aabb(records: Sequence[Mapping[str, object]]) -> list[list[float]]:
    bounds = [cast(Sequence[Sequence[float]], record["aabb_m"]) for record in records]
    return _aggregate_bounds(bounds)


def _aggregate_bounds(bounds: Sequence[Sequence[Sequence[float]]]) -> list[list[float]]:
    if not bounds:
        raise ValueError("cannot aggregate an empty bounds collection")
    minima = [bound[0] for bound in bounds]
    maxima = [bound[1] for bound in bounds]
    return [
        [min(values) for values in zip(*minima, strict=True)],
        [max(values) for values in zip(*maxima, strict=True)],
    ]


def _validate_usd_package(
    directory: Path,
    recipe: ImportRecipe,
    expected_collision_count: int,
    runtime: SimpleNamespace,
) -> dict[str, object]:
    wrapper_path = directory / "tframe.usda"
    stage = runtime.Usd.Stage.Open(str(wrapper_path))
    if stage is None or stage.GetDefaultPrim().GetPath().pathString != recipe.root_prim:
        raise RuntimeError("wrapper USD default prim differs")
    if runtime.UsdGeom.GetStageUpAxis(stage) != runtime.UsdGeom.Tokens.z:
        raise RuntimeError("wrapper USD is not Z-up")
    if not math.isclose(runtime.UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise RuntimeError("wrapper USD metersPerUnit differs")
    collision_count = 0
    rigid_body_count = 0
    absolute_asset_paths: list[str] = []
    for prim in stage.Traverse():
        if prim.HasAPI(runtime.UsdPhysics.CollisionAPI):
            collision_count += 1
        if prim.HasAPI(runtime.UsdPhysics.RigidBodyAPI):
            rigid_body_count += 1
        for attribute in prim.GetAttributes():
            value = attribute.Get()
            if isinstance(value, runtime.Sdf.AssetPath) and Path(value.path).is_absolute():
                absolute_asset_paths.append(value.path)
    if collision_count != expected_collision_count:
        raise RuntimeError(
            f"collision prim count differs: expected={expected_collision_count}, actual={collision_count}"
        )
    if rigid_body_count:
        raise RuntimeError("T-frame USD must not author rigid bodies")
    if absolute_asset_paths:
        raise RuntimeError(f"USD has absolute asset paths: {absolute_asset_paths}")
    return {
        "default_prim": recipe.root_prim,
        "up_axis": "Z",
        "meters_per_unit": 1.0,
        "collision_prim_count": collision_count,
        "rigid_body_prim_count": rigid_body_count,
        "absolute_asset_paths": absolute_asset_paths,
    }


def _inventory_record(geometry: OccurrenceGeometry) -> dict[str, object]:
    return {
        "occurrence_id": geometry.raw.occurrence_id,
        "step_entity_id": geometry.raw.entity_id,
        "parent_occurrence_id": geometry.parent_occurrence_id,
        "product_name": geometry.raw.product_name,
        "classification": geometry.category,
        "role": geometry.role,
        "is_assembly": geometry.is_assembly,
        "shape_valid": geometry.shape_valid,
        "brep_quality_status": geometry.brep_quality_status,
        "brep_check": geometry.brep_check,
        "local_transform_mm": geometry.local_matrix_mm,
        "assembly_transform_mm": geometry.assembly_matrix_mm,
        "rotation_determinant": geometry.rotation_determinant,
        "is_mirrored": geometry.rotation_determinant < 0.0,
        "solid_count": geometry.solid_count,
        "shell_count": geometry.shell_count,
        "closed_shell_count": geometry.closed_shell_count,
        "face_count": geometry.face_count,
        "assembled_aabb_mm": geometry.assembled_aabb_mm,
        "assembled_aabb_method": geometry.assembled_aabb_method,
        "source_color_rgb": geometry.source_color_rgb,
    }


def build(
    *,
    step_path: Path,
    selection_path: Path,
    recipe_path: Path,
    output_directory: Path,
    full_preview: bool,
) -> dict[str, object]:
    started = time.monotonic()
    selection = load_selection_manifest(selection_path)
    recipe = load_import_recipe(recipe_path)
    if recipe.selection_id != selection.selection_id:
        raise RuntimeError("recipe and selection IDs differ")
    if recipe.source_sha256 != selection.source_sha256:
        raise RuntimeError("recipe and selection source hashes differ")
    if recipe.source_size_bytes != selection.source_size_bytes:
        raise RuntimeError("recipe and selection source sizes differ")
    if recipe.witness_ids != selection.ids("mount_witness"):
        raise RuntimeError("recipe and selection mount witnesses differ")
    for scope_name, scope in (("visual", recipe.visual), ("collision", recipe.collision)):
        if _boolean(scope["relative_deflection"], field=f"{scope_name}.relative_deflection"):
            raise RuntimeError(f"{scope_name} relative tessellation is unsupported")
        if _boolean(scope["parallel"], field=f"{scope_name}.parallel"):
            raise RuntimeError(f"{scope_name} parallel tessellation is unsupported")
    if recipe.collision.get("method") != "per_topods_solid_adaptive_convex_or_static_triangle_mesh":
        raise RuntimeError("collision method must remain adaptive per-solid static geometry")
    if _boolean(recipe.collision.get("qhull_joggle"), field="collision.qhull_joggle"):
        raise RuntimeError("QHull joggle must stay disabled for deterministic generation")
    if output_directory.exists():
        raise FileExistsError(f"refusing to overwrite output: {output_directory}")
    if not step_path.is_file():
        raise FileNotFoundError(step_path)
    if step_path.stat().st_size != recipe.source_size_bytes:
        raise RuntimeError("STEP size differs from pinned recipe")
    source_sha256 = _sha256_file(step_path)
    if source_sha256 != recipe.source_sha256:
        raise RuntimeError("STEP SHA-256 differs from pinned recipe")

    runtime = _runtime_modules()
    toolchain = _verify_toolchain(recipe, runtime)
    root_product_name, raw_occurrences = _scan_raw_step(step_path, encoding=recipe.source_encoding)
    parse_started = time.monotonic()
    _document, occurrences = _load_xcaf_occurrences(
        step_path, raw_occurrences, selection, recipe, runtime
    )
    parse_elapsed = time.monotonic() - parse_started
    frame_transform, orientation_transform, shoulder_center_mm = _frame_transform(
        recipe, occurrences, runtime
    )
    inspection_meshes = _build_inspection_meshes(recipe, occurrences, frame_transform, runtime)
    decimals = _integer(
        recipe.canonicalization["json_float_decimal_places"],
        field="canonicalization.json_float_decimal_places",
    )
    quantization = _float(
        recipe.canonicalization["geometry_quantization_m"],
        field="canonicalization.geometry_quantization_m",
    )

    output_directory.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(
        tempfile.mkdtemp(prefix=f".{output_directory.name}-", dir=output_directory.parent)
    )
    inventory_payload = _identity_payload(
        {
            "schema": INVENTORY_SCHEMA,
            "status": selection.status,
            "source": {
                "sha256": source_sha256,
                "size_bytes": recipe.source_size_bytes,
                "step_schema": "CONFIG_CONTROL_DESIGN",
                "length_unit": "millimetre",
                "root_product_name": root_product_name,
                "product_count": 17,
                "occurrence_count": 39,
            },
            "selection_id": selection.selection_id,
            "selection_sha256": _sha256_file(selection_path),
            "toolchain": toolchain,
            "assembled_aabb_mm": _aggregate_bounds(
                [
                    geometry.assembled_aabb_mm
                    for geometry in occurrences.values()
                    if not geometry.is_assembly
                ]
            ),
            "selected_structure_aabb_mm": _aggregate_bounds(
                [
                    geometry.assembled_aabb_mm
                    for geometry in occurrences.values()
                    if geometry.category == "include"
                ]
            ),
            "occurrences": [
                _inventory_record(occurrences[occurrence_id])
                for occurrence_id in sorted(occurrences, key=_natural_occurrence_key)
            ],
        },
        decimals=decimals,
    )
    _write_json(staging / "assembly_inventory.json", inventory_payload, decimals=decimals)
    parse_metrics = {
        "schema": "wujihand.step_assembly_parse_metrics.v1",
        "source_sha256": source_sha256,
        "inventory_identity_sha256": inventory_payload["identity_sha256"],
        "elapsed_seconds": parse_elapsed,
        "peak_rss_kib": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
        "host": platform.node(),
        "platform": platform.platform(),
    }
    _write_json(staging / "assembly_parse_metrics.json", parse_metrics, decimals=decimals)

    _write_cleaned_step(staging / "tframe_cleaned.step", occurrences, runtime)
    visual_meshes: dict[str, tuple[object, object]] = {}
    visual_records: list[dict[str, object]] = []
    for occurrence_id in selection.ids("include"):
        vertices, faces = _tessellate(
            occurrences[occurrence_id].shape,
            linear_deflection_mm=_float(
                recipe.visual["linear_deflection_mm"], field="visual.linear_deflection_mm"
            ),
            angular_deflection_rad=_float(
                recipe.visual["angular_deflection_rad"],
                field="visual.angular_deflection_rad",
            ),
            frame_transform=frame_transform,
            runtime=runtime,
        )
        visual_meshes[occurrence_id] = (vertices, faces)
        visual_records.append(
            {
                "prim_path": f"{recipe.visual_root_prim}/Geometry/{occurrence_id}",
                "occurrence_id": occurrence_id,
                "product_name": occurrences[occurrence_id].raw.product_name,
                "role": occurrences[occurrence_id].role,
                "vertex_count": int(len(vertices)),
                "triangle_count": int(len(faces)),
                "aabb_m": _mesh_aabb(vertices),
                "triangle_digest_sha256": _triangle_digest(
                    vertices, faces, quantization_m=quantization
                ),
            }
        )
    _write_visual_stage(staging / "tframe_visual.usdc", recipe, visual_meshes, occurrences, runtime)
    _write_preview(
        staging / "tframe_selected_preview.glb",
        visual_meshes,
        {occurrence_id: "include" for occurrence_id in visual_meshes},
        runtime,
    )

    collision_records, collision_meshes = _collision_proxies(
        recipe, occurrences, frame_transform, runtime
    )
    _write_collision_stage(
        staging / "tframe_collision.usdc",
        recipe,
        collision_records,
        collision_meshes,
        runtime,
    )
    _write_wrapper(staging / "tframe.usda", recipe, runtime)

    preview_index: list[dict[str, object]] = []
    if full_preview:
        full_meshes: dict[str, tuple[object, object]] = {}
        full_categories: dict[str, str] = {}
        for occurrence_id in sorted(occurrences, key=_natural_occurrence_key):
            geometry = occurrences[occurrence_id]
            preview_index.append(
                {
                    "node": occurrence_id if not geometry.is_assembly else None,
                    "occurrence_id": occurrence_id,
                    "parent_occurrence_id": geometry.parent_occurrence_id,
                    "product_name": geometry.raw.product_name,
                    "classification": geometry.category,
                    "geometry_emitted": not geometry.is_assembly,
                }
            )
            if geometry.is_assembly:
                continue
            if occurrence_id in visual_meshes:
                mesh = visual_meshes[occurrence_id]
            else:
                mesh = inspection_meshes[occurrence_id]
            full_meshes[occurrence_id] = mesh
            full_categories[occurrence_id] = geometry.category
        _write_preview(
            staging / "assembly_preview.glb",
            full_meshes,
            full_categories,
            runtime,
        )
    else:
        preview_index = [
            {
                "node": None,
                "occurrence_id": occurrence_id,
                "parent_occurrence_id": occurrences[occurrence_id].parent_occurrence_id,
                "product_name": occurrences[occurrence_id].raw.product_name,
                "classification": occurrences[occurrence_id].category,
                "geometry_emitted": False,
            }
            for occurrence_id in sorted(occurrences, key=_natural_occurrence_key)
        ]
    _write_json(
        staging / "assembly_preview_index.json",
        {
            "schema": "wujihand.step_assembly_preview_index.v1",
            "source_sha256": source_sha256,
            "full_preview_written": full_preview,
            "records": preview_index,
        },
        decimals=decimals,
    )

    mount_witnesses = []
    for occurrence_id in recipe.witness_ids:
        assembly_matrix = runtime.np.asarray(
            occurrences[occurrence_id].assembly_matrix_mm, dtype=float
        )
        asset_matrix = orientation_transform @ assembly_matrix
        asset_matrix[:3, 3] *= 0.001
        mount_witnesses.append(
            {
                "occurrence_id": occurrence_id,
                "product_name": occurrences[occurrence_id].raw.product_name,
                "assignment": "unresolved_left_right",
                "status": "provisional_inspection_only",
                "base_link_alignment": "not_verified",
                "assembly_transform_mm": assembly_matrix.tolist(),
                "asset_transform_m": asset_matrix.tolist(),
            }
        )
    semantic_payload = _identity_payload(
        {
            "schema": SEMANTIC_SCHEMA,
            "status": recipe.status,
            "source_sha256": source_sha256,
            "selection_id": selection.selection_id,
            "selection_sha256": _sha256_file(selection_path),
            "inventory_identity_sha256": inventory_payload["identity_sha256"],
            "recipe_id": recipe.recipe_id,
            "recipe_sha256": _sha256_file(recipe_path),
            "stage": {
                "root_prim": recipe.root_prim,
                "up_axis": "Z",
                "meters_per_unit": 1.0,
                "static_no_rigid_body": True,
            },
            "coordinate_frame": {
                "frame_id": recipe.coordinate_frame_id,
                "status": recipe.coordinate_frame_status,
                "shoulder_center_assembly_mm": shoulder_center_mm,
                "assembly_to_asset_axis_rows": [
                    list(row) for row in recipe.assembly_to_asset_axis_rows
                ],
            },
            "visual": {
                "occurrence_ids": list(selection.ids("include")),
                "records": visual_records,
                "aabb_m": _aggregate_aabb(visual_records),
                "combined_triangle_digest_sha256": _combined_digest(visual_records),
            },
            "collision": {
                "method": recipe.collision["method"],
                "records": collision_records,
                "aabb_m": _aggregate_aabb(collision_records),
                "combined_triangle_digest_sha256": _combined_digest(collision_records),
            },
            "mount_witnesses": mount_witnesses,
            "assumptions": list((*selection.assumptions, *recipe.assumptions)),
        },
        decimals=decimals,
    )
    _write_json(
        staging / "tframe_semantic_manifest.json",
        semantic_payload,
        decimals=decimals,
    )
    validation = _validate_usd_package(staging, recipe, len(collision_records), runtime)

    primary_files = sorted(
        path
        for path in staging.iterdir()
        if path.name not in {"generation_report.json", "SHA256SUMS"}
    )
    output_hashes = {path.name: _sha256_file(path) for path in primary_files}
    report = {
        "schema": REPORT_SCHEMA,
        "status": recipe.status,
        "build_id": output_directory.name,
        "source": {
            "sha256": source_sha256,
            "size_bytes": recipe.source_size_bytes,
            "raw_step_redistribution": "prohibited_from_normal_git_by_project_policy",
        },
        "derivation": {
            "generator": "tools/import_dual_nero_tframe_step_to_usd.py",
            "generator_sha256": _sha256_file(Path(__file__)),
            "selection": str(selection_path.relative_to(ROOT)),
            "selection_sha256": _sha256_file(selection_path),
            "recipe": str(recipe_path.relative_to(ROOT)),
            "recipe_sha256": _sha256_file(recipe_path),
            "inventory_identity_sha256": inventory_payload["identity_sha256"],
            "semantic_identity_sha256": semantic_payload["identity_sha256"],
            "toolchain": toolchain,
        },
        "outputs": output_hashes,
        "validation": validation,
        "qualification": {
            "workcell_mount_accepted": False,
            "left_right_assigned": False,
            "pinned_nero_base_link_alignment_verified": False,
            "real_hardware_authority": False,
        },
        "elapsed_seconds": time.monotonic() - started,
        "integrity_closure": "SHA256SUMS includes generation_report.json; it excludes itself",
    }
    _write_json(staging / "generation_report.json", report, decimals=decimals)
    closure_files = sorted(path for path in staging.iterdir() if path.name != "SHA256SUMS")
    (staging / "SHA256SUMS").write_text(
        "".join(f"{_sha256_file(path)}  {path.name}\n" for path in closure_files),
        encoding="utf-8",
    )
    staging.replace(output_directory)
    result = {
        "output_directory": str(output_directory),
        "source_sha256": source_sha256,
        "inventory_identity_sha256": inventory_payload["identity_sha256"],
        "semantic_identity_sha256": semantic_payload["identity_sha256"],
        "visual_occurrence_count": len(visual_records),
        "collision_proxy_count": len(collision_records),
        "full_preview_written": full_preview,
        "status": recipe.status,
    }
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", required=True, type=Path)
    parser.add_argument("--selection", type=Path, default=SELECTION_PATH)
    parser.add_argument("--recipe", type=Path, default=RECIPE_PATH)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--full-preview",
        action="store_true",
        help="also tessellate all leaf occurrences into an inspection-only GLB",
    )
    args = parser.parse_args()
    build(
        step_path=args.step.expanduser().resolve(),
        selection_path=args.selection.resolve(),
        recipe_path=args.recipe.resolve(),
        output_directory=args.output_dir.expanduser().resolve(),
        full_preview=args.full_preview,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
