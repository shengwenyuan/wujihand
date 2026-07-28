from __future__ import annotations

from copy import deepcopy
import hashlib
from pathlib import Path
from typing import Any

import pytest
import yaml

from wujihand.adapters.simulation.nero_urdf_import import (
    NERO_ASSET_TRANSFORMER_EXTENSION_VERSION,
    NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION,
    NERO_IMPORTER_EXTENSION_VERSION,
    NERO_IMPORT_ISAAC_VERSION,
    NERO_NORMALIZED_USDA_PATHS,
    NERO_SOURCE_COMMIT,
    NERO_SOURCE_MESH_TREE_SHA256,
    NERO_SOURCE_URDF_SHA256,
    load_nero_urdf_facts,
    load_nero_urdf_import_recipe,
    normalize_imported_nero_text_layers,
    recipe_fingerprint,
)


ROOT = Path(__file__).parents[2]
RECIPE = ROOT / "configs/profiles/agilex_nero_isaac_6_0_1_import_v1.yaml"


def _mapping() -> dict[str, Any]:
    value = yaml.safe_load(RECIPE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write(tmp_path: Path, value: dict[str, Any]) -> Path:
    path = tmp_path / "recipe.yaml"
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def test_nero_import_recipe_freezes_source_versions_and_topology_options() -> None:
    recipe = load_nero_urdf_import_recipe(RECIPE)

    assert recipe.isaac_version == NERO_IMPORT_ISAAC_VERSION
    assert recipe.importer_extension_version == NERO_IMPORTER_EXTENSION_VERSION
    assert recipe.asset_transformer_extension_version == NERO_ASSET_TRANSFORMER_EXTENSION_VERSION
    assert (
        recipe.asset_transformer_rules_extension_version
        == NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION
    )
    assert recipe.source_commit == NERO_SOURCE_COMMIT
    assert recipe.urdf_sha256 == NERO_SOURCE_URDF_SHA256
    assert recipe.mesh_tree_path == "nero/meshes"
    assert recipe.mesh_tree_sha256 == NERO_SOURCE_MESH_TREE_SHA256
    assert recipe.ros_package_name == "agx_arm_description"
    assert recipe.expected_relative_usd_path == (
        "artifacts/derived/isaac/6.0.1/agilex_nero/nero_description/nero_description.usda"
    )
    assert recipe.options.fix_base
    assert not recipe.options.merge_fixed_joints
    assert not recipe.options.merge_mesh
    assert not recipe.options.collision_from_visuals
    assert not recipe.options.allow_self_collision
    assert recipe.options.joint_drive_type == "force"
    assert recipe.options.joint_target_type == "position"
    assert recipe.options.override_joint_stiffness == 400.0
    assert recipe.options.override_joint_damping == 40.0
    assert len(recipe_fingerprint(recipe)) == 64


def test_nero_import_recipe_verifies_source_bytes(tmp_path: Path) -> None:
    recipe = load_nero_urdf_import_recipe(RECIPE)
    source_root = tmp_path / "agx_arm_urdf"
    urdf = source_root / recipe.urdf_path
    urdf.parent.mkdir(parents=True)
    pinned = ROOT.parent / "does-not-exist-locally"
    assert not pinned.exists()
    urdf.write_text("wrong", encoding="utf-8")

    with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
        recipe.verify_source(source_root)
    assert hashlib.sha256(urdf.read_bytes()).hexdigest() != NERO_SOURCE_URDF_SHA256


def test_nero_import_normalization_removes_only_volatile_documentation(
    tmp_path: Path,
) -> None:
    package = tmp_path / "nero_description"
    for index, relative in enumerate(NERO_NORMALIZED_USDA_PATHS):
        path = package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        reference = (
            "    prepend references = @./geometries.usd@</Geometries/link>\n"
            if relative == "payloads/instances.usda"
            else ""
        )
        path.write_text(
            "#usda 1.0\n"
            "(\n"
            f'    defaultPrim = "nero_{index}"\n'
            '    doc = """Generated from Composed Stage of root layer '
            f"/tmp/random-{index}/nero.usdc\n"
            "\n"
            'Generated from Composed Stage of root layer /host/output/base.usd\n"""\n'
            "    metersPerUnit = 1\n"
            ")\n" + reference,
            encoding="utf-8",
        )

    assert normalize_imported_nero_text_layers(package) == NERO_NORMALIZED_USDA_PATHS
    for index, relative in enumerate(NERO_NORMALIZED_USDA_PATHS):
        normalized = (package / relative).read_text(encoding="utf-8")
        assert "Generated from Composed Stage" not in normalized
        assert f'defaultPrim = "nero_{index}"' in normalized
        assert "metersPerUnit = 1" in normalized
    instances = (package / "payloads/instances.usda").read_text(encoding="utf-8")
    assert "@./geometries.usd@" not in instances
    assert "@./geometries.usda@" in instances


def test_nero_import_normalization_rejects_non_importer_doc(tmp_path: Path) -> None:
    package = tmp_path / "nero_description"
    for relative in NERO_NORMALIZED_USDA_PATHS:
        path = package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        reference = (
            "prepend references = @./geometries.usd@</Geometries/link>\n"
            if relative == "payloads/instances.usda"
            else ""
        )
        path.write_text(
            '#usda 1.0\n(\n    doc = """curated documentation"""\n)\n' + reference,
            encoding="utf-8",
        )

    with pytest.raises(RuntimeError, match="non-importer documentation"):
        normalize_imported_nero_text_layers(package)


def test_nero_urdf_facts_parse_exact_joint_and_inertial_chain() -> None:
    urdf = ROOT / "third_party/src/agx_arm_urdf/nero/urdf/nero_description.urdf"
    if not urdf.is_file():
        pytest.skip("restore the pinned AgileX NERO source")

    facts = load_nero_urdf_facts(urdf)

    assert tuple(inertial.link_name for inertial in facts.inertials) == (
        "base_link",
        "link1",
        "link2",
        "link3",
        "link4",
        "link5",
        "link6",
        "link7",
    )
    assert tuple(joint.name for joint in facts.joints) == (
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    )
    assert facts.inertials[0].mass_kg == pytest.approx(1.06458435)
    assert facts.inertials[0].center_of_mass_xyz_m == pytest.approx(
        (-0.00319465997, -0.00005467608, 0.04321758463)
    )
    assert facts.joints[0].origin_xyz_m == pytest.approx((0.0, 0.0, 0.138))
    assert facts.joints[0].origin_quaternion_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert all(joint.effort == pytest.approx(100.0) for joint in facts.joints)


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda value: value["source"].update({"commit": "0" * 40}),
            "approved pin",
        ),
        (
            lambda value: value["options"].update({"merge_fixed_joints": True}),
            "preserve source links",
        ),
        (
            lambda value: value["options"].update({"collision_from_visuals": True}),
            "preserve authored collision",
        ),
        (
            lambda value: value["options"].update({"allow_self_collision": True}),
            "remains unqualified",
        ),
        (
            lambda value: value["options"].update({"override_joint_stiffness": float("nan")}),
            "finite",
        ),
        (
            lambda value: value["output"].update({"root": "../escape"}),
            "project-relative",
        ),
    ),
)
def test_nero_import_recipe_rejects_drift(
    tmp_path: Path,
    mutation: Any,
    message: str,
) -> None:
    value = deepcopy(_mapping())
    mutation(value)

    with pytest.raises(ValueError, match=message):
        load_nero_urdf_import_recipe(_write(tmp_path, value))
