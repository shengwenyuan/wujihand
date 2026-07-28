#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Import the one pinned NV-2 NERO URDF recipe into an Isaac 6 asset package."""

from __future__ import annotations

from importlib.metadata import version
import json
from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation.nero_urdf_import import (
    NERO_ASSET_TRANSFORMER_EXTENSION_VERSION,
    NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION,
    load_nero_urdf_facts,
    load_nero_urdf_import_recipe,
    recipe_fingerprint,
)
from wujihand.adapters.simulation.nero_model import load_nero_model_profile
from wujihand.integrity import sha256_file, sha256_tree
from wujihand.runtime.config_repository import ConfigRepository
from wujihand.runtime.source_lock import SourceLock


RECIPE_PATH = ROOT / "configs/profiles/agilex_nero_isaac_6_0_1_import_v1.yaml"
MODEL_PROFILE_PATH = ROOT / "configs/profiles/agilex_nero_q7_provisional_v1.yaml"
GENERATOR_PATH = ROOT / "tools/import_nero_urdf_to_usd.py"
IMPORT_ADAPTER_PATH = ROOT / "src/wujihand/adapters/simulation/nero_urdf_import.py"


def _normalized_isaac_version(distribution_version: str) -> str:
    components = distribution_version.split(".")
    if len(components) < 3:
        raise RuntimeError(f"unexpected isaacsim distribution version: {distribution_version!r}")
    return ".".join(components[:3])


def _enabled_extension_version(extension_manager: object, name: str) -> str:
    manager = extension_manager
    extension_id = manager.get_enabled_extension_id(name)  # type: ignore[attr-defined]
    if not extension_id:
        raise RuntimeError(f"required Isaac extension is not enabled: {name}")
    data = manager.get_extension_dict(extension_id)  # type: ignore[attr-defined]
    return str(data["package"]["version"])


RECIPE = load_nero_urdf_import_recipe(RECIPE_PATH)
MODEL_PROFILE = load_nero_model_profile(MODEL_PROFILE_PATH)
INSTALLED_ISAAC_DISTRIBUTION_VERSION = version("isaacsim")
INSTALLED_ISAAC_VERSION = _normalized_isaac_version(INSTALLED_ISAAC_DISTRIBUTION_VERSION)
if INSTALLED_ISAAC_VERSION != RECIPE.isaac_version:
    raise SystemExit(
        f"Isaac version mismatch: recipe={RECIPE.isaac_version}, "
        f"installed={INSTALLED_ISAAC_VERSION}"
    )

REPOSITORY = ConfigRepository(ROOT)
SOURCE_LOCK = SourceLock.load(REPOSITORY)
SOURCE_RECORD = SOURCE_LOCK.record(RECIPE.source_lock_id)
if dict(SOURCE_RECORD.revision).get("commit") != RECIPE.source_commit:
    raise SystemExit("source lock and NERO import recipe commit differ")
if SOURCE_RECORD.expected_artifact_hash(RECIPE.urdf_path) != RECIPE.urdf_sha256:
    raise SystemExit("source lock and NERO import recipe URDF hash differ")
if SOURCE_RECORD.expected_tree_hash(RECIPE.mesh_tree_path) != RECIPE.mesh_tree_sha256:
    raise SystemExit("source lock and NERO import recipe mesh tree hash differ")
SOURCE_ROOT = REPOSITORY.resolve_project_path(
    SOURCE_RECORD.local_runtime_path,
    field="NERO source root",
    expect_directory=True,
)
SOURCE_URDF = RECIPE.verify_source(SOURCE_ROOT)
SOURCE_FACTS = load_nero_urdf_facts(SOURCE_URDF)
SOURCE_MESH_TREE_SHA256 = sha256_tree(SOURCE_ROOT / RECIPE.mesh_tree_path)
OUTPUT_ROOT = REPOSITORY.resolve_project_path(
    RECIPE.output_root,
    field="NERO derived output root",
    must_exist=False,
)

from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp({"headless": True})

try:
    import omni.kit.app  # type: ignore[import-not-found]

    from wujihand.adapters.simulation.nero_urdf_import import (
        import_nero_urdf,
        inspect_imported_nero_usd,
        normalize_imported_nero_package,
    )

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    actual_extension_version = _enabled_extension_version(
        extension_manager,
        "isaacsim.asset.importer.urdf",
    )
    if actual_extension_version != RECIPE.importer_extension_version:
        raise RuntimeError(
            "URDF importer extension version mismatch: "
            f"recipe={RECIPE.importer_extension_version}, "
            f"installed={actual_extension_version}"
        )
    actual_transformer_version = _enabled_extension_version(
        extension_manager,
        "isaacsim.asset.transformer",
    )
    if actual_transformer_version != NERO_ASSET_TRANSFORMER_EXTENSION_VERSION:
        raise RuntimeError(
            "asset transformer extension version mismatch: "
            f"recipe={NERO_ASSET_TRANSFORMER_EXTENSION_VERSION}, "
            f"installed={actual_transformer_version}"
        )
    actual_transformer_rules_version = _enabled_extension_version(
        extension_manager,
        "isaacsim.asset.transformer.rules",
    )
    if actual_transformer_rules_version != NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION:
        raise RuntimeError(
            "asset transformer rules extension version mismatch: "
            f"recipe={NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION}, "
            f"installed={actual_transformer_rules_version}"
        )

    final_package = OUTPUT_ROOT / RECIPE.robot_name
    report_path = OUTPUT_ROOT / f"{RECIPE.robot_name}.import.json"
    if final_package.exists() or report_path.exists():
        raise FileExistsError(
            "refusing to replace an existing NERO package or import report; "
            "archive both before rerunning"
        )
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    staging_root = Path(tempfile.mkdtemp(prefix=".nero-import-", dir=OUTPUT_ROOT)).resolve()
    staged = import_nero_urdf(
        RECIPE,
        source_root=SOURCE_ROOT,
        output_root=staging_root,
    )
    normalized_layers = normalize_imported_nero_package(staged.parent)
    inspection = inspect_imported_nero_usd(
        staged,
        recipe=RECIPE,
        model_profile=MODEL_PROFILE,
        source_facts=SOURCE_FACTS,
    )
    package_tree_sha256 = sha256_tree(staged.parent)
    staged.parent.replace(final_package)
    staging_root.rmdir()
    imported = final_package / staged.name
    report = {
        "schema": "wujihand.nero_urdf_import_result.v1",
        "status": "generated_unqualified",
        "recipe_path": RECIPE_PATH.relative_to(ROOT).as_posix(),
        "recipe_sha256": sha256_file(RECIPE_PATH),
        "recipe_fingerprint": recipe_fingerprint(RECIPE),
        "model_profile_path": MODEL_PROFILE_PATH.relative_to(ROOT).as_posix(),
        "model_profile_sha256": sha256_file(MODEL_PROFILE_PATH),
        "isaac_version": INSTALLED_ISAAC_VERSION,
        "isaac_distribution_version": INSTALLED_ISAAC_DISTRIBUTION_VERSION,
        "urdf_importer_extension_version": actual_extension_version,
        "asset_transformer_extension_version": actual_transformer_version,
        "asset_transformer_rules_extension_version": (actual_transformer_rules_version),
        "derivation": {
            "generator_path": GENERATOR_PATH.relative_to(ROOT).as_posix(),
            "generator_sha256": sha256_file(GENERATOR_PATH),
            "import_adapter_path": (IMPORT_ADAPTER_PATH.relative_to(ROOT).as_posix()),
            "import_adapter_sha256": sha256_file(IMPORT_ADAPTER_PATH),
        },
        "source": {
            "lock_id": RECIPE.source_lock_id,
            "commit": RECIPE.source_commit,
            "urdf_path": RECIPE.urdf_path,
            "urdf_sha256": RECIPE.urdf_sha256,
            "mesh_tree_path": RECIPE.mesh_tree_path,
            "mesh_tree_sha256": SOURCE_MESH_TREE_SHA256,
            "ros_package_name": RECIPE.ros_package_name,
        },
        "output": {
            "usd_path": imported.relative_to(ROOT).as_posix(),
            "usd_sha256": sha256_file(imported),
            "package_tree_path": imported.parent.relative_to(ROOT).as_posix(),
            "package_tree_sha256": package_tree_sha256,
        },
        "normalization": {
            "policy": (
                "geometry_usdc_to_usda_remove_volatile_docs_and_author_"
                "physx_self_collision_false_v2"
            ),
            "layers": list(normalized_layers),
        },
        "inspection": inspection,
        "assumptions": list(RECIPE.assumptions),
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
finally:
    simulation_app.close()
