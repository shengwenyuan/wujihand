#!/usr/bin/env python3
# ruff: noqa: E402  # Isaac modules must be imported after SimulationApp starts.
"""Generate the parallel NERO USD that includes the pinned cup-shaped flange."""

from __future__ import annotations

from importlib.metadata import version
import json
from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.adapters.simulation.nero_gripper_flange_import import (
    build_nero_gripper_flange_urdf,
    inspect_nero_gripper_flange_usd,
    load_nero_gripper_flange_import_profile,
)
from wujihand.adapters.simulation.nero_urdf_import import (
    load_nero_urdf_facts,
    load_nero_urdf_import_recipe,
    normalize_imported_nero_package,
)
from wujihand.integrity import sha256_file, sha256_tree
from wujihand.runtime import ConfigRepository, SourceLock


PROFILE_PATH = ROOT / "configs/profiles/agilex_nero_gripper_flange_isaac_6_0_1_import_v1.yaml"
PROFILE = load_nero_gripper_flange_import_profile(PROFILE_PATH)
BASE_RECIPE_PATH = ROOT / PROFILE.base_recipe
BASE_RECIPE = load_nero_urdf_import_recipe(BASE_RECIPE_PATH)
REPOSITORY = ConfigRepository(ROOT)
SOURCE_LOCK = SourceLock.load(REPOSITORY)


def _verify_source_file(source_root: Path, source_name: str, relative: str) -> Path:
    path = source_root / relative
    expected = SOURCE_LOCK.record(source_name).expected_artifact_hash(relative)
    if not path.is_file() or sha256_file(path) != expected:
        raise RuntimeError(f"source-lock mismatch: {source_name}#{relative}")
    return path


BASE_SOURCE = SOURCE_LOCK.record(BASE_RECIPE.source_lock_id)
BASE_ROOT = ROOT / BASE_SOURCE.local_runtime_path
BASE_URDF = BASE_RECIPE.verify_source(BASE_ROOT)
FLANGE_SOURCE = SOURCE_LOCK.record(PROFILE.source_lock_id)
if dict(FLANGE_SOURCE.revision).get("commit") != PROFILE.source_commit:
    raise SystemExit("gripper-flange source commit differs from the import profile")
FLANGE_ROOT = ROOT / FLANGE_SOURCE.local_runtime_path
FLANGE_XACRO = _verify_source_file(FLANGE_ROOT, PROFILE.source_lock_id, PROFILE.xacro_path)


from isaacsim import SimulationApp  # type: ignore[import-not-found]


simulation_app = SimulationApp({"headless": True})

try:
    from isaacsim.asset.importer.urdf import (  # type: ignore[import-not-found]
        URDFImporter,
        URDFImporterConfig,
    )

    if ".".join(version("isaacsim").split(".")[:3]) != BASE_RECIPE.isaac_version:
        raise RuntimeError("Isaac version differs from the pinned NERO recipe")
    output_root = ROOT / PROFILE.output_root
    final_package = output_root / PROFILE.robot_name
    report_path = output_root / f"{PROFILE.robot_name}.import.json"
    if final_package.exists() or report_path.exists():
        raise FileExistsError("refusing to replace the existing NERO flange package")
    output_root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".nero-flange-import-", dir=output_root))
    expanded_urdf = staging / "nero_description.urdf"
    flange_facts = build_nero_gripper_flange_urdf(
        base_urdf=BASE_URDF,
        flange_xacro=FLANGE_XACRO,
        output=expanded_urdf,
        flange_package_name=PROFILE.ros_package_name,
        flange_post_rotation_rpy_rad=PROFILE.flange_post_rotation_rpy_rad,
    )
    for uri in (flange_facts.visual_mesh_uri, flange_facts.collision_mesh_uri):
        relative = uri.removeprefix("package://agx_arm_description/agx_arm_urdf/")
        _verify_source_file(FLANGE_ROOT, PROFILE.source_lock_id, relative)
    imported = Path(
        URDFImporter(
            URDFImporterConfig(
                urdf_path=str(expanded_urdf),
                usd_path=str(staging),
                ros_package_paths=[
                    {
                        "name": BASE_RECIPE.ros_package_name,
                        "path": str(BASE_ROOT.parent),
                    },
                    {
                        "name": PROFILE.ros_package_name,
                        "path": str(FLANGE_ROOT),
                    },
                ],
                **BASE_RECIPE.options.to_mapping(),
            )
        ).import_urdf()
    ).resolve()
    expected = staging / PROFILE.robot_name / f"{PROFILE.robot_name}.usda"
    if imported != expected or not imported.is_file():
        raise RuntimeError(f"Isaac importer returned unexpected path: {imported}")
    expanded_urdf.unlink()
    normalized_layers = normalize_imported_nero_package(imported.parent)
    inspection = inspect_nero_gripper_flange_usd(
        imported,
        base_facts=load_nero_urdf_facts(BASE_URDF),
        flange=flange_facts,
    )
    package_tree_sha256 = sha256_tree(imported.parent)
    imported.parent.replace(final_package)
    staging.rmdir()
    final_usd = final_package / imported.name
    report = {
        "schema": "wujihand.nero_gripper_flange_import_result.v1",
        "status": PROFILE.status,
        "profile_path": PROFILE_PATH.relative_to(ROOT).as_posix(),
        "profile_sha256": sha256_file(PROFILE_PATH),
        "base_recipe_path": BASE_RECIPE_PATH.relative_to(ROOT).as_posix(),
        "base_recipe_sha256": sha256_file(BASE_RECIPE_PATH),
        "isaac_distribution_version": version("isaacsim"),
        "source": {
            "base_lock_id": BASE_SOURCE.name,
            "flange_lock_id": FLANGE_SOURCE.name,
            "commit": PROFILE.source_commit,
            "xacro_path": PROFILE.xacro_path,
            "xacro_sha256": sha256_file(FLANGE_XACRO),
        },
        "gripper_flange_clocking": {
            "post_rotation_rpy_rad": list(PROFILE.flange_post_rotation_rpy_rad),
        },
        "output": {
            "usd_path": final_usd.relative_to(ROOT).as_posix(),
            "usd_sha256": sha256_file(final_usd),
            "package_tree_path": final_package.relative_to(ROOT).as_posix(),
            "package_tree_sha256": package_tree_sha256,
        },
        "normalization": {"layers": list(normalized_layers)},
        "inspection": inspection,
        "assumptions": list(PROFILE.assumptions),
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
finally:
    simulation_app.close()
