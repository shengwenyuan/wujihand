from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from wujihand.integrity import sha256_file, sha256_tree
from wujihand.runtime.config_repository import ConfigRepository
from wujihand.runtime.source_lock import SourceLock


ROOT = Path(__file__).parents[2]
SOURCE_NAME = "realsense-ros-d405-description"
GENERATED_SOURCE_NAME = "isaac-d405-wrist-rig-v2"


def test_d405_source_lock_is_an_exact_official_release_pin() -> None:
    source = SourceLock.load(ConfigRepository(ROOT)).record(SOURCE_NAME)

    assert dict(source.revision) == {
        "commit": "bafc21080c5c8e259dadbb309797949aee0dd950",
        "kind": "git",
        "tag": "4.56.4",
        "url": "https://github.com/realsenseai/realsense-ros.git",
    }
    assert dict(source.artifacts) == {
        "LICENSE": "1c72ac904e86caaa9dbf1740d8d4737264712699e8a7f416cfe15591f21a2cbf",
        "realsense2_description/CMakeLists.txt": (
            "76b86457163c8fab0b9f67dea3a86da373847e65e0a19e162da80605b712ef62"
        ),
        "realsense2_description/meshes/d405.stl": (
            "a248f41149d12b28311829feecbe7a80cf1481fd05e0f5df2c4c7ecd556edd48"
        ),
        "realsense2_description/package.xml": (
            "73c0fe593f574dd3cea0e57378c437c2671d3fdb2db2d32341d599799ee2be59"
        ),
        "realsense2_description/urdf/_d405.urdf.xacro": (
            "5a39829166a7d1a0a90b15afc6a3b074438bb990c9a53d6d2218dbe6848f9616"
        ),
    }


def test_restored_d405_source_files_match_the_lock() -> None:
    repository = ConfigRepository(ROOT)
    source = SourceLock.load(repository).record(SOURCE_NAME)
    source_root = repository.resolve_project_path(
        source.local_runtime_path,
        field="D405 source root",
        must_exist=False,
    )
    if not source_root.is_dir():
        pytest.skip("the pinned sparse D405 source checkout is not restored")

    for relative_path, expected_sha256 in source.artifacts:
        assert sha256_file(source_root / relative_path) == expected_sha256


def test_generated_wrist_rig_assets_and_derivation_match_the_lock() -> None:
    repository = ConfigRepository(ROOT)
    source = SourceLock.load(repository).record(GENERATED_SOURCE_NAME)
    source_root = ROOT / source.local_runtime_path

    assert dict(source.revision) == {
        "kind": "generated",
        "sha256": "044bfb05931e26638336cb3b058ac3652f585d9fc2dbfaaf160aff5e697ae24c",
    }
    assert sha256_tree(source_root) == dict(source.revision)["sha256"]
    for relative_path, expected_sha256 in source.artifacts:
        assert sha256_file(source_root / relative_path) == expected_sha256

    document = yaml.safe_load(
        (ROOT / "third_party/sources.lock.yaml").read_text(encoding="utf-8")
    )
    generated = next(
        item
        for item in document["sources"]
        if item["name"] == GENERATED_SOURCE_NAME
    )
    for path_key, digest_key in (
        ("generated_by", "generator_sha256"),
        ("generation_adapter", "generation_adapter_sha256"),
        ("generator_recipe", "generator_recipe_sha256"),
        ("mount_scad", "mount_scad_sha256"),
    ):
        assert sha256_file(ROOT / generated[path_key]) == generated[digest_key]


def test_generated_wrist_rig_report_records_baked_mirror_and_single_bodies() -> None:
    report = json.loads(
        (
            ROOT
            / "hardware/camera_mounts/nero_hand2_beta1_realsense_d405/generated/"
            "generation_report.json"
        ).read_text(encoding="utf-8")
    )

    assert report["status"] == "qualified"
    assert report["mirror_gate"]["usd_negative_scale_required"] is False
    assert report["camera_warning"].startswith("SIMULATION ONLY: synthetic 140-degree")
    for audit in report["mesh_audits"].values():
        assert audit["body_count"] == 1
        assert audit["shared_edge_component_count"] == 1
        assert audit["watertight"] is True
        assert audit["winding_consistent"] is True
    assert report["collision_proxy_audits"]["mount_collision_right"][
        "covered_vertex_fraction"
    ] > 0.98
    assert report["collision_proxy_audits"]["mount_collision_left"][
        "clear_points_preserved"
    ] is True
