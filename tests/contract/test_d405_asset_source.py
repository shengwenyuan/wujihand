from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.integrity import sha256_file
from wujihand.runtime.config_repository import ConfigRepository
from wujihand.runtime.source_lock import SourceLock


ROOT = Path(__file__).parents[2]
SOURCE_NAME = "realsense-ros-d405-description"


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
