from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.runtime.config_repository import ConfigRepository
from wujihand.runtime.source_lock import SourceLock, sha256_tree
from wujihand.runtime.yaml_loader import load_yaml_strict
from wujihand.specs import ArtifactSpec, ConfigRef


ROOT = Path(__file__).parents[2]


def test_repository_loads_each_layer_and_enforces_expected_id() -> None:
    repository = ConfigRepository(ROOT)

    asset = repository.load_asset("configs/assets/wuji_hand2_beta1_right_v1.yaml")
    binding = repository.load_binding(
        "configs/bindings/isaac/wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
    )
    assembly = repository.load_assembly(
        "configs/assemblies/hand2_right_fixed_v1.yaml"
    )
    workcell = repository.load_workcell("configs/workcells/isaac_hand2_table_v1.yaml")
    session = repository.load_session(
        ConfigRef(
            path="configs/sessions/isaac_hand2_teleop_v1.yaml",
            expected_id="isaac_hand2_teleop_v1",
        )
    )

    assert asset.asset_id == binding.asset_id == "wuji_hand2_beta1_right"
    assert assembly.assembly_id == "hand2_right_fixed_v1"
    assert workcell.workcell_id == "isaac_hand2_table_v1"
    assert session.assembly.expected_id == assembly.assembly_id

    with pytest.raises(ValueError, match="expected 'wrong_session'"):
        repository.load_session(
            ConfigRef(
                path="configs/sessions/isaac_hand2_teleop_v1.yaml",
                expected_id="wrong_session",
            )
        )


def test_repository_rejects_paths_outside_project(tmp_path: Path) -> None:
    repository = ConfigRepository(ROOT)
    outside = tmp_path / "outside.yaml"
    outside.write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="escapes the project root"):
        repository.load_session(outside)
    with pytest.raises(ValueError, match="escapes the project root"):
        repository.resolve_project_path("../outside.yaml", field="test")


def test_strict_yaml_loader_rejects_duplicate_keys_at_every_depth() -> None:
    with pytest.raises(ValueError, match="duplicate YAML mapping key: 'schema'"):
        load_yaml_strict(
            "schema: wujihand.session.v1\n"
            "schema: wujihand.session.v1\n"
        )
    with pytest.raises(ValueError, match="duplicate YAML mapping key: 'path'"):
        load_yaml_strict(
            "assembly:\n"
            "  path: first.yaml\n"
            "  path: second.yaml\n"
        )


def test_source_lock_resolves_expected_artifact_without_requiring_checkout() -> None:
    repository = ConfigRepository(ROOT)
    source_lock = SourceLock.load(repository)

    artifact = source_lock.resolve(
        ArtifactSpec(
            source="wuji-description",
            source_revision="commit:aee64892ebcf8e3237bedc30231bb09476cbc71d",
            path="hand2_beta/body/usd/right/wujihand.usd",
        )
    )
    tree = source_lock.resolve(
        ArtifactSpec(
            source="wuji-description",
            source_revision="commit:aee64892ebcf8e3237bedc30231bb09476cbc71d",
            path="hand2_beta/body/meshes/right",
        ),
        tree=True,
    )

    assert artifact.expected_sha256 == (
        "3cb3dcb18b07621a52a47a8daa98ab82794e3c77d36275d068b3b5b0516e5f00"
    )
    assert tree.expected_sha256 == (
        "4f1a7e96cafb13403ed82c5ef2f18d52a40afb49776ce56ee8f2224280ffcc13"
    )
    assert artifact.absolute_path == (
        ROOT
        / "third_party/src/wuji-description/hand2_beta/body/usd/right/wujihand.usd"
    )


def test_source_lock_rejects_unlocked_or_escaping_artifact() -> None:
    source_lock = SourceLock.load(ConfigRepository(ROOT))

    with pytest.raises(ValueError, match="does not lock artifact"):
        source_lock.resolve(
            ArtifactSpec(
                source="wuji-description",
                source_revision="commit:aee64892ebcf8e3237bedc30231bb09476cbc71d",
                path="hand2_beta/not_locked.usd",
            )
        )
    with pytest.raises(ValueError, match="unknown source-lock"):
        source_lock.resolve(
            ArtifactSpec(
                source="not-a-source",
                source_revision="deadbeef",
                path="asset.usd",
            )
        )
    with pytest.raises(ValueError, match="does not match pinned revision"):
        source_lock.resolve(
            ArtifactSpec(
                source="wuji-description",
                source_revision="wrong-revision",
                path="hand2_beta/body/usd/right/wujihand.usd",
            )
        )


def test_tree_hash_rejects_symbolic_links(tmp_path: Path) -> None:
    tree = tmp_path / "tree"
    tree.mkdir()
    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"outside")
    (tree / "link.bin").symlink_to(outside)

    with pytest.raises(RuntimeError, match="symbolic links"):
        sha256_tree(tree)
