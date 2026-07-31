from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from wujihand.runtime import modelscope_dataset
from wujihand.runtime.modelscope_dataset import (
    ModelScopeDatasetPin,
    ModelScopeManifest,
    ModelScopeManifestEntry,
    ensure_modelscope_dataset,
    verify_modelscope_snapshot,
)


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _fixture(tmp_path: Path) -> tuple[Path, Path, ModelScopeManifest]:
    project = tmp_path / "project"
    project.mkdir()
    seed = tmp_path / "seed"
    (seed / "assets/scenes").mkdir(parents=True)
    readme = b"pinned dataset\n"
    scene = b"#usda 1.0\n"
    (seed / "README.md").write_bytes(readme)
    (seed / "assets/scenes/demo.usda").write_bytes(scene)
    manifest = ModelScopeManifest.build(
        (
            ModelScopeManifestEntry(
                path="README.md",
                kind="blob",
                size=len(readme),
                sha256=_sha256(readme),
            ),
            ModelScopeManifestEntry(
                path="assets",
                kind="tree",
                size=0,
                sha256=None,
            ),
            ModelScopeManifestEntry(
                path="assets/scenes",
                kind="tree",
                size=0,
                sha256=None,
            ),
            ModelScopeManifestEntry(
                path="assets/scenes/demo.usda",
                kind="blob",
                size=len(scene),
                sha256=_sha256(scene),
            ),
        )
    )
    return project, seed, manifest


def test_snapshot_verifier_rejects_extra_and_corrupt_content(
    tmp_path: Path,
) -> None:
    _, seed, manifest = _fixture(tmp_path)

    verify_modelscope_snapshot(seed, manifest)
    (seed / "README.md").write_text("corrupt\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="size mismatch"):
        verify_modelscope_snapshot(seed, manifest)


def test_ensure_seeds_atomically_and_reuses_receipt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project, seed, manifest = _fixture(tmp_path)
    pin = ModelScopeDatasetPin(
        source_name="fixture",
        dataset_id="owner/dataset",
        revision="a" * 40,
        local_runtime_path=f"third_party/src/dataset/{'a' * 40}",
        manifest_sha256=manifest.sha256,
        expected_blob_count=manifest.blob_count,
        expected_tree_count=manifest.tree_count,
        expected_total_size_bytes=manifest.total_size_bytes,
    )
    monkeypatch.setattr(
        modelscope_dataset,
        "fetch_modelscope_manifest",
        lambda _: manifest,
    )

    seeded = ensure_modelscope_dataset(project, pin, seed_from=seed)
    ready = ensure_modelscope_dataset(project, pin, allow_network=False)

    assert seeded.action == "seeded"
    assert ready.action == "ready"
    assert (seeded.target / "assets/scenes/demo.usda").is_file()
    assert seeded.receipt.is_file()


def test_seed_repairs_an_interrupted_staging_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project, seed, manifest = _fixture(tmp_path)
    revision = "b" * 40
    pin = ModelScopeDatasetPin(
        source_name="fixture",
        dataset_id="owner/dataset",
        revision=revision,
        local_runtime_path=f"third_party/src/dataset/{revision}",
        manifest_sha256=manifest.sha256,
        expected_blob_count=manifest.blob_count,
        expected_tree_count=manifest.tree_count,
        expected_total_size_bytes=manifest.total_size_bytes,
    )
    monkeypatch.setattr(
        modelscope_dataset,
        "fetch_modelscope_manifest",
        lambda _: manifest,
    )
    staging = (
        project / "third_party/src/dataset" / f".{revision}.staging"
    )
    (staging / "assets/scenes").mkdir(parents=True)
    (staging / "assets/scenes/demo.usda").write_bytes(b"partial")

    result = ensure_modelscope_dataset(project, pin, seed_from=seed)

    assert result.action == "seeded"
    assert (result.target / "assets/scenes/demo.usda").read_bytes() == (
        b"#usda 1.0\n"
    )
