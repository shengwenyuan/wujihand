from __future__ import annotations

from pathlib import Path

import pytest

from teleoperation_quality.artifact import load_run_artifact


def test_complete_checksummed_run_is_accepted(run_root: Path) -> None:
    artifact = load_run_artifact(run_root)

    assert artifact.run_id == "fixture-run"
    assert artifact.expected_topics == ("/fixture",)
    assert artifact.rosbag_metadata["message_count"] == 1
    assert len(artifact.mcap_paths) == 1


def test_tampered_material_is_rejected(run_root: Path) -> None:
    with (run_root / "manifest.json").open("a", encoding="utf-8") as stream:
        stream.write(" ")

    with pytest.raises(ValueError, match="checksum mismatch"):
        load_run_artifact(run_root)


def test_unchecksummed_raw_file_is_rejected(run_root: Path) -> None:
    (run_root / "raw" / "unexpected.bin").write_bytes(b"not in checksum inventory")

    with pytest.raises(ValueError, match="unchecksummed material"):
        load_run_artifact(run_root)
