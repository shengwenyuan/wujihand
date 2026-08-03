from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from teleoperation_quality.metrics import AnalysisConfig
from teleoperation_quality.model import BagDataset
from teleoperation_quality.pipeline import analyze_run


class FakeReader:
    def __init__(self, dataset: BagDataset) -> None:
        self.dataset = dataset

    def read(
        self,
        rosbag_root: str | Path,
        *,
        expected_run_id: str | None = None,
    ) -> BagDataset:
        assert Path(rosbag_root).name == "rosbag2"
        assert expected_run_id == "fixture-run"
        return self.dataset


def test_pipeline_is_atomic_checksummed_and_does_not_modify_input(
    run_root: Path,
    dataset: BagDataset,
    tmp_path: Path,
) -> None:
    before = {
        path.relative_to(run_root): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in run_root.rglob("*")
        if path.is_file()
    }
    output = tmp_path / "quality-output"

    result = analyze_run(
        run_root,
        output,
        reader=FakeReader(dataset),
        config=AnalysisConfig(
            expected_control_hz=50.0,
            control_rate_tolerance_fraction=0.01,
            p95_tick_interval_limit_ms=20.1,
            p95_comparable_input_age_limit_ms=10.0,
        ),
    )

    assert result == output.resolve()
    assert (output / "summary.json").is_file()
    assert (output / "report.html").is_file()
    assert (output / "derived" / "aligned_ticks.csv").is_file()
    assert (output / "derived" / "q27_samples.csv").is_file()
    assert len(tuple((output / "plots").glob("*.png"))) == 12

    checksum_lines = (output / "checksums.sha256").read_text(encoding="utf-8").splitlines()
    for line in checksum_lines:
        expected, relative = line.split(maxsplit=1)
        assert hashlib.sha256((output / relative).read_bytes()).hexdigest() == expected
    after = {
        path.relative_to(run_root): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in run_root.rglob("*")
        if path.is_file()
    }
    assert after == before


def test_pipeline_refuses_to_write_inside_immutable_run(
    run_root: Path,
    dataset: BagDataset,
) -> None:
    with pytest.raises(ValueError, match="must not be inside"):
        analyze_run(
            run_root,
            run_root / "analysis",
            reader=FakeReader(dataset),
        )
