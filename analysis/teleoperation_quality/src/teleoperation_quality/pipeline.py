"""Atomic, read-only-input analysis pipeline."""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Protocol
from uuid import uuid4

from .artifact import RunArtifact, load_run_artifact
from .metrics import AnalysisConfig, MetricBundle, compute_metrics
from .model import BagDataset
from .plots import write_plots
from .report import write_report
from .ros2_reader import Ros2BagReader
from .version import ANALYZER_SCHEMA, ANALYZER_VERSION


class DatasetReader(Protocol):
    def read(
        self,
        rosbag_root: str | Path,
        *,
        expected_run_id: str | None = None,
    ) -> BagDataset: ...


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _csv_value(value: Any) -> str | int | float:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    if isinstance(value, (str, int, float)):
        return value
    return str(value)


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        if not fields:
            stream.write("\n")
            return
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="raise")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key)) for key in fields})


def _flatten_summary(value: Any, *, prefix: str = "") -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if isinstance(value, dict):
        for key in sorted(value):
            path = f"{prefix}.{key}" if prefix else str(key)
            rows.extend(_flatten_summary(value[key], prefix=path))
    else:
        rows.append({"metric": prefix, "value": value})
    return rows


def _analyzer_source_hashes() -> dict[str, str]:
    package_root = Path(__file__).resolve().parent
    project_root = package_root.parent.parent
    paths = sorted(package_root.glob("*.py"))
    for name in ("pyproject.toml", "uv.lock", "README.md"):
        candidate = project_root / name
        if candidate.is_file():
            paths.append(candidate)
    return {path.relative_to(project_root).as_posix(): _sha256(path) for path in sorted(paths)}


def _write_tables(root: Path, bundle: MetricBundle) -> None:
    for name, rows in sorted(bundle.tables.items()):
        _write_csv(root / f"{name}.csv", rows)
    derived = root / "derived"
    derived.mkdir()
    for name, rows in sorted(bundle.derived_tables.items()):
        _write_csv(derived / f"{name}.csv", rows)


def _write_checksums(root: Path) -> dict[str, str]:
    paths = sorted(
        path for path in root.rglob("*") if path.is_file() and path.name != "checksums.sha256"
    )
    checksums = {path.relative_to(root).as_posix(): _sha256(path) for path in paths}
    content = "".join(f"{digest}  {relative}\n" for relative, digest in checksums.items())
    (root / "checksums.sha256").write_text(content, encoding="utf-8")
    return checksums


def _write_output(
    root: Path,
    artifact: RunArtifact,
    bundle: MetricBundle,
    config: AnalysisConfig,
) -> None:
    summary = {
        "schema": ANALYZER_SCHEMA,
        "analyzer_version": ANALYZER_VERSION,
        **bundle.summary,
    }
    _write_json(root / "summary.json", summary)
    _write_csv(root / "summary.csv", _flatten_summary(summary))
    _write_tables(root, bundle)
    figures = write_plots(bundle, root / "plots")
    _write_csv(root / "figure_manifest.csv", figures)
    write_report(
        MetricBundle(
            summary=summary,
            tables=bundle.tables,
            derived_tables=bundle.derived_tables,
        ),
        figures,
        root / "report.html",
    )
    analyzer_manifest = {
        "schema": ANALYZER_SCHEMA,
        "analyzer_version": ANALYZER_VERSION,
        "generated_utc": datetime.now(UTC).isoformat(),
        "run_id": artifact.run_id,
        "input_root": str(artifact.root),
        "input_checksums": artifact.input_checksums,
        "analysis_config": config.to_mapping(),
        "analysis_window": bundle.summary["analysis_window"],
        "analyzer_source_sha256": _analyzer_source_hashes(),
    }
    _write_json(root / "analyzer_manifest.json", analyzer_manifest)
    _write_checksums(root)


def analyze_run(
    run_root: str | Path,
    output_root: str | Path,
    *,
    config: AnalysisConfig | None = None,
    reader: DatasetReader | None = None,
) -> Path:
    """Analyze one immutable run and atomically create a new output directory."""

    artifact = load_run_artifact(run_root)
    if config is None:
        config = AnalysisConfig()
    output = Path(output_root).expanduser().resolve()
    if output.exists():
        raise ValueError(f"analysis output already exists: {output}")
    if output.is_relative_to(artifact.root):
        raise ValueError("analysis output must not be inside the immutable input run")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.parent / f".{output.name}.tmp-{uuid4().hex}"
    temporary.mkdir()
    try:
        source = Ros2BagReader() if reader is None else reader
        dataset = source.read(
            artifact.root / "raw" / "rosbag2",
            expected_run_id=artifact.run_id,
        )
        bundle = compute_metrics(artifact, dataset, config)
        revalidated = load_run_artifact(artifact.root)
        if revalidated.input_checksums != artifact.input_checksums:
            raise ValueError("input checksums changed while analysis was running")
        _write_output(temporary, artifact, bundle, config)
        temporary.rename(output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return output


__all__ = ["DatasetReader", "analyze_run"]
