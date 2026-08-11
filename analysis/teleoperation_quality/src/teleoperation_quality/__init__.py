"""Versioned offline analysis for immutable teleoperation runs."""

from typing import TYPE_CHECKING, Any

from .artifact import RunArtifact, load_run_artifact
from .metrics import AnalysisConfig, MetricBundle, compute_metrics
from .model import BagDataset
from .version import ANALYZER_SCHEMA, ANALYZER_VERSION

if TYPE_CHECKING:
    from .pipeline import analyze_run

__all__ = [
    "ANALYZER_SCHEMA",
    "ANALYZER_VERSION",
    "AnalysisConfig",
    "BagDataset",
    "MetricBundle",
    "RunArtifact",
    "analyze_run",
    "compute_metrics",
    "load_run_artifact",
]


def __getattr__(name: str) -> Any:
    if name == "analyze_run":
        from .pipeline import analyze_run

        return analyze_run
    raise AttributeError(name)
