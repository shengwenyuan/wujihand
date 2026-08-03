from __future__ import annotations

import pytest

from teleoperation_quality.statistics import SequenceRow, distribution, sequence_metrics


def test_distribution_has_frozen_linear_quantiles_and_missing_count() -> None:
    result = distribution([1.0, 2.0, None, 4.0])

    assert result["count"] == 3
    assert result["missing"] == 1
    assert result["p50"] == pytest.approx(2.0)
    assert result["p95"] == pytest.approx(3.8)
    assert result["iqr"] == pytest.approx(1.5)


def test_sequence_gaps_use_unique_sequence_set_without_reorder_double_count() -> None:
    result = sequence_metrics(SequenceRow("producer", 7, value) for value in (1, 3, 2, 3, 5))[0]

    assert result["unique_received"] == 4
    assert result["inferred_missing"] == 1
    assert result["duplicates"] == 1
    assert result["reordered"] == 1
    assert result["observed_discontinuity_ratio"] == pytest.approx(0.2)
