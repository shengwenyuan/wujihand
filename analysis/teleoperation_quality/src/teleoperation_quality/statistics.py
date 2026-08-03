"""Frozen descriptive-statistics and sequence-gap definitions."""

from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from itertools import pairwise

import numpy as np


def distribution(values: Iterable[float | int | None]) -> dict[str, float | int | None]:
    """Describe finite values while retaining an explicit missing count."""

    raw = tuple(values)
    finite = np.asarray(
        [float(value) for value in raw if value is not None and math.isfinite(float(value))],
        dtype=np.float64,
    )
    missing = len(raw) - int(finite.size)
    if finite.size == 0:
        return {
            "count": 0,
            "missing": missing,
            "mean": None,
            "minimum": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "maximum": None,
            "iqr": None,
        }
    q25, p50, q75, p95, p99 = np.percentile(
        finite,
        [25.0, 50.0, 75.0, 95.0, 99.0],
        method="linear",
    )
    return {
        "count": int(finite.size),
        "missing": missing,
        "mean": float(np.mean(finite)),
        "minimum": float(np.min(finite)),
        "p50": float(p50),
        "p95": float(p95),
        "p99": float(p99),
        "maximum": float(np.max(finite)),
        "iqr": float(q75 - q25),
    }


def effective_rate_hz(times_ns: Sequence[int]) -> float | None:
    if len(times_ns) < 2:
        return None
    elapsed_ns = times_ns[-1] - times_ns[0]
    if elapsed_ns <= 0:
        return None
    return (len(times_ns) - 1) * 1e9 / elapsed_ns


@dataclass(frozen=True, slots=True)
class SequenceRow:
    producer_instance: str
    transport_epoch: int
    sequence: int


def sequence_metrics(rows: Iterable[SequenceRow]) -> list[dict[str, float | int | str]]:
    """Measure observed discontinuities without naming them transport loss."""

    groups: dict[tuple[str, int], list[int]] = defaultdict(list)
    for row in rows:
        groups[(row.producer_instance, row.transport_epoch)].append(row.sequence)
    result: list[dict[str, float | int | str]] = []
    for (producer, epoch), values in sorted(groups.items()):
        unique_values = sorted(set(values))
        inferred_missing = sum(
            max(0, current - previous - 1) for previous, current in pairwise(unique_values)
        )
        unique_received = len(unique_values)
        duplicates = len(values) - unique_received
        reordered = sum(current < previous for previous, current in pairwise(values))
        denominator = unique_received + inferred_missing
        result.append(
            {
                "producer_instance": producer,
                "transport_epoch": epoch,
                "observed": len(values),
                "unique_received": unique_received,
                "first_sequence": values[0],
                "last_sequence": values[-1],
                "inferred_missing": inferred_missing,
                "duplicates": duplicates,
                "reordered": reordered,
                "observed_discontinuity_ratio": (
                    inferred_missing / denominator if denominator else 0.0
                ),
            }
        )
    return result


def finite_non_negative_delta_ms(later_ns: int, earlier_ns: int | None) -> float | None:
    if earlier_ns is None or later_ns < earlier_ns:
        return None
    return (later_ns - earlier_ns) / 1e6


def ratio(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


__all__ = [
    "SequenceRow",
    "distribution",
    "effective_rate_hz",
    "finite_non_negative_delta_ms",
    "ratio",
    "sequence_metrics",
]
