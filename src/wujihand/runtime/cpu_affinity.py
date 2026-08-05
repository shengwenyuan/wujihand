"""Explicit Linux CPU affinity for latency-sensitive runtime processes."""

from __future__ import annotations

import os
from typing import Callable, Iterable


AffinityGetter = Callable[[int], Iterable[int]]
AffinitySetter = Callable[[int, Iterable[int]], None]


def parse_cpu_affinity(value: str) -> tuple[int, ...]:
    """Parse a taskset-style CPU list such as ``0-3,8,10-11``."""

    cpus: set[int] = set()
    for raw_part in value.split(","):
        part = raw_part.strip()
        if not part:
            raise ValueError("CPU affinity contains an empty segment")
        bounds = part.split("-")
        if len(bounds) == 1:
            cpus.add(_cpu_index(bounds[0]))
            continue
        if len(bounds) != 2:
            raise ValueError(f"invalid CPU affinity segment: {part!r}")
        start = _cpu_index(bounds[0])
        stop = _cpu_index(bounds[1])
        if stop < start:
            raise ValueError(f"CPU affinity range is reversed: {part!r}")
        cpus.update(range(start, stop + 1))
    if not cpus:
        raise ValueError("CPU affinity must select at least one CPU")
    return tuple(sorted(cpus))


def configure_current_process_cpu_affinity(
    value: str | None,
    *,
    get_affinity: AffinityGetter | None = None,
    set_affinity: AffinitySetter | None = None,
) -> tuple[int, ...] | None:
    """Apply and verify one affinity before latency-sensitive libraries start."""

    if value is None or not value.strip():
        return None
    requested = parse_cpu_affinity(value)
    getter = get_affinity or getattr(os, "sched_getaffinity", None)
    setter = set_affinity or getattr(os, "sched_setaffinity", None)
    if getter is None or setter is None:
        raise RuntimeError("CPU affinity requires Linux sched_getaffinity support")
    available = set(getter(0))
    unavailable = set(requested) - available
    if unavailable:
        rendered = ",".join(str(cpu) for cpu in sorted(unavailable))
        raise ValueError(f"CPU affinity selects unavailable CPUs: {rendered}")
    setter(0, requested)
    applied = tuple(sorted(set(getter(0))))
    if applied != requested:
        raise RuntimeError(
            f"CPU affinity verification failed: requested={requested}, applied={applied}"
        )
    return applied


def _cpu_index(value: str) -> int:
    stripped = value.strip()
    if not stripped.isdecimal():
        raise ValueError(f"invalid CPU index: {value!r}")
    return int(stripped)


__all__ = ["configure_current_process_cpu_affinity", "parse_cpu_affinity"]
