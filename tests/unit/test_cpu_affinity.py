from __future__ import annotations

import pytest

from wujihand.runtime import (
    configure_current_process_cpu_affinity,
    parse_cpu_affinity,
)


def test_parse_cpu_affinity_expands_ranges_and_deduplicates() -> None:
    assert parse_cpu_affinity("0-3,2,8,10-11") == (0, 1, 2, 3, 8, 10, 11)


@pytest.mark.parametrize("value", ("", "0,", "3-1", "1-2-3", "cpu0"))
def test_parse_cpu_affinity_rejects_invalid_values(value: str) -> None:
    with pytest.raises(ValueError):
        parse_cpu_affinity(value)


def test_configure_cpu_affinity_applies_and_verifies_requested_set() -> None:
    current = {0, 1, 2, 3}

    def get_affinity(_: int) -> set[int]:
        return set(current)

    def set_affinity(_: int, value: object) -> None:
        current.clear()
        current.update(value)  # type: ignore[arg-type]

    applied = configure_current_process_cpu_affinity(
        "0-1",
        get_affinity=get_affinity,
        set_affinity=set_affinity,
    )

    assert applied == (0, 1)
    assert current == {0, 1}


def test_configure_cpu_affinity_rejects_unavailable_cpu() -> None:
    with pytest.raises(ValueError, match="unavailable CPUs: 4"):
        configure_current_process_cpu_affinity(
            "0,4",
            get_affinity=lambda _: {0, 1},
            set_affinity=lambda _pid, _cpus: None,
        )
