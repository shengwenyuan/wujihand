from __future__ import annotations

import pytest

from wujihand.runtime.isaac_d405_camera_capture import (
    POSE_HISTORY_JOIN_TOLERANCE_NS,
    nearest_pose_stamp_ns,
    reference_time_to_stamp_ns,
    simulation_seconds_to_stamp_ns,
)


@pytest.mark.parametrize(
    ("reference", "expected_stamp_ns"),
    (
        ((333_333_333, 1_000_000_000), 333_333_333),
        ((1, 30), 33_333_333),
        ((1, 8), 125_000_000),
        ((1, 2_000_000_000), 1),
    ),
)
def test_completed_reference_time_uses_frozen_nanosecond_rounding(
    reference: tuple[int, int],
    expected_stamp_ns: int,
) -> None:
    assert reference_time_to_stamp_ns(reference) == expected_stamp_ns


def test_simulation_stamp_rejects_invalid_times() -> None:
    assert simulation_seconds_to_stamp_ns(1.0 / 30.0) == 33_333_333
    for value in (-1.0, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="finite and non-negative"):
            simulation_seconds_to_stamp_ns(value)


def test_completed_reference_rejects_invalid_denominator() -> None:
    with pytest.raises(ValueError, match="non-negative"):
        reference_time_to_stamp_ns((1, 0))


def test_pose_history_join_tolerates_only_sub_microsecond_rounding() -> None:
    assert POSE_HISTORY_JOIN_TOLERANCE_NS == 1_000
    assert (
        nearest_pose_stamp_ns(
            (933_333_286, 941_666_620),
            reference_stamp_ns=933_333_333,
        )
        == 933_333_286
    )
    assert (
        nearest_pose_stamp_ns(
            (933_332_332, 941_666_620),
            reference_stamp_ns=933_333_333,
        )
        is None
    )


def test_pose_history_join_is_deterministic_on_an_exact_tie() -> None:
    assert (
        nearest_pose_stamp_ns(
            (99, 101),
            reference_stamp_ns=100,
            tolerance_ns=1,
        )
        == 99
    )
    with pytest.raises(ValueError, match="non-negative"):
        nearest_pose_stamp_ns((100,), reference_stamp_ns=100, tolerance_ns=-1)
