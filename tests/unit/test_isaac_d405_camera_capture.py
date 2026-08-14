from __future__ import annotations

from pathlib import Path
from threading import Event

import numpy as np
from numpy.typing import NDArray
import pytest

from wujihand_ros2.conversion.camera import CAMERA_RECTIFICATION_ROW_MAJOR
from wujihand.runtime.isaac_d405_camera_capture import (
    POSE_HISTORY_JOIN_TOLERANCE_NS,
    _CompletedFrameSink,
    _is_capture_phase,
    nearest_pose_stamp_ns,
    reference_time_to_stamp_ns,
    scheduled_capture_stamp_ns,
    simulation_seconds_to_stamp_ns,
)
from wujihand.runtime.config_repository import ConfigRepository


ROOT = Path(__file__).parents[2]
CAMERA_PROFILE = "configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"


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


def test_capture_phase_stamp_stays_on_rational_grid_after_long_float_drift() -> None:
    activation_stamp_ns = 900_000_047

    assert (
        scheduled_capture_stamp_ns(
            activation_stamp_ns=activation_stamp_ns,
            capture_rate_hz=30.0,
            control_tick_id=0,
        )
        == 933_333_333
    )
    assert (
        scheduled_capture_stamp_ns(
            activation_stamp_ns=activation_stamp_ns,
            capture_rate_hz=30.0,
            control_tick_id=548,
        )
        == 19_200_000_000
    )


@pytest.mark.parametrize(
    ("rate_hz", "tick_id"),
    ((0.0, 0), (29.5, 0), (30.0, -1)),
)
def test_capture_phase_stamp_rejects_non_d405_grid(
    rate_hz: float,
    tick_id: int,
) -> None:
    with pytest.raises(ValueError):
        scheduled_capture_stamp_ns(
            activation_stamp_ns=0,
            capture_rate_hz=rate_hz,
            control_tick_id=tick_id,
        )


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


def test_camera_info_rectification_is_identity() -> None:
    assert CAMERA_RECTIFICATION_ROW_MAJOR == (
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_pose_history_samples_only_the_frozen_30_hz_capture_phase() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(CAMERA_PROFILE)

    observed = [
        (tick, substep)
        for tick in range(6)
        for substep in range(4)
        if _is_capture_phase(
            profile,
            control_tick_id=tick,
            physics_substep_ordinal=substep,
        )
    ]

    assert observed == [(tick, 3) for tick in range(6)]


def test_completed_frame_sink_moves_host_copy_off_writer_callback() -> None:
    profile = ConfigRepository(ROOT).load_isaac_camera_profile(CAMERA_PROFILE)
    copy_started = Event()
    allow_copy = Event()

    def blocking_host_copy(value: object) -> NDArray[np.generic]:
        copy_started.set()
        if not allow_copy.wait(timeout=2.0):
            raise TimeoutError("test did not release host copy")
        return np.asarray(value).copy()

    sink = _CompletedFrameSink(
        profile,
        clone_payload=lambda value: np.asarray(value).copy(),
        payload_to_numpy=blocking_host_copy,
    )
    try:
        sink.write(
            {
                "reference_time": (1, 30),
                "rgb": np.zeros(profile.rgb.source_shape, dtype=np.uint8),
                profile.depth.annotator: np.ones(
                    profile.depth.source_shape,
                    dtype=np.float32,
                ),
            }
        )
        assert copy_started.wait(timeout=1.0)
        assert sink.pop_all() == ()

        allow_copy.set()
        (record,) = sink.pop_all(wait_for_pending=True)
        assert record.reference_time == (1, 30)
        assert record.rgba.shape == profile.rgb.source_shape
        assert record.depth.shape == profile.depth.source_shape
        assert record.callback_end_ns >= record.callback_start_ns
    finally:
        allow_copy.set()
        sink.close()
