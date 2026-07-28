from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.teleoperation import RelativeTrackerTranslationMapper
from wujihand.domain import TrackedRigidBodySample, TrackingState


def sample(
    *,
    sequence: int,
    host_time_ns: int,
    position_m: tuple[float, float, float] = (1.0, 2.0, 3.0),
    state: TrackingState = TrackingState.RUNNING,
) -> TrackedRigidBodySample:
    valid = state is TrackingState.RUNNING
    return TrackedRigidBodySample(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=position_m if valid else None,
        quat_wxyz=(1.0, 0.0, 0.0, 0.0) if valid else None,
        connected=True,
        pose_valid=valid,
        tracking_state=state,
        quality=1.0 if valid else None,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


def mapper() -> RelativeTrackerTranslationMapper:
    return RelativeTrackerTranslationMapper(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
        # OpenVR: +x right, +y up, -z forward.
        # Workcell: +x right, +y table-inward, +z up.
        tracker_to_world=((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
        scale=0.25,
        max_delta_m=0.08,
        stale_after_s=0.25,
    )


def test_relative_mapping_freezes_orientation_slice_and_maps_xyz() -> None:
    subject = mapper()
    reference = subject.arm(
        sample(sequence=10, host_time_ns=1_000_000_000),
        (0.4, 0.5, 0.6),
        now_ns=1_010_000_000,
    )
    decision = subject.advance(
        sample(
            sequence=11,
            host_time_ns=1_020_000_000,
            position_m=(1.2, 1.8, 2.8),
        ),
        now_ns=1_030_000_000,
    )

    assert reference.reason == "reference_established"
    assert decision.accepted
    assert not decision.clamped
    assert decision.tracker_delta_m == pytest.approx((0.2, -0.2, -0.2))
    assert decision.world_delta_m == pytest.approx((0.05, 0.05, -0.05), abs=1e-12)
    assert decision.target_position_m == pytest.approx((0.45, 0.55, 0.55))


def test_each_world_axis_is_clamped_about_reference() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        (0.0, 0.0, 0.0),
        now_ns=101,
    )
    decision = subject.advance(
        sample(
            sequence=1,
            host_time_ns=102,
            position_m=(11.0, 12.0, -7.0),
        ),
        now_ns=103,
    )

    assert decision.clamped
    assert decision.world_delta_m == pytest.approx((0.08, 0.08, 0.08))
    assert decision.target_position_m == pytest.approx((0.08, 0.08, 0.08))


def test_missing_sample_holds_briefly_then_requires_new_reference() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=1_000_000_000),
        (0.1, 0.2, 0.3),
        now_ns=1_000_000_001,
    )

    held = subject.advance(None, now_ns=1_100_000_000)
    stale = subject.advance(None, now_ns=1_300_000_001)

    assert held.reason == "no_new_sample_hold"
    assert held.target_position_m == pytest.approx((0.1, 0.2, 0.3))
    assert stale.reason == "stale_input_reference_required"
    assert stale.target_position_m is None
    assert stale.requires_reference
    assert subject.requires_reference


@pytest.mark.parametrize(
    ("replacement", "reason"),
    (
        ({"tracking_state": TrackingState.LOST}, "tracking_lost_reference_required"),
        ({"stream_id": "vive.left"}, "identity_or_frame_mismatch_reference_required"),
    ),
)
def test_invalid_or_wrong_identity_sample_disarms_epoch(
    replacement: dict[str, object],
    reason: str,
) -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=0, host_time_ns=100),
        np.zeros(3),
        now_ns=101,
    )
    values: dict[str, object] = {
        "stream_id": "vive.right",
        "device_serial": "LHR-24B6E288",
        "logical_role": "operator_right",
        "sequence": 1,
        "tracking_frame": "vive_tracking",
        "position_m": (1.0, 2.0, 3.0),
        "quat_wxyz": (1.0, 0.0, 0.0, 0.0),
        "connected": True,
        "pose_valid": True,
        "tracking_state": TrackingState.RUNNING,
        "quality": 1.0,
        "host_time_ns": 102,
        "device_time_ns": None,
    }
    values.update(replacement)
    if values["tracking_state"] is not TrackingState.RUNNING:
        values.update(position_m=None, quat_wxyz=None, pose_valid=False, quality=None)
    decision = subject.advance(
        TrackedRigidBodySample(**values),  # type: ignore[arg-type]
        now_ns=103,
    )

    assert decision.reason == reason
    assert decision.requires_reference
    assert subject.requires_reference


def test_non_monotonic_sample_disarms_instead_of_reusing_pose() -> None:
    subject = mapper()
    subject.arm(
        sample(sequence=3, host_time_ns=100),
        (0.0, 0.0, 0.0),
        now_ns=101,
    )
    decision = subject.advance(
        sample(sequence=3, host_time_ns=102),
        now_ns=103,
    )

    assert decision.reason == "non_monotonic_sequence_reference_required"
    assert decision.requires_reference
