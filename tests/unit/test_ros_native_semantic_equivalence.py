from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace
from typing import Any

import numpy as np

from wujihand.adapters.transport import (
    decode_tracking_datagram,
    encode_tracking_datagram,
)
from wujihand.application.teleoperation import RelativeTrackerPoseMapper
from wujihand.domain import TrackedRigidBodySample, TrackingState
from wujihand_ros2.conversion import (
    tracked_sample_from_message,
    tracked_sample_to_message,
)


def _factory() -> Any:
    return SimpleNamespace()


def _sample(
    *,
    sequence: int,
    host_time_ns: int,
    position_m: tuple[float, float, float],
) -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id="tracker_right",
        device_serial="tracker-right",
        logical_role="operator_right",
        producer_instance="fixture-producer",
        transport_epoch=4,
        tracking_setup_revision="workstation2_v1",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=position_m,
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        connected=True,
        pose_valid=True,
        tracking_state=TrackingState.RUNNING,
        quality=1.0,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


def _mapper() -> RelativeTrackerPoseMapper:
    return RelativeTrackerPoseMapper(
        stream_id="tracker_right",
        device_serial="tracker-right",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
        tracker_to_workcell=(
            (-1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        translation_scale=1.0,
        max_translation_delta_m=0.4,
        rotation_scale=1.0,
        max_rotation_delta_rad=np.deg2rad(90.0),
        stale_after_s=0.25,
    )


def test_udp_and_ros_tracker_wires_produce_identical_mapping() -> None:
    reference = _sample(
        sequence=0,
        host_time_ns=1_000_000_000,
        position_m=(0.0, 0.0, 1.0),
    )
    moved = replace(
        reference,
        sequence=1,
        host_time_ns=1_010_000_000,
        position_m=(0.1, -0.2, 1.05),
    )
    udp_reference = decode_tracking_datagram(
        encode_tracking_datagram(reference)
    )
    udp_moved = decode_tracking_datagram(
        encode_tracking_datagram(moved)
    )
    ros_reference = tracked_sample_from_message(
        tracked_sample_to_message(reference, factory=_factory)
    )
    ros_moved = tracked_sample_from_message(
        tracked_sample_to_message(moved, factory=_factory)
    )
    assert udp_reference == ros_reference == reference
    assert udp_moved == ros_moved == moved

    udp_mapper = _mapper()
    ros_mapper = _mapper()
    udp_mapper.arm(
        udp_reference,
        (0.5, 0.0, 0.7),
        (1.0, 0.0, 0.0, 0.0),
        now_ns=1_001_000_000,
    )
    ros_mapper.arm(
        ros_reference,
        (0.5, 0.0, 0.7),
        (1.0, 0.0, 0.0, 0.0),
        now_ns=1_001_000_000,
    )
    udp_decision = udp_mapper.advance(
        udp_moved,
        now_ns=1_011_000_000,
    )
    ros_decision = ros_mapper.advance(
        ros_moved,
        now_ns=1_011_000_000,
    )

    assert udp_decision.reason == ros_decision.reason
    assert udp_decision.accepted == ros_decision.accepted
    assert np.array_equal(
        udp_decision.target_position_m,
        ros_decision.target_position_m,
    )
    assert np.array_equal(
        udp_decision.target_orientation_wxyz,
        ros_decision.target_orientation_wxyz,
    )
