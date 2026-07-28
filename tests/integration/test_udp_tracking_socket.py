"""Socket-level checks for canonical Tracker loopback transport."""

from __future__ import annotations

from dataclasses import replace
import time

import pytest

from wujihand.adapters.transport import (
    UdpTrackingSampleReceiver,
    UdpTrackingSampleSender,
)
from wujihand.domain import TrackedRigidBodySample, TrackingState


def sample(sequence: int, host_time_ns: int) -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=(0.1 * sequence, 0.0, 0.0),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        connected=True,
        pose_valid=True,
        tracking_state=TrackingState.RUNNING,
        quality=1.0,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


@pytest.mark.requires_socket
def test_tracking_receiver_keeps_newest_monotonic_loopback_sample() -> None:
    with UdpTrackingSampleReceiver(
        0,
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
    ) as receiver, UdpTrackingSampleSender(receiver.port) as sender:
        sender.send(sample(0, 100))
        sender.send(sample(1, 200))
        time.sleep(0.01)

        latest = receiver.receive_latest(now_ns=300)

        assert latest is not None
        assert latest.sequence == 1
        assert latest.position_m == pytest.approx((0.1, 0.0, 0.0))
        assert receiver.accepted == 1
        assert receiver.rejected == 0
        assert receiver.receive_latest(now_ns=301) is None


@pytest.mark.requires_socket
def test_tracking_receiver_rejects_wrong_identity_and_future_timestamp() -> None:
    with UdpTrackingSampleReceiver(
        0,
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
    ) as receiver, UdpTrackingSampleSender(receiver.port) as sender:
        sender.send(sample(0, 100))
        sender.send(replace(sample(1, 200), stream_id="vive.left"))
        time.sleep(0.01)

        assert receiver.receive_latest(now_ns=99) is None
        assert receiver.rejected == 2
