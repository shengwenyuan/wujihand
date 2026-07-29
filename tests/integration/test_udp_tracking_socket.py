"""Socket-level checks for canonical Tracker loopback transport."""

from __future__ import annotations

from dataclasses import replace
import time

import pytest

from wujihand.adapters.transport import (
    UdpTrackingSampleReceiver,
    UdpTrackingSampleSender,
)
from wujihand.application.teleoperation import TrackerReferenceReadinessGate
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


def calibrating_sample(
    sequence: int,
    host_time_ns: int,
) -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=None,
        quat_wxyz=None,
        connected=True,
        pose_valid=False,
        tracking_state=TrackingState.CALIBRATING,
        quality=None,
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
def test_tracking_receiver_batch_preserves_transient_calibrating_state() -> None:
    with UdpTrackingSampleReceiver(
        0,
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
    ) as receiver, UdpTrackingSampleSender(receiver.port) as sender:
        sender.send(sample(0, 100))
        sender.send(calibrating_sample(1, 200))
        sender.send(sample(2, 300))
        time.sleep(0.01)

        received = receiver.receive_available(now_ns=400)

        assert [item.sequence for item in received] == [0, 1, 2]
        assert [item.tracking_state for item in received] == [
            TrackingState.RUNNING,
            TrackingState.CALIBRATING,
            TrackingState.RUNNING,
        ]
        assert receiver.accepted == 1
        assert receiver.rejected == 0


@pytest.mark.requires_socket
def test_calibrating_between_running_packets_prevents_reference_readiness() -> None:
    gate = TrackerReferenceReadinessGate(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
        stable_after_s=0.000_000_2,
        max_sample_gap_s=0.25,
    )
    with UdpTrackingSampleReceiver(
        0,
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        tracking_frame="vive_tracking",
    ) as receiver, UdpTrackingSampleSender(receiver.port) as sender:
        sender.send(sample(0, 100))
        time.sleep(0.01)
        first_batch = tuple(
            gate.observe(received)
            for received in receiver.receive_available(now_ns=150)
        )
        assert len(first_batch) == 1
        assert not first_batch[0].ready

        sender.send(calibrating_sample(1, 200))
        sender.send(sample(2, 400))
        time.sleep(0.01)
        decisions = tuple(
            gate.observe(received)
            for received in receiver.receive_available(now_ns=500)
        )

        assert [decision.reason for decision in decisions] == [
            "tracking_calibrating",
            "stabilizing_running",
        ]
        assert not decisions[-1].ready
        assert decisions[-1].consecutive_running_samples == 1


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
