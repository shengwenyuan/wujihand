"""Socket-level checks for the atomic hand command transport."""

from __future__ import annotations

import time
import uuid

import numpy as np
import pytest

from wujihand.adapters.transport import UdpHandCommandReceiver, UdpHandCommandSender


@pytest.mark.requires_socket
def test_hand_command_loopback_receiver_keeps_newest_atomic_command() -> None:
    with UdpHandCommandReceiver(0) as receiver, UdpHandCommandSender(receiver.port) as sender:
        sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=100,
            quality=0.8,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)
        assert receiver.receive_latest() is not None
        first = sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=150,
            quality=0.8,
            calibration_id="neutral-a",
        )
        second = sender.send(
            np.ones(20),
            [0.0, 1.0, 0.0, 0.0],
            host_time_ns=200,
            quality=0.9,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)

        latest = receiver.receive_latest()

        assert first.sequence == 1
        assert second.sequence == 2
        assert latest is not None
        assert latest.sequence == 2
        assert latest.host_time_ns == 200
        assert latest.quality == 0.9
        np.testing.assert_array_equal(latest.q20, np.ones(20))
        np.testing.assert_array_equal(latest.root_delta_quat_wxyz, [0.0, 1.0, 0.0, 0.0])
        assert receiver.accepted == 2
        assert receiver.rejected == 0


@pytest.mark.requires_socket
def test_hand_command_receiver_preserves_new_epoch_identity_before_latest_delta() -> None:
    with UdpHandCommandReceiver(0) as receiver, UdpHandCommandSender(receiver.port) as sender:
        identity = sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=100,
            quality=1.0,
            calibration_id="neutral-a",
        )
        delta = sender.send(
            np.ones(20),
            [0.0, 1.0, 0.0, 0.0],
            host_time_ns=200,
            quality=1.0,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)

        received_identity = receiver.receive_latest()
        received_delta = receiver.receive_latest()
        assert received_identity is not None
        assert received_delta is not None
        assert received_identity.sequence == identity.sequence
        assert received_identity.calibration_id == identity.calibration_id
        assert received_delta.sequence == delta.sequence
        assert received_delta.calibration_id == delta.calibration_id
        assert receiver.accepted == 2
        assert receiver.rejected == 0


@pytest.mark.requires_socket
def test_hand_command_receiver_rejects_future_watermark_without_poisoning_stream() -> None:
    with UdpHandCommandReceiver(0) as receiver, UdpHandCommandSender(receiver.port) as sender:
        sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=time.monotonic_ns() + 1_000_000_000,
            quality=1.0,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)
        assert receiver.receive_latest() is None

        command = sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=time.monotonic_ns(),
            quality=1.0,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)
        received = receiver.receive_latest()
        assert received is not None
        assert received.sequence == command.sequence
        assert receiver.rejected == 1


@pytest.mark.requires_socket
def test_hand_command_receiver_rejects_stale_time_and_sequence() -> None:
    session_id = str(uuid.uuid4())
    with (
        UdpHandCommandReceiver(0) as receiver,
        UdpHandCommandSender(receiver.port, session_id=session_id) as sender,
    ):
        sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=200,
            quality=1.0,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)
        assert receiver.receive_latest() is not None

        sender.sequence = 0
        sender.send(
            np.ones(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=300,
            quality=1.0,
            calibration_id="neutral-a",
        )
        sender.sequence = 2
        sender.send(
            np.ones(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=100,
            quality=1.0,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)

        assert receiver.receive_latest() is None
        assert receiver.accepted == 1
        assert receiver.rejected == 2


@pytest.mark.requires_socket
def test_hand_command_sender_does_not_advance_sequence_on_validation_failure() -> None:
    with UdpHandCommandReceiver(0) as receiver, UdpHandCommandSender(receiver.port) as sender:
        with pytest.raises(ValueError, match="unit norm"):
            sender.send(
                np.zeros(20),
                [0.0, 0.0, 0.0, 0.0],
                host_time_ns=100,
                quality=1.0,
                calibration_id="neutral-a",
            )
        command = sender.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=101,
            quality=1.0,
            calibration_id="neutral-a",
        )
        assert command.sequence == 0


@pytest.mark.requires_socket
def test_hand_command_receiver_keeps_sequence_history_across_session_switches() -> None:
    session_a = str(uuid.uuid4())
    session_b = str(uuid.uuid4())
    with (
        UdpHandCommandReceiver(0) as receiver,
        UdpHandCommandSender(receiver.port, session_id=session_a) as sender_a,
        UdpHandCommandSender(receiver.port, session_id=session_b) as sender_b,
    ):
        sender_a.sequence = 5
        sender_a.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=100,
            quality=1.0,
            calibration_id="neutral-a",
        )
        sender_b.send(
            np.zeros(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=200,
            quality=1.0,
            calibration_id="neutral-b",
        )
        time.sleep(0.01)
        latest = receiver.receive_latest()
        assert latest is not None
        assert latest.session_id == session_b

        sender_a.sequence = 5
        sender_a.send(
            np.ones(20),
            [1.0, 0.0, 0.0, 0.0],
            host_time_ns=300,
            quality=1.0,
            calibration_id="neutral-a",
        )
        time.sleep(0.01)
        assert receiver.receive_latest() is None
        assert receiver.rejected == 1
