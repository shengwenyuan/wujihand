"""Socket-level checks for the local q20 transport."""

from __future__ import annotations

import time

import numpy as np
import pytest

from wujihand.adapters.transport import UdpJointCommandReceiver, UdpJointCommandSender


@pytest.mark.requires_socket
def test_loopback_receiver_keeps_newest_packet() -> None:
    with UdpJointCommandReceiver(0) as receiver, UdpJointCommandSender(receiver.port) as sender:
        first = sender.send(np.zeros(20), host_time_ns=100)
        second = sender.send(np.ones(20), host_time_ns=200)
        time.sleep(0.01)

        latest = receiver.receive_latest()

        assert first.sequence == 0
        assert second.sequence == 1
        assert latest is not None
        assert latest.sequence == 1
        assert latest.host_time_ns == 200
        np.testing.assert_array_equal(latest.q20, np.ones(20))
        assert receiver.accepted == 1
        assert receiver.rejected == 0
        assert receiver.receive_latest() is None
