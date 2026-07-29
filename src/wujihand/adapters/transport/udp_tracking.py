"""Strict loopback UDP transport for canonical Tracker pose samples."""

from __future__ import annotations

import socket
import time

from wujihand.adapters.storage import (
    decode_tracking_sample_json,
    encode_tracking_sample_json,
)
from wujihand.domain import TrackedRigidBodySample


TRACKING_LOOPBACK = "127.0.0.1"
MAX_TRACKING_DATAGRAM_BYTES = 2048


def encode_tracking_datagram(sample: TrackedRigidBodySample) -> bytes:
    """Encode exactly one validated canonical sample into a bounded datagram."""

    if type(sample) is not TrackedRigidBodySample:
        raise TypeError("sample must be a TrackedRigidBodySample")
    encoded = encode_tracking_sample_json(sample).encode("utf-8")
    if len(encoded) > MAX_TRACKING_DATAGRAM_BYTES:
        raise ValueError("tracking sample datagram is too large")
    return encoded


def decode_tracking_datagram(data: bytes) -> TrackedRigidBodySample:
    """Decode one strict canonical sample and reject oversized datagrams."""

    if (
        not isinstance(data, bytes)
        or not data
        or len(data) > MAX_TRACKING_DATAGRAM_BYTES
    ):
        raise ValueError("invalid tracking sample datagram size")
    return decode_tracking_sample_json(data)


class UdpTrackingSampleSender:
    """Publish canonical samples exclusively to an IPv4 loopback endpoint."""

    def __init__(self, port: int) -> None:
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("port must be an integer in [1, 65535]")
        self._destination = (TRACKING_LOOPBACK, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, sample: TrackedRigidBodySample) -> None:
        self._socket.sendto(
            encode_tracking_datagram(sample),
            self._destination,
        )

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpTrackingSampleSender:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class UdpTrackingSampleReceiver:
    """Drain loopback input and expose monotonic canonical samples."""

    def __init__(
        self,
        port: int,
        *,
        stream_id: str,
        device_serial: str,
        logical_role: str,
        producer_instance: str,
        transport_epoch: int,
        tracking_setup_revision: str,
        tracking_frame: str,
    ) -> None:
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("port must be an integer in [0, 65535]")
        for field, value in (
            ("stream_id", stream_id),
            ("device_serial", device_serial),
            ("logical_role", logical_role),
            ("producer_instance", producer_instance),
            ("tracking_setup_revision", tracking_setup_revision),
            ("tracking_frame", tracking_frame),
        ):
            if not isinstance(value, str) or not value or len(value) > 128:
                raise ValueError(f"{field} must be a bounded non-empty string")
        self.stream_id = stream_id
        self.device_serial = device_serial
        self.logical_role = logical_role
        if type(transport_epoch) is not int or transport_epoch < 0:
            raise ValueError("transport_epoch must be a non-negative integer")
        self.producer_instance = producer_instance
        self.transport_epoch = transport_epoch
        self.tracking_setup_revision = tracking_setup_revision
        self.tracking_frame = tracking_frame
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((TRACKING_LOOPBACK, port))
        self._socket.setblocking(False)
        self.last_host_time_ns = -1
        self.last_sequence = -1
        self.accepted = 0
        self.rejected = 0

    @property
    def port(self) -> int:
        return int(self._socket.getsockname()[1])

    def receive_latest(
        self,
        *,
        now_ns: int | None = None,
    ) -> TrackedRigidBodySample | None:
        """Return the newest valid datagram after draining the socket."""

        samples = self.receive_available(now_ns=now_ns)
        return samples[-1] if samples else None

    def receive_available(
        self,
        *,
        now_ns: int | None = None,
    ) -> tuple[TrackedRigidBodySample, ...]:
        """Drain and return every new monotonic canonical sample in order.

        Reference-readiness logic uses this batch form so a transient degraded
        state cannot be hidden by a newer packet from the same socket drain.
        Steady-state consumers may continue to use :meth:`receive_latest`.
        """

        now = time.monotonic_ns() if now_ns is None else now_ns
        if type(now) is not int or now < 0:
            raise ValueError("now_ns must be a non-negative integer")
        candidates: list[TrackedRigidBodySample] = []
        while True:
            try:
                data, address = self._socket.recvfrom(
                    MAX_TRACKING_DATAGRAM_BYTES + 1
                )
            except BlockingIOError:
                break
            if address[0] != TRACKING_LOOPBACK:
                self.rejected += 1
                continue
            try:
                sample = decode_tracking_datagram(data)
            except ValueError:
                self.rejected += 1
                continue
            if (
                sample.stream_id != self.stream_id
                or sample.device_serial != self.device_serial
                or sample.logical_role != self.logical_role
                or sample.producer_instance != self.producer_instance
                or sample.transport_epoch != self.transport_epoch
                or sample.tracking_setup_revision
                != self.tracking_setup_revision
                or sample.tracking_frame != self.tracking_frame
                or sample.host_time_ns > now
            ):
                self.rejected += 1
                continue
            candidates.append(sample)

        selected: list[TrackedRigidBodySample] = []
        previous_host_time_ns = self.last_host_time_ns
        previous_sequence = self.last_sequence
        for sample in sorted(
            candidates,
            key=lambda item: (item.host_time_ns, item.sequence),
        ):
            if (
                sample.host_time_ns <= previous_host_time_ns
                or sample.sequence <= previous_sequence
            ):
                self.rejected += 1
                continue
            selected.append(sample)
            previous_host_time_ns = sample.host_time_ns
            previous_sequence = sample.sequence

        if selected:
            self.last_host_time_ns = selected[-1].host_time_ns
            self.last_sequence = selected[-1].sequence
            self.accepted += 1
        return tuple(selected)

    def authorize_epoch(
        self,
        *,
        producer_instance: str,
        transport_epoch: int,
        tracking_setup_revision: str,
    ) -> None:
        """Authorize one launcher-observed epoch and reject queued old packets."""

        for field, value in (
            ("producer_instance", producer_instance),
            ("tracking_setup_revision", tracking_setup_revision),
        ):
            if not isinstance(value, str) or not value or len(value) > 128:
                raise ValueError(f"{field} must be a bounded non-empty string")
        if type(transport_epoch) is not int or transport_epoch < 0:
            raise ValueError("transport_epoch must be a non-negative integer")
        if (
            producer_instance == self.producer_instance
            and transport_epoch == self.transport_epoch
            and tracking_setup_revision == self.tracking_setup_revision
        ):
            return
        while True:
            try:
                self._socket.recvfrom(MAX_TRACKING_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                break
            self.rejected += 1
        self.producer_instance = producer_instance
        self.transport_epoch = transport_epoch
        self.tracking_setup_revision = tracking_setup_revision
        self.last_host_time_ns = -1
        self.last_sequence = -1

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpTrackingSampleReceiver:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


__all__ = [
    "MAX_TRACKING_DATAGRAM_BYTES",
    "TRACKING_LOOPBACK",
    "UdpTrackingSampleReceiver",
    "UdpTrackingSampleSender",
    "decode_tracking_datagram",
    "encode_tracking_datagram",
]
