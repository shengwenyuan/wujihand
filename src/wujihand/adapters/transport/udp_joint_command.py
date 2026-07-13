"""Validated loopback-only q20 datagrams for the perception/Isaac boundary."""

from __future__ import annotations

import json
import socket
import uuid
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from wujihand.domain import HAND2_RIGHT_LAYOUT
from wujihand.domain.joints import FloatArray


SCHEMA = "wujihand.q20.v1"
LAYOUT = "wuji_hand2_right_firmware_v1"
MAX_DATAGRAM_BYTES = 4096
LOOPBACK = "127.0.0.1"


@dataclass(frozen=True, slots=True)
class JointCommandPacket:
    session_id: str
    sequence: int
    host_time_ns: int
    q20: FloatArray

    def __post_init__(self) -> None:
        try:
            uuid.UUID(self.session_id)
        except ValueError as exc:
            raise ValueError("session_id must be a UUID") from exc
        if self.sequence < 0 or self.host_time_ns < 0:
            raise ValueError("sequence and host_time_ns must be non-negative")
        q20 = HAND2_RIGHT_LAYOUT.validate_vector(self.q20)
        object.__setattr__(self, "q20", q20.copy())


def encode_packet(packet: JointCommandPacket) -> bytes:
    payload = {
        "schema": SCHEMA,
        "layout": LAYOUT,
        "session_id": packet.session_id,
        "sequence": packet.sequence,
        "host_time_ns": packet.host_time_ns,
        "q20": packet.q20.tolist(),
    }
    encoded = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError("joint command datagram is too large")
    return encoded


def decode_packet(data: bytes) -> JointCommandPacket:
    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("invalid joint command datagram size")
    try:
        payload = json.loads(data)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("joint command datagram is not valid JSON") from exc
    expected = {"schema", "layout", "session_id", "sequence", "host_time_ns", "q20"}
    if not isinstance(payload, dict) or set(payload) != expected:
        raise ValueError("joint command fields do not match schema")
    if payload["schema"] != SCHEMA or payload["layout"] != LAYOUT:
        raise ValueError("joint command schema or layout mismatch")
    if type(payload["sequence"]) is not int or type(payload["host_time_ns"]) is not int:
        raise ValueError("sequence and host_time_ns must be integers")
    try:
        q20 = np.asarray(payload["q20"], dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("q20 is not a numeric joint vector") from exc
    return JointCommandPacket(
        session_id=str(payload["session_id"]),
        sequence=payload["sequence"],
        host_time_ns=payload["host_time_ns"],
        q20=q20,
    )


class UdpJointCommandSender:
    def __init__(self, port: int) -> None:
        if not 1 <= port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        self._destination = (LOOPBACK, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.session_id = str(uuid.uuid4())
        self.sequence = 0

    def send(self, q20: Sequence[float], *, host_time_ns: int) -> JointCommandPacket:
        packet = JointCommandPacket(
            session_id=self.session_id,
            sequence=self.sequence,
            host_time_ns=host_time_ns,
            q20=np.asarray(q20, dtype=np.float64),
        )
        self._socket.sendto(encode_packet(packet), self._destination)
        self.sequence += 1
        return packet

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpJointCommandSender:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class UdpJointCommandReceiver:
    """Drain loopback datagrams and return only the newest monotonic packet."""

    def __init__(self, port: int) -> None:
        if not 0 <= port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((LOOPBACK, port))
        self._socket.setblocking(False)
        self.last_host_time_ns = -1
        self.accepted = 0
        self.rejected = 0

    @property
    def port(self) -> int:
        return int(self._socket.getsockname()[1])

    def receive_latest(self) -> JointCommandPacket | None:
        newest: JointCommandPacket | None = None
        while True:
            try:
                data, address = self._socket.recvfrom(MAX_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                break
            if address[0] != LOOPBACK:
                self.rejected += 1
                continue
            try:
                packet = decode_packet(data)
            except ValueError:
                self.rejected += 1
                continue
            if packet.host_time_ns <= self.last_host_time_ns:
                self.rejected += 1
                continue
            if newest is None or packet.host_time_ns > newest.host_time_ns:
                newest = packet
        if newest is not None:
            self.last_host_time_ns = newest.host_time_ns
            self.accepted += 1
        return newest

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpJointCommandReceiver:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
