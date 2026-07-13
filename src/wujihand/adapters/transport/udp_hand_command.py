"""Strict loopback UDP transport for atomic Hand 2 commands."""

from __future__ import annotations

import json
import socket
import time
import uuid
from collections.abc import Sequence
from typing import Any, Final

import numpy as np

from wujihand.ports import HandCommand


MAX_HAND_COMMAND_DATAGRAM_BYTES: Final = 4096
HAND_COMMAND_LOOPBACK: Final = "127.0.0.1"

_FIELDS: Final = frozenset(
    {
        "schema",
        "layout",
        "session_id",
        "sequence",
        "host_time_ns",
        "q20",
        "root_delta_quat_wxyz",
        "pose_frame",
        "quat_order",
        "quality",
        "calibration_id",
    }
)


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number {value!r} is not allowed")


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON field {key!r}")
        result[key] = value
    return result


def _numeric_vector(value: object, *, field: str) -> np.ndarray[Any, np.dtype[np.float64]]:
    if not isinstance(value, list) or any(type(item) not in (int, float) for item in value):
        raise ValueError(f"{field} must be a JSON array of numbers")
    try:
        return np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{field} must be a JSON array of finite numbers") from exc


def encode_hand_command(command: HandCommand) -> bytes:
    """Serialize one already-validated canonical command."""

    payload = {
        "schema": command.schema,
        "layout": command.layout,
        "session_id": command.session_id,
        "sequence": command.sequence,
        "host_time_ns": command.host_time_ns,
        "q20": command.q20.tolist(),
        "root_delta_quat_wxyz": command.root_delta_quat_wxyz.tolist(),
        "pose_frame": command.pose_frame,
        "quat_order": command.quat_order,
        "quality": command.quality,
        "calibration_id": command.calibration_id,
    }
    encoded = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_HAND_COMMAND_DATAGRAM_BYTES:
        raise ValueError("hand command datagram is too large")
    return encoded


def decode_hand_command(data: bytes) -> HandCommand:
    """Decode a datagram only when it exactly matches the v2 wire contract."""

    if not isinstance(data, bytes) or not data or len(data) > MAX_HAND_COMMAND_DATAGRAM_BYTES:
        raise ValueError("invalid hand command datagram size")
    try:
        payload = json.loads(
            data,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("hand command datagram is not valid strict JSON") from exc
    if not isinstance(payload, dict) or set(payload) != _FIELDS:
        raise ValueError("hand command fields do not match schema")

    q20 = _numeric_vector(payload["q20"], field="q20")
    root_quat = _numeric_vector(
        payload["root_delta_quat_wxyz"],
        field="root_delta_quat_wxyz",
    )
    return HandCommand(
        schema=payload["schema"],
        layout=payload["layout"],
        session_id=payload["session_id"],
        sequence=payload["sequence"],
        host_time_ns=payload["host_time_ns"],
        q20=q20,
        root_delta_quat_wxyz=root_quat,
        pose_frame=payload["pose_frame"],
        quat_order=payload["quat_order"],
        quality=payload["quality"],
        calibration_id=payload["calibration_id"],
    )


class UdpHandCommandSender:
    """Send atomic commands exclusively to an IPv4 loopback endpoint."""

    def __init__(self, port: int, *, session_id: str | None = None) -> None:
        if not 1 <= port <= 65535:
            raise ValueError("port must be in [1, 65535]")
        self._destination = (HAND_COMMAND_LOOPBACK, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.session_id = str(uuid.uuid4()) if session_id is None else session_id
        try:
            uuid.UUID(self.session_id)
        except (AttributeError, ValueError) as exc:
            self._socket.close()
            raise ValueError("session_id must be a UUID string") from exc
        self.sequence = 0

    def send(
        self,
        q20: Sequence[float],
        root_delta_quat_wxyz: Sequence[float],
        *,
        host_time_ns: int,
        quality: float,
        calibration_id: str,
    ) -> HandCommand:
        command = HandCommand(
            session_id=self.session_id,
            sequence=self.sequence,
            host_time_ns=host_time_ns,
            q20=np.asarray(q20, dtype=np.float64),
            root_delta_quat_wxyz=np.asarray(root_delta_quat_wxyz, dtype=np.float64),
            quality=quality,
            calibration_id=calibration_id,
        )
        self._socket.sendto(encode_hand_command(command), self._destination)
        self.sequence += 1
        return command

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpHandCommandSender:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class UdpHandCommandReceiver:
    """Drain loopback commands while preserving each calibration epoch's first packet.

    Within an established epoch only the newest command is returned.  When a
    drain contains a new epoch's identity followed by deltas, the first command
    is delivered now and the newest delta is deferred to the next call.
    """

    def __init__(self, port: int) -> None:
        if not 0 <= port <= 65535:
            raise ValueError("port must be in [0, 65535]")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((HAND_COMMAND_LOOPBACK, port))
        self._socket.setblocking(False)
        self.last_host_time_ns = -1
        self.last_session_id: str | None = None
        self.last_sequence = -1
        self.last_calibration_id: str | None = None
        self._last_sequence_by_session: dict[str, int] = {}
        self._deferred_latest: HandCommand | None = None
        self.accepted = 0
        self.rejected = 0

    @property
    def port(self) -> int:
        return int(self._socket.getsockname()[1])

    def receive_latest(self) -> HandCommand | None:
        candidates: list[HandCommand] = []
        while True:
            try:
                data, address = self._socket.recvfrom(MAX_HAND_COMMAND_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                break
            if address[0] != HAND_COMMAND_LOOPBACK:
                self.rejected += 1
                continue
            try:
                command = decode_hand_command(data)
            except ValueError:
                self.rejected += 1
                continue
            if command.host_time_ns > time.monotonic_ns():
                self.rejected += 1
                continue
            if command.host_time_ns <= self.last_host_time_ns:
                self.rejected += 1
                continue
            candidates.append(command)

        ordered: list[HandCommand] = []
        for command in sorted(candidates, key=lambda item: (item.host_time_ns, item.sequence)):
            previous_sequence = self._last_sequence_by_session.get(command.session_id, -1)
            if command.sequence <= previous_sequence:
                self.rejected += 1
                continue
            # Remember every drained monotonic command, including valid commands
            # superseded by a newer session in the same receive cycle.
            self._last_sequence_by_session[command.session_id] = command.sequence
            ordered.append(command)

        if self._deferred_latest is not None:
            ordered.append(self._deferred_latest)
            self._deferred_latest = None
            ordered.sort(key=lambda item: (item.host_time_ns, item.sequence))

        selected: HandCommand | None = None
        if ordered:
            newest = ordered[-1]
            selected = newest
            if newest.calibration_id != self.last_calibration_id:
                selected = next(
                    command
                    for command in ordered
                    if command.calibration_id == newest.calibration_id
                )
                if selected is not newest:
                    self._deferred_latest = newest

        if selected is not None:
            self.last_host_time_ns = selected.host_time_ns
            self.last_session_id = selected.session_id
            self.last_sequence = selected.sequence
            self.last_calibration_id = selected.calibration_id
            self.accepted += 1
        return selected

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> UdpHandCommandReceiver:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
