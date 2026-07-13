from __future__ import annotations

import json
import uuid

import numpy as np
import pytest

from wujihand.adapters.transport import (
    JointCommandPacket,
    decode_packet,
    encode_packet,
)


def packet(timestamp: int = 10) -> JointCommandPacket:
    return JointCommandPacket(str(uuid.uuid4()), 2, timestamp, np.arange(20, dtype=float))


def test_packet_round_trip() -> None:
    original = packet()
    decoded = decode_packet(encode_packet(original))
    assert decoded.session_id == original.session_id
    assert decoded.sequence == 2
    np.testing.assert_array_equal(decoded.q20, original.q20)


def test_packet_rejects_schema_shape_and_nan() -> None:
    payload = json.loads(encode_packet(packet()))
    payload["schema"] = "wrong"
    with pytest.raises(ValueError, match="mismatch"):
        decode_packet(json.dumps(payload).encode())
    with pytest.raises(ValueError, match="shape"):
        JointCommandPacket(str(uuid.uuid4()), 0, 0, np.zeros(19))
    with pytest.raises(ValueError, match="NaN"):
        JointCommandPacket(str(uuid.uuid4()), 0, 0, np.full(20, np.nan))


def test_packet_rejects_non_numeric_q20() -> None:
    payload = json.loads(encode_packet(packet()))
    payload["q20"] = ["not-a-number"] * 20
    with pytest.raises(ValueError, match="numeric"):
        decode_packet(json.dumps(payload).encode())
