"""Wire-compatibility contract for the atomic hand command."""

from __future__ import annotations

import json
import uuid

import numpy as np
import pytest

from wujihand.adapters.transport import (
    JointCommandPacket,
    decode_hand_command,
    decode_packet,
    encode_hand_command,
    encode_packet,
)
from wujihand.ports import HandCommand


V2_FIELDS = {
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


def test_hand_command_v2_exact_wire_fields() -> None:
    command = HandCommand(
        session_id=str(uuid.uuid4()),
        sequence=0,
        host_time_ns=1,
        q20=np.zeros(20),
        root_delta_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        quality=1.0,
        calibration_id="neutral-a",
    )

    payload = json.loads(encode_hand_command(command))

    assert set(payload) == V2_FIELDS
    assert payload["schema"] == "wujihand.hand_command.v2"
    assert payload["layout"] == "wuji_hand2_right_firmware_v1"
    assert payload["pose_frame"] == "hand2_right_neutral"
    assert payload["quat_order"] == "wxyz"


def test_hand_command_v1_and_v2_remain_incompatible() -> None:
    session_id = str(uuid.uuid4())
    v1 = JointCommandPacket(session_id, 0, 1, np.zeros(20))
    v2 = HandCommand(
        session_id=session_id,
        sequence=0,
        host_time_ns=1,
        q20=np.zeros(20),
        root_delta_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        quality=1.0,
        calibration_id="neutral-a",
    )

    with pytest.raises(ValueError, match="fields"):
        decode_hand_command(encode_packet(v1))
    with pytest.raises(ValueError, match="fields"):
        decode_packet(encode_hand_command(v2))
