from __future__ import annotations

import json
import uuid
from collections.abc import Callable

import numpy as np
import pytest

from wujihand.adapters.transport import decode_hand_command, encode_hand_command
from wujihand.ports import (
    HAND_COMMAND_LAYOUT,
    HAND_COMMAND_POSE_FRAME,
    HAND_COMMAND_QUAT_ORDER,
    HAND_COMMAND_SCHEMA,
    HandCommand,
)


def hand_command(**overrides: object) -> HandCommand:
    values: dict[str, object] = {
        "session_id": str(uuid.uuid4()),
        "sequence": 2,
        "host_time_ns": 10,
        "q20": np.arange(20, dtype=np.float64),
        "root_delta_quat_wxyz": np.array([1.0, 0.0, 0.0, 0.0]),
        "quality": 0.75,
        "calibration_id": "neutral-2026-07-13",
    }
    values.update(overrides)
    return HandCommand(**values)  # type: ignore[arg-type]


def test_hand_command_v2_round_trip_is_atomic() -> None:
    original = hand_command()

    decoded = decode_hand_command(encode_hand_command(original))

    assert decoded.schema == HAND_COMMAND_SCHEMA
    assert decoded.layout == HAND_COMMAND_LAYOUT
    assert decoded.pose_frame == HAND_COMMAND_POSE_FRAME
    assert decoded.quat_order == HAND_COMMAND_QUAT_ORDER
    assert decoded.session_id == original.session_id
    assert decoded.sequence == 2
    assert decoded.host_time_ns == 10
    assert decoded.quality == 0.75
    assert decoded.calibration_id == "neutral-2026-07-13"
    np.testing.assert_array_equal(decoded.q20, np.arange(20, dtype=np.float64))
    np.testing.assert_array_equal(decoded.root_delta_quat_wxyz, [1.0, 0.0, 0.0, 0.0])


def test_hand_command_v2_owns_readonly_vectors() -> None:
    q20 = np.zeros(20)
    root_quat = np.array([1.0, 0.0, 0.0, 0.0])
    command = hand_command(q20=q20, root_delta_quat_wxyz=root_quat)

    q20[0] = 1.0
    root_quat[0] = 0.0

    assert command.q20[0] == 0.0
    assert command.root_delta_quat_wxyz[0] == 1.0
    with pytest.raises(ValueError, match="read-only"):
        command.q20[0] = 1.0


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("schema", "wujihand.hand_command.v1", "schema"),
        ("layout", "wrong", "layout"),
        ("pose_frame", "world", "pose_frame"),
        ("quat_order", "xyzw", "quat_order"),
        ("session_id", "not-a-uuid", "UUID"),
        ("sequence", -1, "non-negative"),
        ("sequence", True, "integers"),
        ("host_time_ns", 1.5, "integers"),
        ("quality", -0.01, r"\[0, 1\]"),
        ("quality", 1.01, r"\[0, 1\]"),
        ("quality", float("nan"), r"\[0, 1\]"),
        ("quality", True, r"\[0, 1\]"),
        ("calibration_id", "", "non-empty"),
        ("calibration_id", " " * 3, "non-empty"),
        ("calibration_id", "x" * 129, "128"),
    ],
)
def test_hand_command_v2_rejects_invalid_metadata(
    field: str,
    value: object,
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        hand_command(**{field: value})


@pytest.mark.parametrize(
    ("field", "value", "error"),
    [
        ("q20", np.zeros(19), "shape"),
        ("q20", np.full(20, np.nan), "NaN"),
        ("q20", ["bad"] * 20, "numeric"),
        ("root_delta_quat_wxyz", np.zeros(4), "unit norm"),
        ("root_delta_quat_wxyz", [2.0, 0.0, 0.0, 0.0], "unit norm"),
        ("root_delta_quat_wxyz", [1.0, 0.0, 0.0], "shape"),
        ("root_delta_quat_wxyz", [float("inf"), 0.0, 0.0, 0.0], "infinity"),
    ],
)
def test_hand_command_v2_rejects_invalid_vectors(
    field: str,
    value: object,
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        hand_command(**{field: value})


def test_hand_command_v2_accepts_quality_boundaries_and_near_unit_quaternion() -> None:
    assert hand_command(quality=0).quality == 0.0
    assert hand_command(quality=1).quality == 1.0
    command = hand_command(root_delta_quat_wxyz=[1.0 + 5.0e-7, 0.0, 0.0, 0.0])
    assert command.root_delta_quat_wxyz[0] == pytest.approx(1.0 + 5.0e-7)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda payload: payload.pop("quality"),
        lambda payload: payload.update({"extra": 1}),
        lambda payload: payload.update({"schema": "wrong"}),
        lambda payload: payload.update({"q20": [True] * 20}),
        lambda payload: payload.update({"root_delta_quat_wxyz": [0.0] * 4}),
    ],
)
def test_hand_command_v2_decoder_rejects_schema_mutations(
    mutation: Callable[[dict[str, object]], object],
) -> None:
    payload = json.loads(encode_hand_command(hand_command()))
    mutation(payload)
    with pytest.raises(ValueError):
        decode_hand_command(json.dumps(payload).encode("utf-8"))


def test_hand_command_v2_decoder_rejects_duplicate_and_nonfinite_json() -> None:
    payload = encode_hand_command(hand_command()).decode("utf-8")
    duplicate = payload[:-1] + ',"quality":0.5}'
    with pytest.raises(ValueError, match="strict JSON"):
        decode_hand_command(duplicate.encode("utf-8"))
    with pytest.raises(ValueError, match="strict JSON"):
        decode_hand_command(payload.replace('"quality":0.75', '"quality":NaN').encode("utf-8"))


@pytest.mark.parametrize("data", [b"", b"not-json", b"{" + b" " * 4096])
def test_hand_command_v2_decoder_rejects_invalid_datagrams(data: bytes) -> None:
    with pytest.raises(ValueError):
        decode_hand_command(data)
