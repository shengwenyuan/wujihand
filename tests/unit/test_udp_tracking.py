from __future__ import annotations

import json

import pytest

from wujihand.adapters.transport import (
    decode_tracking_datagram,
    encode_tracking_datagram,
)
from wujihand.domain import TrackedRigidBodySample, TrackingState


def sample(sequence: int = 4, host_time_ns: int = 100) -> TrackedRigidBodySample:
    return TrackedRigidBodySample(
        stream_id="vive.right",
        device_serial="LHR-24B6E288",
        logical_role="operator_right",
        producer_instance="openvr_fixture",
        transport_epoch=1,
        tracking_setup_revision="standing_fixture_v1",
        sequence=sequence,
        tracking_frame="vive_tracking",
        position_m=(0.1, 0.2, 0.3),
        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        connected=True,
        pose_valid=True,
        tracking_state=TrackingState.RUNNING,
        quality=1.0,
        host_time_ns=host_time_ns,
        device_time_ns=None,
    )


def test_tracking_datagram_round_trip_preserves_canonical_contract() -> None:
    original = sample()
    decoded = decode_tracking_datagram(encode_tracking_datagram(original))

    assert decoded == original


def test_tracking_datagram_rejects_schema_drift_and_non_finite_json() -> None:
    payload = json.loads(encode_tracking_datagram(sample()))
    payload["schema"] = "wrong.v1"
    with pytest.raises(ValueError, match="schema"):
        decode_tracking_datagram(json.dumps(payload).encode())

    payload = json.loads(encode_tracking_datagram(sample()))
    payload["position_m"][1] = float("nan")
    with pytest.raises(ValueError, match="strict JSON"):
        decode_tracking_datagram(json.dumps(payload).encode())


def test_tracking_datagram_rejects_empty_and_oversized_input() -> None:
    with pytest.raises(ValueError, match="size"):
        decode_tracking_datagram(b"")
    with pytest.raises(ValueError, match="size"):
        decode_tracking_datagram(b"x" * 2049)
