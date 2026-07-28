from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path

import pytest

from wujihand.adapters.storage import (
    decode_clutch_event_json,
    decode_tracking_sample_json,
    encode_clutch_event_json,
    encode_tracking_sample_json,
    read_tracking_samples_jsonl,
    write_tracking_samples_jsonl,
)
from wujihand.domain.tracking import (
    ClutchEdge,
    ClutchEvent,
    TrackedRigidBodySample,
    TrackingState,
)


def _sample(**overrides: object) -> TrackedRigidBodySample:
    values: dict[str, object] = {
        "stream_id": "operator_tracker_right",
        "device_serial": "sanitized-tracker-1",
        "logical_role": "operator_right",
        "sequence": 2,
        "tracking_frame": "vive_tracking",
        "position_m": (0.1, 0.2, 0.3),
        "quat_wxyz": (1.0, 0.0, 0.0, 0.0),
        "connected": True,
        "pose_valid": True,
        "tracking_state": TrackingState.RUNNING,
        "quality": 1.0,
        "host_time_ns": 10,
        "device_time_ns": None,
    }
    values.update(overrides)
    return TrackedRigidBodySample(**values)  # type: ignore[arg-type]


def _event() -> ClutchEvent:
    return ClutchEvent(
        stream_id="operator_tracker_right",
        device_serial="sanitized-tracker-1",
        logical_role="operator_right",
        input_id="tracker_clutch",
        edge=ClutchEdge.PRESSED,
        sequence=0,
        host_time_ns=10,
        epoch_request=True,
    )


def test_tracking_sample_json_round_trip_preserves_canonical_fields() -> None:
    decoded = decode_tracking_sample_json(encode_tracking_sample_json(_sample()))

    assert decoded == _sample()
    assert decoded.position_m == (0.1, 0.2, 0.3)
    assert decoded.quat_wxyz == (1.0, 0.0, 0.0, 0.0)


def test_invalid_tracking_sample_round_trip_never_invents_pose() -> None:
    lost = _sample(
        position_m=None,
        quat_wxyz=None,
        pose_valid=False,
        tracking_state=TrackingState.LOST,
        quality=None,
    )

    decoded = decode_tracking_sample_json(encode_tracking_sample_json(lost))

    assert not decoded.pose_valid
    assert decoded.position_m is None
    assert decoded.quat_wxyz is None


def test_clutch_event_json_round_trip_preserves_edge_and_epoch_request() -> None:
    assert decode_clutch_event_json(encode_clutch_event_json(_event())) == _event()


@pytest.mark.parametrize(
    "mutation",
    [
        lambda payload: payload.pop("tracking_frame"),
        lambda payload: payload.update({"extra": 1}),
        lambda payload: payload.update({"tracking_state": "made_up"}),
        lambda payload: payload.update({"position_m": [True, 0.0, 0.0]}),
        lambda payload: payload.update({"quat_wxyz": [0.0, 0.0, 0.0, 0.0]}),
    ],
)
def test_tracking_decoder_rejects_schema_mutations(
    mutation: Callable[[dict[str, object]], object],
) -> None:
    payload = json.loads(encode_tracking_sample_json(_sample()))
    mutation(payload)
    with pytest.raises(ValueError):
        decode_tracking_sample_json(json.dumps(payload))


def test_tracking_decoder_rejects_duplicate_and_nonfinite_json() -> None:
    encoded = encode_tracking_sample_json(_sample())
    duplicate = encoded[:-1] + ',"quality":0.5}'
    with pytest.raises(ValueError, match="strict JSON"):
        decode_tracking_sample_json(duplicate)
    with pytest.raises(ValueError, match="strict JSON"):
        decode_tracking_sample_json(encoded.replace('"quality":1.0', '"quality":NaN'))


def test_jsonl_reader_rejects_truncated_final_record(tmp_path: Path) -> None:
    path = tmp_path / "samples.jsonl"
    path.write_text(encode_tracking_sample_json(_sample()), encoding="utf-8")
    with pytest.raises(ValueError, match="truncated"):
        read_tracking_samples_jsonl(path)


def test_jsonl_file_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "samples.jsonl"
    samples = (_sample(sequence=0), _sample(sequence=1, host_time_ns=11))
    write_tracking_samples_jsonl(path, samples)
    assert read_tracking_samples_jsonl(path) == samples
