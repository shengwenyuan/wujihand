"""Strict JSONL codec for VIVE tracking qualification artifacts."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
import json
from pathlib import Path
from typing import Any, Final

from wujihand.domain.tracking import (
    ClutchEdge,
    ClutchEvent,
    TrackedRigidBodySample,
    TrackingState,
)


_SAMPLE_FIELDS: Final = frozenset(
    {
        "schema",
        "stream_id",
        "device_serial",
        "logical_role",
        "sequence",
        "tracking_frame",
        "position_m",
        "quat_wxyz",
        "connected",
        "pose_valid",
        "tracking_state",
        "quality",
        "host_time_ns",
        "device_time_ns",
        "clock_domain",
    }
)
_EVENT_FIELDS: Final = frozenset(
    {
        "schema",
        "stream_id",
        "device_serial",
        "logical_role",
        "input_id",
        "edge",
        "sequence",
        "host_time_ns",
        "clock_domain",
        "epoch_request",
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


def _decode_mapping(data: str | bytes, *, fields: frozenset[str]) -> Mapping[str, object]:
    if not isinstance(data, (str, bytes)) or not data:
        raise ValueError("tracking record must be non-empty JSON")
    try:
        payload = json.loads(
            data,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("tracking record is not valid strict JSON") from exc
    if not isinstance(payload, dict) or set(payload) != fields:
        raise ValueError("tracking record fields do not match schema")
    return payload


def _optional_numeric_array(
    value: object,
    *,
    field: str,
    size: int,
) -> tuple[float, ...] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{field} must be null or a JSON array of length {size}")
    if any(type(item) not in (int, float) for item in value):
        raise ValueError(f"{field} must contain only JSON numbers")
    return tuple(float(item) for item in value)


def tracking_sample_to_mapping(sample: TrackedRigidBodySample) -> dict[str, object]:
    """Convert one validated sample to JSON-native values."""

    return {
        "schema": sample.schema,
        "stream_id": sample.stream_id,
        "device_serial": sample.device_serial,
        "logical_role": sample.logical_role,
        "sequence": sample.sequence,
        "tracking_frame": sample.tracking_frame,
        "position_m": None if sample.position_m is None else list(sample.position_m),
        "quat_wxyz": None if sample.quat_wxyz is None else list(sample.quat_wxyz),
        "connected": sample.connected,
        "pose_valid": sample.pose_valid,
        "tracking_state": sample.tracking_state.value,
        "quality": sample.quality,
        "host_time_ns": sample.host_time_ns,
        "device_time_ns": sample.device_time_ns,
        "clock_domain": sample.clock_domain,
    }


def clutch_event_to_mapping(event: ClutchEvent) -> dict[str, object]:
    """Convert one validated clutch event to JSON-native values."""

    return {
        "schema": event.schema,
        "stream_id": event.stream_id,
        "device_serial": event.device_serial,
        "logical_role": event.logical_role,
        "input_id": event.input_id,
        "edge": event.edge.value,
        "sequence": event.sequence,
        "host_time_ns": event.host_time_ns,
        "clock_domain": event.clock_domain,
        "epoch_request": event.epoch_request,
    }


def encode_tracking_sample_json(sample: TrackedRigidBodySample) -> str:
    """Encode one canonical sample as a deterministic strict JSON object."""

    return json.dumps(
        tracking_sample_to_mapping(sample),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def encode_clutch_event_json(event: ClutchEvent) -> str:
    """Encode one canonical input edge as a deterministic strict JSON object."""

    return json.dumps(
        clutch_event_to_mapping(event),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def decode_tracking_sample_json(data: str | bytes) -> TrackedRigidBodySample:
    """Decode one line only when it exactly matches sample schema v1."""

    payload = _decode_mapping(data, fields=_SAMPLE_FIELDS)
    try:
        state = TrackingState(payload["tracking_state"])
    except (TypeError, ValueError) as exc:
        raise ValueError("tracking_state is not a supported value") from exc
    position = _optional_numeric_array(payload["position_m"], field="position_m", size=3)
    quaternion = _optional_numeric_array(payload["quat_wxyz"], field="quat_wxyz", size=4)
    return TrackedRigidBodySample(
        schema=payload["schema"],  # type: ignore[arg-type]
        stream_id=payload["stream_id"],  # type: ignore[arg-type]
        device_serial=payload["device_serial"],  # type: ignore[arg-type]
        logical_role=payload["logical_role"],  # type: ignore[arg-type]
        sequence=payload["sequence"],  # type: ignore[arg-type]
        tracking_frame=payload["tracking_frame"],  # type: ignore[arg-type]
        position_m=position,  # type: ignore[arg-type]
        quat_wxyz=quaternion,  # type: ignore[arg-type]
        connected=payload["connected"],  # type: ignore[arg-type]
        pose_valid=payload["pose_valid"],  # type: ignore[arg-type]
        tracking_state=state,
        quality=payload["quality"],  # type: ignore[arg-type]
        host_time_ns=payload["host_time_ns"],  # type: ignore[arg-type]
        device_time_ns=payload["device_time_ns"],  # type: ignore[arg-type]
        clock_domain=payload["clock_domain"],  # type: ignore[arg-type]
    )


def decode_clutch_event_json(data: str | bytes) -> ClutchEvent:
    """Decode one line only when it exactly matches clutch event schema v1."""

    payload = _decode_mapping(data, fields=_EVENT_FIELDS)
    try:
        edge = ClutchEdge(payload["edge"])
    except (TypeError, ValueError) as exc:
        raise ValueError("edge is not a supported value") from exc
    return ClutchEvent(
        schema=payload["schema"],  # type: ignore[arg-type]
        stream_id=payload["stream_id"],  # type: ignore[arg-type]
        device_serial=payload["device_serial"],  # type: ignore[arg-type]
        logical_role=payload["logical_role"],  # type: ignore[arg-type]
        input_id=payload["input_id"],  # type: ignore[arg-type]
        edge=edge,
        sequence=payload["sequence"],  # type: ignore[arg-type]
        host_time_ns=payload["host_time_ns"],  # type: ignore[arg-type]
        clock_domain=payload["clock_domain"],  # type: ignore[arg-type]
        epoch_request=payload["epoch_request"],  # type: ignore[arg-type]
    )


def _write_jsonl(path: Path, lines: Iterable[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = "".join(f"{line}\n" for line in lines)
    path.write_text(encoded, encoding="utf-8")


def write_tracking_samples_jsonl(
    path: str | Path,
    samples: Iterable[TrackedRigidBodySample],
) -> None:
    """Write a bounded sequence of canonical samples."""

    _write_jsonl(Path(path), (encode_tracking_sample_json(sample) for sample in samples))


def write_clutch_events_jsonl(
    path: str | Path,
    events: Iterable[ClutchEvent],
) -> None:
    """Write a bounded sequence of canonical input edges."""

    _write_jsonl(Path(path), (encode_clutch_event_json(event) for event in events))


def _read_jsonl(path: Path) -> list[str]:
    data = path.read_text(encoding="utf-8")
    if data and not data.endswith("\n"):
        raise ValueError(f"{path} has a truncated final JSONL record")
    lines = data.splitlines()
    if any(not line for line in lines):
        raise ValueError(f"{path} contains an empty JSONL record")
    return lines


def read_tracking_samples_jsonl(path: str | Path) -> tuple[TrackedRigidBodySample, ...]:
    """Read and validate every sample record before returning any."""

    return tuple(decode_tracking_sample_json(line) for line in _read_jsonl(Path(path)))


def read_clutch_events_jsonl(path: str | Path) -> tuple[ClutchEvent, ...]:
    """Read and validate every clutch event record before returning any."""

    return tuple(decode_clutch_event_json(line) for line in _read_jsonl(Path(path)))


__all__ = [
    "clutch_event_to_mapping",
    "decode_clutch_event_json",
    "decode_tracking_sample_json",
    "encode_clutch_event_json",
    "encode_tracking_sample_json",
    "read_clutch_events_jsonl",
    "read_tracking_samples_jsonl",
    "tracking_sample_to_mapping",
    "write_clutch_events_jsonl",
    "write_tracking_samples_jsonl",
]
