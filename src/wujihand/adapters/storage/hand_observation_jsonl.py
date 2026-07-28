"""Strict JSONL fixtures and bounded replay for canonical hand observations.

The persisted form contains only simulator- and SDK-independent domain values.
It is therefore suitable for sanitized regression fixtures.  Replay rebases the
host-comparable timestamps onto the caller's monotonic clock while preserving
the recorded source age and all device/calibration/transform provenance.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
import json
import math
from pathlib import Path
import time
from typing import Any, Final, cast

from wujihand.domain.hand_teleoperation import (
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    MediaPipeHandLandmark,
)
from wujihand.domain.pose import validate_host_time_ns


_OBSERVATION_FIELDS: Final = frozenset(
    {
        "schema",
        "side",
        "sequence",
        "source_id",
        "calibration_id",
        "transform_id",
        "source_time_ns",
        "receive_time_ns",
        "device_time_ns",
        "device_clock_domain",
        "clock_domain",
        "frame_id",
        "landmark_layout",
        "position_unit",
        "landmarks",
    }
)
_LANDMARK_FIELDS: Final = frozenset({"name", "position_m", "confidence"})


class HandObservationReplayExhausted(EOFError):
    """Raised when a bounded hand-observation replay has no record remaining."""


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number {value!r} is not allowed")


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON field {key!r}")
        result[key] = value
    return result


def _decode_object(data: str | bytes) -> dict[str, object]:
    if not isinstance(data, (str, bytes)) or not data:
        raise ValueError("canonical hand observation must be non-empty JSON")
    try:
        decoded: object = json.loads(
            data,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("canonical hand observation is not valid strict JSON") from exc
    if not isinstance(decoded, dict) or set(decoded) != _OBSERVATION_FIELDS:
        raise ValueError("canonical hand observation fields do not match schema")
    return cast(dict[str, object], decoded)


def _string(value: object, *, field: str) -> str:
    if type(value) is not str:
        raise ValueError(f"{field} must be a JSON string")
    return value


def _optional_string(value: object, *, field: str) -> str | None:
    if value is None:
        return None
    return _string(value, field=field)


def _integer(value: object, *, field: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{field} must be a JSON integer")
    return value


def _optional_integer(value: object, *, field: str) -> int | None:
    if value is None:
        return None
    return _integer(value, field=field)


def _number(value: object, *, field: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{field} must be a finite JSON number")
    number = float(cast(int | float, value))
    if not math.isfinite(number):
        raise ValueError(f"{field} must be a finite JSON number")
    return number


def _position(value: object, *, field: str) -> tuple[float, float, float] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field} must be null or a JSON array of length 3")
    result = tuple(
        _number(component, field=f"{field}[{index}]") for index, component in enumerate(value)
    )
    return cast(tuple[float, float, float], result)


def _decode_landmark(value: object, *, index: int) -> HandLandmark:
    if not isinstance(value, dict) or set(value) != _LANDMARK_FIELDS:
        raise ValueError(f"landmarks[{index}] fields do not match the landmark schema")
    payload = cast(dict[str, object], value)
    try:
        name = MediaPipeHandLandmark(_string(payload["name"], field=f"landmarks[{index}].name"))
    except ValueError as exc:
        raise ValueError(f"landmarks[{index}].name is not a canonical landmark") from exc
    return HandLandmark(
        name=name,
        position_m=_position(
            payload["position_m"],
            field=f"landmarks[{index}].position_m",
        ),
        confidence=_number(
            payload["confidence"],
            field=f"landmarks[{index}].confidence",
        ),
    )


def canonical_hand_observation_to_mapping(
    observation: CanonicalHandObservation,
) -> dict[str, object]:
    """Convert one validated canonical observation to deterministic JSON values."""

    if type(observation) is not CanonicalHandObservation:
        raise ValueError("observation must be a CanonicalHandObservation")
    return {
        "schema": observation.schema,
        "side": observation.side.value,
        "sequence": observation.sequence,
        "source_id": observation.source_id,
        "calibration_id": observation.calibration_id,
        "transform_id": observation.transform_id,
        "source_time_ns": observation.source_time_ns,
        "receive_time_ns": observation.receive_time_ns,
        "device_time_ns": observation.device_time_ns,
        "device_clock_domain": observation.device_clock_domain,
        "clock_domain": observation.clock_domain,
        "frame_id": observation.frame_id,
        "landmark_layout": observation.landmark_layout,
        "position_unit": observation.position_unit,
        "landmarks": [
            {
                "name": landmark.name.value,
                "position_m": (None if landmark.position_m is None else list(landmark.position_m)),
                "confidence": landmark.confidence,
            }
            for landmark in observation.landmarks
        ],
    }


def encode_canonical_hand_observation_json(
    observation: CanonicalHandObservation,
) -> str:
    """Encode one canonical observation as one strict deterministic JSON object."""

    return json.dumps(
        canonical_hand_observation_to_mapping(observation),
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def decode_canonical_hand_observation_json(
    data: str | bytes,
) -> CanonicalHandObservation:
    """Decode one observation only when all outer and landmark fields are exact."""

    payload = _decode_object(data)
    try:
        side = HandSide(_string(payload["side"], field="side"))
    except ValueError as exc:
        raise ValueError("side is not a supported anatomical side") from exc

    landmarks_value = payload["landmarks"]
    if not isinstance(landmarks_value, list):
        raise ValueError("landmarks must be a JSON array")
    landmarks = tuple(
        _decode_landmark(value, index=index) for index, value in enumerate(landmarks_value)
    )
    return CanonicalHandObservation(
        schema=_string(payload["schema"], field="schema"),
        side=side,
        sequence=_integer(payload["sequence"], field="sequence"),
        source_id=_string(payload["source_id"], field="source_id"),
        calibration_id=_string(payload["calibration_id"], field="calibration_id"),
        transform_id=_string(payload["transform_id"], field="transform_id"),
        source_time_ns=_optional_integer(
            payload["source_time_ns"],
            field="source_time_ns",
        ),
        receive_time_ns=_integer(
            payload["receive_time_ns"],
            field="receive_time_ns",
        ),
        device_time_ns=_optional_integer(
            payload["device_time_ns"],
            field="device_time_ns",
        ),
        device_clock_domain=_optional_string(
            payload["device_clock_domain"],
            field="device_clock_domain",
        ),
        clock_domain=_string(payload["clock_domain"], field="clock_domain"),
        frame_id=_string(payload["frame_id"], field="frame_id"),
        landmark_layout=_string(
            payload["landmark_layout"],
            field="landmark_layout",
        ),
        position_unit=_string(payload["position_unit"], field="position_unit"),
        landmarks=landmarks,
    )


def _read_jsonl(path: Path) -> tuple[str, ...]:
    data = path.read_text(encoding="utf-8")
    if data and not data.endswith("\n"):
        raise ValueError(f"{path} has a truncated final JSONL record")
    lines = tuple(data.splitlines())
    if any(not line for line in lines):
        raise ValueError(f"{path} contains an empty JSONL record")
    return lines


def read_canonical_hand_observations_jsonl(
    path: str | Path,
) -> tuple[CanonicalHandObservation, ...]:
    """Read and validate every bounded fixture record before returning any."""

    return tuple(decode_canonical_hand_observation_json(line) for line in _read_jsonl(Path(path)))


def write_canonical_hand_observations_jsonl(
    path: str | Path,
    observations: Iterable[CanonicalHandObservation],
) -> None:
    """Write a bounded sequence of sanitized canonical observations."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded = "".join(
        f"{encode_canonical_hand_observation_json(observation)}\n" for observation in observations
    )
    destination.write_text(encoded, encoding="utf-8")


class CanonicalHandObservationReplayAdapter:
    """Replay a finite canonical fixture through ``HandObservationInputPort``.

    ``source_time_ns`` is shifted by the same delta as ``receive_time_ns`` so
    its recorded source age remains unchanged.  Device timestamps are retained
    unchanged because their clock domain is device-local provenance.
    """

    def __init__(
        self,
        observations: Sequence[CanonicalHandObservation],
    ) -> None:
        records = tuple(observations)
        if any(type(record) is not CanonicalHandObservation for record in records):
            raise ValueError("observations must contain only CanonicalHandObservation values")
        self._observations = records
        self._started = False
        self._index = 0
        self._last_receive_time_ns = -1

    @classmethod
    def from_jsonl(
        cls,
        path: str | Path,
    ) -> CanonicalHandObservationReplayAdapter:
        """Load and fully validate a finite JSONL fixture before replay."""

        return cls(read_canonical_hand_observations_jsonl(path))

    def start(self) -> None:
        """Start a replay epoch at record zero."""

        if self._started:
            raise RuntimeError("canonical hand observation replay is already started")
        self._started = True
        self._reset_index()

    def reset(self) -> None:
        """Reset an active replay to record zero and begin a fresh time epoch."""

        if not self._started:
            raise RuntimeError("start() must succeed before reset()")
        self._reset_index()

    def poll(
        self,
        *,
        receive_time_ns: int | None = None,
    ) -> CanonicalHandObservation:
        """Return the next fixture record rebased to a fresh host receive time."""

        if not self._started:
            raise RuntimeError("start() must succeed before poll()")
        if self._index >= len(self._observations):
            raise HandObservationReplayExhausted(
                "canonical hand observation replay reached end of fixture"
            )

        timestamp = (
            time.monotonic_ns()
            if receive_time_ns is None
            else validate_host_time_ns(receive_time_ns)
        )
        if timestamp <= self._last_receive_time_ns:
            raise ValueError("receive_time_ns must increase strictly between replay polls")

        recorded = self._observations[self._index]
        source_time_ns: int | None
        if recorded.source_time_ns is None:
            source_time_ns = None
        else:
            recorded_source_age_ns = recorded.receive_time_ns - recorded.source_time_ns
            if recorded_source_age_ns > timestamp:
                raise ValueError("receive_time_ns is too small to preserve recorded source age")
            source_time_ns = timestamp - recorded_source_age_ns

        replayed = CanonicalHandObservation(
            schema=recorded.schema,
            side=recorded.side,
            sequence=recorded.sequence,
            source_id=recorded.source_id,
            calibration_id=recorded.calibration_id,
            transform_id=recorded.transform_id,
            source_time_ns=source_time_ns,
            receive_time_ns=timestamp,
            device_time_ns=recorded.device_time_ns,
            device_clock_domain=recorded.device_clock_domain,
            clock_domain=recorded.clock_domain,
            frame_id=recorded.frame_id,
            landmark_layout=recorded.landmark_layout,
            position_unit=recorded.position_unit,
            landmarks=recorded.landmarks,
        )
        self._index += 1
        self._last_receive_time_ns = timestamp
        return replayed

    def close(self) -> None:
        """Close replay idempotently and discard the current cursor."""

        self._started = False
        self._reset_index()

    def __enter__(self) -> CanonicalHandObservationReplayAdapter:
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _reset_index(self) -> None:
        self._index = 0
        self._last_receive_time_ns = -1


__all__ = [
    "CanonicalHandObservationReplayAdapter",
    "HandObservationReplayExhausted",
    "canonical_hand_observation_to_mapping",
    "decode_canonical_hand_observation_json",
    "encode_canonical_hand_observation_json",
    "read_canonical_hand_observations_jsonl",
    "write_canonical_hand_observations_jsonl",
]
