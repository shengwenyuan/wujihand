"""ROS envelope conversion for canonical 21-landmark hand observations."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import re
from typing import Final, Protocol, cast

from wujihand.domain import (
    CanonicalHandObservation,
    HandLandmark,
    HandSide,
    MediaPipeHandLandmark,
)

from ._message import new_message


HAND_OBSERVATION_ENVELOPE_SCHEMA: Final = (
    "wujihand.ros_hand_observation_envelope.v1"
)
_TOKEN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/+-]{0,127}$")


@dataclass(frozen=True, slots=True)
class HandObservationTransportEnvelope:
    producer_instance: str
    transport_epoch: int
    observation: CanonicalHandObservation

    def __post_init__(self) -> None:
        if _TOKEN.fullmatch(self.producer_instance) is None:
            raise ValueError("producer_instance must be transport-safe")
        if (
            type(self.transport_epoch) is not int
            or self.transport_epoch < 0
        ):
            raise ValueError("transport_epoch must be non-negative")


class HandObservationMessage(Protocol):
    schema: str
    producer_instance: str
    transport_epoch: int
    side: str
    sequence: int
    source_id: str
    calibration_id: str
    transform_id: str
    has_source_time: bool
    source_time_ns: int
    receive_time_ns: int
    has_device_time: bool
    device_time_ns: int
    device_clock_domain: str
    frame_id: str
    landmark_names: Sequence[str]
    landmark_valid: Sequence[bool]
    landmark_positions_m: Sequence[float]
    landmark_confidence: Sequence[float]
    clock_domain: str
    landmark_layout: str
    position_unit: str
    observation_schema: str


def hand_envelope_to_message(
    envelope: HandObservationTransportEnvelope,
    *,
    factory: Callable[[], HandObservationMessage] | None = None,
) -> HandObservationMessage:
    observation = envelope.observation
    message = new_message(factory, class_name="HandObservationEnvelope")
    message.schema = HAND_OBSERVATION_ENVELOPE_SCHEMA
    message.producer_instance = envelope.producer_instance
    message.transport_epoch = envelope.transport_epoch
    message.side = observation.side.value
    message.sequence = observation.sequence
    message.source_id = observation.source_id
    message.calibration_id = observation.calibration_id
    message.transform_id = observation.transform_id
    message.has_source_time = observation.source_time_ns is not None
    message.source_time_ns = (
        0 if observation.source_time_ns is None else observation.source_time_ns
    )
    message.receive_time_ns = observation.receive_time_ns
    message.has_device_time = observation.device_time_ns is not None
    message.device_time_ns = (
        0 if observation.device_time_ns is None else observation.device_time_ns
    )
    message.device_clock_domain = observation.device_clock_domain or ""
    message.frame_id = observation.frame_id
    message.landmark_names = tuple(
        landmark.name.value for landmark in observation.landmarks
    )
    message.landmark_valid = tuple(
        landmark.position_m is not None for landmark in observation.landmarks
    )
    message.landmark_positions_m = tuple(
        coordinate
        for landmark in observation.landmarks
        for coordinate in (
            (0.0, 0.0, 0.0)
            if landmark.position_m is None
            else landmark.position_m
        )
    )
    message.landmark_confidence = tuple(
        landmark.confidence for landmark in observation.landmarks
    )
    message.clock_domain = observation.clock_domain
    message.landmark_layout = observation.landmark_layout
    message.position_unit = observation.position_unit
    message.observation_schema = observation.schema
    return message


def hand_envelope_from_message(
    message: HandObservationMessage,
) -> HandObservationTransportEnvelope:
    if message.schema != HAND_OBSERVATION_ENVELOPE_SCHEMA:
        raise ValueError(
            f"schema must be {HAND_OBSERVATION_ENVELOPE_SCHEMA!r}"
        )
    if not message.has_source_time and message.source_time_ns != 0:
        raise ValueError("missing source time must use a zero wire sentinel")
    if not message.has_device_time and message.device_time_ns != 0:
        raise ValueError("missing device time must use a zero wire sentinel")
    if not message.has_device_time and message.device_clock_domain:
        raise ValueError("device clock domain requires device time")
    names = tuple(message.landmark_names)
    valid = tuple(message.landmark_valid)
    positions = tuple(float(value) for value in message.landmark_positions_m)
    confidence = tuple(float(value) for value in message.landmark_confidence)
    if not (
        len(names) == len(valid) == len(confidence) == 21
        and len(positions) == 63
    ):
        raise ValueError("hand observation message has invalid landmark shape")
    landmarks: list[HandLandmark] = []
    for index, name in enumerate(names):
        xyz = positions[index * 3 : index * 3 + 3]
        if not valid[index] and any(value != 0.0 for value in xyz):
            raise ValueError(
                "missing landmark must use zero position wire sentinels"
            )
        landmarks.append(
            HandLandmark(
                name=MediaPipeHandLandmark(name),
                position_m=(
                    cast(tuple[float, float, float], xyz)
                    if valid[index]
                    else None
                ),
                confidence=confidence[index],
            )
        )
    observation = CanonicalHandObservation(
        schema=message.observation_schema,
        side=HandSide(message.side),
        sequence=message.sequence,
        source_id=message.source_id,
        calibration_id=message.calibration_id,
        transform_id=message.transform_id,
        source_time_ns=(
            message.source_time_ns if message.has_source_time else None
        ),
        receive_time_ns=message.receive_time_ns,
        device_time_ns=(
            message.device_time_ns if message.has_device_time else None
        ),
        device_clock_domain=(
            message.device_clock_domain if message.has_device_time else None
        ),
        frame_id=message.frame_id,
        landmarks=tuple(landmarks),
        clock_domain=message.clock_domain,
        landmark_layout=message.landmark_layout,
        position_unit=message.position_unit,
    )
    return HandObservationTransportEnvelope(
        producer_instance=message.producer_instance,
        transport_epoch=message.transport_epoch,
        observation=observation,
    )
