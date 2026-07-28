"""SDK- and simulator-independent hand observation and intent contracts.

The canonical observation fixes the input geometry to the 21 named MediaPipe
hand landmarks in metres.  A Hand 2 intent remains an application-level
retargeting result: it is not evidence that a command was supervised, sent, or
executed.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
import re
from typing import Final, cast

from .hand2 import HAND2_LAYOUT_IDS, hand2_layout
from .pose import validate_calibration_id, validate_frame_id, validate_host_time_ns
from .tracking import HOST_MONOTONIC_CLOCK_DOMAIN


CANONICAL_HAND_OBSERVATION_SCHEMA: Final = "wujihand.canonical_hand_observation.v1"
HAND_INTENT_SCHEMA: Final = "wujihand.hand_intent.v1"
MEDIAPIPE_HAND_LANDMARK_LAYOUT: Final = "mediapipe.hand_landmarks.v1"
HAND_POSITION_UNIT: Final = "m"
HAND_JOINT_POSITION_UNIT: Final = "rad"

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/+-]{0,127}$")


class HandSide(str, Enum):
    """An explicit anatomical side."""

    LEFT = "left"
    RIGHT = "right"


class MediaPipeHandLandmark(str, Enum):
    """The canonical MediaPipe 21-landmark semantic names."""

    WRIST = "wrist"
    THUMB_CMC = "thumb_cmc"
    THUMB_MCP = "thumb_mcp"
    THUMB_IP = "thumb_ip"
    THUMB_TIP = "thumb_tip"
    INDEX_FINGER_MCP = "index_finger_mcp"
    INDEX_FINGER_PIP = "index_finger_pip"
    INDEX_FINGER_DIP = "index_finger_dip"
    INDEX_FINGER_TIP = "index_finger_tip"
    MIDDLE_FINGER_MCP = "middle_finger_mcp"
    MIDDLE_FINGER_PIP = "middle_finger_pip"
    MIDDLE_FINGER_DIP = "middle_finger_dip"
    MIDDLE_FINGER_TIP = "middle_finger_tip"
    RING_FINGER_MCP = "ring_finger_mcp"
    RING_FINGER_PIP = "ring_finger_pip"
    RING_FINGER_DIP = "ring_finger_dip"
    RING_FINGER_TIP = "ring_finger_tip"
    PINKY_MCP = "pinky_mcp"
    PINKY_PIP = "pinky_pip"
    PINKY_DIP = "pinky_dip"
    PINKY_TIP = "pinky_tip"


MEDIAPIPE_HAND_LANDMARK_NAMES: Final = tuple(MediaPipeHandLandmark)


class RetargetStatus(str, Enum):
    """Status of a retargeting result that is safe to represent as an intent.

    A failed solve must not produce a ``HandIntent``.
    """

    SUCCESS = "success"
    DEGRADED = "degraded"


def _validate_identifier(value: object, *, field: str) -> str:
    if type(value) is not str or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(f"{field} must be a bounded transport-safe identifier")
    return value


def _validate_sequence(value: object) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("sequence must be a non-negative integer")
    return value


def _validate_optional_time_ns(value: object, *, field: str) -> int | None:
    if value is None:
        return None
    if type(value) is not int or value < 0:
        raise ValueError(f"{field} must be a non-negative integer or None")
    return value


def _validate_confidence(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field} must be a finite number in [0, 1]")
    confidence = float(value)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError(f"{field} must be a finite number in [0, 1]")
    return confidence


def _finite_vector(
    value: object,
    *,
    size: int,
    field: str,
) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Iterable):
        raise ValueError(f"{field} must contain exactly {size} finite numbers")
    items: tuple[object, ...] = tuple(value)
    if len(items) != size:
        raise ValueError(f"{field} must contain exactly {size} finite numbers")

    result: list[float] = []
    for item in items:
        if isinstance(item, bool) or not isinstance(item, Real):
            raise ValueError(f"{field} must contain exactly {size} finite numbers")
        try:
            number = float(item)
        except (OverflowError, ValueError) as exc:
            raise ValueError(f"{field} must contain exactly {size} finite numbers") from exc
        if not math.isfinite(number):
            raise ValueError(f"{field} must contain exactly {size} finite numbers")
        result.append(number)
    return tuple(result)


@dataclass(frozen=True, slots=True, kw_only=True)
class HandLandmark:
    """One named canonical landmark, or an explicit missing observation."""

    name: MediaPipeHandLandmark
    position_m: tuple[float, float, float] | None
    confidence: float

    def __post_init__(self) -> None:
        if type(self.name) is not MediaPipeHandLandmark:
            raise ValueError("name must be a MediaPipeHandLandmark")
        confidence = _validate_confidence(self.confidence, field="confidence")
        object.__setattr__(self, "confidence", confidence)

        if self.position_m is None:
            if confidence != 0.0:
                raise ValueError("a missing landmark must have confidence=0")
            return

        position = _finite_vector(self.position_m, size=3, field="position_m")
        object.__setattr__(
            self,
            "position_m",
            cast(tuple[float, float, float], position),
        )


@dataclass(frozen=True, slots=True, kw_only=True)
class CanonicalHandObservation:
    """One normalized, immutable 21-landmark hand observation.

    ``source_time_ns`` and ``receive_time_ns`` share ``clock_domain`` and are
    therefore comparable.  ``source_time_ns`` remains ``None`` when the input
    adapter cannot obtain a trustworthy acquisition timestamp.
    """

    side: HandSide
    sequence: int
    source_id: str
    calibration_id: str
    transform_id: str
    source_time_ns: int | None
    receive_time_ns: int
    device_time_ns: int | None
    frame_id: str
    landmarks: tuple[HandLandmark, ...]
    device_clock_domain: str | None = None
    clock_domain: str = HOST_MONOTONIC_CLOCK_DOMAIN
    landmark_layout: str = MEDIAPIPE_HAND_LANDMARK_LAYOUT
    position_unit: str = HAND_POSITION_UNIT
    schema: str = CANONICAL_HAND_OBSERVATION_SCHEMA

    def __post_init__(self) -> None:
        if type(self.schema) is not str or self.schema != CANONICAL_HAND_OBSERVATION_SCHEMA:
            raise ValueError(f"schema must be {CANONICAL_HAND_OBSERVATION_SCHEMA!r}")
        if type(self.side) is not HandSide:
            raise ValueError("side must be a HandSide")
        object.__setattr__(self, "sequence", _validate_sequence(self.sequence))
        object.__setattr__(
            self,
            "source_id",
            _validate_identifier(self.source_id, field="source_id"),
        )
        object.__setattr__(
            self,
            "calibration_id",
            validate_calibration_id(self.calibration_id),
        )
        object.__setattr__(
            self,
            "transform_id",
            _validate_identifier(self.transform_id, field="transform_id"),
        )

        source_time_ns = _validate_optional_time_ns(
            self.source_time_ns,
            field="source_time_ns",
        )
        receive_time_ns = validate_host_time_ns(self.receive_time_ns)
        if source_time_ns is not None and source_time_ns > receive_time_ns:
            raise ValueError("source_time_ns must not be later than receive_time_ns")
        object.__setattr__(self, "source_time_ns", source_time_ns)
        object.__setattr__(self, "receive_time_ns", receive_time_ns)

        device_time_ns = _validate_optional_time_ns(
            self.device_time_ns,
            field="device_time_ns",
        )
        object.__setattr__(self, "device_time_ns", device_time_ns)
        if device_time_ns is None:
            if self.device_clock_domain is not None:
                raise ValueError("device_clock_domain requires device_time_ns")
        else:
            object.__setattr__(
                self,
                "device_clock_domain",
                _validate_identifier(
                    self.device_clock_domain,
                    field="device_clock_domain",
                ),
            )

        object.__setattr__(self, "frame_id", validate_frame_id(self.frame_id))
        if (
            type(self.clock_domain) is not str
            or self.clock_domain != HOST_MONOTONIC_CLOCK_DOMAIN
        ):
            raise ValueError(f"clock_domain must be {HOST_MONOTONIC_CLOCK_DOMAIN!r}")
        if (
            type(self.landmark_layout) is not str
            or self.landmark_layout != MEDIAPIPE_HAND_LANDMARK_LAYOUT
        ):
            raise ValueError(f"landmark_layout must be {MEDIAPIPE_HAND_LANDMARK_LAYOUT!r}")
        if type(self.position_unit) is not str or self.position_unit != HAND_POSITION_UNIT:
            raise ValueError(f"position_unit must be {HAND_POSITION_UNIT!r}")

        try:
            landmarks = tuple(self.landmarks)
        except TypeError as exc:
            raise ValueError("landmarks must contain the 21 canonical landmarks") from exc
        if any(type(landmark) is not HandLandmark for landmark in landmarks):
            raise ValueError("landmarks must contain only HandLandmark values")
        names = tuple(landmark.name for landmark in landmarks)
        if names != MEDIAPIPE_HAND_LANDMARK_NAMES:
            raise ValueError("landmarks must use the canonical MediaPipe 21-name order")
        object.__setattr__(self, "landmarks", landmarks)


@dataclass(frozen=True, slots=True, kw_only=True)
class HandIntent:
    """One Hand 2 q20 retargeting intent with complete input provenance."""

    side: HandSide
    sequence: int
    source_observation: CanonicalHandObservation
    q20_rad: tuple[float, ...]
    layout_id: str
    produced_time_ns: int
    retarget_status: RetargetStatus
    retarget_confidence: float
    retarget_model_id: str
    retarget_config_id: str
    clock_domain: str = HOST_MONOTONIC_CLOCK_DOMAIN
    joint_position_unit: str = HAND_JOINT_POSITION_UNIT
    schema: str = HAND_INTENT_SCHEMA

    def __post_init__(self) -> None:
        if type(self.schema) is not str or self.schema != HAND_INTENT_SCHEMA:
            raise ValueError(f"schema must be {HAND_INTENT_SCHEMA!r}")
        if type(self.side) is not HandSide:
            raise ValueError("side must be a HandSide")
        object.__setattr__(self, "sequence", _validate_sequence(self.sequence))
        if type(self.source_observation) is not CanonicalHandObservation:
            raise ValueError("source_observation must be a CanonicalHandObservation")
        if self.source_observation.side is not self.side:
            raise ValueError("intent side must match source observation side")

        layout_id = _validate_identifier(self.layout_id, field="layout_id")
        expected_layout_id = HAND2_LAYOUT_IDS[self.side.value]
        if layout_id != expected_layout_id:
            raise ValueError(
                f"layout_id must be {expected_layout_id!r} for side {self.side.value!r}"
            )
        object.__setattr__(self, "layout_id", layout_id)
        layout = hand2_layout(self.side.value)
        q20_rad = _finite_vector(self.q20_rad, size=layout.size, field="q20_rad")
        layout.validate_vector(q20_rad)
        object.__setattr__(self, "q20_rad", q20_rad)

        produced_time_ns = validate_host_time_ns(self.produced_time_ns)
        if produced_time_ns < self.source_observation.receive_time_ns:
            raise ValueError("produced_time_ns must not precede source receive_time_ns")
        object.__setattr__(self, "produced_time_ns", produced_time_ns)
        if (
            type(self.clock_domain) is not str
            or self.clock_domain != HOST_MONOTONIC_CLOCK_DOMAIN
        ):
            raise ValueError(f"clock_domain must be {HOST_MONOTONIC_CLOCK_DOMAIN!r}")
        clock_domain = self.clock_domain
        if clock_domain != self.source_observation.clock_domain:
            raise ValueError("intent clock_domain must match source observation clock_domain")
        object.__setattr__(self, "clock_domain", clock_domain)

        if type(self.retarget_status) is not RetargetStatus:
            raise ValueError("retarget_status must be a RetargetStatus")
        object.__setattr__(
            self,
            "retarget_confidence",
            _validate_confidence(
                self.retarget_confidence,
                field="retarget_confidence",
            ),
        )
        object.__setattr__(
            self,
            "retarget_model_id",
            _validate_identifier(
                self.retarget_model_id,
                field="retarget_model_id",
            ),
        )
        object.__setattr__(
            self,
            "retarget_config_id",
            _validate_identifier(
                self.retarget_config_id,
                field="retarget_config_id",
            ),
        )
        if (
            type(self.joint_position_unit) is not str
            or self.joint_position_unit != HAND_JOINT_POSITION_UNIT
        ):
            raise ValueError(f"joint_position_unit must be {HAND_JOINT_POSITION_UNIT!r}")

    @property
    def source_age_ns(self) -> int:
        """Age of the canonical observation when this intent was produced."""

        source_time_ns = self.source_observation.source_time_ns
        reference_time_ns = (
            self.source_observation.receive_time_ns
            if source_time_ns is None
            else source_time_ns
        )
        return self.produced_time_ns - reference_time_ns


__all__ = [
    "CANONICAL_HAND_OBSERVATION_SCHEMA",
    "HAND_INTENT_SCHEMA",
    "HAND_JOINT_POSITION_UNIT",
    "HAND_POSITION_UNIT",
    "MEDIAPIPE_HAND_LANDMARK_LAYOUT",
    "MEDIAPIPE_HAND_LANDMARK_NAMES",
    "CanonicalHandObservation",
    "HandIntent",
    "HandLandmark",
    "HandSide",
    "MediaPipeHandLandmark",
    "RetargetStatus",
]
