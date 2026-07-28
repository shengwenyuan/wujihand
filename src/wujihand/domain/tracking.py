"""Device-independent rigid-body tracking and clutch event contracts.

Schema v1 fixes position units to metres, quaternions to active scalar-first
``wxyz`` rotations, and host timestamps to a monotonic clock.  Invalid tracking
never carries a stale or partial pose: consumers must observe the explicit
tracking state instead.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
import re
from typing import Final, cast

from .pose import validate_host_time_ns, validate_unit_quaternion_wxyz


TRACKED_RIGID_BODY_SAMPLE_SCHEMA: Final = "wujihand.tracked_rigid_body_sample.v1"
CLUTCH_EVENT_SCHEMA: Final = "wujihand.clutch_event.v1"
TRACKING_POSITION_UNIT: Final = "m"
TRACKING_QUATERNION_ORDER: Final = "wxyz"
TRACKING_QUATERNION_CONVENTION: Final = "active"
HOST_MONOTONIC_CLOCK_DOMAIN: Final = "host_monotonic"

_TOKEN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/+-]{0,127}$")


class TrackingState(str, Enum):
    """Backend-neutral status of one tracked rigid body."""

    UNINITIALIZED = "uninitialized"
    CALIBRATING = "calibrating"
    RUNNING = "running"
    OUT_OF_RANGE = "out_of_range"
    ROTATION_ONLY = "rotation_only"
    LOST = "lost"


class ClutchEdge(str, Enum):
    """Physical input edge used by clutch or deadman handling."""

    PRESSED = "pressed"
    RELEASED = "released"


def _validate_token(value: object, *, field: str) -> str:
    if type(value) is not str or _TOKEN.fullmatch(value) is None:
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


def _validate_quality(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("quality must be a finite number in (0, 1]")
    quality = float(value)
    if not math.isfinite(quality) or not 0.0 < quality <= 1.0:
        raise ValueError("quality must be a finite number in (0, 1]")
    return quality


def _validate_clock_domain(value: object) -> str:
    if type(value) is not str or value != HOST_MONOTONIC_CLOCK_DOMAIN:
        raise ValueError(f"clock_domain must be {HOST_MONOTONIC_CLOCK_DOMAIN!r}")
    return value


@dataclass(frozen=True, slots=True, kw_only=True)
class TrackedRigidBodySample:
    """One immutable 6-DoF tracking observation or explicit invalid state.

    Only ``RUNNING`` samples are actionable full poses.  Every other state has
    ``pose_valid=False`` and carries neither position nor orientation, so a
    consumer cannot accidentally reuse a last-known pose as fresh input.
    ``LOST`` may remain physically connected when optical tracking is lost.
    """

    stream_id: str
    device_serial: str
    logical_role: str
    sequence: int
    tracking_frame: str
    position_m: tuple[float, float, float] | None
    quat_wxyz: tuple[float, float, float, float] | None
    connected: bool
    pose_valid: bool
    tracking_state: TrackingState
    quality: float | None
    host_time_ns: int
    device_time_ns: int | None
    clock_domain: str = HOST_MONOTONIC_CLOCK_DOMAIN
    schema: str = TRACKED_RIGID_BODY_SAMPLE_SCHEMA

    def __post_init__(self) -> None:
        if type(self.schema) is not str or self.schema != TRACKED_RIGID_BODY_SAMPLE_SCHEMA:
            raise ValueError(f"schema must be {TRACKED_RIGID_BODY_SAMPLE_SCHEMA!r}")
        object.__setattr__(self, "stream_id", _validate_token(self.stream_id, field="stream_id"))
        object.__setattr__(
            self,
            "device_serial",
            _validate_token(self.device_serial, field="device_serial"),
        )
        object.__setattr__(
            self,
            "logical_role",
            _validate_token(self.logical_role, field="logical_role"),
        )
        object.__setattr__(self, "sequence", _validate_sequence(self.sequence))
        object.__setattr__(
            self,
            "tracking_frame",
            _validate_token(self.tracking_frame, field="tracking_frame"),
        )
        if type(self.connected) is not bool or type(self.pose_valid) is not bool:
            raise ValueError("connected and pose_valid must be booleans")
        if type(self.tracking_state) is not TrackingState:
            raise ValueError("tracking_state must be a TrackingState")
        object.__setattr__(self, "host_time_ns", validate_host_time_ns(self.host_time_ns))
        object.__setattr__(
            self,
            "device_time_ns",
            _validate_optional_time_ns(
                self.device_time_ns,
                field="device_time_ns",
            ),
        )
        object.__setattr__(self, "clock_domain", _validate_clock_domain(self.clock_domain))

        if self.pose_valid:
            if self.tracking_state is not TrackingState.RUNNING:
                raise ValueError("only RUNNING may carry a valid pose")
            if not self.connected:
                raise ValueError("a valid pose requires a connected device")
            if self.position_m is None or self.quat_wxyz is None:
                raise ValueError("a valid pose requires position_m and quat_wxyz")
            position = _finite_vector(
                self.position_m,
                size=3,
                field="position_m",
            )
            quaternion_values = _finite_vector(
                self.quat_wxyz,
                size=4,
                field="quat_wxyz",
            )
            quaternion = validate_unit_quaternion_wxyz(quaternion_values)
            if self.quality is None:
                raise ValueError("a valid pose requires quality")
            quality = _validate_quality(self.quality)
            object.__setattr__(
                self,
                "position_m",
                cast(tuple[float, float, float], position),
            )
            object.__setattr__(
                self,
                "quat_wxyz",
                cast(
                    tuple[float, float, float, float],
                    tuple(float(value) for value in quaternion),
                ),
            )
            object.__setattr__(self, "quality", quality)
            return

        if self.tracking_state is TrackingState.RUNNING:
            raise ValueError("RUNNING requires pose_valid=True")
        if self.position_m is not None or self.quat_wxyz is not None:
            raise ValueError("an invalid pose must not carry position_m or quat_wxyz")
        if self.quality is not None:
            raise ValueError("an invalid pose must have quality=None")
        if not self.connected and self.tracking_state not in {
            TrackingState.UNINITIALIZED,
            TrackingState.LOST,
        }:
            raise ValueError("a disconnected device must be UNINITIALIZED or LOST")


@dataclass(frozen=True, slots=True, kw_only=True)
class ClutchEvent:
    """One immutable clutch/deadman input edge in host monotonic time."""

    stream_id: str
    device_serial: str
    logical_role: str
    input_id: str
    edge: ClutchEdge
    sequence: int
    host_time_ns: int
    epoch_request: bool
    clock_domain: str = HOST_MONOTONIC_CLOCK_DOMAIN
    schema: str = CLUTCH_EVENT_SCHEMA

    def __post_init__(self) -> None:
        if type(self.schema) is not str or self.schema != CLUTCH_EVENT_SCHEMA:
            raise ValueError(f"schema must be {CLUTCH_EVENT_SCHEMA!r}")
        for field in (
            "stream_id",
            "device_serial",
            "logical_role",
            "input_id",
        ):
            object.__setattr__(
                self,
                field,
                _validate_token(getattr(self, field), field=field),
            )
        if type(self.edge) is not ClutchEdge:
            raise ValueError("edge must be a ClutchEdge")
        object.__setattr__(self, "sequence", _validate_sequence(self.sequence))
        object.__setattr__(self, "host_time_ns", validate_host_time_ns(self.host_time_ns))
        object.__setattr__(self, "clock_domain", _validate_clock_domain(self.clock_domain))
        if type(self.epoch_request) is not bool:
            raise ValueError("epoch_request must be a boolean")
        if self.epoch_request and self.edge is not ClutchEdge.PRESSED:
            raise ValueError("only a PRESSED edge may request a new epoch")


__all__ = [
    "CLUTCH_EVENT_SCHEMA",
    "HOST_MONOTONIC_CLOCK_DOMAIN",
    "TRACKED_RIGID_BODY_SAMPLE_SCHEMA",
    "TRACKING_POSITION_UNIT",
    "TRACKING_QUATERNION_CONVENTION",
    "TRACKING_QUATERNION_ORDER",
    "ClutchEdge",
    "ClutchEvent",
    "TrackedRigidBodySample",
    "TrackingState",
]
