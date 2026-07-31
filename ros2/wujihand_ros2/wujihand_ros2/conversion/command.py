"""ROS observation conversion for supervised route decisions."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
from typing import Final, Protocol

from wujihand.application.supervision import SafetyDecision, SafetyState

from ._message import new_message


ROUTE_COMMAND_SCHEMA: Final = "wujihand.ros_route_command.v1"
SAFETY_EVENT_SCHEMA: Final = "wujihand.ros_safety_event.v1"


class RouteCommandMessage(Protocol):
    schema: str
    instance_id: str
    group_id: str
    layout_id: str
    positions: Sequence[float]
    produced_time_ns: int
    safety_state: str
    safety_reason: str
    position_clamped: bool
    rate_limited: bool
    clock_domain: str


class SafetyEventMessage(Protocol):
    schema: str
    instance_id: str
    group_id: str
    safety_state: str
    reason: str
    position_clamped: bool
    rate_limited: bool
    host_time_ns: int
    clock_domain: str


@dataclass(frozen=True, slots=True)
class RouteCommandObservation:
    instance_id: str
    group_id: str
    layout_id: str
    positions: tuple[float, ...]
    produced_time_ns: int
    safety_state: SafetyState
    safety_reason: str
    position_clamped: bool
    rate_limited: bool

    def __post_init__(self) -> None:
        if not 1 <= len(self.positions) <= 27:
            raise ValueError("positions must contain 1..27 values")
        if not all(math.isfinite(value) for value in self.positions):
            raise ValueError("positions must be finite")
        if type(self.produced_time_ns) is not int or self.produced_time_ns < 0:
            raise ValueError("produced_time_ns must be non-negative")


@dataclass(frozen=True, slots=True)
class SafetyEventObservation:
    instance_id: str
    group_id: str
    state: SafetyState
    reason: str
    position_clamped: bool
    rate_limited: bool
    host_time_ns: int


def route_command_from_decision(
    *,
    instance_id: str,
    group_id: str,
    layout_id: str,
    decision: SafetyDecision,
    produced_time_ns: int,
) -> RouteCommandObservation:
    return RouteCommandObservation(
        instance_id=instance_id,
        group_id=group_id,
        layout_id=layout_id,
        positions=tuple(float(value) for value in decision.command),
        produced_time_ns=produced_time_ns,
        safety_state=decision.state,
        safety_reason=decision.reason,
        position_clamped=decision.position_clamped,
        rate_limited=decision.rate_limited,
    )


def route_command_to_message(
    observation: RouteCommandObservation,
    *,
    factory: Callable[[], RouteCommandMessage] | None = None,
) -> RouteCommandMessage:
    message = new_message(factory, class_name="RouteCommand")
    message.schema = ROUTE_COMMAND_SCHEMA
    message.instance_id = observation.instance_id
    message.group_id = observation.group_id
    message.layout_id = observation.layout_id
    message.positions = observation.positions
    message.produced_time_ns = observation.produced_time_ns
    message.safety_state = observation.safety_state.value
    message.safety_reason = observation.safety_reason
    message.position_clamped = observation.position_clamped
    message.rate_limited = observation.rate_limited
    message.clock_domain = "host_monotonic"
    return message


def route_command_from_message(
    message: RouteCommandMessage,
) -> RouteCommandObservation:
    if message.schema != ROUTE_COMMAND_SCHEMA:
        raise ValueError(f"schema must be {ROUTE_COMMAND_SCHEMA!r}")
    if message.clock_domain != "host_monotonic":
        raise ValueError("route command clock must be host_monotonic")
    return RouteCommandObservation(
        instance_id=message.instance_id,
        group_id=message.group_id,
        layout_id=message.layout_id,
        positions=tuple(float(value) for value in message.positions),
        produced_time_ns=message.produced_time_ns,
        safety_state=SafetyState(message.safety_state),
        safety_reason=message.safety_reason,
        position_clamped=message.position_clamped,
        rate_limited=message.rate_limited,
    )


def safety_event_to_message(
    event: SafetyEventObservation,
    *,
    factory: Callable[[], SafetyEventMessage] | None = None,
) -> SafetyEventMessage:
    message = new_message(factory, class_name="SafetyEvent")
    message.schema = SAFETY_EVENT_SCHEMA
    message.instance_id = event.instance_id
    message.group_id = event.group_id
    message.safety_state = event.state.value
    message.reason = event.reason
    message.position_clamped = event.position_clamped
    message.rate_limited = event.rate_limited
    message.host_time_ns = event.host_time_ns
    message.clock_domain = "host_monotonic"
    return message


def safety_event_from_message(
    message: SafetyEventMessage,
) -> SafetyEventObservation:
    if message.schema != SAFETY_EVENT_SCHEMA:
        raise ValueError(f"schema must be {SAFETY_EVENT_SCHEMA!r}")
    if message.clock_domain != "host_monotonic":
        raise ValueError("safety event clock must be host_monotonic")
    return SafetyEventObservation(
        instance_id=message.instance_id,
        group_id=message.group_id,
        state=SafetyState(message.safety_state),
        reason=message.reason,
        position_clamped=message.position_clamped,
        rate_limited=message.rate_limited,
        host_time_ns=message.host_time_ns,
    )


__all__ = [
    "ROUTE_COMMAND_SCHEMA",
    "SAFETY_EVENT_SCHEMA",
    "RouteCommandObservation",
    "SafetyEventObservation",
    "route_command_from_decision",
    "route_command_from_message",
    "route_command_to_message",
    "safety_event_from_message",
    "safety_event_to_message",
]
