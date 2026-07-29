"""Application boundary for device-independent tracking input."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from wujihand.domain import ClutchEvent, TrackedRigidBodySample


def _bounded_text(value: object, *, field: str) -> str:
    if type(value) is not str:
        raise ValueError(f"{field} must be a string")
    if not 1 <= len(value) <= 128 or value != value.strip():
        raise ValueError(f"{field} must contain 1..128 trimmed characters")
    if any(not character.isprintable() for character in value):
        raise ValueError(f"{field} must contain printable characters")
    return value


@dataclass(frozen=True, slots=True, kw_only=True)
class TrackerInventoryItem:
    """Stable device identity without an ephemeral runtime device index."""

    serial: str
    device_class: str
    model: str
    manufacturer: str
    connected: bool

    def __post_init__(self) -> None:
        for field in ("serial", "device_class", "model", "manufacturer"):
            object.__setattr__(
                self,
                field,
                _bounded_text(getattr(self, field), field=field),
            )
        if type(self.connected) is not bool:
            raise ValueError("connected must be a boolean")


@dataclass(frozen=True, slots=True, kw_only=True)
class TrackingPoll:
    """One atomic pose sample and the clutch edges observed with it."""

    sample: TrackedRigidBodySample
    clutch_events: tuple[ClutchEvent, ...] = ()

    def __post_init__(self) -> None:
        if type(self.sample) is not TrackedRigidBodySample:
            raise ValueError("sample must be a TrackedRigidBodySample")
        try:
            events = tuple(self.clutch_events)
        except TypeError as exc:
            raise ValueError("clutch_events must be a sequence of ClutchEvent") from exc
        if any(type(event) is not ClutchEvent for event in events):
            raise ValueError("clutch_events must contain only ClutchEvent")
        for event in events:
            if (
                event.stream_id != self.sample.stream_id
                or event.device_serial != self.sample.device_serial
                or event.logical_role != self.sample.logical_role
                or event.producer_instance != self.sample.producer_instance
                or event.transport_epoch != self.sample.transport_epoch
                or event.tracking_setup_revision
                != self.sample.tracking_setup_revision
                or event.clock_domain != self.sample.clock_domain
            ):
                raise ValueError("clutch event identity and clock must match the tracking sample")
        object.__setattr__(self, "clutch_events", events)


@runtime_checkable
class TrackingInputPort(Protocol):
    """Non-blocking source of atomic normalized tracking input.

    Each poll must return a sample.  Device or optical loss is represented by
    an explicit ``LOST`` sample rather than an empty batch.  ``host_time_ns``
    lets deterministic tests provide the monotonic acquisition time; ``None``
    asks the adapter to read its host monotonic clock.
    """

    def inventory(self) -> tuple[TrackerInventoryItem, ...]:
        """Return stable identities for currently known tracking devices."""

        ...

    def start(self) -> TrackerInventoryItem:
        """Open the configured serial and return its resolved identity."""

        ...

    def poll(self, *, host_time_ns: int | None = None) -> TrackingPoll:
        """Acquire one sample and its associated clutch/deadman edges."""

        ...

    def close(self) -> None:
        """Release runtime resources; safe adapters make this idempotent."""

        ...


__all__ = ["TrackerInventoryItem", "TrackingInputPort", "TrackingPoll"]
