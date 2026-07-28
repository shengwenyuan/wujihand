"""Application boundaries for normalized hand input and retargeting."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from wujihand.domain.hand_teleoperation import CanonicalHandObservation, HandIntent


@runtime_checkable
class HandObservationInputPort(Protocol):
    """Source of canonical hand observations from one configured input."""

    def start(self) -> None:
        """Open the input source and complete adapter-specific setup."""

        ...

    def poll(self, *, receive_time_ns: int | None = None) -> CanonicalHandObservation:
        """Acquire one observation.

        ``receive_time_ns`` lets tests and orchestrators provide the receiving
        host's monotonic timestamp.  ``None`` asks the adapter to read it.
        """

        ...

    def close(self) -> None:
        """Release runtime resources; adapters should make this idempotent."""

        ...


@runtime_checkable
class RetargetPort(Protocol):
    """Map canonical landmarks to a side-matched Hand 2 q20 intent."""

    def retarget(
        self,
        observation: CanonicalHandObservation,
        *,
        sequence: int,
        produced_time_ns: int | None = None,
    ) -> HandIntent:
        """Return one successful or degraded intent.

        A failed solve must not synthesize an executable intent.
        """

        ...

    def reset(self) -> None:
        """Clear adapter warm-start and filtering state."""

        ...


__all__ = ["HandObservationInputPort", "RetargetPort"]
