"""Side-neutral composition for one or two Glove-to-Hand 2 controllers."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from wujihand.domain import HandSide
from wujihand.ports import NoHandObservationAvailable

from .glove_hand2 import (
    GloveHand2SimulationController,
    Hand2SimulationStep,
)


@dataclass(frozen=True, slots=True)
class SideHand2SimulationStep:
    """One side-labelled q20 decision in a shared simulation tick."""

    side: HandSide
    step: Hand2SimulationStep


class GloveHand2ControllerSet:
    """Advance configured hands independently with deterministic lifetimes."""

    def __init__(
        self,
        controllers: Mapping[
            HandSide,
            GloveHand2SimulationController,
        ],
    ) -> None:
        if not isinstance(controllers, Mapping):
            raise TypeError("controllers must be a mapping")
        copied = dict(controllers)
        if set(copied) - set(HandSide):
            raise ValueError("controller keys must be HandSide values")
        if len(copied) > 2:
            raise ValueError("controllers may contain at most two hands")
        for side, controller in copied.items():
            if not isinstance(controller, GloveHand2SimulationController):
                raise TypeError(
                    "controllers must be GloveHand2SimulationController values"
                )
            if controller.side is not side:
                raise ValueError("controller key and configured side must match")
        self._controllers = tuple(
            (side, copied[side])
            for side in (HandSide.LEFT, HandSide.RIGHT)
            if side in copied
        )
        self._started: list[
            tuple[HandSide, GloveHand2SimulationController]
        ] = []
        self._active = False
        self._closed = False

    @property
    def sides(self) -> tuple[HandSide, ...]:
        return tuple(side for side, _ in self._controllers)

    def start(self, *, now_ns: int) -> None:
        """Start left then right and roll back in reverse order on failure."""

        if self._closed:
            raise RuntimeError("Glove controller set is closed")
        if self._active:
            raise RuntimeError("Glove controller set is already started")
        try:
            for side, controller in self._controllers:
                controller.start(now_ns=now_ns)
                self._started.append((side, controller))
        except Exception:
            self._close_started()
            raise
        self._active = True

    def step(
        self,
        *,
        now_ns: int,
    ) -> tuple[SideHand2SimulationStep, ...]:
        """Poll each side once; a missing frame advances only that side."""

        if not self._active or self._closed:
            raise RuntimeError("start() must succeed before step()")
        decisions: list[SideHand2SimulationStep] = []
        for side, controller in self._started:
            try:
                step = controller.poll(now_ns=now_ns)
            except NoHandObservationAvailable:
                step = controller.advance_without_observation(
                    now_ns=now_ns
                )
            except (RuntimeError, ValueError) as exc:
                step = controller.reject_observation(
                    now_ns=now_ns,
                    reason=f"input_rejected:{type(exc).__name__}",
                )
            decisions.append(
                SideHand2SimulationStep(side=side, step=step)
            )
        return tuple(decisions)

    def invalidate_input_epoch(
        self,
        side: HandSide,
    ) -> None:
        """Schedule a side-local hold for a changed ROS producer epoch."""

        if not self._active or self._closed:
            raise RuntimeError("start() must succeed before invalidation")
        for configured_side, controller in self._started:
            if configured_side is side:
                controller.invalidate_input_epoch()
                return
        raise KeyError(side)

    def close(self) -> None:
        """Close right then left while preserving the first cleanup error."""

        if self._closed:
            return
        self._closed = True
        self._active = False
        first_error = self._close_started()
        if first_error is not None:
            raise first_error

    def _close_started(self) -> Exception | None:
        first_error: Exception | None = None
        while self._started:
            _, controller = self._started.pop()
            try:
                controller.close()
            except Exception as exc:  # pragma: no cover - defensive cleanup
                if first_error is None:
                    first_error = exc
        return first_error


__all__ = [
    "GloveHand2ControllerSet",
    "SideHand2SimulationStep",
]
