"""Port-only composition for one Glove-driven simulated Hand 2.

The use case knows canonical domain contracts and ports only.  Device SDK
objects and simulator articulation objects remain in their respective
adapters/composition root.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import cast

import numpy as np

from wujihand.application.supervision import JointCommandSupervisor, SafetyDecision
from wujihand.domain import CanonicalHandObservation, HandIntent, HandSide, hand2_layout
from wujihand.domain.hand2 import HAND2_LAYOUT_IDS
from wujihand.domain.joints import FloatArray
from wujihand.ports import HandObservationInputPort, RetargetPort


@dataclass(frozen=True, slots=True)
class Hand2SimulationStep:
    """One supervised command and whether a new observation was accepted."""

    intent: HandIntent | None
    decision: SafetyDecision
    rejection_reason: str | None = None


class GloveHand2SimulationController:
    """Compose one canonical glove input, retargeter, and q20 supervisor."""

    def __init__(
        self,
        side: HandSide,
        observation_input: HandObservationInputPort,
        retargeter: RetargetPort,
        supervisor: JointCommandSupervisor,
    ) -> None:
        if type(side) is not HandSide:
            raise ValueError("side must be a HandSide")
        if supervisor.layout != hand2_layout(side.value):
            raise ValueError("supervisor layout must match the configured Hand 2 side")
        self.side = side
        self.observation_input = observation_input
        self.retargeter = retargeter
        self.supervisor = supervisor
        self._started = False
        self._closed = False
        self._intent_sequence = 0
        self._last_intent: HandIntent | None = None
        self._pending_hold_reason: str | None = None

    def start(self, *, now_ns: int) -> SafetyDecision:
        """Open input and reset all state at a bounded simulation epoch."""

        if self._closed:
            raise RuntimeError("Glove Hand 2 simulation controller is closed")
        if self._started:
            raise RuntimeError("Glove Hand 2 simulation controller is already started")
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError("now_ns must be a non-negative integer")
        self.observation_input.start()
        try:
            self.retargeter.reset()
            decision = self.supervisor.arm(now_ns)
        except Exception:
            self.observation_input.close()
            raise
        self._started = True
        self._intent_sequence = 0
        self._last_intent = None
        self._pending_hold_reason = None
        return decision

    def poll(self, *, now_ns: int) -> Hand2SimulationStep:
        """Acquire, retarget, and supervise one newly available canonical frame."""

        self._require_started()
        if self._pending_hold_reason is not None:
            reason = self._pending_hold_reason
            self._pending_hold_reason = None
            return Hand2SimulationStep(
                intent=None,
                decision=self.supervisor.hold(
                    now_ns=now_ns,
                    reason=reason,
                ),
                rejection_reason=reason,
            )
        observation = self.observation_input.poll(receive_time_ns=now_ns)
        try:
            self._validate_observation_side(observation)
            intent = self.retargeter.retarget(
                observation,
                sequence=self._intent_sequence,
                produced_time_ns=now_ns,
            )
            if (
                intent.side is not self.side
                or intent.source_observation is not observation
                or intent.layout_id != HAND2_LAYOUT_IDS[self.side.value]
            ):
                raise RuntimeError("retargeter returned an intent for the wrong source/side/layout")
        except (RuntimeError, ValueError) as exc:
            return self._advance(
                now_ns=now_ns,
                rejection_reason=f"retarget_rejected:{type(exc).__name__}",
            )
        decision = self.supervisor.step(
            intent.q20_rad,
            now_ns=now_ns,
            input_time_ns=observation.receive_time_ns,
        )
        self._intent_sequence += 1
        self._last_intent = intent
        return Hand2SimulationStep(intent=intent, decision=decision)

    def advance_without_observation(self, *, now_ns: int) -> Hand2SimulationStep:
        """Advance supervision using the last intent until it becomes stale."""

        self._require_started()
        return self._advance(now_ns=now_ns, rejection_reason=None)

    def reject_observation(
        self,
        *,
        now_ns: int,
        reason: str,
    ) -> Hand2SimulationStep:
        """Reject an input-adapter frame without creating a new q20 intent."""

        self._require_started()
        if not reason or len(reason) > 128:
            raise ValueError("rejection reason must be a bounded non-empty string")
        return self._advance(now_ns=now_ns, rejection_reason=reason)

    def invalidate_input_epoch(
        self,
    ) -> None:
        """Forget transport state and hold once on the next control tick."""

        self._require_started()
        self.retargeter.reset()
        self._last_intent = None
        self._intent_sequence = 0
        self._pending_hold_reason = "hand_input_epoch_changed_hold"

    def _advance(
        self,
        *,
        now_ns: int,
        rejection_reason: str | None,
    ) -> Hand2SimulationStep:
        intent = self._last_intent
        decision = self.supervisor.step(
            None if intent is None else intent.q20_rad,
            now_ns=now_ns,
            input_time_ns=(None if intent is None else intent.source_observation.receive_time_ns),
        )
        return Hand2SimulationStep(
            intent=None,
            decision=decision,
            rejection_reason=rejection_reason,
        )

    def close(self) -> SafetyDecision:
        """Close owned port lifetimes and disarm at the configured rest command."""

        if self._closed:
            return self.supervisor.disarm()
        self._closed = True
        self._started = False
        first_error: Exception | None = None
        try:
            self.observation_input.close()
        except Exception as exc:  # pragma: no cover - defensive adapter cleanup
            first_error = exc
        close = getattr(self.retargeter, "close", None)
        if callable(close):
            try:
                close()
            except Exception as exc:  # pragma: no cover - defensive adapter cleanup
                if first_error is None:
                    first_error = exc
        decision = self.supervisor.disarm()
        self._last_intent = None
        self._pending_hold_reason = None
        if first_error is not None:
            raise first_error
        return decision

    def _require_started(self) -> None:
        if not self._started or self._closed:
            raise RuntimeError("start() must succeed before advancing teleoperation")

    def _validate_observation_side(
        self,
        observation: CanonicalHandObservation,
    ) -> None:
        if type(observation) is not CanonicalHandObservation:
            raise RuntimeError("input port returned a non-canonical hand observation")
        if observation.side is not self.side:
            raise RuntimeError("input port returned the wrong hand side")


def compose_q27_hand_target(
    baseline_q27: object,
    hand_indices_q20: object,
    command_q20: object,
) -> FloatArray:
    """Replace exactly one q20 partition while preserving the q7 partition."""

    try:
        baseline = np.asarray(baseline_q27, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("baseline_q27 must be a finite numeric q27 vector") from exc
    if baseline.shape != (27,) or not np.isfinite(baseline).all():
        raise ValueError("baseline_q27 must be a finite numeric q27 vector")

    if isinstance(hand_indices_q20, (str, bytes, bytearray)) or not isinstance(
        hand_indices_q20,
        Iterable,
    ):
        raise ValueError("hand_indices_q20 must contain 20 unique q27 indices")
    raw_indices: tuple[object, ...] = tuple(hand_indices_q20)
    if (
        len(raw_indices) != 20
        or any(type(index) is not int or not 0 <= index < 27 for index in raw_indices)
        or len(set(raw_indices)) != 20
    ):
        raise ValueError("hand_indices_q20 must contain 20 unique q27 indices")
    indices = cast(tuple[int, ...], raw_indices)

    try:
        command = np.asarray(command_q20, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("command_q20 must be a finite numeric q20 vector") from exc
    if command.shape != (20,) or not np.isfinite(command).all():
        raise ValueError("command_q20 must be a finite numeric q20 vector")

    result = baseline.copy()
    result[np.asarray(indices, dtype=np.int64)] = command
    return result
