"""Transport-neutral one-tick orchestration for dual teleoperation."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol

from wujihand.domain import TrackedRigidBodySample

from .glove_hand2_set import (
    GloveHand2ControllerSet,
    SideHand2SimulationStep,
)
from .tracker_arm_simulation import (
    TrackerArmSimulationController,
    TrackerArmSimulationStep,
)


class TrackerSampleInputPort(Protocol):
    def receive_available(
        self,
        *,
        now_ns: int,
    ) -> Sequence[TrackedRigidBodySample]: ...


@dataclass(frozen=True, slots=True)
class SideTrackerArmSimulationStep:
    side: str
    step: TrackerArmSimulationStep


@dataclass(frozen=True, slots=True)
class DualTeleoperationCycleResult:
    arm_steps: tuple[SideTrackerArmSimulationStep, ...]
    hand_steps: tuple[SideHand2SimulationStep, ...]


class DualTeleoperationCycle:
    """Advance all configured routes once without knowing their transport."""

    def __init__(
        self,
        *,
        arm_inputs: Mapping[str, TrackerSampleInputPort],
        arm_controllers: Mapping[str, TrackerArmSimulationController],
        hand_controllers: GloveHand2ControllerSet,
    ) -> None:
        if set(arm_inputs) != set(arm_controllers):
            raise ValueError(
                "arm inputs and controllers must cover the same sides"
            )
        if not set(arm_inputs).issubset({"left", "right"}):
            raise ValueError("arm input sides must be left or right")
        self._arm_inputs = dict(arm_inputs)
        self._arm_controllers = dict(arm_controllers)
        self._hand_controllers = hand_controllers

    def step(
        self,
        *,
        feedback_q7_rad: Mapping[str, Sequence[float]],
        now_ns: int,
    ) -> DualTeleoperationCycleResult:
        if set(feedback_q7_rad) != set(self._arm_controllers):
            raise ValueError(
                "arm feedback must cover every configured arm controller"
            )
        arm_steps = tuple(
            SideTrackerArmSimulationStep(
                side=side,
                step=self._arm_controllers[side].step(
                    self._arm_inputs[side].receive_available(now_ns=now_ns),
                    feedback_q7_rad=feedback_q7_rad[side],
                    now_ns=now_ns,
                ),
            )
            for side in sorted(self._arm_controllers)
        )
        return DualTeleoperationCycleResult(
            arm_steps=arm_steps,
            hand_steps=self._hand_controllers.step(now_ns=now_ns),
        )


__all__ = [
    "DualTeleoperationCycle",
    "DualTeleoperationCycleResult",
    "SideTrackerArmSimulationStep",
    "TrackerSampleInputPort",
]
