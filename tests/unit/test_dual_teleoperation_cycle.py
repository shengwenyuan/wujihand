from __future__ import annotations

import numpy as np

from wujihand.application.teleoperation import DualTeleoperationCycle


class Input:
    def __init__(self) -> None:
        self.calls: list[int] = []

    def receive_available(self, *, now_ns: int) -> tuple[()]:
        self.calls.append(now_ns)
        return ()


class ArmController:
    def __init__(self) -> None:
        self.calls: list[tuple[object, np.ndarray, int]] = []

    def step(
        self,
        samples: object,
        *,
        feedback_q7_rad: np.ndarray,
        now_ns: int,
    ) -> str:
        self.calls.append((samples, feedback_q7_rad, now_ns))
        return "arm-step"


class HandControllers:
    def __init__(self) -> None:
        self.calls: list[int] = []

    def step(self, *, now_ns: int) -> tuple[str, ...]:
        self.calls.append(now_ns)
        return ("hand-step",)


def test_cycle_advances_each_transport_neutral_route_once() -> None:
    input_port = Input()
    arm = ArmController()
    hands = HandControllers()
    cycle = DualTeleoperationCycle(
        arm_inputs={"right": input_port},
        arm_controllers={"right": arm},  # type: ignore[arg-type]
        hand_controllers=hands,  # type: ignore[arg-type]
    )
    feedback = np.arange(7, dtype=np.float64)

    result = cycle.step(
        feedback_q7_rad={"right": feedback},
        now_ns=10,
    )

    assert input_port.calls == [10]
    assert len(arm.calls) == 1
    assert arm.calls[0][1] is feedback
    assert hands.calls == [10]
    assert result.arm_steps[0].side == "right"
    assert result.arm_steps[0].step == "arm-step"
    assert result.hand_steps == ("hand-step",)


def test_cycle_rejects_feedback_coverage_drift() -> None:
    cycle = DualTeleoperationCycle(
        arm_inputs={"left": Input()},
        arm_controllers={"left": ArmController()},  # type: ignore[arg-type]
        hand_controllers=HandControllers(),  # type: ignore[arg-type]
    )

    try:
        cycle.step(feedback_q7_rad={}, now_ns=1)
    except ValueError as exc:
        assert "cover every configured" in str(exc)
    else:
        raise AssertionError("feedback coverage drift must fail closed")
