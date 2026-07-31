from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import numpy as np

from wujihand.application.supervision import SafetyDecision, SafetyState
from wujihand_ros2.conversion import (
    SafetyEventObservation,
    route_command_from_decision,
    route_command_from_message,
    route_command_to_message,
    safety_event_from_message,
    safety_event_to_message,
)


def message_factory() -> Any:
    return SimpleNamespace()


def test_supervised_route_command_round_trip() -> None:
    observation = route_command_from_decision(
        instance_id="nero_left",
        group_id="arm_joints",
        layout_id="agilex_nero.q7.v1",
        decision=SafetyDecision(
            command=np.arange(7, dtype=np.float64) * 0.1,
            state=SafetyState.DEGRADED,
            reason="ik_failure_hold",
            position_clamped=False,
            rate_limited=True,
        ),
        produced_time_ns=1234,
    )
    message = route_command_to_message(
        observation,
        factory=message_factory,
    )

    assert route_command_from_message(message) == observation


def test_safety_event_round_trip() -> None:
    event = SafetyEventObservation(
        instance_id="hand_right",
        group_id="finger_joints",
        state=SafetyState.TRACKING,
        reason="tracking",
        position_clamped=False,
        rate_limited=False,
        host_time_ns=5678,
    )

    assert safety_event_from_message(
        safety_event_to_message(event, factory=message_factory)
    ) == event
