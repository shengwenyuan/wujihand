"""Fail-closed position-command supervision for simulation and later adapters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np
import numpy.typing as npt

from wujihand.domain.joints import FloatArray, JointLayout


class SafetyState(str, Enum):
    DISARMED = "disarmed"
    TRACKING = "tracking"
    DEGRADED = "degraded"


@dataclass(frozen=True, slots=True)
class SafetyDecision:
    command: FloatArray
    state: SafetyState
    reason: str
    position_clamped: bool
    rate_limited: bool


class JointCommandSupervisor:
    """Clamp positions, limit slew rate, and return to rest after stale input."""

    def __init__(
        self,
        layout: JointLayout,
        rest_position: Sequence[float],
        *,
        stale_after_s: float = 0.25,
        velocity_scale: float = 0.20,
    ) -> None:
        if stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be positive")
        if not 0.0 < velocity_scale <= 1.0:
            raise ValueError("velocity_scale must be in (0, 1]")
        self.layout = layout
        self.rest = layout.clamp(rest_position)
        self.stale_after_ns = int(stale_after_s * 1e9)
        self.max_velocity = np.asarray(layout.velocity, dtype=np.float64) * velocity_scale
        self.state = SafetyState.DISARMED
        self.last_command = self.rest.copy()
        self.last_step_ns: int | None = None

    def arm(self, now_ns: int) -> SafetyDecision:
        if now_ns < 0:
            raise ValueError("now_ns must be non-negative")
        self.state = SafetyState.TRACKING
        self.last_command = self.rest.copy()
        self.last_step_ns = now_ns
        return self._decision("armed_at_rest", False, False)

    def disarm(self) -> SafetyDecision:
        self.state = SafetyState.DISARMED
        self.last_command = self.rest.copy()
        self.last_step_ns = None
        return self._decision("disarmed_at_rest", False, False)

    def step(
        self,
        intent: Sequence[float] | npt.NDArray[np.floating] | None,
        *,
        now_ns: int,
        input_time_ns: int | None = None,
    ) -> SafetyDecision:
        if self.state is SafetyState.DISARMED:
            return self._decision("disarmed", False, False)
        if self.last_step_ns is None or now_ns <= self.last_step_ns:
            raise ValueError("now_ns must increase strictly while armed")

        target = self.rest
        reason = "missing_input_return_to_rest"
        position_clamped = False
        valid_input = False
        if intent is not None:
            if input_time_ns is None or input_time_ns > now_ns:
                reason = "invalid_input_timestamp_return_to_rest"
            elif now_ns - input_time_ns > self.stale_after_ns:
                reason = "stale_input_return_to_rest"
            else:
                try:
                    raw = self.layout.validate_vector(intent)
                except ValueError:
                    reason = "invalid_input_return_to_rest"
                else:
                    target = self.layout.clamp(raw)
                    position_clamped = not np.array_equal(target, raw)
                    valid_input = True
                    reason = "tracking_clamped" if position_clamped else "tracking"

        dt_s = (now_ns - self.last_step_ns) / 1e9
        max_delta = self.max_velocity * dt_s
        delta = target - self.last_command
        limited_delta = np.clip(delta, -max_delta, max_delta)
        rate_limited = not np.allclose(limited_delta, delta, rtol=0.0, atol=1e-12)
        self.last_command = self.layout.clamp(self.last_command + limited_delta)
        self.last_step_ns = now_ns
        self.state = SafetyState.TRACKING if valid_input else SafetyState.DEGRADED
        return self._decision(reason, position_clamped, rate_limited)

    def _decision(self, reason: str, position_clamped: bool, rate_limited: bool) -> SafetyDecision:
        return SafetyDecision(
            command=self.last_command.copy(),
            state=self.state,
            reason=reason,
            position_clamped=position_clamped,
            rate_limited=rate_limited,
        )
