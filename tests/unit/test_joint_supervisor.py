from __future__ import annotations

import numpy as np

from wujihand.application.supervision import JointCommandSupervisor, SafetyState
from wujihand.domain.joints import JointLayout


def layout() -> JointLayout:
    return JointLayout(("a", "b"), (-1.0, -2.0), (1.0, 2.0), (2.0, 4.0))


def test_disarmed_is_fail_closed_at_rest() -> None:
    supervisor = JointCommandSupervisor(layout(), [0.1, -0.2])
    decision = supervisor.step([1.0, 1.0], now_ns=1, input_time_ns=1)
    assert decision.state is SafetyState.DISARMED
    np.testing.assert_array_equal(decision.command, [0.1, -0.2])


def test_tracking_clamps_and_rate_limits() -> None:
    supervisor = JointCommandSupervisor(layout(), [0.0, 0.0], velocity_scale=0.5)
    supervisor.arm(0)
    decision = supervisor.step([9.0, 9.0], now_ns=100_000_000, input_time_ns=90_000_000)
    assert decision.state is SafetyState.TRACKING
    assert decision.position_clamped
    assert decision.rate_limited
    np.testing.assert_allclose(decision.command, [0.1, 0.2])


def test_stale_boundary_then_gradual_return_to_rest() -> None:
    supervisor = JointCommandSupervisor(layout(), [0.0, 0.0], velocity_scale=0.5)
    supervisor.arm(0)
    supervisor.step([0.5, 1.0], now_ns=500_000_000, input_time_ns=500_000_000)

    boundary = supervisor.step([0.5, 1.0], now_ns=750_000_000, input_time_ns=500_000_000)
    assert boundary.state is SafetyState.TRACKING
    np.testing.assert_allclose(boundary.command, [0.5, 1.0])

    decision = supervisor.step([0.5, 1.0], now_ns=760_000_000, input_time_ns=500_000_000)
    assert decision.state is SafetyState.DEGRADED
    assert decision.reason == "stale_input_return_to_rest"
    assert decision.rate_limited
    np.testing.assert_allclose(decision.command, [0.49, 0.98])


def test_explicit_hold_preserves_command_and_advances_time() -> None:
    supervisor = JointCommandSupervisor(
        layout(),
        [0.0, 0.0],
        velocity_scale=1.0,
    )
    supervisor.arm(0)
    tracked = supervisor.step(
        [0.5, -0.5],
        now_ns=100_000_000,
        input_time_ns=100_000_000,
    )
    held = supervisor.hold(
        now_ns=200_000_000,
        reason="tracking_reference_required_hold",
    )

    np.testing.assert_array_equal(held.command, tracked.command)
    assert held.state is SafetyState.DEGRADED
    assert held.reason == "tracking_reference_required_hold"


def test_invalid_vector_never_reaches_output() -> None:
    supervisor = JointCommandSupervisor(layout(), [0.0, 0.0])
    supervisor.arm(0)
    decision = supervisor.step([np.nan, 0.0], now_ns=10_000_000, input_time_ns=10_000_000)
    assert decision.state is SafetyState.DEGRADED
    assert decision.reason == "invalid_input_return_to_rest"
    assert np.isfinite(decision.command).all()
