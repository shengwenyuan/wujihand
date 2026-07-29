from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.qualification import (
    FULL_SCRIPTED_Q27_SETTLING_POLICY,
    GLOVE_LIVE_Q27_READINESS_POLICY,
    Q27ReadinessPolicy,
    q27_window_max_delta_rad,
)


def test_glove_live_policy_is_shorter_and_nonblocking_after_bounded_warmup() -> None:
    full = FULL_SCRIPTED_Q27_SETTLING_POLICY
    live = GLOVE_LIVE_Q27_READINESS_POLICY

    assert live.window_frames < full.window_frames
    assert live.maximum_windows < full.maximum_windows
    assert live.max_window_delta_rad > full.max_window_delta_rad
    assert live.require_convergence is False
    assert live.window_frames * live.maximum_windows == 60


def test_q27_window_delta_covers_both_sides() -> None:
    previous = {
        "left": np.zeros(27),
        "right": np.zeros(27),
    }
    current = {
        "left": np.full(27, 0.01),
        "right": np.full(27, -0.025),
    }

    assert q27_window_max_delta_rad(previous, current) == pytest.approx(0.025)


@pytest.mark.parametrize(
    ("previous", "current"),
    (
        ({"left": np.zeros(27)}, {"right": np.zeros(27)}),
        ({"left": np.zeros(26)}, {"left": np.zeros(26)}),
        ({"left": np.zeros(27)}, {"left": np.full(27, np.nan)}),
        ({}, {}),
    ),
)
def test_q27_window_delta_rejects_malformed_feedback(
    previous: dict[str, object],
    current: dict[str, object],
) -> None:
    with pytest.raises(ValueError, match="q27 readiness"):
        q27_window_max_delta_rad(previous, current)


@pytest.mark.parametrize(
    "kwargs",
    (
        {"policy_id": ""},
        {"window_frames": 0},
        {"minimum_windows": 1},
        {"minimum_windows": 3, "maximum_windows": 2},
        {"max_window_delta_rad": 0.0},
        {"require_convergence": 1},
    ),
)
def test_readiness_policy_rejects_invalid_configuration(
    kwargs: dict[str, object],
) -> None:
    values: dict[str, object] = {
        "policy_id": "fixture.live.v1",
        "window_frames": 10,
        "minimum_windows": 2,
        "maximum_windows": 4,
        "max_window_delta_rad": 0.01,
        "require_convergence": False,
    }
    values.update(kwargs)
    with pytest.raises(ValueError):
        Q27ReadinessPolicy(**values)  # type: ignore[arg-type]
