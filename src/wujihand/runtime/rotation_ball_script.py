"""Deterministic scripted command used to qualify the rotation-ball scene."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import numpy.typing as npt

from wujihand.domain.pose import euler_zyx_to_quaternion_wxyz

from .rotation_ball_config import RotationBallConfig


FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True, slots=True)
class ScriptedRotationBallTarget:
    """One atomic finger/orientation intent and its qualification phase."""

    q20: FloatArray
    root_delta_quat_wxyz: FloatArray
    phase: str


def _smoothstep(value: float) -> float:
    clamped = float(np.clip(value, 0.0, 1.0))
    return clamped * clamped * (3.0 - 2.0 * clamped)


def _blend(start: float, end: float, progress: float) -> float:
    return start + (end - start) * _smoothstep(progress)


def scripted_rotation_ball_target(
    elapsed_s: float,
    config: RotationBallConfig,
) -> ScriptedRotationBallTarget:
    """Return task-home/open -> tilt -> close -> counter-tilt -> hold -> release.

    The sphere remains a dynamic rigid body throughout.  The script only drives
    the physical wrist D6 and q20 joints; it never moves or attaches the ball.
    """

    if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
        raise ValueError("elapsed_s must be finite and non-negative")
    rest = np.zeros(20, dtype=np.float64)
    close = np.asarray(config.script.close_q20, dtype=np.float64)
    hold = np.asarray(config.script.hold_q20, dtype=np.float64)
    q20: FloatArray

    if elapsed_s < 1.0:
        pitch = 0.0
        q20 = rest
        phase = "settle_home_open"
    elif elapsed_s < 3.0:
        pitch = _blend(
            0.0,
            config.script.pregrasp_delta_pitch_rad,
            (elapsed_s - 1.0) / 2.0,
        )
        q20 = rest
        phase = "tilt_to_pregrasp"
    elif elapsed_s < 6.0:
        pitch = config.script.pregrasp_delta_pitch_rad
        q20 = close * _smoothstep((elapsed_s - 3.0) / 3.0)
        phase = "close_fingers"
    elif elapsed_s < 6.5:
        pitch = config.script.pregrasp_delta_pitch_rad
        q20 = close
        phase = "settle_grasp"
    elif elapsed_s < 8.5:
        pitch = _blend(
            config.script.pregrasp_delta_pitch_rad,
            config.script.lifted_delta_pitch_rad,
            (elapsed_s - 6.5) / 2.0,
        )
        squeeze = _smoothstep((elapsed_s - 6.5) / 2.0)
        q20 = close + (hold - close) * squeeze
        phase = "counter_tilt_lift"
    elif elapsed_s < 10.0:
        pitch = config.script.lifted_delta_pitch_rad
        q20 = hold
        phase = "qualification_hold"
    elif elapsed_s < 11.0:
        pitch = config.script.lifted_delta_pitch_rad
        q20 = hold * (1.0 - _smoothstep(elapsed_s - 10.0))
        phase = "release"
    elif elapsed_s < 13.0:
        pitch = _blend(
            config.script.lifted_delta_pitch_rad,
            0.0,
            (elapsed_s - 11.0) / 2.0,
        )
        q20 = rest
        phase = "return_home"
    else:
        pitch = 0.0
        q20 = rest
        phase = "complete"

    quaternion = euler_zyx_to_quaternion_wxyz(yaw=0.0, pitch=pitch, roll=0.0)
    return ScriptedRotationBallTarget(
        q20=q20.copy(),
        root_delta_quat_wxyz=quaternion,
        phase=phase,
    )


__all__ = ["ScriptedRotationBallTarget", "scripted_rotation_ball_target"]
