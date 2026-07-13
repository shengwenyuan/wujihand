from __future__ import annotations

import numpy as np
import pytest

from wujihand.domain.pose import quaternion_wxyz_to_euler_zyx
from wujihand.runtime import load_rotation_ball_config
from wujihand.runtime.rotation_ball_script import scripted_rotation_ball_target


def _config():  # type: ignore[no-untyped-def]
    return load_rotation_ball_config("configs/base/hand2_rotation_ball_v1.yaml")


@pytest.mark.parametrize(
    ("time_s", "phase"),
    [
        (0.0, "settle_home_open"),
        (2.0, "tilt_to_pregrasp"),
        (4.5, "close_fingers"),
        (6.25, "settle_grasp"),
        (7.5, "counter_tilt_lift"),
        (9.0, "qualification_hold"),
        (10.5, "release"),
        (12.0, "return_home"),
        (13.0, "complete"),
    ],
)
def test_script_phase_sequence(time_s: float, phase: str) -> None:
    target = scripted_rotation_ball_target(time_s, _config())
    assert target.phase == phase
    assert target.q20.shape == (20,)
    assert np.isfinite(target.q20).all()
    assert np.linalg.norm(target.root_delta_quat_wxyz) == pytest.approx(1.0)


def test_script_is_open_at_home_then_closes_and_counter_tilts() -> None:
    config = _config()
    initial = scripted_rotation_ball_target(0.0, config)
    pregrasp = scripted_rotation_ball_target(4.0, config)
    closed = scripted_rotation_ball_target(6.25, config)
    held = scripted_rotation_ball_target(9.0, config)

    np.testing.assert_array_equal(initial.q20, np.zeros(20))
    np.testing.assert_allclose(closed.q20, config.script.close_q20)
    np.testing.assert_allclose(held.q20, config.script.hold_q20)
    _, pregrasp_pitch, _ = quaternion_wxyz_to_euler_zyx(
        pregrasp.root_delta_quat_wxyz
    )
    _, lifted_pitch, _ = quaternion_wxyz_to_euler_zyx(held.root_delta_quat_wxyz)
    assert pregrasp_pitch == pytest.approx(config.script.pregrasp_delta_pitch_rad)
    assert lifted_pitch == pytest.approx(config.script.lifted_delta_pitch_rad)
    assert lifted_pitch < pregrasp_pitch


def test_script_closes_all_fingers_slowly_and_synchronously() -> None:
    config = _config()
    halfway = scripted_rotation_ball_target(4.5, config)

    np.testing.assert_allclose(halfway.q20, np.asarray(config.script.close_q20) * 0.5)


def test_script_rejects_invalid_time() -> None:
    with pytest.raises(ValueError, match="elapsed_s"):
        scripted_rotation_ball_target(-0.1, _config())
