from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from wujihand.domain.pose import (
    euler_zyx_to_quaternion_wxyz,
    multiply_quaternions_wxyz,
    quaternion_wxyz_to_euler_zyx,
)
from wujihand.runtime import load_rotation_ball_config


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/base/hand2_rotation_ball_v2026_6_27_v1.yaml"


def test_rotation_ball_profile_loads_with_fixed_flange_and_table_ball() -> None:
    config = load_rotation_ball_config(PROFILE)

    assert config.name == "hand2_right_rotation_ball_v2026_6_27_v1"
    assert config.table.top_z_m == pytest.approx(0.38)
    assert config.flange.position_m == (0.0, 0.0, 0.454)
    assert config.flange.neutral_quat_wxyz == pytest.approx(
        (0.8660254037844386, 0.0, 0.5, 0.0)
    )
    assert config.ball.center_m == (0.130, 0.025, 0.410)
    assert config.ball.radius_m == pytest.approx(0.030)
    assert config.ball.center_m[2] == pytest.approx(config.table.top_z_m + config.ball.radius_m)
    assert config.script.pregrasp_delta_pitch_rad == pytest.approx(0.2792526803190927)
    assert config.script.lifted_delta_pitch_rad == pytest.approx(0.0)
    assert len(config.script.close_q20) == 20
    assert len(config.script.hold_q20) == 20
    assert config.wrist.min_quality == pytest.approx(0.5)
    assert config.provenance["tag"] == "v2026.6.27"


def test_rotation_ball_profile_rejects_flange_on_table(tmp_path: Path) -> None:
    data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    data["flange"]["position_m"][2] = 0.38
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="above the table"):
        load_rotation_ball_config(bad)


def test_task_home_is_60_degrees_and_script_pregrasp_is_physical_76_degrees() -> None:
    config = load_rotation_ball_config(PROFILE)
    _, home_pitch, _ = quaternion_wxyz_to_euler_zyx(
        config.flange.neutral_quat_wxyz
    )
    relative_pregrasp = euler_zyx_to_quaternion_wxyz(
        yaw=0.0,
        pitch=config.script.pregrasp_delta_pitch_rad,
        roll=0.0,
    )
    physical_pregrasp = multiply_quaternions_wxyz(
        config.flange.neutral_quat_wxyz,
        relative_pregrasp,
    )
    _, pregrasp_pitch, _ = quaternion_wxyz_to_euler_zyx(physical_pregrasp)

    assert home_pitch == pytest.approx(1.0471975511965976)
    assert pregrasp_pitch == pytest.approx(1.3264502315156905)


def test_rotation_ball_profile_rejects_exact_ninety_degree_tilt(tmp_path: Path) -> None:
    data = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    data["wrist"]["pitch_limit_rad"] = 1.5707963267948966
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data), encoding="utf-8")

    with pytest.raises(ValueError, match="90-degree"):
        load_rotation_ball_config(bad)
