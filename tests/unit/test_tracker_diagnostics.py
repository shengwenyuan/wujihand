from __future__ import annotations

import math

import pytest

from wujihand.application.teleoperation import (
    joint_limit_margins,
    nearest_joint_limit_margin,
    tracker_target_motion,
)
from wujihand.domain.joints import JointLayout
from wujihand.domain.pose import euler_zyx_to_quaternion_wxyz


def layout() -> JointLayout:
    return JointLayout(
        names=("joint1", "joint2"),
        lower=(-1.0, -2.0),
        upper=(1.0, 2.0),
        velocity=(2.0, 3.0),
    )


def test_tracker_target_motion_reports_step_and_rate() -> None:
    motion = tracker_target_motion(
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
        1_000_000_000,
        (0.1, 0.0, 0.0),
        euler_zyx_to_quaternion_wxyz(
            yaw=math.pi / 2.0,
            pitch=0.0,
            roll=0.0,
        ),
        1_500_000_000,
    )

    assert motion.sample_interval_s == pytest.approx(0.5)
    assert motion.translation_step_m == pytest.approx(0.1)
    assert motion.translation_speed_m_s == pytest.approx(0.2)
    assert motion.rotation_step_rad == pytest.approx(math.pi / 2.0)
    assert motion.rotation_speed_rad_s == pytest.approx(math.pi)


def test_tracker_target_motion_requires_increasing_time() -> None:
    with pytest.raises(ValueError, match="greater"):
        tracker_target_motion(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            100,
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            100,
        )


def test_joint_limit_margins_preserve_signed_boundary_distance() -> None:
    margins = joint_limit_margins(layout(), (0.75, -2.1))

    assert margins[0].joint_name == "joint1"
    assert margins[0].nearest_limit == "upper"
    assert margins[0].margin_to_nearest_limit_rad == pytest.approx(0.25)
    assert margins[0].within_limits
    assert margins[1].joint_name == "joint2"
    assert margins[1].nearest_limit == "lower"
    assert margins[1].margin_to_nearest_limit_rad == pytest.approx(-0.1)
    assert not margins[1].within_limits


def test_nearest_joint_limit_margin_selects_smallest_margin() -> None:
    nearest = nearest_joint_limit_margin(layout(), (0.2, 1.9))

    assert nearest.joint_name == "joint2"
    assert nearest.nearest_limit == "upper"
    assert nearest.margin_to_nearest_limit_rad == pytest.approx(0.1)
