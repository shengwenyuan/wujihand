from __future__ import annotations

from pathlib import Path

import yaml

from wujihand.runtime import ConfigRepository


ROOT = Path(__file__).parents[2]


def test_rosbag_input_qos_is_a_projection_of_committed_profile() -> None:
    profile = ConfigRepository(ROOT).load_ros_qos_profile(
        "configs/profiles/ros2_jazzy_dual_teleoperation_qos_v1.yaml"
    )
    projection = yaml.safe_load(
        (
            ROOT
            / "configs/profiles/"
            "ros2_jazzy_dual_teleoperation_rosbag_qos_v1.yaml"
        ).read_text(encoding="utf-8")
    )
    assert isinstance(projection, dict)
    expected_channel = {
        "tracker": "tracker_sample",
        "glove": "glove_observation",
    }
    for topic, values in projection.items():
        channel = (
            "tracking_lifecycle"
            if topic.endswith("/lifecycle")
            else expected_channel[
                "tracker" if "/tracker/" in topic else "glove"
            ]
        )
        policy = profile.policy(channel)
        assert values == {
            "history": policy.history,
            "depth": policy.depth,
            "reliability": policy.reliability,
            "durability": policy.durability,
        }
