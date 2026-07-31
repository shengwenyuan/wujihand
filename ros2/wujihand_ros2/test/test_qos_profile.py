from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "ros2/wujihand_ros2"))

from rclpy.qos import (  # type: ignore[import-not-found]  # noqa: E402
    DurabilityPolicy,
    ReliabilityPolicy,
)
from wujihand.specs import RosQosPolicySpec  # noqa: E402
from wujihand_ros2.qos import qos_profile  # noqa: E402


def test_committed_qos_contract_maps_to_rclpy() -> None:
    policy = RosQosPolicySpec.from_mapping(
        {
            "channel": "tracker_sample",
            "history": "keep_last",
            "depth": 1,
            "reliability": "best_effort",
            "durability": "volatile",
            "deadline_ms": None,
            "lifespan_ms": None,
        },
        field="fixture",
    )

    qos = qos_profile(policy)

    assert qos.depth == 1
    assert qos.reliability is ReliabilityPolicy.BEST_EFFORT
    assert qos.durability is DurabilityPolicy.VOLATILE
