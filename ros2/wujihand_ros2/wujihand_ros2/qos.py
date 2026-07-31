"""Build explicit rclpy QoS from the committed profile."""

from __future__ import annotations

from rclpy.duration import Duration  # type: ignore[import-not-found]
from rclpy.qos import (  # type: ignore[import-not-found]
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from wujihand.specs import RosQosPolicySpec


def qos_profile(policy: RosQosPolicySpec) -> QoSProfile:
    profile = QoSProfile(
        history={
            "keep_last": HistoryPolicy.KEEP_LAST,
        }[policy.history],
        depth=policy.depth,
        reliability={
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
            "reliable": ReliabilityPolicy.RELIABLE,
        }[policy.reliability],
        durability={
            "volatile": DurabilityPolicy.VOLATILE,
            "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        }[policy.durability],
    )
    if policy.deadline_ms is not None:
        profile.deadline = Duration(
            nanoseconds=policy.deadline_ms * 1_000_000
        )
    if policy.lifespan_ms is not None:
        profile.lifespan = Duration(
            nanoseconds=policy.lifespan_ms * 1_000_000
        )
    return profile


__all__ = ["qos_profile"]
