"""Pure recording-plan helpers shared by launch and offline tests."""

from __future__ import annotations

from collections.abc import Iterable

from wujihand.runtime import DualTeleoperationRoutePlan


def recording_topics(
    namespace: str,
    route_plan: DualTeleoperationRoutePlan,
    *,
    include_synthetic_d405: bool = False,
    include_dataset_facts: bool = False,
) -> tuple[str, ...]:
    """Return the frozen allowlist for one resolved ROS control graph."""

    if include_synthetic_d405 and include_dataset_facts:
        raise ValueError(
            "dataset recording uses separate offline RGB and cannot include online D405 payloads"
        )
    root = "/" + namespace.strip("/")
    topics: list[str] = []
    tracker_sides = sorted(
        route.side for route in route_plan.routes if route.source.kind == "vive_tracker"
    )
    glove_sides = sorted(
        route.side for route in route_plan.routes if route.source.kind == "wuji_glove"
    )
    topics.extend(f"{root}/input/tracker/{side}/sample" for side in tracker_sides)
    if tracker_sides:
        topics.append(f"{root}/input/tracker/lifecycle")
    topics.extend(f"{root}/input/glove/{side}/observation" for side in glove_sides)
    for route in route_plan.routes:
        if route.source.kind not in {"vive_tracker", "wuji_glove"}:
            continue
        kind = "arm" if route.group_id == "arm_joints" else "hand"
        topics.extend(
            f"{root}/{route.side}/{kind}/{leaf}" for leaf in ("command", "feedback", "safety")
        )
    topics.extend(
        (
            f"{root}/runtime/tick",
            f"{root}/scene/rigid_body_state",
            f"{root}/recording/status",
        )
    )
    if include_synthetic_d405:
        for side in ("left", "right"):
            base = f"{root}/{side}/wrist_camera"
            topics.extend(
                (
                    f"{base}/color/image_raw",
                    f"{base}/depth/image_raw",
                    f"{base}/camera_info",
                    f"{base}/frame_truth",
                )
            )
        topics.extend(("/tf", "/tf_static"))
    if include_dataset_facts:
        topics.extend(
            (
                f"{root}/dataset/episode_boundary",
                f"{root}/dataset/simulation_state",
            )
        )
    return _unique(topics)


def source_topics(
    namespace: str,
    route_plan: DualTeleoperationRoutePlan,
) -> tuple[str, ...]:
    """Topics that need consumer + recorder discovery before activation."""

    topics = recording_topics(namespace, route_plan)
    return tuple(topic for topic in topics if "/input/" in topic)


def _unique(values: Iterable[str]) -> tuple[str, ...]:
    result = tuple(values)
    if len(set(result)) != len(result):
        raise ValueError("recording topic allowlist contains duplicates")
    return result


__all__ = ["recording_topics", "source_topics"]
