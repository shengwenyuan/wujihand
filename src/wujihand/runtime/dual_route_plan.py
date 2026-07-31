"""Transport-neutral compiler for the canonical four control routes."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from wujihand.specs import (
    DeploymentSourceSpec,
    DeploymentSpec,
    LocalSourceBindingSpec,
    RosDeploymentSpec,
)


_ROUTE_DEFINITIONS = (
    ("nero_left", "arm_joints", "left"),
    ("hand_left", "finger_joints", "left"),
    ("nero_right", "arm_joints", "right"),
    ("hand_right", "finger_joints", "right"),
)
_SOURCE_KINDS_BY_GROUP = {
    "arm_joints": frozenset({"vive_tracker", "arm_hold_fixture"}),
    "finger_joints": frozenset({"wuji_glove", "hand_rest_fixture"}),
}
_LIVE_SOURCE_KINDS = frozenset({"vive_tracker", "wuji_glove"})


@dataclass(frozen=True, slots=True)
class DualTeleoperationRoute:
    instance_id: str
    group_id: str
    side: str
    source: DeploymentSourceSpec
    local_binding: LocalSourceBindingSpec | None


@dataclass(frozen=True, slots=True)
class DualTeleoperationRoutePlan:
    deployment_id: str
    routes: tuple[
        DualTeleoperationRoute,
        DualTeleoperationRoute,
        DualTeleoperationRoute,
        DualTeleoperationRoute,
    ]

    def route(
        self,
        instance_id: str,
        group_id: str,
    ) -> DualTeleoperationRoute:
        for route in self.routes:
            if (
                route.instance_id == instance_id
                and route.group_id == group_id
            ):
                return route
        raise KeyError((instance_id, group_id))


def build_dual_teleoperation_route_plan(
    deployment: DeploymentSpec | RosDeploymentSpec,
    *,
    local_sources: Mapping[str, LocalSourceBindingSpec],
) -> DualTeleoperationRoutePlan:
    """Compile route ownership without depending on UDP or ROS processes."""

    sources = {source.source_id: source for source in deployment.sources}
    sources_by_route = {
        (binding.instance_id, binding.group_id): sources[binding.source_id]
        for binding in deployment.control_bindings
    }
    expected_routes = {
        (instance_id, group_id)
        for instance_id, group_id, _ in _ROUTE_DEFINITIONS
    }
    if (
        len(deployment.control_bindings) != len(expected_routes)
        or set(sources_by_route) != expected_routes
    ):
        raise ValueError(
            "dual teleoperation requires the canonical four control routes"
        )

    routes: list[DualTeleoperationRoute] = []
    for instance_id, group_id, side in _ROUTE_DEFINITIONS:
        source = sources_by_route[(instance_id, group_id)]
        if source.kind not in _SOURCE_KINDS_BY_GROUP[group_id]:
            raise ValueError(
                f"{instance_id}/{group_id} does not accept source kind "
                f"{source.kind!r}"
            )
        if source.side != side:
            raise ValueError(
                f"{instance_id}/{group_id} requires side {side!r}"
            )
        local_binding = (
            None
            if source.local_binding_key is None
            else local_sources.get(source.local_binding_key)
        )
        if (source.kind in _LIVE_SOURCE_KINDS) != (
            local_binding is not None
        ):
            raise ValueError(
                f"{instance_id}/{group_id} source binding does not match "
                f"source kind {source.kind!r}"
            )
        if (
            local_binding is not None
            and local_binding.source_kind != source.kind
        ):
            raise ValueError(
                f"{instance_id}/{group_id} local source kind differs"
            )
        routes.append(
            DualTeleoperationRoute(
                instance_id=instance_id,
                group_id=group_id,
                side=side,
                source=source,
                local_binding=local_binding,
            )
        )

    if not any(route.source.kind == "vive_tracker" for route in routes):
        raise ValueError(
            "dual teleoperation requires at least one Tracker route"
        )
    return DualTeleoperationRoutePlan(
        deployment_id=deployment.deployment_id,
        routes=(routes[0], routes[1], routes[2], routes[3]),
    )


__all__ = [
    "DualTeleoperationRoute",
    "DualTeleoperationRoutePlan",
    "build_dual_teleoperation_route_plan",
]
