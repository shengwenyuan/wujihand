"""Compile native/UDP control routes from a resolved DeploymentSpec."""

from __future__ import annotations

from dataclasses import dataclass

from wujihand.specs import DeploymentSourceSpec, LocalSourceBindingSpec

from .deployment_resolver import (
    ResolvedDeployment,
    ResolvedDeploymentSource,
)

NATIVE_DUAL_RUNTIME_COMPONENT = "isaac_nero_hand2_native_dual_runtime"

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
class NativeDualRoutePlan:
    """One canonical control target and its resolved source."""

    instance_id: str
    group_id: str
    side: str
    resolved_source: ResolvedDeploymentSource

    @property
    def source(self) -> DeploymentSourceSpec:
        return self.resolved_source.source

    @property
    def local_binding(self) -> LocalSourceBindingSpec | None:
        return self.resolved_source.local_binding


@dataclass(frozen=True, slots=True)
class NativeDualRuntimePlan:
    """Four explicit control routes for one native/UDP runtime."""

    deployment_id: str
    routes: tuple[
        NativeDualRoutePlan,
        NativeDualRoutePlan,
        NativeDualRoutePlan,
        NativeDualRoutePlan,
    ]

    def route(self, instance_id: str, group_id: str) -> NativeDualRoutePlan:
        for route in self.routes:
            if (
                route.instance_id == instance_id
                and route.group_id == group_id
            ):
                return route
        raise KeyError((instance_id, group_id))


def build_native_dual_runtime_plan(
    resolved: ResolvedDeployment,
) -> NativeDualRuntimePlan:
    """Compile source ownership without mode or transport switches."""

    runtime = resolved.process("isaac_runtime").process
    if runtime.lifecycle != "in_process":
        raise ValueError("native dual Isaac runtime must be in-process")
    if runtime.component_id != NATIVE_DUAL_RUNTIME_COMPONENT:
        raise ValueError(
            "native dual Isaac runtime component is unsupported: "
            f"{runtime.component_id}"
        )

    control_bindings = resolved.deployment.control_bindings
    sources_by_route = {
        (binding.instance_id, binding.group_id): resolved.source(
            binding.source_id
        )
        for binding in control_bindings
    }
    expected_routes = {
        (instance_id, group_id)
        for instance_id, group_id, _ in _ROUTE_DEFINITIONS
    }
    if (
        len(control_bindings) != len(expected_routes)
        or set(sources_by_route) != expected_routes
    ):
        raise ValueError(
            "native dual runtime requires the canonical four control routes"
        )

    routes: list[NativeDualRoutePlan] = []
    for instance_id, group_id, side in _ROUTE_DEFINITIONS:
        source = sources_by_route[(instance_id, group_id)]
        kind = source.source.kind
        if kind not in _SOURCE_KINDS_BY_GROUP[group_id]:
            raise ValueError(
                f"{instance_id}/{group_id} does not accept source kind "
                f"{kind!r}"
            )
        if source.source.side != side:
            raise ValueError(
                f"{instance_id}/{group_id} requires side {side!r}"
            )
        if (kind in _LIVE_SOURCE_KINDS) != (
            source.local_binding is not None
        ):
            raise ValueError(
                f"{instance_id}/{group_id} source binding does not match "
                f"source kind {kind!r}"
            )
        routes.append(
            NativeDualRoutePlan(
                instance_id=instance_id,
                group_id=group_id,
                side=side,
                resolved_source=source,
            )
        )

    if not any(
        route.source.kind == "vive_tracker" for route in routes
    ):
        raise ValueError(
            "native/UDP runtime requires at least one Tracker route"
        )
    return NativeDualRuntimePlan(
        deployment_id=resolved.deployment.deployment_id,
        routes=(routes[0], routes[1], routes[2], routes[3]),
    )


__all__ = [
    "NATIVE_DUAL_RUNTIME_COMPONENT",
    "NativeDualRoutePlan",
    "NativeDualRuntimePlan",
    "build_native_dual_runtime_plan",
]
