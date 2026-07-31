"""NV-4 native process validation around the shared four-route compiler."""

from __future__ import annotations

from .deployment_resolver import ResolvedDeployment
from .dual_route_plan import (
    DualTeleoperationRoute,
    DualTeleoperationRoutePlan,
    build_dual_teleoperation_route_plan,
)


NATIVE_DUAL_RUNTIME_COMPONENT = "isaac_nero_hand2_native_dual_runtime"

NativeDualRoutePlan = DualTeleoperationRoute
NativeDualRuntimePlan = DualTeleoperationRoutePlan


def build_native_dual_runtime_plan(
    resolved: ResolvedDeployment,
) -> NativeDualRuntimePlan:
    """Validate native ownership, then compile transport-neutral routes."""

    runtime = resolved.process("isaac_runtime").process
    if runtime.lifecycle != "in_process":
        raise ValueError("native dual Isaac runtime must be in-process")
    if runtime.component_id != NATIVE_DUAL_RUNTIME_COMPONENT:
        raise ValueError(
            "native dual Isaac runtime component is unsupported: "
            f"{runtime.component_id}"
        )
    local_sources = {
        item.local_binding.binding_key: item.local_binding
        for item in resolved.sources
        if item.local_binding is not None
    }
    return build_dual_teleoperation_route_plan(
        resolved.deployment,
        local_sources=local_sources,
    )


__all__ = [
    "NATIVE_DUAL_RUNTIME_COMPONENT",
    "NativeDualRoutePlan",
    "NativeDualRuntimePlan",
    "build_native_dual_runtime_plan",
]
