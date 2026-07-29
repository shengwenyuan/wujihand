"""Compile NV-4 control ownership from a resolved DeploymentSpec."""

from __future__ import annotations

from dataclasses import dataclass

from .deployment_resolver import (
    ResolvedDeployment,
    ResolvedDeploymentSource,
)


@dataclass(frozen=True, slots=True)
class NativeDualSidePlan:
    """One side's arm and hand source ownership."""

    side: str
    arm: ResolvedDeploymentSource
    hand: ResolvedDeploymentSource

    @property
    def live(self) -> bool:
        return (
            self.arm.source.kind == "vive_tracker"
            and self.hand.source.kind == "wuji_glove"
        )


@dataclass(frozen=True, slots=True)
class NativeDualRuntimePlan:
    """Exactly two side plans with explicit live/fixture ownership."""

    deployment_id: str
    sides: tuple[NativeDualSidePlan, NativeDualSidePlan]

    def side(self, side: str) -> NativeDualSidePlan:
        for plan in self.sides:
            if plan.side == side:
                return plan
        raise KeyError(side)

    @property
    def live_sides(self) -> tuple[str, ...]:
        return tuple(plan.side for plan in self.sides if plan.live)


def build_native_dual_runtime_plan(
    resolved: ResolvedDeployment,
) -> NativeDualRuntimePlan:
    """Compile source routes without adding runner-side mode branches."""

    routes = {
        (binding.instance_id, binding.group_id): resolved.source(
            binding.source_id
        )
        for binding in resolved.deployment.control_bindings
    }
    expected_routes = {
        ("nero_left", "arm_joints"),
        ("hand_left", "finger_joints"),
        ("nero_right", "arm_joints"),
        ("hand_right", "finger_joints"),
    }
    if set(routes) != expected_routes:
        raise ValueError(
            "native dual runtime requires the canonical four control routes"
        )

    side_plans: list[NativeDualSidePlan] = []
    for side in ("left", "right"):
        arm = routes[(f"nero_{side}", "arm_joints")]
        hand = routes[(f"hand_{side}", "finger_joints")]
        kinds = (arm.source.kind, hand.source.kind)
        if kinds not in {
            ("vive_tracker", "wuji_glove"),
            ("arm_hold_fixture", "hand_rest_fixture"),
        }:
            raise ValueError(
                f"{side} arm and hand must be jointly live or jointly fixture-held"
            )
        if kinds == ("vive_tracker", "wuji_glove") and (
            arm.local_binding is None or hand.local_binding is None
        ):
            raise ValueError(f"{side} live sources require local bindings")
        if kinds == ("arm_hold_fixture", "hand_rest_fixture") and (
            arm.local_binding is not None or hand.local_binding is not None
        ):
            raise ValueError(
                f"{side} fixture sources must not have local bindings"
            )
        side_plans.append(
            NativeDualSidePlan(
                side=side,
                arm=arm,
                hand=hand,
            )
        )
    result = NativeDualRuntimePlan(
        deployment_id=resolved.deployment.deployment_id,
        sides=(side_plans[0], side_plans[1]),
    )
    if not result.live_sides:
        raise ValueError("native dual runtime requires at least one live side")
    return result


__all__ = [
    "NativeDualRuntimePlan",
    "NativeDualSidePlan",
    "build_native_dual_runtime_plan",
]
