"""Transport-neutral projection used to compare deployment semantics."""

from __future__ import annotations

from dataclasses import dataclass
import re

from wujihand.specs import (
    DeploymentSpec,
    RosDeploymentSpec,
    SessionSpec,
)


_SHA256 = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True, slots=True)
class SessionControlFacts:
    backend: str
    assembly: tuple[str, str]
    workcell: tuple[str, str]
    bindings: tuple[tuple[str, str, str], ...]
    placements: tuple[tuple[str, str], ...]
    layouts: tuple[tuple[str, str, str], ...]


@dataclass(frozen=True, slots=True)
class DeploymentRouteFacts:
    instance_id: str
    group_id: str
    layout_id: str
    source_id: str
    source_kind: str
    side: str
    logical_role: str
    local_binding_key: str | None


@dataclass(frozen=True, slots=True)
class CommonDeploymentProjection:
    session: SessionControlFacts
    tracking_setup_revision: str
    tracking_frame: str
    tracking_qualification_status: str
    mapping_reference: tuple[str, str]
    mapping_sha256: str
    routes: tuple[DeploymentRouteFacts, ...]


def common_deployment_projection(
    deployment: DeploymentSpec | RosDeploymentSpec,
    session: SessionSpec,
    *,
    mapping_sha256: str,
) -> CommonDeploymentProjection:
    """Project only facts that native/UDP and ROS must hold equal."""

    if not _SHA256.fullmatch(mapping_sha256):
        raise ValueError("mapping_sha256 must be a lowercase SHA-256 digest")
    if deployment.session.expected_id != session.session_id:
        raise ValueError("deployment and Session identity differ")
    sources = {source.source_id: source for source in deployment.sources}
    layouts = {
        (layout.instance_id, layout.group_id): layout.layout_id
        for layout in session.runtime.control_layouts
    }
    route_facts: list[DeploymentRouteFacts] = []
    for binding in deployment.control_bindings:
        route = (binding.instance_id, binding.group_id)
        try:
            source = sources[binding.source_id]
        except KeyError as exc:
            raise ValueError(
                f"control route references unknown source {binding.source_id!r}"
            ) from exc
        try:
            layout_id = layouts[route]
        except KeyError as exc:
            raise ValueError(
                f"control route {route!r} is absent from Session layouts"
            ) from exc
        route_facts.append(
            DeploymentRouteFacts(
                instance_id=binding.instance_id,
                group_id=binding.group_id,
                layout_id=layout_id,
                source_id=source.source_id,
                source_kind=source.kind,
                side=source.side,
                logical_role=source.logical_role,
                local_binding_key=source.local_binding_key,
            )
        )
    return CommonDeploymentProjection(
        session=SessionControlFacts(
            backend=session.backend,
            assembly=(
                session.assembly.path,
                session.assembly.expected_id,
            ),
            workcell=(
                session.workcell.path,
                session.workcell.expected_id,
            ),
            bindings=tuple(
                (instance_id, reference.path, reference.expected_id)
                for instance_id, reference in session.bindings
            ),
            placements=session.placements,
            layouts=tuple(
                sorted(
                    (
                        layout.instance_id,
                        layout.group_id,
                        layout.layout_id,
                    )
                    for layout in session.runtime.control_layouts
                )
            ),
        ),
        tracking_setup_revision=deployment.tracking_setup.setup_revision,
        tracking_frame=deployment.tracking_setup.tracking_frame,
        tracking_qualification_status=(
            deployment.tracking_setup.qualification_status
        ),
        mapping_reference=(
            deployment.tracking_setup.mapping.path,
            deployment.tracking_setup.mapping.expected_id,
        ),
        mapping_sha256=mapping_sha256,
        routes=tuple(
            sorted(
                route_facts,
                key=lambda item: (item.instance_id, item.group_id),
            )
        ),
    )


__all__ = [
    "CommonDeploymentProjection",
    "DeploymentRouteFacts",
    "SessionControlFacts",
    "common_deployment_projection",
]
