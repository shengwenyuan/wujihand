"""Backend-neutral multi-asset assembly forest specification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Self

from .common import (
    ConfigRef,
    PoseSpec,
    require_exact_mapping,
    require_sequence,
    require_string,
    validate_identifier,
)


ASSEMBLY_SCHEMA = "wujihand.assembly_spec.v1"


@dataclass(frozen=True, slots=True)
class AssetInstanceSpec:
    """One namespaced instance of an asset manifest."""

    instance_id: str
    asset: ConfigRef
    role: str
    namespace: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "instance") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"instance_id", "asset", "role", "namespace"}),
            field=field,
        )
        return cls(
            instance_id=validate_identifier(
                data["instance_id"], field=f"{field}.instance_id"
            ),
            asset=ConfigRef.from_mapping(data["asset"], field=f"{field}.asset"),
            role=validate_identifier(data["role"], field=f"{field}.role"),
            namespace=validate_identifier(
                data["namespace"], field=f"{field}.namespace"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "instance_id": self.instance_id,
            "asset": self.asset.to_mapping(),
            "role": self.role,
            "namespace": self.namespace,
        }


@dataclass(frozen=True, slots=True)
class AttachmentEndpointSpec:
    """Canonical frame on an assembly asset instance."""

    instance: str
    frame: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "endpoint") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"instance", "frame"}),
            field=field,
        )
        return cls(
            instance=validate_identifier(data["instance"], field=f"{field}.instance"),
            frame=validate_identifier(data["frame"], field=f"{field}.frame"),
        )

    def to_mapping(self) -> dict[str, object]:
        return {"instance": self.instance, "frame": self.frame}


@dataclass(frozen=True, slots=True)
class AttachmentSpec:
    """Directed rigid attachment from a parent canonical frame to a child frame."""

    attachment_id: str
    parent: AttachmentEndpointSpec
    child: AttachmentEndpointSpec
    transform: PoseSpec
    assumption: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "attachment") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"attachment_id", "parent", "child", "transform", "assumption"}
            ),
            field=field,
        )
        return cls(
            attachment_id=validate_identifier(
                data["attachment_id"], field=f"{field}.attachment_id"
            ),
            parent=AttachmentEndpointSpec.from_mapping(
                data["parent"], field=f"{field}.parent"
            ),
            child=AttachmentEndpointSpec.from_mapping(
                data["child"], field=f"{field}.child"
            ),
            transform=PoseSpec.from_mapping(
                data["transform"], field=f"{field}.transform"
            ),
            assumption=require_string(
                data["assumption"], field=f"{field}.assumption"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "attachment_id": self.attachment_id,
            "parent": self.parent.to_mapping(),
            "child": self.child.to_mapping(),
            "transform": self.transform.to_mapping(),
            "assumption": self.assumption,
        }


def _validate_forest(
    instance_ids: tuple[str, ...],
    roots: tuple[str, ...],
    attachments: tuple[AttachmentSpec, ...],
    *,
    field: str,
) -> None:
    known = set(instance_ids)
    parent_by_child: dict[str, str] = {}
    children_by_parent: dict[str, list[str]] = {
        instance_id: [] for instance_id in instance_ids
    }
    for attachment in attachments:
        parent = attachment.parent.instance
        child = attachment.child.instance
        if parent not in known:
            raise ValueError(
                f"{field}.attachments references unknown parent instance {parent!r}"
            )
        if child not in known:
            raise ValueError(
                f"{field}.attachments references unknown child instance {child!r}"
            )
        if child in parent_by_child:
            raise ValueError(
                f"{field}.attachments gives instance {child!r} multiple parents"
            )
        parent_by_child[child] = parent
        children_by_parent[parent].append(child)

    state: dict[str, int] = {}

    def visit(instance_id: str) -> None:
        marker = state.get(instance_id, 0)
        if marker == 1:
            raise ValueError(f"{field}.attachments must form an acyclic forest")
        if marker == 2:
            return
        state[instance_id] = 1
        for child_id in children_by_parent[instance_id]:
            visit(child_id)
        state[instance_id] = 2

    for instance_id in instance_ids:
        visit(instance_id)

    computed_roots = known - set(parent_by_child)
    if set(roots) != computed_roots:
        raise ValueError(
            f"{field}.roots must exactly name the forest roots; "
            f"expected={sorted(computed_roots)}"
        )


@dataclass(frozen=True, slots=True)
class AssemblySpec:
    """A single- or multi-root forest of backend-neutral asset instances."""

    schema: str
    assembly_id: str
    instances: tuple[AssetInstanceSpec, ...]
    roots: tuple[str, ...]
    attachments: tuple[AttachmentSpec, ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "assembly") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"schema", "assembly_id", "instances", "roots", "attachments"}
            ),
            field=field,
        )
        schema = require_string(data["schema"], field=f"{field}.schema")
        if schema != ASSEMBLY_SCHEMA:
            raise ValueError(f"{field}.schema must be {ASSEMBLY_SCHEMA!r}")
        instances = tuple(
            AssetInstanceSpec.from_mapping(item, field=f"{field}.instances[{index}]")
            for index, item in enumerate(
                require_sequence(data["instances"], field=f"{field}.instances")
            )
        )
        if not instances:
            raise ValueError(f"{field}.instances must not be empty")
        instance_ids = tuple(instance.instance_id for instance in instances)
        if len(set(instance_ids)) != len(instance_ids):
            raise ValueError(f"{field}.instances instance_id values must be unique")
        namespaces = tuple(instance.namespace for instance in instances)
        if len(set(namespaces)) != len(namespaces):
            raise ValueError(f"{field}.instances namespace values must be unique")
        roots = tuple(
            validate_identifier(item, field=f"{field}.roots[{index}]")
            for index, item in enumerate(
                require_sequence(data["roots"], field=f"{field}.roots")
            )
        )
        if not roots:
            raise ValueError(f"{field}.roots must not be empty")
        if len(set(roots)) != len(roots):
            raise ValueError(f"{field}.roots values must be unique")
        attachments = tuple(
            AttachmentSpec.from_mapping(
                item, field=f"{field}.attachments[{index}]"
            )
            for index, item in enumerate(
                require_sequence(data["attachments"], field=f"{field}.attachments")
            )
        )
        attachment_ids = tuple(
            attachment.attachment_id for attachment in attachments
        )
        if len(set(attachment_ids)) != len(attachment_ids):
            raise ValueError(
                f"{field}.attachments attachment_id values must be unique"
            )
        _validate_forest(instance_ids, roots, attachments, field=field)
        return cls(
            schema=schema,
            assembly_id=validate_identifier(
                data["assembly_id"], field=f"{field}.assembly_id"
            ),
            instances=instances,
            roots=roots,
            attachments=attachments,
        )

    def instance(self, instance_id: str) -> AssetInstanceSpec:
        for instance in self.instances:
            if instance.instance_id == instance_id:
                return instance
        raise KeyError(instance_id)

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "assembly_id": self.assembly_id,
            "instances": [instance.to_mapping() for instance in self.instances],
            "roots": list(self.roots),
            "attachments": [
                attachment.to_mapping() for attachment in self.attachments
            ],
        }


__all__ = [
    "ASSEMBLY_SCHEMA",
    "AssemblySpec",
    "AssetInstanceSpec",
    "AttachmentEndpointSpec",
    "AttachmentSpec",
]
