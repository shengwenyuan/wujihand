"""Filesystem boundary for strict project configuration documents.

The spec package deliberately has no YAML or filesystem dependency.  This
repository is the single place that turns a project-relative reference into a
typed immutable specification.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import TypeVar, cast

from wujihand.specs import (
    AssemblySpec,
    AssetManifest,
    BackendBinding,
    ConfigRef,
    DeploymentSpec,
    DualTeleoperationProfile,
    LocalDeviceBindingSpec,
    NativeDualTeleoperationProfile,
    RosDeploymentSpec,
    RosLocalRuntimeBindingSpec,
    RosQosProfileSpec,
    SessionSpec,
    WorkcellSpec,
)

from .yaml_loader import load_yaml_strict


SpecT = TypeVar(
    "SpecT",
    AssetManifest,
    BackendBinding,
    AssemblySpec,
    WorkcellSpec,
    SessionSpec,
    DeploymentSpec,
    DualTeleoperationProfile,
    LocalDeviceBindingSpec,
    NativeDualTeleoperationProfile,
    RosDeploymentSpec,
    RosLocalRuntimeBindingSpec,
    RosQosProfileSpec,
)


class ConfigRepository:
    """Load typed project YAML inside one project root and fail closed on escapes."""

    def __init__(self, project_root: str | Path) -> None:
        root = Path(project_root).resolve()
        if not root.is_dir():
            raise ValueError(f"project root is not a directory: {root}")
        self._project_root = root

    @property
    def project_root(self) -> Path:
        return self._project_root

    def resolve_project_path(
        self,
        reference: str | Path,
        *,
        field: str,
        must_exist: bool = True,
        expect_directory: bool = False,
    ) -> Path:
        """Resolve a CLI or internal path while keeping it under the project root."""

        raw = Path(reference)
        candidate = raw if raw.is_absolute() else self._project_root / raw
        resolved = candidate.resolve()
        try:
            resolved.relative_to(self._project_root)
        except ValueError as exc:
            raise ValueError(f"{field} escapes the project root: {reference}") from exc
        if must_exist:
            if expect_directory and not resolved.is_dir():
                raise FileNotFoundError(f"{field} directory not found: {resolved}")
            if not expect_directory and not resolved.is_file():
                raise FileNotFoundError(f"{field} file not found: {resolved}")
        return resolved

    def project_relative(self, path: str | Path, *, field: str) -> str:
        """Return a stable POSIX project-relative path for hashing and reports."""

        resolved = self.resolve_project_path(path, field=field, must_exist=False)
        return resolved.relative_to(self._project_root).as_posix()

    def load_asset(self, reference: ConfigRef | str | Path) -> AssetManifest:
        return self._load(
            reference,
            field="asset",
            spec_type=AssetManifest,
            id_attribute="asset_id",
        )

    def load_binding(self, reference: ConfigRef | str | Path) -> BackendBinding:
        return self._load(
            reference,
            field="binding",
            spec_type=BackendBinding,
            id_attribute="binding_id",
        )

    def load_assembly(self, reference: ConfigRef | str | Path) -> AssemblySpec:
        return self._load(
            reference,
            field="assembly",
            spec_type=AssemblySpec,
            id_attribute="assembly_id",
        )

    def load_workcell(self, reference: ConfigRef | str | Path) -> WorkcellSpec:
        return self._load(
            reference,
            field="workcell",
            spec_type=WorkcellSpec,
            id_attribute="workcell_id",
        )

    def load_session(self, reference: ConfigRef | str | Path) -> SessionSpec:
        return self._load(
            reference,
            field="session",
            spec_type=SessionSpec,
            id_attribute="session_id",
        )

    def load_deployment(self, reference: ConfigRef | str | Path) -> DeploymentSpec:
        return self._load(
            reference,
            field="deployment",
            spec_type=DeploymentSpec,
            id_attribute="deployment_id",
        )

    def load_local_device_binding(
        self,
        reference: ConfigRef | str | Path,
    ) -> LocalDeviceBindingSpec:
        return self._load(
            reference,
            field="local device binding",
            spec_type=LocalDeviceBindingSpec,
            id_attribute="binding_id",
        )

    def load_native_dual_teleoperation_profile(
        self,
        reference: ConfigRef | str | Path,
    ) -> NativeDualTeleoperationProfile:
        return self._load(
            reference,
            field="native dual teleoperation profile",
            spec_type=NativeDualTeleoperationProfile,
            id_attribute="profile_id",
        )

    def load_dual_teleoperation_profile(
        self,
        reference: ConfigRef | str | Path,
    ) -> DualTeleoperationProfile:
        return self._load(
            reference,
            field="dual teleoperation profile",
            spec_type=DualTeleoperationProfile,
            id_attribute="profile_id",
        )

    def load_ros_deployment(
        self,
        reference: ConfigRef | str | Path,
    ) -> RosDeploymentSpec:
        return self._load(
            reference,
            field="ROS deployment",
            spec_type=RosDeploymentSpec,
            id_attribute="deployment_id",
        )

    def load_ros_qos_profile(
        self,
        reference: ConfigRef | str | Path,
    ) -> RosQosProfileSpec:
        return self._load(
            reference,
            field="ROS QoS profile",
            spec_type=RosQosProfileSpec,
            id_attribute="profile_id",
        )

    def load_ros_local_runtime_binding(
        self,
        reference: ConfigRef | str | Path,
    ) -> RosLocalRuntimeBindingSpec:
        return self._load(
            reference,
            field="ROS local runtime binding",
            spec_type=RosLocalRuntimeBindingSpec,
            id_attribute="binding_id",
        )

    def validate_profile_reference(self, reference: ConfigRef) -> str:
        """Validate a generic profile ID and return its project-relative path."""

        path = self.resolve_project_path(
            reference.path,
            field="referenced compatibility profile",
        )
        document = load_yaml_strict(path.read_text(encoding="utf-8"))
        if not isinstance(document, Mapping):
            raise ValueError(
                f"compatibility profile must contain a mapping: {path}"
            )
        actual_id = document.get("profile_id")
        if actual_id != reference.expected_id:
            raise ValueError(
                "compatibility profile reference expected "
                f"{reference.expected_id!r}, loaded {actual_id!r}"
            )
        return self.project_relative(
            path,
            field="referenced compatibility profile",
        )

    def _load(
        self,
        reference: ConfigRef | str | Path,
        *,
        field: str,
        spec_type: type[SpecT],
        id_attribute: str,
    ) -> SpecT:
        expected_id: str | None
        if isinstance(reference, ConfigRef):
            source_path: str | Path = reference.path
            expected_id = reference.expected_id
        else:
            source_path = reference
            expected_id = None
        path = self.resolve_project_path(source_path, field=f"{field} config")
        document = load_yaml_strict(path.read_text(encoding="utf-8"))
        if not isinstance(document, Mapping):
            raise ValueError(f"{field} config must contain a mapping: {path}")
        mapping = cast(Mapping[str, object], document)
        value = spec_type.from_mapping(mapping, field=field)
        actual_id = cast(str, getattr(value, id_attribute))
        if expected_id is not None and actual_id != expected_id:
            raise ValueError(
                f"{field} reference expected {expected_id!r}, loaded {actual_id!r} "
                f"from {self.project_relative(path, field=f'{field} config')}"
            )
        return value


__all__ = ["ConfigRepository"]
