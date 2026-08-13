"""Fail-closed preflight for the dual NERO + Hand2 8.3 recording chain."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Self

from wujihand.domain import HandSide
from wujihand.specs.common import (
    ConfigRef,
    PoseSpec,
    require_exact_mapping,
    require_sequence,
    validate_identifier,
    validate_project_reference,
)

from .config_repository import ConfigRepository
from .isaac_workcell_plan import (
    ResolvedIsaacWorkcellPlan,
    resolve_isaac_workcell_plan,
)
from .ros_deployment_resolver import ResolvedRosDeployment, RosDeploymentResolver
from .source_lock import sha256_file
from .wuji_hand2_matched_chain import (
    MatchedChainPreflightReceipt,
    WujiSdkRuntimeFacts,
    WujiSdkUserManager,
    load_matched_chain_qualification_policy,
    preflight_wuji_hand2_matched_chain,
)
from .yaml_loader import load_yaml_strict


QUALIFICATION_SCHEMA = "wujihand.wuji_hand2_record_chain_qualification.v1"
PREFLIGHT_RECEIPT_SCHEMA = "wujihand.wuji_hand2_record_chain_preflight.v1"


@dataclass(frozen=True, slots=True)
class RecordChainDescriptionPolicy:
    release: str
    asset_revision: str
    roots: tuple[tuple[HandSide, str], ...]
    root_orientation_compensations: tuple[tuple[HandSide, tuple[float, float, float, float]], ...]

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "release",
                    "asset_revision",
                    "roots",
                    "root_orientation_compensation_quat_wxyz",
                }
            ),
            field=field,
        )
        raw_roots = require_exact_mapping(
            data["roots"],
            expected=frozenset({"left", "right"}),
            field=f"{field}.roots",
        )
        raw_compensations = require_exact_mapping(
            data["root_orientation_compensation_quat_wxyz"],
            expected=frozenset({"left", "right"}),
            field=f"{field}.root_orientation_compensation_quat_wxyz",
        )
        return cls(
            release=validate_identifier(data["release"], field=f"{field}.release"),
            asset_revision=validate_identifier(
                data["asset_revision"], field=f"{field}.asset_revision"
            ),
            roots=tuple(
                (
                    side,
                    validate_identifier(raw_roots[side.value], field=f"{field}.roots.{side.value}"),
                )
                for side in HandSide
            ),
            root_orientation_compensations=tuple(
                (
                    side,
                    PoseSpec.from_mapping(
                        {
                            "position_m": [0.0, 0.0, 0.0],
                            "quat_wxyz": raw_compensations[side.value],
                        },
                        field=(f"{field}.root_orientation_compensation_quat_wxyz.{side.value}"),
                    ).quat_wxyz,
                )
                for side in HandSide
            ),
        )

    def root(self, side: HandSide) -> str:
        for candidate, root in self.roots:
            if candidate is side:
                return root
        raise KeyError(side)

    def root_orientation_compensation(self, side: HandSide) -> tuple[float, float, float, float]:
        for candidate, quaternion in self.root_orientation_compensations:
            if candidate is side:
                return quaternion
        raise KeyError(side)


@dataclass(frozen=True, slots=True)
class RecordChainNeroPolicy:
    asset_id: str
    binding_id: str
    profile_id: str
    attachment: PoseSpec
    parent_frame: str
    child_frame: str
    assembly_attachment_quaternions: (
        tuple[tuple[HandSide, tuple[float, float, float, float]], ...] | None
    )

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        optional_key = "assembly_attachment_quat_wxyz_by_side"
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"asset_id", "binding_id", "profile_id", "attachment"}
                | (
                    {optional_key}
                    if isinstance(value, Mapping) and optional_key in value
                    else set()
                )
            ),
            field=field,
        )
        attachment = require_exact_mapping(
            data["attachment"],
            expected=frozenset({"parent_frame", "child_frame", "position_m", "quat_wxyz"}),
            field=f"{field}.attachment",
        )
        raw_quaternions = (
            None
            if optional_key not in data
            else require_exact_mapping(
                data[optional_key],
                expected=frozenset({"left", "right"}),
                field=f"{field}.{optional_key}",
            )
        )
        return cls(
            asset_id=validate_identifier(data["asset_id"], field=f"{field}.asset_id"),
            binding_id=validate_identifier(data["binding_id"], field=f"{field}.binding_id"),
            profile_id=validate_identifier(data["profile_id"], field=f"{field}.profile_id"),
            attachment=PoseSpec.from_mapping(
                {
                    "position_m": attachment["position_m"],
                    "quat_wxyz": attachment["quat_wxyz"],
                },
                field=f"{field}.attachment.transform",
            ),
            parent_frame=validate_identifier(
                attachment["parent_frame"], field=f"{field}.attachment.parent_frame"
            ),
            child_frame=validate_identifier(
                attachment["child_frame"], field=f"{field}.attachment.child_frame"
            ),
            assembly_attachment_quaternions=(
                None
                if raw_quaternions is None
                else tuple(
                    (
                        side,
                        PoseSpec.from_mapping(
                            {
                                "position_m": [0.0, 0.0, 0.0],
                                "quat_wxyz": raw_quaternions[side.value],
                            },
                            field=f"{field}.{optional_key}.{side.value}",
                        ).quat_wxyz,
                    )
                    for side in HandSide
                )
            ),
        )

    def assembly_attachment_quaternion(
        self, side: HandSide
    ) -> tuple[float, float, float, float] | None:
        if self.assembly_attachment_quaternions is None:
            return None
        for candidate, quaternion in self.assembly_attachment_quaternions:
            if candidate is side:
                return quaternion
        raise KeyError(side)


@dataclass(frozen=True, slots=True)
class RecordChainQualificationPolicy:
    qualification_id: str
    matched_chain: ConfigRef
    deployment: ConfigRef
    assembly: ConfigRef
    task_scene: ConfigRef | None
    description: RecordChainDescriptionPolicy
    nero: RecordChainNeroPolicy
    required_sdk_processes: tuple[str, ...]

    @classmethod
    def from_mapping(cls, value: object) -> Self:
        optional_key = "task_scene"
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "schema",
                    "qualification_id",
                    "matched_chain",
                    "deployment",
                    "assembly",
                    "description",
                    "nero",
                    "required_sdk_processes",
                }
                | (
                    {optional_key}
                    if isinstance(value, Mapping) and optional_key in value
                    else set()
                )
            ),
            field="record_chain_qualification",
        )
        if data["schema"] != QUALIFICATION_SCHEMA:
            raise ValueError(f"record_chain_qualification.schema must be {QUALIFICATION_SCHEMA!r}")
        processes = tuple(
            validate_identifier(
                item, field=f"record_chain_qualification.required_sdk_processes[{index}]"
            )
            for index, item in enumerate(
                require_sequence(
                    data["required_sdk_processes"],
                    field="record_chain_qualification.required_sdk_processes",
                )
            )
        )
        if processes != ("glove_source", "isaac_consumer"):
            raise ValueError(
                "record chain requires exactly glove_source and isaac_consumer SDK runtimes"
            )
        return cls(
            qualification_id=validate_identifier(
                data["qualification_id"],
                field="record_chain_qualification.qualification_id",
            ),
            matched_chain=ConfigRef.from_mapping(
                data["matched_chain"], field="record_chain_qualification.matched_chain"
            ),
            deployment=ConfigRef.from_mapping(
                data["deployment"], field="record_chain_qualification.deployment"
            ),
            assembly=ConfigRef.from_mapping(
                data["assembly"], field="record_chain_qualification.assembly"
            ),
            task_scene=(
                None
                if optional_key not in data
                else ConfigRef.from_mapping(
                    data[optional_key],
                    field="record_chain_qualification.task_scene",
                )
            ),
            description=RecordChainDescriptionPolicy.from_mapping(
                data["description"], field="record_chain_qualification.description"
            ),
            nero=RecordChainNeroPolicy.from_mapping(
                data["nero"], field="record_chain_qualification.nero"
            ),
            required_sdk_processes=processes,
        )


@dataclass(frozen=True, slots=True)
class RecordChainProcessReceipt:
    process_id: str
    environment_id: str
    executable: Path
    resolved_executable: Path
    sdk_version: str
    sdk_module_path: Path

    def to_mapping(self) -> dict[str, object]:
        return {
            "process_id": self.process_id,
            "environment_id": self.environment_id,
            "executable": str(self.executable),
            "resolved_executable": str(self.resolved_executable),
            "sdk_version": self.sdk_version,
            "sdk_module_path": str(self.sdk_module_path),
        }


@dataclass(frozen=True, slots=True)
class RecordChainTaskSceneReceipt:
    path: str
    profile_id: str
    sha256: str

    def to_mapping(self) -> dict[str, object]:
        return {
            "path": self.path,
            "profile_id": self.profile_id,
            "sha256": self.sha256,
        }


@dataclass(frozen=True, slots=True)
class RecordChainPreflightReceipt:
    qualification_id: str
    input_mode: str
    deployment_path: str
    deployment_id: str
    deployment_hash: str
    local_binding_hash: str
    session_id: str
    session_hash: str
    assembly_path: str
    assembly_id: str
    assembly_sha256: str
    task_scene: RecordChainTaskSceneReceipt | None
    dataset_profile_id: str
    q54_profile_id: str
    dataset_source_mode: str
    description_release: str
    description_root_orientation_compensations: tuple[
        tuple[HandSide, tuple[float, float, float, float]], ...
    ]
    process_receipts: tuple[RecordChainProcessReceipt, ...]
    side_receipts: tuple[MatchedChainPreflightReceipt, ...]

    def to_mapping(self) -> dict[str, object]:
        dataset_eligible = self.dataset_source_mode == "live_teleoperation"
        return {
            "schema": PREFLIGHT_RECEIPT_SCHEMA,
            "passed": True,
            "input_mode": self.input_mode,
            "device_access_attempted": False,
            "isaac_started": False,
            "qualification_id": self.qualification_id,
            "deployment": {
                "path": self.deployment_path,
                "deployment_id": self.deployment_id,
                "deployment_hash": self.deployment_hash,
                "local_binding_hash": self.local_binding_hash,
                "session_id": self.session_id,
                "session_hash": self.session_hash,
                "assembly_path": self.assembly_path,
                "assembly_id": self.assembly_id,
                "assembly_sha256": self.assembly_sha256,
            },
            "task_scene": (None if self.task_scene is None else self.task_scene.to_mapping()),
            "dataset": {
                "profile_id": self.dataset_profile_id,
                "q54_profile_id": self.q54_profile_id,
                "source_mode": self.dataset_source_mode,
                "qualification_only": not dataset_eligible,
                "dataset_eligible": dataset_eligible,
            },
            "description": {
                "release": self.description_release,
                "root_orientation_compensation_quat_wxyz": {
                    side.value: list(quaternion)
                    for side, quaternion in (self.description_root_orientation_compensations)
                },
                "beta_warning": (
                    "Wuji Hand2 remains Beta1; any SDK, Description, Studio user-model, "
                    "or calibrated URDF change invalidates this receipt."
                ),
            },
            "sdk_processes": [item.to_mapping() for item in self.process_receipts],
            "hands": {item.side.value: item.to_mapping() for item in self.side_receipts},
            "scope": ("simulation-only dual NERO + dual Hand2; no NERO or Hand2 hardware access"),
        }


def load_record_chain_qualification_policy(
    path: str | Path,
) -> RecordChainQualificationPolicy:
    return RecordChainQualificationPolicy.from_mapping(
        load_yaml_strict(Path(path).read_text(encoding="utf-8"))
    )


def preflight_wuji_hand2_record_chain(
    project_root: str | Path,
    *,
    qualification_path: str | Path,
    deployment_path: str | Path,
    local_runtime_binding_path: str | Path,
    matched_chain_binding_path: str | Path,
    input_mode: str,
    dataset_source_mode: str | None = None,
    sdk_runtime: WujiSdkRuntimeFacts,
    user_manager: WujiSdkUserManager,
    studio_processes: Sequence[str] = (),
    home_dir: str | Path | None = None,
    verify_artifacts: bool = True,
) -> RecordChainPreflightReceipt:
    """Close the complete 8.3 recording identity without opening devices or Isaac."""

    if input_mode not in {"stub", "glove"}:
        raise ValueError("input_mode must be 'stub' or 'glove'")
    if dataset_source_mode is None:
        dataset_source_mode = "synthetic_fixture" if input_mode == "stub" else "live_qualification"
    if dataset_source_mode not in {
        "synthetic_fixture",
        "live_qualification",
        "live_teleoperation",
    }:
        raise ValueError("unsupported dataset_source_mode")
    if (input_mode == "stub") != (dataset_source_mode == "synthetic_fixture"):
        raise ValueError("stub input and synthetic_fixture source mode must be paired")
    del verify_artifacts
    root = Path(project_root).resolve()
    policy = load_record_chain_qualification_policy(qualification_path)
    requested_deployment = _project_relative(root, deployment_path, field="record chain deployment")
    if requested_deployment != policy.deployment.path:
        raise RuntimeError("record chain deployment differs from the qualification policy")

    matched_policy_path = root / policy.matched_chain.path
    matched_policy = load_matched_chain_qualification_policy(matched_policy_path)
    if matched_policy.qualification_id != policy.matched_chain.expected_id:
        raise RuntimeError("matched-chain qualification identity differs")
    if (
        matched_policy.description.release != policy.description.release
        or matched_policy.description.asset_revision != policy.description.asset_revision
    ):
        raise RuntimeError("record and matched-chain Description versions differ")

    resolved = RosDeploymentResolver(root).resolve(
        requested_deployment,
        local_binding=local_runtime_binding_path,
        verify_artifacts=False,
    )
    if resolved.deployment.deployment_id != policy.deployment.expected_id:
        raise RuntimeError("record chain deployment ID differs")
    if (
        resolved.session.assembly_path != policy.assembly.path
        or resolved.session.assembly.assembly_id != policy.assembly.expected_id
    ):
        raise RuntimeError("record chain assembly identity differs")
    if resolved.session.session.runtime_role != "teleop_consumer":
        raise RuntimeError("record chain Session must be the teleoperation consumer")
    dataset_reference = resolved.session.session.dataset_profile
    if dataset_reference is None:
        raise RuntimeError("record chain Session must pin a mini dataset profile")
    from wujihand.dataset.profile import load_mini_dataset_profile

    dataset_profile = load_mini_dataset_profile(root, dataset_reference.path)
    if dataset_profile.profile_id != dataset_reference.expected_id:
        raise RuntimeError("record chain mini dataset profile identity differs")

    processes: list[RecordChainProcessReceipt] = []
    for process_id in policy.required_sdk_processes:
        process = resolved.local_binding.process(process_id)
        executable = Path(process.executable)
        resolved_executable = executable.resolve()
        if resolved_executable != sdk_runtime.executable_path.resolve():
            raise RuntimeError(f"{process_id} does not use the qualified SDK interpreter")
        processes.append(
            RecordChainProcessReceipt(
                process_id=process_id,
                environment_id=process.environment_id,
                executable=executable,
                resolved_executable=resolved_executable,
                sdk_version=sdk_runtime.module_version,
                sdk_module_path=sdk_runtime.module_path,
            )
        )

    side_receipts = tuple(
        preflight_wuji_hand2_matched_chain(
            root,
            qualification_path=matched_policy_path,
            local_binding_path=matched_chain_binding_path,
            side=side,
            input_mode=input_mode,
            sdk_runtime=sdk_runtime,
            user_manager=user_manager,
            studio_processes=studio_processes,
            home_dir=home_dir,
            verify_artifacts=False,
        )
        for side in HandSide
    )
    _validate_local_glove_bindings(resolved, side_receipts)
    _validate_description_and_nero(root, policy, resolved)
    task_scene = _resolve_task_scene_identity(root, policy, resolved)

    return RecordChainPreflightReceipt(
        qualification_id=policy.qualification_id,
        input_mode=input_mode,
        deployment_path=requested_deployment,
        deployment_id=resolved.deployment.deployment_id,
        deployment_hash=resolved.deployment_hash,
        local_binding_hash=resolved.local_binding_hash,
        session_id=resolved.session.session.session_id,
        session_hash=resolved.session.session_hash,
        assembly_path=resolved.session.assembly_path,
        assembly_id=resolved.session.assembly.assembly_id,
        assembly_sha256=sha256_file(root / resolved.session.assembly_path),
        task_scene=task_scene,
        dataset_profile_id=dataset_profile.profile_id,
        q54_profile_id=dataset_profile.q54.profile_id,
        dataset_source_mode=dataset_source_mode,
        description_release=policy.description.release,
        description_root_orientation_compensations=(
            policy.description.root_orientation_compensations
        ),
        process_receipts=tuple(processes),
        side_receipts=side_receipts,
    )


def _project_relative(root: Path, value: str | Path, *, field: str) -> str:
    raw = Path(value)
    path = raw.resolve() if raw.is_absolute() else (root / raw).resolve()
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes project root") from exc
    return validate_project_reference(relative.as_posix(), field=field)


def _validate_local_glove_bindings(
    resolved: ResolvedRosDeployment,
    side_receipts: tuple[MatchedChainPreflightReceipt, ...],
) -> None:
    by_side = {item.side: item for item in side_receipts}
    for side in HandSide:
        route = resolved.route_plan.route(f"hand_{side.value}", "finger_joints")
        if route.local_binding is None:
            raise RuntimeError(f"{side.value} Glove local binding is missing")
        receipt = by_side[side]
        if route.local_binding.device_identity != receipt.serial_number:
            raise RuntimeError(f"{side.value} Glove serial differs from Studio calibration")
        if route.local_binding.calibration_id != receipt.calibration_id:
            raise RuntimeError(
                f"{side.value} Glove calibration_id does not pin the Studio user URDF"
            )


def _validate_description_and_nero(
    root: Path,
    policy: RecordChainQualificationPolicy,
    resolved: ResolvedRosDeployment,
) -> None:
    for side in HandSide:
        hand_id = f"hand_{side.value}"
        arm_id = f"nero_{side.value}"
        hand = resolved.session.instance(hand_id)
        arm = resolved.session.instance(arm_id)
        if (
            hand.asset.product != "wuji_hand_2"
            or hand.asset.side != side.value
            or hand.asset.revision != policy.description.asset_revision
            or hand.binding.root != policy.description.root(side)
            or hand.artifact is None
        ):
            raise RuntimeError(f"{side.value} Hand2 8.3 identity/root differs")
        description_sources = (hand.artifact, *hand.resource_trees)
        if not any(
            dict(resource.source.revision).get("tag") == policy.description.release
            for resource in description_sources
        ):
            raise RuntimeError(f"{side.value} Hand2 Description version differs")
        if (
            arm.asset.asset_id != policy.nero.asset_id
            or arm.binding.binding_id != policy.nero.binding_id
            or arm.asset.canonical_profile is None
        ):
            raise RuntimeError(f"{side.value} NERO pinned model/binding differs")
        arm_profile_path = arm.asset.canonical_profile
        profile = load_yaml_strict((root / arm_profile_path).read_text(encoding="utf-8"))
        if not isinstance(profile, Mapping) or profile.get("profile_id") != policy.nero.profile_id:
            raise RuntimeError(f"{side.value} NERO provisional simulation profile differs")
        attachments = tuple(
            item
            for item in resolved.session.assembly.attachments
            if item.parent.instance == arm_id and item.child.instance == hand_id
        )
        if len(attachments) != 1:
            raise RuntimeError(f"{side.value} NERO-to-Hand2 attachment is not unique")
        attachment = attachments[0]
        expected_quaternion = policy.nero.assembly_attachment_quaternion(side)
        if expected_quaternion is None:
            expected_quaternion = _quaternion_product(
                policy.nero.attachment.quat_wxyz,
                policy.description.root_orientation_compensation(side),
            )
        if (
            attachment.parent.frame != policy.nero.parent_frame
            or attachment.child.frame != policy.nero.child_frame
            or attachment.transform.position_m != policy.nero.attachment.position_m
            or not _quaternions_equivalent(attachment.transform.quat_wxyz, expected_quaternion)
        ):
            raise RuntimeError(f"{side.value} NERO-to-Hand2 attachment differs")


def _resolve_task_scene_identity(
    root: Path,
    policy: RecordChainQualificationPolicy,
    resolved: ResolvedRosDeployment,
) -> RecordChainTaskSceneReceipt | None:
    if policy.task_scene is None:
        return None
    repository = ConfigRepository(root)
    path = repository.resolve_project_path(
        policy.task_scene.path,
        field="record chain task scene",
    )
    profile = repository.load_isaac_task_scene_profile(path)
    if profile.profile_id != policy.task_scene.expected_id:
        raise RuntimeError("record chain task-scene identity differs")
    plan = resolve_isaac_workcell_plan(
        root,
        resolved.session.workcell,
        task_scene=policy.task_scene.path,
        verify_content=False,
    )
    if plan.task_scene_profile_id != profile.profile_id:
        raise RuntimeError("record chain task scene does not compose with the Workcell")
    return RecordChainTaskSceneReceipt(
        path=policy.task_scene.path,
        profile_id=profile.profile_id,
        sha256=sha256_file(path),
    )


def load_record_chain_preflight_receipt(path: str | Path) -> Mapping[str, object]:
    import json

    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"record-chain preflight receipt is unreadable: {exc}") from exc
    dataset = document.get("dataset") if isinstance(document, Mapping) else None
    source_mode = dataset.get("source_mode") if isinstance(dataset, Mapping) else None
    dataset_eligible = source_mode == "live_teleoperation"
    if (
        not isinstance(document, Mapping)
        or document.get("schema") != PREFLIGHT_RECEIPT_SCHEMA
        or document.get("passed") is not True
        or document.get("device_access_attempted") is not False
        or document.get("isaac_started") is not False
        or not isinstance(dataset, Mapping)
        or source_mode not in {"synthetic_fixture", "live_qualification", "live_teleoperation"}
        or dataset.get("qualification_only") is not (not dataset_eligible)
        or dataset.get("dataset_eligible") is not dataset_eligible
    ):
        raise ValueError("record-chain preflight receipt is not a passed offline receipt")
    return document


def resolve_record_chain_workcell_plan(
    project_root: str | Path,
    resolved: ResolvedRosDeployment,
    receipt: Mapping[str, object],
    *,
    verify_content: bool,
) -> ResolvedIsaacWorkcellPlan | None:
    deployment = receipt.get("deployment")
    if (
        not isinstance(deployment, Mapping)
        or deployment.get("deployment_id") != resolved.deployment.deployment_id
        or deployment.get("deployment_hash") != resolved.deployment_hash
        or deployment.get("local_binding_hash") != resolved.local_binding_hash
        or deployment.get("session_id") != resolved.session.session.session_id
        or deployment.get("session_hash") != resolved.session.session_hash
        or deployment.get("assembly_id") != resolved.session.assembly.assembly_id
    ):
        raise ValueError("record-chain preflight receipt does not close this runtime")
    raw_task_scene = receipt.get("task_scene")
    if raw_task_scene is None:
        return None
    task_scene = require_exact_mapping(
        raw_task_scene,
        expected=frozenset({"path", "profile_id", "sha256"}),
        field="record-chain preflight task_scene",
    )
    path = validate_project_reference(
        task_scene["path"],
        field="record-chain preflight task_scene.path",
    )
    profile_id = validate_identifier(
        task_scene["profile_id"],
        field="record-chain preflight task_scene.profile_id",
    )
    scene_path = Path(project_root).resolve() / path
    if sha256_file(scene_path) != task_scene["sha256"]:
        raise ValueError("record-chain task-scene configuration changed after preflight")
    plan = resolve_isaac_workcell_plan(
        project_root,
        resolved.session.workcell,
        task_scene=path,
        verify_content=verify_content,
    )
    if plan.task_scene_profile_id != profile_id or plan.task_scene_profile_path != path:
        raise ValueError("record-chain task-scene identity differs from the resolved plan")
    return plan


def _quaternion_product(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    raw = (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
    norm = math.sqrt(sum(value * value for value in raw))
    return tuple(value / norm for value in raw)  # type: ignore[return-value]


def _quaternions_equivalent(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
    *,
    atol: float = 1e-12,
) -> bool:
    direct = max(abs(lhs - rhs) for lhs, rhs in zip(left, right, strict=True))
    negated = max(abs(lhs + rhs) for lhs, rhs in zip(left, right, strict=True))
    return min(direct, negated) <= atol


__all__ = [
    "PREFLIGHT_RECEIPT_SCHEMA",
    "QUALIFICATION_SCHEMA",
    "RecordChainPreflightReceipt",
    "RecordChainQualificationPolicy",
    "RecordChainTaskSceneReceipt",
    "load_record_chain_preflight_receipt",
    "load_record_chain_qualification_policy",
    "preflight_wuji_hand2_record_chain",
    "resolve_record_chain_workcell_plan",
]
