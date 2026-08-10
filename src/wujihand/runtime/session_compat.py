"""Compatibility bridges from resolved Sessions to current specialized runners."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Collection, cast

import yaml

from wujihand.specs import EntitySpec, PoseSpec, WorkcellFrameSpec

from .mujoco_table_config import (
    MujocoTableSceneConfig,
    load_mujoco_table_scene_config,
)
from .rotation_ball_config import RotationBallConfig, load_rotation_ball_config
from .session_resolver import ResolvedInstance, ResolvedSession, SessionResolver
from .source_lock import ResolvedArtifact


MUJOCO_TABLE_SESSION = Path(
    "configs/sessions/mujoco_fr3v2_hand2_right_table_v1.yaml"
)
ISAAC_FIXED_PREVIEW_SESSION = Path(
    "configs/sessions/isaac_hand2_right_fixed_qualification_v2026_6_27_v1.yaml"
)
ISAAC_FIXED_TELEOP_SESSION = Path("configs/sessions/isaac_hand2_teleop_v1.yaml")
ISAAC_ROTATION_QUALIFICATION_SESSION = Path(
    "configs/sessions/isaac_hand2_right_rotation_ball_qualification_v1.yaml"
)
ISAAC_ROTATION_TELEOP_SESSION = Path(
    "configs/sessions/isaac_hand2_right_rotation_ball_teleop_v1.yaml"
)
MEDIAPIPE_Q20_SESSION = Path(
    "configs/sessions/mediapipe_hand2_q20_udp_v1.yaml"
)
MEDIAPIPE_HAND_COMMAND_SESSION = Path(
    "configs/sessions/mediapipe_hand2_hand_command_udp_v1.yaml"
)

_HAND_ASSET_PATH = (
    "configs/assets/wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
)
_FR3_ASSET_PATH = "configs/assets/franka_fr3_v2_v1.yaml"
_WRIST_ASSET_PATH = "configs/assets/fixed_xyz_wrist3_v1.yaml"
_HAND_ISAAC_BINDING_PATH = (
    "configs/bindings/isaac/wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
)
_HAND_MUJOCO_BINDING_PATH = (
    "configs/bindings/mujoco/wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
)
_FR3_MUJOCO_BINDING_PATH = (
    "configs/bindings/mujoco/franka_fr3_v2_menagerie_71f066a_v1.yaml"
)
_WRIST_ISAAC_BINDING_PATH = "configs/bindings/isaac/fixed_xyz_wrist3_d6_v1.yaml"
_MUJOCO_ASSEMBLY_PATH = (
    "configs/assemblies/fr3v2_hand2_right_identity_v2026_6_27_v1.yaml"
)
_MUJOCO_WORKCELL_PATH = (
    "configs/workcells/mujoco_long_edge_table_pedestal_v1.yaml"
)
_FIXED_ASSEMBLY_PATHS = frozenset(
    {
        "configs/assemblies/hand2_left_fixed_v2026_6_27_v1.yaml",
        "configs/assemblies/hand2_right_fixed_v2026_6_27_v1.yaml",
        "configs/assemblies/hand2_left_fixed_v2026_8_3_v1.yaml",
        "configs/assemblies/hand2_right_fixed_v2026_8_3_v1.yaml",
    }
)
_FIXED_WORKCELL_PATHS = frozenset(
    {
        "configs/workcells/isaac_hand2_table_v1.yaml",
        "configs/workcells/isaac_hand2_left_table_v2026_6_27_v1.yaml",
        "configs/workcells/isaac_hand2_right_table_v2026_6_27_v1.yaml",
        "configs/workcells/isaac_hand2_left_table_v2026_8_3_v1.yaml",
        "configs/workcells/isaac_hand2_right_table_v2026_8_3_v1.yaml",
    }
)
_ROTATION_ASSEMBLY_PATH = (
    "configs/assemblies/hand2_right_rotation_mount_v2026_6_27_v1.yaml"
)
_ROTATION_WORKCELL_PATH = "configs/workcells/isaac_hand2_rotation_ball_v1.yaml"


@dataclass(frozen=True, slots=True)
class IsaacHandRuntime:
    resolved: ResolvedSession
    asset_path: Path
    profile_path: Path


@dataclass(frozen=True, slots=True)
class FixedHandWorkcellRuntime:
    table: EntitySpec
    hand_mount: PoseSpec
    camera_eye_m: tuple[float, float, float]
    camera_target_m: tuple[float, float, float]


def resolve_mujoco_table_runtime(
    project_root: str | Path,
    *,
    session_path: str | Path | None = None,
    scene_profile_override: str | Path | None = None,
) -> tuple[ResolvedSession, MujocoTableSceneConfig, Path]:
    """Resolve the MuJoCo Session and validate its current typed compatibility leaf."""

    root = Path(project_root).resolve()
    selected = session_path or root / MUJOCO_TABLE_SESSION
    overrides = (
        {}
        if scene_profile_override is None
        else {"scene_profile": scene_profile_override}
    )
    resolved = SessionResolver(root).resolve(selected, overrides=overrides)
    _require_session(
        resolved,
        backend="mujoco",
        runtime_roles={"simulation"},
    )
    _require_mujoco_leaf(resolved)
    profile_ref = (
        scene_profile_override
        if scene_profile_override is not None
        else resolved.session.runtime.compatibility_profile
    )
    if profile_ref is None:
        raise ValueError("MuJoCo table Session requires a compatibility profile")
    _validate_profile_owned_workcell(
        resolved,
        profile_overridden=scene_profile_override is not None,
    )
    profile_path = _existing_file(profile_ref, root=root, field="MuJoCo scene profile")
    config = load_mujoco_table_scene_config(profile_path)
    arm = _single_instance_of_kind(resolved, "robot_arm")
    hand = _single_instance_of_kind(resolved, "robot_hand")
    _validate_group_against_profile(
        arm,
        group_id="arm_joints",
        profile_path=_existing_file(
            config.assets.arm_profile,
            root=root,
            field="FR3 profile",
        ),
    )
    _validate_group_against_profile(
        hand,
        group_id="finger_joints",
        profile_path=_existing_file(
            config.assets.hand_profile,
            root=root,
            field="Hand 2 profile",
        ),
    )
    _require_artifact(arm, loader="mjcf")
    _require_artifact(hand, loader="mjcf")
    assert arm.artifact is not None
    assert hand.artifact is not None
    arm_tree = _single_resource_tree(arm)
    hand_tree = _single_resource_tree(hand)
    expected = {
        "arm_profile": Path(_required_binding_profile(arm)),
        "arm_mjcf": arm.artifact.absolute_path.relative_to(root),
        "arm_mjcf_sha256": arm.artifact.expected_sha256,
        "arm_asset_dir": arm_tree.absolute_path.relative_to(root),
        "arm_asset_tree_sha256": arm_tree.expected_sha256,
        "hand_profile": Path(_required_binding_profile(hand)),
        "hand_mjcf": hand.artifact.absolute_path.relative_to(root),
        "hand_mjcf_sha256": hand.artifact.expected_sha256,
        "hand_asset_dir": hand_tree.absolute_path.relative_to(root),
        "hand_asset_tree_sha256": hand_tree.expected_sha256,
    }
    actual = {
        "arm_profile": config.assets.arm_profile,
        "arm_mjcf": config.assets.arm_mjcf,
        "arm_mjcf_sha256": config.assets.arm_mjcf_sha256,
        "arm_asset_dir": config.assets.arm_asset_dir,
        "arm_asset_tree_sha256": config.assets.arm_asset_tree_sha256,
        "hand_profile": config.assets.hand_profile,
        "hand_mjcf": config.assets.hand_mjcf,
        "hand_mjcf_sha256": config.assets.hand_mjcf_sha256,
        "hand_asset_dir": config.assets.hand_asset_dir,
        "hand_asset_tree_sha256": config.assets.hand_asset_tree_sha256,
    }
    if actual != expected:
        raise ValueError("MuJoCo compatibility profile disagrees with resolved bindings")
    attachments = resolved.assembly.attachments
    if len(attachments) != 1:
        raise ValueError("current MuJoCo compatibility leaf requires one attachment")
    attachment = attachments[0]
    parent_backend_frame = arm.binding.backend_frame(attachment.parent.frame)
    child_backend_frame = hand.binding.backend_frame(attachment.child.frame)
    if (
        config.hand_attachment.parent_body != parent_backend_frame
        or config.hand_attachment.child_body != child_backend_frame
        or config.hand_attachment.position_m != attachment.transform.position_m
        or config.hand_attachment.quat_wxyz != attachment.transform.quat_wxyz
        or config.hand_attachment.assumption != attachment.assumption
    ):
        raise ValueError("MuJoCo compatibility profile disagrees with assembly attachment")
    return resolved, config, profile_path


def resolve_isaac_hand_runtime(
    project_root: str | Path,
    *,
    session_path: str | Path,
    runtime_roles: Collection[str],
    asset_override: str | Path | None = None,
    profile_override: str | Path | None = None,
    additional_overrides: Mapping[str, str | Path] | None = None,
) -> IsaacHandRuntime:
    """Resolve an Isaac Session and derive its pinned hand USD/profile paths."""

    root = Path(project_root).resolve()
    overrides: dict[str, str | Path] = dict(additional_overrides or {})
    if asset_override is not None:
        overrides["asset"] = asset_override
    if profile_override is not None:
        overrides["profile"] = profile_override
    resolved = SessionResolver(root).resolve(session_path, overrides=overrides)
    _require_session(resolved, backend="isaac", runtime_roles=runtime_roles)
    if resolved.assembly_path in _FIXED_ASSEMBLY_PATHS:
        _require_fixed_leaf(resolved)
    elif resolved.assembly_path == _ROTATION_ASSEMBLY_PATH:
        _require_rotation_leaf(resolved)
    else:
        raise ValueError(
            "current Isaac compatibility runner supports only the fixed-hand "
            "and rotation-ball assemblies"
        )
    hand = _single_instance_of_kind(resolved, "robot_hand")
    _require_artifact(hand, loader="usd")
    assert hand.artifact is not None
    asset_path = (
        hand.artifact.absolute_path
        if asset_override is None
        else _existing_file(asset_override, root=root, field="Hand 2 USD")
    )
    profile_path = (
        _existing_file(
            _required_binding_profile(hand), root=root, field="Hand 2 profile"
        )
        if profile_override is None
        else _existing_file(profile_override, root=root, field="Hand 2 profile")
    )
    _validate_group_against_profile(
        hand,
        group_id="finger_joints",
        profile_path=profile_path,
    )
    _validate_profile_artifact(hand, profile_path=profile_path)
    return IsaacHandRuntime(
        resolved=resolved,
        asset_path=asset_path,
        profile_path=profile_path,
    )


def resolve_rotation_ball_runtime(
    project_root: str | Path,
    *,
    session_path: str | Path,
    runtime_roles: Collection[str],
    asset_override: str | Path | None = None,
    profile_override: str | Path | None = None,
    scene_profile_override: str | Path | None = None,
) -> tuple[IsaacHandRuntime, RotationBallConfig, Path]:
    """Resolve the rotation-ball Session and validate the typed task profile."""

    root = Path(project_root).resolve()
    runtime = resolve_isaac_hand_runtime(
        root,
        session_path=session_path,
        runtime_roles=runtime_roles,
        asset_override=asset_override,
        profile_override=profile_override,
        additional_overrides=(
            {}
            if scene_profile_override is None
            else {"scene_profile": scene_profile_override}
        ),
    )
    profile_ref = (
        scene_profile_override
        if scene_profile_override is not None
        else runtime.resolved.session.runtime.compatibility_profile
    )
    if profile_ref is None:
        raise ValueError("rotation-ball Session requires a compatibility profile")
    _validate_profile_owned_workcell(
        runtime.resolved,
        profile_overridden=scene_profile_override is not None,
    )
    profile_path = _existing_file(
        profile_ref, root=root, field="rotation-ball scene profile"
    )
    scene = load_rotation_ball_config(profile_path)
    hand = _single_instance_of_kind(runtime.resolved, "robot_hand")
    assert hand.artifact is not None
    if (
        scene.provenance.get("usd") != hand.artifact.relative_path
        or scene.provenance.get("usd_sha256") != hand.artifact.expected_sha256
    ):
        raise ValueError(
            "rotation-ball compatibility profile disagrees with Hand 2 binding"
        )
    return runtime, scene, profile_path


def fixed_hand_workcell_runtime(
    resolved: ResolvedSession,
) -> FixedHandWorkcellRuntime:
    """Extract current fixed-hand geometry from its generic Workcell."""

    if resolved.workcell.compatibility_profile is not None:
        raise ValueError("fixed-hand Workcell must own its geometry directly")
    if {entity.entity_id for entity in resolved.workcell.entities} != {
        "ground",
        "table",
    }:
        raise ValueError(
            "fixed-hand compatibility leaf requires exactly ground and table entities"
        )
    if {frame.frame_id for frame in resolved.workcell.frames} != {
        "camera_eye",
        "camera_target",
    }:
        raise ValueError(
            "fixed-hand compatibility leaf requires exactly two camera frames"
        )
    ground = _entity(resolved, "ground")
    table = _entity(resolved, "table")
    if (
        ground.frame != resolved.workcell.world_frame
        or ground.transform != PoseSpec.identity()
        or ground.primitive.kind != "plane"
        or ground.mobility != "fixed"
        or ground.mass_kg is not None
    ):
        raise ValueError("fixed-hand Workcell ground must be the default world plane")
    if (
        table.primitive.kind != "box"
        or table.mobility != "fixed"
        or table.mass_kg is not None
        or table.transform.quat_wxyz != PoseSpec.identity().quat_wxyz
    ):
        raise ValueError("fixed-hand Workcell table must be a fixed box")
    root_instance = _single_assembly_root(resolved)
    mount = resolved.workcell.mount(resolved.session.mount_for(root_instance))
    eye = _frame(resolved, "camera_eye")
    target = _frame(resolved, "camera_target")
    if (
        table.frame != resolved.workcell.world_frame
        or mount.frame != resolved.workcell.world_frame
        or eye.parent != resolved.workcell.world_frame
        or target.parent != resolved.workcell.world_frame
    ):
        raise ValueError("fixed-hand Workcell runtime values must be world-relative")
    return FixedHandWorkcellRuntime(
        table=table,
        hand_mount=mount.transform,
        camera_eye_m=eye.transform.position_m,
        camera_target_m=target.transform.position_m,
    )


def resolve_mediapipe_session(
    project_root: str | Path,
    *,
    session_path: str | Path,
    expected_transport_contract: str | None,
) -> ResolvedSession:
    """Resolve a MediaPipe producer and validate the selected wire contract."""

    root = Path(project_root).resolve()
    resolved = SessionResolver(root).resolve(session_path)
    _require_session(
        resolved,
        backend="isaac",
        runtime_roles={"teleop_producer"},
    )
    if resolved.assembly_path in _FIXED_ASSEMBLY_PATHS:
        _require_fixed_leaf(resolved)
    elif resolved.assembly_path == _ROTATION_ASSEMBLY_PATH:
        _require_rotation_leaf(resolved)
    else:
        raise ValueError(
            "current MediaPipe compatibility runner supports only the fixed-hand "
            "and rotation-ball target assemblies"
        )
    if (
        expected_transport_contract is not None
        and resolved.session.runtime.transport_contract
        != expected_transport_contract
    ):
        raise ValueError(
            "MediaPipe Session transport contract does not match the selected "
            "publish option"
        )
    hand = _single_instance_of_kind(resolved, "robot_hand")
    _validate_group_against_profile(
        hand,
        group_id="finger_joints",
        profile_path=_existing_file(
            _required_binding_profile(hand),
            root=root,
            field="Hand 2 profile",
        ),
    )
    return resolved


def _require_session(
    resolved: ResolvedSession,
    *,
    backend: str,
    runtime_roles: Collection[str],
) -> None:
    if resolved.session.backend != backend:
        raise ValueError(
            f"runner requires backend {backend!r}, got {resolved.session.backend!r}"
        )
    if resolved.session.runtime_role not in runtime_roles:
        raise ValueError(
            f"runner does not support runtime role {resolved.session.runtime_role!r}; "
            f"expected one of {sorted(runtime_roles)}"
        )


def _require_mujoco_leaf(resolved: ResolvedSession) -> None:
    _require_leaf_files(
        resolved,
        assembly_path=_MUJOCO_ASSEMBLY_PATH,
        workcell_path=_MUJOCO_WORKCELL_PATH,
        instances={
            "arm": ("robot_arm", _FR3_ASSET_PATH, _FR3_MUJOCO_BINDING_PATH),
            "hand": ("robot_hand", _HAND_ASSET_PATH, _HAND_MUJOCO_BINDING_PATH),
        },
        roots=("arm",),
    )


def _require_fixed_leaf(resolved: ResolvedSession) -> None:
    if resolved.assembly_path not in _FIXED_ASSEMBLY_PATHS:
        raise ValueError("fixed-hand compatibility leaf requires a versioned Assembly")
    if resolved.workcell_path not in _FIXED_WORKCELL_PATHS:
        raise ValueError("fixed-hand compatibility leaf requires its pinned Workcell")
    if {instance.instance_id for instance in resolved.instances} != {"hand"}:
        raise ValueError("fixed-hand compatibility leaf requires exactly one hand instance")
    if resolved.assembly.roots != ("hand",):
        raise ValueError("fixed-hand compatibility leaf requires the hand as its only root")
    if resolved.assembly.attachments:
        raise ValueError("fixed-hand compatibility leaf does not support attachments")
    if (
        resolved.session.runtime_role in {"teleop_producer", "teleop_consumer"}
        and resolved.session.runtime.transport_contract != "wujihand.q20.v1"
    ):
        raise ValueError(
            "fixed-hand compatibility leaf requires wujihand.q20.v1 transport"
        )
    hand = resolved.instance("hand")
    _require_isaac_hand_binding(hand)


def _require_rotation_leaf(resolved: ResolvedSession) -> None:
    _require_leaf_files(
        resolved,
        assembly_path=_ROTATION_ASSEMBLY_PATH,
        workcell_path=_ROTATION_WORKCELL_PATH,
        instances={
            "wrist": (
                "virtual_mechanism",
                _WRIST_ASSET_PATH,
                _WRIST_ISAAC_BINDING_PATH,
            ),
            "hand": ("robot_hand", _HAND_ASSET_PATH, _HAND_ISAAC_BINDING_PATH),
        },
        roots=("wrist",),
    )
    hand = resolved.instance("hand")
    wrist = resolved.instance("wrist")
    if (
        resolved.session.runtime_role in {"teleop_producer", "teleop_consumer"}
        and resolved.session.runtime.transport_contract
        != "wujihand.hand_command.v2"
    ):
        raise ValueError(
            "rotation compatibility leaf requires wujihand.hand_command.v2 transport"
        )
    _require_isaac_hand_binding(hand)
    wrist_group = wrist.binding.group_binding("wrist_rotation")
    if (
        wrist.binding.loader != "procedural"
        or wrist.binding.builder != "hand2_rotation_mount_d6_v1"
        or wrist_group.joints != ("rotX", "rotY", "rotZ")
        or wrist_group.actuators
    ):
        raise ValueError(
            "rotation compatibility leaf requires the pinned D6 wrist binding"
        )
    attachments = resolved.assembly.attachments
    if len(attachments) != 1:
        raise ValueError("rotation compatibility leaf requires one attachment")
    attachment = attachments[0]
    if (
        attachment.parent.instance != "wrist"
        or attachment.parent.frame != "hand_mount"
        or attachment.child.instance != "hand"
        or attachment.child.frame != "hand_base"
        or attachment.transform != PoseSpec.identity()
    ):
        raise ValueError(
            "rotation compatibility leaf requires the identity wrist-to-hand attachment"
        )


def _require_leaf_files(
    resolved: ResolvedSession,
    *,
    assembly_path: str,
    workcell_path: str,
    instances: Mapping[str, tuple[str, str, str]],
    roots: tuple[str, ...],
) -> None:
    if (
        resolved.assembly_path != assembly_path
        or resolved.workcell_path != workcell_path
    ):
        raise ValueError(
            "current compatibility leaf requires its pinned Assembly and Workcell"
        )
    actual_instances = {instance.instance_id for instance in resolved.instances}
    if actual_instances != set(instances):
        raise ValueError(
            "current compatibility leaf does not support additional or missing "
            f"instances: expected={sorted(instances)}, actual={sorted(actual_instances)}"
        )
    if resolved.assembly.roots != roots:
        raise ValueError(
            f"current compatibility leaf requires roots {roots!r}"
        )
    for instance_id, (kind, asset_path, binding_path) in instances.items():
        instance = resolved.instance(instance_id)
        if (
            instance.asset.kind != kind
            or instance.asset_path != asset_path
            or instance.binding_path != binding_path
        ):
            raise ValueError(
                f"current compatibility leaf rejects substituted {instance_id!r} "
                "asset or binding contracts"
            )


def _require_isaac_hand_binding(hand: ResolvedInstance) -> None:
    group = hand.binding.group_binding("finger_joints")
    expected_root = hand.binding.backend_frame(hand.asset.frame_name("base"))
    if (
        hand.asset.kind != "robot_hand"
        or hand.binding.loader != "usd"
        or hand.binding.root != expected_root
        or group.actuators
    ):
        raise ValueError(
            "Isaac Hand 2 compatibility leaf requires a coherent pinned USD binding"
        )


def _validate_group_against_profile(
    instance: ResolvedInstance,
    *,
    group_id: str,
    profile_path: Path,
) -> None:
    names = _profile_joint_names(profile_path)
    group = instance.binding.group_binding(group_id)
    if group.joints != names:
        raise ValueError(
            f"binding {instance.binding.binding_id!r} joint order disagrees "
            f"with profile {profile_path}"
        )
    if instance.binding.backend == "mujoco":
        if len(group.actuators) != len(group.joints):
            raise ValueError(
                f"MuJoCo binding {instance.binding.binding_id!r} requires one "
                "actuator per joint"
            )
    elif group.actuators:
        raise ValueError(
            f"Isaac binding {instance.binding.binding_id!r} must use Drive/DOF APIs "
            "instead of MuJoCo-style actuator names"
        )


def _profile_joint_names(path: Path) -> tuple[str, ...]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"model profile must be a mapping: {path}")
    profile = cast(Mapping[str, object], value)
    joints = profile.get("joints")
    if not isinstance(joints, list) or not joints:
        raise ValueError(f"model profile joints must be a non-empty list: {path}")
    names: list[str] = []
    for index, joint in enumerate(joints):
        if not isinstance(joint, Mapping):
            raise ValueError(f"model profile joint {index} must be a mapping: {path}")
        name = joint.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"model profile joint {index} has no valid name: {path}")
        names.append(name)
    if len(set(names)) != len(names):
        raise ValueError(f"model profile joint names must be unique: {path}")
    return tuple(names)


def _validate_profile_artifact(
    instance: ResolvedInstance,
    *,
    profile_path: Path,
) -> None:
    if instance.artifact is None:
        raise ValueError("Hand 2 profile validation requires a resolved artifact")
    value = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError(f"model profile must be a mapping: {profile_path}")
    derived = value.get("derived_from")
    if not isinstance(derived, Mapping):
        raise ValueError(f"model profile must declare derived_from: {profile_path}")
    loader = instance.binding.loader
    if loader not in {"usd", "mjcf"}:
        raise ValueError(f"unsupported Hand 2 artifact loader {loader!r}")
    expected_path = derived.get(loader)
    expected_hash = derived.get(f"{loader}_sha256")
    if (
        expected_path != instance.artifact.relative_path
        or expected_hash != instance.artifact.expected_sha256
    ):
        raise ValueError(
            f"Hand 2 profile {profile_path} disagrees with the resolved {loader.upper()} artifact"
        )
    source_revision = dict(instance.artifact.source.revision)
    for field in ("tag", "commit"):
        expected = source_revision.get(field)
        if expected is not None and derived.get(field) != expected:
            raise ValueError(
                f"Hand 2 profile {profile_path} disagrees with source {field} {expected!r}"
            )


def _single_instance_of_kind(
    resolved: ResolvedSession, kind: str
) -> ResolvedInstance:
    matches = [
        instance for instance in resolved.instances if instance.asset.kind == kind
    ]
    if len(matches) != 1:
        raise ValueError(
            f"current compatibility leaf requires one {kind} instance, got "
            f"{[instance.instance_id for instance in matches]}"
        )
    return matches[0]


def _require_artifact(instance: ResolvedInstance, *, loader: str) -> None:
    if instance.binding.loader != loader or instance.artifact is None:
        raise ValueError(
            f"instance {instance.instance_id!r} requires a {loader} artifact binding"
        )


def _single_resource_tree(instance: ResolvedInstance) -> ResolvedArtifact:
    if len(instance.resource_trees) != 1:
        raise ValueError(
            f"instance {instance.instance_id!r} requires one resource tree"
        )
    return instance.resource_trees[0]


def _required_binding_profile(instance: ResolvedInstance) -> str:
    profile = instance.binding.compatibility_profile
    if profile is None:
        raise ValueError(
            f"instance {instance.instance_id!r} requires a binding compatibility profile"
        )
    return profile


def _existing_file(
    reference: str | Path, *, root: Path, field: str
) -> Path:
    raw = Path(reference)
    path = raw if raw.is_absolute() else root / raw
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"{field} not found: {resolved}")
    return resolved


def _entity(resolved: ResolvedSession, entity_id: str) -> EntitySpec:
    for entity in resolved.workcell.entities:
        if entity.entity_id == entity_id:
            return entity
    raise ValueError(f"Workcell entity not found: {entity_id}")


def _frame(resolved: ResolvedSession, frame_id: str) -> WorkcellFrameSpec:
    for frame in resolved.workcell.frames:
        if frame.frame_id == frame_id:
            return frame
    raise ValueError(f"Workcell frame not found: {frame_id}")


def _single_assembly_root(resolved: ResolvedSession) -> str:
    if len(resolved.assembly.roots) != 1:
        raise ValueError(
            "current compatibility leaf requires exactly one assembly root"
        )
    return resolved.assembly.roots[0]


def _validate_profile_owned_workcell(
    resolved: ResolvedSession,
    *,
    profile_overridden: bool,
) -> None:
    """Reject Workcell values that a typed compatibility leaf would ignore."""

    expected_semantics = {
        _MUJOCO_WORKCELL_PATH: (
            "arm_pedestal_profile_mount",
            "arm_pedestal_mount",
        ),
        _ROTATION_WORKCELL_PATH: (
            "rotation_flange_profile_mount",
            "rotation_flange_mount",
        ),
    }
    expected = expected_semantics.get(resolved.workcell_path)
    if expected is None:
        raise ValueError("unsupported profile-owned compatibility Workcell")
    expected_frame, expected_mount = expected
    if (
        {frame.frame_id for frame in resolved.workcell.frames} != {expected_frame}
        or {mount.mount_id for mount in resolved.workcell.mounts}
        != {expected_mount}
        or resolved.workcell.entities
    ):
        raise ValueError(
            "profile-owned Workcell contains values the compatibility leaf "
            "cannot consume"
        )
    workcell_profile = resolved.workcell.compatibility_profile
    runtime_profile = resolved.session.runtime.compatibility_profile
    if workcell_profile is None:
        raise ValueError("compatibility Workcell requires a typed profile")
    if not profile_overridden and workcell_profile != runtime_profile:
        raise ValueError(
            "Workcell and Session compatibility profiles must reference the same file"
        )
    root_instance = _single_assembly_root(resolved)
    mount = resolved.workcell.mount(resolved.session.mount_for(root_instance))
    identity = PoseSpec.identity()
    if mount.frame != expected_frame or mount.transform != identity:
        raise ValueError(
            "profile-owned Workcell mount must use its pinned identity frame"
        )
    frame = _frame(resolved, mount.frame)
    if (
        frame.parent != resolved.workcell.world_frame
        or frame.transform != identity
    ):
        raise ValueError(
            "profile-owned Workcell mount frame must remain identity in world"
        )


__all__ = [
    "FixedHandWorkcellRuntime",
    "ISAAC_FIXED_PREVIEW_SESSION",
    "ISAAC_FIXED_TELEOP_SESSION",
    "ISAAC_ROTATION_QUALIFICATION_SESSION",
    "ISAAC_ROTATION_TELEOP_SESSION",
    "IsaacHandRuntime",
    "MEDIAPIPE_HAND_COMMAND_SESSION",
    "MEDIAPIPE_Q20_SESSION",
    "MUJOCO_TABLE_SESSION",
    "fixed_hand_workcell_runtime",
    "resolve_isaac_hand_runtime",
    "resolve_mediapipe_session",
    "resolve_mujoco_table_runtime",
    "resolve_rotation_ball_runtime",
]
