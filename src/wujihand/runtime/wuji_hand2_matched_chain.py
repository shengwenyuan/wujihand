"""Fail-closed Wuji SDK 8.3 + Description 8.3 Hand2 qualification preflight."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Protocol, Self
import xml.etree.ElementTree as ET

import psutil  # type: ignore[import-untyped]

from wujihand.domain import HAND2_LAYOUT_IDS, HandSide
from wujihand.integrity import sha256_file
from wujihand.specs.common import (
    require_exact_mapping,
    require_string,
    validate_identifier,
    validate_project_reference,
)

from .session_resolver import SessionResolver
from .yaml_loader import load_yaml_strict


QUALIFICATION_SCHEMA = "wujihand.wuji_hand2_matched_chain_qualification.v1"
LOCAL_BINDING_SCHEMA = "wujihand.wuji_hand2_matched_chain_local_binding.v1"
PREFLIGHT_RECEIPT_SCHEMA = "wujihand.wuji_hand2_matched_chain_preflight.v1"


def _sha256(value: object, *, field: str) -> str:
    digest = require_string(value, field=field)
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ValueError(f"{field} must be a lowercase SHA-256 digest")
    return digest


def _absolute_path(value: object, *, field: str) -> Path:
    path = Path(require_string(value, field=field))
    if not path.is_absolute():
        raise ValueError(f"{field} must be an absolute host-local path")
    return path


def _positive_integer(value: object, *, field: str) -> int:
    if type(value) is not int or value < 1:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _version_identifier(value: object, *, field: str) -> str:
    version = require_string(value, field=field)
    if len(version) > 64 or any(
        character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.+-"
        for character in version
    ):
        raise ValueError(f"{field} must be a bounded version identifier")
    return version


@dataclass(frozen=True, slots=True)
class MatchedChainSdkPolicy:
    distribution: str
    package_version: str
    wheel_sha256: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"distribution", "package_version", "wheel_sha256"}),
            field=field,
        )
        return cls(
            distribution=validate_identifier(data["distribution"], field=f"{field}.distribution"),
            package_version=_version_identifier(
                data["package_version"], field=f"{field}.package_version"
            ),
            wheel_sha256=_sha256(data["wheel_sha256"], field=f"{field}.wheel_sha256"),
        )


@dataclass(frozen=True, slots=True)
class MatchedChainDescriptionPolicy:
    release: str
    hand2_model_revision: str
    source_name: str
    commit: str
    asset_revision: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"release", "hand2_model_revision", "source_name", "commit", "asset_revision"}
            ),
            field=field,
        )
        commit = require_string(data["commit"], field=f"{field}.commit")
        if len(commit) != 40 or any(character not in "0123456789abcdef" for character in commit):
            raise ValueError(f"{field}.commit must be a lowercase Git commit")
        return cls(
            release=validate_identifier(data["release"], field=f"{field}.release"),
            hand2_model_revision=validate_identifier(
                data["hand2_model_revision"], field=f"{field}.hand2_model_revision"
            ),
            source_name=validate_identifier(data["source_name"], field=f"{field}.source_name"),
            commit=commit,
            asset_revision=validate_identifier(
                data["asset_revision"], field=f"{field}.asset_revision"
            ),
        )


@dataclass(frozen=True, slots=True)
class MatchedChainSidePolicy:
    session_path: str
    session_id: str
    binding_root: str
    layout_id: str
    urdf_filename: str
    urdf_link_count: int
    urdf_joint_count: int

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {
                    "session_path",
                    "session_id",
                    "binding_root",
                    "layout_id",
                    "urdf_filename",
                    "urdf_link_count",
                    "urdf_joint_count",
                }
            ),
            field=field,
        )
        filename = require_string(data["urdf_filename"], field=f"{field}.urdf_filename")
        if Path(filename).name != filename or not filename.endswith("_hand.urdf"):
            raise ValueError(f"{field}.urdf_filename must be a side-specific URDF filename")
        return cls(
            session_path=validate_project_reference(
                data["session_path"], field=f"{field}.session_path"
            ),
            session_id=validate_identifier(data["session_id"], field=f"{field}.session_id"),
            binding_root=validate_identifier(
                data["binding_root"], field=f"{field}.binding_root"
            ),
            layout_id=validate_identifier(data["layout_id"], field=f"{field}.layout_id"),
            urdf_filename=filename,
            urdf_link_count=_positive_integer(
                data["urdf_link_count"], field=f"{field}.urdf_link_count"
            ),
            urdf_joint_count=_positive_integer(
                data["urdf_joint_count"], field=f"{field}.urdf_joint_count"
            ),
        )


@dataclass(frozen=True, slots=True)
class MatchedChainQualificationPolicy:
    qualification_id: str
    sdk: MatchedChainSdkPolicy
    description: MatchedChainDescriptionPolicy
    sides: tuple[tuple[HandSide, MatchedChainSidePolicy], ...]

    @classmethod
    def from_mapping(cls, value: object) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"schema", "qualification_id", "sdk", "description", "sides"}),
            field="qualification",
        )
        if data["schema"] != QUALIFICATION_SCHEMA:
            raise ValueError(f"qualification.schema must be {QUALIFICATION_SCHEMA!r}")
        side_data = require_exact_mapping(
            data["sides"], expected=frozenset({"left", "right"}), field="qualification.sides"
        )
        sides = tuple(
            (
                side,
                MatchedChainSidePolicy.from_mapping(
                    side_data[side.value], field=f"qualification.sides.{side.value}"
                ),
            )
            for side in HandSide
        )
        for side, policy in sides:
            if policy.urdf_filename != f"{side.value}_hand.urdf":
                raise ValueError(f"qualification {side.value} URDF filename has the wrong side")
            if policy.layout_id != HAND2_LAYOUT_IDS[side.value]:
                raise ValueError(f"qualification {side.value} layout is not canonical Hand2 q20")
        return cls(
            qualification_id=validate_identifier(
                data["qualification_id"], field="qualification.qualification_id"
            ),
            sdk=MatchedChainSdkPolicy.from_mapping(data["sdk"], field="qualification.sdk"),
            description=MatchedChainDescriptionPolicy.from_mapping(
                data["description"], field="qualification.description"
            ),
            sides=sides,
        )

    def side(self, side: HandSide) -> MatchedChainSidePolicy:
        for candidate, policy in self.sides:
            if candidate is side:
                return policy
        raise KeyError(side)


@dataclass(frozen=True, slots=True)
class MatchedChainLocalHandBinding:
    serial_number: str
    urdf_path: Path
    urdf_sha256: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"serial_number", "urdf_path", "urdf_sha256"}),
            field=field,
        )
        return cls(
            serial_number=validate_identifier(
                data["serial_number"], field=f"{field}.serial_number"
            ),
            urdf_path=_absolute_path(data["urdf_path"], field=f"{field}.urdf_path"),
            urdf_sha256=_sha256(data["urdf_sha256"], field=f"{field}.urdf_sha256"),
        )


@dataclass(frozen=True, slots=True)
class MatchedChainLocalBinding:
    binding_id: str
    interpreter: Path
    sdk_module_root: Path
    sdk_wheel: Path
    user_id: str
    user_display_name: str
    user_models_dir: Path
    hands: tuple[tuple[HandSide, MatchedChainLocalHandBinding], ...]

    @classmethod
    def from_mapping(cls, value: object) -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset(
                {"schema", "binding_id", "interpreter", "sdk_module_root", "sdk_wheel", "user", "hands"}
            ),
            field="local_binding",
        )
        if data["schema"] != LOCAL_BINDING_SCHEMA:
            raise ValueError(f"local_binding.schema must be {LOCAL_BINDING_SCHEMA!r}")
        user = require_exact_mapping(
            data["user"],
            expected=frozenset({"user_id", "display_name", "models_dir"}),
            field="local_binding.user",
        )
        raw_hands = require_exact_mapping(
            data["hands"], expected=frozenset({"left", "right"}), field="local_binding.hands"
        )
        return cls(
            binding_id=validate_identifier(
                data["binding_id"], field="local_binding.binding_id"
            ),
            interpreter=_absolute_path(data["interpreter"], field="local_binding.interpreter"),
            sdk_module_root=_absolute_path(
                data["sdk_module_root"], field="local_binding.sdk_module_root"
            ),
            sdk_wheel=_absolute_path(data["sdk_wheel"], field="local_binding.sdk_wheel"),
            user_id=validate_identifier(user["user_id"], field="local_binding.user.user_id"),
            user_display_name=require_string(
                user["display_name"], field="local_binding.user.display_name"
            ),
            user_models_dir=_absolute_path(
                user["models_dir"], field="local_binding.user.models_dir"
            ),
            hands=tuple(
                (
                    side,
                    MatchedChainLocalHandBinding.from_mapping(
                        raw_hands[side.value], field=f"local_binding.hands.{side.value}"
                    ),
                )
                for side in HandSide
            ),
        )

    def hand(self, side: HandSide) -> MatchedChainLocalHandBinding:
        for candidate, binding in self.hands:
            if candidate is side:
                return binding
        raise KeyError(side)


@dataclass(frozen=True, slots=True)
class WujiSdkRuntimeFacts:
    distribution_version: str
    module_version: str
    module_path: Path
    executable_path: Path


class WujiSdkUserManager(Protocol):
    def list_users(self) -> Sequence[Mapping[str, object]]: ...

    def switch_user(self, user_id: str) -> object: ...

    def current_user(self) -> Mapping[str, object]: ...


@dataclass(frozen=True, slots=True)
class MatchedChainPreflightReceipt:
    qualification_id: str
    binding_id: str
    side: HandSide
    input_mode: str
    calibration_id: str
    serial_number: str
    sdk_version: str
    sdk_module_path: Path
    sdk_wheel_sha256: str
    sdk_user_id: str
    sdk_user_display_name: str
    calibrated_urdf_path: Path
    calibrated_urdf_sha256: str
    description_release: str
    hand2_model_revision: str
    description_commit: str
    description_source: str
    description_artifact_path: str
    description_artifact_sha256: str
    session_path: str
    session_id: str
    session_hash: str
    binding_root: str
    layout_id: str
    studio_processes: tuple[str, ...]

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": PREFLIGHT_RECEIPT_SCHEMA,
            "passed": True,
            "qualification_id": self.qualification_id,
            "binding_id": self.binding_id,
            "side": self.side.value,
            "input_mode": self.input_mode,
            "device_access_attempted": False,
            "isaac_started": False,
            "calibration_id": self.calibration_id,
            "glove_serial_number": self.serial_number,
            "sdk": {
                "version": self.sdk_version,
                "module_path": str(self.sdk_module_path),
                "wheel_sha256": self.sdk_wheel_sha256,
                "user_id": self.sdk_user_id,
                "user_display_name": self.sdk_user_display_name,
                "calibrated_urdf_path": str(self.calibrated_urdf_path),
                "calibrated_urdf_sha256": self.calibrated_urdf_sha256,
            },
            "description": {
                "release": self.description_release,
                "hand2_model_revision": self.hand2_model_revision,
                "commit": self.description_commit,
                "source": self.description_source,
                "artifact_path": self.description_artifact_path,
                "artifact_sha256": self.description_artifact_sha256,
                "session_path": self.session_path,
                "session_id": self.session_id,
                "session_hash": self.session_hash,
                "binding_root": self.binding_root,
                "layout_id": self.layout_id,
            },
            "studio_processes": list(self.studio_processes),
            "beta_warning": (
                "Wuji Hand2 remains Beta1; SDK or Description updates invalidate this receipt."
            ),
        }


def load_matched_chain_qualification_policy(
    path: str | Path,
) -> MatchedChainQualificationPolicy:
    return MatchedChainQualificationPolicy.from_mapping(
        load_yaml_strict(Path(path).read_text(encoding="utf-8"))
    )


def load_matched_chain_local_binding(path: str | Path) -> MatchedChainLocalBinding:
    return MatchedChainLocalBinding.from_mapping(
        load_yaml_strict(Path(path).read_text(encoding="utf-8"))
    )


def detect_wuji_studio_processes() -> tuple[str, ...]:
    """Return bounded process identities that can own a Glove subscription."""

    matches: list[str] = []
    for process in psutil.process_iter(("pid", "name", "exe")):
        try:
            name = str(process.info.get("name") or "")
            executable = Path(str(process.info.get("exe") or "")).name
        except (psutil.AccessDenied, psutil.NoSuchProcess, psutil.ZombieProcess):
            continue
        if name.lower() in {"wuji-studio", "wuji_studio"} or executable.lower() in {
            "wuji-studio",
            "wuji_studio",
        }:
            matches.append(f"pid={process.pid}:{name or executable}")
    return tuple(sorted(matches))


def inspect_wuji_sdk_runtime(module: object, *, distribution_version: str) -> WujiSdkRuntimeFacts:
    module_version = getattr(module, "__version__", None)
    module_file = getattr(module, "__file__", None)
    if type(module_version) is not str or type(module_file) is not str:
        raise ValueError("Wuji SDK module does not expose __version__ and __file__")
    return WujiSdkRuntimeFacts(
        distribution_version=distribution_version,
        module_version=module_version,
        module_path=Path(module_file),
        executable_path=Path(sys.executable),
    )


def preflight_wuji_hand2_matched_chain(
    project_root: str | Path,
    *,
    qualification_path: str | Path,
    local_binding_path: str | Path,
    side: HandSide,
    input_mode: str,
    sdk_runtime: WujiSdkRuntimeFacts,
    user_manager: WujiSdkUserManager,
    studio_processes: Sequence[str] = (),
    home_dir: str | Path | None = None,
    verify_artifacts: bool = True,
) -> MatchedChainPreflightReceipt:
    """Close all matched-chain identities without connecting a device or Isaac."""

    if type(side) is not HandSide:
        raise ValueError("side must be a HandSide")
    if input_mode not in {"stub", "glove"}:
        raise ValueError("input_mode must be 'stub' or 'glove'")
    policy = load_matched_chain_qualification_policy(qualification_path)
    local = load_matched_chain_local_binding(local_binding_path)
    side_policy = policy.side(side)
    hand = local.hand(side)

    expected_version = policy.sdk.package_version
    if (
        sdk_runtime.distribution_version != expected_version
        or sdk_runtime.module_version != expected_version
    ):
        raise RuntimeError(
            "Wuji SDK distribution/module version mismatch: "
            f"expected={expected_version}, distribution={sdk_runtime.distribution_version}, "
            f"module={sdk_runtime.module_version}"
        )
    if sdk_runtime.executable_path.resolve() != local.interpreter.resolve():
        raise RuntimeError("qualification is running under the wrong Python interpreter")
    module_path = sdk_runtime.module_path.resolve()
    module_root = local.sdk_module_root.resolve()
    if not module_path.is_relative_to(module_root):
        raise RuntimeError("Wuji SDK module was not loaded from the pinned 8.3 overlay")
    if not local.sdk_wheel.is_file():
        raise FileNotFoundError(f"pinned Wuji SDK wheel not found: {local.sdk_wheel}")
    wheel_sha256 = sha256_file(local.sdk_wheel)
    if wheel_sha256 != policy.sdk.wheel_sha256:
        raise RuntimeError("Wuji SDK wheel SHA-256 differs from the qualification policy")

    users = tuple(user_manager.list_users())
    matches = tuple(user for user in users if user.get("user_id") == local.user_id)
    if len(matches) != 1 or matches[0].get("is_default") is not False:
        raise RuntimeError("qualification requires one unique non-default Wuji SDK user")
    if matches[0].get("display_name") != local.user_display_name:
        raise RuntimeError("Wuji SDK user display name differs from the local binding")
    user_manager.switch_user(local.user_id)
    current_user = user_manager.current_user()
    if (
        current_user.get("user_id") != local.user_id
        or current_user.get("display_name") != local.user_display_name
        or current_user.get("is_default") is not False
    ):
        raise RuntimeError("Wuji SDK did not retain the required named user")

    host_home = Path.home() if home_dir is None else Path(home_dir)
    expected_models_dir = (
        host_home / ".wuji" / "sdk" / "users" / local.user_id / "models"
    ).resolve()
    if local.user_models_dir.resolve() != expected_models_dir:
        raise RuntimeError("calibrated models directory is outside the SDK named-user registry")
    expected_urdf = (expected_models_dir / side_policy.urdf_filename).resolve()
    if hand.urdf_path.resolve() != expected_urdf:
        raise RuntimeError("calibrated URDF path does not match user, side, and SDK registry")
    if not expected_urdf.is_file():
        raise FileNotFoundError(f"calibrated user URDF not found: {expected_urdf}")
    actual_urdf_sha256 = sha256_file(expected_urdf)
    if actual_urdf_sha256 != hand.urdf_sha256:
        raise RuntimeError("calibrated user URDF SHA-256 differs from the local binding")
    _validate_user_urdf(expected_urdf, side=side, policy=side_policy)

    processes = tuple(studio_processes)
    if input_mode == "glove" and processes:
        raise RuntimeError(
            "Wuji Studio must be closed before live Glove qualification: " + ", ".join(processes)
        )

    root = Path(project_root).resolve()
    resolved = SessionResolver(root).resolve(
        side_policy.session_path,
        verify_artifacts=verify_artifacts,
    )
    if (
        resolved.session.session_id != side_policy.session_id
        or resolved.session.backend != "isaac"
        or resolved.session.runtime_role != "qualification"
    ):
        raise RuntimeError("Description qualification Session identity/role mismatch")
    if len(resolved.instances) != 1:
        raise RuntimeError("matched-chain qualification Session must contain one Hand2")
    instance = resolved.instances[0]
    if (
        instance.asset.kind != "robot_hand"
        or instance.asset.side != side.value
        or instance.asset.revision != policy.description.asset_revision
        or instance.binding.root != side_policy.binding_root
    ):
        raise RuntimeError("resolved Hand2 asset, side, revision, or root differs from policy")
    if instance.artifact is None:
        raise RuntimeError("resolved Hand2 Session has no USD artifact")
    source = instance.artifact.source
    revisions = dict(source.revision)
    if (
        source.name != policy.description.source_name
        or revisions.get("tag") != policy.description.release
        or revisions.get("commit") != policy.description.commit
    ):
        raise RuntimeError("resolved Wuji Description source identity differs from policy")
    layouts = tuple(
        layout
        for layout in resolved.session.runtime.control_layouts
        if layout.instance_id == instance.instance_id and layout.group_id == "finger_joints"
    )
    if len(layouts) != 1 or layouts[0].layout_id != side_policy.layout_id:
        raise RuntimeError("resolved Session does not expose the expected canonical Hand2 q20")

    calibration_id = (
        f"wuji_sdk.user.{local.user_id}.{side.value}."
        f"urdf_{actual_urdf_sha256[:12]}.sdk_{expected_version}"
    )
    if len(calibration_id) > 128:
        raise RuntimeError("generated calibration_id exceeds the canonical transport bound")
    return MatchedChainPreflightReceipt(
        qualification_id=policy.qualification_id,
        binding_id=local.binding_id,
        side=side,
        input_mode=input_mode,
        calibration_id=calibration_id,
        serial_number=hand.serial_number,
        sdk_version=expected_version,
        sdk_module_path=module_path,
        sdk_wheel_sha256=wheel_sha256,
        sdk_user_id=local.user_id,
        sdk_user_display_name=local.user_display_name,
        calibrated_urdf_path=expected_urdf,
        calibrated_urdf_sha256=actual_urdf_sha256,
        description_release=policy.description.release,
        hand2_model_revision=policy.description.hand2_model_revision,
        description_commit=policy.description.commit,
        description_source=source.name,
        description_artifact_path=instance.artifact.relative_path,
        description_artifact_sha256=instance.artifact.expected_sha256,
        session_path=side_policy.session_path,
        session_id=resolved.session.session_id,
        session_hash=resolved.session_hash,
        binding_root=instance.binding.root,
        layout_id=layouts[0].layout_id,
        studio_processes=processes,
    )


def _validate_user_urdf(
    path: Path,
    *,
    side: HandSide,
    policy: MatchedChainSidePolicy,
) -> None:
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise RuntimeError("calibrated user URDF is not well-formed XML") from exc
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = tuple(link.get("name") for link in links)
    joint_names = tuple(joint.get("name") for joint in joints)
    if root.tag != "robot" or root.get("name") != f"{side.value}_hand":
        raise RuntimeError("calibrated user URDF robot name has the wrong side")
    if len(links) != policy.urdf_link_count or len(joints) != policy.urdf_joint_count:
        raise RuntimeError("calibrated user URDF topology differs from the Studio 8.3 policy")
    if (
        any(name is None for name in link_names + joint_names)
        or len(set(link_names)) != len(link_names)
        or len(set(joint_names)) != len(joint_names)
        or "wrist" not in link_names
    ):
        raise RuntimeError("calibrated user URDF has invalid or duplicate topology names")


__all__ = [
    "LOCAL_BINDING_SCHEMA",
    "PREFLIGHT_RECEIPT_SCHEMA",
    "QUALIFICATION_SCHEMA",
    "MatchedChainLocalBinding",
    "MatchedChainPreflightReceipt",
    "MatchedChainQualificationPolicy",
    "WujiSdkRuntimeFacts",
    "WujiSdkUserManager",
    "detect_wuji_studio_processes",
    "inspect_wuji_sdk_runtime",
    "load_matched_chain_local_binding",
    "load_matched_chain_qualification_policy",
    "preflight_wuji_hand2_matched_chain",
]
