"""Strict q54 dataset profile loading and explicit runtime reordering."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Final, cast

from wujihand.integrity import sha256_file
from wujihand.runtime.yaml_loader import load_yaml_strict


Q54_JOINT_PROFILE_SCHEMA: Final = "wujihand.dataset_joint_profile.v1"
MINI_DATASET_PROFILE_SCHEMA: Final = "wujihand.mini_dataset_profile.v1"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_.:/+-]{0,127}$")
_SIDES = ("left", "right")
_GROUPS = ("arm", "hand")


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _exact(
    value: object,
    *,
    keys: frozenset[str],
    field: str,
) -> Mapping[str, object]:
    result = _mapping(value, field=field)
    actual = frozenset(result)
    if actual != keys:
        raise ValueError(
            f"{field} keys differ from schema: "
            f"missing={sorted(keys - actual)}, unexpected={sorted(actual - keys)}"
        )
    return result


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field} must be a non-blank trimmed string")
    return value


def _identifier(value: object, *, field: str) -> str:
    result = _string(value, field=field)
    if _IDENTIFIER.fullmatch(result) is None:
        raise ValueError(f"{field} must be a bounded identifier")
    return result


def _integer(value: object, *, field: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{field} must be an integer")
    return value


def _finite(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be finite")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _project_path(value: object, *, field: str) -> str:
    result = _string(value, field=field)
    path = Path(result)
    if path.is_absolute() or ".." in path.parts or result.startswith("~") or "\\" in result:
        raise ValueError(f"{field} must be a safe project-relative path")
    return path.as_posix()


def _resolve(project_root: Path, reference: str, *, field: str) -> Path:
    path = (project_root / reference).resolve()
    try:
        path.relative_to(project_root)
    except ValueError as exc:
        raise ValueError(f"{field} escapes project root") from exc
    if not path.is_file():
        raise FileNotFoundError(f"{field} not found: {path}")
    return path


@dataclass(frozen=True, slots=True)
class SourceProfilePin:
    profile_id: str
    path: str
    sha256: str
    fact_scope: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> SourceProfilePin:
        data = _exact(
            value,
            keys=frozenset({"profile_id", "path", "sha256", "fact_scope"}),
            field=field,
        )
        digest = _string(data["sha256"], field=f"{field}.sha256")
        if _SHA256.fullmatch(digest) is None:
            raise ValueError(f"{field}.sha256 must be a lowercase SHA-256")
        return cls(
            profile_id=_identifier(data["profile_id"], field=f"{field}.profile_id"),
            path=_project_path(data["path"], field=f"{field}.path"),
            sha256=digest,
            fact_scope=_identifier(data["fact_scope"], field=f"{field}.fact_scope"),
        )


@dataclass(frozen=True, slots=True)
class DatasetConfigPin:
    path: str
    expected_id: str
    sha256: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> DatasetConfigPin:
        data = _exact(
            value,
            keys=frozenset({"path", "expected_id", "sha256"}),
            field=field,
        )
        digest = _string(data["sha256"], field=f"{field}.sha256")
        if _SHA256.fullmatch(digest) is None:
            raise ValueError(f"{field}.sha256 must be a lowercase SHA-256")
        return cls(
            path=_project_path(data["path"], field=f"{field}.path"),
            expected_id=_identifier(
                data["expected_id"],
                field=f"{field}.expected_id",
            ),
            sha256=digest,
        )


@dataclass(frozen=True, slots=True)
class DatasetCameraRole:
    logical_id: str
    feature_key: str
    carrier_identity: str
    profile: DatasetConfigPin
    payload_whitelist: tuple[str, ...]
    physical_calibration_compatible: bool

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> DatasetCameraRole:
        data = _exact(
            value,
            keys=frozenset(
                {
                    "logical_id",
                    "feature_key",
                    "carrier_identity",
                    "profile",
                    "payload_whitelist",
                    "physical_calibration_compatible",
                }
            ),
            field=field,
        )
        payloads = tuple(
            _identifier(item, field=f"{field}.payload_whitelist[{index}]")
            for index, item in enumerate(
                _sequence(data["payload_whitelist"], field=f"{field}.payload_whitelist")
            )
        )
        if payloads != ("rgb",):
            raise ValueError(f"{field} must expose RGB only")
        compatible = data["physical_calibration_compatible"]
        if type(compatible) is not bool or compatible:
            raise ValueError(f"{field}.physical_calibration_compatible must be false")
        return cls(
            logical_id=_identifier(data["logical_id"], field=f"{field}.logical_id"),
            feature_key=_identifier(data["feature_key"], field=f"{field}.feature_key"),
            carrier_identity=_identifier(
                data["carrier_identity"],
                field=f"{field}.carrier_identity",
            ),
            profile=DatasetConfigPin.from_mapping(
                data["profile"],
                field=f"{field}.profile",
            ),
            payload_whitelist=payloads,
            physical_calibration_compatible=False,
        )


@dataclass(frozen=True, slots=True)
class MiniDatasetProfile:
    profile_id: str
    status: str
    robot_configuration: str
    q54: Q54JointProfile
    physics_hz: int
    control_hz: int
    gui_preview_hz: int
    policy_fps: int
    cameras: tuple[DatasetCameraRole, ...]
    lerobot_commit: str
    lerobot_python: str
    retained_episode_hard_limit: int
    release_control_rate_tolerance_fraction: float
    release_minimum_real_time_factor: float
    release_maximum_input_age_ms: float
    file_sha256: str

    def __post_init__(self) -> None:
        if (self.physics_hz, self.control_hz, self.gui_preview_hz, self.policy_fps) != (
            120,
            60,
            20,
            30,
        ):
            raise ValueError("dataset timing must remain 120/60/20/30")
        logical = tuple(camera.logical_id for camera in self.cameras)
        if logical != ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb"):
            raise ValueError("camera roles must be scene, left wrist, right wrist RGB")
        features = tuple(camera.feature_key for camera in self.cameras)
        if len(set(features)) != 3:
            raise ValueError("camera feature keys must be unique")
        if not 0 < self.retained_episode_hard_limit < 20:
            raise ValueError("retained episode hard limit must be between 1 and 19")


def _load_pinned_mapping(
    root: Path,
    pin: DatasetConfigPin,
    *,
    field: str,
) -> Mapping[str, object]:
    path = _resolve(root, pin.path, field=field)
    actual = sha256_file(path)
    if actual != pin.sha256:
        raise ValueError(f"{field} hash differs: expected={pin.sha256}, actual={actual}")
    document = load_yaml_strict(path.read_text(encoding="utf-8"))
    mapping = _mapping(document, field=field)
    if mapping.get("profile_id") != pin.expected_id:
        raise ValueError(f"{field} expected profile ID differs")
    return mapping


@dataclass(frozen=True, slots=True)
class Q54JointSpec:
    global_index: int
    canonical_name: str
    side: str
    group: str
    group_index: int
    source_instance_id: str
    source_group_id: str
    source_joint_name: str
    source_index_q27: int
    source_profile_id: str
    unit: str
    sign: int
    zero_offset_rad: float
    lower_rad: float
    upper_rad: float
    max_velocity_rad_s: float
    fact_scope: str
    real_hardware_mapping_status: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> Q54JointSpec:
        keys = frozenset(
            {
                "global_index",
                "canonical_name",
                "side",
                "group",
                "group_index",
                "source_instance_id",
                "source_group_id",
                "source_joint_name",
                "source_index_q27",
                "source_profile_id",
                "unit",
                "sign",
                "zero_offset_rad",
                "lower_rad",
                "upper_rad",
                "max_velocity_rad_s",
                "fact_scope",
                "real_hardware_mapping_status",
            }
        )
        data = _exact(value, keys=keys, field=field)
        side = _string(data["side"], field=f"{field}.side")
        group = _string(data["group"], field=f"{field}.group")
        if side not in _SIDES:
            raise ValueError(f"{field}.side must be left or right")
        if group not in _GROUPS:
            raise ValueError(f"{field}.group must be arm or hand")
        sign = _integer(data["sign"], field=f"{field}.sign")
        if sign not in {-1, 1}:
            raise ValueError(f"{field}.sign must be -1 or 1")
        lower = _finite(data["lower_rad"], field=f"{field}.lower_rad")
        upper = _finite(data["upper_rad"], field=f"{field}.upper_rad")
        if upper <= lower:
            raise ValueError(f"{field} upper_rad must exceed lower_rad")
        velocity = _finite(
            data["max_velocity_rad_s"],
            field=f"{field}.max_velocity_rad_s",
        )
        if velocity <= 0.0:
            raise ValueError(f"{field}.max_velocity_rad_s must be positive")
        unit = _string(data["unit"], field=f"{field}.unit")
        if unit != "rad":
            raise ValueError(f"{field}.unit must be rad")
        return cls(
            global_index=_integer(data["global_index"], field=f"{field}.global_index"),
            canonical_name=_identifier(
                data["canonical_name"],
                field=f"{field}.canonical_name",
            ),
            side=side,
            group=group,
            group_index=_integer(data["group_index"], field=f"{field}.group_index"),
            source_instance_id=_identifier(
                data["source_instance_id"],
                field=f"{field}.source_instance_id",
            ),
            source_group_id=_identifier(
                data["source_group_id"],
                field=f"{field}.source_group_id",
            ),
            source_joint_name=_identifier(
                data["source_joint_name"],
                field=f"{field}.source_joint_name",
            ),
            source_index_q27=_integer(
                data["source_index_q27"],
                field=f"{field}.source_index_q27",
            ),
            source_profile_id=_identifier(
                data["source_profile_id"],
                field=f"{field}.source_profile_id",
            ),
            unit=unit,
            sign=sign,
            zero_offset_rad=_finite(
                data["zero_offset_rad"],
                field=f"{field}.zero_offset_rad",
            ),
            lower_rad=lower,
            upper_rad=upper,
            max_velocity_rad_s=velocity,
            fact_scope=_identifier(data["fact_scope"], field=f"{field}.fact_scope"),
            real_hardware_mapping_status=_identifier(
                data["real_hardware_mapping_status"],
                field=f"{field}.real_hardware_mapping_status",
            ),
        )


@dataclass(frozen=True, slots=True)
class Q54JointProfile:
    profile_id: str
    status: str
    dimension: int
    order_contract: str
    source_profiles: tuple[SourceProfilePin, ...]
    joints: tuple[Q54JointSpec, ...]
    file_sha256: str

    def __post_init__(self) -> None:
        if self.dimension != 54 or len(self.joints) != 54:
            raise ValueError("q54 profile must contain exactly 54 joints")
        indices = tuple(joint.global_index for joint in self.joints)
        if indices != tuple(range(54)):
            raise ValueError("q54 global indices must be contiguous and ordered 0..53")
        names = tuple(joint.canonical_name for joint in self.joints)
        if len(set(names)) != 54:
            raise ValueError("q54 canonical names must be unique")
        source_profile_ids = {profile.profile_id for profile in self.source_profiles}
        if {joint.source_profile_id for joint in self.joints} != source_profile_ids:
            raise ValueError("q54 joints must use every and only pinned source profile")
        expected_blocks = (
            ("left", "arm", 7),
            ("left", "hand", 20),
            ("right", "arm", 7),
            ("right", "hand", 20),
        )
        cursor = 0
        for side, group, size in expected_blocks:
            block = self.joints[cursor : cursor + size]
            if any(item.side != side or item.group != group for item in block):
                raise ValueError("q54 order must be left arm, left hand, right arm, right hand")
            if tuple(item.group_index for item in block) != tuple(range(size)):
                raise ValueError(f"{side}.{group} group indices must be contiguous")
            cursor += size
        for side in _SIDES:
            source_names = tuple(
                joint.source_joint_name for joint in self.joints if joint.side == side
            )
            if len(set(source_names)) != 27:
                raise ValueError(f"{side} q27 source joint names must be unique")
            source_indices = tuple(
                joint.source_index_q27 for joint in self.joints if joint.side == side
            )
            if sorted(source_indices) != list(range(27)):
                raise ValueError(f"{side} q27 source indices must be one permutation of 0..26")

    @property
    def canonical_names(self) -> tuple[str, ...]:
        return tuple(item.canonical_name for item in self.joints)

    def assemble_from_q27(
        self,
        *,
        left_q27_rad: Sequence[float],
        right_q27_rad: Sequence[float],
    ) -> tuple[float, ...]:
        side_values = {"left": left_q27_rad, "right": right_q27_rad}
        result: list[float] = []
        for joint in self.joints:
            values = side_values[joint.side]
            if len(values) != 27:
                raise ValueError(f"{joint.side}_q27_rad must contain exactly 27 values")
            value = _finite(
                values[joint.source_index_q27],
                field=f"{joint.side}_q27_rad[{joint.source_index_q27}]",
            )
            result.append(joint.sign * value + joint.zero_offset_rad)
        return tuple(result)

    def assemble_velocity_from_q27(
        self,
        *,
        left_qdot27_rad_s: Sequence[float],
        right_qdot27_rad_s: Sequence[float],
    ) -> tuple[float, ...]:
        """Apply only the canonical sign transform to qdot; offsets never apply."""

        side_values = {
            "left": left_qdot27_rad_s,
            "right": right_qdot27_rad_s,
        }
        result: list[float] = []
        for joint in self.joints:
            values = side_values[joint.side]
            if len(values) != 27:
                raise ValueError(f"{joint.side}_qdot27_rad_s must contain exactly 27 values")
            value = _finite(
                values[joint.source_index_q27],
                field=(f"{joint.side}_qdot27_rad_s[{joint.source_index_q27}]"),
            )
            result.append(joint.sign * value)
        return tuple(result)

    def decompose_to_q27(
        self,
        q54_rad: Sequence[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Invert the canonical sign/zero transform into left/right source q27."""

        if len(q54_rad) != 54:
            raise ValueError("q54_rad must contain exactly 54 values")
        sides = {"left": [0.0] * 27, "right": [0.0] * 27}
        for joint, canonical in zip(self.joints, q54_rad, strict=True):
            value = _finite(canonical, field=f"q54_rad[{joint.global_index}]")
            sides[joint.side][joint.source_index_q27] = (value - joint.zero_offset_rad) / joint.sign
        return tuple(sides["left"]), tuple(sides["right"])

    def decompose_velocity_to_q27(
        self,
        qdot54_rad_s: Sequence[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        if len(qdot54_rad_s) != 54:
            raise ValueError("qdot54_rad_s must contain exactly 54 values")
        sides = {"left": [0.0] * 27, "right": [0.0] * 27}
        for joint, canonical in zip(self.joints, qdot54_rad_s, strict=True):
            value = _finite(
                canonical,
                field=f"qdot54_rad_s[{joint.global_index}]",
            )
            sides[joint.side][joint.source_index_q27] = value / joint.sign
        return tuple(sides["left"]), tuple(sides["right"])

    def reorder_runtime_positions(
        self,
        *,
        left_names: Sequence[str],
        left_positions_rad: Sequence[float],
        right_names: Sequence[str],
        right_positions_rad: Sequence[float],
    ) -> tuple[float, ...]:
        runtime = {
            "left": self._runtime_map(left_names, left_positions_rad, side="left"),
            "right": self._runtime_map(right_names, right_positions_rad, side="right"),
        }
        return tuple(
            joint.sign * runtime[joint.side][joint.source_joint_name] + joint.zero_offset_rad
            for joint in self.joints
        )

    def _runtime_map(
        self,
        names: Sequence[str],
        positions: Sequence[float],
        *,
        side: str,
    ) -> dict[str, float]:
        if len(names) != 27 or len(positions) != 27:
            raise ValueError(f"{side} runtime inventory must contain exactly 27 joints")
        if len(set(names)) != 27:
            raise ValueError(f"{side} runtime joint names must be unique")
        expected = {joint.source_joint_name for joint in self.joints if joint.side == side}
        actual = set(names)
        if actual != expected:
            raise ValueError(
                f"{side} runtime joint inventory differs: "
                f"missing={sorted(expected - actual)}, unexpected={sorted(actual - expected)}"
            )
        return {
            name: _finite(value, field=f"{side}.{name}")
            for name, value in zip(names, positions, strict=True)
        }

    def validate_runtime_inventory(
        self,
        *,
        left_names: Sequence[str],
        left_limits_rad: Sequence[Sequence[float]],
        right_names: Sequence[str],
        right_limits_rad: Sequence[Sequence[float]],
        absolute_tolerance_rad: float = 1e-4,
    ) -> Q54RuntimeInventory:
        """Close runtime name/index/limit facts before an episode can be READY."""

        tolerance = _finite(
            absolute_tolerance_rad,
            field="absolute_tolerance_rad",
        )
        if tolerance <= 0.0:
            raise ValueError("absolute_tolerance_rad must be positive")
        sides = {
            "left": (tuple(left_names), tuple(left_limits_rad)),
            "right": (tuple(right_names), tuple(right_limits_rad)),
        }
        canonical_source_indices: list[int] = []
        runtime_limits: list[tuple[float, float]] = []
        for side, (names, limits) in sides.items():
            if len(names) != 27 or len(limits) != 27:
                raise ValueError(f"{side} runtime inventory must contain exactly 27 joints")
            if len(set(names)) != 27:
                raise ValueError(f"{side} runtime joint names must be unique")
            expected_names = {
                joint.source_joint_name for joint in self.joints if joint.side == side
            }
            if set(names) != expected_names:
                raise ValueError(
                    f"{side} runtime joint inventory differs: "
                    f"missing={sorted(expected_names - set(names))}, "
                    f"unexpected={sorted(set(names) - expected_names)}"
                )
            index_by_name = {name: index for index, name in enumerate(names)}
            for joint in (item for item in self.joints if item.side == side):
                runtime_index = index_by_name[joint.source_joint_name]
                if runtime_index != joint.source_index_q27:
                    raise ValueError(
                        f"{side}.{joint.source_joint_name} runtime index differs: "
                        f"expected={joint.source_index_q27}, actual={runtime_index}"
                    )
                limit = tuple(limits[runtime_index])
                if len(limit) != 2:
                    raise ValueError(f"{side}.{joint.source_joint_name} limit must have two values")
                actual_lower = _finite(
                    limit[0],
                    field=f"{side}.{joint.source_joint_name}.lower",
                )
                actual_upper = _finite(
                    limit[1],
                    field=f"{side}.{joint.source_joint_name}.upper",
                )
                expected_runtime = (
                    (
                        joint.lower_rad - joint.zero_offset_rad,
                        joint.upper_rad - joint.zero_offset_rad,
                    )
                    if joint.sign == 1
                    else (
                        joint.zero_offset_rad - joint.upper_rad,
                        joint.zero_offset_rad - joint.lower_rad,
                    )
                )
                if not all(
                    math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance)
                    for actual, expected in zip(
                        (actual_lower, actual_upper),
                        expected_runtime,
                        strict=True,
                    )
                ):
                    raise ValueError(
                        f"{side}.{joint.source_joint_name} runtime limits differ: "
                        f"expected={expected_runtime}, "
                        f"actual={(actual_lower, actual_upper)}"
                    )
                canonical_source_indices.append(runtime_index)
                runtime_limits.append((actual_lower, actual_upper))
        return Q54RuntimeInventory(
            profile_id=self.profile_id,
            profile_sha256=self.file_sha256,
            canonical_names=self.canonical_names,
            left_runtime_names=tuple(left_names),
            right_runtime_names=tuple(right_names),
            canonical_source_indices=tuple(canonical_source_indices),
            runtime_limits_rad=tuple(runtime_limits),
        )


@dataclass(frozen=True, slots=True)
class Q54RuntimeInventory:
    profile_id: str
    profile_sha256: str
    canonical_names: tuple[str, ...]
    left_runtime_names: tuple[str, ...]
    right_runtime_names: tuple[str, ...]
    canonical_source_indices: tuple[int, ...]
    runtime_limits_rad: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        if len(self.profile_sha256) != 64 or _SHA256.fullmatch(self.profile_sha256) is None:
            raise ValueError("q54 runtime inventory profile hash is invalid")
        if (
            len(self.canonical_names) != 54
            or len(set(self.canonical_names)) != 54
            or len(self.left_runtime_names) != 27
            or len(self.right_runtime_names) != 27
            or len(self.canonical_source_indices) != 54
            or len(self.runtime_limits_rad) != 54
        ):
            raise ValueError("q54 runtime inventory dimensions differ")

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": "wujihand.q54_runtime_inventory.v1",
            "profile_id": self.profile_id,
            "profile_sha256": self.profile_sha256,
            "canonical_names": list(self.canonical_names),
            "left_runtime_names": list(self.left_runtime_names),
            "right_runtime_names": list(self.right_runtime_names),
            "canonical_source_indices": list(self.canonical_source_indices),
            "runtime_limits_rad": [list(value) for value in self.runtime_limits_rad],
        }


def load_q54_joint_profile(
    project_root: str | Path,
    reference: str | Path,
) -> Q54JointProfile:
    root = Path(project_root).resolve()
    if not root.is_dir():
        raise ValueError(f"project root is not a directory: {root}")
    raw_reference = Path(reference)
    path = (
        raw_reference.resolve() if raw_reference.is_absolute() else (root / raw_reference).resolve()
    )
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError("q54 profile escapes project root") from exc
    if not path.is_file():
        raise FileNotFoundError(f"q54 profile not found: {path}")
    document = load_yaml_strict(path.read_text(encoding="utf-8"))
    data = _exact(
        document,
        keys=frozenset(
            {
                "schema",
                "profile_id",
                "status",
                "dimension",
                "order_contract",
                "source_profiles",
                "joints",
            }
        ),
        field="q54 profile",
    )
    if data["schema"] != Q54_JOINT_PROFILE_SCHEMA:
        raise ValueError(f"q54 profile schema must be {Q54_JOINT_PROFILE_SCHEMA!r}")
    pins = tuple(
        SourceProfilePin.from_mapping(item, field=f"source_profiles[{index}]")
        for index, item in enumerate(_sequence(data["source_profiles"], field="source_profiles"))
    )
    if len({pin.profile_id for pin in pins}) != len(pins):
        raise ValueError("source profile IDs must be unique")
    for pin in pins:
        source = _resolve(root, pin.path, field=f"source profile {pin.profile_id}")
        actual = sha256_file(source)
        if actual != pin.sha256:
            raise ValueError(
                f"source profile hash differs for {pin.profile_id}: "
                f"expected={pin.sha256}, actual={actual}"
            )
    joints = tuple(
        Q54JointSpec.from_mapping(item, field=f"joints[{index}]")
        for index, item in enumerate(_sequence(data["joints"], field="joints"))
    )
    return Q54JointProfile(
        profile_id=_identifier(data["profile_id"], field="profile_id"),
        status=_identifier(data["status"], field="status"),
        dimension=_integer(data["dimension"], field="dimension"),
        order_contract=_identifier(data["order_contract"], field="order_contract"),
        source_profiles=pins,
        joints=joints,
        file_sha256=sha256_file(path),
    )


def load_mini_dataset_profile(
    project_root: str | Path,
    reference: str | Path,
) -> MiniDatasetProfile:
    root = Path(project_root).resolve()
    if not root.is_dir():
        raise ValueError(f"project root is not a directory: {root}")
    raw_reference = Path(reference)
    path = (
        raw_reference.resolve() if raw_reference.is_absolute() else (root / raw_reference).resolve()
    )
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError("mini dataset profile escapes project root") from exc
    if not path.is_file():
        raise FileNotFoundError(f"mini dataset profile not found: {path}")
    document = load_yaml_strict(path.read_text(encoding="utf-8"))
    data = _exact(
        document,
        keys=frozenset(
            {
                "schema",
                "profile_id",
                "status",
                "robot_configuration",
                "joint_profile",
                "timing",
                "episode",
                "cameras",
                "vision_storage",
                "raw_capabilities",
                "lerobot",
                "collection",
                "release_gates",
            }
        ),
        field="mini dataset profile",
    )
    if data["schema"] != MINI_DATASET_PROFILE_SCHEMA:
        raise ValueError(f"mini dataset profile schema must be {MINI_DATASET_PROFILE_SCHEMA!r}")
    joint_pin = DatasetConfigPin.from_mapping(data["joint_profile"], field="joint_profile")
    _load_pinned_mapping(root, joint_pin, field="joint profile")
    q54 = load_q54_joint_profile(root, joint_pin.path)
    if q54.profile_id != joint_pin.expected_id or q54.file_sha256 != joint_pin.sha256:
        raise ValueError("joint profile identity differs from its pin")

    timing = _exact(
        data["timing"],
        keys=frozenset(
            {
                "physics_hz",
                "control_hz",
                "gui_preview_hz",
                "policy_fps",
                "selection",
                "observation_phase",
            }
        ),
        field="timing",
    )
    if timing["selection"] != "relative_even_control_index_no_interpolation_v1":
        raise ValueError("timing.selection must preserve exact relative-even selection")
    if timing["observation_phase"] != "pre_action":
        raise ValueError("timing.observation_phase must be pre_action")

    episode = _exact(
        data["episode"],
        keys=frozenset(
            {
                "identity",
                "current_stop_request",
                "require_recorder_ready",
                "require_live_inputs",
                "require_references",
                "require_scene_settled",
                "dispositions",
                "success_semantics",
            }
        ),
        field="episode",
    )
    required_episode_values = {
        "identity": "one_run_one_episode",
        "current_stop_request": "ctrl_c_complete_current_control_tick",
        "success_semantics": "absent",
    }
    if any(episode[key] != value for key, value in required_episode_values.items()):
        raise ValueError("episode identity, stop or success semantics differ")
    for key in (
        "require_recorder_ready",
        "require_live_inputs",
        "require_references",
        "require_scene_settled",
    ):
        if episode[key] is not True:
            raise ValueError(f"episode.{key} must be true")
    dispositions = tuple(
        _identifier(item, field=f"episode.dispositions[{index}]")
        for index, item in enumerate(
            _sequence(episode["dispositions"], field="episode.dispositions")
        )
    )
    if dispositions != ("accepted", "rejected", "incomplete"):
        raise ValueError("episode dispositions must not encode task success")

    cameras = tuple(
        DatasetCameraRole.from_mapping(item, field=f"cameras[{index}]")
        for index, item in enumerate(_sequence(data["cameras"], field="cameras"))
    )
    for camera in cameras:
        source = _load_pinned_mapping(
            root,
            camera.profile,
            field=f"camera profile {camera.logical_id}",
        )
        if source.get("simulation_only") is not True:
            raise ValueError(f"camera profile {camera.logical_id} must be simulation-only")
        warning = source.get("warning")
        if not isinstance(warning, str) or "SIMULATION ONLY" not in warning:
            raise ValueError(f"camera profile {camera.logical_id} lacks its simulation warning")
        capture = _mapping(source.get("capture"), field=f"camera {camera.logical_id}.capture")
        if (
            capture.get("width_px"),
            capture.get("height_px"),
            float(cast(float, capture.get("rate_hz"))),
        ) != (640, 480, 30.0):
            raise ValueError(f"camera profile {camera.logical_id} must be 640x480 at 30 Hz")

    vision = _exact(
        data["vision_storage"],
        keys=frozenset(
            {
                "truth_format",
                "width_px",
                "height_px",
                "atomic_publish",
                "per_frame_checksum",
            }
        ),
        field="vision_storage",
    )
    if (
        vision["truth_format"] != "png_rgb8_lossless"
        or vision["width_px"] != 640
        or vision["height_px"] != 480
        or vision["atomic_publish"] is not True
        or vision["per_frame_checksum"] is not True
    ):
        raise ValueError("vision storage must remain lossless RGB8 640x480 and atomic")

    raw = _exact(
        data["raw_capabilities"],
        keys=frozenset({"depth", "effort", "contact"}),
        field="raw_capabilities",
    )
    if raw != {
        "depth": "omitted",
        "effort": "omitted",
        "contact": "best_effort_non_blocking",
    }:
        raise ValueError("raw capability scope differs from the accepted plan")

    lerobot = _exact(
        data["lerobot"],
        keys=frozenset(
            {
                "version",
                "source",
                "tag",
                "commit",
                "python",
                "fps",
                "action_semantics",
                "publish_to_hub",
            }
        ),
        field="lerobot",
    )
    expected_lerobot = {
        "version": "0.6.1",
        "source": "https://github.com/huggingface/lerobot.git",
        "tag": "v0.6.1",
        "commit": "7e241bd630a3719a56157a497ce5d08f244784f1",
        "python": ">=3.12,<3.14",
        "fps": 30,
        "action_semantics": "absolute_q54_rad",
        "publish_to_hub": False,
    }
    if dict(lerobot) != expected_lerobot:
        raise ValueError("LeRobot source/runtime pin differs")

    collection = _exact(
        data["collection"],
        keys=frozenset(
            {
                "diagnostic_episodes",
                "formal_accepted_minimum",
                "formal_accepted_recommended_maximum",
                "retained_episode_hard_limit",
            }
        ),
        field="collection",
    )
    diagnostic = _integer(collection["diagnostic_episodes"], field="diagnostic_episodes")
    formal_min = _integer(
        collection["formal_accepted_minimum"],
        field="formal_accepted_minimum",
    )
    formal_max = _integer(
        collection["formal_accepted_recommended_maximum"],
        field="formal_accepted_recommended_maximum",
    )
    hard_limit = _integer(
        collection["retained_episode_hard_limit"],
        field="retained_episode_hard_limit",
    )
    if (diagnostic, formal_min, formal_max, hard_limit) != (2, 6, 12, 18):
        raise ValueError("collection size must remain 2 diagnostic, 6-12 formal, <=18 total")

    gates = _exact(
        data["release_gates"],
        keys=frozenset(
            {
                "control_rate_tolerance_fraction",
                "minimum_real_time_factor",
                "schedule_miss_limit",
                "maximum_input_age_ms",
                "fixture_translation_drift_limit_m",
                "fixture_rotation_drift_limit_rad",
                "require_lerobot_finalize_reopen",
            }
        ),
        field="release_gates",
    )
    if (
        gates["schedule_miss_limit"] != 0
        or gates["require_lerobot_finalize_reopen"] is not True
    ):
        raise ValueError("release gates require zero-miss strict grade and LeRobot round-trip")

    return MiniDatasetProfile(
        profile_id=_identifier(data["profile_id"], field="profile_id"),
        status=_identifier(data["status"], field="status"),
        robot_configuration=_identifier(
            data["robot_configuration"],
            field="robot_configuration",
        ),
        q54=q54,
        physics_hz=_integer(timing["physics_hz"], field="timing.physics_hz"),
        control_hz=_integer(timing["control_hz"], field="timing.control_hz"),
        gui_preview_hz=_integer(
            timing["gui_preview_hz"],
            field="timing.gui_preview_hz",
        ),
        policy_fps=_integer(timing["policy_fps"], field="timing.policy_fps"),
        cameras=cameras,
        lerobot_commit=cast(str, lerobot["commit"]),
        lerobot_python=cast(str, lerobot["python"]),
        retained_episode_hard_limit=hard_limit,
        release_control_rate_tolerance_fraction=_finite(
            gates["control_rate_tolerance_fraction"],
            field="release_gates.control_rate_tolerance_fraction",
        ),
        release_minimum_real_time_factor=_finite(
            gates["minimum_real_time_factor"],
            field="release_gates.minimum_real_time_factor",
        ),
        release_maximum_input_age_ms=_finite(
            gates["maximum_input_age_ms"],
            field="release_gates.maximum_input_age_ms",
        ),
        file_sha256=sha256_file(path),
    )


__all__ = [
    "MINI_DATASET_PROFILE_SCHEMA",
    "Q54_JOINT_PROFILE_SCHEMA",
    "DatasetCameraRole",
    "DatasetConfigPin",
    "MiniDatasetProfile",
    "Q54JointProfile",
    "Q54JointSpec",
    "Q54RuntimeInventory",
    "SourceProfilePin",
    "load_mini_dataset_profile",
    "load_q54_joint_profile",
]
