"""Transport-neutral dataset recording facts.

The live process only publishes immutable facts from this module.  Dataset
selection, metrics, rendering and model-specific transforms remain offline.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
from numbers import Real
from typing import ClassVar, Final, cast

from .recording import validate_recording_token, validate_run_id


DATASET_EPISODE_BOUNDARY_SCHEMA: Final = "wujihand.dataset_episode_boundary.v1"
SIMULATION_STATE_FRAME_SCHEMA: Final = "wujihand.simulation_state_frame.v1"


class DatasetEpisodeEvent(str, Enum):
    OPENED = "opened"
    READY = "ready"
    RECORDING = "recording"
    STOP_REQUESTED = "stop_requested"
    CLOSED = "closed"


class DatasetSourceMode(str, Enum):
    LIVE_TELEOPERATION = "live_teleoperation"
    SYNTHETIC_FIXTURE = "synthetic_fixture"
    REPLAY = "replay"


class SimulationFramePhase(str, Enum):
    PRE_ACTION = "pre_action"
    POST_ACTION = "post_action"


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _exact_mapping(
    value: object,
    *,
    field: str,
    keys: frozenset[str],
) -> Mapping[str, object]:
    result = _mapping(value, field=field)
    if frozenset(result) != keys:
        raise ValueError(f"{field} keys differ from the schema")
    return result


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be a string")
    return value


def _boolean(value: object, *, field: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{field} must be a boolean")
    return value


def _non_negative_int(value: object, *, field: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{field} must be a non-negative integer")
    return value


def _optional_non_negative_int(value: object, *, field: str) -> int | None:
    if value is None:
        return None
    return _non_negative_int(value, field=field)


def _finite_float(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{field} must be finite")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _optional_non_negative_float(value: object, *, field: str) -> float | None:
    if value is None:
        return None
    result = _finite_float(value, field=field)
    if result < 0.0:
        raise ValueError(f"{field} must be non-negative")
    return result


def _finite_vector(value: object, *, size: int, field: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Iterable):
        raise ValueError(f"{field} must contain {size} finite values")
    values = tuple(value)
    if len(values) != size:
        raise ValueError(f"{field} must contain {size} finite values")
    return tuple(
        _finite_float(item, field=f"{field}[{index}]") for index, item in enumerate(values)
    )


def _unit_quaternion(value: object, *, field: str) -> tuple[float, float, float, float]:
    values = _finite_vector(value, size=4, field=field)
    norm = math.sqrt(sum(item * item for item in values))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{field} must be a scalar-first unit quaternion")
    return values[0], values[1], values[2], values[3]


def _safe_prim_path(value: object, *, field: str) -> str:
    if type(value) is not str or not value.startswith("/") or ".." in value.split("/"):
        raise ValueError(f"{field} must be an absolute USD prim path")
    if len(value) > 512:
        raise ValueError(f"{field} must be at most 512 characters")
    return value


@dataclass(frozen=True, slots=True)
class DatasetEpisodeBoundary:
    run_id: str
    episode_id: str
    collection_id: str
    event: DatasetEpisodeEvent
    reason: str
    host_time_ns: int
    control_index: int | None
    tick_id: int | None
    simulation_time_s: float | None
    recorder_ready: bool
    inputs_ready: bool
    references_ready: bool
    scene_settled: bool
    source_mode: DatasetSourceMode
    dataset_eligible: bool
    requested_signal: int | None = None
    effective_final_control_index: int | None = None

    def __post_init__(self) -> None:
        validate_run_id(self.run_id)
        validate_run_id(self.episode_id, field="episode_id")
        validate_recording_token(self.collection_id, field="collection_id")
        if self.episode_id != self.run_id:
            raise ValueError("one-run-one-episode requires episode_id == run_id")
        if not isinstance(self.event, DatasetEpisodeEvent):
            raise ValueError("event must be a DatasetEpisodeEvent")
        if type(self.reason) is not str or not self.reason or len(self.reason) > 256:
            raise ValueError("reason must be a bounded non-empty string")
        for name, value in (
            ("recorder_ready", self.recorder_ready),
            ("inputs_ready", self.inputs_ready),
            ("references_ready", self.references_ready),
            ("scene_settled", self.scene_settled),
            ("dataset_eligible", self.dataset_eligible),
        ):
            _boolean(value, field=name)
        _non_negative_int(self.host_time_ns, field="host_time_ns")
        _optional_non_negative_int(self.control_index, field="control_index")
        _optional_non_negative_int(self.tick_id, field="tick_id")
        _optional_non_negative_float(self.simulation_time_s, field="simulation_time_s")
        if (self.control_index is None) != (self.tick_id is None):
            raise ValueError("control_index and tick_id must be present together")
        if not isinstance(self.source_mode, DatasetSourceMode):
            raise ValueError("source_mode must be a DatasetSourceMode")
        if self.source_mode is not DatasetSourceMode.LIVE_TELEOPERATION and self.dataset_eligible:
            raise ValueError("fixture and replay episodes cannot be dataset eligible")
        gates = (
            self.recorder_ready,
            self.inputs_ready,
            self.references_ready,
            self.scene_settled,
        )
        if self.event in {DatasetEpisodeEvent.READY, DatasetEpisodeEvent.RECORDING} and not all(
            gates
        ):
            raise ValueError("ready/recording requires every readiness gate")
        if self.event is DatasetEpisodeEvent.STOP_REQUESTED:
            if self.requested_signal is None or self.requested_signal <= 0:
                raise ValueError("stop_requested requires a positive requested_signal")
            if self.control_index is None or self.effective_final_control_index is None:
                raise ValueError("stop_requested requires the complete effective final tick")
            if self.control_index != self.effective_final_control_index:
                raise ValueError("stop_requested control_index must be the effective final index")
        elif self.requested_signal is not None:
            raise ValueError("requested_signal is only valid on stop_requested")
        if self.event is DatasetEpisodeEvent.CLOSED:
            if self.control_index is None or self.effective_final_control_index is None:
                raise ValueError("closed requires the effective final control index")
            if self.control_index != self.effective_final_control_index:
                raise ValueError("closed control_index must be the effective final index")

    @property
    def schema(self) -> str:
        return DATASET_EPISODE_BOUNDARY_SCHEMA

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "run_id": self.run_id,
            "episode_id": self.episode_id,
            "collection_id": self.collection_id,
            "event": self.event.value,
            "reason": self.reason,
            "host_time_ns": self.host_time_ns,
            "control_index": self.control_index,
            "tick_id": self.tick_id,
            "simulation_time_s": self.simulation_time_s,
            "recorder_ready": self.recorder_ready,
            "inputs_ready": self.inputs_ready,
            "references_ready": self.references_ready,
            "scene_settled": self.scene_settled,
            "source_mode": self.source_mode.value,
            "dataset_eligible": self.dataset_eligible,
            "requested_signal": self.requested_signal,
            "effective_final_control_index": self.effective_final_control_index,
        }

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "episode_boundary",
    ) -> DatasetEpisodeBoundary:
        keys = frozenset(
            {
                "schema",
                "run_id",
                "episode_id",
                "collection_id",
                "event",
                "reason",
                "host_time_ns",
                "control_index",
                "tick_id",
                "simulation_time_s",
                "recorder_ready",
                "inputs_ready",
                "references_ready",
                "scene_settled",
                "source_mode",
                "dataset_eligible",
                "requested_signal",
                "effective_final_control_index",
            }
        )
        data = _exact_mapping(value, field=field, keys=keys)
        if data["schema"] != DATASET_EPISODE_BOUNDARY_SCHEMA:
            raise ValueError(f"{field}.schema differs")
        try:
            event = DatasetEpisodeEvent(_string(data["event"], field=f"{field}.event"))
            source_mode = DatasetSourceMode(
                _string(data["source_mode"], field=f"{field}.source_mode")
            )
        except ValueError as exc:
            raise ValueError(f"{field} enum value differs") from exc
        return cls(
            run_id=_string(data["run_id"], field=f"{field}.run_id"),
            episode_id=_string(data["episode_id"], field=f"{field}.episode_id"),
            collection_id=_string(
                data["collection_id"],
                field=f"{field}.collection_id",
            ),
            event=event,
            reason=_string(data["reason"], field=f"{field}.reason"),
            host_time_ns=_non_negative_int(
                data["host_time_ns"],
                field=f"{field}.host_time_ns",
            ),
            control_index=_optional_non_negative_int(
                data["control_index"],
                field=f"{field}.control_index",
            ),
            tick_id=_optional_non_negative_int(
                data["tick_id"],
                field=f"{field}.tick_id",
            ),
            simulation_time_s=_optional_non_negative_float(
                data["simulation_time_s"],
                field=f"{field}.simulation_time_s",
            ),
            recorder_ready=_boolean(
                data["recorder_ready"],
                field=f"{field}.recorder_ready",
            ),
            inputs_ready=_boolean(
                data["inputs_ready"],
                field=f"{field}.inputs_ready",
            ),
            references_ready=_boolean(
                data["references_ready"],
                field=f"{field}.references_ready",
            ),
            scene_settled=_boolean(
                data["scene_settled"],
                field=f"{field}.scene_settled",
            ),
            source_mode=source_mode,
            dataset_eligible=_boolean(
                data["dataset_eligible"],
                field=f"{field}.dataset_eligible",
            ),
            requested_signal=_optional_non_negative_int(
                data["requested_signal"],
                field=f"{field}.requested_signal",
            ),
            effective_final_control_index=_optional_non_negative_int(
                data["effective_final_control_index"],
                field=f"{field}.effective_final_control_index",
            ),
        )


@dataclass(frozen=True, slots=True)
class DynamicRigidBodyTruth:
    logical_object_id: str
    prim_path: str
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    linear_velocity_m_s: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]
    sleeping: bool | None
    kinematic: bool
    valid: bool

    def __post_init__(self) -> None:
        validate_recording_token(self.logical_object_id, field="logical_object_id")
        _safe_prim_path(self.prim_path, field="prim_path")
        _finite_vector(self.position_m, size=3, field="position_m")
        if self.valid:
            _unit_quaternion(self.quat_wxyz, field="quat_wxyz")
        elif any(_finite_vector(self.quat_wxyz, size=4, field="quat_wxyz")):
            raise ValueError("invalid rigid-body truth must use a zero quaternion sentinel")
        _finite_vector(self.linear_velocity_m_s, size=3, field="linear_velocity_m_s")
        _finite_vector(
            self.angular_velocity_rad_s,
            size=3,
            field="angular_velocity_rad_s",
        )
        if self.sleeping is not None:
            _boolean(self.sleeping, field="sleeping")
        _boolean(self.kinematic, field="kinematic")
        _boolean(self.valid, field="valid")

    def to_mapping(self) -> dict[str, object]:
        return {
            "logical_object_id": self.logical_object_id,
            "prim_path": self.prim_path,
            "position_m": list(self.position_m),
            "quat_wxyz": list(self.quat_wxyz),
            "linear_velocity_m_s": list(self.linear_velocity_m_s),
            "angular_velocity_rad_s": list(self.angular_velocity_rad_s),
            "sleeping": self.sleeping,
            "kinematic": self.kinematic,
            "valid": self.valid,
        }

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "rigid_body",
    ) -> DynamicRigidBodyTruth:
        keys = frozenset(
            {
                "logical_object_id",
                "prim_path",
                "position_m",
                "quat_wxyz",
                "linear_velocity_m_s",
                "angular_velocity_rad_s",
                "sleeping",
                "kinematic",
                "valid",
            }
        )
        data = _exact_mapping(value, field=field, keys=keys)
        return cls(
            logical_object_id=_string(
                data["logical_object_id"],
                field=f"{field}.logical_object_id",
            ),
            prim_path=_string(data["prim_path"], field=f"{field}.prim_path"),
            position_m=cast(
                tuple[float, float, float],
                _finite_vector(data["position_m"], size=3, field=f"{field}.position_m"),
            ),
            quat_wxyz=cast(
                tuple[float, float, float, float],
                _finite_vector(data["quat_wxyz"], size=4, field=f"{field}.quat_wxyz"),
            ),
            linear_velocity_m_s=cast(
                tuple[float, float, float],
                _finite_vector(
                    data["linear_velocity_m_s"],
                    size=3,
                    field=f"{field}.linear_velocity_m_s",
                ),
            ),
            angular_velocity_rad_s=cast(
                tuple[float, float, float],
                _finite_vector(
                    data["angular_velocity_rad_s"],
                    size=3,
                    field=f"{field}.angular_velocity_rad_s",
                ),
            ),
            sleeping=(
                None
                if data["sleeping"] is None
                else _boolean(data["sleeping"], field=f"{field}.sleeping")
            ),
            kinematic=_boolean(data["kinematic"], field=f"{field}.kinematic"),
            valid=_boolean(data["valid"], field=f"{field}.valid"),
        )


@dataclass(frozen=True, slots=True)
class KinematicLinkTruth:
    side: str
    logical_link_id: str
    prim_path: str
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    valid: bool

    def __post_init__(self) -> None:
        if self.side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        validate_recording_token(self.logical_link_id, field="logical_link_id")
        _safe_prim_path(self.prim_path, field="prim_path")
        _finite_vector(self.position_m, size=3, field="position_m")
        if self.valid:
            _unit_quaternion(self.quat_wxyz, field="quat_wxyz")
        elif any(_finite_vector(self.quat_wxyz, size=4, field="quat_wxyz")):
            raise ValueError("invalid kinematic truth must use a zero quaternion sentinel")
        _boolean(self.valid, field="valid")

    def to_mapping(self) -> dict[str, object]:
        return {
            "side": self.side,
            "logical_link_id": self.logical_link_id,
            "prim_path": self.prim_path,
            "position_m": list(self.position_m),
            "quat_wxyz": list(self.quat_wxyz),
            "valid": self.valid,
        }

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "kinematic_link",
    ) -> KinematicLinkTruth:
        data = _exact_mapping(
            value,
            field=field,
            keys=frozenset(
                {
                    "side",
                    "logical_link_id",
                    "prim_path",
                    "position_m",
                    "quat_wxyz",
                    "valid",
                }
            ),
        )
        return cls(
            side=_string(data["side"], field=f"{field}.side"),
            logical_link_id=_string(
                data["logical_link_id"],
                field=f"{field}.logical_link_id",
            ),
            prim_path=_string(data["prim_path"], field=f"{field}.prim_path"),
            position_m=cast(
                tuple[float, float, float],
                _finite_vector(data["position_m"], size=3, field=f"{field}.position_m"),
            ),
            quat_wxyz=cast(
                tuple[float, float, float, float],
                _finite_vector(data["quat_wxyz"], size=4, field=f"{field}.quat_wxyz"),
            ),
            valid=_boolean(data["valid"], field=f"{field}.valid"),
        )


@dataclass(frozen=True, slots=True)
class SimulationStateFrame:
    state_source_api: ClassVar[str] = "isaac_articulation_usd_physics_v1"
    world_frame_id: ClassVar[str] = "world"
    quaternion_order: ClassVar[str] = "wxyz"
    joint_position_unit: ClassVar[str] = "rad"
    joint_velocity_unit: ClassVar[str] = "rad_s"
    angular_velocity_unit: ClassVar[str] = "rad_s"

    run_id: str
    episode_id: str
    control_index: int
    tick_id: int
    phase: SimulationFramePhase
    simulation_time_s: float
    physics_boundary_index: int
    q54_rad: tuple[float, ...]
    qdot54_rad_s: tuple[float, ...]
    rigid_bodies: tuple[DynamicRigidBodyTruth, ...]
    kinematic_links: tuple[KinematicLinkTruth, ...]
    expected_rigid_body_count: int
    expected_kinematic_link_count: int
    payload_digest_sha256: str

    def __post_init__(self) -> None:
        self._validate_facts()
        expected_digest = self.calculate_payload_digest()
        if self.payload_digest_sha256 != expected_digest:
            raise ValueError("payload_digest_sha256 does not match the frame facts")

    def _validate_facts(self) -> None:
        validate_run_id(self.run_id)
        validate_run_id(self.episode_id, field="episode_id")
        if self.episode_id != self.run_id:
            raise ValueError("one-run-one-episode requires episode_id == run_id")
        _non_negative_int(self.control_index, field="control_index")
        _non_negative_int(self.tick_id, field="tick_id")
        if self.control_index != self.tick_id:
            raise ValueError("current dataset contract requires control_index == tick_id")
        if not isinstance(self.phase, SimulationFramePhase):
            raise ValueError("phase must be a SimulationFramePhase")
        if _finite_float(self.simulation_time_s, field="simulation_time_s") < 0.0:
            raise ValueError("simulation_time_s must be non-negative")
        _non_negative_int(self.physics_boundary_index, field="physics_boundary_index")
        _finite_vector(self.q54_rad, size=54, field="q54_rad")
        _finite_vector(self.qdot54_rad_s, size=54, field="qdot54_rad_s")
        _non_negative_int(
            self.expected_rigid_body_count,
            field="expected_rigid_body_count",
        )
        _non_negative_int(
            self.expected_kinematic_link_count,
            field="expected_kinematic_link_count",
        )
        if len(self.rigid_bodies) != self.expected_rigid_body_count:
            raise ValueError("rigid-body closure count differs from inventory")
        if len(self.kinematic_links) != self.expected_kinematic_link_count:
            raise ValueError("kinematic-link closure count differs from inventory")
        object_ids = tuple(item.logical_object_id for item in self.rigid_bodies)
        if len(set(object_ids)) != len(object_ids):
            raise ValueError("rigid-body logical IDs must be unique")
        link_ids = tuple((item.side, item.logical_link_id) for item in self.kinematic_links)
        if len(set(link_ids)) != len(link_ids):
            raise ValueError("kinematic-link side/ID pairs must be unique")

    @classmethod
    def create(
        cls,
        *,
        run_id: str,
        episode_id: str,
        control_index: int,
        tick_id: int,
        phase: SimulationFramePhase,
        simulation_time_s: float,
        physics_boundary_index: int,
        q54_rad: Iterable[float],
        qdot54_rad_s: Iterable[float],
        rigid_bodies: Iterable[DynamicRigidBodyTruth],
        kinematic_links: Iterable[KinematicLinkTruth],
        expected_rigid_body_count: int,
        expected_kinematic_link_count: int,
    ) -> SimulationStateFrame:
        q54 = tuple(q54_rad)
        qdot54 = tuple(qdot54_rad_s)
        bodies = tuple(rigid_bodies)
        links = tuple(kinematic_links)
        provisional = cls.__new__(cls)
        object.__setattr__(provisional, "run_id", run_id)
        object.__setattr__(provisional, "episode_id", episode_id)
        object.__setattr__(provisional, "control_index", control_index)
        object.__setattr__(provisional, "tick_id", tick_id)
        object.__setattr__(provisional, "phase", phase)
        object.__setattr__(provisional, "simulation_time_s", simulation_time_s)
        object.__setattr__(provisional, "physics_boundary_index", physics_boundary_index)
        object.__setattr__(provisional, "q54_rad", q54)
        object.__setattr__(provisional, "qdot54_rad_s", qdot54)
        object.__setattr__(provisional, "rigid_bodies", bodies)
        object.__setattr__(provisional, "kinematic_links", links)
        object.__setattr__(
            provisional,
            "expected_rigid_body_count",
            expected_rigid_body_count,
        )
        object.__setattr__(
            provisional,
            "expected_kinematic_link_count",
            expected_kinematic_link_count,
        )
        object.__setattr__(provisional, "payload_digest_sha256", "")
        provisional._validate_facts()
        digest = provisional.calculate_payload_digest()
        object.__setattr__(provisional, "payload_digest_sha256", digest)
        return provisional

    @property
    def schema(self) -> str:
        return SIMULATION_STATE_FRAME_SCHEMA

    def _payload_mapping(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "episode_id": self.episode_id,
            "control_index": self.control_index,
            "tick_id": self.tick_id,
            "phase": self.phase.value,
            "simulation_time_s": self.simulation_time_s,
            "physics_boundary_index": self.physics_boundary_index,
            "state_source_api": self.state_source_api,
            "world_frame_id": self.world_frame_id,
            "quaternion_order": self.quaternion_order,
            "joint_position_unit": self.joint_position_unit,
            "joint_velocity_unit": self.joint_velocity_unit,
            "angular_velocity_unit": self.angular_velocity_unit,
            "q54_rad": list(self.q54_rad),
            "qdot54_rad_s": list(self.qdot54_rad_s),
            "rigid_bodies": [item.to_mapping() for item in self.rigid_bodies],
            "kinematic_links": [item.to_mapping() for item in self.kinematic_links],
            "expected_rigid_body_count": self.expected_rigid_body_count,
            "expected_kinematic_link_count": self.expected_kinematic_link_count,
        }

    def calculate_payload_digest(self) -> str:
        encoded = json.dumps(
            self._payload_mapping(),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def as_next_pre_action(
        self,
        *,
        control_index: int,
        simulation_time_s: float,
        physics_boundary_index: int,
        q54_rad: Iterable[float],
        qdot54_rad_s: Iterable[float],
    ) -> SimulationStateFrame:
        """Reuse an adjacent post-state only after exact no-advance closure."""

        q54, qdot54 = self.validate_next_pre_action(
            control_index=control_index,
            simulation_time_s=simulation_time_s,
            physics_boundary_index=physics_boundary_index,
            q54_rad=q54_rad,
            qdot54_rad_s=qdot54_rad_s,
        )
        return SimulationStateFrame.create(
            run_id=self.run_id,
            episode_id=self.episode_id,
            control_index=control_index,
            tick_id=control_index,
            phase=SimulationFramePhase.PRE_ACTION,
            simulation_time_s=simulation_time_s,
            physics_boundary_index=physics_boundary_index,
            q54_rad=q54,
            qdot54_rad_s=qdot54,
            rigid_bodies=self.rigid_bodies,
            kinematic_links=self.kinematic_links,
            expected_rigid_body_count=self.expected_rigid_body_count,
            expected_kinematic_link_count=self.expected_kinematic_link_count,
        )

    def validate_next_pre_action(
        self,
        *,
        control_index: int,
        simulation_time_s: float,
        physics_boundary_index: int,
        q54_rad: Iterable[float],
        qdot54_rad_s: Iterable[float],
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Validate exact adjacent reuse without constructing or hashing the frame."""

        q54 = tuple(q54_rad)
        qdot54 = tuple(qdot54_rad_s)
        if self.phase is not SimulationFramePhase.POST_ACTION:
            raise ValueError("only a post-action state can close the next pre-action state")
        if control_index != self.control_index + 1:
            raise ValueError("reused pre-action state must be control-index adjacent")
        if simulation_time_s != self.simulation_time_s:
            raise ValueError("reused pre-action state observed simulation-time advance")
        if physics_boundary_index != self.physics_boundary_index:
            raise ValueError("reused pre-action state observed physics-boundary advance")
        if q54 != self.q54_rad or qdot54 != self.qdot54_rad_s:
            raise ValueError("reused pre-action state differs from live q54 readback")
        return q54, qdot54

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            **self._payload_mapping(),
            "payload_digest_sha256": self.payload_digest_sha256,
        }

    @classmethod
    def from_mapping(
        cls,
        value: object,
        *,
        field: str = "simulation_state_frame",
    ) -> SimulationStateFrame:
        data = _exact_mapping(
            value,
            field=field,
            keys=frozenset(
                {
                    "schema",
                    "run_id",
                    "episode_id",
                    "control_index",
                    "tick_id",
                    "phase",
                    "simulation_time_s",
                    "physics_boundary_index",
                    "state_source_api",
                    "world_frame_id",
                    "quaternion_order",
                    "joint_position_unit",
                    "joint_velocity_unit",
                    "angular_velocity_unit",
                    "q54_rad",
                    "qdot54_rad_s",
                    "rigid_bodies",
                    "kinematic_links",
                    "expected_rigid_body_count",
                    "expected_kinematic_link_count",
                    "payload_digest_sha256",
                }
            ),
        )
        if data["schema"] != SIMULATION_STATE_FRAME_SCHEMA:
            raise ValueError(f"{field}.schema differs")
        fixed_literals = {
            "state_source_api": cls.state_source_api,
            "world_frame_id": cls.world_frame_id,
            "quaternion_order": cls.quaternion_order,
            "joint_position_unit": cls.joint_position_unit,
            "joint_velocity_unit": cls.joint_velocity_unit,
            "angular_velocity_unit": cls.angular_velocity_unit,
        }
        if any(data[key] != expected for key, expected in fixed_literals.items()):
            raise ValueError(f"{field} coordinate or unit convention differs")
        try:
            phase = SimulationFramePhase(_string(data["phase"], field=f"{field}.phase"))
        except ValueError as exc:
            raise ValueError(f"{field}.phase differs") from exc
        bodies_raw = _sequence(data["rigid_bodies"], field=f"{field}.rigid_bodies")
        links_raw = _sequence(
            data["kinematic_links"],
            field=f"{field}.kinematic_links",
        )
        return cls(
            run_id=_string(data["run_id"], field=f"{field}.run_id"),
            episode_id=_string(data["episode_id"], field=f"{field}.episode_id"),
            control_index=_non_negative_int(
                data["control_index"],
                field=f"{field}.control_index",
            ),
            tick_id=_non_negative_int(data["tick_id"], field=f"{field}.tick_id"),
            phase=phase,
            simulation_time_s=_finite_float(
                data["simulation_time_s"],
                field=f"{field}.simulation_time_s",
            ),
            physics_boundary_index=_non_negative_int(
                data["physics_boundary_index"],
                field=f"{field}.physics_boundary_index",
            ),
            q54_rad=_finite_vector(data["q54_rad"], size=54, field=f"{field}.q54_rad"),
            qdot54_rad_s=_finite_vector(
                data["qdot54_rad_s"],
                size=54,
                field=f"{field}.qdot54_rad_s",
            ),
            rigid_bodies=tuple(
                DynamicRigidBodyTruth.from_mapping(
                    item,
                    field=f"{field}.rigid_bodies[{index}]",
                )
                for index, item in enumerate(bodies_raw)
            ),
            kinematic_links=tuple(
                KinematicLinkTruth.from_mapping(
                    item,
                    field=f"{field}.kinematic_links[{index}]",
                )
                for index, item in enumerate(links_raw)
            ),
            expected_rigid_body_count=_non_negative_int(
                data["expected_rigid_body_count"],
                field=f"{field}.expected_rigid_body_count",
            ),
            expected_kinematic_link_count=_non_negative_int(
                data["expected_kinematic_link_count"],
                field=f"{field}.expected_kinematic_link_count",
            ),
            payload_digest_sha256=_string(
                data["payload_digest_sha256"],
                field=f"{field}.payload_digest_sha256",
            ),
        )


__all__ = [
    "DATASET_EPISODE_BOUNDARY_SCHEMA",
    "SIMULATION_STATE_FRAME_SCHEMA",
    "DatasetEpisodeBoundary",
    "DatasetEpisodeEvent",
    "DatasetSourceMode",
    "DynamicRigidBodyTruth",
    "KinematicLinkTruth",
    "SimulationFramePhase",
    "SimulationStateFrame",
]
