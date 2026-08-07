"""Integrity gates and non-blocking quality grades for one immutable episode."""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
import math
from typing import Final, cast

from wujihand.domain.recording import validate_run_id

from wujihand.domain.dataset_recording import (
    DatasetEpisodeBoundary,
    DatasetEpisodeEvent,
    DatasetSourceMode,
    SimulationFramePhase,
    SimulationStateFrame,
)

from .alignment import RawTransition
from .profile import Q54JointProfile


RELEASE_DECISION_SCHEMA: Final = "wujihand.dataset_release_decision.v3"
PREVIOUS_RELEASE_DECISION_SCHEMA: Final = "wujihand.dataset_release_decision.v2"
LEGACY_RELEASE_DECISION_SCHEMA: Final = "wujihand.dataset_release_decision.v1"
NORMALIZED_EPISODE_FACTS_SCHEMA: Final = "wujihand.normalized_episode_facts.v1"

GATE_SEVERITIES: Final = frozenset({"hard", "warning", "advisory"})
RELEASE_GRADES: Final = frozenset(
    {"strict_qualified", "usable_with_warnings", "rejected"}
)
QUALITY_GRADES: Final = ("A", "B", "C", "D")
RGB_CAMERA_IDS: Final = ("scene_rgb", "left_wrist_rgb", "right_wrist_rgb")

REQUIRED_ROUTE_FACTS: Final = frozenset(
    {
        "left.tracker.raw_selected",
        "left.tracker.raw_active",
        "left.glove.q21_selected",
        "left.glove.q21_active",
        "left.arm.q7_candidate",
        "left.hand.q20_intent",
        "left.applied.q27",
        "right.tracker.raw_selected",
        "right.tracker.raw_active",
        "right.glove.q21_selected",
        "right.glove.q21_active",
        "right.arm.q7_candidate",
        "right.hand.q20_intent",
        "right.applied.q27",
    }
)
REQUIRED_INTEGRITY_ROUTE_FACTS: Final = frozenset(
    {"left.applied.q27", "right.applied.q27"}
)


def _strict_sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


@dataclass(frozen=True, slots=True)
class ReleaseGateConfig:
    expected_control_hz: float = 60.0
    expected_physics_hz: float = 120.0
    control_rate_tolerance_fraction: float = 0.02
    minimum_real_time_factor: float = 0.95
    maximum_input_age_ms: float = 20.0
    q54_continuity_atol_rad: float = 1e-8
    simulation_time_atol_s: float = 1e-9
    fixture_translation_drift_limit_m: float = 1e-6
    fixture_rotation_drift_limit_rad: float = 1e-6
    maximum_missed_control_fraction: float = 0.005
    maximum_consecutive_missed_control_periods: int = 2
    maximum_control_interval_s: float = 0.05
    physics_grid_time_atol_s: float = 5e-6
    critical_gap_joint_velocity_fraction: float = 0.25
    critical_gap_object_linear_speed_m_s: float = 0.05
    critical_gap_object_proximity_m: float = 0.10

    def __post_init__(self) -> None:
        values = (
            self.expected_control_hz,
            self.expected_physics_hz,
            self.control_rate_tolerance_fraction,
            self.minimum_real_time_factor,
            self.maximum_input_age_ms,
            self.q54_continuity_atol_rad,
            self.simulation_time_atol_s,
            self.fixture_translation_drift_limit_m,
            self.fixture_rotation_drift_limit_rad,
            self.maximum_missed_control_fraction,
            self.maximum_control_interval_s,
            self.physics_grid_time_atol_s,
            self.critical_gap_joint_velocity_fraction,
            self.critical_gap_object_linear_speed_m_s,
            self.critical_gap_object_proximity_m,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("release gate configuration values must be finite and positive")
        if self.maximum_missed_control_fraction > 1.0:
            raise ValueError("maximum missed-control fraction must not exceed one")
        if (
            type(self.maximum_consecutive_missed_control_periods) is not int
            or self.maximum_consecutive_missed_control_periods < 1
        ):
            raise ValueError("maximum consecutive missed-control periods must be positive")


@dataclass(frozen=True, slots=True)
class EpisodeQualityConfig:
    """Versioned ABCD thresholds; no grade produced here is a release veto."""

    control_hz_lower: tuple[float, float, float] = (59.5, 58.0, 55.0)
    real_time_factor_lower: tuple[float, float, float] = (0.99, 0.97, 0.90)
    missed_fraction_upper: tuple[float, float, float] = (0.005, 0.02, 0.03)
    consecutive_misses_upper: tuple[float, float, float] = (1.0, 2.0, 5.0)
    maximum_interval_ms_upper: tuple[float, float, float] = (50.0, 100.0, 250.0)
    selected_missing_upper: tuple[float, float, float] = (0.01, 0.05, 0.15)
    active_missing_upper: tuple[float, float, float] = (0.01, 0.05, 0.15)
    q7_candidate_missing_upper: tuple[float, float, float] = (0.01, 0.05, 0.25)
    actionable_lower: tuple[float, float, float] = (0.95, 0.80, 0.70)
    input_age_p95_ms_upper: tuple[float, float, float] = (20.0, 50.0, 100.0)
    critical_gap_fraction_upper: tuple[float, float, float] = (0.0, 0.005, 0.03)
    rate_limit_fraction_upper: tuple[float, float, float] = (0.05, 0.15, 0.30)
    clamp_fraction_upper: tuple[float, float, float] = (0.01, 0.05, 0.15)

    def __post_init__(self) -> None:
        lower = (self.control_hz_lower, self.real_time_factor_lower, self.actionable_lower)
        upper = (
            self.missed_fraction_upper,
            self.consecutive_misses_upper,
            self.maximum_interval_ms_upper,
            self.selected_missing_upper,
            self.active_missing_upper,
            self.q7_candidate_missing_upper,
            self.input_age_p95_ms_upper,
            self.critical_gap_fraction_upper,
            self.rate_limit_fraction_upper,
            self.clamp_fraction_upper,
        )
        if any(
            len(values) != 3 or any(not math.isfinite(value) or value < 0.0 for value in values)
            for values in (*lower, *upper)
        ):
            raise ValueError("episode quality thresholds must contain three finite values")
        if any(not first > second > third for first, second, third in lower):
            raise ValueError("lower-is-better quality thresholds are not strictly descending")
        if any(not first < second < third for first, second, third in upper):
            raise ValueError("upper-is-better quality thresholds are not strictly ascending")


@dataclass(frozen=True, slots=True)
class SourceEpochFact:
    source_id: str
    producer_instance: str
    transport_epoch: int

    def __post_init__(self) -> None:
        if not self.source_id or not self.producer_instance or self.transport_epoch < 0:
            raise ValueError("source epoch facts must be non-empty and non-negative")

    def to_mapping(self) -> dict[str, object]:
        return {
            "source_id": self.source_id,
            "producer_instance": self.producer_instance,
            "transport_epoch": self.transport_epoch,
        }

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> SourceEpochFact:
        if not isinstance(value, Mapping) or frozenset(value) != {
            "source_id",
            "producer_instance",
            "transport_epoch",
        }:
            raise ValueError(f"{field} keys differ")
        data = cast(Mapping[str, object], value)
        if (
            not isinstance(data["source_id"], str)
            or not isinstance(data["producer_instance"], str)
            or type(data["transport_epoch"]) is not int
        ):
            raise ValueError(f"{field} types differ")
        return cls(
            source_id=data["source_id"],
            producer_instance=data["producer_instance"],
            transport_epoch=data["transport_epoch"],
        )


@dataclass(frozen=True, slots=True)
class ControlTickFacts:
    transition: RawTransition
    tick_time_ns: int
    schedule_slot: int
    missed_control_periods_before_tick: int
    physics_substep_indices: tuple[int, int]
    route_fact_keys: frozenset[str]
    source_epochs: tuple[SourceEpochFact, ...]
    comparable_input_age_ms: tuple[tuple[str, float], ...]
    pre_action_frame: SimulationStateFrame
    post_action_frame: SimulationStateFrame

    def __post_init__(self) -> None:
        if self.tick_time_ns < 0 or self.schedule_slot < 0:
            raise ValueError("tick_time_ns and schedule_slot must be non-negative")
        if self.missed_control_periods_before_tick < 0:
            raise ValueError("missed_control_periods_before_tick must be non-negative")
        first, second = self.physics_substep_indices
        if first < 0 or second != first + 1:
            raise ValueError("each control tick must contain two consecutive physics substeps")
        if len({fact.source_id for fact in self.source_epochs}) != len(self.source_epochs):
            raise ValueError("source epoch facts must contain unique source IDs")
        age_keys = tuple(key for key, _ in self.comparable_input_age_ms)
        if len(set(age_keys)) != len(age_keys):
            raise ValueError("input age facts must have unique keys")
        if any(
            not math.isfinite(value) or value < 0.0 for _, value in self.comparable_input_age_ms
        ):
            raise ValueError("input ages must be finite and non-negative")
        for phase, frame in (
            (SimulationFramePhase.PRE_ACTION, self.pre_action_frame),
            (SimulationFramePhase.POST_ACTION, self.post_action_frame),
        ):
            if frame.phase is not phase:
                raise ValueError(f"{phase.value} frame has the wrong phase")
            if frame.run_id != self.transition.run_id:
                raise ValueError("state frame and transition run IDs differ")
            if frame.control_index != self.transition.control_index:
                raise ValueError("state frame and transition control indices differ")

    def to_mapping(self) -> dict[str, object]:
        return {
            "transition": self.transition.to_mapping(),
            "tick_time_ns": self.tick_time_ns,
            "schedule_slot": self.schedule_slot,
            "missed_control_periods_before_tick": (self.missed_control_periods_before_tick),
            "physics_substep_indices": list(self.physics_substep_indices),
            "route_fact_keys": sorted(self.route_fact_keys),
            "source_epochs": [item.to_mapping() for item in self.source_epochs],
            "comparable_input_age_ms": [
                [key, value] for key, value in self.comparable_input_age_ms
            ],
            "pre_action_frame": self.pre_action_frame.to_mapping(),
            "post_action_frame": self.post_action_frame.to_mapping(),
        }

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> ControlTickFacts:
        expected = frozenset(
            {
                "transition",
                "tick_time_ns",
                "schedule_slot",
                "missed_control_periods_before_tick",
                "physics_substep_indices",
                "route_fact_keys",
                "source_epochs",
                "comparable_input_age_ms",
                "pre_action_frame",
                "post_action_frame",
            }
        )
        if not isinstance(value, Mapping) or frozenset(value) != expected:
            raise ValueError(f"{field} keys differ")
        data = cast(Mapping[str, object], value)
        integer_fields = (
            "tick_time_ns",
            "schedule_slot",
            "missed_control_periods_before_tick",
        )
        if any(type(data[key]) is not int for key in integer_fields):
            raise ValueError(f"{field} integer types differ")
        indices = _strict_sequence(data["physics_substep_indices"], field=f"{field}.indices")
        if len(indices) != 2 or any(type(item) is not int for item in indices):
            raise ValueError(f"{field}.physics_substep_indices differ")
        raw_keys = _strict_sequence(data["route_fact_keys"], field=f"{field}.route_fact_keys")
        if any(not isinstance(item, str) for item in raw_keys):
            raise ValueError(f"{field}.route_fact_keys types differ")
        raw_epochs = _strict_sequence(data["source_epochs"], field=f"{field}.source_epochs")
        raw_ages = _strict_sequence(
            data["comparable_input_age_ms"],
            field=f"{field}.comparable_input_age_ms",
        )
        ages: list[tuple[str, float]] = []
        for index, raw in enumerate(raw_ages):
            pair = _strict_sequence(raw, field=f"{field}.ages[{index}]")
            if len(pair) != 2 or not isinstance(pair[0], str):
                raise ValueError(f"{field}.ages[{index}] differs")
            number = pair[1]
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise ValueError(f"{field}.ages[{index}] value differs")
            ages.append((pair[0], float(number)))
        return cls(
            transition=RawTransition.from_mapping(
                data["transition"],
                field=f"{field}.transition",
            ),
            tick_time_ns=cast(int, data["tick_time_ns"]),
            schedule_slot=cast(int, data["schedule_slot"]),
            missed_control_periods_before_tick=cast(
                int,
                data["missed_control_periods_before_tick"],
            ),
            physics_substep_indices=cast(tuple[int, int], tuple(indices)),
            route_fact_keys=frozenset(cast(Sequence[str], raw_keys)),
            source_epochs=tuple(
                SourceEpochFact.from_mapping(item, field=f"{field}.source_epochs[{index}]")
                for index, item in enumerate(raw_epochs)
            ),
            comparable_input_age_ms=tuple(ages),
            pre_action_frame=SimulationStateFrame.from_mapping(
                data["pre_action_frame"],
                field=f"{field}.pre_action_frame",
            ),
            post_action_frame=SimulationStateFrame.from_mapping(
                data["post_action_frame"],
                field=f"{field}.post_action_frame",
            ),
        )


@dataclass(frozen=True, slots=True)
class NormalizedEpisodeFacts:
    run_id: str
    boundaries: tuple[DatasetEpisodeBoundary, ...]
    ticks: tuple[ControlTickFacts, ...]
    q54_profile_id: str
    q54_profile_sha256: str
    q54_runtime_names: tuple[str, ...]
    artifact_complete: bool
    checksums_verified: bool
    recorder_inventory_complete: bool
    unknown_schemas: tuple[str, ...]
    fixture_translation_drift_m: float
    fixture_rotation_drift_rad: float

    def __post_init__(self) -> None:
        validate_run_id(self.run_id)
        if not self.q54_profile_id or len(self.q54_profile_sha256) != 64:
            raise ValueError("q54 profile identity must be complete")
        if any(
            not math.isfinite(value) or value < 0.0
            for value in (
                self.fixture_translation_drift_m,
                self.fixture_rotation_drift_rad,
            )
        ):
            raise ValueError("fixture drift must be finite and non-negative")

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": NORMALIZED_EPISODE_FACTS_SCHEMA,
            "run_id": self.run_id,
            "boundaries": [item.to_mapping() for item in self.boundaries],
            "ticks": [item.to_mapping() for item in self.ticks],
            "q54_profile_id": self.q54_profile_id,
            "q54_profile_sha256": self.q54_profile_sha256,
            "q54_runtime_names": list(self.q54_runtime_names),
            "artifact_complete": self.artifact_complete,
            "checksums_verified": self.checksums_verified,
            "recorder_inventory_complete": self.recorder_inventory_complete,
            "unknown_schemas": list(self.unknown_schemas),
            "fixture_translation_drift_m": self.fixture_translation_drift_m,
            "fixture_rotation_drift_rad": self.fixture_rotation_drift_rad,
        }

    @classmethod
    def from_mapping(cls, value: object) -> NormalizedEpisodeFacts:
        expected = frozenset(
            {
                "schema",
                "run_id",
                "boundaries",
                "ticks",
                "q54_profile_id",
                "q54_profile_sha256",
                "q54_runtime_names",
                "artifact_complete",
                "checksums_verified",
                "recorder_inventory_complete",
                "unknown_schemas",
                "fixture_translation_drift_m",
                "fixture_rotation_drift_rad",
            }
        )
        if not isinstance(value, Mapping) or frozenset(value) != expected:
            raise ValueError("normalized episode facts keys differ")
        data = cast(Mapping[str, object], value)
        if data["schema"] != NORMALIZED_EPISODE_FACTS_SCHEMA:
            raise ValueError("normalized episode facts schema differs")
        string_fields = ("run_id", "q54_profile_id", "q54_profile_sha256")
        if any(not isinstance(data[key], str) for key in string_fields):
            raise ValueError("normalized episode fact string types differ")
        boolean_fields = (
            "artifact_complete",
            "checksums_verified",
            "recorder_inventory_complete",
        )
        if any(type(data[key]) is not bool for key in boolean_fields):
            raise ValueError("normalized episode fact boolean types differ")
        boundaries = _strict_sequence(data["boundaries"], field="boundaries")
        ticks = _strict_sequence(data["ticks"], field="ticks")
        names = _strict_sequence(data["q54_runtime_names"], field="q54_runtime_names")
        schemas = _strict_sequence(data["unknown_schemas"], field="unknown_schemas")
        if any(not isinstance(item, str) for item in (*names, *schemas)):
            raise ValueError("normalized episode sequence string types differ")
        drift: list[float] = []
        for key in ("fixture_translation_drift_m", "fixture_rotation_drift_rad"):
            item = data[key]
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ValueError(f"{key} must be numeric")
            drift.append(float(item))
        return cls(
            run_id=cast(str, data["run_id"]),
            boundaries=tuple(
                DatasetEpisodeBoundary.from_mapping(item, field=f"boundaries[{index}]")
                for index, item in enumerate(boundaries)
            ),
            ticks=tuple(
                ControlTickFacts.from_mapping(item, field=f"ticks[{index}]")
                for index, item in enumerate(ticks)
            ),
            q54_profile_id=cast(str, data["q54_profile_id"]),
            q54_profile_sha256=cast(str, data["q54_profile_sha256"]),
            q54_runtime_names=tuple(cast(Sequence[str], names)),
            artifact_complete=cast(bool, data["artifact_complete"]),
            checksums_verified=cast(bool, data["checksums_verified"]),
            recorder_inventory_complete=cast(bool, data["recorder_inventory_complete"]),
            unknown_schemas=tuple(cast(Sequence[str], schemas)),
            fixture_translation_drift_m=drift[0],
            fixture_rotation_drift_rad=drift[1],
        )


@dataclass(frozen=True, slots=True)
class ReleaseGateResult:
    name: str
    passed: bool
    expected: object
    observed: object
    reason: str
    severity: str = "hard"

    def __post_init__(self) -> None:
        if self.severity not in GATE_SEVERITIES:
            raise ValueError("release gate severity differs")

    def to_mapping(self) -> dict[str, object]:
        return {
            "name": self.name,
            "passed": self.passed,
            "expected": self.expected,
            "observed": self.observed,
            "reason": self.reason,
            "severity": self.severity,
        }

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> ReleaseGateResult:
        if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
            raise ValueError(f"{field} must be a string-keyed mapping")
        data = cast(Mapping[str, object], value)
        keys = frozenset(data)
        legacy_keys = {"name", "passed", "expected", "observed", "reason"}
        if keys not in {frozenset(legacy_keys), frozenset((*legacy_keys, "severity"))}:
            raise ValueError(f"{field} keys differ")
        if not isinstance(data["name"], str) or not data["name"]:
            raise ValueError(f"{field}.name must be non-empty")
        if type(data["passed"]) is not bool:
            raise ValueError(f"{field}.passed must be boolean")
        if not isinstance(data["reason"], str) or not data["reason"]:
            raise ValueError(f"{field}.reason must be non-empty")
        severity = data.get("severity", "hard")
        if not isinstance(severity, str) or severity not in GATE_SEVERITIES:
            raise ValueError(f"{field}.severity differs")
        return cls(
            name=data["name"],
            passed=data["passed"],
            expected=data["expected"],
            observed=data["observed"],
            reason=data["reason"],
            severity=severity,
        )


@dataclass(frozen=True, slots=True)
class EpisodeQualityMetric:
    name: str
    grade: str
    observed: float
    numerator: int | None = None
    denominator: int | None = None

    def __post_init__(self) -> None:
        if not self.name or self.grade not in QUALITY_GRADES:
            raise ValueError("quality metric identity or grade differs")
        if not math.isfinite(self.observed):
            raise ValueError("quality metric observed value must be finite")
        if (self.numerator is None) != (self.denominator is None):
            raise ValueError("quality metric ratio counts must be both present or absent")
        if self.numerator is not None and (
            type(self.numerator) is not int
            or type(self.denominator) is not int
            or self.numerator < 0
            or self.denominator <= 0
            or self.numerator > self.denominator
        ):
            raise ValueError("quality metric ratio counts differ")

    def to_mapping(self) -> dict[str, object]:
        return {
            "name": self.name,
            "grade": self.grade,
            "observed": self.observed,
            "numerator": self.numerator,
            "denominator": self.denominator,
        }

    @classmethod
    def from_mapping(cls, value: object, *, field: str) -> EpisodeQualityMetric:
        if not isinstance(value, Mapping) or frozenset(value) != {
            "name",
            "grade",
            "observed",
            "numerator",
            "denominator",
        }:
            raise ValueError(f"{field} keys differ")
        data = cast(Mapping[str, object], value)
        if not isinstance(data["name"], str) or not isinstance(data["grade"], str):
            raise ValueError(f"{field} string fields differ")
        observed = data["observed"]
        if isinstance(observed, bool) or not isinstance(observed, (int, float)):
            raise ValueError(f"{field}.observed differs")
        for key in ("numerator", "denominator"):
            if data[key] is not None and type(data[key]) is not int:
                raise ValueError(f"{field}.{key} differs")
        return cls(
            name=data["name"],
            grade=data["grade"],
            observed=float(observed),
            numerator=cast(int | None, data["numerator"]),
            denominator=cast(int | None, data["denominator"]),
        )


@dataclass(frozen=True, slots=True)
class ReleaseDecision:
    run_id: str
    passed: bool
    gates: tuple[ReleaseGateResult, ...]
    quality_metrics: tuple[EpisodeQualityMetric, ...] = ()

    def __post_init__(self) -> None:
        expected_passed = not any(
            not gate.passed and gate.severity == "hard" for gate in self.gates
        )
        if self.passed != expected_passed:
            raise ValueError("release decision passed value differs from hard gates")
        if len({metric.name for metric in self.quality_metrics}) != len(self.quality_metrics):
            raise ValueError("release decision quality metric names are duplicated")

    @property
    def quality_grade(self) -> str:
        if not self.quality_metrics:
            return "A"
        return max(
            (metric.grade for metric in self.quality_metrics),
            key=QUALITY_GRADES.index,
        )

    @property
    def grade(self) -> str:
        if not self.passed:
            return "rejected"
        if any(not gate.passed and gate.severity == "warning" for gate in self.gates):
            return "usable_with_warnings"
        return "strict_qualified"

    @property
    def rejection_reasons(self) -> tuple[str, ...]:
        return tuple(
            gate.reason
            for gate in self.gates
            if not gate.passed and gate.severity == "hard"
        )

    @property
    def warning_reasons(self) -> tuple[str, ...]:
        return tuple(
            gate.reason
            for gate in self.gates
            if not gate.passed and gate.severity == "warning"
        )

    @property
    def advisory_reasons(self) -> tuple[str, ...]:
        return tuple(
            gate.reason
            for gate in self.gates
            if not gate.passed and gate.severity == "advisory"
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": RELEASE_DECISION_SCHEMA,
            "run_id": self.run_id,
            "passed": self.passed,
            "grade": self.grade,
            "quality_grade": self.quality_grade,
            "quality_metrics": [item.to_mapping() for item in self.quality_metrics],
            "gates": [gate.to_mapping() for gate in self.gates],
            "rejection_reasons": list(self.rejection_reasons),
            "warning_reasons": list(self.warning_reasons),
            "advisory_reasons": list(self.advisory_reasons),
        }

    @classmethod
    def from_mapping(cls, value: object) -> ReleaseDecision:
        if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
            raise ValueError("release decision must be a string-keyed mapping")
        data = cast(Mapping[str, object], value)
        legacy = data.get("schema") == LEGACY_RELEASE_DECISION_SCHEMA
        previous = data.get("schema") == PREVIOUS_RELEASE_DECISION_SCHEMA
        expected = (
            {
                "schema",
                "run_id",
                "passed",
                "gates",
                "rejection_reasons",
            }
            if legacy
            else {
                "schema",
                "run_id",
                "passed",
                "grade",
                "gates",
                "rejection_reasons",
                "warning_reasons",
                "advisory_reasons",
            }
            if previous
            else {
                "schema",
                "run_id",
                "passed",
                "grade",
                "quality_grade",
                "quality_metrics",
                "gates",
                "rejection_reasons",
                "warning_reasons",
                "advisory_reasons",
            }
        )
        if frozenset(data) != frozenset(expected):
            raise ValueError("release decision keys differ")
        if data["schema"] not in {
            RELEASE_DECISION_SCHEMA,
            PREVIOUS_RELEASE_DECISION_SCHEMA,
            LEGACY_RELEASE_DECISION_SCHEMA,
        }:
            raise ValueError("release decision schema differs")
        run_id = validate_run_id(data["run_id"])
        if type(data["passed"]) is not bool:
            raise ValueError("release decision passed must be boolean")
        raw_gates = data["gates"]
        raw_reasons = data["rejection_reasons"]
        if not isinstance(raw_gates, Sequence) or isinstance(
            raw_gates,
            (str, bytes, bytearray),
        ):
            raise ValueError("release decision gates must be a sequence")
        if not isinstance(raw_reasons, Sequence) or isinstance(
            raw_reasons,
            (str, bytes, bytearray),
        ):
            raise ValueError("release decision rejection reasons must be a sequence")
        gates = tuple(
            ReleaseGateResult.from_mapping(item, field=f"gates[{index}]")
            for index, item in enumerate(raw_gates)
        )
        raw_metrics = () if legacy or previous else _strict_sequence(
            data["quality_metrics"],
            field="quality_metrics",
        )
        decision = cls(
            run_id=run_id,
            passed=data["passed"],
            gates=gates,
            quality_metrics=tuple(
                EpisodeQualityMetric.from_mapping(item, field=f"quality_metrics[{index}]")
                for index, item in enumerate(raw_metrics)
            ),
        )
        if tuple(raw_reasons) != decision.rejection_reasons:
            raise ValueError("release decision rejection reason closure differs")
        if not legacy:
            if data["grade"] != decision.grade or data["grade"] not in RELEASE_GRADES:
                raise ValueError("release decision grade differs")
            if tuple(cast(Sequence[object], data["warning_reasons"])) != (
                decision.warning_reasons
            ):
                raise ValueError("release decision warning reason closure differs")
            if tuple(cast(Sequence[object], data["advisory_reasons"])) != (
                decision.advisory_reasons
            ):
                raise ValueError("release decision advisory reason closure differs")
        if not legacy and not previous and data["quality_grade"] != decision.quality_grade:
            raise ValueError("release decision quality grade differs")
        return decision


def _gate(
    name: str,
    condition: bool,
    *,
    expected: object,
    observed: object,
    reason: str,
    severity: str = "hard",
) -> ReleaseGateResult:
    return ReleaseGateResult(
        name=name,
        passed=bool(condition),
        expected=expected,
        observed=observed,
        reason="passed" if condition else reason,
        severity=severity,
    )


def evaluate_rgb_frame_grid(
    *,
    expected_frame_count: int,
    availability: Mapping[tuple[int, str], tuple[int, int] | None],
    maximum_missing_fraction: float = 0.01,
    maximum_consecutive_missing_frames: int = 1,
) -> ReleaseGateResult:
    """Grade an explicit 30 Hz RGB grid without inventing missing payloads.

    Every frame/camera key must exist. ``None`` is the only accepted missing
    marker; available cameras on one frame must share one rational renderer
    reference, and that reference must increase across frames.
    """

    if type(expected_frame_count) is not int or expected_frame_count <= 0:
        raise ValueError("RGB expected frame count must be positive")
    if (
        not math.isfinite(maximum_missing_fraction)
        or not 0.0 < maximum_missing_fraction <= 1.0
    ):
        raise ValueError("RGB missing fraction limit must be in (0, 1]")
    if (
        type(maximum_consecutive_missing_frames) is not int
        or maximum_consecutive_missing_frames < 1
    ):
        raise ValueError("RGB consecutive missing-frame limit must be positive")
    expected_keys = {
        (frame_index, camera_id)
        for frame_index in range(expected_frame_count)
        for camera_id in RGB_CAMERA_IDS
    }
    keys_complete = set(availability) == expected_keys
    references_valid = True
    frame_references: list[tuple[int, int]] = []
    if keys_complete:
        for frame_index in range(expected_frame_count):
            references: set[tuple[int, int]] = set()
            for camera_id in RGB_CAMERA_IDS:
                reference = availability[(frame_index, camera_id)]
                if reference is None:
                    continue
                if (
                    not isinstance(reference, tuple)
                    or len(reference) != 2
                    or any(type(item) is not int for item in reference)
                    or reference[0] < 0
                    or reference[1] <= 0
                ):
                    references_valid = False
                    continue
                references.add(reference)
            if len(references) != 1:
                references_valid = False
            else:
                frame_references.append(next(iter(references)))
    reference_monotonic = references_valid and len(frame_references) == expected_frame_count
    if reference_monotonic:
        reference_monotonic = all(
            current[0] * previous[1] > previous[0] * current[1]
            for previous, current in zip(
                frame_references,
                frame_references[1:],
                strict=False,
            )
        )
    missing_keys = tuple(
        key for key in sorted(expected_keys) if availability.get(key) is None
    )
    missing_fraction = len(missing_keys) / len(expected_keys)
    maximum_missing_run = 0
    for camera_id in RGB_CAMERA_IDS:
        current_run = 0
        for frame_index in range(expected_frame_count):
            if availability.get((frame_index, camera_id)) is None:
                current_run += 1
                maximum_missing_run = max(maximum_missing_run, current_run)
            else:
                current_run = 0
    structural_closure = keys_complete and references_valid and reference_monotonic
    within_diagnostic_budget = (
        structural_closure
        and missing_fraction <= maximum_missing_fraction
        and maximum_missing_run <= maximum_consecutive_missing_frames
    )
    strict = structural_closure and not missing_keys
    return ReleaseGateResult(
        name="rgb_30hz_frame_grid",
        passed=strict,
        expected={
            "camera_ids": list(RGB_CAMERA_IDS),
            "frame_indices": [0, expected_frame_count - 1],
            "diagnostic_missing_fraction_maximum": maximum_missing_fraction,
            "diagnostic_consecutive_missing_maximum": (
                maximum_consecutive_missing_frames
            ),
            "shared_strictly_increasing_reference": True,
            "explicit_missing_mask": True,
        },
        observed={
            "keys_complete": keys_complete,
            "references_valid": references_valid,
            "references_strictly_increasing": reference_monotonic,
            "missing_samples": len(missing_keys),
            "missing_fraction": missing_fraction,
            "maximum_consecutive_missing_frames": maximum_missing_run,
            "missing_keys": [list(key) for key in missing_keys],
        },
        reason=(
            "passed"
            if strict
            else (
                "rgb_isolated_missing_frames_within_diagnostic_budget"
                if within_diagnostic_budget
                else "rgb_gap_budget_or_reference_closure_failed"
            )
        ),
        severity="hard",
    )


def _episode_lifecycle(boundaries: tuple[DatasetEpisodeBoundary, ...], run_id: str) -> bool:
    expected = (
        DatasetEpisodeEvent.OPENED,
        DatasetEpisodeEvent.READY,
        DatasetEpisodeEvent.RECORDING,
        DatasetEpisodeEvent.STOP_REQUESTED,
        DatasetEpisodeEvent.CLOSED,
    )
    return (
        tuple(item.event for item in boundaries) == expected
        and all(item.run_id == run_id for item in boundaries)
        and all(
            earlier.host_time_ns <= later.host_time_ns
            for earlier, later in zip(boundaries, boundaries[1:], strict=False)
        )
    )


def _source_epochs_stable(ticks: tuple[ControlTickFacts, ...]) -> bool:
    by_source: dict[str, set[tuple[str, int]]] = {}
    for tick in ticks:
        for fact in tick.source_epochs:
            by_source.setdefault(fact.source_id, set()).add(
                (fact.producer_instance, fact.transport_epoch)
            )
    return all(len(values) == 1 for values in by_source.values())


def _rate_hz(ticks: tuple[ControlTickFacts, ...]) -> float | None:
    if len(ticks) < 2:
        return None
    elapsed_ns = ticks[-1].tick_time_ns - ticks[0].tick_time_ns
    if elapsed_ns <= 0:
        return None
    return (len(ticks) - 1) * 1e9 / elapsed_ns


def _real_time_factor(ticks: tuple[ControlTickFacts, ...]) -> float | None:
    if len(ticks) < 2:
        return None
    host_s = (ticks[-1].tick_time_ns - ticks[0].tick_time_ns) / 1e9
    simulation_s = (
        ticks[-1].transition.simulation_time_before_s - ticks[0].transition.simulation_time_before_s
    )
    if host_s <= 0.0 or simulation_s < 0.0:
        return None
    return simulation_s / host_s


def _q54_close(first: Iterable[float], second: Iterable[float], *, atol: float) -> bool:
    return all(
        math.isclose(left, right, rel_tol=0.0, abs_tol=atol)
        for left, right in zip(first, second, strict=True)
    )


def _physics_grid_closure(
    ticks: tuple[ControlTickFacts, ...],
    *,
    physics_hz: float,
    time_atol_s: float,
) -> tuple[bool, dict[str, object]]:
    origins: set[int] = set()
    maximum_time_error_s = 0.0
    per_tick_2_to_1 = bool(ticks)
    for tick in ticks:
        pre = tick.pre_action_frame
        post = tick.post_action_frame
        per_tick_2_to_1 = per_tick_2_to_1 and (
            tick.physics_substep_indices
            == (pre.physics_boundary_index, pre.physics_boundary_index + 1)
            and post.physics_boundary_index == pre.physics_boundary_index + 2
        )
        for frame in (pre, post):
            grid_index = round(frame.simulation_time_s * physics_hz)
            grid_time_s = grid_index / physics_hz
            maximum_time_error_s = max(
                maximum_time_error_s,
                abs(grid_time_s - frame.simulation_time_s),
            )
            origins.add(grid_index - frame.physics_boundary_index)
    cross_tick_indices = bool(ticks) and all(
        current.physics_substep_indices[0] == previous.physics_substep_indices[1] + 1
        and current.pre_action_frame.physics_boundary_index
        == previous.post_action_frame.physics_boundary_index
        for previous, current in zip(ticks, ticks[1:], strict=False)
    )
    passed = (
        per_tick_2_to_1
        and cross_tick_indices
        and len(origins) == 1
        and maximum_time_error_s <= time_atol_s
    )
    return passed, {
        "per_tick_2_to_1": per_tick_2_to_1,
        "cross_tick_indices": cross_tick_indices,
        "source_grid_origins": sorted(origins),
        "maximum_time_error_s": maximum_time_error_s,
    }


def _gap_motion_contexts(
    ticks: tuple[ControlTickFacts, ...],
    *,
    profile: Q54JointProfile,
    config: ReleaseGateConfig,
) -> tuple[dict[str, object], ...]:
    contexts: list[dict[str, object]] = []
    joint_limits = tuple(joint.max_velocity_rad_s for joint in profile.joints)
    for index, tick in enumerate(ticks):
        if tick.missed_control_periods_before_tick <= 0:
            continue
        frames = [tick.pre_action_frame, tick.post_action_frame]
        if index > 0:
            frames.insert(0, ticks[index - 1].post_action_frame)
        joint_velocity_fraction = max(
            (
                abs(value) / limit
                for frame in frames
                for value, limit in zip(frame.qdot54_rad_s, joint_limits, strict=True)
            ),
            default=0.0,
        )
        object_speed_m_s = max(
            (
                math.sqrt(sum(value * value for value in body.linear_velocity_m_s))
                for frame in frames
                for body in frame.rigid_bodies
                if body.valid
            ),
            default=0.0,
        )
        distances = tuple(
            math.dist(body.position_m, link.position_m)
            for frame in frames
            for body in frame.rigid_bodies
            if body.valid
            for link in frame.kinematic_links
            if link.valid
        )
        object_proximity_m = min(distances) if distances else None
        critical_reasons: list[str] = []
        if joint_velocity_fraction >= config.critical_gap_joint_velocity_fraction:
            critical_reasons.append("fast_joint_motion")
        if object_speed_m_s >= config.critical_gap_object_linear_speed_m_s:
            critical_reasons.append("moving_object")
        if (
            object_proximity_m is not None
            and object_proximity_m <= config.critical_gap_object_proximity_m
        ):
            critical_reasons.append("manipulation_proximity")
        contexts.append(
            {
                "control_index": tick.transition.control_index,
                "missed_periods": tick.missed_control_periods_before_tick,
                "joint_velocity_fraction": joint_velocity_fraction,
                "object_linear_speed_m_s": object_speed_m_s,
                "object_link_proximity_m": object_proximity_m,
                "critical_reasons": critical_reasons,
            }
        )
    return tuple(contexts)


def _percentile(values: Sequence[float], quantile: float) -> float:
    if not values or not 0.0 <= quantile <= 1.0:
        raise ValueError("quality percentile input differs")
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _grade_upper(value: float, thresholds: tuple[float, float, float]) -> str:
    for grade, threshold in zip(QUALITY_GRADES[:3], thresholds, strict=True):
        if value <= threshold:
            return grade
    return "D"


def _grade_lower(value: float, thresholds: tuple[float, float, float]) -> str:
    for grade, threshold in zip(QUALITY_GRADES[:3], thresholds, strict=True):
        if value >= threshold:
            return grade
    return "D"


def _ratio_metric(
    name: str,
    *,
    numerator: int,
    denominator: int,
    thresholds: tuple[float, float, float],
    higher_is_better: bool = False,
) -> EpisodeQualityMetric:
    if denominator <= 0 or not 0 <= numerator <= denominator:
        raise ValueError(f"{name} quality ratio differs")
    value = numerator / denominator
    grade = _grade_lower(value, thresholds) if higher_is_better else _grade_upper(
        value,
        thresholds,
    )
    return EpisodeQualityMetric(
        name=name,
        grade=grade,
        observed=value,
        numerator=numerator,
        denominator=denominator,
    )


def _route_missing_metric(
    ticks: tuple[ControlTickFacts, ...],
    *,
    name: str,
    route_keys: tuple[str, ...],
    thresholds: tuple[float, float, float],
) -> EpisodeQualityMetric:
    denominator = len(ticks)
    missing_by_route = tuple(
        sum(key not in tick.route_fact_keys for tick in ticks) for key in route_keys
    )
    numerator = max(missing_by_route, default=denominator)
    return _ratio_metric(
        name,
        numerator=numerator,
        denominator=denominator,
        thresholds=thresholds,
    )


def evaluate_episode_quality(
    facts: NormalizedEpisodeFacts,
    profile: Q54JointProfile,
    *,
    gate_config: ReleaseGateConfig = ReleaseGateConfig(),
    quality_config: EpisodeQualityConfig = EpisodeQualityConfig(),
) -> tuple[EpisodeQualityMetric, ...]:
    """Return a deterministic ABCD metric vector without changing integrity."""

    ticks = facts.ticks
    if not ticks:
        return ()
    rate = _rate_hz(ticks)
    rtf = _real_time_factor(ticks)
    if rate is None or rtf is None:
        return ()
    schedule_misses = sum(tick.missed_control_periods_before_tick for tick in ticks)
    expected_periods = len(ticks) + schedule_misses
    maximum_consecutive = max(
        tick.missed_control_periods_before_tick for tick in ticks
    )
    intervals_ms = tuple(
        (current.tick_time_ns - previous.tick_time_ns) / 1e6
        for previous, current in zip(ticks, ticks[1:], strict=False)
    )
    maximum_interval_ms = max(intervals_ms, default=0.0)
    selected = _route_missing_metric(
        ticks,
        name="selected_missing_fraction_worst_route",
        route_keys=(
            "left.tracker.raw_selected",
            "left.glove.q21_selected",
            "right.tracker.raw_selected",
            "right.glove.q21_selected",
        ),
        thresholds=quality_config.selected_missing_upper,
    )
    active = _route_missing_metric(
        ticks,
        name="active_provenance_missing_fraction_worst_route",
        route_keys=(
            "left.tracker.raw_active",
            "left.glove.q21_active",
            "right.tracker.raw_active",
            "right.glove.q21_active",
        ),
        thresholds=quality_config.active_missing_upper,
    )
    q7 = _route_missing_metric(
        ticks,
        name="q7_candidate_missing_fraction_worst_arm",
        route_keys=("left.arm.q7_candidate", "right.arm.q7_candidate"),
        thresholds=quality_config.q7_candidate_missing_upper,
    )
    actionable_keys = {
        "left.tracker.raw_selected",
        "left.glove.q21_selected",
        "left.arm.q7_candidate",
        "left.hand.q20_intent",
        "right.tracker.raw_selected",
        "right.glove.q21_selected",
        "right.arm.q7_candidate",
        "right.hand.q20_intent",
    }
    actionable_count = sum(
        tick.route_fact_keys.issuperset(actionable_keys) for tick in ticks
    )
    input_ages_by_route: dict[str, list[float]] = {}
    for tick in ticks:
        for route, age in tick.comparable_input_age_ms:
            input_ages_by_route.setdefault(route, []).append(age)
    worst_p95 = max(
        (_percentile(values, 0.95) for values in input_ages_by_route.values()),
        default=0.0,
    )
    critical = _gap_motion_contexts(ticks, profile=profile, config=gate_config)
    critical_count = sum(bool(item["critical_reasons"]) for item in critical)
    rate_limit_keys = (
        "left.arm.rate_limited",
        "left.hand.rate_limited",
        "right.arm.rate_limited",
        "right.hand.rate_limited",
    )
    clamp_keys = (
        "left.arm.clamped",
        "left.hand.clamped",
        "right.arm.clamped",
        "right.hand.clamped",
    )
    worst_rate_limit = max(
        (sum(key in tick.route_fact_keys for tick in ticks) for key in rate_limit_keys),
        default=0,
    )
    worst_clamp = max(
        (sum(key in tick.route_fact_keys for tick in ticks) for key in clamp_keys),
        default=0,
    )
    return (
        EpisodeQualityMetric(
            "control_effective_hz",
            _grade_lower(rate, quality_config.control_hz_lower),
            rate,
        ),
        EpisodeQualityMetric(
            "physics_real_time_factor",
            _grade_lower(rtf, quality_config.real_time_factor_lower),
            rtf,
        ),
        _ratio_metric(
            "scheduler_missed_fraction",
            numerator=schedule_misses,
            denominator=expected_periods,
            thresholds=quality_config.missed_fraction_upper,
        ),
        EpisodeQualityMetric(
            "maximum_consecutive_missed_periods",
            _grade_upper(float(maximum_consecutive), quality_config.consecutive_misses_upper),
            float(maximum_consecutive),
        ),
        EpisodeQualityMetric(
            "maximum_control_interval_ms",
            _grade_upper(maximum_interval_ms, quality_config.maximum_interval_ms_upper),
            maximum_interval_ms,
        ),
        selected,
        active,
        q7,
        _ratio_metric(
            "four_stream_actionable_fraction",
            numerator=actionable_count,
            denominator=len(ticks),
            thresholds=quality_config.actionable_lower,
            higher_is_better=True,
        ),
        EpisodeQualityMetric(
            "input_age_p95_ms_worst_route",
            _grade_upper(worst_p95, quality_config.input_age_p95_ms_upper),
            worst_p95,
        ),
        _ratio_metric(
            "critical_motion_gap_fraction",
            numerator=critical_count,
            denominator=len(ticks),
            thresholds=quality_config.critical_gap_fraction_upper,
        ),
        _ratio_metric(
            "rate_limit_fraction_worst_route",
            numerator=worst_rate_limit,
            denominator=len(ticks),
            thresholds=quality_config.rate_limit_fraction_upper,
        ),
        _ratio_metric(
            "clamp_fraction_worst_route",
            numerator=worst_clamp,
            denominator=len(ticks),
            thresholds=quality_config.clamp_fraction_upper,
        ),
    )


def validate_episode_release(
    facts: NormalizedEpisodeFacts,
    profile: Q54JointProfile,
    *,
    config: ReleaseGateConfig = ReleaseGateConfig(),
    quality_config: EpisodeQualityConfig = EpisodeQualityConfig(),
) -> ReleaseDecision:
    """Evaluate every hard gate without converting a failure into an exception."""

    ticks = facts.ticks
    transitions = tuple(tick.transition for tick in ticks)
    indices = tuple(row.control_index for row in transitions)
    contiguous = bool(indices) and indices == tuple(range(indices[0], indices[0] + len(indices)))
    same_run = all(row.run_id == facts.run_id for row in transitions)
    complete_ticks = bool(transitions) and all(row.complete for row in transitions)
    route_complete = bool(ticks) and all(
        tick.route_fact_keys.issuperset(REQUIRED_INTEGRITY_ROUTE_FACTS) for tick in ticks
    )
    schedule_misses = sum(tick.missed_control_periods_before_tick for tick in ticks)
    expected_control_periods = len(ticks) + schedule_misses
    missed_control_fraction = (
        schedule_misses / expected_control_periods if expected_control_periods else None
    )
    maximum_consecutive_misses = max(
        (tick.missed_control_periods_before_tick for tick in ticks),
        default=0,
    )
    control_intervals_s = tuple(
        (current.tick_time_ns - previous.tick_time_ns) / 1e9
        for previous, current in zip(ticks, ticks[1:], strict=False)
    )
    host_time_monotonic = bool(ticks) and all(value > 0.0 for value in control_intervals_s)
    maximum_control_interval_s = max(control_intervals_s, default=0.0)
    schedule_slot_mask_closed = bool(ticks) and all(
        current.schedule_slot - previous.schedule_slot
        == 1 + current.missed_control_periods_before_tick
        for previous, current in zip(ticks, ticks[1:], strict=False)
    )
    physics_grid_closed, physics_grid_observed = _physics_grid_closure(
        ticks,
        physics_hz=config.expected_physics_hz,
        time_atol_s=config.physics_grid_time_atol_s,
    )
    gap_contexts = _gap_motion_contexts(ticks, profile=profile, config=config)
    critical_gap_contexts = tuple(
        context for context in gap_contexts if context["critical_reasons"]
    )
    frame_digests_match = bool(ticks) and all(
        tick.pre_action_frame.payload_digest_sha256 == tick.transition.pre_action_state_digest
        for tick in ticks
    )
    frame_q54_match = bool(ticks) and all(
        _q54_close(
            tick.pre_action_frame.q54_rad,
            tick.transition.pre_feedback_q54_rad,
            atol=config.q54_continuity_atol_rad,
        )
        and _q54_close(
            tick.post_action_frame.q54_rad,
            tick.transition.post_feedback_q54_rad,
            atol=config.q54_continuity_atol_rad,
        )
        for tick in ticks
    )
    frame_times_match = bool(ticks) and all(
        math.isclose(
            tick.pre_action_frame.simulation_time_s,
            tick.transition.simulation_time_before_s,
            rel_tol=0.0,
            abs_tol=config.simulation_time_atol_s,
        )
        and math.isclose(
            tick.post_action_frame.simulation_time_s,
            tick.transition.simulation_time_after_s,
            rel_tol=0.0,
            abs_tol=config.simulation_time_atol_s,
        )
        for tick in ticks
    )
    adjacent_closed = bool(ticks) and all(
        math.isclose(
            previous.transition.simulation_time_after_s,
            current.transition.simulation_time_before_s,
            rel_tol=0.0,
            abs_tol=config.simulation_time_atol_s,
        )
        and _q54_close(
            previous.transition.post_feedback_q54_rad,
            current.transition.pre_feedback_q54_rad,
            atol=config.q54_continuity_atol_rad,
        )
        for previous, current in zip(ticks, ticks[1:], strict=False)
    )
    input_ages = tuple(age for tick in ticks for _, age in tick.comparable_input_age_ms)
    input_age_max = max(input_ages) if input_ages else None
    rate = _rate_hz(ticks)
    rtf = _real_time_factor(ticks)
    source_modes = {item.source_mode for item in facts.boundaries}
    eligible = bool(facts.boundaries) and all(item.dataset_eligible for item in facts.boundaries)
    final_index = transitions[-1].control_index if transitions else None
    boundary_final = (
        facts.boundaries[-1].effective_final_control_index if facts.boundaries else None
    )
    quality_metrics = evaluate_episode_quality(
        facts,
        profile,
        gate_config=config,
        quality_config=quality_config,
    )
    quality_by_name = {item.name: item for item in quality_metrics}

    gates = (
        _gate(
            "artifact_closure",
            facts.artifact_complete and facts.checksums_verified,
            expected="complete_and_checksums_verified",
            observed={
                "artifact_complete": facts.artifact_complete,
                "checksums_verified": facts.checksums_verified,
            },
            reason="artifact_or_checksum_incomplete",
        ),
        _gate(
            "known_schemas",
            not facts.unknown_schemas,
            expected=[],
            observed=list(facts.unknown_schemas),
            reason="unknown_schema_present",
        ),
        _gate(
            "recorder_inventory",
            facts.recorder_inventory_complete,
            expected=True,
            observed=facts.recorder_inventory_complete,
            reason="recorder_inventory_incomplete",
        ),
        _gate(
            "episode_lifecycle",
            _episode_lifecycle(facts.boundaries, facts.run_id),
            expected=[item.value for item in DatasetEpisodeEvent],
            observed=[item.event.value for item in facts.boundaries],
            reason="episode_lifecycle_invalid",
        ),
        _gate(
            "dataset_source",
            source_modes == {DatasetSourceMode.LIVE_TELEOPERATION} and eligible,
            expected="live_teleoperation_and_eligible",
            observed={
                "source_modes": sorted(item.value for item in source_modes),
                "eligible": eligible,
            },
            reason="fixture_replay_or_ineligible_source",
        ),
        _gate(
            "q54_identity",
            facts.q54_profile_id == profile.profile_id
            and facts.q54_profile_sha256 == profile.file_sha256
            and facts.q54_runtime_names == profile.canonical_names,
            expected={
                "profile_id": profile.profile_id,
                "profile_sha256": profile.file_sha256,
                "dimension": 54,
            },
            observed={
                "profile_id": facts.q54_profile_id,
                "profile_sha256": facts.q54_profile_sha256,
                "dimension": len(facts.q54_runtime_names),
            },
            reason="q54_profile_or_runtime_inventory_mismatch",
        ),
        _gate(
            "complete_contiguous_ticks",
            contiguous and same_run and complete_ticks,
            expected="one_run_contiguous_complete_ticks",
            observed={
                "count": len(transitions),
                "first": indices[0] if indices else None,
                "last": indices[-1] if indices else None,
                "same_run": same_run,
                "complete": complete_ticks,
            },
            reason="tick_gap_wrong_run_or_incomplete_tick",
        ),
        _gate(
            "effective_final_tick",
            final_index is not None and boundary_final == final_index,
            expected=final_index,
            observed=boundary_final,
            reason="boundary_final_tick_mismatch",
        ),
        _gate(
            "route_fact_completeness",
            route_complete,
            expected=sorted(REQUIRED_INTEGRITY_ROUTE_FACTS),
            observed="all_ticks_complete" if route_complete else "missing_route_facts",
            reason="applied_q27_fact_missing",
        ),
        _gate(
            "source_epoch_stability",
            _source_epochs_stable(ticks),
            expected="one_producer_epoch_per_source",
            observed="stable" if _source_epochs_stable(ticks) else "changed_or_missing",
            reason="source_epoch_changed_or_missing",
        ),
        _gate(
            "pre_post_state_closure",
            frame_digests_match and frame_q54_match and frame_times_match and adjacent_closed,
            expected="digest_q54_time_and_adjacent_closure",
            observed={
                "digest": frame_digests_match,
                "q54": frame_q54_match,
                "time": frame_times_match,
                "adjacent": adjacent_closed,
            },
            reason="pre_post_state_closure_failed",
        ),
        _gate(
            "physics_2_to_1_and_time_grid",
            physics_grid_closed,
            expected={
                "physics_hz": config.expected_physics_hz,
                "substeps_per_actual_control_tick": 2,
                "one_fixed_integer_origin": True,
                "maximum_float_time_error_s": config.physics_grid_time_atol_s,
            },
            observed=physics_grid_observed,
            reason="physics_index_or_timestamp_gap",
        ),
        ReleaseGateResult(
            name="control_schedule_gap_quality",
            passed=schedule_misses == 0,
            expected={
                "strict_missed_periods": 0,
                "release_role": "quality_only",
            },
            observed={
                "actual_tick_count": len(ticks),
                "missed_periods": schedule_misses,
                "expected_periods": expected_control_periods,
                "missed_fraction": missed_control_fraction,
                "maximum_consecutive_missed_periods": maximum_consecutive_misses,
                "maximum_control_interval_s": maximum_control_interval_s,
                "critical_gap_contexts": list(critical_gap_contexts),
            },
            reason="control_schedule_gap_quality_downgrade",
            severity="warning",
        ),
        _gate(
            "schedule_slot_mask_closure",
            schedule_slot_mask_closed,
            expected="slot_delta_equals_one_plus_explicit_missed_periods",
            observed=schedule_slot_mask_closed,
            reason="schedule_slot_and_missing_mask_conflict",
        ),
        _gate(
            "host_time_order",
            host_time_monotonic,
            expected="strictly_increasing_tick_time_ns",
            observed={
                "strictly_increasing": host_time_monotonic,
                "maximum_control_interval_s": maximum_control_interval_s,
            },
            reason="host_time_reversed_or_duplicated",
        ),
        _gate(
            "control_rate",
            quality_by_name.get("control_effective_hz") is not None
            and quality_by_name["control_effective_hz"].grade == "A",
            expected="quality_grade_A_non_blocking",
            observed=rate,
            reason="control_rate_out_of_range",
            severity="warning",
        ),
        _gate(
            "real_time_factor",
            quality_by_name.get("physics_real_time_factor") is not None
            and quality_by_name["physics_real_time_factor"].grade == "A",
            expected="quality_grade_A_non_blocking",
            observed=rtf,
            reason="real_time_factor_below_limit",
            severity="warning",
        ),
        _gate(
            "input_age",
            quality_by_name.get("input_age_p95_ms_worst_route") is not None
            and quality_by_name["input_age_p95_ms_worst_route"].grade == "A",
            expected="worst_route_p95_quality_grade_A_non_blocking",
            observed={
                "maximum_ms": input_age_max,
                "worst_route_p95_ms": (
                    None
                    if quality_by_name.get("input_age_p95_ms_worst_route") is None
                    else quality_by_name["input_age_p95_ms_worst_route"].observed
                ),
            },
            reason="input_age_missing_or_above_limit",
            severity="warning",
        ),
        ReleaseGateResult(
            name="legacy_fixed_fixture_diagnostic",
            passed=(
            facts.fixture_translation_drift_m <= config.fixture_translation_drift_limit_m
            and facts.fixture_rotation_drift_rad <= config.fixture_rotation_drift_limit_rad
            ),
            expected={
                "translation_m": config.fixture_translation_drift_limit_m,
                "rotation_rad": config.fixture_rotation_drift_limit_rad,
                "release_role": "advisory_until_task_neutral_static_reference_manifest",
            },
            observed={
                "translation_m": facts.fixture_translation_drift_m,
                "rotation_rad": facts.fixture_rotation_drift_rad,
            },
            reason=(
                "passed"
                if facts.fixture_translation_drift_m <= config.fixture_translation_drift_limit_m
                and facts.fixture_rotation_drift_rad <= config.fixture_rotation_drift_limit_rad
                else "legacy_fixed_fixture_drift_above_limit"
            ),
            severity="advisory",
        ),
    )
    return ReleaseDecision(
        run_id=facts.run_id,
        passed=not any(
            not gate.passed and gate.severity == "hard" for gate in gates
        ),
        gates=gates,
        quality_metrics=quality_metrics,
    )


__all__ = [
    "NORMALIZED_EPISODE_FACTS_SCHEMA",
    "RELEASE_DECISION_SCHEMA",
    "REQUIRED_INTEGRITY_ROUTE_FACTS",
    "REQUIRED_ROUTE_FACTS",
    "ControlTickFacts",
    "EpisodeQualityConfig",
    "EpisodeQualityMetric",
    "NormalizedEpisodeFacts",
    "ReleaseDecision",
    "ReleaseGateConfig",
    "ReleaseGateResult",
    "SourceEpochFact",
    "evaluate_episode_quality",
    "evaluate_rgb_frame_grid",
    "validate_episode_release",
]
