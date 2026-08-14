"""Exact causal identity mapping from immutable 30 Hz transitions."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from numbers import Real
from typing import Iterable, cast

from wujihand.domain.recording import validate_run_id


ALIGNMENT_SCHEMA = "wujihand.dataset_alignment.v4"


def _vector54(value: Iterable[float], *, field: str) -> tuple[float, ...]:
    raw = tuple(value)
    if len(raw) != 54 or any(
        isinstance(item, bool) or not isinstance(item, Real) for item in raw
    ):
        raise ValueError(f"{field} must contain 54 finite values")
    result = tuple(float(item) for item in raw)
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{field} must contain 54 finite values")
    return result


@dataclass(frozen=True, slots=True)
class RawTransition:
    run_id: str
    control_index: int
    tick_id: int
    simulation_time_before_s: float
    simulation_time_after_s: float
    pre_feedback_q54_rad: tuple[float, ...]
    applied_target_q54_rad: tuple[float, ...]
    post_feedback_q54_rad: tuple[float, ...]
    pre_action_state_digest: str
    complete: bool = True

    def __post_init__(self) -> None:
        validate_run_id(self.run_id)
        if (
            type(self.control_index) is not int
            or type(self.tick_id) is not int
            or self.control_index < 0
            or self.tick_id < 0
        ):
            raise ValueError("control_index and tick_id must be non-negative")
        if self.control_index != self.tick_id:
            raise ValueError("current contract requires control_index == tick_id")
        if not math.isfinite(self.simulation_time_before_s) or not math.isfinite(
            self.simulation_time_after_s
        ):
            raise ValueError("simulation times must be finite")
        if self.simulation_time_after_s <= self.simulation_time_before_s:
            raise ValueError("simulation_time_after_s must exceed before time")
        _vector54(self.pre_feedback_q54_rad, field="pre_feedback_q54_rad")
        _vector54(self.applied_target_q54_rad, field="applied_target_q54_rad")
        _vector54(self.post_feedback_q54_rad, field="post_feedback_q54_rad")
        if len(self.pre_action_state_digest) != 64 or any(
            char not in "0123456789abcdef" for char in self.pre_action_state_digest
        ):
            raise ValueError("pre_action_state_digest must be a lowercase SHA-256")
        if type(self.complete) is not bool:
            raise ValueError("complete must be a boolean")

    def to_mapping(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "control_index": self.control_index,
            "tick_id": self.tick_id,
            "simulation_time_before_s": self.simulation_time_before_s,
            "simulation_time_after_s": self.simulation_time_after_s,
            "pre_feedback_q54_rad": list(self.pre_feedback_q54_rad),
            "applied_target_q54_rad": list(self.applied_target_q54_rad),
            "post_feedback_q54_rad": list(self.post_feedback_q54_rad),
            "pre_action_state_digest": self.pre_action_state_digest,
            "complete": self.complete,
        }

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "transition") -> RawTransition:
        if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
            raise ValueError(f"{field} must be a string-keyed mapping")
        data = cast(Mapping[str, object], value)
        expected = frozenset(
            {
                "run_id",
                "control_index",
                "tick_id",
                "simulation_time_before_s",
                "simulation_time_after_s",
                "pre_feedback_q54_rad",
                "applied_target_q54_rad",
                "post_feedback_q54_rad",
                "pre_action_state_digest",
                "complete",
            }
        )
        if frozenset(data) != expected:
            raise ValueError(f"{field} keys differ from the transition schema")
        vectors: dict[str, tuple[float, ...]] = {}
        for key in (
            "pre_feedback_q54_rad",
            "applied_target_q54_rad",
            "post_feedback_q54_rad",
        ):
            raw = data[key]
            if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes, bytearray)):
                raise ValueError(f"{field}.{key} must be a sequence")
            vectors[key] = _vector54(cast(Sequence[float], raw), field=f"{field}.{key}")
        if not isinstance(data["run_id"], str):
            raise ValueError(f"{field}.run_id must be a string")
        for key in ("control_index", "tick_id"):
            if type(data[key]) is not int:
                raise ValueError(f"{field}.{key} must be an integer")
        for key in ("simulation_time_before_s", "simulation_time_after_s"):
            if isinstance(data[key], bool) or not isinstance(data[key], Real):
                raise ValueError(f"{field}.{key} must be finite")
        if not isinstance(data["pre_action_state_digest"], str):
            raise ValueError(f"{field}.pre_action_state_digest must be a string")
        if type(data["complete"]) is not bool:
            raise ValueError(f"{field}.complete must be a boolean")
        return cls(
            run_id=data["run_id"],
            control_index=cast(int, data["control_index"]),
            tick_id=cast(int, data["tick_id"]),
            simulation_time_before_s=float(cast(Real, data["simulation_time_before_s"])),
            simulation_time_after_s=float(cast(Real, data["simulation_time_after_s"])),
            pre_feedback_q54_rad=vectors["pre_feedback_q54_rad"],
            applied_target_q54_rad=vectors["applied_target_q54_rad"],
            post_feedback_q54_rad=vectors["post_feedback_q54_rad"],
            pre_action_state_digest=data["pre_action_state_digest"],
            complete=data["complete"],
        )


@dataclass(frozen=True, slots=True)
class AlignmentFrame:
    dataset_frame_index: int
    source_control_index: int
    source_tick_id: int
    timestamp_s: float
    simulation_time_s: float
    observation_q54_rad: tuple[float, ...]
    action_q54_rad: tuple[float, ...]
    source_state_digest: str
    temporal_continuity: bool = True
    missing_control_periods_before: int = 0
    temporal_segment_index: int = 0
    gap_before_row: bool = False
    transition_valid: bool = False

    def __post_init__(self) -> None:
        if (
            type(self.dataset_frame_index) is not int
            or type(self.source_control_index) is not int
            or type(self.source_tick_id) is not int
            or min(
                self.dataset_frame_index,
                self.source_control_index,
                self.source_tick_id,
            )
            < 0
        ):
            raise ValueError("alignment frame indices must be non-negative integers")
        if self.source_control_index != self.source_tick_id:
            raise ValueError("alignment source control_index and tick_id must match")
        expected_timestamp = self.dataset_frame_index / 30.0
        if not math.isfinite(self.timestamp_s) or not math.isclose(
            self.timestamp_s,
            expected_timestamp,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("alignment timestamp must equal frame_index / 30")
        if not math.isfinite(self.simulation_time_s) or self.simulation_time_s < 0.0:
            raise ValueError("alignment simulation time must be finite and non-negative")
        _vector54(self.observation_q54_rad, field="observation_q54_rad")
        _vector54(self.action_q54_rad, field="action_q54_rad")
        if len(self.source_state_digest) != 64 or any(
            char not in "0123456789abcdef" for char in self.source_state_digest
        ):
            raise ValueError("alignment source state digest must be a lowercase SHA-256")
        if type(self.temporal_continuity) is not bool:
            raise ValueError("alignment temporal continuity must be boolean")
        if (
            type(self.missing_control_periods_before) is not int
            or self.missing_control_periods_before < 0
            or type(self.temporal_segment_index) is not int
            or self.temporal_segment_index < 0
        ):
            raise ValueError("alignment gap mask fields must be non-negative integers")
        if self.temporal_continuity != (self.missing_control_periods_before == 0):
            raise ValueError("alignment temporal continuity and missing mask differ")
        if self.gap_before_row != (self.missing_control_periods_before > 0):
            raise ValueError("alignment gap_before_row and missing mask differ")
        if type(self.transition_valid) is not bool:
            raise ValueError("alignment transition_valid must be boolean")

    def to_mapping(self) -> dict[str, object]:
        return {
            "dataset_frame_index": self.dataset_frame_index,
            "source_control_index": self.source_control_index,
            "source_tick_id": self.source_tick_id,
            "timestamp_s": self.timestamp_s,
            "simulation_time_s": self.simulation_time_s,
            "observation_q54_rad": list(self.observation_q54_rad),
            "action_q54_rad": list(self.action_q54_rad),
            "source_state_digest": self.source_state_digest,
            "temporal_continuity": self.temporal_continuity,
            "missing_control_periods_before": self.missing_control_periods_before,
            "temporal_segment_index": self.temporal_segment_index,
            "gap_before_row": self.gap_before_row,
            "transition_valid": self.transition_valid,
        }

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "frame") -> AlignmentFrame:
        if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
            raise ValueError(f"{field} must be a string-keyed mapping")
        data = cast(Mapping[str, object], value)
        expected = frozenset(
            {
                "dataset_frame_index",
                "source_control_index",
                "source_tick_id",
                "timestamp_s",
                "simulation_time_s",
                "observation_q54_rad",
                "action_q54_rad",
                "source_state_digest",
            }
        )
        expected |= {
            "temporal_continuity",
            "missing_control_periods_before",
            "temporal_segment_index",
            "gap_before_row",
            "transition_valid",
        }
        if frozenset(data) != expected:
            raise ValueError(f"{field} keys differ from the alignment frame schema")
        for key in ("dataset_frame_index", "source_control_index", "source_tick_id"):
            if type(data[key]) is not int:
                raise ValueError(f"{field}.{key} must be an integer")
        for key in ("timestamp_s", "simulation_time_s"):
            if isinstance(data[key], bool) or not isinstance(data[key], Real):
                raise ValueError(f"{field}.{key} must be finite")
        vectors: dict[str, tuple[float, ...]] = {}
        for key in ("observation_q54_rad", "action_q54_rad"):
            raw = data[key]
            if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes, bytearray)):
                raise ValueError(f"{field}.{key} must be a sequence")
            vectors[key] = _vector54(cast(Sequence[float], raw), field=f"{field}.{key}")
        digest = data["source_state_digest"]
        if not isinstance(digest, str):
            raise ValueError(f"{field}.source_state_digest must be a string")
        return cls(
            dataset_frame_index=cast(int, data["dataset_frame_index"]),
            source_control_index=cast(int, data["source_control_index"]),
            source_tick_id=cast(int, data["source_tick_id"]),
            timestamp_s=float(cast(Real, data["timestamp_s"])),
            simulation_time_s=float(cast(Real, data["simulation_time_s"])),
            observation_q54_rad=vectors["observation_q54_rad"],
            action_q54_rad=vectors["action_q54_rad"],
            source_state_digest=digest,
            temporal_continuity=cast(bool, data["temporal_continuity"]),
            missing_control_periods_before=cast(
                int, data["missing_control_periods_before"]
            ),
            temporal_segment_index=cast(int, data["temporal_segment_index"]),
            gap_before_row=cast(bool, data["gap_before_row"]),
            transition_valid=cast(bool, data["transition_valid"]),
        )


@dataclass(frozen=True, slots=True)
class ExactAlignment:
    run_id: str
    source_first_control_index: int
    source_last_control_index: int
    source_transition_count: int
    frames: tuple[AlignmentFrame, ...]
    digest_sha256: str
    gap_ticks: tuple[tuple[int, int], ...] = ()

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": ALIGNMENT_SCHEMA,
            "run_id": self.run_id,
            "source_first_control_index": self.source_first_control_index,
            "source_last_control_index": self.source_last_control_index,
            "source_transition_count": self.source_transition_count,
            "selection": "relative_all_control_index_no_interpolation_v1",
            "fps": 30,
            "frames": [frame.to_mapping() for frame in self.frames],
            "gap_ticks": [
                {"control_index": index, "missing_control_periods_before": missing}
                for index, missing in self.gap_ticks
            ],
            "digest_sha256": self.digest_sha256,
        }


def _alignment_digest(payload: dict[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def build_exact_30hz_alignment(
    transitions: Iterable[RawTransition],
    *,
    missed_control_periods_before_tick: Mapping[int, int] | None = None,
) -> ExactAlignment:
    rows = tuple(transitions)
    if not rows:
        raise ValueError("alignment requires at least one transition")
    run_ids = {row.run_id for row in rows}
    if len(run_ids) != 1:
        raise ValueError("all transitions must belong to one run")
    if any(not row.complete for row in rows):
        raise ValueError("alignment refuses incomplete transitions")
    indices = tuple(row.control_index for row in rows)
    if indices != tuple(range(indices[0], indices[0] + len(indices))):
        raise ValueError("control indices must be contiguous and ordered")
    for previous, current in zip(rows, rows[1:], strict=False):
        if not math.isclose(
            previous.simulation_time_after_s,
            current.simulation_time_before_s,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError("adjacent simulation transitions are not closed")
        if any(
            not math.isclose(first, second, rel_tol=0.0, abs_tol=1e-8)
            for first, second in zip(
                previous.post_feedback_q54_rad,
                current.pre_feedback_q54_rad,
                strict=True,
            )
        ):
            raise ValueError("adjacent q54 post/pre feedback is not continuous")
    raw_missing = missed_control_periods_before_tick or {}
    if any(
        type(index) is not int
        or index not in indices
        or type(missing) is not int
        or missing < 0
        for index, missing in raw_missing.items()
    ):
        raise ValueError("alignment missed-control mask differs")
    missing_by_index = {index: int(raw_missing.get(index, 0)) for index in indices}
    gap_ticks = tuple(
        (index, missing)
        for index, missing in missing_by_index.items()
        if missing > 0
    )
    frames: list[AlignmentFrame] = []
    temporal_segment_index = 0
    for dataset_frame_index, row in enumerate(rows):
        missing_before = missing_by_index[row.control_index]
        if dataset_frame_index > 0 and missing_before > 0:
            temporal_segment_index += 1
        transition_valid = (
            dataset_frame_index + 1 < len(rows)
            and missing_by_index[rows[dataset_frame_index + 1].control_index] == 0
        )
        frames.append(
            AlignmentFrame(
                dataset_frame_index=dataset_frame_index,
                source_control_index=row.control_index,
                source_tick_id=row.tick_id,
                timestamp_s=dataset_frame_index / 30.0,
                simulation_time_s=row.simulation_time_before_s,
                observation_q54_rad=row.pre_feedback_q54_rad,
                action_q54_rad=row.applied_target_q54_rad,
                source_state_digest=row.pre_action_state_digest,
                temporal_continuity=missing_before == 0,
                missing_control_periods_before=missing_before,
                temporal_segment_index=temporal_segment_index,
                gap_before_row=missing_before > 0,
                transition_valid=transition_valid,
            )
        )
    frame_rows = tuple(frames)
    payload: dict[str, object] = {
        "schema": ALIGNMENT_SCHEMA,
        "run_id": rows[0].run_id,
        "source_first_control_index": indices[0],
        "source_last_control_index": indices[-1],
        "source_transition_count": len(rows),
        "selection": "relative_all_control_index_no_interpolation_v1",
        "fps": 30,
        "frames": [frame.to_mapping() for frame in frame_rows],
        "gap_ticks": [
            {"control_index": index, "missing_control_periods_before": missing}
            for index, missing in gap_ticks
        ],
    }
    return ExactAlignment(
        run_id=rows[0].run_id,
        source_first_control_index=indices[0],
        source_last_control_index=indices[-1],
        source_transition_count=len(rows),
        frames=frame_rows,
        digest_sha256=_alignment_digest(payload),
        gap_ticks=gap_ticks,
    )


__all__ = [
    "ALIGNMENT_SCHEMA",
    "AlignmentFrame",
    "ExactAlignment",
    "RawTransition",
    "build_exact_30hz_alignment",
]
