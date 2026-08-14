"""Versioned metric definitions for the ROS 2 causal recording profile."""

from __future__ import annotations

import json
from collections import Counter, defaultdict
from dataclasses import dataclass
from itertools import pairwise
from typing import Any, Literal

import numpy as np

from .artifact import RunArtifact
from .model import BagDataset, Side, SourceRef, TickRecord
from .statistics import (
    SequenceRow,
    distribution,
    effective_rate_hz,
    finite_non_negative_delta_ms,
    ratio,
    sequence_metrics,
)

TICK_SCHEMA_V2 = "wujihand.teleoperation_tick_trace.v2"


@dataclass(frozen=True, slots=True)
class AnalysisConfig:
    expected_control_hz: float = 30.0
    expected_physics_hz: float = 120.0
    expected_render_hz: float = 15.0
    control_rate_tolerance_fraction: float = 0.02
    render_rate_tolerance_fraction: float = 0.05
    minimum_real_time_factor: float = 0.95
    p95_tick_interval_limit_ms: float = 35.0
    gui_p95_tick_interval_limit_ms: float = 35.0
    p95_comparable_input_age_limit_ms: float = 20.0
    q27_composition_atol_rad: float = 1e-12

    def __post_init__(self) -> None:
        positive = (
            self.expected_control_hz,
            self.expected_physics_hz,
            self.expected_render_hz,
            self.control_rate_tolerance_fraction,
            self.render_rate_tolerance_fraction,
            self.minimum_real_time_factor,
            self.p95_tick_interval_limit_ms,
            self.gui_p95_tick_interval_limit_ms,
            self.p95_comparable_input_age_limit_ms,
            self.q27_composition_atol_rad,
        )
        if any(value <= 0.0 or not np.isfinite(value) for value in positive):
            raise ValueError("analysis reference values must be finite and positive")

    def to_mapping(self) -> dict[str, float]:
        return {
            "expected_control_hz": self.expected_control_hz,
            "expected_physics_hz": self.expected_physics_hz,
            "expected_render_hz": self.expected_render_hz,
            "control_rate_tolerance_fraction": self.control_rate_tolerance_fraction,
            "render_rate_tolerance_fraction": self.render_rate_tolerance_fraction,
            "minimum_real_time_factor": self.minimum_real_time_factor,
            "p95_tick_interval_limit_ms": self.p95_tick_interval_limit_ms,
            "gui_p95_tick_interval_limit_ms": self.gui_p95_tick_interval_limit_ms,
            "p95_comparable_input_age_limit_ms": (self.p95_comparable_input_age_limit_ms),
            "q27_composition_atol_rad": self.q27_composition_atol_rad,
        }


@dataclass(frozen=True, slots=True)
class MetricBundle:
    summary: dict[str, Any]
    tables: dict[str, tuple[dict[str, Any], ...]]
    derived_tables: dict[str, tuple[dict[str, Any], ...]]


def _flatten(prefix: str, value: dict[str, Any]) -> dict[str, Any]:
    return {f"{prefix}_{key}": item for key, item in value.items()}


def _intervals_ms(times_ns: list[int]) -> tuple[list[float], int]:
    values: list[float] = []
    invalid = 0
    for first, second in pairwise(times_ns):
        if second <= first:
            invalid += 1
        else:
            values.append((second - first) / 1e6)
    return values, invalid


def _source_age_ms(tick_time_ns: int, source: SourceRef | None) -> float | None:
    return finite_non_negative_delta_ms(
        tick_time_ns,
        None if source is None else source.source_time_ns,
    )


def _input_age_ms(
    tick_time_ns: int,
    source: SourceRef | None,
) -> tuple[float | None, str | None]:
    """Use acquisition time when available, otherwise the comparable receive time."""

    if source is None:
        return None, None
    if source.source_time_ns is not None:
        return finite_non_negative_delta_ms(tick_time_ns, source.source_time_ns), "source_time_ns"
    return finite_non_negative_delta_ms(tick_time_ns, source.receive_time_ns), "receive_time_ns"


def _receive_age_ms(tick_time_ns: int, source: SourceRef | None) -> float | None:
    return finite_non_negative_delta_ms(
        tick_time_ns,
        None if source is None else source.receive_time_ns,
    )


def _callback_queue_ms(source: SourceRef | None) -> float | None:
    if source is None:
        return None
    return finite_non_negative_delta_ms(source.callback_time_ns, source.receive_time_ns)


def _selection_age_ms(tick_time_ns: int, source: SourceRef | None) -> float | None:
    return finite_non_negative_delta_ms(
        tick_time_ns,
        None if source is None else source.callback_time_ns,
    )


def _topic_metrics(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    expected = set(artifact.expected_topics)
    observed = {item.topic: item for item in dataset.topics}
    metadata_counts = {
        str(entry["topic_metadata"]["name"]): int(entry["message_count"])
        for entry in artifact.rosbag_metadata["topics_with_message_count"]
    }
    rows: list[dict[str, Any]] = []
    for topic in sorted(expected | observed.keys()):
        item = observed.get(topic)
        rate = None
        if (
            item is not None
            and item.first_bag_time_ns is not None
            and item.last_bag_time_ns is not None
        ):
            rate = effective_rate_hz([item.first_bag_time_ns, item.last_bag_time_ns])
            if rate is not None:
                rate *= max(0, item.count - 1)
        rows.append(
            {
                "topic": topic,
                "message_type": None if item is None else item.message_type,
                "expected": topic in expected,
                "count": 0 if item is None else item.count,
                "metadata_count": metadata_counts.get(topic),
                "metadata_count_matches": (
                    item is not None and item.count == metadata_counts.get(topic)
                ),
                "validated_count": 0 if item is None else item.validated_count,
                "effective_hz": rate,
                "nonempty": item is not None and item.count > 0,
                "all_messages_validated": (item is not None and item.count == item.validated_count),
            }
        )
    all_times = [
        value
        for item in dataset.topics
        for value in (item.first_bag_time_ns, item.last_bag_time_ns)
        if value is not None
    ]
    bag_duration_s = (max(all_times) - min(all_times)) / 1e9 if all_times else None
    summary = {
        "expected_topic_count": len(expected),
        "observed_topic_count": len(observed),
        "expected_nonempty_topic_count": sum(
            1 for row in rows if row["expected"] and row["nonempty"]
        ),
        "validated_message_count": sum(item.validated_count for item in dataset.topics),
        "all_observed_messages_validated": all(
            item.count == item.validated_count for item in dataset.topics
        ),
        "all_decoded_counts_match_metadata": all(
            item.count == metadata_counts.get(item.topic) for item in dataset.topics
        ),
        "missing_or_empty_expected_topics": [
            row["topic"] for row in rows if row["expected"] and not row["nonempty"]
        ],
        "extra_topics": [row["topic"] for row in rows if not row["expected"]],
        "message_count": sum(item.count for item in dataset.topics),
        "metadata_message_count": int(artifact.rosbag_metadata["message_count"]),
        "bag_duration_s": bag_duration_s,
    }
    return rows, summary


def _source_metrics(
    dataset: BagDataset, artifact: RunArtifact
) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
]:
    source_rows: list[dict[str, Any]] = []
    sequence_rows: list[dict[str, Any]] = []
    derived: list[dict[str, Any]] = []
    success_confidence = float(
        artifact.manifest.get("control", {})
        .get("glove", {})
        .get("success_landmark_confidence", 0.6)
    )
    for side in ("left", "right"):
        tracker = [item for item in dataset.trackers if item.side == side]
        tracker_times = [item.host_time_ns for item in tracker]
        intervals, invalid_intervals = _intervals_ms(tracker_times)
        tracker_row: dict[str, Any] = {
            "kind": "tracker",
            "side": side,
            "count": len(tracker),
            "effective_hz": effective_rate_hz(tracker_times),
            "invalid_timestamp_intervals": invalid_intervals,
            "pose_valid_ratio": ratio(sum(item.pose_valid for item in tracker), len(tracker)),
            "connected_ratio": ratio(sum(item.connected for item in tracker), len(tracker)),
            "running_ratio": ratio(
                sum(item.tracking_state == "running" for item in tracker), len(tracker)
            ),
            **_flatten("interval_ms", distribution(intervals)),
            **_flatten("quality", distribution(item.quality for item in tracker)),
        }
        source_rows.append(tracker_row)
        grouped = sequence_metrics(
            SequenceRow(item.producer_instance, item.transport_epoch, item.sequence)
            for item in tracker
        )
        for grouped_row in grouped:
            sequence_rows.append({"kind": "tracker", "side": side, **grouped_row})
        if tracker_times:
            origin = tracker_times[0]
            for tracker_item in tracker:
                tracker_sample_row: dict[str, Any] = {
                    "kind": "tracker",
                    "side": side,
                    "time_s": (tracker_item.host_time_ns - origin) / 1e9,
                    "producer_instance": tracker_item.producer_instance,
                    "transport_epoch": tracker_item.transport_epoch,
                    "source_id": tracker_item.source_id,
                    "sequence": tracker_item.sequence,
                    "valid_landmarks": None,
                    "minimum_confidence": None,
                    "median_confidence": None,
                    "pose_valid": tracker_item.pose_valid,
                    "connected": tracker_item.connected,
                    "tracking_state": tracker_item.tracking_state,
                    "position_x_m": tracker_item.position_m[0],
                    "position_y_m": tracker_item.position_m[1],
                    "position_z_m": tracker_item.position_m[2],
                    "quaternion_w": tracker_item.quaternion_wxyz[0],
                    "quaternion_x": tracker_item.quaternion_wxyz[1],
                    "quaternion_y": tracker_item.quaternion_wxyz[2],
                    "quaternion_z": tracker_item.quaternion_wxyz[3],
                }
                derived.append(tracker_sample_row)

        glove = [item for item in dataset.gloves if item.side == side]
        glove_times = [item.receive_time_ns for item in glove]
        intervals, invalid_intervals = _intervals_ms(glove_times)
        minima = [item.minimum_confidence for item in glove]
        glove_row: dict[str, Any] = {
            "kind": "glove",
            "side": side,
            "count": len(glove),
            "effective_hz": effective_rate_hz(glove_times),
            "invalid_timestamp_intervals": invalid_intervals,
            "all_21_landmarks_valid_ratio": ratio(
                sum(item.valid_landmarks == 21 for item in glove), len(glove)
            ),
            "valid_landmark_ratio": ratio(
                sum(item.valid_landmarks for item in glove), len(glove) * 21
            ),
            "below_success_confidence_ratio": ratio(
                sum(value is not None and value < success_confidence for value in minima),
                len(glove),
            ),
            "success_confidence_reference": success_confidence,
            **_flatten("interval_ms", distribution(intervals)),
            **_flatten("minimum_confidence", distribution(minima)),
            **_flatten(
                "median_confidence",
                distribution(item.median_confidence for item in glove),
            ),
        }
        source_rows.append(glove_row)
        grouped = sequence_metrics(
            SequenceRow(item.producer_instance, item.transport_epoch, item.sequence)
            for item in glove
        )
        for grouped_row in grouped:
            sequence_rows.append({"kind": "glove", "side": side, **grouped_row})
        if glove_times:
            origin = glove_times[0]
            for glove_item in glove:
                glove_sample_row: dict[str, Any] = {
                    "kind": "glove",
                    "side": side,
                    "time_s": (glove_item.receive_time_ns - origin) / 1e9,
                    "producer_instance": glove_item.producer_instance,
                    "transport_epoch": glove_item.transport_epoch,
                    "source_id": glove_item.source_id,
                    "sequence": glove_item.sequence,
                    "calibration_id": glove_item.calibration_id,
                    "transform_id": glove_item.transform_id,
                    "frame_id": glove_item.frame_id,
                    "landmark_layout": glove_item.landmark_layout,
                    "valid_landmarks": glove_item.valid_landmarks,
                    "minimum_confidence": glove_item.minimum_confidence,
                    "median_confidence": glove_item.median_confidence,
                    "pose_valid": None,
                    "connected": None,
                    "tracking_state": None,
                }
                for index, valid in enumerate(glove_item.landmark_valid):
                    glove_sample_row[f"landmark_{index:02d}_valid"] = valid
                    glove_sample_row[f"landmark_{index:02d}_confidence"] = (
                        glove_item.landmark_confidence[index]
                    )
                    offset = index * 3
                    glove_sample_row[f"landmark_{index:02d}_x_m"] = glove_item.landmark_positions_m[
                        offset
                    ]
                    glove_sample_row[f"landmark_{index:02d}_y_m"] = glove_item.landmark_positions_m[
                        offset + 1
                    ]
                    glove_sample_row[f"landmark_{index:02d}_z_m"] = glove_item.landmark_positions_m[
                        offset + 2
                    ]
                derived.append(glove_sample_row)
    return source_rows, sequence_rows, derived


def _partitions(artifact: RunArtifact) -> dict[Side, tuple[tuple[int, ...], tuple[int, ...]]]:
    raw = artifact.manifest.get("q27_partitions")
    if not isinstance(raw, dict):
        raise TypeError("manifest q27_partitions is required")
    result: dict[Side, tuple[tuple[int, ...], tuple[int, ...]]] = {}
    for side in ("left", "right"):
        value = raw.get(side)
        if not isinstance(value, dict):
            raise TypeError(f"manifest q27_partitions.{side} is required")
        arm = tuple(int(index) for index in value.get("arm_indices_q7", []))
        hand = tuple(int(index) for index in value.get("hand_indices_q20", []))
        if (
            len(arm) != 7
            or len(hand) != 20
            or set(arm) & set(hand)
            or set(arm) | set(hand) != set(range(27))
        ):
            raise ValueError(f"manifest q27 partition for {side} must cover 27 unique indices")
        result[side] = (arm, hand)
    return result


def _tick_integrity(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> tuple[dict[str, Any], list[TickRecord], dict[int, dict[Side, TickRecord]]]:
    by_tick: dict[int, dict[Side, TickRecord]] = defaultdict(dict)
    duplicate_side_records = 0
    for tick in dataset.ticks:
        if tick.side in by_tick[tick.tick_id]:
            duplicate_side_records += 1
        else:
            by_tick[tick.tick_id][tick.side] = tick
    representatives = [by_tick[tick_id][min(by_tick[tick_id])] for tick_id in sorted(by_tick)]
    paired = [tick_id for tick_id, sides in by_tick.items() if set(sides) == {"left", "right"}]
    time_mismatches = 0
    for tick_id in paired:
        left = by_tick[tick_id]["left"]
        right = by_tick[tick_id]["right"]
        if (
            left.schema != right.schema
            or left.times != right.times
            or left.execution != right.execution
        ):
            time_mismatches += 1
    tick_ids = sorted(by_tick)
    inferred_missing_tick_ids = (
        sum(max(0, second - first - 1) for first, second in pairwise(tick_ids)) if tick_ids else 0
    )
    expected = int(artifact.receipt.get("completed_ticks", -1))
    missing_hand_routes = sum(tick.hand is None for tick in dataset.ticks)
    return (
        {
            "trace_record_count": len(dataset.ticks),
            "unique_tick_count": len(by_tick),
            "receipt_completed_ticks": expected,
            "paired_tick_count": len(paired),
            "unpaired_tick_count": len(by_tick) - len(paired),
            "duplicate_side_records": duplicate_side_records,
            "side_time_mismatch_count": time_mismatches,
            "inferred_missing_tick_ids": inferred_missing_tick_ids,
            "missing_hand_route_records": missing_hand_routes,
        },
        representatives,
        by_tick,
    )


def _stage_metrics(
    representatives: list[TickRecord],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    stage_values: dict[str, list[float | None]] = defaultdict(list)
    rows: list[dict[str, Any]] = []
    for tick in representatives:
        times = tick.times
        input_name = "snapshot_ms" if tick.execution is not None else "spin_ms"
        simulation_name = "physics_ms" if tick.execution is not None else "world_step_ms"
        pipeline_start_ns = (
            times.tick_time_ns if tick.execution is not None else times.spin_start_ns
        )
        durations = {
            input_name: finite_non_negative_delta_ms(times.spin_end_ns, times.spin_start_ns),
            "control_ms": finite_non_negative_delta_ms(
                times.control_end_ns, times.control_start_ns
            ),
            "apply_ms": finite_non_negative_delta_ms(times.apply_end_ns, times.apply_start_ns),
            simulation_name: finite_non_negative_delta_ms(
                times.world_step_end_ns, times.world_step_start_ns
            ),
            "pipeline_ms": finite_non_negative_delta_ms(times.trace_time_ns, pipeline_start_ns),
        }
        rows.append({"tick_id": tick.tick_id, **durations})
        for name, value in durations.items():
            stage_values[name].append(value)
    summary = {name: distribution(values) for name, values in sorted(stage_values.items())}
    return rows, summary


def _execution_metrics(
    representatives: list[TickRecord],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    schema_counts = Counter(tick.schema for tick in representatives)
    rows: list[dict[str, Any]] = []
    substep_host_durations_ms: list[float] = []
    substep_simulation_dt_s: list[float] = []
    substep_indices: list[int] = []
    render_tick_times_ns: list[int] = []
    missed_periods = 0
    for tick in representatives:
        execution = tick.execution
        if execution is None:
            continue
        substep_simulation_dt_s.extend(
            current - previous
            for previous, current in pairwise(
                (
                    execution.simulation_time_before_s,
                    *execution.physics_substep_sim_times_s,
                )
            )
        )
        substep_indices.extend(execution.physics_substep_indices)
        for start_ns, end_ns in zip(
            execution.physics_substep_start_ns,
            execution.physics_substep_end_ns,
            strict=True,
        ):
            substep_host_durations_ms.append((end_ns - start_ns) / 1e6)
        missed_periods += execution.missed_control_periods_before_tick
        if execution.rendered:
            render_tick_times_ns.append(tick.times.tick_time_ns)
        row = {
                "tick_id": tick.tick_id,
                "schema": tick.schema,
                "schedule_slot": execution.schedule_slot,
                "scheduled_control_time_ns": execution.scheduled_control_time_ns,
                "actual_control_time_ns": tick.times.tick_time_ns,
                "control_lateness_ms": execution.control_lateness_ns / 1e6,
                "missed_control_periods_before_tick": (
                    execution.missed_control_periods_before_tick
                ),
                "simulation_time_before_s": execution.simulation_time_before_s,
                "simulation_time_after_s": execution.simulation_time_after_s,
                "simulation_advance_s": (
                    execution.simulation_time_after_s - execution.simulation_time_before_s
                ),
                "rendered": execution.rendered,
                "render_index": execution.render_index,
            }
        for ordinal, (substep_index, simulation_time, start_ns, end_ns) in enumerate(
            zip(
                execution.physics_substep_indices,
                execution.physics_substep_sim_times_s,
                execution.physics_substep_start_ns,
                execution.physics_substep_end_ns,
                strict=True,
            )
        ):
            row[f"physics_substep_{ordinal}_index"] = substep_index
            row[f"physics_substep_{ordinal}_sim_time_s"] = simulation_time
            row[f"physics_substep_{ordinal}_host_ms"] = (end_ns - start_ns) / 1e6
        rows.append(row)
    executions = [tick.execution for tick in representatives if tick.execution is not None]
    wall_span_s = (
        (representatives[-1].times.world_step_end_ns - representatives[0].times.world_step_start_ns)
        / 1e9
        if len(executions) == len(representatives) and representatives
        else None
    )
    simulation_advance_s = sum(
        execution.simulation_time_after_s - execution.simulation_time_before_s
        for execution in executions
    )
    consecutive_substep_indices = all(
        second == first + 1 for first, second in pairwise(substep_indices)
    )
    schedule_slots = [execution.schedule_slot for execution in executions]
    control_indices = [execution.control_index for execution in executions]
    render_indices = [
        execution.render_index for execution in executions if execution.render_index is not None
    ]
    schedule_gap_matches_missed_periods = bool(executions) and (
        executions[0].schedule_slot == 0
        and executions[0].missed_control_periods_before_tick == 0
        and all(
            second.schedule_slot - first.schedule_slot - 1
            == second.missed_control_periods_before_tick
            for first, second in pairwise(executions)
        )
    )
    simulation_time_continuous = all(
        np.isclose(
            first.simulation_time_after_s,
            second.simulation_time_before_s,
            rtol=0.0,
            atol=1e-9,
        )
        for first, second in pairwise(executions)
    )
    return rows, {
        "trace_schema_counts": dict(sorted(schema_counts.items())),
        "uniform_trace_schema": len(schema_counts) <= 1,
        "execution_record_count": len(executions),
        "execution_facts_complete": len(executions) == len(representatives),
        "physics_substep_count": len(substep_indices),
        "consecutive_physics_substep_indices": consecutive_substep_indices,
        "strictly_increasing_schedule_slots": all(
            second > first for first, second in pairwise(schedule_slots)
        ),
        "schedule_gap_matches_missed_periods": schedule_gap_matches_missed_periods,
        "simulation_time_continuous": simulation_time_continuous,
        "sequential_control_indices": control_indices == list(range(len(control_indices))),
        "sequential_render_indices": render_indices == list(range(len(render_indices))),
        "missed_control_periods": missed_periods,
        "rendered_tick_count": len(render_tick_times_ns),
        "rendered_control_indices": [
            execution.control_index for execution in executions if execution.rendered
        ],
        "render_effective_hz": effective_rate_hz(render_tick_times_ns),
        "simulation_advance_s": simulation_advance_s,
        "wall_span_s": wall_span_s,
        "real_time_factor": (
            simulation_advance_s / wall_span_s
            if wall_span_s is not None and wall_span_s > 0.0
            else None
        ),
        "physics_substep_host_ms": distribution(substep_host_durations_ms),
        "physics_substep_simulation_dt_s": distribution(substep_simulation_dt_s),
        "control_lateness_ms": distribution(
            [execution.control_lateness_ns / 1e6 for execution in executions]
        ),
    }


def _route_metrics(
    artifact: RunArtifact,
    dataset: BagDataset,
    by_tick: dict[int, dict[Side, TickRecord]],
) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    dict[str, Any],
]:
    partitions = _partitions(artifact)
    route_rows: list[dict[str, Any]] = []
    joint_rows: list[dict[str, Any]] = []
    tick_rows: list[dict[str, Any]] = []
    state_rows: list[dict[str, Any]] = []
    route_lookup: dict[tuple[Side, str], dict[str, Any]] = {}
    first_tick_ns = min((tick.times.tick_time_ns for tick in dataset.ticks), default=0)

    for side in ("left", "right"):
        ticks = sorted(
            (tick for tick in dataset.ticks if tick.side == side), key=lambda item: item.tick_id
        )
        duration_by_tick_ns = {
            first.tick_id: max(0, second.times.tick_time_ns - first.times.tick_time_ns)
            for first, second in pairwise(ticks)
        }
        if ticks:
            duration_by_tick_ns[ticks[-1].tick_id] = 0
        total_duration_ns = sum(duration_by_tick_ns.values())
        arm_indices, hand_indices = partitions[side]
        for chain in ("arm", "hand"):
            input_ages: list[float | None] = []
            input_age_bases: Counter[str] = Counter()
            source_ages: list[float | None] = []
            receive_ages: list[float | None] = []
            callback_queue: list[float | None] = []
            selection_ages: list[float | None] = []
            error_vectors: list[np.ndarray[Any, np.dtype[np.float64]]] = []
            composition_vectors: list[np.ndarray[Any, np.dtype[np.float64]]] = []
            command_vectors: list[np.ndarray[Any, np.dtype[np.float64]]] = []
            feedback_vectors: list[np.ndarray[Any, np.dtype[np.float64]]] = []
            intent_command_vectors: list[np.ndarray[Any, np.dtype[np.float64]]] = []
            actionable_count = 0
            actionable_duration_ns = 0
            active_source_count = 0
            active_source_duration_ns = 0
            new_source_count = 0
            new_source_tick_times_ns: list[int] = []
            position_clamped_count = 0
            rate_limited_count = 0
            safety_states: Counter[str] = Counter()
            safety_state_duration_ns: Counter[str] = Counter()
            safety_reasons: Counter[str] = Counter()
            safety_reason_duration_ns: Counter[str] = Counter()
            controller_states: Counter[str] = Counter()

            for tick in ticks:
                active: SourceRef | None
                source: SourceRef | None
                intent_command: np.ndarray[Any, np.dtype[np.float64]] | None
                if chain == "arm":
                    arm_value = tick.arm
                    active = arm_value.active_source
                    source = arm_value.source
                    command = np.asarray(arm_value.command_q7_rad, dtype=np.float64)
                    indices = np.asarray(arm_indices, dtype=np.int64)
                    safety_state = arm_value.safety_state
                    safety_reason = arm_value.safety_reason
                    position_clamped = arm_value.position_clamped
                    rate_limited = arm_value.rate_limited
                    controller_states[arm_value.controller_state] += 1
                    intent_command = None
                else:
                    if tick.hand is None:
                        continue
                    hand_value = tick.hand
                    active = hand_value.active_source
                    source = hand_value.source
                    command = np.asarray(hand_value.command_q20_rad, dtype=np.float64)
                    indices = np.asarray(hand_indices, dtype=np.int64)
                    safety_state = hand_value.safety_state
                    safety_reason = hand_value.safety_reason
                    position_clamped = hand_value.position_clamped
                    rate_limited = hand_value.rate_limited
                    intent_command = (
                        command - np.asarray(hand_value.intent_q20_rad, dtype=np.float64)
                        if hand_value.intent_q20_rad is not None
                        else None
                    )
                active_source_count += active is not None
                new_source_count += source is not None
                if source is not None:
                    new_source_tick_times_ns.append(tick.times.tick_time_ns)
                actionable = safety_state == "tracking" and active is not None
                actionable_count += actionable
                tick_duration_ns = duration_by_tick_ns.get(tick.tick_id, 0)
                actionable_duration_ns += tick_duration_ns if actionable else 0
                active_source_duration_ns += tick_duration_ns if active is not None else 0
                position_clamped_count += position_clamped
                rate_limited_count += rate_limited
                safety_states[safety_state] += 1
                safety_state_duration_ns[safety_state] += tick_duration_ns
                safety_reasons[safety_reason] += 1
                safety_reason_duration_ns[safety_reason] += tick_duration_ns
                input_age, input_age_basis = _input_age_ms(tick.times.tick_time_ns, active)
                source_age = _source_age_ms(tick.times.tick_time_ns, active)
                receive_age = _receive_age_ms(tick.times.tick_time_ns, active)
                queue_age = _callback_queue_ms(active)
                selection_age = _selection_age_ms(tick.times.tick_time_ns, active)
                input_ages.append(input_age)
                if input_age_basis is not None:
                    input_age_bases[input_age_basis] += 1
                source_ages.append(source_age)
                receive_ages.append(receive_age)
                callback_queue.append(queue_age)
                selection_ages.append(selection_age)

                applied = np.asarray(tick.applied_target_q27_rad, dtype=np.float64)[indices]
                feedback = np.asarray(tick.post_feedback_q27_rad, dtype=np.float64)[indices]
                error = feedback - command
                composition = applied - command
                error_vectors.append(error)
                composition_vectors.append(composition)
                command_vectors.append(command)
                feedback_vectors.append(feedback)
                if intent_command is not None:
                    intent_command_vectors.append(intent_command)
                tick_row: dict[str, Any] = {
                    "tick_id": tick.tick_id,
                    "side": side,
                    "time_s": (tick.times.tick_time_ns - first_tick_ns) / 1e9,
                    "chain": chain,
                    "actionable": actionable,
                    "active_source_present": active is not None,
                    "new_source_selected": source is not None,
                    "input_age_ms": input_age,
                    "input_age_basis": input_age_basis,
                    "source_age_ms": source_age,
                    "receive_age_ms": receive_age,
                    "callback_queue_ms": queue_age,
                    "selection_age_ms": selection_age,
                    "safety_state": safety_state,
                    "safety_reason": safety_reason,
                    "position_clamped": position_clamped,
                    "rate_limited": rate_limited,
                    "active_source_id": None if active is None else active.source_id,
                    "active_producer_instance": (
                        None if active is None else active.producer_instance
                    ),
                    "active_transport_epoch": (None if active is None else active.transport_epoch),
                    "active_sequence": None if active is None else active.sequence,
                    "new_source_id": None if source is None else source.source_id,
                    "new_producer_instance": (None if source is None else source.producer_instance),
                    "new_transport_epoch": (None if source is None else source.transport_epoch),
                    "new_sequence": None if source is None else source.sequence,
                }
                if chain == "arm":
                    tick_row.update(
                        {
                            "controller_state": tick.arm.controller_state,
                            "controller_reason": tick.arm.controller_reason,
                            "reference_epoch": tick.arm.reference_epoch,
                            "reference_established": tick.arm.reference_established,
                            "reference_revoked": tick.arm.reference_revoked,
                            "has_mapping": tick.arm.has_mapping,
                            "mapping_accepted": tick.arm.mapping_accepted,
                            "mapping_requires_reference": (tick.arm.mapping_requires_reference),
                            "mapping_reason": tick.arm.mapping_reason,
                            "has_kinematics": tick.arm.has_kinematics,
                            "ik_succeeded": tick.arm.ik_succeeded,
                            "solver_reported_success": (tick.arm.solver_reported_success),
                            "kinematics_reason": tick.arm.kinematics_reason,
                            "position_residual_m": tick.arm.position_residual_m,
                            "orientation_residual_rad": (tick.arm.orientation_residual_rad),
                        }
                    )
                    if tick.arm.target_position_m is not None:
                        for index, item in enumerate(tick.arm.target_position_m):
                            tick_row[f"target_position_{index}"] = item
                    if tick.arm.target_quaternion_wxyz is not None:
                        for index, item in enumerate(tick.arm.target_quaternion_wxyz):
                            tick_row[f"target_quaternion_{index}"] = item
                    if tick.arm.candidate_q7_rad is not None:
                        for index, item in enumerate(tick.arm.candidate_q7_rad):
                            tick_row[f"ik_candidate_j{index}"] = item
                elif tick.hand is not None:
                    tick_row.update(
                        {
                            "has_intent": tick.hand.has_intent,
                            "intent_is_new": tick.hand.intent_is_new,
                            "intent_sequence": tick.hand.intent_sequence,
                            "intent_layout_id": tick.hand.intent_layout_id,
                            "intent_produced_time_ns": (tick.hand.intent_produced_time_ns),
                            "retarget_status": tick.hand.retarget_status,
                            "retarget_confidence": tick.hand.retarget_confidence,
                            "rejection_reason": tick.hand.rejection_reason,
                        }
                    )
                    if tick.hand.intent_q20_rad is not None:
                        for index, item in enumerate(tick.hand.intent_q20_rad):
                            tick_row[f"intent_j{index}"] = item
                for index, item in enumerate(command):
                    tick_row[f"command_j{index}"] = float(item)
                    tick_row[f"feedback_j{index}"] = float(feedback[index])
                    tick_row[f"error_j{index}"] = float(error[index])
                    tick_row[f"composition_error_j{index}"] = float(composition[index])
                tick_rows.append(tick_row)

            denominator = len(error_vectors)
            errors = np.vstack(error_vectors) if error_vectors else np.empty((0, 0))
            compositions = (
                np.vstack(composition_vectors) if composition_vectors else np.empty((0, 0))
            )
            commands = np.vstack(command_vectors) if command_vectors else np.empty((0, 0))
            feedbacks = np.vstack(feedback_vectors) if feedback_vectors else np.empty((0, 0))
            absolute = np.abs(errors).reshape(-1) if errors.size else np.asarray([])
            new_source_intervals, invalid_new_source_intervals = _intervals_ms(
                new_source_tick_times_ns
            )
            control_window_ns = (
                ticks[-1].times.tick_time_ns - ticks[0].times.tick_time_ns if len(ticks) >= 2 else 0
            )
            new_source_full_window_hz = (
                (new_source_count - 1) * 1e9 / control_window_ns
                if new_source_count >= 2 and control_window_ns > 0
                else None
            )
            route_row = {
                "side": side,
                "chain": chain,
                "tick_count": denominator,
                "actionable_coverage": ratio(actionable_duration_ns, total_duration_ns),
                "actionable_tick_ratio": ratio(actionable_count, denominator),
                "active_source_coverage": ratio(active_source_duration_ns, total_duration_ns),
                "active_source_tick_ratio": ratio(active_source_count, denominator),
                "new_source_tick_ratio": ratio(new_source_count, denominator),
                "new_source_effective_hz": effective_rate_hz(new_source_tick_times_ns),
                "new_source_full_window_hz": new_source_full_window_hz,
                "new_source_invalid_timestamp_intervals": (invalid_new_source_intervals),
                **_flatten(
                    "new_source_interval_ms",
                    distribution(new_source_intervals),
                ),
                "position_clamped_ratio": ratio(position_clamped_count, denominator),
                "rate_limited_ratio": ratio(rate_limited_count, denominator),
                "command_feedback_rmse_rad": (
                    float(np.sqrt(np.mean(np.square(errors)))) if errors.size else None
                ),
                "command_feedback_p95_abs_rad": (
                    float(np.percentile(absolute, 95.0)) if absolute.size else None
                ),
                "command_feedback_max_abs_rad": (
                    float(np.max(absolute)) if absolute.size else None
                ),
                "applied_composition_max_abs_rad": (
                    float(np.max(np.abs(compositions))) if compositions.size else None
                ),
                "input_age_basis": "+".join(sorted(input_age_bases)) or None,
                **_flatten("input_age_ms", distribution(input_ages)),
                **_flatten("source_age_ms", distribution(source_ages)),
                **_flatten("receive_age_ms", distribution(receive_ages)),
                **_flatten("callback_queue_ms", distribution(callback_queue)),
                **_flatten("selection_age_ms", distribution(selection_ages)),
            }
            if intent_command_vectors:
                intent_command_values = np.abs(np.vstack(intent_command_vectors)).reshape(-1)
                route_row["intent_command_p95_abs_rad"] = float(
                    np.percentile(intent_command_values, 95.0)
                )
                route_row["intent_command_max_abs_rad"] = float(np.max(intent_command_values))
            else:
                route_row["intent_command_p95_abs_rad"] = None
                route_row["intent_command_max_abs_rad"] = None
            route_rows.append(route_row)
            route_lookup[(side, chain)] = route_row

            if errors.size:
                for index in range(errors.shape[1]):
                    joint_absolute = np.abs(errors[:, index])
                    joint_rows.append(
                        {
                            "side": side,
                            "chain": chain,
                            "joint_index": index,
                            "rmse_rad": float(np.sqrt(np.mean(np.square(errors[:, index])))),
                            "p95_abs_error_rad": float(np.percentile(joint_absolute, 95.0)),
                            "max_abs_error_rad": float(np.max(joint_absolute)),
                            "command_range_rad": float(np.ptp(commands[:, index])),
                            "feedback_range_rad": float(np.ptp(feedbacks[:, index])),
                        }
                    )
            for state, count in sorted(safety_states.items()):
                state_rows.append(
                    {
                        "side": side,
                        "chain": chain,
                        "category": "safety_state",
                        "value": state,
                        "count": count,
                        "tick_ratio": ratio(count, denominator),
                        "duration_ratio": ratio(safety_state_duration_ns[state], total_duration_ns),
                    }
                )
            for reason, count in sorted(safety_reasons.items()):
                state_rows.append(
                    {
                        "side": side,
                        "chain": chain,
                        "category": "safety_reason",
                        "value": reason,
                        "count": count,
                        "tick_ratio": ratio(count, denominator),
                        "duration_ratio": ratio(
                            safety_reason_duration_ns[reason], total_duration_ns
                        ),
                    }
                )
            for state, count in sorted(controller_states.items()):
                state_rows.append(
                    {
                        "side": side,
                        "chain": chain,
                        "category": "arm_controller_state",
                        "value": state,
                        "count": count,
                        "tick_ratio": ratio(count, denominator),
                        "duration_ratio": None,
                    }
                )

    paired_ids = sorted(
        tick_id for tick_id, sides in by_tick.items() if set(sides) == {"left", "right"}
    )
    paired_weights_ns = {
        first: max(
            0,
            by_tick[second]["left"].times.tick_time_ns - by_tick[first]["left"].times.tick_time_ns,
        )
        for first, second in pairwise(paired_ids)
    }
    if paired_ids:
        paired_weights_ns[paired_ids[-1]] = 0
    four_stream_ticks = 0
    four_stream_duration_ns = 0
    for tick_id in paired_ids:
        sides = by_tick[tick_id]
        left = sides["left"]
        right = sides["right"]
        left_hand = left.hand
        right_hand = right.hand
        values = [
            left.arm.safety_state == "tracking" and left.arm.active_source is not None,
            left_hand is not None
            and left_hand.safety_state == "tracking"
            and left_hand.active_source is not None,
            right.arm.safety_state == "tracking" and right.arm.active_source is not None,
            right_hand is not None
            and right_hand.safety_state == "tracking"
            and right_hand.active_source is not None,
        ]
        simultaneous = all(values)
        four_stream_ticks += simultaneous
        if simultaneous:
            four_stream_duration_ns += paired_weights_ns[tick_id]
    summary = {
        "four_stream_actionable_coverage": ratio(
            four_stream_duration_ns, sum(paired_weights_ns.values())
        ),
        "four_stream_actionable_tick_ratio": ratio(four_stream_ticks, len(paired_ids)),
        "paired_tick_count": len(paired_ids),
        "maximum_applied_composition_error_rad": max(
            (
                float(row["applied_composition_max_abs_rad"])
                for row in route_rows
                if row["applied_composition_max_abs_rad"] is not None
            ),
            default=None,
        ),
        "route_lookup": {f"{side}.{chain}": row for (side, chain), row in route_lookup.items()},
    }
    return route_rows, joint_rows, tick_rows, state_rows, summary


def _ik_and_hand_metrics(dataset: BagDataset) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    ik_rows: list[dict[str, Any]] = []
    hand_rows: list[dict[str, Any]] = []
    for side in ("left", "right"):
        ticks = [tick for tick in dataset.ticks if tick.side == side]
        arm = [tick.arm for tick in ticks]
        attempts = [item for item in arm if item.has_kinematics]
        mappings = [item for item in arm if item.has_mapping]
        ik_rows.append(
            {
                "side": side,
                "tick_count": len(ticks),
                "mapping_count": len(mappings),
                "mapping_accepted_ratio": ratio(
                    sum(item.mapping_accepted for item in mappings), len(mappings)
                ),
                "translation_clamped_ratio": ratio(
                    sum(item.translation_clamped for item in mappings), len(mappings)
                ),
                "rotation_clamped_ratio": ratio(
                    sum(item.rotation_clamped for item in mappings), len(mappings)
                ),
                "ik_attempt_count": len(attempts),
                "ik_success_ratio": ratio(
                    sum(item.ik_succeeded for item in attempts), len(attempts)
                ),
                "reference_established_count": sum(item.reference_established for item in arm),
                "reference_revoked_count": sum(item.reference_revoked for item in arm),
                **_flatten(
                    "position_residual_m",
                    distribution(item.position_residual_m for item in attempts),
                ),
                **_flatten(
                    "orientation_residual_rad",
                    distribution(item.orientation_residual_rad for item in attempts),
                ),
                "kinematics_reason_counts": json.dumps(
                    Counter(item.kinematics_reason for item in attempts),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
                "mapping_reason_counts": json.dumps(
                    Counter(item.mapping_reason for item in mappings),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
            }
        )
        hands = [tick.hand for tick in ticks if tick.hand is not None]
        intents = [hand for hand in hands if hand.has_intent]
        hand_rows.append(
            {
                "side": side,
                "tick_count": len(hands),
                "intent_present_ratio": ratio(len(intents), len(hands)),
                "new_intent_count": sum(hand.intent_is_new for hand in hands),
                "new_intent_tick_ratio": ratio(
                    sum(hand.intent_is_new for hand in hands), len(hands)
                ),
                "rejection_count": sum(hand.rejection_reason is not None for hand in hands),
                **_flatten(
                    "retarget_confidence",
                    distribution(hand.retarget_confidence for hand in intents),
                ),
                "retarget_status_counts": json.dumps(
                    Counter(hand.retarget_status for hand in intents),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
                "rejection_reason_counts": json.dumps(
                    Counter(
                        hand.rejection_reason for hand in hands if hand.rejection_reason is not None
                    ),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
            }
        )
    return ik_rows, hand_rows


def _source_key(
    *,
    kind: str,
    side: Side,
    source_id: str,
    producer_instance: str,
    transport_epoch: int,
    sequence: int,
) -> tuple[str, Side, str, str, int, int]:
    return (
        kind,
        side,
        source_id,
        producer_instance,
        transport_epoch,
        sequence,
    )


def _causal_join_metrics(
    dataset: BagDataset,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    raw_counts: Counter[tuple[str, Side, str, str, int, int]] = Counter()
    for tracker_item in dataset.trackers:
        raw_counts[
            _source_key(
                kind="tracker",
                side=tracker_item.side,
                source_id=tracker_item.source_id,
                producer_instance=tracker_item.producer_instance,
                transport_epoch=tracker_item.transport_epoch,
                sequence=tracker_item.sequence,
            )
        ] += 1
    for glove_item in dataset.gloves:
        raw_counts[
            _source_key(
                kind="glove",
                side=glove_item.side,
                source_id=glove_item.source_id,
                producer_instance=glove_item.producer_instance,
                transport_epoch=glove_item.transport_epoch,
                sequence=glove_item.sequence,
            )
        ] += 1

    summary_rows: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    all_exact = True
    for side in ("left", "right"):
        ticks = sorted(
            (item for item in dataset.ticks if item.side == side),
            key=lambda item: item.tick_id,
        )
        for chain, kind in (("arm", "tracker"), ("hand", "glove")):
            raw_keys = {key for key in raw_counts if key[0] == kind and key[1] == side}
            row: dict[str, Any] = {
                "side": side,
                "chain": chain,
                "raw_sample_count": sum(raw_counts[key] for key in raw_keys),
                "raw_unique_key_count": len(raw_keys),
                "raw_duplicate_key_count": sum(
                    count - 1 for key, count in raw_counts.items() if key in raw_keys
                ),
            }
            for relation in ("new", "active"):
                references: list[SourceRef] = []
                exact_count = 0
                unique_keys: set[tuple[str, Side, str, str, int, int]] = set()
                for tick in ticks:
                    hand = tick.hand
                    if chain == "arm":
                        source = tick.arm.source if relation == "new" else tick.arm.active_source
                    elif hand is None:
                        source = None
                    else:
                        source = hand.source if relation == "new" else hand.active_source
                    if source is None:
                        continue
                    references.append(source)
                    key = _source_key(
                        kind=kind,
                        side=side,
                        source_id=source.source_id,
                        producer_instance=source.producer_instance,
                        transport_epoch=source.transport_epoch,
                        sequence=source.sequence,
                    )
                    unique_keys.add(key)
                    matches = raw_counts[key]
                    exact_count += matches == 1
                    samples.append(
                        {
                            "tick_id": tick.tick_id,
                            "side": side,
                            "chain": chain,
                            "relation": relation,
                            "source_id": source.source_id,
                            "producer_instance": source.producer_instance,
                            "transport_epoch": source.transport_epoch,
                            "sequence": source.sequence,
                            "raw_match_count": matches,
                            "exact_join": matches == 1,
                        }
                    )
                prefix = f"{relation}_source"
                row[f"{prefix}_reference_count"] = len(references)
                row[f"{prefix}_unique_reference_count"] = len(unique_keys)
                row[f"{prefix}_exact_join_ratio"] = ratio(exact_count, len(references))
                row[f"{prefix}_unique_raw_coverage"] = ratio(
                    len(unique_keys & raw_keys), len(raw_keys)
                )
                if exact_count != len(references):
                    all_exact = False
            summary_rows.append(row)
    return (
        summary_rows,
        samples,
        {
            "all_source_references_join_exactly_once": all_exact,
            "raw_duplicate_key_count": sum(count - 1 for count in raw_counts.values()),
        },
    )


def _receipt_metrics(
    artifact: RunArtifact,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    input_rows: list[dict[str, Any]] = []
    raw_health = artifact.receipt.get("input_health", {})
    if isinstance(raw_health, dict):
        for route, value in sorted(raw_health.items()):
            if not isinstance(value, dict):
                continue
            inbox = value.get("inbox", {})
            if not isinstance(inbox, dict):
                inbox = {}
            accepted = int(inbox.get("accepted", 0))
            overwritten = int(inbox.get("overwritten", 0))
            pending = int(inbox.get("pending", 0))
            explicit_accounting = "drained" in inbox and "discarded" in inbox
            drained = int(inbox["drained"]) if explicit_accounting else None
            discarded = int(inbox["discarded"]) if explicit_accounting else None
            kind, _, side = route.partition("_")
            input_rows.append(
                {
                    "kind": kind,
                    "side": side,
                    "accepted": accepted,
                    "drained": drained,
                    "discarded": discarded,
                    "overwritten": overwritten,
                    "pending": pending,
                    "explicit_accounting": explicit_accounting,
                    "overwrite_ratio": ratio(overwritten, accepted),
                    "rebinds": int(inbox.get("rebinds", 0)),
                    "rejected_old_epoch": int(inbox.get("rejected_old_epoch", 0)),
                    "rejected_old_producer": int(inbox.get("rejected_old_producer", 0)),
                    "rejected_sequence": int(inbox.get("rejected_sequence", 0)),
                    "rejected_contract": int(value.get("rejected_contract", 0)),
                    "rejected_future_time": int(value.get("rejected_future_time", 0)),
                    "rejected_identity": int(value.get("rejected_identity", 0)),
                    "lifecycle_resets": int(value.get("lifecycle_resets", 0)),
                }
            )
    controller_rows = [
        {"event": str(event), "count": int(count)}
        for event, count in sorted(artifact.receipt.get("controller_health", {}).items())
    ]
    return input_rows, controller_rows


def _q27_samples(dataset: BagDataset) -> list[dict[str, Any]]:
    origin = min((item.times.tick_time_ns for item in dataset.ticks), default=0)
    rows: list[dict[str, Any]] = []
    for tick in sorted(dataset.ticks, key=lambda item: (item.tick_id, item.side)):
        row: dict[str, Any] = {
            "tick_id": tick.tick_id,
            "side": tick.side,
            "time_s": (tick.times.tick_time_ns - origin) / 1e9,
        }
        for prefix, vector in (
            ("pre_feedback", tick.pre_feedback_q27_rad),
            ("applied_target", tick.applied_target_q27_rad),
            ("post_feedback", tick.post_feedback_q27_rad),
        ):
            for index, value in enumerate(vector):
                row[f"{prefix}_j{index}"] = value
        rows.append(row)
    return rows


def _episode_durations_s(
    rows: list[dict[str, Any]],
    *,
    field: str,
) -> list[float]:
    durations: list[float] = []
    active_duration = 0.0
    in_episode = False
    for first, second in pairwise(rows):
        active = bool(first[field])
        interval = max(0.0, float(second["time_s"]) - float(first["time_s"]))
        if active:
            active_duration += interval
            in_episode = True
        elif in_episode:
            durations.append(active_duration)
            active_duration = 0.0
            in_episode = False
    if in_episode:
        durations.append(active_duration)
    return durations


def _route_episode_metrics(tick_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for side in ("left", "right"):
        for chain in ("arm", "hand"):
            rows = sorted(
                (dict(row) for row in tick_rows if row["side"] == side and row["chain"] == chain),
                key=lambda row: int(row["tick_id"]),
            )
            for row in rows:
                row["unactionable"] = not bool(row["actionable"])
                row["degraded"] = row["safety_state"] != "tracking"
                row["no_active_source"] = not bool(row["active_source_present"])
            for field in ("unactionable", "degraded", "no_active_source"):
                durations = _episode_durations_s(rows, field=field)
                result.append(
                    {
                        "side": side,
                        "chain": chain,
                        "episode_kind": field,
                        "episode_count": len(durations),
                        "total_duration_s": sum(durations),
                        **_flatten("duration_s", distribution(durations)),
                    }
                )
    return result


def _skew_metrics(by_tick: dict[int, dict[Side, TickRecord]]) -> list[dict[str, Any]]:
    values: dict[tuple[str, str], list[float | None]] = defaultdict(list)
    for sides in by_tick.values():
        if set(sides) != {"left", "right"}:
            continue
        left = sides["left"]
        right = sides["right"]

        def skew(
            first: SourceRef | None,
            second: SourceRef | None,
            basis: str,
        ) -> float | None:
            if first is None or second is None:
                return None
            first_time = getattr(first, basis)
            second_time = getattr(second, basis)
            if first_time is None or second_time is None:
                return None
            return abs(int(first_time) - int(second_time)) / 1e6

        pairs = {
            "tracker_left_right": (
                left.arm.active_source,
                right.arm.active_source,
            ),
            "glove_left_right": (
                None if left.hand is None else left.hand.active_source,
                None if right.hand is None else right.hand.active_source,
            ),
            "left_tracker_glove": (
                left.arm.active_source,
                None if left.hand is None else left.hand.active_source,
            ),
            "right_tracker_glove": (
                right.arm.active_source,
                None if right.hand is None else right.hand.active_source,
            ),
        }
        for pair_name, (first, second) in pairs.items():
            for basis in ("source_time_ns", "receive_time_ns", "callback_time_ns"):
                values[(pair_name, basis)].append(skew(first, second, basis))
    return [
        {"pair": pair_name, "basis": basis, **_flatten("skew_ms", distribution(items))}
        for (pair_name, basis), items in sorted(values.items())
    ]


def _scene_metrics(dataset: BagDataset) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    groups: dict[str, list[Any]] = defaultdict(list)
    for item in dataset.scenes:
        groups[item.prim_path].append(item)
    rows: list[dict[str, Any]] = []
    derived: list[dict[str, Any]] = []
    for prim_path, items in sorted(groups.items()):
        items.sort(key=lambda item: item.recorded_time_ns)
        times = [item.recorded_time_ns for item in items]
        positions = np.asarray([item.position_m for item in items], dtype=np.float64)
        deltas = np.diff(positions, axis=0)
        path_length = float(np.sum(np.linalg.norm(deltas, axis=1))) if len(items) > 1 else 0.0
        displacement = float(np.linalg.norm(positions[-1] - positions[0])) if len(items) else None
        velocity_norms = [
            float(np.linalg.norm(item.linear_velocity_m_s))
            if item.linear_velocity_m_s is not None
            else None
            for item in items
        ]
        rows.append(
            {
                "prim_path": prim_path,
                "count": len(items),
                "effective_hz": effective_rate_hz(times),
                "path_length_m": path_length,
                "final_displacement_m": displacement,
                "maximum_height_delta_m": (
                    float(np.max(positions[:, 2]) - positions[0, 2]) if len(items) else None
                ),
                "kinematic_enabled_ratio": ratio(
                    sum(item.kinematic_enabled for item in items), len(items)
                ),
                **_flatten("linear_speed_m_s", distribution(velocity_norms)),
            }
        )
        if items:
            origin = items[0].recorded_time_ns
            for item in items:
                derived.append(
                    {
                        "prim_path": prim_path,
                        "tick_id": item.tick_id,
                        "time_s": (item.recorded_time_ns - origin) / 1e9,
                        "x_m": item.position_m[0],
                        "y_m": item.position_m[1],
                        "z_m": item.position_m[2],
                        "linear_velocity_m_s": (
                            None
                            if item.linear_velocity_m_s is None
                            else float(np.linalg.norm(item.linear_velocity_m_s))
                        ),
                    }
                )
    return rows, derived


def _scene_integrity(
    artifact: RunArtifact,
    dataset: BagDataset,
    unique_tick_count: int,
) -> dict[str, Any]:
    scene = artifact.manifest.get("scene", {})
    expected_paths = (
        tuple(str(value) for value in scene.get("rigid_body_paths", []))
        if isinstance(scene, dict)
        else ()
    )
    observed_paths = sorted({item.prim_path for item in dataset.scenes})
    by_path: dict[str, list[Any]] = defaultdict(list)
    for item in dataset.scenes:
        by_path[item.prim_path].append(item)
    per_path = {
        path: {
            "record_count": len(by_path[path]),
            "unique_tick_count": len({item.tick_id for item in by_path[path]}),
            "duplicate_tick_records": len(by_path[path])
            - len({item.tick_id for item in by_path[path]}),
        }
        for path in observed_paths
    }
    return {
        "expected_dynamic_prim_paths": list(expected_paths),
        "observed_dynamic_prim_paths": observed_paths,
        "path_inventory_matches": set(expected_paths) == set(observed_paths),
        "every_expected_prim_has_one_record_per_tick": bool(expected_paths)
        and all(
            path in per_path
            and per_path[path]["unique_tick_count"] == unique_tick_count
            and per_path[path]["duplicate_tick_records"] == 0
            for path in expected_paths
        ),
        "per_path": per_path,
    }


def _camera_metrics(
    artifact: RunArtifact,
    dataset: BagDataset,
) -> tuple[
    list[dict[str, Any]],
    list[dict[str, Any]],
    list[dict[str, Any]],
    dict[str, Any],
]:
    required = any("/wrist_camera/" in topic for topic in artifact.expected_topics) or isinstance(
        artifact.manifest.get("synthetic_d405_wrist_cameras"),
        dict,
    )
    frames_by_side = {
        side: sorted(
            (frame for frame in dataset.camera_frames if frame.side == side),
            key=lambda frame: frame.camera_frame_index,
        )
        for side in ("left", "right")
    }
    rows: list[dict[str, Any]] = []
    derived: list[dict[str, Any]] = []
    for side, frames in frames_by_side.items():
        rate = effective_rate_hz([frame.stamp_ns for frame in frames])
        rows.append(
            {
                "side": side,
                "frame_count": len(frames),
                "effective_hz": rate,
                "first_frame_index": (None if not frames else frames[0].camera_frame_index),
                "last_frame_index": (None if not frames else frames[-1].camera_frame_index),
                "first_stamp_ns": None if not frames else frames[0].stamp_ns,
                "last_stamp_ns": None if not frames else frames[-1].stamp_ns,
                "finite_depth_pixel_ratio": ratio(
                    sum(frame.finite_depth_pixels for frame in frames),
                    sum(frame.width_px * frame.height_px for frame in frames),
                ),
                "color_payload_bytes": sum(frame.color_payload_bytes for frame in frames),
                "depth_payload_bytes": sum(frame.depth_payload_bytes for frame in frames),
            }
        )
        for frame in frames:
            derived.append(
                {
                    "side": side,
                    "camera_frame_index": frame.camera_frame_index,
                    "stamp_ns": frame.stamp_ns,
                    "capture_sim_time_s": frame.capture_sim_time_s,
                    "control_tick_id": frame.control_tick_id,
                    "physics_substep_index": frame.physics_substep_index,
                    "color_bag_time_ns": frame.color_bag_time_ns,
                    "depth_bag_time_ns": frame.depth_bag_time_ns,
                    "camera_info_bag_time_ns": frame.camera_info_bag_time_ns,
                    "truth_bag_time_ns": frame.truth_bag_time_ns,
                    "host_capture_duration_ms": (
                        frame.host_capture_end_ns - frame.host_capture_start_ns
                    )
                    / 1e6,
                    "finite_depth_pixels": frame.finite_depth_pixels,
                }
            )

    transform_rows = [
        {
            "kind": "static" if item.static else "dynamic",
            "stamp_ns": item.stamp_ns,
            "bag_time_ns": item.bag_time_ns,
            "parent_frame_id": item.parent_frame_id,
            "child_frame_id": item.child_frame_id,
            "parent_from_child_row_major": [
                value for row in item.parent_from_child for value in row
            ],
        }
        for item in dataset.transforms
        if item.child_frame_id.endswith(("_hand_base", "_wrist_camera_optical"))
    ]

    topic_counts = {item.topic: item.count for item in dataset.topics}
    metadata_counts = {
        str(entry["topic_metadata"]["name"]): int(entry["message_count"])
        for entry in artifact.rosbag_metadata["topics_with_message_count"]
    }
    per_side_topic_counts: dict[str, dict[str, int]] = {}
    for side in ("left", "right"):
        counts: dict[str, int] = {}
        for kind, suffix in (
            ("color", "/color/image_raw"),
            ("depth", "/depth/image_raw"),
            ("camera_info", "/camera_info"),
            ("truth", "/frame_truth"),
        ):
            matches = [
                topic
                for topic in artifact.expected_topics
                if f"/{side}/wrist_camera" in topic and topic.endswith(suffix)
            ]
            counts[kind] = topic_counts.get(matches[0], 0) if len(matches) == 1 else -1
            if len(matches) == 1 and metadata_counts.get(matches[0]) != counts[kind]:
                counts[kind] = -1
        per_side_topic_counts[side] = counts
    topic_bundle_counts_match = all(
        all(count == len(frames_by_side[side]) for count in counts.values())
        for side, counts in per_side_topic_counts.items()
    )

    receipt_camera = artifact.receipt.get("synthetic_d405_wrist_cameras")
    receipt_sides = receipt_camera.get("sides") if isinstance(receipt_camera, dict) else None
    receipt_counts: dict[str, dict[str, int]] = {}
    for side in ("left", "right"):
        value = receipt_sides.get(side) if isinstance(receipt_sides, dict) else None
        receipt_counts[side] = {
            "capture_count": int(value.get("capture_count", -1)) if isinstance(value, dict) else -1,
            "publish_count": int(value.get("publish_count", -1)) if isinstance(value, dict) else -1,
        }
    expected_frames_per_side = int(artifact.receipt.get("completed_ticks", 0)) // 2
    receipt_counts_match = all(
        receipt_counts[side]["capture_count"]
        == receipt_counts[side]["publish_count"]
        == len(frames_by_side[side])
        == expected_frames_per_side
        for side in ("left", "right")
    )
    receipt_closed = isinstance(receipt_camera, dict) and receipt_camera.get("closed") is True

    manifest_camera = artifact.manifest.get("synthetic_d405_wrist_cameras")
    inventory = manifest_camera.get("cameras") if isinstance(manifest_camera, dict) else None
    inventory_by_side = (
        {str(value.get("side")): value for value in inventory if isinstance(value, dict)}
        if isinstance(inventory, list)
        else {}
    )
    manifest_matches = set(inventory_by_side) == {"left", "right"}
    if manifest_matches:
        for side, frames in frames_by_side.items():
            item = inventory_by_side[side]
            readback = item.get("api_readback")
            calibration = item.get("derived_calibration")
            profile = item.get("profile")
            capture = profile.get("capture") if isinstance(profile, dict) else None
            expected_static = item.get("hand_base_from_camera_optical_row_major")
            if not (
                isinstance(readback, dict)
                and isinstance(calibration, dict)
                and isinstance(capture, dict)
                and isinstance(expected_static, list)
                and int(readback.get("width_px", -1)) == 640
                and int(readback.get("height_px", -1)) == 480
                and float(capture.get("rate_hz", -1.0)) == 30.0
            ):
                manifest_matches = False
                break
            for frame in frames:
                static_flat = [
                    value for row in frame.hand_base_from_camera_optical for value in row
                ]
                if not (
                    frame.hand_base_frame_id == item.get("parent_frame_id")
                    and frame.optical_frame_id == item.get("optical_frame_id")
                    and np.allclose(
                        frame.k_row_major,
                        calibration.get("k_row_major", ()),
                        rtol=0.0,
                        atol=1e-9,
                    )
                    and np.allclose(
                        frame.d,
                        calibration.get("d", ()),
                        rtol=0.0,
                        atol=1e-12,
                    )
                    and np.allclose(
                        frame.r_row_major,
                        calibration.get("r_row_major", ()),
                        rtol=0.0,
                        atol=1e-12,
                    )
                    and np.allclose(
                        frame.p_row_major,
                        calibration.get("p_row_major", ()),
                        rtol=0.0,
                        atol=1e-9,
                    )
                    and np.allclose(
                        static_flat,
                        expected_static,
                        rtol=0.0,
                        atol=1e-8,
                    )
                ):
                    manifest_matches = False
                    break
            if not manifest_matches:
                break

    dynamic_camera_edges = sum(
        not item.static
        and item.parent_frame_id == "world"
        and item.child_frame_id.endswith("_hand_base")
        for item in dataset.transforms
    )
    static_camera_edges = sum(
        item.static and item.child_frame_id.endswith("_wrist_camera_optical")
        for item in dataset.transforms
    )
    extrinsic_inventory_matches = (
        dynamic_camera_edges == len(dataset.camera_frames) and static_camera_edges == 2
    )
    dual_identity_aligned = bool(frames_by_side["left"]) and [
        (frame.camera_frame_index, frame.stamp_ns) for frame in frames_by_side["left"]
    ] == [(frame.camera_frame_index, frame.stamp_ns) for frame in frames_by_side["right"]]
    rate_matches = all(
        row["effective_hz"] is not None
        and np.isclose(float(row["effective_hz"]), 30.0, rtol=0.0, atol=1e-6)
        for row in rows
    )
    summary = {
        "required": required,
        "frame_count_by_side": {side: len(frames) for side, frames in frames_by_side.items()},
        "expected_frames_per_side": expected_frames_per_side,
        "topic_counts_by_side": per_side_topic_counts,
        "topic_bundle_counts_match": topic_bundle_counts_match,
        "receipt_counts_by_side": receipt_counts,
        "receipt_counts_match": receipt_counts_match,
        "receipt_closed": receipt_closed,
        "manifest_calibration_and_static_extrinsic_match": manifest_matches,
        "dynamic_camera_tf_edge_count": dynamic_camera_edges,
        "static_camera_tf_edge_count": static_camera_edges,
        "extrinsic_inventory_matches": extrinsic_inventory_matches,
        "dual_completed_frame_identities_align": dual_identity_aligned,
        "effective_rate_is_30_hz": rate_matches,
        "reader_fail_closed_integrity_passed": (
            bool(dataset.camera_frames) if required else not dataset.camera_frames
        ),
    }
    return rows, transform_rows, derived, summary


def _capabilities(artifact: RunArtifact) -> list[dict[str, Any]]:
    manifest_capabilities = artifact.manifest.get("capabilities", {})
    rows = (
        [
            {"capability": key, "available": value is not False, "detail": value}
            for key, value in sorted(manifest_capabilities.items())
        ]
        if isinstance(manifest_capabilities, dict)
        else []
    )
    rows.extend(
        (
            {
                "capability": "offline_control_rate_and_jitter",
                "available": True,
                "detail": "tick_time_ns",
            },
            {
                "capability": "offline_source_age",
                "available": True,
                "detail": "active source provenance uses host_monotonic",
            },
            {
                "capability": "offline_q27_composition",
                "available": True,
                "detail": "manifest partitions plus applied_target_q27",
            },
            {
                "capability": "offline_synthetic_d405_integrity",
                "available": isinstance(
                    artifact.manifest.get("synthetic_d405_wrist_cameras"),
                    dict,
                ),
                "detail": "RGB/depth/CameraInfo/frame_truth plus dynamic/static TF closure",
            },
            {
                "capability": "offline_command_feedback_lag",
                "available": False,
                "detail": "no pre-registered dynamic excitation window",
            },
            {
                "capability": "offline_normalized_joint_error",
                "available": False,
                "detail": "analysis ranges are not embedded in the immutable run",
            },
            {
                "capability": "offline_task_success",
                "available": False,
                "detail": "task_truth is absent",
            },
        )
    )
    return rows


def _gate(
    category: Literal["structural", "planned_target"],
    name: str,
    expected: Any,
    observed: Any,
    passed: bool,
) -> dict[str, Any]:
    return {
        "category": category,
        "name": name,
        "expected": expected,
        "observed": observed,
        "passed": passed,
    }


def compute_metrics(
    artifact: RunArtifact,
    dataset: BagDataset,
    config: AnalysisConfig | None = None,
) -> MetricBundle:
    if config is None:
        config = AnalysisConfig()
    simulation_timing = artifact.manifest.get("simulation_timing", {})
    gui = (
        bool(simulation_timing.get("gui", False)) if isinstance(simulation_timing, dict) else False
    )
    effective_p95_tick_interval_limit_ms = (
        config.gui_p95_tick_interval_limit_ms if gui else config.p95_tick_interval_limit_ms
    )
    physics_substeps_per_control = round(config.expected_physics_hz / config.expected_control_hz)
    control_ticks_per_render = round(config.expected_control_hz / config.expected_render_hz)
    timing_ratios_are_integral = bool(
        np.isclose(
            config.expected_physics_hz / config.expected_control_hz,
            physics_substeps_per_control,
            rtol=0.0,
            atol=1e-12,
        )
        and np.isclose(
            config.expected_control_hz / config.expected_render_hz,
            control_ticks_per_render,
            rtol=0.0,
            atol=1e-12,
        )
    )
    topic_rows, topic_summary = _topic_metrics(artifact, dataset)
    source_rows, sequence_rows, source_samples = _source_metrics(dataset, artifact)
    tick_integrity, representatives, by_tick = _tick_integrity(artifact, dataset)
    stage_samples, stage_summary = _stage_metrics(representatives)
    execution_samples, execution_summary = _execution_metrics(representatives)
    route_rows, joint_rows, tick_rows, state_rows, route_summary = _route_metrics(
        artifact, dataset, by_tick
    )
    join_rows, join_samples, join_summary = _causal_join_metrics(dataset)
    receipt_input_rows, controller_health_rows = _receipt_metrics(artifact)
    join_lookup = {(str(row["side"]), str(row["chain"])): row for row in join_rows}
    expected_receipt_routes = {
        ("left", "arm"),
        ("left", "hand"),
        ("right", "arm"),
        ("right", "hand"),
    }
    observed_receipt_routes: set[tuple[str, str]] = set()
    for receipt_row in receipt_input_rows:
        chain = "arm" if receipt_row["kind"] == "tracker" else "hand"
        route_key = (str(receipt_row["side"]), chain)
        observed_receipt_routes.add(route_key)
        join_row = join_lookup.get(route_key)
        selected = None if join_row is None else int(join_row["new_source_reference_count"])
        explicit_accounting = bool(receipt_row["explicit_accounting"])
        if explicit_accounting:
            expected_selected = int(receipt_row["drained"]) - int(
                receipt_row["rejected_future_time"]
            )
            inbox_conserved = int(receipt_row["accepted"]) == (
                int(receipt_row["drained"])
                + int(receipt_row["discarded"])
                + int(receipt_row["overwritten"])
                + int(receipt_row["pending"])
            )
        else:
            expected_selected = (
                int(receipt_row["accepted"])
                - int(receipt_row["overwritten"])
                - int(receipt_row["pending"])
            )
            inbox_conserved = True
        receipt_row["trace_selected_count"] = selected
        receipt_row["expected_trace_selected_count"] = expected_selected
        receipt_row["inbox_conserved"] = inbox_conserved
        receipt_row["accepted_selection_accounted"] = (
            selected == expected_selected and inbox_conserved
        )
    receipt_selection_accounted = observed_receipt_routes == expected_receipt_routes and all(
        bool(row["accepted_selection_accounted"]) for row in receipt_input_rows
    )
    episode_rows = _route_episode_metrics(tick_rows)
    q27_samples = _q27_samples(dataset)
    ik_rows, hand_rows = _ik_and_hand_metrics(dataset)
    skew_rows = _skew_metrics(by_tick)
    scene_rows, scene_samples = _scene_metrics(dataset)
    scene_integrity = _scene_integrity(
        artifact,
        dataset,
        int(tick_integrity["unique_tick_count"]),
    )
    camera_rows, camera_transform_rows, camera_samples, camera_summary = _camera_metrics(
        artifact,
        dataset,
    )
    capability_rows = _capabilities(artifact)

    control_times = [tick.times.tick_time_ns for tick in representatives]
    control_intervals, invalid_control_intervals = _intervals_ms(control_times)
    control_rate = effective_rate_hz(control_times)
    interval_summary = distribution(control_intervals)
    planned_period_ms = 1000.0 / config.expected_control_hz
    planned_period_miss_ratio = ratio(
        sum(value > planned_period_ms for value in control_intervals),
        len(control_intervals),
    )
    p95_limit_exceedance_ratio = ratio(
        sum(value > effective_p95_tick_interval_limit_ms for value in control_intervals),
        len(control_intervals),
    )
    if execution_summary["execution_facts_complete"] and representatives:
        exclusive_stage_names = ("snapshot_ms", "control_ms", "apply_ms", "physics_ms")
    else:
        exclusive_stage_names = ("spin_ms", "control_ms", "apply_ms", "world_step_ms")
    exclusive_stage_p95 = {name: stage_summary[name]["p95"] for name in exclusive_stage_names}
    dominant_exclusive_stage_by_p95 = max(
        exclusive_stage_names,
        key=lambda name: float(exclusive_stage_p95[name] or float("-inf")),
    )
    status_states = [item.state for item in dataset.statuses]
    sequence_anomalies = sum(
        int(row["inferred_missing"]) + int(row["duplicates"]) + int(row["reordered"])
        for row in sequence_rows
    )
    composition_max = route_summary["maximum_applied_composition_error_rad"]
    v2_trace = execution_summary["trace_schema_counts"] == {TICK_SCHEMA_V2: len(representatives)}
    v2_has_explicit_inbox_accounting = observed_receipt_routes == expected_receipt_routes and all(
        bool(row["explicit_accounting"]) for row in receipt_input_rows
    )
    gates = [
        _gate("structural", "receipt_complete", "complete", artifact.receipt["state"], True),
        _gate(
            "structural",
            "expected_topics_nonempty",
            f"{len(artifact.expected_topics)}/{len(artifact.expected_topics)}",
            f"{topic_summary['expected_nonempty_topic_count']}/{len(artifact.expected_topics)}",
            not topic_summary["missing_or_empty_expected_topics"],
        ),
        _gate(
            "structural",
            "all_recorded_messages_schema_validated",
            topic_summary["message_count"],
            topic_summary["validated_message_count"],
            bool(topic_summary["all_observed_messages_validated"]),
        ),
        _gate(
            "structural",
            "decoded_counts_match_rosbag_metadata",
            topic_summary["metadata_message_count"],
            topic_summary["message_count"],
            bool(topic_summary["all_decoded_counts_match_metadata"]),
        ),
        _gate(
            "structural",
            "recording_status_closed",
            "started and consumer_completed",
            ",".join(status_states),
            "started" in status_states and "consumer_completed" in status_states,
        ),
        _gate(
            "structural",
            "tick_count_matches_receipt",
            tick_integrity["receipt_completed_ticks"],
            tick_integrity["unique_tick_count"],
            tick_integrity["unique_tick_count"] == tick_integrity["receipt_completed_ticks"],
        ),
        _gate(
            "structural",
            "two_sides_per_tick",
            "no unpaired, duplicate, or missing tick ids",
            {
                "unpaired": tick_integrity["unpaired_tick_count"],
                "duplicates": tick_integrity["duplicate_side_records"],
                "missing_ids": tick_integrity["inferred_missing_tick_ids"],
            },
            tick_integrity["unpaired_tick_count"] == 0
            and tick_integrity["duplicate_side_records"] == 0
            and tick_integrity["inferred_missing_tick_ids"] == 0,
        ),
        _gate(
            "structural",
            "side_stage_times_identical",
            0,
            tick_integrity["side_time_mismatch_count"],
            tick_integrity["side_time_mismatch_count"] == 0,
        ),
        _gate(
            "structural",
            "tick_trace_schema_uniform",
            "one supported trace schema per run",
            execution_summary["trace_schema_counts"],
            bool(execution_summary["uniform_trace_schema"]),
        ),
        _gate(
            "structural",
            "four_route_traces_present",
            0,
            tick_integrity["missing_hand_route_records"],
            tick_integrity["missing_hand_route_records"] == 0,
        ),
        _gate(
            "structural",
            "raw_source_observed_sequences_continuous",
            0,
            sequence_anomalies,
            sequence_anomalies == 0,
        ),
        _gate(
            "structural",
            "tick_source_references_join_raw_inputs",
            "every reference joins exactly one raw q21/tracker sample; no duplicate raw keys",
            join_summary,
            bool(join_summary["all_source_references_join_exactly_once"])
            and join_summary["raw_duplicate_key_count"] == 0,
        ),
        _gate(
            "structural",
            "receipt_inbox_selection_accounted",
            (
                "accepted = drained + discarded + overwritten + pending; "
                "drained = trace-selected + rejected-future for all four routes"
            ),
            [
                {
                    "side": row["side"],
                    "kind": row["kind"],
                    "accepted": row["accepted"],
                    "drained": row["drained"],
                    "discarded": row["discarded"],
                    "trace_selected": row["trace_selected_count"],
                    "overwritten": row["overwritten"],
                    "pending": row["pending"],
                }
                for row in receipt_input_rows
            ],
            receipt_selection_accounted,
        ),
        _gate(
            "structural",
            "q27_composition_exact",
            f"<= {config.q27_composition_atol_rad:g} rad",
            composition_max,
            composition_max is not None
            and float(composition_max) <= config.q27_composition_atol_rad,
        ),
        _gate(
            "structural",
            "dynamic_scene_per_tick",
            "manifest dynamic prim inventory; exactly one state per control tick",
            scene_integrity,
            bool(scene_integrity["path_inventory_matches"])
            and bool(scene_integrity["every_expected_prim_has_one_record_per_tick"]),
        ),
    ]
    if v2_trace:
        expected_physics_steps = len(representatives) * physics_substeps_per_control
        receipt_physics_steps = int(artifact.receipt.get("completed_physics_steps", -1))
        receipt_renders = int(artifact.receipt.get("completed_renders", -1))
        configured_timing = artifact.receipt.get("configured_timing", {})
        manifest_timing_matches = (
            timing_ratios_are_integral
            and isinstance(simulation_timing, dict)
            and (
                float(simulation_timing.get("physics_hz", -1.0)) == config.expected_physics_hz
                and float(simulation_timing.get("control_hz", -1.0)) == config.expected_control_hz
                and float(simulation_timing.get("rendering_hz", -1.0)) == config.expected_render_hz
                and int(simulation_timing.get("physics_substeps_per_control", -1))
                == physics_substeps_per_control
                and int(simulation_timing.get("control_ticks_per_render", -1))
                == control_ticks_per_render
            )
        )
        receipt_timing_matches = (
            timing_ratios_are_integral
            and isinstance(configured_timing, dict)
            and (
                float(configured_timing.get("physics_hz", -1.0)) == config.expected_physics_hz
                and float(configured_timing.get("control_hz", -1.0)) == config.expected_control_hz
                and float(configured_timing.get("render_hz", -1.0)) == config.expected_render_hz
                and int(configured_timing.get("physics_substeps_per_control", -1))
                == physics_substeps_per_control
                and int(configured_timing.get("control_ticks_per_render", -1))
                == control_ticks_per_render
            )
        )
        gates.extend(
            (
                _gate(
                    "structural",
                    "v2_execution_facts_complete",
                    (
                        "one execution record and "
                        f"{physics_substeps_per_control} consecutive physics substeps per tick"
                    ),
                    execution_summary,
                    bool(execution_summary["execution_facts_complete"])
                    and execution_summary["physics_substep_count"] == expected_physics_steps
                    and bool(execution_summary["consecutive_physics_substep_indices"])
                    and bool(execution_summary["sequential_control_indices"])
                    and bool(execution_summary["strictly_increasing_schedule_slots"])
                    and bool(execution_summary["schedule_gap_matches_missed_periods"])
                    and bool(execution_summary["simulation_time_continuous"]),
                ),
                _gate(
                    "structural",
                    "v2_explicit_mailbox_accounting",
                    "drained and discarded counters on all four routes",
                    v2_has_explicit_inbox_accounting,
                    v2_has_explicit_inbox_accounting,
                ),
                _gate(
                    "structural",
                    "v2_timing_configuration_consistent",
                    (
                        "manifest and receipt match analyzer timing: "
                        f"{physics_substeps_per_control} physics/control and "
                        f"{control_ticks_per_render} control/render"
                    ),
                    {
                        "manifest": simulation_timing,
                        "receipt": configured_timing,
                    },
                    manifest_timing_matches and receipt_timing_matches,
                ),
                _gate(
                    "structural",
                    "receipt_physics_count_matches_trace",
                    expected_physics_steps,
                    receipt_physics_steps,
                    receipt_physics_steps == expected_physics_steps,
                ),
                _gate(
                    "structural",
                    "receipt_render_count_matches_trace",
                    execution_summary["rendered_tick_count"],
                    receipt_renders,
                    receipt_renders == execution_summary["rendered_tick_count"]
                    and bool(execution_summary["sequential_render_indices"]),
                ),
                _gate(
                    "structural",
                    "headless_has_no_render_ticks",
                    0 if not gui else "GUI enabled",
                    execution_summary["rendered_tick_count"],
                    gui or execution_summary["rendered_tick_count"] == 0,
                ),
            )
        )
    if camera_summary["required"]:
        camera_structural_checks = (
            "reader_fail_closed_integrity_passed",
            "topic_bundle_counts_match",
            "receipt_counts_match",
            "receipt_closed",
            "manifest_calibration_and_static_extrinsic_match",
            "extrinsic_inventory_matches",
            "dual_completed_frame_identities_align",
        )
        for name in camera_structural_checks:
            gates.append(
                _gate(
                    "structural",
                    f"d405_{name}",
                    True,
                    camera_summary[name],
                    bool(camera_summary[name]),
                )
            )
    rate_low = config.expected_control_hz * (1.0 - config.control_rate_tolerance_fraction)
    rate_high = config.expected_control_hz * (1.0 + config.control_rate_tolerance_fraction)
    gates.append(
        _gate(
            "planned_target",
            "control_rate",
            f"{config.expected_control_hz:g} Hz ±{config.control_rate_tolerance_fraction * 100:g}%",
            control_rate,
            control_rate is not None and rate_low <= control_rate <= rate_high,
        )
    )
    interval_p95 = interval_summary["p95"]
    gates.append(
        _gate(
            "planned_target",
            "p95_tick_interval",
            f"<= {effective_p95_tick_interval_limit_ms:g} ms",
            interval_p95,
            interval_p95 is not None
            and float(interval_p95) <= effective_p95_tick_interval_limit_ms,
        )
    )
    for row in route_rows:
        observed = row["input_age_ms_p95"]
        gates.append(
            _gate(
                "planned_target",
                f"{row['side']}_{row['chain']}_p95_comparable_input_age",
                f"< {config.p95_comparable_input_age_limit_ms:g} ms",
                {"value_ms": observed, "basis": row["input_age_basis"]},
                observed is not None and float(observed) < config.p95_comparable_input_age_limit_ms,
            )
        )

    if v2_trace:
        real_time_factor = execution_summary["real_time_factor"]
        gates.append(
            _gate(
                "planned_target",
                "physics_real_time_factor",
                f">= {config.minimum_real_time_factor:g}",
                real_time_factor,
                real_time_factor is not None
                and float(real_time_factor) >= config.minimum_real_time_factor,
            )
        )
        gates.append(
            _gate(
                "planned_target",
                "missed_control_periods",
                0,
                execution_summary["missed_control_periods"],
                execution_summary["missed_control_periods"] == 0,
            )
        )
        substep_dt = execution_summary["physics_substep_simulation_dt_s"]
        expected_dt_s = 1.0 / config.expected_physics_hz
        minimum_dt = substep_dt["minimum"]
        maximum_dt = substep_dt["maximum"]
        dt_matches = (
            minimum_dt is not None
            and maximum_dt is not None
            and np.isclose(float(minimum_dt), expected_dt_s, rtol=0.0, atol=1e-9)
            and np.isclose(float(maximum_dt), expected_dt_s, rtol=0.0, atol=1e-9)
        )
        gates.append(
            _gate(
                "planned_target",
                "physics_substep_dt",
                f"{expected_dt_s:g} s",
                {"minimum": minimum_dt, "maximum": maximum_dt},
                bool(dt_matches),
            )
        )
        if gui:
            render_rate = execution_summary["render_effective_hz"]
            render_low = config.expected_render_hz * (1.0 - config.render_rate_tolerance_fraction)
            render_high = config.expected_render_hz * (1.0 + config.render_rate_tolerance_fraction)
            gates.append(
                _gate(
                    "planned_target",
                    "render_rate",
                    (
                        f"{config.expected_render_hz:g} Hz "
                        f"±{config.render_rate_tolerance_fraction * 100:g}%"
                    ),
                    render_rate,
                    render_rate is not None and render_low <= float(render_rate) <= render_high,
                )
            )
            render_stride = config.expected_control_hz / config.expected_render_hz
            expected_stride = round(render_stride)
            rendered_indices = execution_summary["rendered_control_indices"]
            expected_rendered_indices = [
                execution.control_index
                for execution in (tick.execution for tick in representatives)
                if execution is not None
                and expected_stride > 0
                and (execution.control_index + 1) % expected_stride == 0
            ]
            gates.append(
                _gate(
                    "planned_target",
                    "render_cadence",
                    f"every {expected_stride} control ticks",
                    rendered_indices,
                    bool(
                        np.isclose(
                            render_stride,
                            expected_stride,
                            rtol=0.0,
                            atol=1e-12,
                        )
                    )
                    and rendered_indices == expected_rendered_indices,
                )
            )

    if camera_summary["required"]:
        gates.append(
            _gate(
                "planned_target",
                "d405_camera_rate",
                "30 Hz completed-frame identity cadence on both sides",
                [{"side": row["side"], "effective_hz": row["effective_hz"]} for row in camera_rows],
                bool(camera_summary["effective_rate_is_30_hz"]),
            )
        )

    summary = {
        "run_id": artifact.run_id,
        "analysis_window": "full recorded control tick span; no warm-up or task trimming",
        "config": {
            **config.to_mapping(),
            "effective_p95_tick_interval_limit_ms": (effective_p95_tick_interval_limit_ms),
        },
        "artifact": {
            "state": artifact.receipt["state"],
            "input_checksum_count": len(artifact.input_checksums),
            "mcap_segments": len(artifact.mcap_paths),
        },
        "bag": topic_summary,
        "recording_status_states": status_states,
        "ticks": tick_integrity,
        "control": {
            "effective_hz": control_rate,
            "invalid_timestamp_intervals": invalid_control_intervals,
            "planned_period_ms": planned_period_ms,
            "planned_period_miss_ratio": planned_period_miss_ratio,
            "p95_limit_exceedance_ratio": p95_limit_exceedance_ratio,
            "interval_ms": interval_summary,
            "stage_ms": stage_summary,
            "execution": execution_summary,
            "exclusive_stage_p95_ms": exclusive_stage_p95,
            "dominant_exclusive_stage_by_p95": dominant_exclusive_stage_by_p95,
        },
        "routes": route_summary,
        "causal_join": join_summary,
        "receipt_inbox_selection_accounted": receipt_selection_accounted,
        "scene_integrity": scene_integrity,
        "scene_object_count": len(scene_rows),
        "synthetic_d405_wrist_cameras": camera_summary,
        "structural_gates_passed": all(
            row["passed"] for row in gates if row["category"] == "structural"
        ),
        "planned_targets_passed": all(
            row["passed"] for row in gates if row["category"] == "planned_target"
        ),
        "unsupported_quantitative_conclusions": [
            "task success or failure",
            "contact, force, penetration, palm or fingertip quality",
            "real NERO or Hand2 hardware following quality",
            "command-feedback lag without a pre-registered excited window",
            "normalized arm-versus-hand ranking without embedded analysis ranges",
            "glove sensor-to-host latency because source acquisition time is unavailable",
            "table or bowl runtime drift because fixed bodies are manifest-only",
        ],
    }
    tables = {
        "topic_inventory": tuple(topic_rows),
        "source_metrics": tuple(source_rows),
        "sequence_metrics": tuple(sequence_rows),
        "route_metrics": tuple(route_rows),
        "causal_join_metrics": tuple(join_rows),
        "route_episode_metrics": tuple(episode_rows),
        "joint_metrics": tuple(joint_rows),
        "state_counts": tuple(state_rows),
        "ik_metrics": tuple(ik_rows),
        "hand_metrics": tuple(hand_rows),
        "source_skew_metrics": tuple(skew_rows),
        "scene_metrics": tuple(scene_rows),
        "camera_integrity": tuple(camera_rows),
        "camera_transforms": tuple(camera_transform_rows),
        "receipt_input_metrics": tuple(receipt_input_rows),
        "receipt_controller_health": tuple(controller_health_rows),
        "capabilities": tuple(capability_rows),
        "gates": tuple(gates),
    }
    derived_tables = {
        "aligned_ticks": tuple(tick_rows),
        "stage_samples": tuple(stage_samples),
        "execution_samples": tuple(execution_samples),
        "source_samples": tuple(source_samples),
        "source_join_samples": tuple(join_samples),
        "q27_samples": tuple(q27_samples),
        "scene_samples": tuple(scene_samples),
        "camera_frames": tuple(camera_samples),
    }
    return MetricBundle(summary=summary, tables=tables, derived_tables=derived_tables)


__all__ = ["AnalysisConfig", "MetricBundle", "compute_metrics"]
