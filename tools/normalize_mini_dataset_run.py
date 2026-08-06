#!/usr/bin/env python3
# ruff: noqa: E402  # Repository source paths are resolved before local imports.
"""Extract strict 008 release facts from one complete ROS2 MCAP episode."""

from __future__ import annotations

import argparse
from collections import Counter
from collections.abc import Sequence
import json
import math
from pathlib import Path
import sys
from typing import cast


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "analysis/teleoperation_quality/src"))

from teleoperation_quality.artifact import RunArtifact, load_run_artifact
from teleoperation_quality.model import (
    BagDataset,
    DatasetBoundaryRecord,
    DatasetKinematicLinkRecord,
    DatasetRigidBodyRecord,
    SimulationStateRecord,
    SourceRef,
    TickRecord,
)
from teleoperation_quality.ros2_reader import Ros2BagReader
from wujihand.dataset import (
    ControlTickFacts,
    NormalizedEpisodeFacts,
    load_mini_dataset_profile,
    parse_dataset_truth_inventories,
    validate_q54_runtime_inventory,
    validate_state_truth_inventory,
    write_normalized_episode_artifact,
)
from wujihand.dataset.alignment import RawTransition
from wujihand.dataset.release import SourceEpochFact
from wujihand.domain.dataset_recording import (
    DatasetEpisodeBoundary,
    DatasetEpisodeEvent,
    DatasetSourceMode,
    DynamicRigidBodyTruth,
    KinematicLinkTruth,
    SimulationFramePhase,
    SimulationStateFrame,
)


def _mapping(value: object, *, field: str) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _boundary(record: DatasetBoundaryRecord) -> DatasetEpisodeBoundary:
    return DatasetEpisodeBoundary(
        run_id=record.run_id,
        episode_id=record.episode_id,
        collection_id=record.collection_id,
        event=DatasetEpisodeEvent(record.event),
        reason=record.reason,
        host_time_ns=record.host_time_ns,
        control_index=record.control_index,
        tick_id=record.tick_id,
        simulation_time_s=record.simulation_time_s,
        recorder_ready=record.recorder_ready,
        inputs_ready=record.inputs_ready,
        references_ready=record.references_ready,
        scene_settled=record.scene_settled,
        source_mode=DatasetSourceMode(record.source_mode),
        dataset_eligible=record.dataset_eligible,
        requested_signal=record.requested_signal,
        effective_final_control_index=record.effective_final_control_index,
    )


def _rigid(record: DatasetRigidBodyRecord) -> DynamicRigidBodyTruth:
    return DynamicRigidBodyTruth(
        logical_object_id=record.logical_object_id,
        prim_path=record.prim_path,
        position_m=record.position_m,
        quat_wxyz=record.quat_wxyz,
        linear_velocity_m_s=record.linear_velocity_m_s,
        angular_velocity_rad_s=record.angular_velocity_rad_s,
        sleeping=record.sleeping,
        kinematic=record.kinematic,
        valid=record.valid,
    )


def _link(record: DatasetKinematicLinkRecord) -> KinematicLinkTruth:
    return KinematicLinkTruth(
        side=record.side,
        logical_link_id=record.logical_link_id,
        prim_path=record.prim_path,
        position_m=record.position_m,
        quat_wxyz=record.quat_wxyz,
        valid=record.valid,
    )


def _state(record: SimulationStateRecord) -> SimulationStateFrame:
    return SimulationStateFrame(
        run_id=record.run_id,
        episode_id=record.episode_id,
        control_index=record.control_index,
        tick_id=record.tick_id,
        phase=SimulationFramePhase(record.phase),
        simulation_time_s=record.simulation_time_s,
        physics_boundary_index=record.physics_boundary_index,
        q54_rad=record.q54_rad,
        qdot54_rad_s=record.qdot54_rad_s,
        rigid_bodies=tuple(_rigid(item) for item in record.rigid_bodies),
        kinematic_links=tuple(_link(item) for item in record.kinematic_links),
        expected_rigid_body_count=record.expected_rigid_body_count,
        expected_kinematic_link_count=record.expected_kinematic_link_count,
        payload_digest_sha256=record.payload_digest_sha256,
    )


def _source_key(side: str, source: SourceRef) -> tuple[str, str, str, int, int]:
    return (
        side,
        source.source_id,
        source.producer_instance,
        source.transport_epoch,
        source.sequence,
    )


def _raw_source_inventories(dataset: BagDataset) -> tuple[Counter[tuple[object, ...]], ...]:
    trackers: Counter[tuple[object, ...]] = Counter(
        (
            item.side,
            item.source_id,
            item.producer_instance,
            item.transport_epoch,
            item.sequence,
        )
        for item in dataset.trackers
    )
    gloves: Counter[tuple[object, ...]] = Counter(
        (
            item.side,
            item.source_id,
            item.producer_instance,
            item.transport_epoch,
            item.sequence,
        )
        for item in dataset.gloves
    )
    return trackers, gloves


def _require_raw_source(
    inventory: Counter[tuple[object, ...]],
    *,
    side: str,
    source: SourceRef | None,
    field: str,
) -> SourceRef:
    if source is None or inventory[_source_key(side, source)] != 1:
        raise ValueError(f"{field} does not uniquely resolve to one raw source sample")
    return source


def _age_ms(tick_time_ns: int, source: SourceRef, *, field: str) -> float:
    comparable_ns = source.source_time_ns
    if comparable_ns is None:
        comparable_ns = source.receive_time_ns
    age_ns = tick_time_ns - comparable_ns
    if age_ns < 0:
        raise ValueError(f"{field} is newer than the atomic tick cutoff")
    return age_ns / 1e6


def _expected_applied_q27(tick: TickRecord, profile: object) -> tuple[float, ...]:
    # The typed profile is intentionally inspected by stable public fields here;
    # the analysis adapter must never assume Hand2 occupies contiguous q27 columns.
    joints = cast(object, profile)
    values = [0.0] * 27
    typed_joints = cast(Sequence[object], getattr(joints, "joints"))
    for joint in typed_joints:
        if getattr(joint, "side") != tick.side:
            continue
        group = str(getattr(joint, "group"))
        group_index = int(getattr(joint, "group_index"))
        source_index = int(getattr(joint, "source_index_q27"))
        if group == "arm":
            values[source_index] = tick.arm.command_q7_rad[group_index]
        else:
            if tick.hand is None:
                raise ValueError(f"{tick.side} hand route is absent")
            values[source_index] = tick.hand.command_q20_rad[group_index]
    return tuple(values)


def _tick_facts(
    *,
    run_id: str,
    left: TickRecord,
    right: TickRecord,
    states: dict[tuple[int, str], SimulationStateFrame],
    profile: object,
    tracker_inventory: Counter[tuple[object, ...]],
    glove_inventory: Counter[tuple[object, ...]],
) -> ControlTickFacts:
    if (
        left.tick_id != right.tick_id
        or left.times != right.times
        or left.execution != right.execution
    ):
        raise ValueError("left/right tick identity, stage times or execution differ")
    execution = left.execution
    if execution is None:
        raise ValueError("008 requires TeleoperationTickTrace.v2 execution facts")
    by_side = {"left": left, "right": right}
    source_epochs: list[SourceEpochFact] = []
    input_ages: list[tuple[str, float]] = []
    route_facts: set[str] = set()
    for side in ("left", "right"):
        tick = by_side[side]
        tracker = _require_raw_source(
            tracker_inventory,
            side=side,
            source=tick.arm.source,
            field=f"{side} selected tracker",
        )
        active_tracker = _require_raw_source(
            tracker_inventory,
            side=side,
            source=tick.arm.active_source,
            field=f"{side} active tracker",
        )
        if tick.hand is None:
            raise ValueError(f"{side} hand tick is missing")
        glove = _require_raw_source(
            glove_inventory,
            side=side,
            source=tick.hand.source,
            field=f"{side} selected glove",
        )
        active_glove = _require_raw_source(
            glove_inventory,
            side=side,
            source=tick.hand.active_source,
            field=f"{side} active glove",
        )
        del tracker, glove
        route_facts.update(
            {
                f"{side}.tracker.raw_selected",
                f"{side}.glove.q21_selected",
                f"{side}.applied.q27",
            }
        )
        if tick.arm.candidate_q7_rad is not None:
            route_facts.add(f"{side}.arm.q7_candidate")
        if tick.hand.has_intent and tick.hand.intent_q20_rad is not None:
            route_facts.add(f"{side}.hand.q20_intent")
        expected_applied = _expected_applied_q27(tick, profile)
        if any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
            for actual, expected in zip(
                tick.applied_target_q27_rad,
                expected_applied,
                strict=True,
            )
        ):
            raise ValueError(f"{side} q7/q20 commands do not compose to applied q27")
        for source in (active_tracker, active_glove):
            source_epochs.append(
                SourceEpochFact(
                    source_id=source.source_id,
                    producer_instance=source.producer_instance,
                    transport_epoch=source.transport_epoch,
                )
            )
        input_ages.extend(
            (
                (
                    f"{side}.tracker",
                    _age_ms(left.times.tick_time_ns, active_tracker, field="tracker"),
                ),
                (f"{side}.glove", _age_ms(left.times.tick_time_ns, active_glove, field="glove")),
            )
        )

    pre = states[(left.tick_id, SimulationFramePhase.PRE_ACTION.value)]
    post = states[(left.tick_id, SimulationFramePhase.POST_ACTION.value)]
    q54_profile = cast(object, profile)
    assemble = getattr(q54_profile, "assemble_from_q27")
    pre_q54 = tuple(
        assemble(
            left_q27_rad=left.pre_feedback_q27_rad,
            right_q27_rad=right.pre_feedback_q27_rad,
        )
    )
    applied_q54 = tuple(
        assemble(
            left_q27_rad=left.applied_target_q27_rad,
            right_q27_rad=right.applied_target_q27_rad,
        )
    )
    post_q54 = tuple(
        assemble(
            left_q27_rad=left.post_feedback_q27_rad,
            right_q27_rad=right.post_feedback_q27_rad,
        )
    )
    transition = RawTransition(
        run_id=run_id,
        control_index=execution.control_index,
        tick_id=left.tick_id,
        simulation_time_before_s=execution.simulation_time_before_s,
        simulation_time_after_s=execution.simulation_time_after_s,
        pre_feedback_q54_rad=pre_q54,
        applied_target_q54_rad=applied_q54,
        post_feedback_q54_rad=post_q54,
        pre_action_state_digest=pre.payload_digest_sha256,
    )
    return ControlTickFacts(
        transition=transition,
        tick_time_ns=left.times.tick_time_ns,
        schedule_slot=execution.schedule_slot,
        missed_control_periods_before_tick=(execution.missed_control_periods_before_tick),
        physics_substep_indices=execution.physics_substep_indices,
        route_fact_keys=frozenset(route_facts),
        source_epochs=tuple(source_epochs),
        comparable_input_age_ms=tuple(input_ages),
        pre_action_frame=pre,
        post_action_frame=post,
    )


def _fixture_drift(artifact: RunArtifact) -> tuple[float, float]:
    scene = _mapping(artifact.manifest.get("scene"), field="manifest.scene")
    initial_raw = _sequence(scene.get("fixed_body_states"), field="fixed body initial states")
    final_raw = _sequence(
        artifact.receipt.get("final_fixed_body_states"),
        field="fixed body final states",
    )

    def states(raw: Sequence[object], *, field: str) -> dict[str, tuple[tuple[float, ...], ...]]:
        result: dict[str, tuple[tuple[float, ...], ...]] = {}
        for index, item in enumerate(raw):
            value = _mapping(item, field=f"{field}[{index}]")
            path = value.get("prim_path")
            position = value.get("position_m")
            quaternion = value.get("quat_wxyz")
            if not isinstance(path, str):
                raise ValueError(f"{field}[{index}].prim_path differs")
            p = tuple(float(number) for number in _sequence(position, field="position"))
            q = tuple(float(number) for number in _sequence(quaternion, field="quaternion"))
            if len(p) != 3 or len(q) != 4 or path in result:
                raise ValueError(f"{field}[{index}] dimensions or identity differ")
            result[path] = (p, q)
        return result

    initial = states(initial_raw, field="initial")
    final = states(final_raw, field="final")
    if not initial or set(initial) != set(final):
        raise ValueError("fixed fixture initial/final inventories differ")
    translation = 0.0
    rotation = 0.0
    for path in initial:
        first_position, first_quaternion = initial[path]
        last_position, last_quaternion = final[path]
        translation = max(
            translation,
            math.sqrt(
                sum(
                    (first - last) ** 2
                    for first, last in zip(first_position, last_position, strict=True)
                )
            ),
        )
        first_norm = math.sqrt(sum(value * value for value in first_quaternion))
        last_norm = math.sqrt(sum(value * value for value in last_quaternion))
        if first_norm <= 0.0 or last_norm <= 0.0:
            raise ValueError("fixed fixture quaternion is invalid")
        dot = abs(
            sum(first * last for first, last in zip(first_quaternion, last_quaternion, strict=True))
            / (first_norm * last_norm)
        )
        rotation = max(rotation, 2.0 * math.acos(min(1.0, dot)))
    return translation, rotation


def normalize_episode(artifact: RunArtifact, dataset: BagDataset) -> NormalizedEpisodeFacts:
    dataset_manifest = _mapping(artifact.manifest.get("dataset"), field="manifest.dataset")
    profile_path = dataset_manifest.get("profile_path")
    if not isinstance(profile_path, str):
        raise ValueError("manifest dataset profile path is missing")
    dataset_profile = load_mini_dataset_profile(ROOT, profile_path)
    if dataset_manifest.get("profile_sha256") != dataset_profile.file_sha256:
        raise ValueError("manifest dataset profile hash differs")
    runtime_names = validate_q54_runtime_inventory(
        dataset_manifest.get("q54_runtime_inventory"),
        profile=dataset_profile.q54,
    )
    objects, links = parse_dataset_truth_inventories(dataset_manifest)

    boundaries = tuple(
        _boundary(item)
        for item in sorted(dataset.dataset_boundaries, key=lambda item: item.host_time_ns)
    )
    events = {item.event: item for item in boundaries}
    recording = events.get(DatasetEpisodeEvent.RECORDING)
    stopped = events.get(DatasetEpisodeEvent.STOP_REQUESTED)
    if recording is None or recording.control_index is None:
        raise ValueError("episode has no recording boundary")
    if stopped is None or stopped.effective_final_control_index is None:
        raise ValueError("episode has no complete stop boundary")
    first_index = recording.control_index
    final_index = stopped.effective_final_control_index
    if final_index < first_index:
        raise ValueError("episode final index precedes first candidate index")

    state_counts = Counter((item.control_index, item.phase) for item in dataset.simulation_states)
    expected_state_keys = {
        (index, phase)
        for index in range(first_index, final_index + 1)
        for phase in ("pre_action", "post_action")
    }
    if set(state_counts) != expected_state_keys or any(
        count != 1 for count in state_counts.values()
    ):
        raise ValueError("candidate pre/post simulation-state closure differs")
    states = {(item.control_index, item.phase): _state(item) for item in dataset.simulation_states}
    for frame in states.values():
        validate_state_truth_inventory(
            frame,
            run_id=artifact.run_id,
            objects=objects,
            links=links,
        )

    tick_counts = Counter((item.tick_id, item.side) for item in dataset.ticks)
    candidate_tick_keys = {
        (index, side) for index in range(first_index, final_index + 1) for side in ("left", "right")
    }
    if any(tick_counts[key] != 1 for key in candidate_tick_keys):
        raise ValueError("candidate left/right tick closure differs")
    ticks_by_key = {(item.tick_id, item.side): item for item in dataset.ticks}
    tracker_inventory, glove_inventory = _raw_source_inventories(dataset)
    ticks = tuple(
        _tick_facts(
            run_id=artifact.run_id,
            left=ticks_by_key[(index, "left")],
            right=ticks_by_key[(index, "right")],
            states=states,
            profile=dataset_profile.q54,
            tracker_inventory=tracker_inventory,
            glove_inventory=glove_inventory,
        )
        for index in range(first_index, final_index + 1)
    )
    observed_topics = {item.topic for item in dataset.topics}
    recorder_inventory_complete = observed_topics == set(artifact.expected_topics) and all(
        item.count > 0 and item.validated_count == item.count for item in dataset.topics
    )
    fixture_translation, fixture_rotation = _fixture_drift(artifact)
    return NormalizedEpisodeFacts(
        run_id=artifact.run_id,
        boundaries=boundaries,
        ticks=ticks,
        q54_profile_id=dataset_profile.q54.profile_id,
        q54_profile_sha256=dataset_profile.q54.file_sha256,
        q54_runtime_names=runtime_names,
        artifact_complete=True,
        checksums_verified=True,
        recorder_inventory_complete=recorder_inventory_complete,
        unknown_schemas=(),
        fixture_translation_drift_m=fixture_translation,
        fixture_rotation_drift_rad=fixture_rotation,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        artifact = load_run_artifact(arguments.run_root)
        dataset = Ros2BagReader().read(
            artifact.root / "raw" / "rosbag2",
            expected_run_id=artifact.run_id,
        )
        facts = normalize_episode(artifact, dataset)
        normalized = write_normalized_episode_artifact(artifact.root, facts)
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        print(json.dumps({"passed": False, "reason": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "passed": True,
                "run_id": artifact.run_id,
                "tick_count": len(facts.ticks),
                "artifact": str(normalized.root),
                "facts_sha256": normalized.facts_sha256,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
