#!/usr/bin/env python3
# ruff: noqa: E402  # Repository source paths are resolved before local imports.
"""Fail-closed validator for the deterministic ROS2-Isaac-GUI fixture run."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import sys
from typing import Any, cast

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "analysis/teleoperation_quality/src"))

from teleoperation_quality.model import BagDataset, TickRecord
from teleoperation_quality.ros2_reader import Ros2BagReader
from wujihand.application.qualification.dataset_preview_fixture import (
    FIXTURE_PRODUCER,
    FIXTURE_PROFILE_ID,
    MOTION_FRAMES,
    REFERENCE_FRAMES,
    REQUIRED_FRAMES,
    RETURN_FRAMES,
    fixture_profile_sha256,
)
from wujihand.dataset import load_mini_dataset_profile


MIN_GROUP_DELTA_RAD = 0.05
MAX_A_RETURN_DELTA_RAD = 0.10
STATE_CLOSURE_LIMIT = 2e-5


def _mapping(value: object, *, field: str) -> dict[str, Any]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(dict[str, Any], value)


def _contains_fields(
    value: object,
    expected: dict[str, object] | None,
) -> bool:
    return expected is None or (
        isinstance(value, dict)
        and all(value.get(key) == item for key, item in expected.items())
    )


def _load_json(path: Path) -> dict[str, Any]:
    return _mapping(json.loads(path.read_text(encoding="utf-8")), field=str(path))


def _expected_preview_component_source_counts(
    deployment: dict[str, Any],
) -> dict[str, int]:
    identity = (
        deployment.get("deployment_id"),
        deployment.get("session_id"),
        deployment.get("assembly_path"),
    )
    known = {
        (
            "isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v3",
            "isaac_nero_dual_hand2_triview_q54_mini_dataset_v1",
            (
                "configs/assemblies/"
                "nero_dual_hand2_d405_wrist_rig_simulation_nominal_v2026_6_27_v1.yaml"
            ),
        ): {"left_arm": 8, "left_hand": 27, "right_arm": 8, "right_hand": 27},
        (
            "isaac_nero_hand2_ros_dual_triview_q54_mini_dataset_v2026_8_3_v1",
            "isaac_nero_dual_hand2_triview_q54_mini_dataset_v2026_8_3_v1",
            (
                "configs/assemblies/"
                "nero_dual_hand2_d405_wrist_rig_simulation_nominal_v2026_8_3_v1.yaml"
            ),
        ): {"left_arm": 8, "left_hand": 26, "right_arm": 8, "right_hand": 26},
        (
            "isaac_nero_hand2_ros_dual_tframe_triview_q54_v2026_8_3_v1",
            "isaac_nero_dual_hand2_tframe_triview_q54_v2026_8_3_v1",
            (
                "configs/assemblies/"
                "nero_dual_hand2_d405_wrist_rig_tframe_v2026_8_3_v1.yaml"
            ),
        ): {"left_arm": 8, "left_hand": 26, "right_arm": 8, "right_hand": 26},
        (
            (
                "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
                "triview_q54_self_collision_v1"
            ),
            (
                "isaac_nero_dual_hand2_tframe_gripper_flange_collision_proxy_"
                "triview_q54_self_collision_v1"
            ),
            (
                "configs/assemblies/"
                "nero_dual_hand2_d405_wrist_rig_tframe_"
                "gripper_flange_collision_proxy_v1.yaml"
            ),
        ): {"left_arm": 9, "left_hand": 26, "right_arm": 9, "right_hand": 26},
    }
    try:
        return known[identity]
    except KeyError as exc:
        raise ValueError(
            f"preview component inventory is not qualified for deployment identity {identity!r}"
        ) from exc


def _plateau(sequence: int) -> str | None:
    margin = 120
    if REFERENCE_FRAMES - margin <= sequence < REFERENCE_FRAMES:
        return "a_reference"
    motion_end = REFERENCE_FRAMES + MOTION_FRAMES
    if motion_end - margin <= sequence < motion_end:
        return "b_motion"
    return_end = motion_end + RETURN_FRAMES
    if return_end - margin <= sequence < return_end:
        return "a_return"
    return None


def _plateau_commands(
    dataset: BagDataset,
) -> tuple[dict[str, dict[str, dict[str, list[float]]]], dict[str, int]]:
    values: dict[tuple[str, str, str], list[np.ndarray]] = defaultdict(list)
    counts: dict[str, int] = defaultdict(int)
    for tick in dataset.ticks:
        hand = tick.hand
        if tick.arm.active_source is None or hand is None or hand.active_source is None:
            continue
        if (
            tick.arm.active_source.producer_instance != FIXTURE_PRODUCER
            or hand.active_source.producer_instance != FIXTURE_PRODUCER
            or tick.arm.active_source.transport_epoch != 1
            or hand.active_source.transport_epoch != 1
        ):
            continue
        phase = _plateau(min(tick.arm.active_source.sequence, hand.active_source.sequence))
        if phase is None:
            continue
        values[(phase, tick.side, "arm_q7")].append(
            np.asarray(tick.arm.command_q7_rad, dtype=np.float64)
        )
        values[(phase, tick.side, "hand_q20")].append(
            np.asarray(hand.command_q20_rad, dtype=np.float64)
        )
        counts[phase] += 1
    result: dict[str, dict[str, dict[str, list[float]]]] = {}
    for phase in ("a_reference", "b_motion", "a_return"):
        result[phase] = {}
        for side in ("left", "right"):
            result[phase][side] = {}
            for group in ("arm_q7", "hand_q20"):
                samples = values[(phase, side, group)]
                if not samples:
                    raise ValueError(f"missing {phase}/{side}/{group} fixture plateau")
                result[phase][side][group] = np.median(
                    np.stack(samples),
                    axis=0,
                ).tolist()
    return result, dict(counts)


def _max_delta(left: list[float], right: list[float]) -> float:
    return float(
        np.max(
            np.abs(np.asarray(left, dtype=np.float64) - np.asarray(right, dtype=np.float64)),
            initial=0.0,
        )
    )


def _command_deltas(
    plateaus: dict[str, dict[str, dict[str, list[float]]]],
) -> tuple[dict[str, dict[str, float]], dict[str, dict[str, float]]]:
    motion: dict[str, dict[str, float]] = {}
    returned: dict[str, dict[str, float]] = {}
    for side in ("left", "right"):
        motion[side] = {}
        returned[side] = {}
        for group in ("arm_q7", "hand_q20"):
            reference = plateaus["a_reference"][side][group]
            motion[side][group] = _max_delta(
                reference,
                plateaus["b_motion"][side][group],
            )
            returned[side][group] = _max_delta(
                reference,
                plateaus["a_return"][side][group],
            )
    return motion, returned


def _q54_closure(dataset: BagDataset, profile: Any) -> tuple[float, dict[str, float]]:
    ticks: dict[tuple[int, str], TickRecord] = {
        (item.tick_id, item.side): item for item in dataset.ticks
    }
    maximum_error = 0.0
    minimum = np.full(54, math.inf, dtype=np.float64)
    maximum = np.full(54, -math.inf, dtype=np.float64)
    post_count = 0
    for state in dataset.simulation_states:
        if state.phase != "post_action":
            continue
        left = ticks.get((state.control_index, "left"))
        right = ticks.get((state.control_index, "right"))
        if left is None or right is None:
            raise ValueError("post-action q54 does not join to both side tick traces")
        expected = np.asarray(
            profile.q54.assemble_from_q27(
                left_q27_rad=left.post_feedback_q27_rad,
                right_q27_rad=right.post_feedback_q27_rad,
            ),
            dtype=np.float64,
        )
        actual = np.asarray(state.q54_rad, dtype=np.float64)
        maximum_error = max(
            maximum_error,
            float(np.max(np.abs(actual - expected), initial=0.0)),
        )
        minimum = np.minimum(minimum, actual)
        maximum = np.maximum(maximum, actual)
        if len(state.kinematic_links) != 14 or not all(
            item.valid for item in state.kinematic_links
        ):
            raise ValueError("post-action state lacks the complete 14-link truth")
        post_count += 1
    if post_count == 0:
        raise ValueError("qualification MCAP contains no post-action q54 state")
    ranges = maximum - minimum
    return maximum_error, {
        "left_arm_q7": float(np.max(ranges[0:7], initial=0.0)),
        "left_hand_q20": float(np.max(ranges[7:27], initial=0.0)),
        "right_arm_q7": float(np.max(ranges[27:34], initial=0.0)),
        "right_hand_q20": float(np.max(ranges[34:54], initial=0.0)),
    }


def validate(run_root: Path) -> dict[str, Any]:
    root = run_root.resolve()
    if "artifacts/diagnostics/dataset-preview-qualification" not in root.as_posix():
        raise ValueError("qualification run must be isolated under artifacts/diagnostics")
    run_id = root.name
    fixture = _load_json(root / "fixture/receipt.json")
    main = _load_json(root / "receipt.json")
    manifest = _load_json(root / "manifest.json")
    preview = _load_json(root / "derived/live_preview/receipt.json")
    recorder = _load_json(root / "recorder.json")
    dataset_manifest = _mapping(manifest.get("dataset"), field="manifest.dataset")
    deployment_manifest = _mapping(
        manifest.get("deployment"), field="manifest.deployment"
    )
    expected_preview_component_source_counts = (
        _expected_preview_component_source_counts(deployment_manifest)
    )
    profile = load_mini_dataset_profile(ROOT, dataset_manifest["profile_path"])
    bag = Ros2BagReader().read(root / "raw/rosbag2", expected_run_id=run_id)

    plateaus, plateau_counts = _plateau_commands(bag)
    motion_deltas, return_deltas = _command_deltas(plateaus)
    q54_error, q54_ranges = _q54_closure(bag, profile)
    topic_names = {item.topic for item in bag.topics}
    boundaries_isolated = bool(bag.dataset_boundaries) and all(
        item.source_mode == "synthetic_fixture" and not item.dataset_eligible
        for item in bag.dataset_boundaries
    )
    raw_identity = bool(bag.trackers and bag.gloves) and all(
        item.producer_instance == FIXTURE_PRODUCER and item.transport_epoch == 1
        for item in (*bag.trackers, *bag.gloves)
    )
    active_ticks = [
        item
        for item in bag.ticks
        if item.arm.active_source is not None
        and item.hand is not None
        and item.hand.active_source is not None
    ]
    provenance_closed = bool(active_ticks) and all(
        item.arm.active_source is not None
        and item.arm.active_source.producer_instance == FIXTURE_PRODUCER
        and item.hand is not None
        and item.hand.active_source is not None
        and item.hand.active_source.producer_instance == FIXTURE_PRODUCER
        for item in active_ticks
    )
    command_motion = all(
        value >= MIN_GROUP_DELTA_RAD for side in motion_deltas.values() for value in side.values()
    )
    command_return = all(
        value <= MAX_A_RETURN_DELTA_RAD
        for side in return_deltas.values()
        for value in side.values()
    )
    preview_groups = _mapping(
        preview.get("source_q54_group_max_range_rad"),
        field="preview source q54 groups",
    )
    preview_component_source_counts = _mapping(
        preview.get("component_source_pose_counts"),
        field="preview component source counts",
    )
    preview_component_replay_counts = _mapping(
        preview.get("component_replay_pose_counts"),
        field="preview component replay counts",
    )
    preview_component_renderable_counts = _mapping(
        preview.get("component_renderable_geometry_counts"),
        field="preview component renderable counts",
    )
    preview_component_motion_required = _mapping(
        preview.get("component_source_motion_required"),
        field="preview component source motion required",
    )
    preview_component_motion_passed = _mapping(
        preview.get("component_renderable_motion_passed"),
        field="preview component renderable motion passed",
    )
    expected_task_scene = (
        {
            "path": "configs/scenes/isaac_robolab_banana_bowl_low_table_v2.yaml",
            "profile_id": "isaac_robolab_banana_bowl_low_table_v2",
        }
        if deployment_manifest.get("deployment_id")
        == "isaac_nero_hand2_ros_dual_tframe_triview_q54_v2026_8_3_v1"
        else None
    )
    resolved_control = _mapping(
        manifest.get("resolved_control_artifacts"),
        field="manifest resolved control artifacts",
    )
    raw_record_chain = resolved_control.get("record_chain_preflight")
    record_chain = (
        {}
        if raw_record_chain is None
        else _mapping(
            raw_record_chain,
            field="manifest record-chain preflight",
        )
    )
    main_scene_plan = _mapping(
        _mapping(manifest.get("scene"), field="manifest scene").get("plan"),
        field="manifest scene plan",
    )
    preview_scene_plan = _mapping(
        _mapping(preview.get("scene"), field="preview scene").get("plan"),
        field="preview scene plan",
    )
    acceptance = {
        "fixture_profile": (
            fixture.get("passed") is True
            and fixture.get("profile_id") == FIXTURE_PROFILE_ID
            and fixture.get("run_id") == run_id
            and isinstance(fixture.get("recording_started_barrier_host_time_ns"), int)
            and fixture.get("profile_sha256") == fixture_profile_sha256()
            and int(fixture.get("completed_frames", 0)) >= REQUIRED_FRAMES
            and int(fixture.get("missed_periods", -1)) == 0
            and fixture.get("python_gc_frozen_during_run") is True
            and int(fixture.get("python_gc_frozen_object_count", 0)) > 0
        ),
        "synthetic_isolation": (
            dataset_manifest.get("source_mode") == "synthetic_fixture"
            and dataset_manifest.get("dataset_eligible") is False
            and boundaries_isolated
        ),
        "raw_source_identity": raw_identity,
        "active_provenance": provenance_closed,
        "command_a_b_motion": command_motion,
        "command_a_return": command_return,
        "q54_command_closure": q54_error < STATE_CLOSURE_LIMIT,
        "q54_four_group_motion": all(value >= MIN_GROUP_DELTA_RAD for value in q54_ranges.values()),
        "main_complete": (
            main.get("consumer_state") == "consumer_completed"
            and main.get("failure_reason") is None
            and main.get("recording_failure_reason") is None
        ),
        "main_60hz_zero_miss": (
            _mapping(main.get("controller_health"), field="main controller health").get(
                "scheduler.missed_control_periods",
                -1,
            )
            == 0
        ),
        "physics_120hz_ratio": (
            int(main.get("completed_physics_steps", -1)) == 2 * int(main.get("completed_ticks", 0))
        ),
        "preview_20hz_zero_miss": (
            preview.get("passed") is True
            and int(preview.get("missed_render_periods", -1)) == 0
            and abs(float(preview.get("effective_render_hz", 0.0)) - 20.0) / 20.0 <= 0.05
        ),
        "task_scene_and_preview_visual_policy": (
            _contains_fields(record_chain.get("task_scene"), expected_task_scene)
            and (
                expected_task_scene is None
                or preview.get("record_chain_task_scene")
                == record_chain.get("task_scene")
            )
            and (
                expected_task_scene is None
                or main_scene_plan.get("task_scene_profile_id")
                == expected_task_scene["profile_id"]
            )
            and (
                expected_task_scene is None
                or preview_scene_plan.get("task_scene_profile_id")
                == expected_task_scene["profile_id"]
            )
            and preview.get("background_color_rgb_readback") == [0.3, 0.3, 0.3]
            and float(preview.get("render_max_ms", math.inf)) < 50.0
        ),
        "preview_q54_four_group_motion": all(
            float(preview_groups.get(group, 0.0)) >= MIN_GROUP_DELTA_RAD for group in q54_ranges
        ),
        "preview_link_closure": (
            int(preview.get("source_kinematic_link_count", 0)) > 14
            and preview.get("pose_replay_backend") == "usd_parent_first"
            and preview.get("pose_write_backend") == "usd"
            and preview.get("pose_readback_backend") == "usd"
            and preview.get("pose_application_phase")
            == "synchronous_pre_render_transaction"
            and int(preview.get("pose_apply_count", 0))
            == int(preview.get("source_frames_applied", -4))
            + int(preview.get("qa_pose_apply_count", -1))
            and preview.get("local_physics_replay_enabled") is False
            and preview.get("scene_materialization_mode") == "visual_replay_only"
            and preview.get("simulation_manager_physics_view_active") is False
            and preview.get("world_physics_view_active") is False
            and preview.get("local_physics_simulating") is False
            and preview.get("timeline_stopped_for_replay") is True
            and preview.get("render_transaction")
            == "reference_time_annotated_single_tick_replicator_step_zero_subframes_delta_time_zero"
            and preview.get("multi_tick_rendering_enabled") is False
            and preview.get("viewport_sync_annotator") == "ReferenceTime"
            and preview.get("viewport_sync_annotator_attached") is True
            and str(preview.get("viewport_render_product_path", "")).startswith(
                "/Render/"
            )
            and int(preview.get("pose_closure_checks", 0)) >= 4
            and float(preview.get("pose_position_max_abs_error_m", math.inf))
            < STATE_CLOSURE_LIMIT
            and int(preview.get("renderable_geometry_count", 0)) > 0
            and int(preview.get("replay_pose_prim_count", 0))
            == int(preview.get("renderable_geometry_count", -1))
            and preview_component_source_counts
            == expected_preview_component_source_counts
            and all(
                int(preview_component_replay_counts.get(component, -1))
                == int(preview_component_source_counts.get(component, -2))
                for component in ("left_arm", "right_arm")
            )
            and all(
                int(preview_component_renderable_counts.get(component, 0))
                == int(preview_component_replay_counts.get(component, -1))
                > 0
                for component in ("left_arm", "left_hand", "right_arm", "right_hand")
            )
            and preview.get("renderable_geometry_motion_passed") is True
        ),
        "preview_four_component_motion": all(
            preview_component_motion_required.get(component) is True
            and preview_component_motion_passed.get(component) is True
            for component in ("left_arm", "left_hand", "right_arm", "right_hand")
        ),
        "viewport_visible_motion": preview.get("viewport_visible_motion_passed") is True,
        "viewport_static_repeat": preview.get("viewport_static_repeat_passed") is True,
        "recorder_complete": (
            recorder.get("state") == "exited"
            and recorder.get("exit_code") == 0
            and recorder.get("consumer_terminal_observed") is True
        ),
        "operator_preview_not_recorded": not any(
            "/operator_preview/" in topic for topic in topic_names
        ),
    }
    failures = [name for name, passed in acceptance.items() if not passed]
    return {
        "schema": "wujihand.dataset_preview_fixture_qualification.v1",
        "passed": not failures,
        "run_id": run_id,
        "fixture_profile_id": FIXTURE_PROFILE_ID,
        "fixture_profile_sha256": fixture_profile_sha256(),
        "acceptance": acceptance,
        "failures": failures,
        "observed": {
            "fixture_completed_frames": fixture.get("completed_frames"),
            "fixture_effective_hz": fixture.get("effective_hz"),
            "plateau_counts": plateau_counts,
            "command_plateaus_rad": plateaus,
            "command_motion_max_delta_rad": motion_deltas,
            "command_return_max_delta_rad": return_deltas,
            "q54_max_abs_error_rad": q54_error,
            "q54_group_max_range_rad": q54_ranges,
            "preview_viewport_pixel_changed_fraction": preview.get(
                "viewport_pixel_changed_fraction"
            ),
            "preview_repeat_pixel_max_abs_delta": preview.get(
                "viewport_repeat_pixel_max_abs_delta"
            ),
            "preview_pose_position_max_abs_error_m": preview.get(
                "pose_position_max_abs_error_m"
            ),
            "preview_source_kinematic_link_count": preview.get(
                "source_kinematic_link_count"
            ),
            "preview_renderable_geometry_count": preview.get(
                "renderable_geometry_count"
            ),
            "preview_renderable_geometry_matrix_max_delta": preview.get(
                "renderable_geometry_matrix_max_delta"
            ),
            "preview_pose_apply_count": preview.get(
                "pose_apply_count"
            ),
            "preview_qa_pose_apply_count": preview.get("qa_pose_apply_count"),
            "preview_component_source_pose_counts": preview_component_source_counts,
            "preview_component_replay_pose_counts": preview_component_replay_counts,
            "preview_component_renderable_geometry_counts": (
                preview_component_renderable_counts
            ),
            "preview_component_source_motion_required": (
                preview_component_motion_required
            ),
            "preview_component_renderable_matrix_max_delta": preview.get(
                "component_renderable_matrix_max_delta"
            ),
            "preview_component_renderable_motion_passed": (
                preview_component_motion_passed
            ),
            "preview_capture_frame_numbers": preview.get("viewport_capture_frame_numbers"),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    args = parser.parse_args()
    destination = args.run_root.resolve() / "qualification/receipt.json"
    try:
        result = validate(args.run_root)
    except BaseException as exc:
        result = {
            "schema": "wujihand.dataset_preview_fixture_qualification.v1",
            "passed": False,
            "run_id": args.run_root.name,
            "acceptance": {},
            "failures": ["validator_error"],
            "error": f"{type(exc).__name__}:{exc}",
        }
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
