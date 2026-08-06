"""Deterministic post-hoc quality tables and plots for one policy episode."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import hashlib
from html import escape
import io
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Final, Sequence

from .normalized import load_normalized_episode_artifact
from .policy import PolicyEpisode, load_policy_episode
from .profile import Q54JointProfile
from .release_artifact import load_release_decision_artifact
from .vision import CAMERA_IDS


QUALITY_REPORT_SCHEMA: Final = "wujihand.mini_dataset_quality_report.v1"
QUALITY_ARTIFACT_SCHEMA: Final = "wujihand.mini_dataset_quality_artifact.v1"
_GROUPS: Final = (
    ("left_arm", 0, 7),
    ("left_hand", 7, 27),
    ("right_arm", 27, 34),
    ("right_hand", 34, 54),
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _statistics(values: Sequence[float]) -> tuple[float, float, float, float]:
    if not values:
        raise ValueError("quality statistics require at least one value")
    minimum = min(values)
    maximum = max(values)
    mean = sum(values) / len(values)
    variance = sum((item - mean) ** 2 for item in values) / len(values)
    return minimum, maximum, mean, math.sqrt(variance)


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


def _rms(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("quality RMS requires at least one value")
    return math.sqrt(sum(value * value for value in values) / len(values))


def _fmt(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError("quality output values must be finite")
    return format(value, ".9g")


def _csv_bytes(rows: Sequence[Sequence[object]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def _polyline(
    values: Sequence[float],
    *,
    width: int,
    height: int,
    margin: int,
    minimum: float,
    maximum: float,
) -> str:
    xs: tuple[float, ...]
    if len(values) == 1:
        xs = (width / 2.0,)
    else:
        xs = tuple(
            margin + index * (width - 2 * margin) / (len(values) - 1)
            for index in range(len(values))
        )
    span = max(maximum - minimum, 1e-12)
    points = tuple(
        (
            x,
            height - margin - (value - minimum) * (height - 2 * margin) / span,
        )
        for x, value in zip(xs, values, strict=True)
    )
    return " ".join(f"{_fmt(x)},{_fmt(y)}" for x, y in points)


def _series_svg(
    series: Sequence[tuple[str, Sequence[float]]],
    *,
    title: str,
    y_label: str,
) -> bytes:
    width, height, margin = 1000, 520, 56
    colors = ("#2563eb", "#16a34a", "#dc2626", "#9333ea", "#ea580c")
    non_empty = tuple((name, tuple(values)) for name, values in series if values)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="56" y="30" font-family="sans-serif" font-size="20">{escape(title)}</text>',
        f'<text x="8" y="52" font-family="sans-serif" font-size="13">{escape(y_label)}</text>',
        f'<line x1="{margin}" y1="{height - margin}" x2="{width - margin}" '
        'y2="{height - margin}" stroke="#111"/>',
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height - margin}" '
        'stroke="#111"/>',
    ]
    if non_empty:
        all_values = tuple(value for _, values in non_empty for value in values)
        minimum, maximum = min(all_values), max(all_values)
        for index, ((name, values), color) in enumerate(
            zip(non_empty, colors, strict=False)
        ):
            lines.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="2" points="'
                f'{_polyline(values, width=width, height=height, margin=margin, minimum=minimum, maximum=maximum)}"/>'
            )
            lines.append(
                f'<text x="{margin + index * 190}" y="{height - 14}" '
                f'font-family="sans-serif" font-size="14" fill="{color}">'
                f'{escape(name)}</text>'
            )
    else:
        lines.append(
            '<text x="56" y="90" font-family="sans-serif" font-size="16">No rows.</text>'
        )
    lines.append("</svg>")
    return ("\n".join(lines) + "\n").encode()


def _translation(matrix: Sequence[float]) -> tuple[float, float, float]:
    return matrix[3], matrix[7], matrix[11]


def _rotation_distance(first: Sequence[float], second: Sequence[float]) -> float:
    indices = (0, 1, 2, 4, 5, 6, 8, 9, 10)
    frobenius = sum(first[index] * second[index] for index in indices)
    return math.acos(max(-1.0, min(1.0, (frobenius - 1.0) / 2.0)))


def _distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(first, second, strict=True)))


def _q54_group_svg(episode: PolicyEpisode) -> bytes:
    width, height, margin = 1000, 520, 56
    colors = ("#2563eb", "#16a34a", "#dc2626", "#9333ea")
    series: list[tuple[str, tuple[float, ...]]] = []
    for name, start, stop in _GROUPS:
        values = tuple(
            math.sqrt(
                sum(value * value for value in frame.observation_q54_rad[start:stop])
                / (stop - start)
            )
            for frame in episode.frames
        )
        series.append((name, values))
    all_values = tuple(value for _, values in series for value in values)
    minimum, maximum = min(all_values), max(all_values)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="56" y="30" font-family="sans-serif" font-size="20">q54 group RMS position over dataset time</text>',
        f'<line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" stroke="#111"/>',
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height - margin}" stroke="#111"/>',
    ]
    for index, ((name, values), color) in enumerate(zip(series, colors, strict=True)):
        lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2" points="'
            f'{_polyline(values, width=width, height=height, margin=margin, minimum=minimum, maximum=maximum)}"/>'
        )
        lines.append(
            f'<text x="{margin + index * 190}" y="{height - 14}" font-family="sans-serif" '
            f'font-size="14" fill="{color}">{escape(name)}</text>'
        )
    lines.append("</svg>")
    return ("\n".join(lines) + "\n").encode()


def _object_xy_svg(
    episode: PolicyEpisode,
    objects_by_control: dict[int, tuple[tuple[str, float, float], ...]],
) -> bytes:
    width, height, margin = 800, 620, 56
    by_id: dict[str, list[tuple[float, float]]] = {}
    for frame in episode.frames:
        for object_id, x, y in objects_by_control.get(frame.source_control_index, ()):
            by_id.setdefault(object_id, []).append((x, y))
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="56" y="30" font-family="sans-serif" font-size="20">dynamic object XY trajectory (pre-action)</text>',
    ]
    points = tuple(point for values in by_id.values() for point in values)
    if not points:
        lines.append(
            '<text x="56" y="90" font-family="sans-serif" font-size="16">No dynamic object truth rows.</text>'
        )
    else:
        min_x = min(item[0] for item in points)
        max_x = max(item[0] for item in points)
        min_y = min(item[1] for item in points)
        max_y = max(item[1] for item in points)
        span_x = max(max_x - min_x, 1e-9)
        span_y = max(max_y - min_y, 1e-9)
        colors = ("#ea580c", "#0891b2", "#4f46e5", "#65a30d")
        for index, (object_id, values) in enumerate(sorted(by_id.items())):
            projected = tuple(
                (
                    margin + (x - min_x) * (width - 2 * margin) / span_x,
                    height - margin - (y - min_y) * (height - 2 * margin) / span_y,
                )
                for x, y in values
            )
            color = colors[index % len(colors)]
            path = " ".join(f"{_fmt(x)},{_fmt(y)}" for x, y in projected)
            lines.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="3" points="{path}"/>'
            )
            lines.append(
                f'<text x="{margin + index * 170}" y="{height - 14}" '
                f'font-family="sans-serif" font-size="14" fill="{color}">{escape(object_id)}</text>'
            )
    lines.append("</svg>")
    return ("\n".join(lines) + "\n").encode()


def _vision_samples_html(episode: PolicyEpisode) -> bytes:
    indices = tuple(dict.fromkeys((0, len(episode.frames) // 2, len(episode.frames) - 1)))
    rows = [
        "<!doctype html>",
        '<meta charset="utf-8">',
        "<title>RGB source samples</title>",
        "<h1>Exact pre-action RGB source samples</h1>",
        '<table border="1"><thead><tr><th>frame</th>',
        *(f"<th>{escape(camera)}</th>" for camera in CAMERA_IDS),
        "</tr></thead><tbody>",
    ]
    for index in indices:
        rows.append(f"<tr><th>{index}</th>")
        for camera in CAMERA_IDS:
            record = episode.vision.frame(index, camera)
            relative = Path("..", "..", "vision", record.payload_path).as_posix()
            rows.append(
                f'<td><img width="320" height="240" src="{escape(relative)}" '
                f'alt="{escape(camera)} frame {index}"></td>'
            )
        rows.append("</tr>")
    rows.append("</tbody></table>")
    return ("\n".join(rows) + "\n").encode()


@dataclass(frozen=True, slots=True)
class QualityReportArtifact:
    root: Path
    report_sha256: str
    checksums_sha256: str


def build_quality_report(
    run_root: str | Path,
    q54_profile: Q54JointProfile,
) -> QualityReportArtifact:
    raw = Path(run_root)
    if raw.is_symlink():
        raise ValueError("quality run root must not be a symbolic link")
    root = raw.resolve()
    episode = load_policy_episode(root)
    if len(q54_profile.canonical_names) != 54 or len(set(q54_profile.canonical_names)) != 54:
        raise ValueError("q54 profile is invalid")
    normalized = load_normalized_episode_artifact(
        root / "derived" / "normalized",
        expected_run_id=episode.run_id,
    )
    if (
        normalized.facts.q54_profile_id != q54_profile.profile_id
        or normalized.facts.q54_profile_sha256 != q54_profile.file_sha256
        or normalized.facts.q54_runtime_names != q54_profile.canonical_names
    ):
        raise ValueError("quality q54 profile and normalized runtime inventory differ")
    release = load_release_decision_artifact(
        root / "derived" / "release",
        expected_run_id=episode.run_id,
    )
    if not release.decision.passed:
        raise ValueError("quality report requires a passing release decision")

    ticks_by_control = {
        tick.transition.control_index: tick for tick in normalized.facts.ticks
    }
    if len(ticks_by_control) != len(normalized.facts.ticks):
        raise ValueError("quality normalized ticks contain duplicate control indices")
    selected_ticks = []
    for frame in episode.frames:
        tick = ticks_by_control.get(frame.source_control_index)
        if tick is None:
            raise ValueError("quality policy frame does not resolve to a normalized tick")
        transition = tick.transition
        if (
            frame.source_tick_id != transition.tick_id
            or frame.source_state_digest != transition.pre_action_state_digest
            or not math.isclose(
                frame.simulation_time_s,
                transition.simulation_time_before_s,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            or any(
                not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
                for actual, expected in zip(
                    frame.observation_q54_rad,
                    transition.pre_feedback_q54_rad,
                    strict=True,
                )
            )
            or any(
                not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
                for actual, expected in zip(
                    frame.action_q54_rad,
                    transition.applied_target_q54_rad,
                    strict=True,
                )
            )
        ):
            raise ValueError("quality policy frame and normalized causal source differ")
        selected_ticks.append(tick)

    joint_rows: list[list[object]] = [
        [
            "global_index",
            "canonical_name",
            "side",
            "group",
            "observation_min_rad",
            "observation_max_rad",
            "observation_span_rad",
            "observation_mean_rad",
            "observation_std_rad",
            "action_min_rad",
            "action_max_rad",
            "action_span_rad",
            "max_abs_action_step_rad",
            "range_rad",
            "observation_range_coverage_fraction",
            "tracking_mean_abs_rad",
            "tracking_p95_abs_rad",
            "tracking_max_abs_rad",
            "tracking_rmse_rad",
            "tracking_rmse_range_fraction",
            "pre_action_qdot_rms_rad_s",
            "pre_action_qdot_max_abs_rad_s",
        ]
    ]
    group_spans: dict[str, float] = {}
    tracking_by_frame: dict[str, tuple[float, ...]] = {}
    group_rows: list[list[object]] = [
        [
            "group",
            "joint_count",
            "observation_max_joint_span_rad",
            "tracking_mean_abs_rad",
            "tracking_p95_abs_rad",
            "tracking_max_abs_rad",
            "tracking_rmse_rad",
            "pre_action_qdot_rms_rad_s",
            "pre_action_qdot_max_abs_rad_s",
        ]
    ]
    for joint in q54_profile.joints:
        observations = tuple(
            frame.observation_q54_rad[joint.global_index] for frame in episode.frames
        )
        actions = tuple(frame.action_q54_rad[joint.global_index] for frame in episode.frames)
        tracking_errors = tuple(
            tick.post_action_frame.q54_rad[joint.global_index] - action
            for tick, action in zip(selected_ticks, actions, strict=True)
        )
        absolute_tracking = tuple(abs(value) for value in tracking_errors)
        qdot = tuple(
            tick.pre_action_frame.qdot54_rad_s[joint.global_index] for tick in selected_ticks
        )
        obs_min, obs_max, obs_mean, obs_std = _statistics(observations)
        action_min, action_max, _, _ = _statistics(actions)
        joint_range = joint.upper_rad - joint.lower_rad
        max_step = max(
            (abs(second - first) for first, second in zip(actions, actions[1:])),
            default=0.0,
        )
        joint_rows.append(
            [
                joint.global_index,
                joint.canonical_name,
                joint.side,
                joint.group,
                _fmt(obs_min),
                _fmt(obs_max),
                _fmt(obs_max - obs_min),
                _fmt(obs_mean),
                _fmt(obs_std),
                _fmt(action_min),
                _fmt(action_max),
                _fmt(action_max - action_min),
                _fmt(max_step),
                _fmt(joint_range),
                _fmt((obs_max - obs_min) / joint_range),
                _fmt(sum(absolute_tracking) / len(absolute_tracking)),
                _fmt(_percentile(absolute_tracking, 0.95)),
                _fmt(max(absolute_tracking)),
                _fmt(_rms(tracking_errors)),
                _fmt(_rms(tracking_errors) / joint_range),
                _fmt(_rms(qdot)),
                _fmt(max(abs(value) for value in qdot)),
            ]
        )
    for name, start, stop in _GROUPS:
        group_spans[name] = max(
            (
                max(frame.observation_q54_rad[index] for frame in episode.frames)
                - min(frame.observation_q54_rad[index] for frame in episode.frames)
                for index in range(start, stop)
            ),
            default=0.0,
        )
        tracking = tuple(
            tick.post_action_frame.q54_rad[index] - frame.action_q54_rad[index]
            for tick, frame in zip(selected_ticks, episode.frames, strict=True)
            for index in range(start, stop)
        )
        qdot = tuple(
            tick.pre_action_frame.qdot54_rad_s[index]
            for tick in selected_ticks
            for index in range(start, stop)
        )
        tracking_by_frame[name] = tuple(
            _rms(
                tuple(
                    tick.post_action_frame.q54_rad[index]
                    - frame.action_q54_rad[index]
                    for index in range(start, stop)
                )
            )
            for tick, frame in zip(selected_ticks, episode.frames, strict=True)
        )
        absolute_tracking = tuple(abs(value) for value in tracking)
        group_rows.append(
            [
                name,
                stop - start,
                _fmt(group_spans[name]),
                _fmt(sum(absolute_tracking) / len(absolute_tracking)),
                _fmt(_percentile(absolute_tracking, 0.95)),
                _fmt(max(absolute_tracking)),
                _fmt(_rms(tracking)),
                _fmt(_rms(qdot)),
                _fmt(max(abs(value) for value in qdot)),
            ]
        )

    objects_by_control = {
        tick.transition.control_index: tuple(
            (
                body.logical_object_id,
                body.position_m[0],
                body.position_m[1],
            )
            for body in tick.pre_action_frame.rigid_bodies
            if body.valid
        )
        for tick in normalized.facts.ticks
    }
    object_positions: dict[str, list[tuple[float, float, float]]] = {}
    for tick in selected_ticks:
        for body in tick.pre_action_frame.rigid_bodies:
            if body.valid:
                object_positions.setdefault(body.logical_object_id, []).append(body.position_m)
    object_rows: list[list[object]] = [
        [
            "logical_object_id",
            "sample_count",
            "path_length_m",
            "initial_to_final_displacement_m",
            "minimum_z_m",
            "maximum_z_m",
            "z_span_m",
            "maximum_height_gain_m",
        ]
    ]
    object_metrics: dict[str, dict[str, float | int]] = {}
    for object_id, positions in sorted(object_positions.items()):
        path_length = sum(
            _distance(first, second)
            for first, second in zip(positions, positions[1:], strict=False)
        )
        displacement = _distance(positions[0], positions[-1])
        minimum_z = min(item[2] for item in positions)
        maximum_z = max(item[2] for item in positions)
        height_gain = maximum_z - positions[0][2]
        values: dict[str, float | int] = {
            "sample_count": len(positions),
            "path_length_m": float(_fmt(path_length)),
            "initial_to_final_displacement_m": float(_fmt(displacement)),
            "minimum_z_m": float(_fmt(minimum_z)),
            "maximum_z_m": float(_fmt(maximum_z)),
            "z_span_m": float(_fmt(maximum_z - minimum_z)),
            "maximum_height_gain_m": float(_fmt(height_gain)),
        }
        object_metrics[object_id] = values
        object_rows.append([object_id, *(_fmt(float(value)) for value in values.values())])

    camera_counts = {
        camera: sum(record.camera_id == camera for record in episode.vision.frames)
        for camera in CAMERA_IDS
    }
    camera_rows: list[list[object]] = [
        [
            "camera_id",
            "frame_count",
            "unique_payload_count",
            "horizontal_fov_deg",
            "vertical_fov_deg",
            "fx_px",
            "fy_px",
            "cx_px",
            "cy_px",
            "translation_path_length_m",
            "maximum_translation_from_first_m",
            "maximum_rotation_from_first_rad",
        ]
    ]
    camera_metrics: dict[str, dict[str, float | int]] = {}
    camera_motion_series: list[tuple[str, tuple[float, ...]]] = []
    inventories = {
        item.camera_id: item for item in episode.vision.camera_runtime_inventories
    }
    for camera in CAMERA_IDS:
        records = tuple(
            sorted(
                (item for item in episode.vision.frames if item.camera_id == camera),
                key=lambda item: item.dataset_frame_index,
            )
        )
        inventory = inventories[camera]
        matrices = tuple(item.world_from_camera_optical_row_major for item in records)
        camera_positions = tuple(_translation(item) for item in matrices)
        translation_path = sum(
            _distance(first, second)
            for first, second in zip(camera_positions, camera_positions[1:], strict=False)
        )
        translation_from_first = tuple(
            _distance(camera_positions[0], item) for item in camera_positions
        )
        rotation_from_first = tuple(_rotation_distance(matrices[0], item) for item in matrices)
        k = inventory.calibration.k_row_major
        metrics: dict[str, float | int] = {
            "frame_count": len(records),
            "unique_payload_count": len({item.payload_sha256 for item in records}),
            "horizontal_fov_deg": float(_fmt(inventory.calibration.horizontal_fov_deg)),
            "vertical_fov_deg": float(_fmt(inventory.calibration.vertical_fov_deg)),
            "fx_px": float(_fmt(k[0])),
            "fy_px": float(_fmt(k[4])),
            "cx_px": float(_fmt(k[2])),
            "cy_px": float(_fmt(k[5])),
            "translation_path_length_m": float(_fmt(translation_path)),
            "maximum_translation_from_first_m": float(_fmt(max(translation_from_first))),
            "maximum_rotation_from_first_rad": float(_fmt(max(rotation_from_first))),
        }
        camera_metrics[camera] = metrics
        camera_rows.append([camera, *(_fmt(float(value)) for value in metrics.values())])
        camera_motion_series.append((camera, translation_from_first))

    input_ages: dict[str, list[float]] = {}
    for tick in normalized.facts.ticks:
        for source, age_ms in tick.comparable_input_age_ms:
            input_ages.setdefault(source, []).append(age_ms)
    input_age_rows: list[list[object]] = [
        ["source", "sample_count", "minimum_ms", "p50_ms", "p95_ms", "maximum_ms"]
    ]
    input_age_metrics: dict[str, dict[str, float | int]] = {}
    for source, ages_ms in sorted(input_ages.items()):
        metrics = {
            "sample_count": len(ages_ms),
            "minimum_ms": float(_fmt(min(ages_ms))),
            "p50_ms": float(_fmt(_percentile(ages_ms, 0.5))),
            "p95_ms": float(_fmt(_percentile(ages_ms, 0.95))),
            "maximum_ms": float(_fmt(max(ages_ms))),
        }
        input_age_metrics[source] = metrics
        input_age_rows.append([source, *(_fmt(float(value)) for value in metrics.values())])

    raw_ticks = normalized.facts.ticks
    host_elapsed_s = (raw_ticks[-1].tick_time_ns - raw_ticks[0].tick_time_ns) / 1e9
    simulation_elapsed_s = (
        raw_ticks[-1].transition.simulation_time_after_s
        - raw_ticks[0].transition.simulation_time_before_s
    )
    observed_control_hz = (len(raw_ticks) - 1) / host_elapsed_s
    real_time_factor = simulation_elapsed_s / host_elapsed_s
    schedule_misses = sum(item.missed_control_periods_before_tick for item in raw_ticks)
    duration_s = episode.frames[-1].timestamp_s if episode.frames else 0.0
    report: dict[str, object] = {
        "schema": QUALITY_REPORT_SCHEMA,
        "run_id": episode.run_id,
        "frame_count": len(episode.frames),
        "duration_s": duration_s,
        "q54_profile_id": q54_profile.profile_id,
        "q54_profile_sha256": q54_profile.file_sha256,
        "alignment_digest_sha256": episode.alignment.digest_sha256,
        "vision_alignment_digest_sha256": episode.vision.alignment_digest_sha256,
        "vision_provenance": episode.vision.provenance.to_mapping(),
        "normalized_facts_sha256": normalized.facts_sha256,
        "release_decision_sha256": release.decision_sha256,
        "camera_frame_counts": camera_counts,
        "camera_metrics": camera_metrics,
        "input_age_metrics": input_age_metrics,
        "object_metrics": object_metrics,
        "control_timing": {
            "raw_tick_count": len(raw_ticks),
            "candidate_host_elapsed_s": float(_fmt(host_elapsed_s)),
            "candidate_simulation_elapsed_s": float(_fmt(simulation_elapsed_s)),
            "observed_control_hz": float(_fmt(observed_control_hz)),
            "real_time_factor": float(_fmt(real_time_factor)),
            "schedule_miss_count": schedule_misses,
        },
        "group_observation_max_span_rad": {
            key: float(_fmt(value)) for key, value in group_spans.items()
        },
        "dynamic_object_ids": sorted(
            {object_id for values in objects_by_control.values() for object_id, _, _ in values}
        ),
        "interpretation_scope": (
            "signal completeness and arm/hand/object trend only; no task outcome label"
        ),
    }
    destination = root / "derived" / "quality"
    derived = destination.parent
    if derived.is_symlink():
        raise ValueError("quality derived root must not be a symbolic link")
    temporary = Path(tempfile.mkdtemp(prefix=".quality-", dir=derived))
    try:
        (temporary / "plots").mkdir()
        report_path = temporary / "summary.json"
        report_path.write_text(
            json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        (temporary / "joint_metrics.csv").write_bytes(_csv_bytes(joint_rows))
        (temporary / "group_metrics.csv").write_bytes(_csv_bytes(group_rows))
        (temporary / "input_age_metrics.csv").write_bytes(_csv_bytes(input_age_rows))
        (temporary / "object_metrics.csv").write_bytes(_csv_bytes(object_rows))
        (temporary / "camera_metrics.csv").write_bytes(_csv_bytes(camera_rows))
        (temporary / "episode_metrics.csv").write_bytes(
            _csv_bytes(
                [
                    ["metric", "value"],
                    ["frame_count", len(episode.frames)],
                    ["duration_s", _fmt(duration_s)],
                    ["raw_tick_count", len(raw_ticks)],
                    ["observed_control_hz", _fmt(observed_control_hz)],
                    ["real_time_factor", _fmt(real_time_factor)],
                    ["schedule_miss_count", schedule_misses],
                    [
                        "fixture_translation_drift_m",
                        _fmt(normalized.facts.fixture_translation_drift_m),
                    ],
                    [
                        "fixture_rotation_drift_rad",
                        _fmt(normalized.facts.fixture_rotation_drift_rad),
                    ],
                    *([f"{name}_max_span_rad", _fmt(value)] for name, value in group_spans.items()),
                    *([f"{camera}_frame_count", count] for camera, count in camera_counts.items()),
                ]
            )
        )
        (temporary / "plots" / "q54_groups.svg").write_bytes(_q54_group_svg(episode))
        (temporary / "plots" / "tracking_error.svg").write_bytes(
            _series_svg(
                tuple((name, tracking_by_frame[name]) for name, _, _ in _GROUPS),
                title="post-feedback versus applied q54 group RMS error",
                y_label="rad",
            )
        )
        (temporary / "plots" / "input_age.svg").write_bytes(
            _series_svg(
                tuple((source, tuple(values)) for source, values in sorted(input_ages.items())),
                title="selected tracker/glove comparable input age",
                y_label="ms",
            )
        )
        (temporary / "plots" / "camera_motion.svg").write_bytes(
            _series_svg(
                camera_motion_series,
                title="camera optical-origin displacement from first policy frame",
                y_label="m",
            )
        )
        (temporary / "plots" / "object_xy.svg").write_bytes(
            _object_xy_svg(episode, objects_by_control)
        )
        (temporary / "plots" / "vision_samples.html").write_bytes(_vision_samples_html(episode))
        (temporary / "report.html").write_text(
            '<!doctype html>\n<meta charset="utf-8">\n'
            f"<title>{escape(episode.run_id)} quality</title>\n"
            f"<h1>{escape(episode.run_id)} post-hoc quality</h1>\n"
            "<p>Signal completeness and arm/hand/object trend only; no task outcome label.</p>\n"
            '<img src="plots/q54_groups.svg" width="1000">\n'
            '<img src="plots/tracking_error.svg" width="1000">\n'
            '<img src="plots/input_age.svg" width="1000">\n'
            '<img src="plots/camera_motion.svg" width="1000">\n'
            '<img src="plots/object_xy.svg" width="800">\n'
            '<p><a href="joint_metrics.csv">joint table</a> · '
            '<a href="group_metrics.csv">arm/hand group table</a> · '
            '<a href="camera_metrics.csv">camera table</a> · '
            '<a href="object_metrics.csv">object table</a></p>\n'
            '<p><a href="plots/vision_samples.html">RGB samples</a></p>\n',
            encoding="utf-8",
        )
        payloads = tuple(
            path
            for path in sorted(temporary.rglob("*"))
            if path.is_file() and path.name not in {"manifest.json", "checksums.sha256"}
        )
        checksums = {path.relative_to(temporary).as_posix(): _sha256(path) for path in payloads}
        manifest_path = temporary / "manifest.json"
        manifest_path.write_text(
            json.dumps(
                {
                    "schema": QUALITY_ARTIFACT_SCHEMA,
                    "run_id": episode.run_id,
                    "report_sha256": _sha256(report_path),
                    "file_checksums": checksums,
                },
                sort_keys=True,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        checksums["manifest.json"] = _sha256(manifest_path)
        checksum_path = temporary / "checksums.sha256"
        checksum_path.write_text(
            "".join(f"{digest}  {relative}\n" for relative, digest in sorted(checksums.items())),
            encoding="utf-8",
        )
        report_digest = _sha256(report_path)
        checksums_digest = _sha256(checksum_path)
        if destination.exists() or destination.is_symlink():
            existing_report = destination / "summary.json"
            existing_checksums = destination / "checksums.sha256"
            if (
                destination.is_dir()
                and not destination.is_symlink()
                and existing_report.is_file()
                and existing_checksums.is_file()
                and _sha256(existing_report) == report_digest
                and _sha256(existing_checksums) == checksums_digest
            ):
                shutil.rmtree(temporary)
                return QualityReportArtifact(
                    root=destination,
                    report_sha256=report_digest,
                    checksums_sha256=checksums_digest,
                )
            raise FileExistsError("a different quality artifact already exists")
        os.rename(temporary, destination)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return QualityReportArtifact(
        root=destination,
        report_sha256=report_digest,
        checksums_sha256=checksums_digest,
    )


__all__ = [
    "QUALITY_ARTIFACT_SCHEMA",
    "QUALITY_REPORT_SCHEMA",
    "QualityReportArtifact",
    "build_quality_report",
]
