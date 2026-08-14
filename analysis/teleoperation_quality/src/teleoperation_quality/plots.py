"""Deterministic static figures generated from persisted metric tables."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from itertools import pairwise
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure
from matplotlib.patches import Patch

from .metrics import MetricBundle

COLORS = ("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9")
PNG_METADATA = {"Software": "wujihand-teleoperation-quality"}


def _save(figure: Figure, path: Path) -> None:
    figure.savefig(path, dpi=160, bbox_inches="tight", metadata=PNG_METADATA)
    plt.close(figure)


def _finite(values: Sequence[Any]) -> np.ndarray[Any, np.dtype[np.float64]]:
    result = np.asarray(
        [float(value) for value in values if value is not None and np.isfinite(float(value))],
        dtype=np.float64,
    )
    return result


def _plot_ecdf(axis: Axes, values: Sequence[Any], *, label: str, color: str) -> None:
    finite = np.sort(_finite(values))
    if not finite.size:
        return
    probability = np.arange(1, finite.size + 1, dtype=np.float64) / finite.size
    axis.step(finite, probability, where="post", label=label, color=color, linewidth=1.6)


def _downsample_columns(
    matrix: np.ndarray[Any, np.dtype[np.float64]],
    times: np.ndarray[Any, np.dtype[np.float64]],
    *,
    maximum_columns: int = 1600,
    reducer: Callable[..., np.ndarray[Any, np.dtype[np.float64]]] = np.nanmax,
) -> tuple[np.ndarray[Any, np.dtype[np.float64]], np.ndarray[Any, np.dtype[np.float64]]]:
    if matrix.shape[1] <= maximum_columns:
        return matrix, times
    boundaries = np.linspace(0, matrix.shape[1], maximum_columns + 1, dtype=np.int64)
    reduced = np.column_stack(
        [
            reducer(matrix[:, boundaries[i] : boundaries[i + 1]], axis=1)
            for i in range(maximum_columns)
        ]
    )
    reduced_times = np.asarray(
        [np.mean(times[boundaries[i] : boundaries[i + 1]]) for i in range(maximum_columns)],
        dtype=np.float64,
    )
    return reduced, reduced_times


def _rows(bundle: MetricBundle, name: str, *, derived: bool = False) -> list[dict[str, Any]]:
    source = bundle.derived_tables if derived else bundle.tables
    return list(source[name])


def _frequency_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    source_lookup = {
        (str(row["side"]), str(row["kind"])): row for row in _rows(bundle, "source_metrics")
    }
    route_lookup = {
        (str(row["side"]), str(row["chain"])): row for row in _rows(bundle, "route_metrics")
    }
    routes = (("left", "arm"), ("left", "hand"), ("right", "arm"), ("right", "hand"))
    labels = [f"{side[0].upper()} {chain}" for side, chain in routes]
    raw_values = [
        float(
            source_lookup[(side, "tracker" if chain == "arm" else "glove")]["effective_hz"] or 0.0
        )
        for side, chain in routes
    ]
    selected_values = [
        float(route_lookup[(side, chain)]["new_source_full_window_hz"] or 0.0)
        for side, chain in routes
    ]
    positions = np.arange(len(routes), dtype=np.float64)
    width = 0.34
    figure, axis = plt.subplots(figsize=(9.5, 5.0), constrained_layout=True)
    raw_bars = axis.bar(
        positions - width / 2,
        raw_values,
        width,
        color=COLORS[0],
        label="raw intrinsic rate",
    )
    selected_bars = axis.bar(
        positions + width / 2,
        selected_values,
        width,
        color=COLORS[1],
        label="trace-selected rate (full window)",
    )
    target = float(bundle.summary["config"]["expected_control_hz"])
    control = float(bundle.summary["control"]["effective_hz"] or 0.0)
    axis.axhline(
        target,
        color="#555555",
        linestyle="--",
        linewidth=1.1,
    )
    axis.axhline(control, color=COLORS[2], linestyle=":", linewidth=1.3)
    axis.text(
        0.99,
        target,
        f" planned {target:g} Hz ",
        transform=axis.get_yaxis_transform(),
        ha="right",
        va="bottom",
        fontsize=8,
    )
    axis.text(
        0.99,
        control,
        f" actual control {control:.1f} Hz ",
        transform=axis.get_yaxis_transform(),
        ha="right",
        va="top",
        fontsize=8,
    )
    axis.set_ylabel("Effective rate (Hz)")
    axis.set_xticks(positions, labels)
    axis.set_title("Raw input, control-selected input and control-loop rate")
    axis.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.13),
        ncol=2,
        frameon=False,
    )
    for bars, values in ((raw_bars, raw_values), (selected_bars, selected_values)):
        for bar, value in zip(bars, values, strict=True):
            axis.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 1.4,
                f"{value:.1f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )
    path = output / "01_source_and_control_frequency.png"
    _save(figure, path)
    return {
        "file": path.name,
        "title": "Raw, selected and control frequency",
        "data": "source_metrics.csv; route_metrics.csv; summary.json",
    }


def _tick_interval_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    tick_rows = _rows(bundle, "aligned_ticks", derived=True)
    times = sorted(
        {
            int(row["tick_id"]): float(row["time_s"])
            for row in tick_rows
            if row["chain"] == "arm" and row["side"] == "left"
        }.items()
    )
    intervals = [
        (second[1] - first[1]) * 1000.0 for first, second in pairwise(times) if second[1] > first[1]
    ]
    figure, axis = plt.subplots(figsize=(8.8, 4.8), constrained_layout=True)
    _plot_ecdf(axis, intervals, label="control tick interval", color=COLORS[0])
    target_hz = float(bundle.summary["config"]["expected_control_hz"])
    limit = float(bundle.summary["config"]["effective_p95_tick_interval_limit_ms"])
    axis.axvline(
        1000.0 / target_hz,
        color=COLORS[2],
        linestyle="--",
        label=f"{1000.0 / target_hz:.2f} ms target period",
    )
    axis.axvline(
        limit,
        color=COLORS[1],
        linestyle=":",
        label=f"{limit:g} ms planned P95 limit",
    )
    axis.set_xlabel("Tick interval (ms)")
    axis.set_ylabel("ECDF")
    axis.set_ylim(0.0, 1.01)
    axis.set_title("Control tick interval distribution")
    axis.legend(loc="lower right")
    path = output / "02_control_tick_interval_ecdf.png"
    _save(figure, path)
    return {
        "file": path.name,
        "title": "Control tick interval ECDF",
        "data": "derived/aligned_ticks.csv",
    }


def _source_age_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "aligned_ticks", derived=True)
    figure, axis = plt.subplots(figsize=(8.8, 5.0), constrained_layout=True)
    for index, (side, chain) in enumerate(
        (("left", "arm"), ("right", "arm"), ("left", "hand"), ("right", "hand"))
    ):
        values = [
            row["input_age_ms"] for row in rows if row["side"] == side and row["chain"] == chain
        ]
        _plot_ecdf(
            axis,
            values,
            label=f"{side} {chain}",
            color=COLORS[index],
        )
    limit = float(bundle.summary["config"]["p95_comparable_input_age_limit_ms"])
    axis.axvline(
        limit,
        color="#555555",
        linestyle="--",
        linewidth=1.1,
        label=f"{limit:g} ms planned P95 limit",
    )
    axis.set_xlabel("Comparable active input age (ms)")
    axis.set_ylabel("ECDF")
    axis.set_ylim(0.0, 1.01)
    axis.set_title("Active input freshness (source time, receive-time fallback)")
    axis.legend(loc="lower right")
    path = output / "03_active_source_age_ecdf.png"
    _save(figure, path)
    return {
        "file": path.name,
        "title": "Active source age ECDF",
        "data": "derived/aligned_ticks.csv",
    }


def _state_timeline_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "aligned_ticks", derived=True)
    routes = (("left", "arm"), ("left", "hand"), ("right", "arm"), ("right", "hand"))
    series: list[np.ndarray[Any, np.dtype[np.float64]]] = []
    times: np.ndarray[Any, np.dtype[np.float64]] | None = None
    for side, chain in routes:
        selected = sorted(
            (row for row in rows if row["side"] == side and row["chain"] == chain),
            key=lambda row: int(row["tick_id"]),
        )
        if times is None:
            times = np.asarray([float(row["time_s"]) for row in selected], dtype=np.float64)
        values = np.asarray(
            [
                0.0 if not row["active_source_present"] else 2.0 if row["actionable"] else 1.0
                for row in selected
            ],
            dtype=np.float64,
        )
        series.append(values)
    if times is None or not series:
        times = np.asarray([0.0])
        matrix = np.zeros((4, 1), dtype=np.float64)
    else:
        minimum = min(len(item) for item in series)
        if minimum == 0:
            times = np.asarray([0.0])
            matrix = np.zeros((len(routes), 1), dtype=np.float64)
        else:
            matrix = np.vstack([item[:minimum] for item in series])
            times = times[:minimum]
    matrix, times = _downsample_columns(
        matrix,
        times,
        maximum_columns=10_000,
        reducer=np.nanmin,
    )
    plot_width_inches = max(10.5, min(40.0, matrix.shape[1] / 150.0 + 2.5))
    figure, axis = plt.subplots(
        figsize=(plot_width_inches, 3.5),
        constrained_layout=True,
    )
    cmap = ListedColormap(("#BDBDBD", COLORS[1], COLORS[2]))
    extent_start_s = float(times[0])
    extent_end_s = float(times[-1])
    if extent_end_s <= extent_start_s:
        extent_end_s = extent_start_s + 1.0
    axis.imshow(
        matrix,
        aspect="auto",
        interpolation="nearest",
        origin="upper",
        extent=(extent_start_s, extent_end_s, len(routes) - 0.5, -0.5),
        cmap=cmap,
        vmin=-0.5,
        vmax=2.5,
    )
    axis.set_yticks(range(len(routes)), [f"{side} {chain}" for side, chain in routes])
    axis.set_xlabel("Time from first control tick (s)")
    axis.set_title("Route actionability timeline")
    axis.legend(
        handles=(
            Patch(color="#BDBDBD", label="no active source"),
            Patch(color=COLORS[1], label="degraded / hold"),
            Patch(color=COLORS[2], label="actionable tracking"),
        ),
        loc="upper center",
        bbox_to_anchor=(0.5, -0.22),
        ncol=3,
        frameon=False,
    )
    path = output / "04_route_state_timeline.png"
    _save(figure, path)
    return {"file": path.name, "title": "Route state timeline", "data": "derived/aligned_ticks.csv"}


def _heatmap_plot(
    bundle: MetricBundle,
    output: Path,
    *,
    chain: str,
    field_prefix: str,
    joints: int,
    absolute: bool,
    filename: str,
    title: str,
    colorbar_label: str,
) -> dict[str, str]:
    rows = _rows(bundle, "aligned_ticks", derived=True)
    figure, axes = plt.subplots(2, 1, figsize=(10.5, 6.2), sharex=True, constrained_layout=True)
    matrices: list[np.ndarray[Any, np.dtype[np.float64]]] = []
    times_per_side: list[np.ndarray[Any, np.dtype[np.float64]]] = []
    for side in ("left", "right"):
        selected = sorted(
            (row for row in rows if row["side"] == side and row["chain"] == chain),
            key=lambda row: int(row["tick_id"]),
        )
        matrix = np.asarray(
            [
                [float(row[f"{field_prefix}_j{index}"]) for row in selected]
                for index in range(joints)
            ],
            dtype=np.float64,
        )
        if absolute:
            matrix = np.abs(matrix)
        times = np.asarray([float(row["time_s"]) for row in selected], dtype=np.float64)
        reducer = np.nanmax if absolute else np.nanmean
        matrix, times = _downsample_columns(matrix, times, reducer=reducer)
        matrices.append(matrix)
        times_per_side.append(times)
    if absolute:
        finite_max = max((float(np.nanmax(item)) for item in matrices if item.size), default=1e-12)
        vmin, vmax, cmap = 0.0, max(finite_max, 1e-12), "magma"
    else:
        finite_max = max(
            (float(np.nanmax(np.abs(item))) for item in matrices if item.size), default=1e-12
        )
        vmin, vmax, cmap = -max(finite_max, 1e-12), max(finite_max, 1e-12), "coolwarm"
    image = None
    for axis, side, matrix, times in zip(
        axes, ("left", "right"), matrices, times_per_side, strict=True
    ):
        if not matrix.size or not times.size:
            continue
        image = axis.imshow(
            matrix,
            aspect="auto",
            interpolation="nearest",
            origin="lower",
            extent=(float(times[0]), float(times[-1]), -0.5, joints - 0.5),
            vmin=vmin,
            vmax=vmax,
            cmap=cmap,
        )
        axis.set_ylabel(f"{side} joint")
        axis.set_yticks(range(joints))
    axes[-1].set_xlabel("Time from first control tick (s)")
    figure.suptitle(title)
    if image is not None:
        figure.colorbar(image, ax=axes, label=colorbar_label, shrink=0.88)
    path = output / filename
    _save(figure, path)
    return {"file": path.name, "title": title, "data": "derived/aligned_ticks.csv"}


def _stage_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "stage_samples", derived=True)
    figure, axis = plt.subplots(figsize=(8.8, 5.0), constrained_layout=True)
    fields = (
        (
            ("snapshot_ms", "input snapshot", COLORS[0]),
            ("control_ms", "control", COLORS[1]),
            ("apply_ms", "apply", COLORS[2]),
            ("physics_ms", "four physics substeps", COLORS[3]),
            ("pipeline_ms", "full pipeline", COLORS[4]),
        )
        if rows and "snapshot_ms" in rows[0]
        else (
            ("spin_ms", "ROS spin", COLORS[0]),
            ("control_ms", "control", COLORS[1]),
            ("apply_ms", "apply", COLORS[2]),
            ("world_step_ms", "world step", COLORS[3]),
            ("pipeline_ms", "full pipeline", COLORS[4]),
        )
    )
    for field, label, color in fields:
        _plot_ecdf(axis, [row.get(field) for row in rows], label=label, color=color)
    axis.set_xlabel("Duration (ms)")
    axis.set_ylabel("ECDF")
    axis.set_ylim(0.0, 1.01)
    axis.set_title("Per-tick stage duration distributions")
    axis.legend(loc="lower right")
    path = output / "08_stage_duration_ecdf.png"
    _save(figure, path)
    return {"file": path.name, "title": "Stage duration ECDF", "data": "derived/stage_samples.csv"}


def _execution_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "execution_samples", derived=True)
    figure, axes = plt.subplots(1, 2, figsize=(11.0, 4.7), constrained_layout=True)
    lateness = [row["control_lateness_ms"] for row in rows]
    substep_host = [
        value
        for row in rows
        for field, value in row.items()
        if field.startswith("physics_substep_") and field.endswith("_host_ms")
    ]
    _plot_ecdf(axes[0], lateness, label="control deadline lateness", color=COLORS[0])
    _plot_ecdf(axes[0], substep_host, label="physics substep host time", color=COLORS[1])
    axes[0].set_xlabel("Duration (ms)")
    axes[0].set_ylabel("ECDF")
    axes[0].set_ylim(0.0, 1.01)
    axes[0].set_title("Scheduler and physics execution")
    axes[0].legend(loc="lower right")

    tick_ids = [int(row["tick_id"]) for row in rows]
    simulation_advance_ms = [float(row["simulation_advance_s"]) * 1000.0 for row in rows]
    axes[1].plot(
        tick_ids,
        simulation_advance_ms,
        color=COLORS[2],
        linewidth=1.2,
        label="simulation advance",
    )
    expected_control_ms = 1000.0 / float(bundle.summary["config"]["expected_control_hz"])
    axes[1].axhline(
        expected_control_ms,
        color="#555555",
        linestyle="--",
        label="target control period",
    )
    render_ticks = [int(row["tick_id"]) for row in rows if bool(row["rendered"])]
    if render_ticks:
        axes[1].scatter(
            render_ticks,
            [expected_control_ms] * len(render_ticks),
            color=COLORS[4],
            marker="|",
            s=80,
            label="rendered tick",
        )
    axes[1].set_xlabel("Control tick")
    axes[1].set_ylabel("Simulation advance (ms)")
    axes[1].set_title("Four-substep and render cadence")
    axes[1].legend(loc="best")
    path = output / "13_scheduler_physics_render.png"
    _save(figure, path)
    return {
        "file": path.name,
        "title": "Scheduler, physics and render cadence",
        "data": "derived/execution_samples.csv",
    }


def _glove_confidence_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "source_samples", derived=True)
    figure, axes = plt.subplots(2, 1, figsize=(10.2, 5.8), sharex=False, constrained_layout=True)
    for axis, side, color in zip(axes, ("left", "right"), COLORS[:2], strict=True):
        selected = [row for row in rows if row["kind"] == "glove" and row["side"] == side]
        times = np.asarray([float(row["time_s"]) for row in selected], dtype=np.float64)
        minimum = np.asarray(
            [
                np.nan if row["minimum_confidence"] is None else float(row["minimum_confidence"])
                for row in selected
            ],
            dtype=np.float64,
        )
        median = np.asarray(
            [
                np.nan if row["median_confidence"] is None else float(row["median_confidence"])
                for row in selected
            ],
            dtype=np.float64,
        )
        if times.size:
            axis.plot(times, median, color=color, linewidth=1.0, alpha=0.8, label="median landmark")
            axis.plot(
                times, minimum, color="#333333", linewidth=0.7, alpha=0.8, label="minimum landmark"
            )
        source_row = next(
            row
            for row in bundle.tables["source_metrics"]
            if row["kind"] == "glove" and row["side"] == side
        )
        reference = float(source_row["success_confidence_reference"])
        axis.axhline(
            reference,
            color=COLORS[4],
            linestyle="--",
            linewidth=1.0,
            label="retarget status reference",
        )
        axis.set_ylim(0.0, 1.02)
        axis.set_ylabel(f"{side} confidence")
        axis.legend(loc="lower right", ncol=3, fontsize=8)
    axes[-1].set_xlabel("Time from first glove sample (s)")
    figure.suptitle("Raw glove landmark confidence (no admission rejection)")
    path = output / "09_glove_confidence.png"
    _save(figure, path)
    return {"file": path.name, "title": "Glove confidence", "data": "derived/source_samples.csv"}


def _scene_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "scene_samples", derived=True)
    paths = sorted({str(row["prim_path"]) for row in rows})
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.7), constrained_layout=True)
    for index, prim_path in enumerate(paths):
        selected = [row for row in rows if row["prim_path"] == prim_path]
        color = COLORS[index % len(COLORS)]
        label = prim_path.rsplit("/", maxsplit=1)[-1]
        axes[0].plot(
            [row["x_m"] for row in selected],
            [row["y_m"] for row in selected],
            color=color,
            linewidth=1.2,
            label=label,
        )
        for coordinate, style in (("x_m", "-"), ("y_m", "--"), ("z_m", ":")):
            axes[1].plot(
                [row["time_s"] for row in selected],
                [row[coordinate] for row in selected],
                color=color,
                linestyle=style,
                linewidth=1.0,
                label=f"{label} {coordinate[0]}",
            )
    axes[0].set_xlabel("World x (m)")
    axes[0].set_ylabel("World y (m)")
    axes[0].set_title("XY trajectory")
    axes[0].axis("equal")
    axes[1].set_xlabel("Time from first scene sample (s)")
    axes[1].set_ylabel("World position (m)")
    axes[1].set_title("Position components")
    if paths:
        axes[0].legend(loc="best")
        axes[1].legend(loc="best", fontsize=8)
    else:
        for axis in axes:
            axis.text(
                0.5,
                0.5,
                "No dynamic rigid-body samples",
                ha="center",
                va="center",
                transform=axis.transAxes,
            )
    figure.suptitle("Recorded dynamic scene trajectory")
    path = output / "10_scene_trajectory.png"
    _save(figure, path)
    return {
        "file": path.name,
        "title": "Dynamic scene trajectory",
        "data": "derived/scene_samples.csv",
    }


def _ik_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "ik_metrics")
    figure, axes = plt.subplots(1, 2, figsize=(9.6, 4.2), constrained_layout=True)
    sides = [str(row["side"]) for row in rows]
    positions = np.arange(len(rows), dtype=np.float64)
    width = 0.35
    for axis, prefix, scale, label in (
        (axes[0], "position_residual_m", 1000.0, "Position residual (mm)"),
        (axes[1], "orientation_residual_rad", 180.0 / np.pi, "Orientation residual (deg)"),
    ):
        p50 = [float(row[f"{prefix}_p50"] or 0.0) * scale for row in rows]
        p95 = [float(row[f"{prefix}_p95"] or 0.0) * scale for row in rows]
        axis.bar(positions - width / 2, p50, width, label="p50", color=COLORS[0])
        axis.bar(positions + width / 2, p95, width, label="p95", color=COLORS[1])
        axis.set_xticks(positions, sides)
        axis.set_ylabel(label)
        axis.legend()
    figure.suptitle("IK residual distributions for attempted solves")
    path = output / "11_ik_residual_summary.png"
    _save(figure, path)
    return {"file": path.name, "title": "IK residual summary", "data": "ik_metrics.csv"}


def _skew_plot(bundle: MetricBundle, output: Path) -> dict[str, str]:
    rows = _rows(bundle, "source_skew_metrics")
    pair_order = (
        "tracker_left_right",
        "glove_left_right",
        "left_tracker_glove",
        "right_tracker_glove",
    )
    pair_labels = {
        "tracker_left_right": "Tracker\nL–R",
        "glove_left_right": "Glove\nL–R",
        "left_tracker_glove": "Left\ntracker–glove",
        "right_tracker_glove": "Right\ntracker–glove",
    }
    bases = (
        ("source_time_ns", "Source time"),
        ("receive_time_ns", "Receive time"),
        ("callback_time_ns", "Callback time"),
    )
    lookup = {(str(row["pair"]), str(row["basis"])): row for row in rows}
    positions = np.arange(len(pair_order), dtype=np.float64)
    figure, axes = plt.subplots(
        1,
        len(bases),
        figsize=(13.2, 4.8),
        sharey=True,
        constrained_layout=True,
    )
    for axis, (basis, title) in zip(axes, bases, strict=True):
        selected = [lookup[(pair, basis)] for pair in pair_order]
        p50 = [
            np.nan if row["skew_ms_p50"] is None else float(row["skew_ms_p50"]) for row in selected
        ]
        p95 = [
            np.nan if row["skew_ms_p95"] is None else float(row["skew_ms_p95"]) for row in selected
        ]
        axis.bar(positions - 0.18, p50, 0.36, color=COLORS[0], label="P50")
        axis.bar(positions + 0.18, p95, 0.36, color=COLORS[1], label="P95")
        for position, row in zip(positions, selected, strict=True):
            if row["skew_ms_p95"] is None:
                axis.text(
                    position,
                    0.02,
                    "NA",
                    transform=axis.get_xaxis_transform(),
                    ha="center",
                    va="bottom",
                    fontsize=8,
                )
        axis.set_xticks(
            positions,
            [pair_labels[pair] for pair in pair_order],
            rotation=18,
            ha="right",
        )
        axis.set_title(title)
    axes[0].set_ylabel("Absolute skew (ms)")
    axes[0].legend(loc="upper right", frameon=False)
    figure.suptitle("Cross-stream active-source skew by timestamp basis")
    path = output / "12_cross_stream_skew.png"
    _save(figure, path)
    return {
        "file": path.name,
        "title": "Cross-stream skew",
        "data": "source_skew_metrics.csv",
    }


def write_plots(bundle: MetricBundle, output_root: str | Path) -> tuple[dict[str, str], ...]:
    output = Path(output_root)
    output.mkdir(parents=True, exist_ok=False)
    figures = [
        _frequency_plot(bundle, output),
        _tick_interval_plot(bundle, output),
        _source_age_plot(bundle, output),
        _state_timeline_plot(bundle, output),
        _heatmap_plot(
            bundle,
            output,
            chain="arm",
            field_prefix="error",
            joints=7,
            absolute=True,
            filename="05_arm_joint_error_heatmap.png",
            title="Arm command to post-step feedback absolute error",
            colorbar_label="Absolute error (rad)",
        ),
        _heatmap_plot(
            bundle,
            output,
            chain="hand",
            field_prefix="error",
            joints=20,
            absolute=True,
            filename="06_hand_joint_error_heatmap.png",
            title="Hand command to post-step feedback absolute error",
            colorbar_label="Absolute error (rad)",
        ),
        _heatmap_plot(
            bundle,
            output,
            chain="hand",
            field_prefix="command",
            joints=20,
            absolute=False,
            filename="07_hand_command_heatmap.png",
            title="Applied Hand2 q20 command trajectories",
            colorbar_label="Command (rad)",
        ),
        _stage_plot(bundle, output),
        _glove_confidence_plot(bundle, output),
        _scene_plot(bundle, output),
        _ik_plot(bundle, output),
        _skew_plot(bundle, output),
    ]
    if bundle.derived_tables["execution_samples"]:
        figures.append(_execution_plot(bundle, output))
    return tuple(figures)


__all__ = ["write_plots"]
