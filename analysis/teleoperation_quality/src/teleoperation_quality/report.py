"""Render a self-contained static report from already-computed metric tables."""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Any

from .metrics import MetricBundle


def _display(value: Any) -> str:
    if value is None:
        return "NA"
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def _table(rows: tuple[dict[str, Any], ...], columns: tuple[tuple[str, str], ...]) -> str:
    header = "".join(f"<th>{escape(label)}</th>" for _, label in columns)
    body = []
    for row in rows:
        cells = "".join(f"<td>{escape(_display(row.get(key)))}</td>" for key, _ in columns)
        body.append(f"<tr>{cells}</tr>")
    if not body:
        body.append(f'<tr><td colspan="{len(columns)}">No available rows</td></tr>')
    return f"<table><thead><tr>{header}</tr></thead><tbody>{''.join(body)}</tbody></table>"


def write_report(
    bundle: MetricBundle,
    figures: tuple[dict[str, str], ...],
    output_path: str | Path,
) -> None:
    """Write presentation only; all numbers originate in ``MetricBundle``."""

    summary = bundle.summary
    gates = bundle.tables["gates"]
    structural = tuple(row for row in gates if row["category"] == "structural")
    planned = tuple(row for row in gates if row["category"] == "planned_target")
    unsupported = "".join(
        f"<li>{escape(str(item))}</li>" for item in summary["unsupported_quantitative_conclusions"]
    )
    images = "".join(
        (
            "<figure>"
            f'<img src="plots/{escape(figure["file"])}" '
            f'alt="{escape(figure["title"])}">'
            f"<figcaption>{escape(figure['title'])} — "
            f"{escape(figure['data'])}</figcaption>"
            "</figure>"
        )
        for figure in figures
    )
    route_columns = (
        ("side", "Side"),
        ("chain", "Chain"),
        ("new_source_full_window_hz", "Trace-selected Hz (full window)"),
        ("actionable_coverage", "Actionable coverage"),
        ("input_age_ms_p95", "P95 comparable input age (ms)"),
        ("input_age_basis", "Input time basis"),
        ("command_feedback_rmse_rad", "Command-feedback RMSE (rad)"),
        ("position_clamped_ratio", "Clamp tick ratio"),
        ("rate_limited_ratio", "Rate-limit tick ratio"),
    )
    gate_columns = (
        ("name", "Gate"),
        ("expected", "Expected"),
        ("observed", "Observed"),
        ("passed", "Result"),
    )
    source_columns = (
        ("kind", "Kind"),
        ("side", "Side"),
        ("count", "Samples"),
        ("effective_hz", "Effective Hz"),
        ("interval_ms_p95", "P95 interval (ms)"),
        ("invalid_timestamp_intervals", "Invalid intervals"),
    )
    receipt_columns = (
        ("kind", "Kind"),
        ("side", "Side"),
        ("accepted", "Inbox accepted"),
        ("drained", "Inbox drained"),
        ("discarded", "Inbox discarded"),
        ("overwritten", "Inbox overwritten"),
        ("pending", "Inbox pending"),
        ("trace_selected_count", "Trace-selected"),
        ("accepted_selection_accounted", "Accounted"),
    )
    scene_columns = (
        ("prim_path", "Dynamic prim"),
        ("count", "Samples"),
        ("effective_hz", "Effective Hz"),
        ("path_length_m", "Path length (m)"),
        ("maximum_height_delta_m", "Maximum height delta (m)"),
    )
    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Teleoperation quality — {escape(str(summary["run_id"]))}</title>
<style>
:root {{ color-scheme: light dark; font-family: system-ui, sans-serif; }}
body {{ margin: 2rem auto; max-width: 1120px; padding: 0 1rem; line-height: 1.45; }}
table {{ border-collapse: collapse; display: block; margin: 0 0 1.5rem; overflow-x: auto; }}
th, td {{ border-bottom: 1px solid #8888; padding: .45rem .65rem; text-align: left; }}
th {{ position: sticky; top: 0; }}
.status {{ display: flex; flex-wrap: wrap; gap: 1rem; margin-bottom: 1.5rem; }}
.status span {{ border: 1px solid #8888; border-radius: .35rem; padding: .5rem .75rem; }}
figure {{ margin: 1.5rem 0 2rem; }}
img {{ height: auto; max-width: 100%; }}
figcaption {{ color: #777; font-size: .9rem; }}
code {{ overflow-wrap: anywhere; }}
</style>
</head>
<body>
<h1>Teleoperation quality report</h1>
<p><code>{escape(str(summary["run_id"]))}</code></p>
<div class="status">
<span>Structural gates: {_display(summary["structural_gates_passed"])}</span>
<span>Planned targets: {_display(summary["planned_targets_passed"])}</span>
<span>Control: {escape(_display(summary["control"]["effective_hz"]))} Hz</span>
<span>Tick P95: {escape(_display(summary["control"]["interval_ms"]["p95"]))} ms</span>
<span>Dominant stage: {escape(_display(summary["control"]["dominant_exclusive_stage_by_p95"]))}</span>
<span>Duration: {escape(_display(summary["bag"]["bag_duration_s"]))} s</span>
</div>
<p>Window: {escape(str(summary["analysis_window"]))}</p>
<h2>Structural data gates</h2>
{_table(structural, gate_columns)}
<h2>Planned performance references</h2>
<p>These are planning references, not frozen data-release thresholds.</p>
{_table(planned, gate_columns)}
<h2>Four-route scorecard</h2>
{_table(bundle.tables["route_metrics"], route_columns)}
<h2>Raw input streams</h2>
{_table(bundle.tables["source_metrics"], source_columns)}
<h2>Inbox-to-trace accounting</h2>
{_table(bundle.tables["receipt_input_metrics"], receipt_columns)}
<h2>Recorded dynamic scene objects</h2>
{_table(bundle.tables["scene_metrics"], scene_columns)}
<h2>Unsupported conclusions</h2>
<ul>{unsupported}</ul>
<h2>Figures</h2>
{images}
</body>
</html>
"""
    Path(output_path).write_text(document, encoding="utf-8")


__all__ = ["write_report"]
