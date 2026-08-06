#!/usr/bin/env python3
"""Build one immutable exact 30 Hz alignment artifact from normalized 60 Hz rows."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.dataset import (  # noqa: E402
    RawTransition,
    build_exact_30hz_alignment,
    load_normalized_episode_artifact,
    load_release_decision_artifact,
    write_alignment_artifact,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument(
        "--transitions-jsonl",
        type=Path,
        help="Diagnostic fixture input; production defaults to the gated normalized artifact.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        run_root = args.run_root.resolve()
        if args.transitions_jsonl is None:
            release = load_release_decision_artifact(
                run_root / "derived" / "release",
                expected_run_id=run_root.name,
            )
            if not release.decision.passed:
                raise ValueError("alignment requires a passing release decision")
            normalized = load_normalized_episode_artifact(
                run_root / "derived" / "normalized",
                expected_run_id=run_root.name,
            )
            rows = [item.transition for item in normalized.facts.ticks]
            missing_periods = {
                item.transition.control_index: item.missed_control_periods_before_tick
                for item in normalized.facts.ticks
                if item.missed_control_periods_before_tick > 0
            }
        else:
            rows = []
            missing_periods = {}
            for line_number, line in enumerate(
                args.transitions_jsonl.read_text(encoding="utf-8").splitlines(),
                start=1,
            ):
                if not line:
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid JSON at line {line_number}") from exc
                rows.append(RawTransition.from_mapping(value, field=f"line[{line_number}]"))
        alignment = build_exact_30hz_alignment(
            rows,
            missed_control_periods_before_tick=missing_periods,
        )
        output = write_alignment_artifact(args.run_root, alignment)
    except (OSError, ValueError) as exc:
        print(f"alignment failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "run_id": alignment.run_id,
                "source_transition_count": alignment.source_transition_count,
                "frame_count": len(alignment.frames),
                "alignment_digest_sha256": alignment.digest_sha256,
                "output": str(output),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
