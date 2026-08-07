#!/usr/bin/env python3
"""Render a deterministic source-episode × visual-domain-variant matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VARIANTS = (
    "nominal",
    "dr_warm_bright",
    "dr_cool_dim",
    "dr_neutral_highkey",
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, action="append", required=True)
    parser.add_argument("--variant", action="append", dest="variants")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    variants = tuple(args.variants or DEFAULT_VARIANTS)
    if len(set(variants)) != len(variants):
        raise SystemExit("render matrix variant IDs are duplicated")
    for raw_root in args.run_root:
        run_root = raw_root.resolve()
        if run_root.is_symlink() or not run_root.is_dir():
            raise SystemExit(f"render matrix run root is missing or unsafe: {run_root}")
        logs = run_root / "derived" / "render_logs"
        logs.mkdir(exist_ok=True)
        if logs.is_symlink():
            raise SystemExit(f"render matrix log root is unsafe: {logs}")
        for variant in variants:
            artifact_name = "vision" if variant == "nominal" else f"vision_{variant}"
            destination = run_root / "derived" / artifact_name
            if destination.is_dir() and not destination.is_symlink():
                print(
                    json.dumps(
                        {
                            "event": "skip_existing",
                            "run_id": run_root.name,
                            "variant": variant,
                            "artifact": str(destination),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                continue
            log_path = logs / f"{artifact_name}.log"
            command = (
                sys.executable,
                str(ROOT / "tools/render_mini_dataset_episode.py"),
                "--run-root",
                str(run_root),
                "--render-variant",
                variant,
            )
            print(
                json.dumps(
                    {
                        "event": "start",
                        "run_id": run_root.name,
                        "variant": variant,
                        "log": str(log_path),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            with log_path.open("wb") as log:
                completed = subprocess.run(
                    command,
                    cwd=ROOT,
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            if completed.returncode != 0:
                print(
                    json.dumps(
                        {
                            "event": "failed",
                            "run_id": run_root.name,
                            "variant": variant,
                            "returncode": completed.returncode,
                            "log": str(log_path),
                        },
                        sort_keys=True,
                    ),
                    file=sys.stderr,
                    flush=True,
                )
                return completed.returncode
            print(
                json.dumps(
                    {
                        "event": "complete",
                        "run_id": run_root.name,
                        "variant": variant,
                        "artifact": str(destination),
                        "log": str(log_path),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
