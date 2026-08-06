#!/usr/bin/env python3
"""Build deterministic post-hoc tables and plots for one accepted episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.dataset import build_quality_report, load_mini_dataset_profile  # noqa: E402


DEFAULT_PROFILE = ROOT / "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--dataset-profile", type=Path, default=DEFAULT_PROFILE)
    args = parser.parse_args(argv)
    try:
        profile = load_mini_dataset_profile(ROOT, args.dataset_profile)
        artifact = build_quality_report(args.run_root, profile.q54)
    except (OSError, ValueError) as exc:
        print(f"quality report failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "output": str(artifact.root),
                "report_sha256": artifact.report_sha256,
                "checksums_sha256": artifact.checksums_sha256,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
