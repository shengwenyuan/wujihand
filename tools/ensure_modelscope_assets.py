#!/usr/bin/env python3
# ruff: noqa: E402  # Project imports follow the repository path bootstrap.
"""Restore one pinned ModelScope dataset into its project-relative source path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime.config_repository import ConfigRepository
from wujihand.runtime.modelscope_dataset import (
    ModelScopeDatasetPin,
    ensure_modelscope_dataset,
)
from wujihand.runtime.source_lock import SourceLock


DEFAULT_SOURCE = "modelscope-sss225-robolab-assets"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    parser.add_argument(
        "--seed-from",
        type=Path,
        help="Optional already-downloaded exact snapshot used before network download.",
    )
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument(
        "--network",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--force-full-verify", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repository = ConfigRepository(args.project_root)
    source_lock = SourceLock.load(repository)
    pin = ModelScopeDatasetPin.from_source_record(
        source_lock.record(args.source)
    )
    result = ensure_modelscope_dataset(
        repository.project_root,
        pin,
        seed_from=args.seed_from,
        allow_network=args.network,
        workers=args.workers,
        force_full_verify=args.force_full_verify,
    )
    print(json.dumps(result.to_mapping(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
