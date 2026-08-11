#!/usr/bin/env python3
# ruff: noqa: E402  # Project source is added before importing wujihand.
"""Preflight one Wuji SDK 8.3 + Description 8.3 Hand2 qualification chain."""

from __future__ import annotations

import argparse
from importlib import metadata
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.domain import HandSide
from wujihand.runtime import (
    detect_wuji_studio_processes,
    inspect_wuji_sdk_runtime,
    preflight_wuji_hand2_matched_chain,
)


DEFAULT_QUALIFICATION = (
    ROOT / "configs/qualifications/wuji_hand2_matched_chain_v2026_8_3_v1.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--input", choices=("stub", "glove"), default="stub")
    parser.add_argument("--local-binding", type=Path, required=True)
    parser.add_argument("--qualification", type=Path, default=DEFAULT_QUALIFICATION)
    parser.add_argument(
        "--output",
        type=Path,
        help="Exclusive immutable JSON receipt path; stdout is always emitted.",
    )
    parser.add_argument(
        "--skip-artifact-hash",
        action="store_true",
        help="Development-only: resolve Description identity without hashing restored artifacts.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    import wuji_sdk

    facts = inspect_wuji_sdk_runtime(
        wuji_sdk,
        distribution_version=metadata.version("wuji-sdk"),
    )
    receipt = preflight_wuji_hand2_matched_chain(
        ROOT,
        qualification_path=args.qualification,
        local_binding_path=args.local_binding,
        side=HandSide(args.side),
        input_mode=args.input,
        sdk_runtime=facts,
        user_manager=wuji_sdk.SdkManager.instance(),
        studio_processes=detect_wuji_studio_processes(),
        verify_artifacts=not args.skip_artifact_hash,
    )
    encoded = json.dumps(receipt.to_mapping(), ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
