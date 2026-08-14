#!/usr/bin/env python3
# ruff: noqa: E402  # Project source is added before importing wujihand.
"""Preflight dual NERO + dual Hand2 8.3 + Glove/Tracker + record."""

from __future__ import annotations

import argparse
from importlib import metadata
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.runtime.wuji_hand2_matched_chain import (
    detect_wuji_studio_processes,
    inspect_wuji_sdk_runtime,
)
from wujihand.runtime.wuji_hand2_record_chain import (
    preflight_wuji_hand2_record_chain,
)


DEFAULT_QUALIFICATION = (
    ROOT
    / "configs/qualifications/"
    "isaac_nero_hand2_tframe_gripper_flange_collision_proxy_"
    "self_collision_record_chain_v1.yaml"
)
DEFAULT_DEPLOYMENT = (
    ROOT
    / "configs/deployments/"
    "isaac_nero_hand2_ros_dual_tframe_gripper_flange_collision_proxy_"
    "triview_q54_self_collision_v1.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--qualification", type=Path, default=DEFAULT_QUALIFICATION)
    parser.add_argument("--deployment", type=Path, default=DEFAULT_DEPLOYMENT)
    parser.add_argument("--local-runtime-binding", type=Path, required=True)
    parser.add_argument("--matched-chain-binding", type=Path, required=True)
    parser.add_argument("--input", choices=("stub", "glove"), default="stub")
    parser.add_argument(
        "--dataset-source-mode",
        choices=("synthetic_fixture", "live_qualification", "live_teleoperation"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Exclusive immutable JSON receipt path; stdout is always emitted.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    import wuji_sdk

    facts = inspect_wuji_sdk_runtime(
        wuji_sdk,
        distribution_version=metadata.version("wuji-sdk"),
    )
    receipt = preflight_wuji_hand2_record_chain(
        ROOT,
        qualification_path=args.qualification,
        deployment_path=args.deployment,
        local_runtime_binding_path=args.local_runtime_binding,
        matched_chain_binding_path=args.matched_chain_binding,
        input_mode=args.input,
        dataset_source_mode=args.dataset_source_mode,
        sdk_runtime=facts,
        user_manager=wuji_sdk.SdkManager.instance(),
        studio_processes=detect_wuji_studio_processes(),
    )
    encoded = (
        json.dumps(
            receipt.to_mapping(),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(encoded)
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
