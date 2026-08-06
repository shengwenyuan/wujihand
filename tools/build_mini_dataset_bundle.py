#!/usr/bin/env python3
"""Close one gated raw/alignment/vision episode into an immutable bundle manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import cast


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "analysis/teleoperation_quality/src"))

from teleoperation_quality.artifact import load_run_artifact  # noqa: E402
from wujihand.dataset import (  # noqa: E402
    load_normalized_episode_artifact,
    load_policy_episode,
    load_release_decision_artifact,
    write_episode_bundle,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--collection-id", required=True)
    return parser


def _mapping(value: object, *, field: str) -> dict[str, object]:
    if not isinstance(value, dict) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(dict[str, object], value)


def _digest(value: object, *, field: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
        char not in "0123456789abcdef" for char in value
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256")
    return value


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        raw_root = args.run_root
        if raw_root.is_symlink():
            raise ValueError("run root must not be a symbolic link")
        run_root = raw_root.resolve()
        if not run_root.is_dir():
            raise ValueError("run root must be a directory")
        raw_artifact = load_run_artifact(run_root)
        manifest = _mapping(raw_artifact.manifest, field="raw manifest")
        deployment = _mapping(manifest.get("deployment"), field="raw deployment")
        dataset = _mapping(manifest.get("dataset"), field="raw dataset")
        if dataset.get("episode_id_rule") != "run_id_equals_episode_id":
            raise ValueError("raw dataset episode identity rule differs")
        profile_id = dataset.get("profile_id")
        if not isinstance(profile_id, str) or not profile_id:
            raise ValueError("raw dataset profile ID is invalid")
        release = load_release_decision_artifact(
            run_root / "derived" / "release",
            expected_run_id=run_root.name,
        )
        if not release.decision.passed:
            raise ValueError("failed release decision cannot produce a bundle")
        load_normalized_episode_artifact(
            run_root / "derived" / "normalized",
            expected_run_id=run_root.name,
        )
        policy = load_policy_episode(run_root)
        expected_vision_provenance = {
            "collection_id": args.collection_id,
            "dataset_profile_sha256": _digest(
                dataset.get("profile_sha256"), field="dataset profile hash"
            ),
            "deployment_sha256": _digest(
                deployment.get("deployment_hash"), field="deployment hash"
            ),
            "session_sha256": _digest(
                deployment.get("session_hash"), field="session hash"
            ),
            "assembly_sha256": _digest(
                deployment.get("assembly_sha256"), field="assembly hash"
            ),
            "workcell_sha256": _digest(
                deployment.get("workcell_sha256"), field="workcell hash"
            ),
        }
        observed_vision_provenance = {
            key: getattr(policy.vision.provenance, key)
            for key in expected_vision_provenance
        }
        if observed_vision_provenance != expected_vision_provenance:
            raise ValueError("vision provenance differs from raw/bundle identity")
        artifact = write_episode_bundle(
            run_root,
            collection_id=args.collection_id,
            dataset_profile_id=profile_id,
            dataset_profile_sha256=_digest(
                dataset.get("profile_sha256"), field="dataset profile hash"
            ),
            deployment_hash=_digest(
                deployment.get("deployment_hash"), field="deployment hash"
            ),
            session_hash=_digest(deployment.get("session_hash"), field="session hash"),
            assembly_hash=_digest(
                deployment.get("assembly_sha256"), field="assembly hash"
            ),
            workcell_hash=_digest(
                deployment.get("workcell_sha256"), field="workcell hash"
            ),
        )
    except (FileExistsError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "schema": "wujihand.dataset_bundle_cli_result.v1",
                    "passed": False,
                    "error": str(exc),
                },
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    print(
        json.dumps(
            {
                "schema": "wujihand.dataset_bundle_cli_result.v1",
                "passed": True,
                "run_id": run_root.name,
                "output": str(artifact.path),
                "manifest_sha256": artifact.manifest_sha256,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
