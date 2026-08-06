#!/usr/bin/env python3
"""Register, reject, restore or accept one mini-dataset episode safely."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.dataset import (  # noqa: E402
    CollectionRegistry,
    DatasetEpisodeAnnotation,
    EpisodeDisposition,
    load_mini_dataset_profile,
    load_policy_episode,
    load_release_decision_artifact,
    validate_episode_bundle,
    write_episode_annotation,
)


DEFAULT_DATASET_PROFILE = (
    Path("configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml")
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument("--collection-root", type=Path, required=True)
    parser.add_argument("--collection-id", required=True)
    parser.add_argument("--dataset-profile", type=Path, default=DEFAULT_DATASET_PROFILE)
    subparsers = parser.add_subparsers(dest="command", required=True)

    register = subparsers.add_parser("register")
    register.add_argument("--episode-id", required=True)
    register.add_argument("--run-root", type=Path, required=True)
    register.add_argument("--incomplete", action="store_true")
    register.add_argument("--reason", default="registered")

    reject = subparsers.add_parser("reject")
    reject.add_argument("--episode-id", required=True)
    reject.add_argument("--reason", required=True)

    restore = subparsers.add_parser("restore")
    restore.add_argument("--episode-id", required=True)
    restore.add_argument("--reason", default="restored_for_regate")

    accept = subparsers.add_parser("accept")
    accept.add_argument("--episode-id", required=True)
    accept.add_argument("--release-artifact-root", type=Path, required=True)
    accept.add_argument("--reason", default="release_gates_passed")

    annotate = subparsers.add_parser("annotate")
    annotate.add_argument("--episode-id", required=True)
    annotate.add_argument("--task", required=True)
    annotate.add_argument("--operator-note", default="")

    purge = subparsers.add_parser("purge")
    purge.add_argument("--episode-id", required=True)
    purge.add_argument("--confirm-episode-id", required=True)
    purge.add_argument("--reason", required=True)

    subparsers.add_parser("list")
    subparsers.add_parser("list-exports")
    return parser


def _write_disposition_marker(
    run_root: Path,
    *,
    episode_id: str,
    disposition: str,
    reason: str,
    stale_export_revisions: tuple[str, ...] = (),
    release_decision_sha256: str | None = None,
    bundle_manifest_sha256: str | None = None,
) -> Path:
    if run_root.is_symlink() or not run_root.is_dir() or run_root.name != episode_id:
        raise ValueError("disposition marker run root is unsafe or has the wrong episode ID")
    derived = run_root / "derived"
    derived.mkdir(exist_ok=True)
    if derived.is_symlink():
        raise ValueError("disposition marker derived root must not be a symbolic link")
    destination = derived / "collection_disposition.json"
    payload = (
        json.dumps(
            {
                "schema": "wujihand.dataset_collection_disposition.v1",
                "episode_id": episode_id,
                "disposition": disposition,
                "reason": reason,
                "stale_export_revisions": list(stale_export_revisions),
                "release_decision_sha256": release_decision_sha256,
                "bundle_manifest_sha256": bundle_manifest_sha256,
                "raw_mutated": False,
            },
            sort_keys=True,
            indent=2,
        )
        + "\n"
    ).encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(
        prefix=".collection-disposition-",
        suffix=".tmp",
        dir=derived,
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return destination


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        registry = CollectionRegistry(
            args.project_root,
            args.collection_root,
            collection_id=args.collection_id,
        )
        dataset_profile = load_mini_dataset_profile(
            Path(args.project_root).resolve(),
            args.dataset_profile,
        )
        if args.command == "register":
            record = registry.register(
                args.episode_id,
                args.run_root,
                incomplete=args.incomplete,
                reason=args.reason,
            )
            run_root = Path(args.project_root).resolve() / record.run_root
            marker = _write_disposition_marker(
                run_root,
                episode_id=record.episode_id,
                disposition=record.disposition.value,
                reason=record.reason,
            )
            result: object = {**record.to_mapping(), "marker": str(marker)}
        elif args.command == "reject":
            record = registry.reject(args.episode_id, reason=args.reason)
            stale_exports = registry.stale_exports_for(args.episode_id)
            run_root = Path(args.project_root).resolve() / record.run_root
            marker = _write_disposition_marker(
                run_root,
                episode_id=record.episode_id,
                disposition=record.disposition.value,
                reason=record.reason,
                stale_export_revisions=tuple(item.revision_id for item in stale_exports),
            )
            result = {
                **record.to_mapping(),
                "marker": str(marker),
                "recoverable": True,
                "stale_export_revisions": [item.revision_id for item in stale_exports],
            }
        elif args.command == "restore":
            record = registry.restore(args.episode_id, reason=args.reason)
            stale_exports = registry.stale_exports_for(args.episode_id)
            run_root = Path(args.project_root).resolve() / record.run_root
            marker = _write_disposition_marker(
                run_root,
                episode_id=record.episode_id,
                disposition="candidate_regate_required",
                reason=record.reason,
                stale_export_revisions=tuple(item.revision_id for item in stale_exports),
            )
            result = {
                **record.to_mapping(),
                "marker": str(marker),
                "regate_required": True,
                "stale_export_revisions": [item.revision_id for item in stale_exports],
            }
        elif args.command == "accept":
            matches = tuple(
                record
                for record in registry.records()
                if record.episode_id == args.episode_id
            )
            if len(matches) != 1:
                raise KeyError(f"episode is not registered: {args.episode_id}")
            accepted_count = sum(
                record.disposition is EpisodeDisposition.ACCEPTED
                and record.episode_id != args.episode_id
                for record in registry.records()
            )
            if accepted_count >= dataset_profile.retained_episode_hard_limit:
                raise ValueError("accepted episode count reached the frozen collection limit")
            expected_release_root = (
                Path(args.project_root).resolve()
                / matches[0].run_root
                / "derived"
                / "release"
            ).resolve()
            if args.release_artifact_root.is_symlink():
                raise ValueError("release artifact root must not be a symbolic link")
            supplied_release_root = args.release_artifact_root.resolve()
            if supplied_release_root != expected_release_root:
                raise ValueError("release artifact must belong to the registered run root")
            release = load_release_decision_artifact(
                args.release_artifact_root,
                expected_run_id=args.episode_id,
            )
            if not release.decision.passed:
                raise ValueError("a failed release decision cannot be accepted")
            run_root = Path(args.project_root).resolve() / matches[0].run_root
            bundle = validate_episode_bundle(run_root)
            if (
                bundle.manifest.get("release_decision_sha256")
                != release.decision_sha256
            ):
                raise ValueError("bundle and release decision digests differ")
            load_policy_episode(run_root)
            record = registry.accept(
                args.episode_id,
                release_decision_sha256=release.decision_sha256,
                reason=args.reason,
            )
            marker = _write_disposition_marker(
                run_root,
                episode_id=record.episode_id,
                disposition=record.disposition.value,
                reason=record.reason,
                release_decision_sha256=release.decision_sha256,
                bundle_manifest_sha256=bundle.manifest_sha256,
            )
            result = {**record.to_mapping(), "marker": str(marker)}
        elif args.command == "annotate":
            matches = tuple(
                record
                for record in registry.records()
                if record.episode_id == args.episode_id
            )
            if len(matches) != 1:
                raise KeyError(f"episode is not registered: {args.episode_id}")
            record = matches[0]
            if record.disposition is not EpisodeDisposition.CANDIDATE:
                raise ValueError("only a candidate episode can be annotated")
            annotation = DatasetEpisodeAnnotation(
                run_id=record.episode_id,
                task=args.task,
                operator_note=args.operator_note,
            )
            path = write_episode_annotation(
                Path(args.project_root).resolve() / record.run_root,
                annotation,
            )
            result = {**annotation.to_mapping(), "path": str(path)}
        elif args.command == "purge":
            result = registry.quarantine_for_purge(
                args.episode_id,
                confirmation=args.confirm_episode_id,
                reason=args.reason,
            ).to_mapping()
        elif args.command == "list":
            result = [item.to_mapping() for item in registry.records()]
        else:
            result = [item.to_mapping() for item in registry.exports()]
    except (KeyError, OSError, ValueError) as exc:
        print(f"episode management failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
