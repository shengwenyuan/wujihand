"""Export accepted registry episodes into one immutable LeRobot revision."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from wujihand.dataset import (
    CollectionRegistry,
    EpisodeDisposition,
    load_mini_dataset_profile,
    load_policy_episode,
    load_release_decision_artifact,
    load_visual_domain_variant_profile,
    validate_episode_bundle,
)
from wujihand.domain.recording import validate_recording_token

from .exporter import export_collection

DEFAULT_PROFILE = "configs/profiles/isaac_nero_hand2_triview_q54_mini_dataset_v1.yaml"
DEFAULT_VARIANT_PROFILE = (
    "configs/profiles/isaac_mini_dataset_visual_domain_variants_v1.yaml"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, required=True)
    parser.add_argument("--collection-root", type=Path, required=True)
    parser.add_argument("--collection-id", required=True)
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--variant-profile", default=DEFAULT_VARIANT_PROFILE)
    parser.add_argument(
        "--vision-variants",
        nargs="+",
        default=("nominal",),
    )
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument(
        "--revision-id",
        help="Collection export identity; defaults to the destination directory name.",
    )
    parser.add_argument(
        "--robot-type",
        default="agile_nero_dual_wuji_hand2_simulation",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        project_root = args.project_root.resolve()
        profile = load_mini_dataset_profile(project_root, args.profile)
        variant_profile = load_visual_domain_variant_profile(
            project_root,
            args.variant_profile,
        )
        variant_ids = tuple(args.vision_variants)
        if len(set(variant_ids)) != len(variant_ids):
            raise ValueError("vision variant IDs are duplicated")
        variants = tuple(variant_profile.variant(item) for item in variant_ids)
        registry = CollectionRegistry(
            project_root,
            args.collection_root,
            collection_id=args.collection_id,
        )
        records = tuple(
            record
            for record in registry.records()
            if record.disposition is EpisodeDisposition.ACCEPTED
        )
        if not records:
            raise ValueError("collection contains no accepted episodes")
        if len(records) > profile.retained_episode_hard_limit:
            raise ValueError("accepted episode count exceeds the frozen profile limit")
        if len(records) * len(variants) > profile.retained_episode_hard_limit:
            raise ValueError("expanded visual-variant episode count exceeds the frozen limit")
        if any(record.release_decision_sha256 is None for record in records):
            raise ValueError("accepted episode is missing its release decision digest")
        destination = args.destination.resolve()
        try:
            destination.relative_to(project_root)
        except ValueError as exc:
            raise ValueError("LeRobot destination must remain inside the project root") from exc
        revision_id = validate_recording_token(
            args.revision_id or destination.name,
            field="revision_id",
        )
        episodes = []
        for record in records:
            run_root = project_root / record.run_root
            release = load_release_decision_artifact(
                run_root / "derived" / "release",
                expected_run_id=record.episode_id,
            )
            if (
                not release.decision.passed
                or release.decision_sha256 != record.release_decision_sha256
            ):
                raise ValueError("accepted registry release digest is stale")
            bundle = validate_episode_bundle(run_root)
            if bundle.manifest.get("release_decision_sha256") != release.decision_sha256:
                raise ValueError("accepted bundle release digest is stale")
            if (
                bundle.manifest.get("collection_id") != args.collection_id
                or bundle.manifest.get("dataset_profile_sha256") != profile.file_sha256
            ):
                raise ValueError("accepted bundle collection/profile identity differs")
            expected_provenance = {
                "collection_id": args.collection_id,
                "dataset_profile_sha256": bundle.manifest.get(
                    "dataset_profile_sha256"
                ),
                "deployment_sha256": bundle.manifest.get("deployment_hash"),
                "session_sha256": bundle.manifest.get("session_hash"),
                "assembly_sha256": bundle.manifest.get("assembly_hash"),
                "workcell_sha256": bundle.manifest.get("workcell_hash"),
            }
            for variant in variants:
                artifact_name = (
                    "vision"
                    if variant.variant_id == "nominal"
                    else f"vision_{variant.variant_id}"
                )
                episode = load_policy_episode(
                    run_root,
                    vision_artifact_name=artifact_name,
                    visual_domain_variant=variant,
                    visual_domain_variant_profile_sha256=variant_profile.file_sha256,
                )
                provenance = episode.vision.provenance
                if any(
                    getattr(provenance, key) != expected
                    for key, expected in expected_provenance.items()
                ):
                    raise ValueError("accepted vision and bundle provenance differ")
                expected_renderer_suffix = (
                    f"-{variant.variant_id}-{variant.digest_sha256[:12]}"
                )
                if not provenance.renderer_identity.endswith(expected_renderer_suffix):
                    raise ValueError("accepted vision domain-variant provenance differs")
                episodes.append(episode)
        result = export_collection(
            tuple(episodes),
            profile.q54,
            destination,
            repo_id=args.repo_id,
            robot_type=args.robot_type,
        )
        export_record = registry.record_export(
            revision_id=revision_id,
            dataset_root=result.root,
            manifest_sha256=result.manifest_sha256,
            episode_ids=tuple(record.episode_id for record in records),
        )
    except (FileExistsError, KeyError, OSError, ValueError) as exc:
        print(f"mini dataset export failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "root": str(result.root),
                "repo_id": result.repo_id,
                "episode_count": result.episode_count,
                "frame_count": result.frame_count,
                "manifest_sha256": result.manifest_sha256,
                "checksums_sha256": result.checksums_sha256,
                "collection_export": export_record.to_mapping(),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
