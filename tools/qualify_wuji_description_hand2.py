#!/usr/bin/env python3
# ruff: noqa: E402  # Project modules are imported after adding src to sys.path.
"""Statically qualify the pinned Wuji Hand 2 Description release pair."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from wujihand.integrity import sha256_file
from wujihand.runtime import ConfigRepository, SessionResolver, SourceLock
from wujihand.specs import ArtifactSpec


RELEASES = {
    "v2026.6.27": {
        "token": "v2026_6_27",
        "source": "wuji-description-v2026-6-27",
        "roots": {"left": "l_base_link", "right": "r_base_link"},
    },
    "v2026.8.3": {
        "token": "v2026_8_3",
        "source": "wuji-description-v2026-8-3",
        "roots": {"left": "l_wrist", "right": "r_wrist"},
    },
}
SIDES = ("left", "right")
SESSION_KINDS = ("fixed", "collision")


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional JSON report path.")
    parser.add_argument(
        "--skip-hash-verification",
        action="store_true",
        help="Inspect contracts without reading every locked file/tree.",
    )
    return parser.parse_args()


def _profile(release: str, side: str) -> tuple[Path, dict[str, Any]]:
    token = str(RELEASES[release]["token"])
    path = ROOT / f"configs/profiles/hand2_{side}_{token}.yaml"
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"profile must be a mapping: {path}")
    return path, value


def _source_head(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def _verify_source(
    source_lock: SourceLock,
    source_documents: dict[str, dict[str, Any]],
    source_name: str,
    *,
    verify_hashes: bool,
) -> dict[str, Any]:
    record = source_lock.record(source_name)
    source = source_documents[source_name]
    revision = dict(record.revision)
    commit = revision["commit"]
    source_root = ROOT / record.local_runtime_path
    artifacts = []
    for relative_path, digest in record.artifacts:
        resolved = source_lock.resolve(
            ArtifactSpec(
                source=source_name,
                source_revision=f"commit:{commit}",
                path=relative_path,
            ),
            verify=verify_hashes,
        )
        artifacts.append(
            {"path": relative_path, "sha256": digest, "exists": resolved.absolute_path.is_file()}
        )
    trees = []
    for relative_path, digest in record.asset_trees:
        resolved = source_lock.resolve(
            ArtifactSpec(
                source=source_name,
                source_revision=f"commit:{commit}",
                path=relative_path,
            ),
            tree=True,
            verify=verify_hashes,
        )
        trees.append(
            {"path": relative_path, "sha256": digest, "exists": resolved.absolute_path.is_dir()}
        )
    license_path = source_root / str(source["license_path"])
    actual_license_hash = sha256_file(license_path) if verify_hashes else None
    expected_license_hash = str(source["license_sha256"])
    if verify_hashes and actual_license_hash != expected_license_hash:
        raise RuntimeError(f"{source_name} license SHA-256 mismatch")
    head = _source_head(source_root)
    if head is not None and head != commit:
        raise RuntimeError(f"{source_name} checkout HEAD {head} differs from {commit}")
    return {
        "name": source_name,
        "tag": revision.get("tag"),
        "commit": commit,
        "checkout_head": head,
        "local_runtime_path": record.local_runtime_path,
        "license_sha256": expected_license_hash,
        "artifact_count": len(artifacts),
        "asset_tree_count": len(trees),
        "artifacts": artifacts,
        "asset_trees": trees,
        "hashes_verified": verify_hashes,
    }


def _urdf_summary(path: Path, profile: dict[str, Any]) -> dict[str, Any]:
    robot = ElementTree.parse(path).getroot()
    links = [str(link.attrib["name"]) for link in robot.findall("link")]
    all_joints = robot.findall("joint")
    child_links = {
        str(child.attrib["link"])
        for joint in all_joints
        if (child := joint.find("child")) is not None
    }
    roots = sorted(set(links) - child_links)
    movable = {
        str(joint.attrib["name"]): joint
        for joint in all_joints
        if joint.attrib.get("type") != "fixed"
    }
    profile_joints = profile.get("joints")
    if not isinstance(profile_joints, list):
        raise ValueError("profile joints must be a list")
    profile_names = [str(joint["name"]) for joint in profile_joints]
    missing = sorted(set(profile_names) - set(movable))
    extra = sorted(set(movable) - set(profile_names))
    joint_records = []
    limits_match = not missing and not extra
    for expected in profile_joints:
        name = str(expected["name"])
        joint = movable.get(name)
        if joint is None:
            continue
        axis = joint.find("axis")
        origin = joint.find("origin")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"URDF joint {name} has no limit")
        actual_limit = {
            field: float(limit.attrib[field]) for field in ("lower", "upper", "velocity")
        }
        limits_match = limits_match and all(
            abs(actual_limit[field] - float(expected[field])) <= 1e-9
            for field in ("lower", "upper", "velocity")
        )
        joint_records.append(
            {
                "name": name,
                "type": joint.attrib.get("type"),
                "parent": joint.find("parent").attrib["link"],  # type: ignore[union-attr]
                "child": joint.find("child").attrib["link"],  # type: ignore[union-attr]
                "axis_xyz": None if axis is None else axis.attrib.get("xyz"),
                "origin_xyz": None if origin is None else origin.attrib.get("xyz"),
                "origin_rpy": None if origin is None else origin.attrib.get("rpy"),
                "limit": actual_limit,
            }
        )
    return {
        "path": str(path.relative_to(ROOT)),
        "sha256": sha256_file(path),
        "robot_name": robot.attrib.get("name"),
        "root_links": roots,
        "link_count": len(links),
        "joint_count": len(all_joints),
        "movable_joint_count": len(movable),
        "visual_count": len(robot.findall(".//visual")),
        "collision_count": len(robot.findall(".//collision")),
        "profile_order_match": list(movable) == profile_names,
        "profile_limits_match": limits_match,
        "missing_profile_joints": missing,
        "extra_movable_joints": extra,
        "joints": joint_records,
    }


def _mjcf_summary(path: Path) -> dict[str, Any]:
    model = ElementTree.parse(path).getroot()
    worldbody = model.find("worldbody")
    root_bodies = [] if worldbody is None else [body.attrib.get("name") for body in worldbody.findall("body")]
    joints = [joint.attrib.get("name") for joint in model.findall(".//joint")]
    actuator = model.find("actuator")
    actuators = []
    if actuator is not None:
        actuators = [
            {
                "type": element.tag,
                "name": element.attrib.get("name"),
                "joint": element.attrib.get("joint"),
                "kp": element.attrib.get("kp"),
                "kv": element.attrib.get("kv"),
            }
            for element in actuator
        ]
    return {
        "path": str(path.relative_to(ROOT)),
        "sha256": sha256_file(path),
        "model_name": model.attrib.get("model"),
        "root_bodies": root_bodies,
        "joint_count": len(joints),
        "joint_names": joints,
        "actuator_count": len(actuators),
        "actuators": actuators,
        "geom_count": len(model.findall(".//geom")),
        "exclude_count": len(model.findall(".//contact/exclude")),
        "site_names": [site.attrib.get("name") for site in model.findall(".//site")],
    }


def _session_summary(resolver: SessionResolver, release: str, side: str, kind: str) -> dict[str, Any]:
    token = str(RELEASES[release]["token"])
    path = ROOT / f"configs/sessions/isaac_hand2_{side}_{kind}_qualification_{token}_v1.yaml"
    resolved = resolver.resolve(path)
    hand = resolved.instance("hand")
    return {
        "path": str(path.relative_to(ROOT)),
        "session_id": resolved.session.session_id,
        "session_hash": resolved.session_hash,
        "runtime_role": resolved.session.runtime_role,
        "asset_revision": hand.asset.revision,
        "binding_id": hand.binding.binding_id,
        "binding_root": hand.binding.root,
        "canonical_profile": hand.asset.canonical_profile,
        "sources": [source.name for source in resolved.source_records],
    }


def _source_documents() -> dict[str, dict[str, Any]]:
    value = yaml.safe_load((ROOT / "third_party/sources.lock.yaml").read_text(encoding="utf-8"))
    if not isinstance(value, dict) or not isinstance(value.get("sources"), list):
        raise ValueError("source lock must contain sources")
    return {
        str(source["name"]): source
        for source in value["sources"]
        if isinstance(source, dict) and "name" in source
    }


def main() -> int:
    args = _args()
    repository = ConfigRepository(ROOT)
    source_lock = SourceLock.load(repository)
    resolver = SessionResolver(ROOT)
    source_documents = _source_documents()
    failures: list[str] = []
    report: dict[str, Any] = {
        "schema": "wujihand.wuji_description_hand2_qualification.v1",
        "beta_warning": (
            "Wuji Hand 2 remains Beta1; re-check official updates before reuse."
        ),
        "simulation_only": True,
        "hardware_reusable": False,
        "sources": {},
        "models": {},
        "sessions": [],
        "comparisons": {},
        "failures": failures,
    }
    verify_hashes = not args.skip_hash_verification
    for release, metadata in RELEASES.items():
        source_name = str(metadata["source"])
        report["sources"][release] = _verify_source(
            source_lock,
            source_documents,
            source_name,
            verify_hashes=verify_hashes,
        )
        release_models: dict[str, Any] = {}
        for side in SIDES:
            profile_path, profile = _profile(release, side)
            derived = profile["derived_from"]
            source_root = ROOT / source_lock.record(source_name).local_runtime_path
            urdf = _urdf_summary(source_root / str(derived["urdf"]), profile)
            mjcf_relative = derived.get("mjcf")
            if mjcf_relative is None:
                mjcf_relative = next(
                    path
                    for path, _ in source_lock.record(source_name).artifacts
                    if path.endswith(f"/mjcf/{side}.xml")
                )
            mjcf = _mjcf_summary(source_root / str(mjcf_relative))
            expected_root = str(metadata["roots"][side])  # type: ignore[index]
            if urdf["root_links"] != [expected_root]:
                failures.append(
                    f"{release}/{side}: URDF root {urdf['root_links']} != {expected_root}"
                )
            if not urdf["profile_order_match"] or not urdf["profile_limits_match"]:
                failures.append(f"{release}/{side}: URDF q20/profile contract mismatch")
            release_models[side] = {
                "profile": str(profile_path.relative_to(ROOT)),
                "profile_sha256": sha256_file(profile_path),
                "profile_tag": derived.get("tag"),
                "profile_commit": derived.get("commit"),
                "urdf": urdf,
                "mjcf": mjcf,
            }
            for kind in SESSION_KINDS:
                session = _session_summary(resolver, release, side, kind)
                if session["sources"] != [source_name]:
                    failures.append(
                        f"{release}/{side}/{kind}: resolved sources {session['sources']}"
                    )
                if session["binding_root"] != expected_root:
                    failures.append(
                        f"{release}/{side}/{kind}: binding root {session['binding_root']}"
                    )
                report["sessions"].append(session)
        report["models"][release] = release_models

    comparisons: dict[str, Any] = {}
    for side in SIDES:
        old = report["models"]["v2026.6.27"][side]["urdf"]
        new = report["models"]["v2026.8.3"][side]["urdf"]
        old_joints = {joint["name"]: joint for joint in old["joints"]}
        new_joints = {joint["name"]: joint for joint in new["joints"]}
        names_equal = list(old_joints) == list(new_joints)
        limits_equal = names_equal and all(
            old_joints[name]["limit"] == new_joints[name]["limit"] for name in old_joints
        )
        changed_axes = [
            {
                "joint": name,
                "old": old_joints[name]["axis_xyz"],
                "new": new_joints[name]["axis_xyz"],
            }
            for name in old_joints
            if name in new_joints
            if old_joints[name]["axis_xyz"] != new_joints[name]["axis_xyz"]
        ]
        old_mjcf = report["models"]["v2026.6.27"][side]["mjcf"]
        new_mjcf = report["models"]["v2026.8.3"][side]["mjcf"]
        old_actuators = old_mjcf["actuators"]
        new_actuators = new_mjcf["actuators"]
        actuator_joint_order_equal = [item["joint"] for item in old_actuators] == [
            item["joint"] for item in new_actuators
        ]
        actuator_gains_equal = [
            (item["kp"], item["kv"]) for item in old_actuators
        ] == [(item["kp"], item["kv"]) for item in new_actuators]
        if not names_equal or not limits_equal:
            failures.append(f"{side}: old/new q20 names or limits changed unexpectedly")
        if not actuator_joint_order_equal or not actuator_gains_equal:
            failures.append(f"{side}: old/new MJCF actuator mapping or gains changed")
        comparisons[side] = {
            "q20_names_equal": names_equal,
            "q20_limits_equal": limits_equal,
            "old_root": old["root_links"],
            "new_root": new["root_links"],
            "changed_axis_count": len(changed_axes),
            "changed_axes": changed_axes,
            "link_count": {"old": old["link_count"], "new": new["link_count"]},
            "collision_count": {
                "old": old["collision_count"],
                "new": new["collision_count"],
            },
            "mjcf": {
                "actuator_joint_order_equal": actuator_joint_order_equal,
                "actuator_gains_equal": actuator_gains_equal,
                "actuator_names_changed": [
                    item["name"] for item in old_actuators
                ]
                != [item["name"] for item in new_actuators],
                "geom_count": {
                    "old": old_mjcf["geom_count"],
                    "new": new_mjcf["geom_count"],
                },
                "exclude_count": {
                    "old": old_mjcf["exclude_count"],
                    "new": new_mjcf["exclude_count"],
                },
                "site_names_equal": old_mjcf["site_names"] == new_mjcf["site_names"],
            },
        }
    report["comparisons"] = comparisons
    report["passed"] = not failures
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
        print(
            json.dumps(
                {
                    "passed": report["passed"],
                    "output": str(args.output),
                    "failures": failures,
                    "comparisons": comparisons,
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        print(payload, end="")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
