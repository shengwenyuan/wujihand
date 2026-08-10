from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest

from wujihand.adapters.simulation.hand2_model import load_hand2_model_profile
from wujihand.domain import HAND2_RIGHT_LAYOUT
from wujihand.runtime import (
    SessionResolver,
    load_mujoco_table_scene_config,
    load_rotation_ball_config,
    validate_transport_pair,
)
from wujihand.specs import ControlLayoutSpec


ROOT = Path(__file__).parents[2]
SESSIONS = ROOT / "configs/sessions"

SESSION_NAMES = (
    "isaac_hand2_right_fixed_qualification_v2026_6_27_v1.yaml",
    "isaac_hand2_right_rotation_ball_qualification_v1.yaml",
    "isaac_hand2_right_rotation_ball_teleop_v1.yaml",
    "isaac_hand2_teleop_v1.yaml",
    "isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml",
    "mediapipe_hand2_hand_command_udp_v1.yaml",
    "mediapipe_hand2_q20_udp_v1.yaml",
    "mujoco_fr3v2_hand2_right_table_v1.yaml",
)
SESSION_HASH_GOLDEN = ROOT / "tests/golden/five_layer_session_hashes_v1.json"


@pytest.fixture(scope="module")
def resolver() -> SessionResolver:
    return SessionResolver(ROOT)


@pytest.mark.parametrize("name", SESSION_NAMES)
def test_every_current_session_resolves_without_simulator_import(
    resolver: SessionResolver, name: str
) -> None:
    resolved = resolver.resolve(SESSIONS / name)

    assert resolved.session_hash == resolver.resolve(SESSIONS / name).session_hash
    assert len(resolved.session_hash) == 64
    assert set(dict(resolved.session.bindings)) == {
        instance.instance_id for instance in resolved.instances
    }
    assert set(dict(resolved.session.placements)) == set(resolved.assembly.roots)
    assert all(
        instance.binding.backend == resolved.session.backend
        for instance in resolved.instances
    )


def test_current_session_hashes_match_the_reviewed_golden(
    resolver: SessionResolver,
) -> None:
    document = json.loads(SESSION_HASH_GOLDEN.read_text(encoding="utf-8"))
    assert document["schema"] == "wujihand.session_hash_golden.v1"
    expected = document["sessions"]
    actual = {
        name: resolver.resolve(SESSIONS / name).session_hash
        for name in SESSION_NAMES
    }

    assert actual == expected


def test_cli_overrides_are_part_of_the_resolved_fingerprint(
    resolver: SessionResolver,
) -> None:
    path = SESSIONS / "mujoco_fr3v2_hand2_right_table_v1.yaml"
    baseline = resolver.resolve(path)
    first = resolver.resolve(path, overrides={"scene_profile": "legacy-a.yaml"})
    second = resolver.resolve(path, overrides={"scene_profile": "legacy-b.yaml"})

    assert baseline.session_hash != first.session_hash != second.session_hash
    assert {
        override.key: (override.value_type, override.value)
        for override in first.overrides
    } == {"scene_profile": ("string", "legacy-a.yaml")}


def test_override_fingerprint_preserves_type_and_file_content(
    resolver: SessionResolver,
    tmp_path: Path,
) -> None:
    path = SESSIONS / "mujoco_fr3v2_hand2_right_table_v1.yaml"
    override = tmp_path / "profile.yaml"
    override.write_text("revision: one\n", encoding="utf-8")
    first = resolver.resolve(path, overrides={"profile": override, "attempt": 1})
    string_type = resolver.resolve(path, overrides={"attempt": "1"})
    int_type = resolver.resolve(path, overrides={"attempt": 1})
    override.write_text("revision: two\n", encoding="utf-8")
    second = resolver.resolve(path, overrides={"profile": override, "attempt": 1})

    assert first.session_hash != second.session_hash
    assert string_type.session_hash != int_type.session_hash
    first_profile = next(item for item in first.overrides if item.key == "profile")
    second_profile = next(item for item in second.overrides if item.key == "profile")
    assert first_profile.file_sha256 != second_profile.file_sha256


def test_resolved_snapshot_cannot_be_mutated_behind_its_hash(
    resolver: SessionResolver,
) -> None:
    resolved = resolver.resolve(
        SESSIONS / "isaac_hand2_right_fixed_qualification_v2026_6_27_v1.yaml"
    )
    exported = resolved.to_mapping()
    session = exported["session"]
    assert isinstance(session, dict)
    session["session_id"] = "mutated"

    fresh_session = resolved.snapshot["session"]
    assert isinstance(fresh_session, dict)
    assert (
        fresh_session["session_id"]
        == "isaac_hand2_right_fixed_qualification_v2026_6_27_v1"
    )
    assert resolved.to_mapping()["session_hash"] == resolved.session_hash


def test_mediapipe_sessions_pair_with_only_the_matching_isaac_contract(
    resolver: SessionResolver,
) -> None:
    q20_producer = resolver.resolve(SESSIONS / "mediapipe_hand2_q20_udp_v1.yaml")
    q20_consumer = resolver.resolve(SESSIONS / "isaac_hand2_teleop_v1.yaml")
    pose_producer = resolver.resolve(
        SESSIONS / "mediapipe_hand2_hand_command_udp_v1.yaml"
    )
    pose_consumer = resolver.resolve(
        SESSIONS / "isaac_hand2_right_rotation_ball_teleop_v1.yaml"
    )

    validate_transport_pair(q20_producer, q20_consumer)
    validate_transport_pair(pose_producer, pose_consumer)
    with pytest.raises(ValueError, match="transport contracts differ"):
        validate_transport_pair(q20_producer, pose_consumer)


def test_transport_pair_preserves_repeated_layout_multiplicity(
    resolver: SessionResolver,
) -> None:
    producer = resolver.resolve(SESSIONS / "mediapipe_hand2_q20_udp_v1.yaml")
    consumer = resolver.resolve(SESSIONS / "isaac_hand2_teleop_v1.yaml")
    duplicated_runtime = replace(
        producer.session.runtime,
        control_layouts=(
            *producer.session.runtime.control_layouts,
            ControlLayoutSpec(
                instance_id="second_hand",
                group_id="finger_joints",
                layout_id="wuji_hand2_right_firmware_v1",
            ),
        ),
    )
    duplicated_producer = replace(
        producer,
        session=replace(producer.session, runtime=duplicated_runtime),
        instances=(
            *producer.instances,
            replace(producer.instance("hand"), instance_id="second_hand"),
        ),
    )

    with pytest.raises(ValueError, match="control layouts"):
        validate_transport_pair(duplicated_producer, consumer)


def test_transport_pair_checks_product_revision_side_and_semantics(
    resolver: SessionResolver,
) -> None:
    producer = resolver.resolve(SESSIONS / "mediapipe_hand2_q20_udp_v1.yaml")
    consumer = resolver.resolve(SESSIONS / "isaac_hand2_teleop_v1.yaml")
    hand = producer.instance("hand")
    wrong_hand = replace(hand, asset=replace(hand.asset, side="left"))
    wrong_producer = replace(producer, instances=(wrong_hand,))

    with pytest.raises(ValueError, match="control layouts"):
        validate_transport_pair(wrong_producer, consumer)


def test_hand_asset_binding_and_domain_layout_are_exact(
    resolver: SessionResolver,
) -> None:
    resolved = resolver.resolve(SESSIONS / "isaac_hand2_teleop_v1.yaml")
    hand = resolved.instance("hand")
    profile = load_hand2_model_profile(
        ROOT / "configs/profiles/hand2_right_v2026_6_27.yaml"
    )

    assert (
        hand.asset.revision
        == hand.binding.asset_revision
        == "beta1_description_v2026_6_27"
    )
    assert hand.asset.side == hand.binding.asset_side == "right"
    assert profile.layout == HAND2_RIGHT_LAYOUT
    assert hand.binding.group_binding("finger_joints").joints == profile.layout.names
    assert hand.artifact is not None
    assert hand.artifact.relative_path == (
        "hand2_beta/body/usd/right/wujihand.usd"
    )
    assert dict(hand.artifact.source.revision)["tag"] == "v2026.6.27"


def test_mujoco_session_matches_legacy_typed_leaf(
    resolver: SessionResolver,
) -> None:
    resolved = resolver.resolve(
        SESSIONS / "mujoco_fr3v2_hand2_right_table_v1.yaml"
    )
    legacy = load_mujoco_table_scene_config(
        ROOT / "configs/base/mujoco_fr3v2_hand2_right_table_v2026_6_27_v1.yaml"
    )
    arm = resolved.instance("arm")
    hand = resolved.instance("hand")
    attachment = resolved.assembly.attachments[0]

    assert arm.artifact is not None
    assert hand.artifact is not None
    assert (
        arm.artifact.absolute_path.relative_to(ROOT)
        == legacy.assets.arm_mjcf
    )
    assert (
        hand.artifact.absolute_path.relative_to(ROOT)
        == legacy.assets.hand_mjcf
    )
    assert arm.artifact.expected_sha256 == legacy.assets.arm_mjcf_sha256
    assert hand.artifact.expected_sha256 == legacy.assets.hand_mjcf_sha256
    assert resolved.session.runtime.compatibility_profile == (
        "configs/base/mujoco_fr3v2_hand2_right_table_v2026_6_27_v1.yaml"
    )
    assert attachment.parent.frame == "tool_flange"
    assert attachment.child.frame == "hand_base"
    assert attachment.transform.position_m == legacy.hand_attachment.position_m
    assert attachment.transform.quat_wxyz == legacy.hand_attachment.quat_wxyz


def test_rotation_ball_session_matches_legacy_typed_leaf(
    resolver: SessionResolver,
) -> None:
    resolved = resolver.resolve(
        SESSIONS / "isaac_hand2_right_rotation_ball_qualification_v1.yaml"
    )
    legacy = load_rotation_ball_config(
        ROOT / "configs/base/hand2_rotation_ball_v2026_6_27_v1.yaml"
    )
    hand = resolved.instance("hand")

    assert hand.artifact is not None
    assert hand.artifact.relative_path == legacy.provenance["usd"]
    assert hand.artifact.expected_sha256 == legacy.provenance["usd_sha256"]
    assert resolved.workcell.compatibility_profile == (
        "configs/base/hand2_rotation_ball_v2026_6_27_v1.yaml"
    )
    assert {instance.instance_id for instance in resolved.instances} == {
        "hand",
        "wrist",
    }
    assert sum(
        len(group.joints)
        for instance in resolved.instances
        for group in instance.binding.group_bindings
    ) == 23


def test_fixed_isaac_workcell_owns_existing_geometry(
    resolver: SessionResolver,
) -> None:
    resolved = resolver.resolve(
        SESSIONS / "isaac_hand2_right_fixed_qualification_v2026_6_27_v1.yaml"
    )
    table = next(
        entity for entity in resolved.workcell.entities if entity.entity_id == "table"
    )
    mount = resolved.workcell.mount("hand_fixed_mount")

    assert table.transform.position_m == (0.02, 0.0, 0.35)
    assert table.primitive.size_m == (0.8, 0.6, 0.06)
    assert mount.transform.position_m == (-0.1, 0.0, 0.43)
    assert mount.transform.quat_wxyz == pytest.approx(
        (0.7071067811865476, 0.0, -0.7071067811865475, 0.0)
    )
