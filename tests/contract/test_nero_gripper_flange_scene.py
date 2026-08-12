from pathlib import Path
import struct

import yaml

from wujihand.runtime import ConfigRepository, SessionResolver, SourceLock


ROOT = Path(__file__).parents[2]
SESSION = (
    "configs/sessions/isaac_nero_dual_hand2_tframe_gripper_flange_inspection_v2026_8_3_v1.yaml"
)


def test_gripper_flange_source_and_generated_asset_are_locked() -> None:
    lock = SourceLock.load(ConfigRepository(ROOT))
    source = lock.record("agilex-agx-arm-urdf-nero-gripper-flange")
    generated = lock.record("agilex-nero-gripper-flange-isaac-6-0-1-v1")

    assert dict(source.revision)["commit"] == "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
    assert source.expected_artifact_hash("nero/meshes/gripper_flange.stl") == (
        "1ae6d1c5af001582a3328564839e8eb5f2acec02e7e2b19e65fc3c43f8a9c95a"
    )
    assert dict(generated.revision)["sha256"] == generated.expected_tree_hash("nero_description")


def test_adapter_plate_is_a_locked_627_overlay_on_the_83_hand() -> None:
    lock = SourceLock.load(ConfigRepository(ROOT))
    generated = lock.record("wuji-hand2-adapter-plate-isaac-6-0-1-v1")
    assert set(generated.derived_from) == {
        "wuji-description-v2026-6-27@aee64892ebcf8e3237bedc30231bb09476cbc71d",
        "wuji-description-v2026-8-3@8271644a78d69ed9a4adcf9165d882c64ad33dfa",
    }
    profile = yaml.safe_load(
        (ROOT / "configs/profiles/wuji_hand2_adapter_plate_isaac_6_0_1_import_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert profile["plate_source"]["lock_id"] == "wuji-description-v2026-6-27"
    assert profile["hand_source"]["lock_id"] == "wuji-description-v2026-8-3"
    assert {
        side: evidence["sha256"]
        for side, evidence in profile["hand_source"]["whole_hand_step"].items()
    } == {
        "left": "62f731fcc11e48fa32478f4a730b2e488bdfa05e99c843fe05b670debab74b18",
        "right": "df9ba9fec37f91dc0a2bb0d0b1ff2873159a9df0f7098b31e49059a8159a7067",
    }
    assert (
        "generated_hand_body_and_control_asset_remain_v2026_8_3"
        in profile["assumptions"]
    )


def test_candidate_session_uses_the_parallel_flange_asset() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION, verify_artifacts=False)

    for side in ("left", "right"):
        arm = resolved.instance(f"nero_{side}")
        assert arm.asset.asset_id == "agilex_nero_gripper_flange"
        assert arm.asset.frame_name("kinematic_flange") == "link7"
        assert arm.asset.frame_name("tool_flange") == "gripper_flange"
        assert arm.binding.backend_frame("gripper_flange") == "gripper_flange"
        assert arm.binding.compatibility_profile is None
    arm_to_hand = [
        attachment
        for attachment in resolved.assembly.attachments
        if resolved.assembly.instance(attachment.parent.instance).role == "arm"
    ]
    assert len(arm_to_hand) == 2
    by_side = {
        resolved.instance(attachment.child.instance).asset.side: attachment
        for attachment in arm_to_hand
    }
    assert set(by_side) == {"left", "right"}
    for attachment in by_side.values():
        assert attachment.parent.frame == "gripper_flange"
        assert attachment.transform.position_m == (0.0, 0.0, 0.012)
    assert by_side["left"].transform.quat_wxyz == (0.0, 0.0, 1.0, 0.0)
    assert by_side["right"].transform.quat_wxyz == (0.0, 0.0, 1.0, 0.0)


def test_plate_outer_face_is_placed_on_the_cup_rim() -> None:
    mesh = ROOT / (
        "third_party/src/agx_arm_urdf_nero_gripper_flange/nero/meshes/"
        "gripper_flange.stl"
    )
    data = mesh.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    cup_rim_z = max(
        struct.unpack_from("<12fH", data, 84 + index * 50)[offset + 2]
        for index in range(triangle_count)
        for offset in (3, 6, 9)
    )
    resolved = SessionResolver(ROOT).resolve(SESSION, verify_artifacts=False)
    offsets = {
        attachment.transform.position_m[2]
        for attachment in resolved.assembly.attachments
        if resolved.assembly.instance(attachment.parent.instance).role == "arm"
    }
    assert offsets == {0.012}
    assert abs(cup_rim_z - offsets.pop()) < 1e-6


def test_candidate_keeps_d405_mounts_on_the_historical_plate_frame() -> None:
    resolved = SessionResolver(ROOT).resolve(SESSION, verify_artifacts=False)
    for side in ("left", "right"):
        hand = resolved.instance(f"hand_{side}")
        assert "adapter_plate_candidate" in hand.binding.binding_id
        assert hand.artifact is not None
        assert hand.artifact.source.name == "wuji-hand2-adapter-plate-isaac-6-0-1-v1"
        attachment = next(
            item
            for item in resolved.assembly.attachments
            if item.parent.instance == f"hand_{side}"
            and item.child.instance == f"mount_{side}"
        )
        assert attachment.transform.position_m == (0.0, 0.0, 0.0)
        expected = (
            (0.0, -0.7071067811865476, 0.7071067811865475, 0.0)
            if side == "left"
            else (0.0, -0.7071067811865476, -0.7071067811865475, 0.0)
        )
        assert attachment.transform.quat_wxyz == expected
