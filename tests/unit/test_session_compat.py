from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from wujihand.runtime import SessionResolver
from wujihand.runtime.session_compat import (
    ISAAC_FIXED_PREVIEW_SESSION,
    ISAAC_FIXED_TELEOP_SESSION,
    ISAAC_ROTATION_QUALIFICATION_SESSION,
    MEDIAPIPE_Q20_SESSION,
    MUJOCO_TABLE_SESSION,
    fixed_hand_workcell_runtime,
    resolve_isaac_hand_runtime,
    resolve_mediapipe_session,
    resolve_mujoco_table_runtime,
    resolve_rotation_ball_runtime,
)
from wujihand.specs import MountSpec, PoseSpec, WorkcellFrameSpec


ROOT = Path(__file__).parents[2]


def test_mujoco_bridge_validates_the_legacy_leaf_against_five_layers() -> None:
    resolved, config, path = resolve_mujoco_table_runtime(
        ROOT, session_path=ROOT / MUJOCO_TABLE_SESSION
    )

    assert resolved.session.session_id == "mujoco_fr3v2_hand2_right_table_v1"
    assert config.name == "mujoco_fr3v2_hand2_right_table_v2026_6_27_v1"
    assert path == (
        ROOT / "configs/base/mujoco_fr3v2_hand2_right_table_v2026_6_27_v1.yaml"
    )


def test_rotation_bridge_validates_pinned_usd_provenance() -> None:
    runtime, scene, path = resolve_rotation_ball_runtime(
        ROOT,
        session_path=ROOT / ISAAC_ROTATION_QUALIFICATION_SESSION,
        runtime_roles={"qualification"},
    )

    assert runtime.resolved.session.runtime_role == "qualification"
    assert scene.name == "hand2_right_rotation_ball_v2026_6_27_v1"
    assert path == ROOT / "configs/base/hand2_rotation_ball_v2026_6_27_v1.yaml"


def test_fixed_workcell_bridge_extracts_runner_values() -> None:
    runtime = resolve_isaac_hand_runtime(
        ROOT,
        session_path=ROOT / ISAAC_FIXED_PREVIEW_SESSION,
        runtime_roles={"simulation"},
    )
    workcell = fixed_hand_workcell_runtime(runtime.resolved)

    assert workcell.table.primitive.size_m == (0.8, 0.6, 0.06)
    assert workcell.hand_mount.position_m == (-0.1, 0.0, 0.43)
    assert workcell.camera_eye_m == (0.42, -0.48, 0.88)
    assert workcell.camera_target_m == (-0.04, 0.0, 0.43)


def test_fixed_workcell_bridge_follows_the_session_root_placement() -> None:
    runtime = resolve_isaac_hand_runtime(
        ROOT,
        session_path=ROOT / ISAAC_FIXED_PREVIEW_SESSION,
        runtime_roles={"simulation"},
    )
    alternate_mount = MountSpec(
        mount_id="alternate_hand_mount",
        frame=runtime.resolved.workcell.world_frame,
        transform=PoseSpec(
            position_m=(0.1, 0.2, 0.5),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        ),
    )
    resolved = replace(
        runtime.resolved,
        session=replace(
            runtime.resolved.session,
            placements=(("hand", alternate_mount.mount_id),),
        ),
        workcell=replace(
            runtime.resolved.workcell,
            mounts=(*runtime.resolved.workcell.mounts, alternate_mount),
        ),
    )

    workcell = fixed_hand_workcell_runtime(resolved)

    assert workcell.hand_mount == alternate_mount.transform


def test_compatibility_bridge_rejects_wrong_runtime_role() -> None:
    with pytest.raises(ValueError, match="does not support runtime role"):
        resolve_isaac_hand_runtime(
            ROOT,
            session_path=ROOT / ISAAC_FIXED_PREVIEW_SESSION,
            runtime_roles={"teleop_consumer"},
        )


def test_mediapipe_bridge_requires_selected_wire_contract() -> None:
    resolved = resolve_mediapipe_session(
        ROOT,
        session_path=ROOT / MEDIAPIPE_Q20_SESSION,
        expected_transport_contract="wujihand.q20.v1",
    )
    assert resolved.session.runtime_role == "teleop_producer"

    with pytest.raises(ValueError, match="transport contract"):
        resolve_mediapipe_session(
            ROOT,
            session_path=ROOT / MEDIAPIPE_Q20_SESSION,
            expected_transport_contract="wujihand.hand_command.v2",
        )


def test_fixed_consumer_rejects_the_rotation_wire_contract(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    resolved = SessionResolver(ROOT).resolve(ROOT / ISAAC_FIXED_TELEOP_SESSION)
    wrong_runtime = replace(
        resolved.session.runtime,
        transport_contract="wujihand.hand_command.v2",
    )
    wrong = replace(
        resolved,
        session=replace(resolved.session, runtime=wrong_runtime),
    )
    monkeypatch.setattr(
        SessionResolver,
        "resolve",
        lambda self, *args, **kwargs: wrong,
    )

    with pytest.raises(ValueError, match="requires wujihand.q20.v1"):
        resolve_isaac_hand_runtime(
            ROOT,
            session_path=ROOT / ISAAC_FIXED_TELEOP_SESSION,
            runtime_roles={"teleop_consumer"},
        )


def test_profile_owned_workcell_rejects_unconsumed_frames(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    resolved = SessionResolver(ROOT).resolve(ROOT / MUJOCO_TABLE_SESSION)
    extra_frame = WorkcellFrameSpec(
        frame_id="ignored_frame",
        parent=resolved.workcell.world_frame,
        transform=PoseSpec.identity(),
    )
    wrong = replace(
        resolved,
        workcell=replace(
            resolved.workcell,
            frames=(*resolved.workcell.frames, extra_frame),
        ),
    )
    monkeypatch.setattr(
        SessionResolver,
        "resolve",
        lambda self, *args, **kwargs: wrong,
    )

    with pytest.raises(ValueError, match="cannot consume"):
        resolve_mujoco_table_runtime(
            ROOT,
            session_path=ROOT / MUJOCO_TABLE_SESSION,
        )
