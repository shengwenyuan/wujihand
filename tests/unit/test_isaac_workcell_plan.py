from __future__ import annotations

import json
from pathlib import Path

import pytest

from wujihand.runtime import (
    ConfigRepository,
    SessionResolver,
    resolve_isaac_workcell_plan,
)


ROOT = Path(__file__).parents[2]


def test_primitive_workcell_compiles_without_a_backend_profile() -> None:
    repository = ConfigRepository(ROOT)
    workcell = repository.load_workcell(
        "configs/workcells/isaac_hand2_table_v1.yaml"
    )

    plan = resolve_isaac_workcell_plan(ROOT, workcell)

    assert plan.profile_id is None
    assert plan.imports == ()
    assert plan.policies.ground == "project"
    assert {operation.entity_id for operation in plan.primitives} == {
        "ground",
        "table",
    }


def test_robolab_workcell_compiles_to_content_identity_and_hybrid_ops() -> None:
    repository = ConfigRepository(ROOT)
    workcell = repository.load_workcell(
        "configs/workcells/isaac_robolab_banana_bowl_dual_station_v1.yaml"
    )

    plan = resolve_isaac_workcell_plan(ROOT, workcell)
    mapping = plan.to_mapping()

    assert plan.profile_id == "isaac_robolab_banana_bowl_v1"
    assert plan.imports[0].pose.position_m == (0.0, 0.0, 0.8)
    assert plan.imports[0].content.relative_path == (
        "assets/scenes/banana_bowl.usda"
    )
    assert plan.lighting.content is not None
    assert plan.lighting.content.relative_path == (
        "assets/backgrounds/indoors/photo_studio_01_2k.hdr"
    )
    assert plan.lighting.intensity == 800.0
    assert plan.lighting.exposure == 0.0
    assert {operation.entity_id for operation in plan.primitives} == {
        "left_base_plinth",
        "right_base_plinth",
    }
    assert plan.primitives[0].pose.position_m[2] == pytest.approx(0.4515)
    encoded = json.dumps(mapping, sort_keys=True)
    assert str(ROOT) not in encoded
    assert "local_runtime_path" not in encoded


def test_each_robolab_layout_is_a_distinct_resolved_session() -> None:
    resolver = SessionResolver(ROOT)
    paths = (
        "configs/sessions/isaac_nero_dual_hand2_robolab_base_empty_v1.yaml",
        "configs/sessions/isaac_nero_dual_hand2_robolab_banana_bowl_v1.yaml",
        "configs/sessions/isaac_nero_dual_hand2_robolab_workdesk_v1.yaml",
    )

    sessions = tuple(resolver.resolve(path) for path in paths)

    assert len({session.session_hash for session in sessions}) == len(paths)
    assert {
        session.workcell.mount("table_near_left").mount_id
        for session in sessions
    } == {"table_near_left"}


def test_banana_bowl_teleop_keeps_the_qualified_mount_baseline() -> None:
    session = SessionResolver(ROOT).resolve(
        "configs/sessions/"
        "isaac_nero_dual_hand2_robolab_banana_bowl_teleop_v1.yaml"
    )
    left = session.workcell.mount("table_near_left").transform.position_m
    right = session.workcell.mount("table_near_right").transform.position_m

    assert session.session.runtime_role == "teleop_consumer"
    assert session.session.runtime.transport_contract == (
        "wujihand.dual_teleoperation.v1"
    )
    assert right[0] - left[0] == pytest.approx(0.64)
    assert left[1:] == right[1:]
