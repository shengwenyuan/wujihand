from __future__ import annotations

from pathlib import Path

import pytest

from wujihand.adapters.simulation import (
    load_nero_dual_tabletop_qualification_profile,
    load_nero_link_geometry_alignment,
)
from wujihand.adapters.simulation.nero_model import load_nero_model_profile
from wujihand.runtime import SessionResolver


ROOT = Path(__file__).parents[2]
SESSION = (
    ROOT
    / "configs/sessions/"
    "isaac_nero_dual_hand2_physical_simulation_nominal_v1.yaml"
)


def _resolve():
    return SessionResolver(ROOT).resolve(SESSION)


def test_physical_dual_session_closes_four_instances_and_two_roots() -> None:
    resolved = _resolve()

    assert resolved.session.backend == "isaac"
    assert resolved.session.runtime_role == "simulation"
    assert resolved.session.runtime.transport_contract is None
    assert {instance.instance_id for instance in resolved.instances} == {
        "nero_left",
        "hand_left",
        "nero_right",
        "hand_right",
    }
    assert set(dict(resolved.session.bindings)) == {
        "nero_left",
        "hand_left",
        "nero_right",
        "hand_right",
    }
    assert resolved.assembly.roots == ("nero_left", "nero_right")
    assert dict(resolved.session.placements) == {
        "nero_left": "nero_left_simulation_nominal_mount",
        "nero_right": "nero_right_simulation_nominal_mount",
    }
    assert resolved.session.runtime.compatibility_profile == (
        "configs/profiles/isaac_nero_dual_tabletop_qualification_v1.yaml"
    )


def test_physical_dual_session_routes_exactly_two_q7_and_two_q20_groups() -> None:
    resolved = _resolve()
    expected = {
        ("nero_left", "arm_joints"): ("agilex_nero_q7_v1", 7),
        ("hand_left", "finger_joints"): (
            "wuji_hand2_left_firmware_v1",
            20,
        ),
        ("nero_right", "arm_joints"): ("agilex_nero_q7_v1", 7),
        ("hand_right", "finger_joints"): (
            "wuji_hand2_right_firmware_v1",
            20,
        ),
    }
    routes = {
        (route.instance_id, route.group_id): (
            route.layout_id,
            resolved.instance(route.instance_id)
            .asset.control_group(route.group_id)
            .dof_count,
        )
        for route in resolved.session.runtime.control_layouts
    }

    assert routes == expected
    assert sum(dof_count for _, dof_count in routes.values()) == 54
    assert sorted(dof_count for _, dof_count in routes.values()) == [7, 7, 20, 20]
    for (instance_id, group_id), (_, dof_count) in routes.items():
        group = resolved.instance(instance_id).binding.group_binding(group_id)
        assert len(group.joints) == dof_count
        assert group.actuators == ()


def test_physical_dual_session_owns_per_instance_tabletop_q7_and_isaac_gains() -> None:
    resolved = _resolve()
    profile_path = resolved.session.runtime.compatibility_profile
    assert profile_path is not None
    tabletop = load_nero_dual_tabletop_qualification_profile(
        ROOT / profile_path
    )

    expected_degrees = {
        "nero_left": [-10.0, 45.0, 0.0, 45.0, 90.0, 0.0, 0.0],
        "nero_right": [10.0, 45.0, 0.0, 45.0, 90.0, 0.0, 0.0],
    }
    for instance_id, degrees in expected_degrees.items():
        instance = resolved.instance(instance_id)
        group = instance.asset.control_group("arm_joints")
        assert group.joint_profile is not None
        model = load_nero_model_profile(ROOT / group.joint_profile)
        q7 = tabletop.initial_position(
            instance_id,
            group.group_id,
            group.layout_id,
        )

        assert q7 == pytest.approx(
            [value * 3.141592653589793 / 180.0 for value in degrees]
        )
        assert model.layout.validate_vector(q7) == pytest.approx(q7)

    assert tabletop.arm_drive_gains.stiffness == pytest.approx(6500.0)
    assert tabletop.arm_drive_gains.damping == pytest.approx(220.79402165819616)
    assert (
        "arm_drive_gains_are_isaac_qualification_values_not_hardware_controller_facts"
        in tabletop.assumptions
    )


def test_physical_dual_session_uses_pinned_physical_usd_bindings() -> None:
    resolved = _resolve()
    nero_left = resolved.instance("nero_left")
    nero_right = resolved.instance("nero_right")
    hand_left = resolved.instance("hand_left")
    hand_right = resolved.instance("hand_right")

    assert nero_left.binding_path == nero_right.binding_path
    assert nero_left.binding.namespace_policy == "prefix"
    assert nero_right.binding.namespace_policy == "prefix"
    assert nero_left.artifact is not None
    assert nero_right.artifact is not None
    assert nero_left.artifact.relative_path == nero_right.artifact.relative_path
    assert (
        nero_left.binding.artifact is not None
        and nero_left.binding.artifact.source_revision
        == "sha256:07ba62ca6d7ab79cb76a2148e76743cda78671e1bfa40ad418158554179214a0"
    )
    assert nero_left.binding.compatibility_profile == (
        nero_right.binding.compatibility_profile
    ) == "configs/profiles/agilex_nero_7f_link6_geometry_alignment_v1.yaml"
    alignment = load_nero_link_geometry_alignment(
        ROOT / nero_left.binding.compatibility_profile
    )
    assert alignment.link_name == "link6"
    assert alignment.source_cylinder_axis_local_xyz == (0.0, 1.0, 0.0)
    assert alignment.corrected_cylinder_axis_local_xyz == (1.0, 0.0, 0.0)

    for side, hand in (("left", hand_left), ("right", hand_right)):
        assert hand.binding.namespace_policy == "prefix"
        assert hand.binding.asset_side == side
        assert hand.binding.binding_id.endswith("_physical")
        assert hand.artifact is not None
        assert hand.artifact.relative_path == (
            f"hand2_beta/body/usd/{side}/wujihand.usd"
        )
        assert hand.binding.artifact is not None
        assert hand.binding.artifact.source_revision == (
            "commit:aee64892ebcf8e3237bedc30231bb09476cbc71d"
        )
        assert tuple(tree.relative_path for tree in hand.resource_trees) == (
            f"hand2_beta/body/usd/{side}",
            f"hand2_beta/body/meshes/{side}",
        )
        assert all(len(tree.expected_sha256) == 64 for tree in hand.resource_trees)

    assert {record.name for record in resolved.source_records} == {
        "agilex-agx-arm-urdf",
        "agilex-nero-isaac-6-0-1",
        "wuji-description-v2026-6-27",
    }


def test_physical_dual_namespaces_and_backend_symbols_are_isolated() -> None:
    resolved = _resolve()

    assert {instance.namespace for instance in resolved.instances} == {
        "nero_left",
        "hand2_left",
        "nero_right",
        "hand2_right",
    }
    roots = [instance.effective_root for instance in resolved.instances]
    frames_by_instance = {
        instance.instance_id: {
            instance.qualify_backend_name(name)
            for _, name in instance.binding.frame_map
        }
        for instance in resolved.instances
    }
    joints = [
        instance.qualify_backend_name(name)
        for instance in resolved.instances
        for group in instance.binding.group_bindings
        for name in group.joints
    ]
    assert len(roots) == len(set(roots)) == 4
    assert sum(len(names) for names in frames_by_instance.values()) == len(
        set().union(*frames_by_instance.values())
    )
    assert len(joints) == len(set(joints)) == 54


def test_physical_dual_attachments_and_nominal_workcell_are_explicit() -> None:
    resolved = _resolve()
    attachments = {
        attachment.attachment_id: attachment
        for attachment in resolved.assembly.attachments
    }

    assert set(attachments) == {
        "nero_left_link7_to_hand2_left_base",
        "nero_right_link7_to_hand2_right_base",
    }
    for side in ("left", "right"):
        attachment = attachments[f"nero_{side}_link7_to_hand2_{side}_base"]
        assert (attachment.parent.instance, attachment.parent.frame) == (
            f"nero_{side}",
            "link7",
        )
        assert (attachment.child.instance, attachment.child.frame) == (
            f"hand_{side}",
            "hand_base",
        )
        assert attachment.transform.position_m == (0.023, 0.0, -0.0235)
        assert attachment.transform.quat_wxyz == pytest.approx(
            (2.0**-0.5, 0.0, 2.0**-0.5, 0.0)
        )
        assert attachment.assumption == (
            "mesh_nominal_mount_couples_hand_base_face_to_aligned_link6_positive_x_face"
        )

    assert "simulation_nominal" in resolved.workcell.workcell_id
    table = next(
        entity
        for entity in resolved.workcell.entities
        if entity.entity_id == "simulation_nominal_table"
    )
    assert table.primitive.kind == "box"
    assert table.primitive.size_m == (1.2, 1.2, 0.08)
    table_top = next(
        frame
        for frame in resolved.workcell.frames
        if frame.frame_id == "simulation_nominal_table_top"
    )
    assert table_top.transform.position_m == (0.0, 0.0, 0.8)
    frames = {frame.frame_id: frame for frame in resolved.workcell.frames}
    assert frames["simulation_nominal_camera_oblique_eye"].transform.position_m == (
        1.15,
        -1.55,
        1.45,
    )
    assert frames[
        "simulation_nominal_camera_oblique_target"
    ].transform.position_m == (0.0, -0.03, 1.02)
    assert frames["simulation_nominal_camera_top_eye"].transform.position_m == (
        0.0,
        -0.10,
        2.25,
    )
    assert frames["simulation_nominal_camera_top_target"].transform.position_m == (
        0.0,
        -0.05,
        0.80,
    )
    assert frames[
        "simulation_nominal_camera_right_interface_eye"
    ].transform.position_m == (0.72, -0.035, 1.25)
    assert frames[
        "simulation_nominal_camera_right_interface_target"
    ].transform.position_m == (0.234, -0.035, 1.174)
    left = resolved.workcell.mount("nero_left_simulation_nominal_mount")
    right = resolved.workcell.mount("nero_right_simulation_nominal_mount")
    assert left.frame == right.frame == "simulation_nominal_table_top"
    assert left.transform.position_m == (-0.32, -0.52, 0.0)
    assert right.transform.position_m == (0.32, -0.52, 0.0)
    for mount in (left, right):
        assert mount.transform.quat_wxyz == pytest.approx(
            (2.0**-0.5, 0.0, 0.0, -(2.0**-0.5))
        )
    assert right.transform.position_m[0] - left.transform.position_m[0] == 0.64
