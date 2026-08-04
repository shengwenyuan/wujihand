from __future__ import annotations

import pytest

from wujihand.adapters.simulation.d405_wrist_rig_assets import (
    Triangle,
    audit_collision_proxy,
    audit_mesh,
    determinant,
    optical_frame_contract,
    rotation_matrix_z_y,
    transform_triangles,
)


TETRAHEDRON: tuple[Triangle, ...] = (
    ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
)


def test_mesh_audit_cross_checks_watertight_body_and_face_components() -> None:
    audit = audit_mesh(TETRAHEDRON)

    assert audit.body_count == 1
    assert audit.shared_edge_component_count == 1
    assert audit.watertight
    assert audit.winding_consistent
    assert audit.degenerate_triangle_count == 0

    translated = transform_triangles(
        TETRAHEDRON,
        rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        translation_mm=(3.0, 0.0, 0.0),
    )
    disconnected = audit_mesh((*TETRAHEDRON, *translated))
    assert disconnected.body_count == 2
    assert disconnected.shared_edge_component_count == 2


def test_optical_mirror_is_proper_and_preserves_the_lateral_offset() -> None:
    contract = optical_frame_contract(
        body_rotation=rotation_matrix_z_y(-55.0, 58.0),
        body_translation_mm=(-55.0, 90.0, 30.0),
        optical_origin_from_rear_mm=(0.0, 9.0, 24.0),
    )
    right = contract["right"]
    left = contract["left"]

    assert right["body_translation_in_hand_mm"] == pytest.approx((-55.0, 90.0, 30.0))
    assert left["body_translation_in_hand_mm"] == pytest.approx((-55.0, -90.0, 30.0))
    assert right["rear_mount_to_optical"]["translation_mm"] == pytest.approx(
        (0.0, 9.0, 24.0)
    )
    assert left["rear_mount_to_optical"]["translation_mm"] == pytest.approx(
        (0.0, -9.0, 24.0)
    )
    assert determinant(tuple(tuple(row) for row in right["optical_rotation_in_hand"])) == (
        pytest.approx(1.0)
    )
    assert determinant(tuple(tuple(row) for row in left["optical_rotation_in_hand"])) == (
        pytest.approx(1.0)
    )


def test_collision_proxy_coverage_and_clearance_are_independent() -> None:
    proxy = {
        "primitives": [
            {
                "type": "box",
                "center_mm": [0.5, 0.5, 0.5],
                "size_mm": [2.0, 2.0, 2.0],
            }
        ],
        "required_clear_points_mm": [[4.0, 4.0, 4.0]],
    }

    audit = audit_collision_proxy(TETRAHEDRON, proxy)

    assert audit.covered_vertex_fraction == 1.0
    assert audit.clear_points_preserved
