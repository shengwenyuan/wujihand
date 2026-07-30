from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from wujihand.adapters.storage import load_tracker_workcell_mapping


ROOT = Path(__file__).parents[2]
PROFILE = ROOT / "configs/calibrations/vive_tracker_workcell_workstation2.yaml"


def test_workstation2_owns_one_canonical_tracker_mapping() -> None:
    candidates = sorted(
        path.name
        for path in PROFILE.parent.glob("vive_tracker_workcell_workstation2*.yaml")
    )

    assert candidates == [PROFILE.name]


def test_workstation2_tracker_mapping_is_proper_and_bounded() -> None:
    mapping = load_tracker_workcell_mapping(PROFILE)

    assert mapping.mapping_id == "vive_tracker_workcell_workstation2"
    assert mapping.tracking_frame == "vive_tracking"
    assert mapping.workcell_frame == "world"
    assert mapping.scope == "simulation_only"
    assert mapping.relative_rotation_semantics == "workcell_spatial_delta"

    matrix = np.asarray(mapping.tracker_to_workcell)
    np.testing.assert_allclose(
        matrix,
        np.asarray(
            (
                (0.0, 0.0, -1.0),
                (-1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
            )
        ),
    )
    np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)
    assert np.linalg.det(matrix) == pytest.approx(1.0)
    assert mapping.translation_scale == pytest.approx(1.0)
    assert mapping.max_translation_delta_m == pytest.approx(0.4)
    assert np.sqrt(3.0) * mapping.max_translation_delta_m == pytest.approx(
        0.6928203230275509
    )
    assert mapping.rotation_scale == pytest.approx(1.0)
    assert mapping.max_rotation_delta_deg == pytest.approx(15.0)


def test_workstation2_mapping_owns_translation_and_spatial_rotation_signs() -> None:
    matrix = np.asarray(
        load_tracker_workcell_mapping(PROFILE).tracker_to_workcell
    )

    np.testing.assert_allclose(
        matrix @ np.asarray((0.0, 0.0, -1.0)),
        np.asarray((1.0, 0.0, 0.0)),
    )
    np.testing.assert_allclose(
        matrix @ np.asarray((-1.0, 0.0, 0.0)),
        np.asarray((0.0, 1.0, 0.0)),
    )
    np.testing.assert_allclose(
        matrix @ np.asarray((0.0, 1.0, 0.0)),
        np.asarray((0.0, 0.0, 1.0)),
    )

    np.testing.assert_allclose(
        matrix @ np.asarray((0.0, 0.0, 1.0)),
        np.asarray((-1.0, 0.0, 0.0)),
    )
    np.testing.assert_allclose(
        matrix @ np.asarray((1.0, 0.0, 0.0)),
        np.asarray((0.0, -1.0, 0.0)),
    )
    np.testing.assert_allclose(
        matrix @ np.asarray((0.0, 1.0, 0.0)),
        np.asarray((0.0, 0.0, 1.0)),
    )


def test_mapping_loader_rejects_unreviewed_scope(tmp_path: Path) -> None:
    payload = PROFILE.read_text(encoding="utf-8").replace(
        "scope: simulation_only",
        "scope: hardware",
    )
    candidate = tmp_path / "mapping.yaml"
    candidate.write_text(payload, encoding="utf-8")

    with pytest.raises(ValueError, match="scope"):
        load_tracker_workcell_mapping(candidate)
