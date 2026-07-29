from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from wujihand.adapters.storage import load_tracker_workcell_mapping


ROOT = Path(__file__).parents[2]
PROFILE_V1 = ROOT / "configs/calibrations/vive_tracker_workcell_workstation2_v1.yaml"
PROFILE_V2 = ROOT / "configs/calibrations/vive_tracker_workcell_workstation2_v2.yaml"
PROFILE_V3 = ROOT / "configs/calibrations/vive_tracker_workcell_workstation2_v3.yaml"


def test_workstation2_tracker_mapping_profile_is_proper_and_bounded() -> None:
    mapping = load_tracker_workcell_mapping(PROFILE_V3)

    assert mapping.mapping_id == "vive_tracker_workcell_workstation2_v3"
    assert mapping.tracking_frame == "vive_tracking"
    assert mapping.workcell_frame == "world"
    assert mapping.scope == "simulation_only"
    assert mapping.relative_rotation_semantics == "workcell_spatial_delta"
    matrix = np.asarray(mapping.tracker_to_workcell)
    np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)
    assert np.linalg.det(matrix) == pytest.approx(1.0)
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
    assert mapping.translation_scale == pytest.approx(1.0)
    assert mapping.max_translation_delta_m == pytest.approx(0.4)
    assert np.sqrt(3.0) * mapping.max_translation_delta_m == pytest.approx(
        0.6928203230275509
    )
    assert mapping.max_rotation_delta_deg == pytest.approx(15.0)


def test_workstation2_v2_changes_only_translation_scale_and_revision() -> None:
    previous = load_tracker_workcell_mapping(PROFILE_V1)
    current = load_tracker_workcell_mapping(PROFILE_V2)

    assert current.mapping_id != previous.mapping_id
    assert current.provenance != previous.provenance
    assert previous.translation_scale == pytest.approx(0.25)
    assert current.translation_scale == pytest.approx(1.0)
    assert current.tracker_to_workcell == previous.tracker_to_workcell
    assert current.max_translation_delta_m == previous.max_translation_delta_m
    assert current.rotation_scale == previous.rotation_scale
    assert current.max_rotation_delta_deg == previous.max_rotation_delta_deg


def test_workstation2_v3_changes_only_translation_clamp_and_revision() -> None:
    baseline = load_tracker_workcell_mapping(PROFILE_V2)
    expanded = load_tracker_workcell_mapping(PROFILE_V3)

    assert baseline.mapping_id == "vive_tracker_workcell_workstation2_v2"
    assert baseline.max_translation_delta_m == pytest.approx(0.08)
    assert expanded.mapping_id == "vive_tracker_workcell_workstation2_v3"
    assert expanded.provenance != baseline.provenance
    assert expanded.max_translation_delta_m == pytest.approx(0.4)
    assert expanded.tracker_to_workcell == baseline.tracker_to_workcell
    assert expanded.translation_scale == baseline.translation_scale
    assert expanded.rotation_scale == baseline.rotation_scale
    assert expanded.max_rotation_delta_deg == baseline.max_rotation_delta_deg
    assert expanded.relative_rotation_semantics == baseline.relative_rotation_semantics
    assert expanded.scope == baseline.scope == "simulation_only"


def test_mapping_loader_rejects_unreviewed_scope(tmp_path: Path) -> None:
    payload = PROFILE_V3.read_text(encoding="utf-8").replace(
        "scope: simulation_only",
        "scope: hardware",
    )
    candidate = tmp_path / "mapping.yaml"
    candidate.write_text(payload, encoding="utf-8")

    with pytest.raises(ValueError, match="scope"):
        load_tracker_workcell_mapping(candidate)
