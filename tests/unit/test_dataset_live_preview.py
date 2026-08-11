from __future__ import annotations

import json
from pathlib import Path
import numpy as np
import pytest

from tools.run_isaac_dataset_live_preview import _ancestry_tiers
from tools.run_isaac_dataset_live_preview import _component_path_inventory
from tools.run_isaac_dataset_live_preview import _expected_hand_source_pose_count
from tools.run_isaac_dataset_live_preview import _pose_group_delta
from tools.run_isaac_dataset_live_preview import _q54_group_ranges
from tools.run_isaac_dataset_live_preview import _renderable_geometry_bindings
from tools.run_isaac_dataset_live_preview import _viewport_pixel_difference
from tools.validate_mini_dataset_release import _live_preview_gate


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + "\n", encoding="utf-8")


def test_preview_pose_replay_orders_ancestors_before_descendants() -> None:
    assert _ancestry_tiers(
        (
            "/World/Robot/Arm/Link",
            "/World/Object",
            "/World/Robot",
            "/World/Robot/Arm",
        )
    ) == ((1, 2), (3,), (0,))


def test_preview_renderable_inventory_traverses_instance_proxy_gprims() -> None:
    class Prim:
        def __init__(self, path: str, *, parent: "Prim | None" = None, gprim: bool = False):
            self.path = path
            self.parent = parent
            self.gprim = gprim

        def IsA(self, kind: object) -> bool:
            del kind
            return self.gprim

        def GetPath(self) -> str:
            return self.path

        def GetParent(self) -> "Prim":
            assert self.parent is not None
            return self.parent

        def IsValid(self) -> bool:
            return True

        def IsPseudoRoot(self) -> bool:
            return self.path == "/"

    root = Prim("/")
    owner = Prim("/World/Robots/NeroLeft/link1", parent=root)
    instance = Prim("/World/Robots/NeroLeft/link1/link1", parent=owner)
    proxy_gprim = Prim(
        "/World/Robots/NeroLeft/link1/link1/mesh",
        parent=instance,
        gprim=True,
    )

    class Stage:
        proxy_prims = (owner, instance, proxy_gprim)

    class PrimRange:
        @staticmethod
        def Stage(stage: Stage, predicate: object) -> tuple[Prim, ...]:
            assert predicate == "instance-proxies"
            return stage.proxy_prims

    class Usd:
        @staticmethod
        def TraverseInstanceProxies() -> str:
            return "instance-proxies"

    Usd.PrimRange = PrimRange

    class Imageable:
        def __init__(self, prim: Prim):
            self.prim = prim

        def ComputeVisibility(self) -> str:
            return "inherited"

        def ComputePurpose(self) -> str:
            return "render"

    class UsdGeom:
        Gprim = object()

    UsdGeom.Imageable = Imageable

    assert _renderable_geometry_bindings(
        stage=Stage(),
        pose_paths=(owner.path,),
        usd=Usd,
        usd_geom=UsdGeom,
    ) == ((owner.path, proxy_gprim.path),)


def test_preview_component_inventory_requires_all_nero_arm_links() -> None:
    prefixes = {
        "left_arm": "/World/Robots/NeroLeft",
        "left_hand": "/World/Robots/Hand2Left",
        "right_arm": "/World/Robots/NeroRight",
        "right_hand": "/World/Robots/Hand2Right",
    }
    source = tuple(
        path
        for component, count in (
            ("left_arm", 8),
            ("left_hand", 27),
            ("right_arm", 8),
            ("right_hand", 27),
        )
        for path in (f"{prefixes[component]}/link{index}" for index in range(count))
    )
    replay = tuple(
        path
        for path in source
        if "/Nero" in path or path.endswith("link0")
    )

    source_by_component, replay_by_component = _component_path_inventory(
        pose_paths=source,
        replay_paths=replay,
        component_prefixes=prefixes,
        expected_source_pose_counts={
            "left_arm": 8,
            "left_hand": 27,
            "right_arm": 8,
            "right_hand": 27,
        },
    )

    assert len(source_by_component["left_arm"]) == 8
    assert len(replay_by_component["right_arm"]) == 8
    assert len(replay_by_component["left_hand"]) == 1

    with pytest.raises(RuntimeError, match="left_arm.*incomplete"):
        _component_path_inventory(
            pose_paths=source,
            replay_paths=tuple(path for path in replay if path != source_by_component["left_arm"][0]),
            component_prefixes=prefixes,
            expected_source_pose_counts={
                "left_arm": 8,
                "left_hand": 27,
                "right_arm": 8,
                "right_hand": 27,
            },
        )


def test_preview_hand_pose_count_is_fail_closed_by_description_revision() -> None:
    assert _expected_hand_source_pose_count(
        asset_revision="beta1_description_v2026_6_27",
        side="left",
        backend_base_frame="l_base_link",
    ) == 27
    assert _expected_hand_source_pose_count(
        asset_revision="beta1_description_v2026_8_3",
        side="right",
        backend_base_frame="r_wrist",
    ) == 26
    with pytest.raises(ValueError, match="not qualified"):
        _expected_hand_source_pose_count(
            asset_revision="beta1_description_future",
            side="right",
            backend_base_frame="r_wrist",
        )


def test_preview_component_pose_delta_is_quaternion_sign_invariant() -> None:
    reference = (
        np.asarray([[0.0, 0.0, 0.0]], dtype=np.float64),
        np.asarray([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
    )
    signed_same = (
        np.asarray([[0.0, 0.0, 0.0]], dtype=np.float64),
        np.asarray([[-1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
    )
    assert _pose_group_delta(reference, signed_same) == (0.0, 0.0)

    moved = (
        np.asarray([[0.1, 0.0, 0.0]], dtype=np.float64),
        np.asarray([[np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]], dtype=np.float64),
    )
    position_delta, orientation_delta = _pose_group_delta(reference, moved)
    assert position_delta == pytest.approx(0.1)
    assert orientation_delta == pytest.approx(np.pi / 2.0)


def test_preview_visible_motion_uses_material_pixel_and_q54_groups() -> None:
    baseline = np.zeros((4, 5, 3), dtype=np.uint8)
    motion = baseline.copy()
    motion[1:3, 2:4] = 16

    mean_delta, max_delta, changed_fraction = _viewport_pixel_difference(baseline, motion)

    assert mean_delta == 3.2
    assert max_delta == 16
    assert changed_fraction == 0.2

    minimum = np.zeros(54, dtype=np.float64)
    maximum = minimum.copy()
    maximum[3] = 0.3
    maximum[15] = 0.4
    maximum[32] = 0.5
    maximum[51] = 0.6
    assert _q54_group_ranges(minimum, maximum) == {
        "left_arm_q7": 0.3,
        "left_hand_q20": 0.4,
        "right_arm_q7": 0.5,
        "right_hand_q20": 0.6,
    }


def test_external_live_preview_gate_passes_only_closed_passive_receipt(
    tmp_path: Path,
) -> None:
    run = tmp_path / "episode-001"
    _write_json(
        run / "manifest.json",
        {
            "simulation_timing": {
                "external_gui_preview_required": True,
                "external_gui_preview_hz": 20,
            }
        },
    )
    receipt_path = run / "derived" / "live_preview" / "receipt.json"
    receipt = {
        "schema": "wujihand.dataset_live_preview_receipt.v1",
        "run_id": "episode-001",
        "passed": True,
        "configured_render_hz": 20,
        "effective_render_hz": 20.01,
        "missed_render_periods": 0,
        "control_authority": False,
        "recorded_to_mcap": False,
    }
    _write_json(receipt_path, receipt)

    assert _live_preview_gate(run, run_id=run.name, expected_hz=20).passed

    receipt["missed_render_periods"] = 1
    _write_json(receipt_path, receipt)
    advisory = _live_preview_gate(run, run_id=run.name, expected_hz=20)
    assert advisory.passed
    assert advisory.severity == "advisory"

    receipt["effective_render_hz"] = 10.0
    _write_json(receipt_path, receipt)
    failed = _live_preview_gate(run, run_id=run.name, expected_hz=20)
    assert not failed.passed
    assert failed.severity == "advisory"
    assert failed.reason == "live_preview_missing_or_failed"


def test_external_live_preview_gate_fails_when_receipt_is_missing(
    tmp_path: Path,
) -> None:
    run = tmp_path / "episode-002"
    _write_json(
        run / "manifest.json",
        {
            "simulation_timing": {
                "external_gui_preview_required": True,
                "external_gui_preview_hz": 20,
            }
        },
    )

    gate = _live_preview_gate(run, run_id=run.name, expected_hz=20)

    assert not gate.passed
    assert "error" in gate.observed
