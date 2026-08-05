from pathlib import Path

from wujihand.runtime.isaac_d405_wrist_rig_inspection import (
    preflight_d405_wrist_rig_inspection,
)


ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / "tools/run_isaac_nero_hand2_realsense_d405_mount_inspection.py"


def test_static_inspector_preflight_owns_the_complete_dual_rig_contract() -> None:
    plan = preflight_d405_wrist_rig_inspection(
        project_root=ROOT,
        session_path=(
            "configs/sessions/"
            "isaac_nero_dual_hand2_d405_wrist_rig_physical_simulation_nominal_v1.yaml"
        ),
        qualification_profile_path=(
            "configs/profiles/isaac_nero_hand2_self_collision_qualification_v1.yaml"
        ),
        verify_artifacts=False,
    )

    assert plan.resolved.session.runtime.transport_contract is None
    assert tuple(rig.side for rig in plan.wrist_rigs) == ("left", "right")
    assert plan.self_collision_qualification.physics_hz == 120
    assert all(rig.camera_profile.simulation_only for rig in plan.wrist_rigs)
    assert all(rig.camera_profile.optics.horizontal_fov_deg == 140.0 for rig in plan.wrist_rigs)


def test_static_inspector_preserves_articulation_views_during_exploded_capture() -> None:
    source = TOOL.read_text(encoding="utf-8")

    assert "RemovePrim" not in source
    assert "MakeInvisible" in source
    assert "traceback.print_exc" in source
