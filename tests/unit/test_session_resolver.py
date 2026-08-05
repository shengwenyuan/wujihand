from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import shutil
from typing import Any

import pytest
import yaml

from wujihand.runtime import SessionResolver


ROOT = Path(__file__).parents[2]


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def _write_yaml(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _copy_project(tmp_path: Path) -> Path:
    root = tmp_path / "project"
    shutil.copytree(ROOT / "configs", root / "configs")
    (root / "third_party").mkdir(parents=True)
    shutil.copy2(
        ROOT / "third_party/sources.lock.yaml",
        root / "third_party/sources.lock.yaml",
    )
    return root


def _dual_hand_project(tmp_path: Path, *, namespace_policy: str) -> Path:
    root = tmp_path / "project"
    shutil.copytree(ROOT / "configs/assets", root / "configs/assets")
    shutil.copytree(ROOT / "configs/profiles", root / "configs/profiles")
    (root / "third_party").mkdir(parents=True)
    shutil.copy2(
        ROOT / "third_party/sources.lock.yaml",
        root / "third_party/sources.lock.yaml",
    )

    source_binding = (
        ROOT
        / "configs/bindings/isaac/"
        "wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
    )
    binding = _load_yaml(source_binding)
    binding["binding_id"] = "wuji_hand2_beta1_right_isaac_namespaced_v1"
    binding["namespace_policy"] = namespace_policy
    binding_path = (
        root
        / "configs/bindings/isaac/"
        "wuji_hand2_beta1_right_namespaced_v1.yaml"
    )
    _write_yaml(binding_path, binding)

    asset_ref = {
        "path": "configs/assets/wuji_hand2_beta1_right_v1.yaml",
        "expected_id": "wuji_hand2_beta1_right",
    }
    assembly = {
        "schema": "wujihand.assembly_spec.v1",
        "assembly_id": "dual_hand_forest_v1",
        "instances": [
            {
                "instance_id": "left_station_hand",
                "asset": asset_ref,
                "role": "left_station",
                "namespace": "left",
            },
            {
                "instance_id": "right_station_hand",
                "asset": asset_ref,
                "role": "right_station",
                "namespace": "right",
            },
        ],
        "roots": ["left_station_hand", "right_station_hand"],
        "attachments": [],
    }
    _write_yaml(root / "configs/assemblies/dual_hand_forest_v1.yaml", assembly)

    identity = {
        "position_m": [0.0, 0.0, 0.0],
        "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    workcell = {
        "schema": "wujihand.workcell.v1",
        "workcell_id": "dual_mount_workcell_v1",
        "world_frame": "world",
        "frames": [],
        "mounts": [
            {"mount_id": "left_mount", "frame": "world", "transform": identity},
            {"mount_id": "right_mount", "frame": "world", "transform": identity},
        ],
        "entities": [],
        "compatibility_profile": None,
    }
    _write_yaml(
        root / "configs/workcells/dual_mount_workcell_v1.yaml", workcell
    )

    binding_ref = {
        "path": (
            "configs/bindings/isaac/"
            "wuji_hand2_beta1_right_namespaced_v1.yaml"
        ),
        "expected_id": "wuji_hand2_beta1_right_isaac_namespaced_v1",
    }
    session = {
        "schema": "wujihand.session.v1",
        "session_id": "dual_hand_forest_session_v1",
        "backend": "isaac",
        "runtime_role": "simulation",
        "assembly": {
            "path": "configs/assemblies/dual_hand_forest_v1.yaml",
            "expected_id": "dual_hand_forest_v1",
        },
        "workcell": {
            "path": "configs/workcells/dual_mount_workcell_v1.yaml",
            "expected_id": "dual_mount_workcell_v1",
        },
        "bindings": {
            "left_station_hand": binding_ref,
            "right_station_hand": binding_ref,
        },
        "placements": {
            "left_station_hand": "left_mount",
            "right_station_hand": "right_mount",
        },
        "runtime": {
            "compatibility_profile": None,
            "transport_contract": None,
            "control_layouts": [
                {
                    "instance_id": instance_id,
                    "group_id": "finger_joints",
                    "layout_id": "wuji_hand2_right_firmware_v1",
                }
                for instance_id in ("left_station_hand", "right_station_hand")
            ],
        },
    }
    _write_yaml(
        root / "configs/sessions/dual_hand_forest_session_v1.yaml", session
    )
    return root


def _passive_sensor_project(tmp_path: Path) -> tuple[Path, Path]:
    root = _copy_project(tmp_path)
    asset_path = root / "configs/assets/d405_sim_right_v1.yaml"
    binding_path = root / "configs/bindings/isaac/d405_sim_right_v1.yaml"
    assembly_path = root / "configs/assemblies/d405_sim_right_v1.yaml"
    workcell_path = root / "configs/workcells/d405_sim_right_v1.yaml"
    session_path = root / "configs/sessions/d405_sim_right_v1.yaml"
    identity = {
        "position_m": [0.0, 0.0, 0.0],
        "quat_wxyz": [1.0, 0.0, 0.0, 0.0],
    }
    artifact = {
        "source": "realsense-ros-d405-description",
        "source_revision": "commit:bafc21080c5c8e259dadbb309797949aee0dd950",
        "path": "realsense2_description/meshes/d405.stl",
    }
    _write_yaml(
        asset_path,
        {
            "schema": "wujihand.asset_manifest.v2",
            "asset_id": "realsense_d405_housing_sim_right",
            "revision": "source_bafc2108_sim_v1",
            "kind": "simulated_sensor",
            "product": "realsense_d405_housing",
            "side": "right",
            "frames": {"rear_mount": "rear_mount", "optical": "optical"},
            "control_groups": [],
            "provenance_source": (
                "third_party/sources.lock.yaml#realsense-ros-d405-description"
            ),
            "canonical_profile": None,
        },
    )
    _write_yaml(
        binding_path,
        {
            "schema": "wujihand.backend_binding.v2",
            "binding_id": "realsense_d405_housing_sim_right_isaac_v1",
            "asset_id": "realsense_d405_housing_sim_right",
            "asset_revision": "source_bafc2108_sim_v1",
            "asset_side": "right",
            "backend": "isaac",
            "namespace_policy": "prefix",
            "loader": "mesh",
            "artifact": artifact,
            "collision_artifact": artifact,
            "resource_trees": [],
            "root": "D405",
            "frame_map": {"rear_mount": "rear_mount", "optical": "optical"},
            "group_bindings": [],
            "builder": None,
            "compatibility_profile": None,
            "sensor_profile": (
                "configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"
            ),
        },
    )
    _write_yaml(
        assembly_path,
        {
            "schema": "wujihand.assembly_spec.v1",
            "assembly_id": "d405_sim_right_v1",
            "instances": [
                {
                    "instance_id": "d405_right",
                    "asset": {
                        "path": "configs/assets/d405_sim_right_v1.yaml",
                        "expected_id": "realsense_d405_housing_sim_right",
                    },
                    "role": "right_wrist_camera",
                    "namespace": "right/wrist_camera",
                }
            ],
            "roots": ["d405_right"],
            "attachments": [],
        },
    )
    _write_yaml(
        workcell_path,
        {
            "schema": "wujihand.workcell.v1",
            "workcell_id": "d405_sim_right_v1",
            "world_frame": "world",
            "frames": [],
            "mounts": [
                {"mount_id": "camera_mount", "frame": "world", "transform": identity}
            ],
            "entities": [],
            "compatibility_profile": None,
        },
    )
    _write_yaml(
        session_path,
        {
            "schema": "wujihand.session.v1",
            "session_id": "d405_sim_right_v1",
            "backend": "isaac",
            "runtime_role": "simulation",
            "assembly": {
                "path": "configs/assemblies/d405_sim_right_v1.yaml",
                "expected_id": "d405_sim_right_v1",
            },
            "workcell": {
                "path": "configs/workcells/d405_sim_right_v1.yaml",
                "expected_id": "d405_sim_right_v1",
            },
            "bindings": {
                "d405_right": {
                    "path": "configs/bindings/isaac/d405_sim_right_v1.yaml",
                    "expected_id": "realsense_d405_housing_sim_right_isaac_v1",
                }
            },
            "placements": {"d405_right": "camera_mount"},
            "runtime": {
                "compatibility_profile": None,
                "transport_contract": None,
                "control_layouts": [],
            },
        },
    )
    return root, session_path


def test_prefix_namespace_policy_resolves_repeated_asset_multi_root_forest(
    tmp_path: Path,
) -> None:
    root = _dual_hand_project(tmp_path, namespace_policy="prefix")

    resolved = SessionResolver(root).resolve(
        "configs/sessions/dual_hand_forest_session_v1.yaml"
    )

    assert resolved.assembly.roots == (
        "left_station_hand",
        "right_station_hand",
    )
    assert {instance.effective_root for instance in resolved.instances} == {
        "left:r_base_link",
        "right:r_base_link",
    }


def test_preserve_namespace_policy_rejects_repeated_backend_root(
    tmp_path: Path,
) -> None:
    root = _dual_hand_project(tmp_path, namespace_policy="preserve")

    with pytest.raises(ValueError, match="backend roots collide"):
        SessionResolver(root).resolve(
            "configs/sessions/dual_hand_forest_session_v1.yaml"
        )


def test_resolver_accepts_v2_simulated_sensor_without_dummy_control_route(
    tmp_path: Path,
) -> None:
    root, session_path = _passive_sensor_project(tmp_path)

    resolved = SessionResolver(root).resolve(session_path)
    camera = resolved.instance("d405_right")

    assert camera.asset.kind == "simulated_sensor"
    assert camera.asset.control_groups == ()
    assert camera.binding.group_bindings == ()
    assert camera.artifact is not None
    assert camera.collision_artifact is not None
    assert dict(resolved.referenced_file_hashes).keys() >= {
        "configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"
    }


def test_resolver_rejects_invalid_camera_profile_at_sensor_boundary(
    tmp_path: Path,
) -> None:
    root, session_path = _passive_sensor_project(tmp_path)
    profile_path = (
        root / "configs/profiles/isaac_d405_synthetic_wide_angle_140_v1.yaml"
    )
    profile = _load_yaml(profile_path)
    profile["simulation_only"] = False
    _write_yaml(profile_path, profile)

    with pytest.raises(ValueError, match="simulation_only must be true"):
        SessionResolver(root).resolve(session_path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("asset_revision", "beta2", "targets asset revision"),
        ("asset_side", "left", "targets asset side"),
    ),
)
def test_resolver_rejects_binding_identity_mismatch(
    tmp_path: Path,
    field: str,
    value: str,
    message: str,
) -> None:
    root = _copy_project(tmp_path)
    binding_path = (
        root
        / "configs/bindings/isaac/"
        "wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
    )
    binding = _load_yaml(binding_path)
    binding[field] = value
    _write_yaml(binding_path, binding)

    with pytest.raises(ValueError, match=message):
        SessionResolver(root).resolve(
            "configs/sessions/isaac_hand2_fixed_preview_v1.yaml"
        )


def test_resolver_rejects_backend_mount_layout_and_transport_mismatches(
    tmp_path: Path,
) -> None:
    root = _copy_project(tmp_path)
    session_path = root / "configs/sessions/isaac_hand2_teleop_v1.yaml"
    original = _load_yaml(session_path)

    backend = deepcopy(original)
    backend["backend"] = "mujoco"
    _write_yaml(session_path, backend)
    with pytest.raises(ValueError, match="does not match session backend"):
        SessionResolver(root).resolve(session_path)

    missing_mount = deepcopy(original)
    missing_mount["placements"]["hand"] = "missing_mount"
    _write_yaml(session_path, missing_mount)
    with pytest.raises(ValueError, match="unknown workcell mounts"):
        SessionResolver(root).resolve(session_path)

    wrong_layout = deepcopy(original)
    wrong_layout["runtime"]["control_layouts"][0]["layout_id"] = "wrong_layout"
    _write_yaml(session_path, wrong_layout)
    with pytest.raises(ValueError, match="asset declares"):
        SessionResolver(root).resolve(session_path)

    missing_transport = deepcopy(original)
    missing_transport["runtime"]["transport_contract"] = None
    _write_yaml(session_path, missing_transport)
    with pytest.raises(ValueError, match="requires a transport contract"):
        SessionResolver(root).resolve(session_path)


def test_resolver_rejects_incomplete_binding_and_invalid_attachment_frame(
    tmp_path: Path,
) -> None:
    root = _copy_project(tmp_path)
    session_path = (
        root / "configs/sessions/mujoco_fr3v2_hand2_right_table_v1.yaml"
    )
    session = _load_yaml(session_path)
    session["bindings"].pop("hand")
    _write_yaml(session_path, session)
    with pytest.raises(ValueError, match="must exactly cover assembly instances"):
        SessionResolver(root).resolve(session_path)

    session = _load_yaml(
        ROOT / "configs/sessions/mujoco_fr3v2_hand2_right_table_v1.yaml"
    )
    _write_yaml(session_path, session)
    assembly_path = (
        root / "configs/assemblies/fr3v2_hand2_right_identity_v1.yaml"
    )
    assembly = _load_yaml(assembly_path)
    assembly["attachments"][0]["child"]["frame"] = "missing_frame"
    _write_yaml(assembly_path, assembly)
    with pytest.raises(ValueError, match="is not declared by asset"):
        SessionResolver(root).resolve(session_path)


def test_resolver_rejects_unpinned_source_revision_and_wrong_dof_count(
    tmp_path: Path,
) -> None:
    root = _copy_project(tmp_path)
    binding_path = (
        root
        / "configs/bindings/isaac/"
        "wuji_hand2_beta1_right_v2026_6_27_v1.yaml"
    )
    session_path = root / "configs/sessions/isaac_hand2_fixed_preview_v1.yaml"
    original = _load_yaml(binding_path)

    wrong_revision = deepcopy(original)
    wrong_revision["artifact"]["source_revision"] = f"commit:{'0' * 40}"
    _write_yaml(binding_path, wrong_revision)
    with pytest.raises(ValueError, match="does not match pinned revision"):
        SessionResolver(root).resolve(session_path)

    wrong_count = deepcopy(original)
    wrong_count["group_bindings"][0]["joints"].pop()
    _write_yaml(binding_path, wrong_count)
    with pytest.raises(ValueError, match="asset requires 20"):
        SessionResolver(root).resolve(session_path)


def test_asset_and_binding_provenance_are_independent_and_both_hashed(
    tmp_path: Path,
) -> None:
    root = _copy_project(tmp_path)
    session_path = root / "configs/sessions/isaac_hand2_fixed_preview_v1.yaml"
    baseline = SessionResolver(root).resolve(session_path)
    asset_path = root / "configs/assets/wuji_hand2_beta1_right_v1.yaml"
    asset = _load_yaml(asset_path)
    asset["provenance_source"] = "third_party/sources.lock.yaml#isaaclab"
    _write_yaml(asset_path, asset)

    resolved = SessionResolver(root).resolve(session_path)

    assert {record.name for record in resolved.source_records} == {
        "isaaclab",
        "wuji-description",
    }
    assert resolved.session_hash != baseline.session_hash
