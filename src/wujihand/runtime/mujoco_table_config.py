"""Strict configuration loader for the FR3 v2 + Hand 2 MuJoCo table scene."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any, Mapping, cast

import numpy as np
import yaml

from wujihand.compat.mujoco_table import (
    ArmMountConfig,
    ArmPedestalConfig,
    CameraConfig,
    FloorConfig,
    HandAttachmentConfig,
    MujocoAssetConfig,
    MujocoControlConfig,
    MujocoPhysicsConfig,
    MujocoTableSceneConfig,
    ObservationLightConfig,
    TableConfig,
)


_SHA256 = re.compile(r"^[0-9a-f]{64}$")


def _require_exact_keys(
    value: object, *, expected: set[str], field: str
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise ValueError(
            f"{field} keys differ from schema: missing={missing}, unexpected={unexpected}"
        )
    return value


def _vector(value: object, *, size: int, field: str) -> tuple[float, ...]:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{field} must be a finite length-{size} vector")
    return tuple(float(item) for item in array)


def _positive(value: object, field: str, *, allow_zero: bool = False) -> float:
    number = float(cast(Any, value))
    invalid = number < 0.0 if allow_zero else number <= 0.0
    if not np.isfinite(number) or invalid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{field} must be finite and {qualifier}")
    return number


def _unit_quaternion(value: object, field: str) -> tuple[float, float, float, float]:
    quat = np.asarray(value, dtype=np.float64)
    if quat.shape != (4,) or not np.isfinite(quat).all():
        raise ValueError(f"{field} must be a finite length-4 quaternion")
    if not np.isclose(np.linalg.norm(quat), 1.0, atol=1e-7):
        raise ValueError(f"{field} must be unit length")
    return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])


def _rgba(value: object, field: str) -> tuple[float, float, float, float]:
    color = cast(tuple[float, float, float, float], _vector(value, size=4, field=field))
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ValueError(f"{field} must be in [0, 1]")
    return color


def _rgb(value: object, field: str) -> tuple[float, float, float]:
    color = cast(tuple[float, float, float], _vector(value, size=3, field=field))
    if any(component < 0.0 or component > 1.0 for component in color):
        raise ValueError(f"{field} must be in [0, 1]")
    return color


def load_mujoco_table_scene_config(path: str | Path) -> MujocoTableSceneConfig:
    """Load a scene profile and reject ambiguous geometry or timing."""

    data: dict[str, Any] = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    robot = data.get("robot", {})
    if data.get("schema_version") != 1 or data.get("backend") != "mujoco":
        raise ValueError("unsupported MuJoCo table scene schema")
    if robot != {
        "arm": "franka_fr3_v2",
        "hand_product": "wuji_hand_2_beta_1",
        "hand_side": "right",
    }:
        raise ValueError("scene must be FR3 v2 with Wuji Hand 2 Beta 1 right")

    assets_data = data["assets"]
    assets = MujocoAssetConfig(
        arm_profile=Path(assets_data["arm_profile"]),
        arm_mjcf=Path(assets_data["arm_mjcf"]),
        arm_mjcf_sha256=str(assets_data["arm_mjcf_sha256"]),
        arm_asset_dir=Path(assets_data["arm_asset_dir"]),
        arm_asset_tree_sha256=str(assets_data["arm_asset_tree_sha256"]),
        hand_profile=Path(assets_data["hand_profile"]),
        hand_mjcf=Path(assets_data["hand_mjcf"]),
        hand_mjcf_sha256=str(assets_data["hand_mjcf_sha256"]),
        hand_asset_dir=Path(assets_data["hand_asset_dir"]),
        hand_asset_tree_sha256=str(assets_data["hand_asset_tree_sha256"]),
    )
    for asset_path in (
        assets.arm_profile,
        assets.arm_mjcf,
        assets.arm_asset_dir,
        assets.hand_profile,
        assets.hand_mjcf,
        assets.hand_asset_dir,
    ):
        if asset_path.is_absolute() or ".." in asset_path.parts:
            raise ValueError("asset paths must be project-relative without '..'")
    for field, digest in (
        ("assets.arm_mjcf_sha256", assets.arm_mjcf_sha256),
        ("assets.arm_asset_tree_sha256", assets.arm_asset_tree_sha256),
        ("assets.hand_mjcf_sha256", assets.hand_mjcf_sha256),
        ("assets.hand_asset_tree_sha256", assets.hand_asset_tree_sha256),
    ):
        if not _SHA256.fullmatch(digest):
            raise ValueError(f"{field} must be a lowercase SHA-256")

    physics_data = data["physics"]
    physics = MujocoPhysicsConfig(
        timestep_s=_positive(physics_data["timestep_s"], "physics.timestep_s"),
        integrator=str(physics_data["integrator"]),
        solver=str(physics_data["solver"]),
        jacobian=str(physics_data["jacobian"]),
        iterations=int(physics_data["iterations"]),
        tolerance=_positive(physics_data["tolerance"], "physics.tolerance"),
        gravity_m_s2=cast(
            tuple[float, float, float],
            _vector(physics_data["gravity_m_s2"], size=3, field="physics.gravity_m_s2"),
        ),
    )
    if physics.integrator not in {"Euler", "RK4", "implicit", "implicitfast"}:
        raise ValueError("unsupported MuJoCo integrator")
    if physics.solver not in {"PGS", "CG", "Newton"}:
        raise ValueError("unsupported MuJoCo solver")
    if physics.jacobian not in {"auto", "dense", "sparse"}:
        raise ValueError("unsupported MuJoCo jacobian mode")
    if physics.iterations <= 0:
        raise ValueError("physics.iterations must be positive")

    control_data = data["control"]
    control = MujocoControlConfig(
        rate_hz=_positive(control_data["rate_hz"], "control.rate_hz"),
        physics_substeps=int(control_data["physics_substeps"]),
    )
    if control.physics_substeps <= 0:
        raise ValueError("control.physics_substeps must be positive")
    if not np.isclose(
        1.0 / control.rate_hz,
        control.physics_substeps * physics.timestep_s,
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError("control period must equal physics_substeps * timestep_s")

    floor_data = data["floor"]
    floor = FloorConfig(
        z_m=float(floor_data["z_m"]),
        color_rgba=_rgba(floor_data["color_rgba"], "floor.color_rgba"),
    )
    if not np.isfinite(floor.z_m):
        raise ValueError("floor.z_m must be finite")

    table_data = data["table"]
    table = TableConfig(
        center_m=cast(
            tuple[float, float, float],
            _vector(table_data["center_m"], size=3, field="table.center_m"),
        ),
        size_m=cast(
            tuple[float, float, float],
            _vector(table_data["size_m"], size=3, field="table.size_m"),
        ),
        leg_width_m=_positive(table_data["leg_width_m"], "table.leg_width_m"),
        leg_edge_inset_m=_positive(
            table_data["leg_edge_inset_m"], "table.leg_edge_inset_m", allow_zero=True
        ),
        color_rgba=_rgba(table_data["color_rgba"], "table.color_rgba"),
        friction=cast(
            tuple[float, float, float],
            _vector(table_data["friction"], size=3, field="table.friction"),
        ),
    )
    if any(size <= 0.0 for size in table.size_m):
        raise ValueError("table.size_m must be positive")
    if table.center_m[2] - table.size_m[2] / 2.0 <= floor.z_m:
        raise ValueError("tabletop must be above the floor")
    if table.leg_width_m + 2.0 * table.leg_edge_inset_m >= min(table.size_m[:2]):
        raise ValueError("table legs and edge inset do not fit the tabletop")
    if any(value < 0.0 for value in table.friction):
        raise ValueError("table.friction must be non-negative")
    if table.size_m[0] <= table.size_m[1]:
        raise ValueError("table X dimension must be longer than Y for a y-edge long side")

    pedestal_data = _require_exact_keys(
        data["arm_pedestal"],
        expected={
            "center_xy_m",
            "height_m",
            "top_size_m",
            "bottom_size_m",
            "adjacent_table_edge",
            "bottom_edge_gap_m",
            "color_rgba",
            "friction",
        },
        field="arm_pedestal",
    )
    pedestal = ArmPedestalConfig(
        center_xy_m=cast(
            tuple[float, float],
            _vector(
                pedestal_data["center_xy_m"],
                size=2,
                field="arm_pedestal.center_xy_m",
            ),
        ),
        height_m=_positive(pedestal_data["height_m"], "arm_pedestal.height_m"),
        top_size_m=cast(
            tuple[float, float],
            _vector(
                pedestal_data["top_size_m"],
                size=2,
                field="arm_pedestal.top_size_m",
            ),
        ),
        bottom_size_m=cast(
            tuple[float, float],
            _vector(
                pedestal_data["bottom_size_m"],
                size=2,
                field="arm_pedestal.bottom_size_m",
            ),
        ),
        adjacent_table_edge=str(pedestal_data["adjacent_table_edge"]),
        bottom_edge_gap_m=_positive(
            pedestal_data["bottom_edge_gap_m"],
            "arm_pedestal.bottom_edge_gap_m",
            allow_zero=True,
        ),
        color_rgba=_rgba(pedestal_data["color_rgba"], "arm_pedestal.color_rgba"),
        friction=cast(
            tuple[float, float, float],
            _vector(pedestal_data["friction"], size=3, field="arm_pedestal.friction"),
        ),
        floor_z_m=floor.z_m,
    )
    if any(size <= 0.0 for size in (*pedestal.top_size_m, *pedestal.bottom_size_m)):
        raise ValueError("arm pedestal top and bottom sizes must be positive")
    if any(
        top >= bottom
        for top, bottom in zip(pedestal.top_size_m, pedestal.bottom_size_m, strict=True)
    ):
        raise ValueError("arm pedestal top must be smaller than its bottom in both axes")
    if any(value < 0.0 for value in pedestal.friction):
        raise ValueError("arm_pedestal.friction must be non-negative")
    if pedestal.adjacent_table_edge != "y_max":
        raise ValueError("version one places the pedestal beside the y_max long edge")
    if not np.isclose(pedestal.center_xy_m[0], table.center_m[0], atol=1e-9):
        raise ValueError("arm pedestal must be centered along the table long side")
    expected_pedestal_y = (
        table.y_max_m + pedestal.bottom_edge_gap_m + pedestal.bottom_size_m[1] / 2.0
    )
    if not np.isclose(pedestal.center_xy_m[1], expected_pedestal_y, atol=1e-9):
        raise ValueError("arm pedestal must lie outside y_max at the configured bottom gap")
    if pedestal.bottom_size_m[0] > table.size_m[0]:
        raise ValueError("arm pedestal bottom must fit within the table long-side span")
    if pedestal.top_z_m >= table.top_z_m:
        raise ValueError("arm pedestal top must be lower than the tabletop")

    mount_data = _require_exact_keys(
        data["arm_mount"],
        expected={
            "position_m",
            "quat_wxyz",
            "forward_axis",
            "joint2_clearance_above_table_m",
        },
        field="arm_mount",
    )
    mount = ArmMountConfig(
        position_m=cast(
            tuple[float, float, float],
            _vector(mount_data["position_m"], size=3, field="arm_mount.position_m"),
        ),
        quat_wxyz=_unit_quaternion(mount_data["quat_wxyz"], "arm_mount.quat_wxyz"),
        forward_axis=str(mount_data["forward_axis"]),
        joint2_clearance_above_table_m=_positive(
            mount_data["joint2_clearance_above_table_m"],
            "arm_mount.joint2_clearance_above_table_m",
        ),
    )
    if mount.forward_axis != "local_+x":
        raise ValueError("version one requires the arm's local +X forward axis")
    if mount.joint2_clearance_above_table_m > 0.10:
        raise ValueError("joint2 clearance must remain a slight height offset of at most 0.10 m")
    expected_mount_position = (*pedestal.center_xy_m, pedestal.top_z_m)
    if not np.allclose(mount.position_m, expected_mount_position, atol=1e-9):
        raise ValueError("arm base must be centered on the pedestal top")
    w, x, y, z = mount.quat_wxyz
    local_x_in_world = np.asarray(
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ],
        dtype=np.float64,
    )
    local_z_in_world = np.asarray(
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
        dtype=np.float64,
    )
    if not np.allclose(local_z_in_world, (0.0, 0.0, 1.0), rtol=0.0, atol=1e-9):
        raise ValueError("arm local +Z must remain upright on the horizontal pedestal")
    direction_to_center = np.asarray(table.center_m) - np.asarray(mount.position_m)
    direction_to_center[2] = 0.0
    center_distance = float(np.linalg.norm(direction_to_center))
    if center_distance < 1e-9:
        raise ValueError("arm mount must not coincide with the table center")
    direction_to_center /= center_distance
    if not np.allclose(local_x_in_world, direction_to_center, atol=1e-9):
        raise ValueError("arm local +X must face the table center")

    attachment_data = data["hand_attachment"]
    attachment = HandAttachmentConfig(
        parent_body=str(attachment_data["parent_body"]),
        child_body=str(attachment_data["child_body"]),
        position_m=cast(
            tuple[float, float, float],
            _vector(
                attachment_data["position_m"], size=3, field="hand_attachment.position_m"
            ),
        ),
        quat_wxyz=_unit_quaternion(
            attachment_data["quat_wxyz"], "hand_attachment.quat_wxyz"
        ),
        assumption=str(attachment_data["assumption"]),
    )
    if attachment.parent_body != "fr3v2_link8" or attachment.child_body != "r_base_link":
        raise ValueError("version one requires fr3v2_link8 -> r_base_link")
    if attachment.assumption != "identity_until_physical_adapter_transform_is_measured":
        raise ValueError("attachment transform assumption must remain explicit")
    if not np.allclose(attachment.position_m, (0.0, 0.0, 0.0), atol=1e-12) or not np.allclose(
        np.abs(np.asarray(attachment.quat_wxyz)), (1.0, 0.0, 0.0, 0.0), atol=1e-12
    ):
        raise ValueError("identity attachment assumption requires an identity transform")

    workspace_center = cast(
        tuple[float, float, float],
        _vector(data["workspace"]["center_m"], size=3, field="workspace.center_m"),
    )
    workspace_from_mount = np.asarray(workspace_center) - np.asarray(mount.position_m)
    if float(np.dot(workspace_from_mount, local_x_in_world)) <= 0.0:
        raise ValueError("workspace center must be in front of the pedestal-mounted arm")
    if workspace_center[2] < table.top_z_m:
        raise ValueError("workspace center must not lie below the tabletop")
    if not (
        table.x_min_m <= workspace_center[0] <= table.x_max_m
        and table.y_min_m <= workspace_center[1] <= table.y_max_m
    ):
        raise ValueError("workspace center must lie over the tabletop footprint")

    camera_data = data["camera"]
    camera = CameraConfig(
        eye_m=cast(
            tuple[float, float, float],
            _vector(camera_data["eye_m"], size=3, field="camera.eye_m"),
        ),
        target_m=cast(
            tuple[float, float, float],
            _vector(camera_data["target_m"], size=3, field="camera.target_m"),
        ),
        fovy_deg=_positive(camera_data["fovy_deg"], "camera.fovy_deg"),
        width_px=int(camera_data["width_px"]),
        height_px=int(camera_data["height_px"]),
    )
    if np.allclose(camera.eye_m, camera.target_m) or not 1.0 <= camera.fovy_deg < 179.0:
        raise ValueError("camera must have a distinct target and valid field of view")
    if camera.width_px <= 0 or camera.height_px <= 0:
        raise ValueError("camera pixel dimensions must be positive")

    light_data = _require_exact_keys(
        data["observation_light"],
        expected={
            "direction",
            "ambient_rgb",
            "diffuse_rgb",
            "specular_rgb",
            "cast_shadow",
        },
        field="observation_light",
    )
    direction = np.asarray(
        _vector(
            light_data["direction"], size=3, field="observation_light.direction"
        ),
        dtype=np.float64,
    )
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm < 1e-9:
        raise ValueError("observation_light.direction must be non-zero")
    if direction[2] >= 0.0:
        raise ValueError("observation_light.direction must point downward")
    cast_shadow = light_data["cast_shadow"]
    if not isinstance(cast_shadow, bool):
        raise ValueError("observation_light.cast_shadow must be boolean")
    if cast_shadow:
        raise ValueError("observation light must remain shadow-free")
    ambient_rgb = _rgb(light_data["ambient_rgb"], "observation_light.ambient_rgb")
    diffuse_rgb = _rgb(light_data["diffuse_rgb"], "observation_light.diffuse_rgb")
    if not any((*ambient_rgb, *diffuse_rgb)):
        raise ValueError("observation light ambient/diffuse intensity must be non-zero")
    observation_light = ObservationLightConfig(
        direction=cast(
            tuple[float, float, float], tuple(float(value) for value in direction / direction_norm)
        ),
        ambient_rgb=ambient_rgb,
        diffuse_rgb=diffuse_rgb,
        specular_rgb=_rgb(light_data["specular_rgb"], "observation_light.specular_rgb"),
        cast_shadow=cast_shadow,
    )

    return MujocoTableSceneConfig(
        name=str(data["name"]),
        assets=assets,
        physics=physics,
        control=control,
        floor=floor,
        table=table,
        arm_pedestal=pedestal,
        arm_mount=mount,
        hand_attachment=attachment,
        workspace_center_m=workspace_center,
        camera=camera,
        observation_light=observation_light,
        provenance={key: str(value) for key, value in data["derived_from"].items()},
    )


__all__ = [
    "ArmPedestalConfig",
    "ArmMountConfig",
    "CameraConfig",
    "FloorConfig",
    "HandAttachmentConfig",
    "MujocoAssetConfig",
    "MujocoControlConfig",
    "MujocoPhysicsConfig",
    "MujocoTableSceneConfig",
    "ObservationLightConfig",
    "TableConfig",
    "load_mujoco_table_scene_config",
]
