"""Pure data types for the legacy MuJoCo table compatibility leaf.

This temporary, standard-library-only contract lets runtime loading and the
MuJoCo adapter share values without either placing backend details in the
five-layer specs or making the adapter depend on runtime composition.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


@dataclass(frozen=True, slots=True)
class MujocoAssetConfig:
    arm_profile: Path
    arm_mjcf: Path
    arm_mjcf_sha256: str
    arm_asset_dir: Path
    arm_asset_tree_sha256: str
    hand_profile: Path
    hand_mjcf: Path
    hand_mjcf_sha256: str
    hand_asset_dir: Path
    hand_asset_tree_sha256: str


@dataclass(frozen=True, slots=True)
class MujocoPhysicsConfig:
    timestep_s: float
    integrator: str
    solver: str
    jacobian: str
    iterations: int
    tolerance: float
    gravity_m_s2: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class MujocoControlConfig:
    rate_hz: float
    physics_substeps: int


@dataclass(frozen=True, slots=True)
class FloorConfig:
    z_m: float
    color_rgba: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class TableConfig:
    center_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    leg_width_m: float
    leg_edge_inset_m: float
    color_rgba: tuple[float, float, float, float]
    friction: tuple[float, float, float]

    @property
    def top_z_m(self) -> float:
        return self.center_m[2] + self.size_m[2] / 2.0

    @property
    def x_min_m(self) -> float:
        return self.center_m[0] - self.size_m[0] / 2.0

    @property
    def x_max_m(self) -> float:
        return self.center_m[0] + self.size_m[0] / 2.0

    @property
    def y_min_m(self) -> float:
        return self.center_m[1] - self.size_m[1] / 2.0

    @property
    def y_max_m(self) -> float:
        return self.center_m[1] + self.size_m[1] / 2.0


@dataclass(frozen=True, slots=True)
class ArmPedestalConfig:
    center_xy_m: tuple[float, float]
    height_m: float
    top_size_m: tuple[float, float]
    bottom_size_m: tuple[float, float]
    adjacent_table_edge: str
    bottom_edge_gap_m: float
    color_rgba: tuple[float, float, float, float]
    friction: tuple[float, float, float]
    floor_z_m: float

    @property
    def center_m(self) -> tuple[float, float, float]:
        return (
            self.center_xy_m[0],
            self.center_xy_m[1],
            self.floor_z_m + self.height_m / 2.0,
        )

    @property
    def top_z_m(self) -> float:
        return self.floor_z_m + self.height_m


@dataclass(frozen=True, slots=True)
class ArmMountConfig:
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    forward_axis: str
    joint2_clearance_above_table_m: float


@dataclass(frozen=True, slots=True)
class HandAttachmentConfig:
    parent_body: str
    child_body: str
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    assumption: str


@dataclass(frozen=True, slots=True)
class CameraConfig:
    eye_m: tuple[float, float, float]
    target_m: tuple[float, float, float]
    fovy_deg: float
    width_px: int
    height_px: int


@dataclass(frozen=True, slots=True)
class ObservationLightConfig:
    direction: tuple[float, float, float]
    ambient_rgb: tuple[float, float, float]
    diffuse_rgb: tuple[float, float, float]
    specular_rgb: tuple[float, float, float]
    cast_shadow: bool


@dataclass(frozen=True, slots=True)
class MujocoTableSceneConfig:
    name: str
    assets: MujocoAssetConfig
    physics: MujocoPhysicsConfig
    control: MujocoControlConfig
    floor: FloorConfig
    table: TableConfig
    arm_pedestal: ArmPedestalConfig
    arm_mount: ArmMountConfig
    hand_attachment: HandAttachmentConfig
    workspace_center_m: tuple[float, float, float]
    camera: CameraConfig
    observation_light: ObservationLightConfig
    provenance: Mapping[str, str]


__all__ = [
    "ArmMountConfig",
    "ArmPedestalConfig",
    "CameraConfig",
    "FloorConfig",
    "HandAttachmentConfig",
    "MujocoAssetConfig",
    "MujocoControlConfig",
    "MujocoPhysicsConfig",
    "MujocoTableSceneConfig",
    "ObservationLightConfig",
    "TableConfig",
]
