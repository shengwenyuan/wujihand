"""Dynamic ball and contact helpers for the Hand 2 grasp scene.

Isaac classes are imported only inside authoring functions.  Configuration and
contact-force classification therefore remain available to the normal Python
test environment without importing ``isaacsim`` or ``pxr``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Sequence

import numpy as np
import numpy.typing as npt


FINGER_CONTACT_GROUPS = ("thumb", "index", "middle", "ring", "pinky")
TABLE_CONTACT_GROUP = "table"


def _absolute_prim_path(value: str, field_name: str) -> None:
    if not value.startswith("/") or value == "/" or "//" in value:
        raise ValueError(f"{field_name} must be an absolute USD prim path")


def _finite_tuple(values: Sequence[float], size: int, field_name: str) -> tuple[float, ...]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (size,):
        raise ValueError(f"{field_name} must have shape {(size,)}, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{field_name} contains NaN or infinity")
    return tuple(float(value) for value in array)


@dataclass(frozen=True, slots=True)
class Hand2BallConfig:
    """Validated rigid-sphere parameters in SI units."""

    prim_path: str = "/World/Ball"
    material_prim_path: str = "/World/PhysicsMaterials/Hand2Ball"
    center_xyz_m: tuple[float, float, float] = (0.15, 0.0, 0.405)
    table_top_z_m: float = 0.38
    radius_m: float = 0.025
    mass_kg: float = 0.05
    static_friction: float = 1.0
    dynamic_friction: float = 0.8
    restitution: float = 0.02
    color_rgb: tuple[float, float, float] = (0.90, 0.12, 0.08)

    def __post_init__(self) -> None:
        _absolute_prim_path(self.prim_path, "prim_path")
        _absolute_prim_path(self.material_prim_path, "material_prim_path")
        center = _finite_tuple(self.center_xyz_m, 3, "center_xyz_m")
        color = _finite_tuple(self.color_rgb, 3, "color_rgb")
        if any(component < 0.0 or component > 1.0 for component in color):
            raise ValueError("color_rgb components must be in [0, 1]")
        if not math.isfinite(self.table_top_z_m):
            raise ValueError("table_top_z_m must be finite")
        if not math.isfinite(self.radius_m) or not 0.0 < self.radius_m <= 0.10:
            raise ValueError("radius_m must be finite and in (0, 0.10]")
        if not math.isfinite(self.mass_kg) or not 0.0 < self.mass_kg <= 5.0:
            raise ValueError("mass_kg must be finite and in (0, 5]")
        for name, value in (
            ("static_friction", self.static_friction),
            ("dynamic_friction", self.dynamic_friction),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 5.0:
                raise ValueError(f"{name} must be finite and in [0, 5]")
        if self.static_friction < self.dynamic_friction:
            raise ValueError("static_friction must be at least dynamic_friction")
        if not math.isfinite(self.restitution) or not 0.0 <= self.restitution <= 1.0:
            raise ValueError("restitution must be finite and in [0, 1]")
        sphere_bottom = center[2] - self.radius_m
        if sphere_bottom < self.table_top_z_m - 1e-6:
            raise ValueError("ball initially penetrates the table")


@dataclass(frozen=True, slots=True)
class BallContactFilters:
    """Ordered exact-body filters mapped to semantic finger groups."""

    labels: tuple[str, ...]
    prim_paths: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.labels or len(self.labels) != len(self.prim_paths):
            raise ValueError(
                "contact filter labels and prim paths must be non-empty and equal length"
            )
        allowed = set(FINGER_CONTACT_GROUPS) | {TABLE_CONTACT_GROUP}
        unknown = sorted(set(self.labels) - allowed)
        if unknown:
            raise ValueError(f"unknown contact filter labels: {unknown}")
        if len(set(self.prim_paths)) != len(self.prim_paths):
            raise ValueError("contact filter prim paths must be unique")
        for path in self.prim_paths:
            _absolute_prim_path(path, "contact filter path")


@dataclass(frozen=True, slots=True)
class Hand2BallSceneHandles:
    """Objects returned to an Isaac runner before ``world.reset()``.

    Isaac Sim 5.1's legacy ``RigidContactView`` is not compatible with the
    current ``Scene`` registry lifecycle.  After ``world.reset()``, callers must
    invoke ``contact_view.initialize(world.physics_sim_view)`` explicitly.
    """

    ball: Any
    contact_view: Any
    hand_table_contact_view: Any
    filters: BallContactFilters


def default_ball_contact_filters(
    hand_prim_path: str = "/World/Hand2", table_prim_path: str = "/World/Table"
) -> BallContactFilters:
    """Return exact phalanx-body filters grouped into five fingers and table."""

    _absolute_prim_path(hand_prim_path, "hand_prim_path")
    _absolute_prim_path(table_prim_path, "table_prim_path")
    hand = hand_prim_path.rstrip("/")
    return BallContactFilters(
        labels=(
            *("thumb",) * 3,
            *("index",) * 3,
            *("middle",) * 3,
            *("ring",) * 3,
            *("pinky",) * 3,
            TABLE_CONTACT_GROUP,
        ),
        prim_paths=(
            f"{hand}/r_thumb_proximal_abd",
            f"{hand}/r_thumb_middle",
            f"{hand}/r_thumb_distal",
            f"{hand}/r_index_finger_proximal_abd",
            f"{hand}/r_index_finger_middle",
            f"{hand}/r_index_finger_distal",
            f"{hand}/r_middle_finger_proximal_abd",
            f"{hand}/r_middle_finger_middle",
            f"{hand}/r_middle_finger_distal",
            f"{hand}/r_ring_finger_proximal_abd",
            f"{hand}/r_ring_finger_middle",
            f"{hand}/r_ring_finger_distal",
            f"{hand}/r_pinky_proximal_abd",
            f"{hand}/r_pinky_middle",
            f"{hand}/r_pinky_distal",
            table_prim_path,
        ),
    )


def add_hand2_ball_scene(
    world: Any,
    config: Hand2BallConfig | None = None,
    filters: BallContactFilters | None = None,
    *,
    max_contact_count: int = 64,
) -> Hand2BallSceneHandles:
    """Add a ``DynamicSphere`` and contact view before ``world.reset()``.

    The ball is a genuine rigid body; this helper does not author an attachment,
    kinematic target, or grasp constraint.  Contact forces come from PhysX.
    """

    from isaacsim.core.api.materials import PhysicsMaterial  # type: ignore[import-not-found]
    from isaacsim.core.api.objects import DynamicSphere  # type: ignore[import-not-found]
    from isaacsim.core.api.sensors import RigidContactView  # type: ignore[import-not-found]

    config = config or Hand2BallConfig()
    filters = filters or default_ball_contact_filters()
    if not isinstance(max_contact_count, int) or max_contact_count <= 0:
        raise ValueError("max_contact_count must be a positive integer")
    stage = world.scene.stage
    if stage.GetPrimAtPath(config.prim_path).IsValid():
        raise RuntimeError(f"ball prim already exists: {config.prim_path}")
    material = PhysicsMaterial(
        prim_path=config.material_prim_path,
        name="hand2_ball_material",
        static_friction=config.static_friction,
        dynamic_friction=config.dynamic_friction,
        restitution=config.restitution,
    )
    ball = world.scene.add(
        DynamicSphere(
            prim_path=config.prim_path,
            name="hand2_grasp_ball",
            position=np.asarray(config.center_xyz_m, dtype=np.float64),
            radius=config.radius_m,
            mass=config.mass_kg,
            color=np.asarray(config.color_rgb, dtype=np.float32),
            physics_material=material,
        )
    )
    contact_view = RigidContactView(
        prim_paths_expr=config.prim_path,
        filter_paths_expr=list(filters.prim_paths),
        name="hand2_ball_contacts",
        prepare_contact_sensors=True,
        max_contact_count=max_contact_count,
    )
    table_filter_index = filters.labels.index(TABLE_CONTACT_GROUP)
    hand_root = str(filters.prim_paths[0]).rsplit("/", 1)[0]
    hand_table_contact_view = RigidContactView(
        prim_paths_expr=f"{hand_root}/.*",
        filter_paths_expr=[filters.prim_paths[table_filter_index]],
        name="hand2_table_contacts",
        prepare_contact_sensors=True,
        max_contact_count=max_contact_count,
    )
    # Do not register this legacy view with Scene in Isaac Sim 5.1: it lacks
    # Scene's ``name``, ``is_valid`` and ``post_reset`` object protocol.  The
    # runner initializes it explicitly with World.physics_sim_view after reset.
    return Hand2BallSceneHandles(
        ball=ball,
        contact_view=contact_view,
        hand_table_contact_view=hand_table_contact_view,
        filters=filters,
    )


def contact_groups_from_force_matrix(
    force_matrix_n: npt.ArrayLike,
    filters: BallContactFilters,
    *,
    threshold_n: float = 0.05,
) -> frozenset[str]:
    """Classify PhysX contact-filter forces into semantic contact groups.

    ``RigidContactView.get_contact_force_matrix(dt=physics_dt)`` returns forces
    for one ball as shape ``(1, num_filters, 3)``; the leading singleton may be
    omitted.  Merely seeing finite actuator effort is deliberately not accepted
    as contact evidence.
    """

    if not math.isfinite(threshold_n) or threshold_n < 0.0:
        raise ValueError("threshold_n must be finite and non-negative")
    matrix = np.asarray(force_matrix_n, dtype=np.float64)
    if matrix.shape == (1, len(filters.labels), 3):
        matrix = matrix[0]
    expected_shape = (len(filters.labels), 3)
    if matrix.shape != expected_shape:
        raise ValueError(f"force_matrix_n must have shape {expected_shape} or (1, ...)")
    if not np.isfinite(matrix).all():
        raise ValueError("force_matrix_n contains NaN or infinity")
    magnitudes = np.linalg.norm(matrix, axis=1)
    return frozenset(
        label
        for label, magnitude in zip(filters.labels, magnitudes, strict=True)
        if magnitude >= threshold_n
    )


__all__ = [
    "BallContactFilters",
    "FINGER_CONTACT_GROUPS",
    "Hand2BallConfig",
    "Hand2BallSceneHandles",
    "TABLE_CONTACT_GROUP",
    "add_hand2_ball_scene",
    "contact_groups_from_force_matrix",
    "default_ball_contact_filters",
]
