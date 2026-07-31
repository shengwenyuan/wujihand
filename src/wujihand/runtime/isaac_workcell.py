"""Isaac materialization for source-neutral Workcell plans.

Isaac and USD modules are imported only inside the materialization boundary so
the plan remains usable in ordinary Python tests.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .isaac_workcell_plan import (
    ResolvedIsaacUsdImport,
    ResolvedIsaacWorkcellPlan,
)


@dataclass(frozen=True, slots=True)
class IsaacWorkcellMaterialization:
    plan: ResolvedIsaacWorkcellPlan
    imported_prim_paths: tuple[str, ...]
    primitive_prim_paths: tuple[str, ...]
    collider_paths: tuple[str, ...]
    fixed_collider_paths: tuple[str, ...]
    rigid_body_paths: tuple[str, ...]
    physics_scene_paths: tuple[str, ...]
    light_paths: tuple[str, ...]
    camera_paths: tuple[str, ...]
    dependency_layer_count: int
    dependency_asset_count: int
    runtime_module_dependencies: tuple[str, ...]

    def to_mapping(self) -> dict[str, object]:
        return {
            "plan": self.plan.to_mapping(),
            "imported_prim_paths": list(self.imported_prim_paths),
            "primitive_prim_paths": list(self.primitive_prim_paths),
            "collider_paths": list(self.collider_paths),
            "fixed_collider_paths": list(self.fixed_collider_paths),
            "rigid_body_paths": list(self.rigid_body_paths),
            "physics_scene_paths": list(self.physics_scene_paths),
            "light_paths": list(self.light_paths),
            "camera_paths": list(self.camera_paths),
            "dependency_layer_count": self.dependency_layer_count,
            "dependency_asset_count": self.dependency_asset_count,
            "runtime_module_dependencies": list(
                self.runtime_module_dependencies
            ),
        }


def materialize_isaac_workcell(
    world: Any,
    plan: ResolvedIsaacWorkcellPlan,
) -> IsaacWorkcellMaterialization:
    """Apply one plan to an initialized Isaac ``World``."""

    from isaacsim.core.api.objects import (  # type: ignore[import-not-found]
        FixedCuboid,
    )
    from isaacsim.core.utils.stage import (  # type: ignore[import-not-found]
        add_reference_to_stage,
    )
    from pxr import (  # type: ignore[import-not-found]
        Gf,
        Sdf,
        Usd,
        UsdGeom,
        UsdLux,
        UsdPhysics,
        UsdUtils,
    )

    stage = world.scene.stage
    UsdGeom.Xform.Define(stage, "/World/Environment")
    UsdGeom.Xform.Define(stage, "/World/Workcell")
    UsdGeom.Xform.Define(stage, "/World/Lighting")

    dependency_layers: set[str] = set()
    dependency_assets: set[str] = set()
    runtime_modules: set[str] = set()
    for import_operation in plan.imports:
        source = Usd.Stage.Open(str(import_operation.content.absolute_path))
        if source is None:
            raise RuntimeError(
                "could not open USD scene: "
                f"{import_operation.content.absolute_path}"
            )
        _validate_source_stage(source, import_operation, plan)
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(
            str(import_operation.content.absolute_path)
        )
        dependency_layers.update(
            layer.identifier for layer in layers if layer is not None
        )
        dependency_assets.update(str(asset) for asset in assets)
        missing = tuple(
            str(asset)
            for asset in unresolved
            if not str(asset).endswith(".mdl")
        )
        runtime_modules.update(
            str(asset) for asset in unresolved if str(asset).endswith(".mdl")
        )
        if missing:
            raise RuntimeError(
                f"USD scene has unresolved dependencies: {sorted(missing)}"
            )
        add_reference_to_stage(
            str(import_operation.content.absolute_path),
            import_operation.prim_path,
        )
        prim = stage.GetPrimAtPath(import_operation.prim_path)
        if not prim.IsValid():
            raise RuntimeError(
                f"USD reference did not create {import_operation.prim_path}"
            )
        _set_pose(
            UsdGeom.Xformable(prim),
            import_operation.pose,
            Gf,
            UsdGeom,
        )

    imported_prefixes = tuple(
        import_operation.prim_path
        for import_operation in plan.imports
    )
    imported_physics = tuple(
        str(prim.GetPath())
        for prim in stage.Traverse()
        if _below_any(str(prim.GetPath()), imported_prefixes)
        and prim.IsA(UsdPhysics.Scene)
    )
    if plan.policies.physics_scene == "project" and imported_physics:
        raise RuntimeError(
            "project-owned PhysicsScene conflicts with imported scenes: "
            f"{imported_physics}"
        )
    if plan.policies.physics_scene == "preserve" and len(imported_physics) != 1:
        raise RuntimeError(
            "preserve physics_scene policy requires exactly one imported "
            f"PhysicsScene, found {imported_physics}"
        )

    planes = tuple(
        operation
        for operation in plan.primitives
        if operation.entity.primitive.kind == "plane"
    )
    if planes and plan.policies.ground != "project":
        raise RuntimeError(
            "plane primitives require the project ground policy"
        )
    if plan.policies.ground == "project":
        if len(planes) != 1:
            raise RuntimeError(
                "project ground policy requires exactly one plane primitive"
            )
        world.scene.add_default_ground_plane()

    primitive_paths: list[str] = []
    for primitive_operation in plan.primitives:
        primitive = primitive_operation.entity.primitive
        if primitive.kind == "plane":
            continue
        if primitive_operation.entity.mobility != "fixed":
            raise RuntimeError(
                "Isaac primitive overlays currently require fixed mobility: "
                f"{primitive_operation.entity_id}"
            )
        if primitive.kind != "box" or primitive.size_m is None:
            raise RuntimeError(
                "Isaac primitive overlays currently support plane and box: "
                f"{primitive_operation.entity_id}"
            )
        world.scene.add(
            FixedCuboid(
                prim_path=primitive_operation.prim_path,
                name=primitive_operation.entity_id,
                position=np.asarray(
                    primitive_operation.pose.position_m,
                    dtype=np.float64,
                ),
                orientation=np.asarray(
                    primitive_operation.pose.quat_wxyz,
                    dtype=np.float64,
                ),
                scale=np.asarray(
                    primitive.size_m,
                    dtype=np.float64,
                ),
                size=1.0,
                color=np.asarray((0.28, 0.23, 0.18), dtype=np.float64),
            )
        )
        primitive_paths.append(primitive_operation.prim_path)

    if plan.lighting.mode in {"project", "selected_hdr"}:
        dome = UsdLux.DomeLight.Define(
            stage,
            "/World/Lighting/Environment",
        )
        dome.CreateIntensityAttr(plan.lighting.intensity)
        dome.CreateExposureAttr(plan.lighting.exposure)
        if plan.lighting.mode == "selected_hdr":
            if plan.lighting.content is None:
                raise RuntimeError("selected_hdr lighting has no content")
            dome.CreateTextureFileAttr(
                Sdf.AssetPath(str(plan.lighting.content.absolute_path))
            )
            dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)

    imported_lights = tuple(
        str(prim.GetPath())
        for prim in stage.Traverse()
        if _below_any(str(prim.GetPath()), imported_prefixes)
        and prim.HasAPI(UsdLux.LightAPI)
    )
    if plan.lighting.mode == "preserve" and not imported_lights:
        raise RuntimeError(
            "preserve lighting policy requires an imported light"
        )

    colliders = tuple(
        sorted(
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.CollisionAPI)
            and (
                _below_any(str(prim.GetPath()), imported_prefixes)
                or str(prim.GetPath()).startswith("/World/Workcell/")
                or str(prim.GetPath()).startswith("/World/defaultGroundPlane")
            )
        )
    )
    if (
        plan.expectations is not None
        and len(
            tuple(
                path
                for path in colliders
                if _below_any(path, imported_prefixes)
            )
        )
        < plan.expectations.min_colliders
    ):
        raise RuntimeError(
            "imported collider count is below the profile expectation"
        )
    rigid_bodies = tuple(
        sorted(
            str(prim.GetPath())
            for prim in stage.Traverse()
            if _below_any(str(prim.GetPath()), imported_prefixes)
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and _rigid_body_enabled(prim, UsdPhysics)
        )
    )
    fixed_colliders = tuple(
        path
        for path in colliders
        if not _has_enabled_rigid_body_ancestor(
            stage.GetPrimAtPath(path),
            UsdPhysics,
        )
    )
    physics_scenes = tuple(
        sorted(
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.Scene)
        )
    )
    lights = tuple(
        sorted(
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.HasAPI(UsdLux.LightAPI)
        )
    )
    cameras = tuple(
        sorted(
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.IsA(UsdGeom.Camera)
        )
    )
    return IsaacWorkcellMaterialization(
        plan=plan,
        imported_prim_paths=imported_prefixes,
        primitive_prim_paths=tuple(primitive_paths),
        collider_paths=colliders,
        fixed_collider_paths=fixed_colliders,
        rigid_body_paths=rigid_bodies,
        physics_scene_paths=physics_scenes,
        light_paths=lights,
        camera_paths=cameras,
        dependency_layer_count=len(dependency_layers),
        dependency_asset_count=len(dependency_assets),
        runtime_module_dependencies=tuple(sorted(runtime_modules)),
    )


def _validate_source_stage(
    source: Any,
    operation: ResolvedIsaacUsdImport,
    plan: ResolvedIsaacWorkcellPlan,
) -> None:
    from pxr import UsdGeom

    expectations = plan.expectations
    if expectations is None:
        return
    default_prim = source.GetDefaultPrim()
    if not default_prim.IsValid() or default_prim.GetName() != expectations.default_prim:
        raise RuntimeError(
            f"{operation.import_id} default prim differs: "
            f"expected={expectations.default_prim!r}, "
            f"actual={default_prim.GetName()!r}"
        )
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(source)
    if not math.isclose(
        meters_per_unit,
        expectations.meters_per_unit,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(
            f"{operation.import_id} metersPerUnit differs: {meters_per_unit}"
        )
    up_axis = UsdGeom.GetStageUpAxis(source)
    if up_axis != expectations.up_axis:
        raise RuntimeError(
            f"{operation.import_id} up axis differs: {up_axis!r}"
        )


def _set_pose(
    xformable: Any,
    pose: Any,
    gf: Any,
    usd_geom: Any,
) -> None:
    xformable.ClearXformOpOrder()
    matrix = gf.Matrix4d(1.0)
    matrix.SetRotate(gf.Quatd(*pose.quat_wxyz))
    matrix.SetTranslateOnly(gf.Vec3d(*pose.position_m))
    xformable.AddTransformOp(
        usd_geom.XformOp.PrecisionDouble
    ).Set(matrix)


def _below_any(path: str, prefixes: tuple[str, ...]) -> bool:
    return any(path == prefix or path.startswith(prefix + "/") for prefix in prefixes)


def _rigid_body_enabled(prim: Any, usd_physics: Any) -> bool:
    value = usd_physics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
    return value is not False


def _has_enabled_rigid_body_ancestor(
    prim: Any,
    usd_physics: Any,
) -> bool:
    current = prim
    while current.IsValid():
        if current.HasAPI(usd_physics.RigidBodyAPI) and _rigid_body_enabled(
            current,
            usd_physics,
        ):
            return True
        current = current.GetParent()
    return False


__all__ = [
    "IsaacWorkcellMaterialization",
    "materialize_isaac_workcell",
]
