"""Resolve and materialize the passive dual D405 wrist rig in Isaac."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Literal, cast

import yaml

from wujihand.adapters.simulation.d405_wrist_rig_assets import (
    Matrix3,
    Triangle,
    Vector3,
    determinant,
    load_stl_triangles,
)
from wujihand.integrity import sha256_file
from wujihand.specs import AttachmentSpec, IsaacCameraProfile, PoseSpec

from .config_repository import ConfigRepository
from .session_resolver import ResolvedInstance, ResolvedSession


WristRigCollisionMode = Literal["none", "mount", "all"]
_GENERATED_SOURCE = "isaac-d405-wrist-rig-v2"
_GENERATION_REPORT = "generation_report.json"
_MOUNT_PRODUCT = "nero_hand2_beta1_d405_wrist_mount"
_CAMERA_PRODUCT = "realsense_d405_housing"


@dataclass(frozen=True, slots=True)
class RigidTransform:
    translation_m: Vector3
    rotation: Matrix3


@dataclass(frozen=True, slots=True)
class CollisionBox:
    name: str
    center_m: Vector3
    size_m: Vector3
    rotation: Matrix3


@dataclass(frozen=True, slots=True)
class CollisionCapsuleSegment:
    name: str
    start_m: Vector3
    end_m: Vector3
    radius_m: float


CollisionPrimitive = CollisionBox | CollisionCapsuleSegment


@dataclass(frozen=True, slots=True)
class CompoundCollisionProxy:
    component: str
    side: str
    canonical_frame: str
    primitives: tuple[CollisionPrimitive, ...]


@dataclass(frozen=True, slots=True)
class D405WristRigRuntime:
    side: str
    hand_instance_id: str
    mount_instance_id: str
    camera_instance_id: str
    hand_prim_path: str
    mount_visual_path: Path
    mount_visual_sha256: str
    mount_collision_path: Path
    mount_collision_sha256: str
    camera_visual_path: Path
    camera_visual_sha256: str
    camera_collision_path: Path
    camera_collision_sha256: str
    camera_profile_path: Path
    camera_profile: IsaacCameraProfile
    body_in_hand: RigidTransform
    optical_in_hand: RigidTransform
    optical_in_body: RigidTransform
    mount_collision: CompoundCollisionProxy
    camera_collision: CompoundCollisionProxy
    generation_report_path: Path
    generation_report_sha256: str
    simulation_warning: str


@dataclass(frozen=True, slots=True)
class MassPropertiesSnapshot:
    mass_kg: float | None
    center_of_mass_m: Vector3 | None
    diagonal_inertia_kg_m2: Vector3 | None
    principal_axes_wxyz: tuple[float, float, float, float] | None


@dataclass(frozen=True, slots=True)
class D405WristRigHandles:
    side: str
    root_path: str
    mount_visual_path: str
    camera_visual_path: str
    camera_prim_path: str
    mount_collision_paths: tuple[str, ...]
    camera_collision_paths: tuple[str, ...]
    hand_base_mass_before: MassPropertiesSnapshot
    hand_base_mass_after: MassPropertiesSnapshot


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} must be a string-keyed mapping")
    return cast(Mapping[str, object], value)


def _sequence(value: object, *, field: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _number(value: object, *, field: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise ValueError(f"{field} must be finite{' and positive' if positive else ''}")
    return result


def _vector3(value: object, *, field: str, scale: float = 1.0) -> Vector3:
    items = _sequence(value, field=field)
    if len(items) != 3:
        raise ValueError(f"{field} must contain exactly three values")
    return cast(
        Vector3,
        tuple(
            _number(item, field=f"{field}[{index}]") * scale
            for index, item in enumerate(items)
        ),
    )


def _matrix3(value: object, *, field: str) -> Matrix3:
    rows = _sequence(value, field=field)
    if len(rows) != 3:
        raise ValueError(f"{field} must contain exactly three rows")
    result = cast(
        Matrix3,
        tuple(_vector3(row, field=f"{field}[{index}]") for index, row in enumerate(rows)),
    )
    product = tuple(
        tuple(
            sum(result[row][inner] * result[column][inner] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )
    error = max(
        abs(product[row][column] - (1.0 if row == column else 0.0))
        for row in range(3)
        for column in range(3)
    )
    if error > 1e-9 or abs(determinant(result) - 1.0) > 1e-9:
        raise ValueError(f"{field} must be a proper orthonormal rotation")
    return result


def _is_identity_attachment(transform: PoseSpec) -> bool:
    return transform.position_m == (0.0, 0.0, 0.0) and transform.quat_wxyz == (
        1.0,
        0.0,
        0.0,
        0.0,
    )


def _one_attachment(
    resolved: ResolvedSession,
    *,
    parent: str,
    child: str,
) -> AttachmentSpec:
    matches = tuple(
        attachment
        for attachment in resolved.assembly.attachments
        if attachment.parent.instance == parent and attachment.child.instance == child
    )
    if len(matches) != 1:
        raise RuntimeError(
            f"wrist-rig edge {parent!r} -> {child!r} must occur exactly once"
        )
    return matches[0]


def _validate_generated_artifact(
    report: Mapping[str, object],
    *,
    key: str,
    instance: ResolvedInstance,
    collision: bool,
) -> None:
    artifact = instance.collision_artifact if collision else instance.artifact
    if artifact is None:
        raise RuntimeError(f"{instance.instance_id} is missing a generated artifact")
    output = _mapping(
        _mapping(report.get("outputs"), field="generation report.outputs").get(key),
        field=f"generation report.outputs.{key}",
    )
    if (
        output.get("path") != artifact.relative_path
        or output.get("sha256") != artifact.expected_sha256
    ):
        raise RuntimeError(f"{instance.instance_id} differs from generation report {key}")


def _artifact_family(instance: ResolvedInstance) -> tuple[str, ...]:
    artifacts = (instance.artifact, instance.collision_artifact)
    if any(artifact is None for artifact in artifacts):
        raise RuntimeError(f"passive instance {instance.instance_id} lacks artifacts")
    return tuple(cast(Any, artifact).source.name for artifact in artifacts)


def _load_generation_report(
    mount: ResolvedInstance,
    camera: ResolvedInstance,
) -> tuple[Path, Mapping[str, object], str]:
    if set((*_artifact_family(mount), *_artifact_family(camera))) != {
        _GENERATED_SOURCE
    }:
        raise RuntimeError("wrist-rig assets must share the generated source")
    mount_artifact = cast(Any, mount.artifact)
    report_path = mount_artifact.absolute_path.parents[1] / _GENERATION_REPORT
    expected_hash = mount_artifact.source.expected_artifact_hash(_GENERATION_REPORT)
    if not report_path.is_file() or sha256_file(report_path) != expected_hash:
        raise RuntimeError("D405 wrist-rig generation report is missing or hash-drifted")
    raw = json.loads(report_path.read_text(encoding="utf-8"))
    report = _mapping(raw, field="generation report")
    if (
        report.get("schema")
        != "wujihand.d405_wrist_rig_asset_generation_result.v1"
        or report.get("status") != "qualified"
    ):
        raise ValueError("unsupported or unqualified wrist-rig generation report")
    warning = _string(report.get("camera_warning"), field="camera_warning")
    if "synthetic 140-degree HFOV" not in warning or "not a physical" not in warning:
        raise ValueError("generation report lost the simulation-only camera warning")
    return report_path, report, expected_hash


def load_compound_collision_proxy(
    path: str | Path,
    *,
    expected_side: str,
    expected_canonical_frame: str,
) -> CompoundCollisionProxy:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _mapping(raw, field="collision proxy")
    if data.get("schema") != "wujihand.compound_collision_proxy.v1":
        raise ValueError("unsupported collision proxy schema")
    if data.get("side") != expected_side:
        raise ValueError("collision proxy side differs from resolved asset")
    if data.get("canonical_frame") != expected_canonical_frame:
        raise ValueError("collision proxy canonical frame differs")
    if data.get("units") != "mm":
        raise ValueError("collision proxy must remain in generated millimetres")
    if data.get("rigid_body_policy") != "child_shapes_only_no_mass_or_rigid_body_api":
        raise ValueError("collision proxy rigid-body policy differs")
    primitives: list[CollisionPrimitive] = []
    names: set[str] = set()
    for index, raw_primitive in enumerate(
        _sequence(data.get("primitives"), field="collision proxy.primitives")
    ):
        field = f"collision proxy.primitives[{index}]"
        primitive = _mapping(raw_primitive, field=field)
        name = _string(primitive.get("name"), field=f"{field}.name")
        if name in names:
            raise ValueError("collision proxy primitive names must be unique")
        names.add(name)
        kind = primitive.get("type")
        if kind == "box":
            size = _vector3(primitive.get("size_mm"), field=f"{field}.size_mm", scale=0.001)
            if any(value <= 0.0 for value in size):
                raise ValueError(f"{field}.size_mm must be positive")
            rotation = (
                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
                if primitive.get("rotation") is None
                else _matrix3(primitive.get("rotation"), field=f"{field}.rotation")
            )
            primitives.append(
                CollisionBox(
                    name=name,
                    center_m=_vector3(
                        primitive.get("center_mm"), field=f"{field}.center_mm", scale=0.001
                    ),
                    size_m=size,
                    rotation=rotation,
                )
            )
        elif kind == "capsule_segment":
            radius = _number(
                primitive.get("radius_mm"), field=f"{field}.radius_mm", positive=True
            ) * 0.001
            start = _vector3(
                primitive.get("start_mm"), field=f"{field}.start_mm", scale=0.001
            )
            end = _vector3(
                primitive.get("end_mm"), field=f"{field}.end_mm", scale=0.001
            )
            if math.dist(start, end) <= 1e-9:
                raise ValueError(f"{field} segment must have positive length")
            primitives.append(
                CollisionCapsuleSegment(
                    name=name,
                    start_m=start,
                    end_m=end,
                    radius_m=radius,
                )
            )
        else:
            raise ValueError(f"{field}.type is unsupported: {kind!r}")
    if not primitives:
        raise ValueError("collision proxy must contain primitives")
    return CompoundCollisionProxy(
        component=_string(data.get("component"), field="collision proxy.component"),
        side=expected_side,
        canonical_frame=expected_canonical_frame,
        primitives=tuple(primitives),
    )


def resolve_d405_wrist_rig_runtimes(
    project_root: Path,
    resolved: ResolvedSession,
) -> tuple[D405WristRigRuntime, ...]:
    """Resolve complete hand→mount→D405 subtrees or no wrist rig at all."""

    passive = tuple(
        instance
        for instance in resolved.instances
        if instance.asset.kind in {"passive_component", "simulated_sensor"}
    )
    if not passive:
        return ()
    if len(passive) != 4:
        raise RuntimeError("dual D405 Session must contain exactly four passive instances")
    result: list[D405WristRigRuntime] = []
    for side in ("left", "right"):
        hands = tuple(
            instance
            for instance in resolved.instances
            if instance.asset.product == "wuji_hand_2" and instance.asset.side == side
        )
        mounts = tuple(
            instance
            for instance in passive
            if instance.asset.product == _MOUNT_PRODUCT and instance.asset.side == side
        )
        cameras = tuple(
            instance
            for instance in passive
            if instance.asset.product == _CAMERA_PRODUCT and instance.asset.side == side
        )
        if not (len(hands) == len(mounts) == len(cameras) == 1):
            raise RuntimeError(f"{side} wrist-rig subtree is missing or duplicated")
        hand, mount, camera = hands[0], mounts[0], cameras[0]
        hand_to_mount = _one_attachment(
            resolved, parent=hand.instance_id, child=mount.instance_id
        )
        mount_to_camera = _one_attachment(
            resolved, parent=mount.instance_id, child=camera.instance_id
        )
        if (
            hand_to_mount.parent.frame != hand.asset.frame_name("base")
            or hand_to_mount.child.frame != mount.asset.frame_name("hand_interface")
            or mount_to_camera.parent.frame != mount.asset.frame_name("camera_interface")
            or mount_to_camera.child.frame != camera.asset.frame_name("rear_mount")
            or not _is_identity_attachment(hand_to_mount.transform)
            or not _is_identity_attachment(mount_to_camera.transform)
        ):
            raise RuntimeError(f"{side} wrist-rig canonical attachment contract differs")
        if (
            mount.binding.compatibility_profile != mount.asset.canonical_profile
            or camera.binding.compatibility_profile != camera.asset.canonical_profile
            or mount.asset.canonical_profile != camera.asset.canonical_profile
            or camera.binding.sensor_profile is None
        ):
            raise RuntimeError(f"{side} wrist-rig profile ownership differs")
        report_path, report, report_hash = _load_generation_report(mount, camera)
        _validate_generated_artifact(
            report,
            key=f"mount_visual_{side}",
            instance=mount,
            collision=False,
        )
        _validate_generated_artifact(
            report,
            key=f"mount_collision_{side}",
            instance=mount,
            collision=True,
        )
        _validate_generated_artifact(
            report,
            key=f"d405_visual_{side}",
            instance=camera,
            collision=False,
        )
        _validate_generated_artifact(
            report,
            key=f"d405_collision_{side}",
            instance=camera,
            collision=True,
        )
        frames = _mapping(
            _mapping(report.get("optical_frames"), field="optical_frames").get(side),
            field=f"optical_frames.{side}",
        )
        rear_to_optical = _mapping(
            frames.get("rear_mount_to_optical"),
            field=f"optical_frames.{side}.rear_mount_to_optical",
        )
        mount_visual = cast(Any, mount.artifact)
        mount_collision = cast(Any, mount.collision_artifact)
        camera_visual = cast(Any, camera.artifact)
        camera_collision = cast(Any, camera.collision_artifact)
        camera_profile_path = (project_root / camera.binding.sensor_profile).resolve()
        camera_profile = ConfigRepository(project_root).load_isaac_camera_profile(
            camera_profile_path
        )
        result.append(
            D405WristRigRuntime(
                side=side,
                hand_instance_id=hand.instance_id,
                mount_instance_id=mount.instance_id,
                camera_instance_id=camera.instance_id,
                hand_prim_path=f"/World/Robots/Hand2{side.capitalize()}",
                mount_visual_path=mount_visual.absolute_path,
                mount_visual_sha256=mount_visual.expected_sha256,
                mount_collision_path=mount_collision.absolute_path,
                mount_collision_sha256=mount_collision.expected_sha256,
                camera_visual_path=camera_visual.absolute_path,
                camera_visual_sha256=camera_visual.expected_sha256,
                camera_collision_path=camera_collision.absolute_path,
                camera_collision_sha256=camera_collision.expected_sha256,
                camera_profile_path=camera_profile_path,
                camera_profile=camera_profile,
                body_in_hand=RigidTransform(
                    translation_m=_vector3(
                        frames.get("body_translation_in_hand_mm"),
                        field=f"optical_frames.{side}.body_translation_in_hand_mm",
                        scale=0.001,
                    ),
                    rotation=_matrix3(
                        frames.get("body_rotation_in_hand"),
                        field=f"optical_frames.{side}.body_rotation_in_hand",
                    ),
                ),
                optical_in_hand=RigidTransform(
                    translation_m=_vector3(
                        frames.get("optical_origin_in_hand_mm"),
                        field=f"optical_frames.{side}.optical_origin_in_hand_mm",
                        scale=0.001,
                    ),
                    rotation=_matrix3(
                        frames.get("optical_rotation_in_hand"),
                        field=f"optical_frames.{side}.optical_rotation_in_hand",
                    ),
                ),
                optical_in_body=RigidTransform(
                    translation_m=_vector3(
                        rear_to_optical.get("translation_mm"),
                        field=f"optical_frames.{side}.rear_mount_to_optical.translation_mm",
                        scale=0.001,
                    ),
                    rotation=_matrix3(
                        rear_to_optical.get("rotation"),
                        field=f"optical_frames.{side}.rear_mount_to_optical.rotation",
                    ),
                ),
                mount_collision=load_compound_collision_proxy(
                    mount_collision.absolute_path,
                    expected_side=side,
                    expected_canonical_frame="hand_interface",
                ),
                camera_collision=load_compound_collision_proxy(
                    camera_collision.absolute_path,
                    expected_side=side,
                    expected_canonical_frame="rear_mount",
                ),
                generation_report_path=report_path,
                generation_report_sha256=report_hash,
                simulation_warning=cast(str, report["camera_warning"]),
            )
        )
    return tuple(result)


def _quat_wxyz(rotation: Matrix3) -> tuple[float, float, float, float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            0.25 * scale,
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
        )
    else:
        axis = max(range(3), key=lambda index: rotation[index][index])
        if axis == 0:
            scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
            values = (
                (rotation[2][1] - rotation[1][2]) / scale,
                0.25 * scale,
                (rotation[0][1] + rotation[1][0]) / scale,
                (rotation[0][2] + rotation[2][0]) / scale,
            )
        elif axis == 1:
            scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
            values = (
                (rotation[0][2] - rotation[2][0]) / scale,
                (rotation[0][1] + rotation[1][0]) / scale,
                0.25 * scale,
                (rotation[1][2] + rotation[2][1]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
            values = (
                (rotation[1][0] - rotation[0][1]) / scale,
                (rotation[0][2] + rotation[2][0]) / scale,
                (rotation[1][2] + rotation[2][1]) / scale,
                0.25 * scale,
            )
    norm = math.sqrt(sum(value * value for value in values))
    return cast(tuple[float, float, float, float], tuple(value / norm for value in values))


def _matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return cast(
        Matrix3,
        tuple(
            cast(
                Vector3,
                tuple(
                    sum(left[row][inner] * right[inner][column] for inner in range(3))
                    for column in range(3)
                ),
            )
            for row in range(3)
        ),
    )


def _capsule_quat_wxyz(direction: Vector3) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(value * value for value in direction))
    unit = tuple(value / length for value in direction)
    if unit[2] < -1.0 + 1e-12:
        return (0.0, 1.0, 0.0, 0.0)
    values = (1.0 + unit[2], -unit[1], unit[0], 0.0)
    norm = math.sqrt(sum(value * value for value in values))
    return cast(tuple[float, float, float, float], tuple(value / norm for value in values))


def _set_pose(prim: Any, transform: RigidTransform) -> None:
    from pxr import Gf, UsdGeom  # type: ignore[import-not-found]

    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Quatd(*_quat_wxyz(transform.rotation)))
    matrix.SetTranslateOnly(Gf.Vec3d(*transform.translation_m))
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix)


def _triangle_normal(triangle: Triangle) -> Vector3:
    left = tuple(triangle[1][axis] - triangle[0][axis] for axis in range(3))
    right = tuple(triangle[2][axis] - triangle[0][axis] for axis in range(3))
    cross = (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )
    norm = math.sqrt(sum(value * value for value in cross))
    return cast(Vector3, tuple(value / norm for value in cross))


def _author_visual_mesh(
    stage: Any,
    *,
    path: str,
    source_path: Path,
    expected_sha256: str,
    color: Vector3,
) -> str:
    from pxr import Gf, Sdf, UsdGeom

    if sha256_file(source_path) != expected_sha256:
        raise RuntimeError(f"visual mesh hash drifted: {source_path}")
    triangles = load_stl_triangles(source_path)
    points = [
        Gf.Vec3f(*(coordinate * 0.001 for coordinate in point))
        for triangle in triangles
        for point in triangle
    ]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
    mesh.CreateNormalsAttr([Gf.Vec3f(*_triangle_normal(item)) for item in triangles])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr(UsdGeom.Tokens.rightHanded)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    mesh.GetPrim().CreateAttribute(
        "wujihand:sourceSha256", Sdf.ValueTypeNames.String
    ).Set(expected_sha256)
    return path


def _author_synthetic_camera(
    stage: Any,
    *,
    path: str,
    runtime: D405WristRigRuntime,
) -> str:
    from pxr import Gf, Sdf, UsdGeom

    profile = runtime.camera_profile
    if (
        not profile.simulation_only
        or profile.optics.horizontal_fov_deg != 140.0
        or "not a physical RealSense D405" not in profile.warning
    ):
        raise RuntimeError("D405 Camera prim requires the synthetic 140-degree profile")
    # SIMULATION ONLY: synthetic 140-degree HFOV; not a physical RealSense
    # D405 specification or calibration.
    usd_camera_from_ros_optical: Matrix3 = (
        (1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
        (0.0, 0.0, -1.0),
    )
    camera = UsdGeom.Camera.Define(stage, path)
    _set_pose(
        camera.GetPrim(),
        RigidTransform(
            translation_m=runtime.optical_in_body.translation_m,
            rotation=_matrix_multiply(
                runtime.optical_in_body.rotation,
                usd_camera_from_ros_optical,
            ),
        ),
    )
    optics = profile.optics
    camera.CreateProjectionAttr(optics.projection)
    camera.CreateFocalLengthAttr(optics.focal_length_mm)
    camera.CreateHorizontalApertureAttr(optics.horizontal_aperture_mm)
    camera.CreateVerticalApertureAttr(optics.vertical_aperture_mm)
    camera.CreateHorizontalApertureOffsetAttr(optics.horizontal_aperture_offset_mm)
    camera.CreateVerticalApertureOffsetAttr(optics.vertical_aperture_offset_mm)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*optics.clipping_range_m))
    prim = camera.GetPrim()
    custom = {
        "wujihand:cameraProfileId": (Sdf.ValueTypeNames.String, profile.profile_id),
        "wujihand:projectionClassification": (
            Sdf.ValueTypeNames.String,
            profile.projection_classification,
        ),
        "wujihand:simulationWarning": (Sdf.ValueTypeNames.String, profile.warning),
        "wujihand:simulationOnly": (Sdf.ValueTypeNames.Bool, True),
        "wujihand:captureWidthPx": (
            Sdf.ValueTypeNames.Int,
            profile.capture.width_px,
        ),
        "wujihand:captureHeightPx": (
            Sdf.ValueTypeNames.Int,
            profile.capture.height_px,
        ),
        "wujihand:captureRateHz": (
            Sdf.ValueTypeNames.Double,
            profile.capture.rate_hz,
        ),
        "wujihand:opticalFrameConvention": (
            Sdf.ValueTypeNames.String,
            "ros_optical_x_right_y_down_z_forward",
        ),
    }
    for name, (value_type, value) in custom.items():
        prim.CreateAttribute(name, value_type).Set(value)
    return path


def _mass_snapshot(prim: Any) -> MassPropertiesSnapshot:
    from pxr import UsdPhysics

    api = UsdPhysics.MassAPI(prim)
    center = api.GetCenterOfMassAttr().Get()
    inertia = api.GetDiagonalInertiaAttr().Get()
    axes = api.GetPrincipalAxesAttr().Get()
    imaginary = None if axes is None else axes.GetImaginary()
    return MassPropertiesSnapshot(
        mass_kg=(None if api.GetMassAttr().Get() is None else float(api.GetMassAttr().Get())),
        center_of_mass_m=(
            None if center is None else cast(Vector3, tuple(float(center[index]) for index in range(3)))
        ),
        diagonal_inertia_kg_m2=(
            None if inertia is None else cast(Vector3, tuple(float(inertia[index]) for index in range(3)))
        ),
        principal_axes_wxyz=(
            None
            if axes is None or imaginary is None
            else (
                float(axes.GetReal()),
                float(imaginary[0]),
                float(imaginary[1]),
                float(imaginary[2]),
            )
        ),
    )


def _author_collision_proxy(
    stage: Any,
    *,
    root_path: str,
    proxy: CompoundCollisionProxy,
) -> tuple[str, ...]:
    from pxr import Gf, UsdGeom, UsdPhysics

    UsdGeom.Xform.Define(stage, root_path)
    result: list[str] = []
    for primitive in proxy.primitives:
        path = f"{root_path}/{primitive.name}"
        if isinstance(primitive, CollisionBox):
            schema = UsdGeom.Cube.Define(stage, path)
            schema.CreateSizeAttr(1.0)
            _set_pose(
                schema.GetPrim(),
                RigidTransform(primitive.center_m, primitive.rotation),
            )
            UsdGeom.Xformable(schema.GetPrim()).AddScaleOp(
                UsdGeom.XformOp.PrecisionDouble
            ).Set(Gf.Vec3d(*primitive.size_m))
        else:
            direction = cast(
                Vector3,
                tuple(primitive.end_m[index] - primitive.start_m[index] for index in range(3)),
            )
            length = math.sqrt(sum(value * value for value in direction))
            center = cast(
                Vector3,
                tuple(
                    (primitive.start_m[index] + primitive.end_m[index]) / 2.0
                    for index in range(3)
                ),
            )
            schema = UsdGeom.Capsule.Define(stage, path)
            schema.CreateAxisAttr(UsdGeom.Tokens.z)
            schema.CreateRadiusAttr(primitive.radius_m)
            schema.CreateHeightAttr(length)
            matrix = Gf.Matrix4d(1.0)
            matrix.SetRotate(Gf.Quatd(*_capsule_quat_wxyz(direction)))
            matrix.SetTranslateOnly(Gf.Vec3d(*center))
            xformable = UsdGeom.Xformable(schema.GetPrim())
            xformable.ClearXformOpOrder()
            xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix)
        schema.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(schema.GetPrim())
        result.append(path)
    return tuple(result)


def materialize_isaac_d405_wrist_rigs(
    stage: Any,
    *,
    runtimes: tuple[D405WristRigRuntime, ...],
    hand_base_paths: Mapping[str, str],
    collision_mode: WristRigCollisionMode,
) -> tuple[D405WristRigHandles, ...]:
    """Author passive visuals and selected child-shape collision before reset."""

    from pxr import UsdGeom

    if collision_mode not in {"none", "mount", "all"}:
        raise ValueError("collision_mode must be none, mount or all")
    result: list[D405WristRigHandles] = []
    for runtime in runtimes:
        hand_base_path = hand_base_paths.get(runtime.side)
        if hand_base_path is None or hand_base_path != runtime.hand_prim_path + (
            "/l_base_link" if runtime.side == "left" else "/r_base_link"
        ):
            raise RuntimeError(f"{runtime.side} Hand 2 base path differs from wrist-rig plan")
        hand_base = stage.GetPrimAtPath(hand_base_path)
        if not hand_base.IsValid() or hand_base.IsInstance() or hand_base.IsInstanceProxy():
            raise RuntimeError(f"invalid Hand 2 base for wrist rig: {hand_base_path}")
        mass_before = _mass_snapshot(hand_base)
        root_path = f"{hand_base_path}/D405WristRig"
        if stage.GetPrimAtPath(root_path).IsValid():
            raise RuntimeError(f"wrist-rig root already exists: {root_path}")
        UsdGeom.Xform.Define(stage, root_path)
        mount_visual_path = _author_visual_mesh(
            stage,
            path=f"{root_path}/MountV2Visual",
            source_path=runtime.mount_visual_path,
            expected_sha256=runtime.mount_visual_sha256,
            color=(0.11, 0.32, 0.72),
        )
        camera_root = UsdGeom.Xform.Define(stage, f"{root_path}/D405Housing")
        _set_pose(camera_root.GetPrim(), runtime.body_in_hand)
        camera_visual_path = _author_visual_mesh(
            stage,
            path=f"{root_path}/D405Housing/Visual",
            source_path=runtime.camera_visual_path,
            expected_sha256=runtime.camera_visual_sha256,
            color=(0.08, 0.09, 0.105),
        )
        camera_prim_path = _author_synthetic_camera(
            stage,
            path=f"{root_path}/D405Housing/OpticalCamera",
            runtime=runtime,
        )
        mount_collision_paths = (
            _author_collision_proxy(
                stage,
                root_path=f"{root_path}/MountCollision",
                proxy=runtime.mount_collision,
            )
            if collision_mode in {"mount", "all"}
            else ()
        )
        camera_collision_paths = (
            _author_collision_proxy(
                stage,
                root_path=f"{root_path}/D405Housing/Collision",
                proxy=runtime.camera_collision,
            )
            if collision_mode == "all"
            else ()
        )
        mass_after = _mass_snapshot(hand_base)
        if mass_after != mass_before:
            raise RuntimeError(f"{runtime.side} Hand 2 authored mass properties changed")
        result.append(
            D405WristRigHandles(
                side=runtime.side,
                root_path=root_path,
                mount_visual_path=mount_visual_path,
                camera_visual_path=camera_visual_path,
                camera_prim_path=camera_prim_path,
                mount_collision_paths=mount_collision_paths,
                camera_collision_paths=camera_collision_paths,
                hand_base_mass_before=mass_before,
                hand_base_mass_after=mass_after,
            )
        )
    return tuple(result)


__all__ = [
    "CollisionBox",
    "CollisionCapsuleSegment",
    "CompoundCollisionProxy",
    "D405WristRigHandles",
    "D405WristRigRuntime",
    "MassPropertiesSnapshot",
    "RigidTransform",
    "WristRigCollisionMode",
    "load_compound_collision_proxy",
    "materialize_isaac_d405_wrist_rigs",
    "resolve_d405_wrist_rig_runtimes",
]
