"""Visual-only Hand 2 wrist-mount geometry for Isaac inspection."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import struct
from typing import Any, cast


MOUNT_INSPECTION_CONFIG_ID = "nero_hand2_beta1_gemini305_wrist_mount_v1"
OVERLAY_NAME = "Gemini305WristMountInspection"


@dataclass(frozen=True, slots=True)
class NeroHand2Gemini305MountConfig:
    """Frozen values used for the versioned v1 inspection candidate."""

    config_id: str = MOUNT_INSPECTION_CONFIG_ID
    side: str = "right"
    scad_source: str = (
        "hardware/camera_mounts/nero_hand2_beta1_gemini305/"
        "nero_hand2_beta1_gemini305_wrist_mount_v1.scad"
    )
    mount_file_sha256: str = "e6db31e7784fc2bc6760ab96bbc0e9aa2baa02e71d80e34fb579784a51b24da1"
    mount_geometry_sha256: str = "60dba54d756765c7292bc44bc0c2e281cf3375c487e616416a53c0b2ef82f1e8"
    aligned_camera_mesh_sha256: str = (
        "11698236dcf2dd714c720e446e1326763e6024376dda52002652450dfec4d59d"
    )
    dorsal_sign: int = -1
    camera_center_dorsal_offset_mm: float = 70.0
    camera_rear_plane_z_mm: float = 42.0
    camera_tilt_deg: float = 16.0
    camera_plate_thickness_mm: float = 3.2
    camera_body_size_mm: tuple[float, float, float] = (42.0, 42.0, 23.0)
    camera_rear_hole_pitch_mm: float = 20.0
    camera_maximum_screw_insertion_mm: float = 4.8
    acceptance_projection_origin_from_rear_mm: tuple[float, float, float] = (
        0.0,
        9.0,
        24.0,
    )
    color_render_aspect: tuple[int, int] = (16, 10)
    official_color_hfov_deg: float = 94.0
    acceptance_color_hfov_deg: float = 140.0
    assembly_eye_hand_base_mm: tuple[float, float, float] = (-150.0, -150.0, -130.0)
    assembly_target_hand_base_mm: tuple[float, float, float] = (-30.0, 0.0, 10.0)

    def __post_init__(self) -> None:
        if self.side != "right" or self.dorsal_sign not in {-1, 1}:
            raise ValueError("the v1 config must target one valid right-hand dorsal side")
        positive = (
            *self.camera_body_size_mm,
            self.camera_center_dorsal_offset_mm,
            self.camera_plate_thickness_mm,
            self.camera_rear_hole_pitch_mm,
            self.camera_maximum_screw_insertion_mm,
            self.official_color_hfov_deg,
            self.acceptance_color_hfov_deg,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("mount and camera dimensions must be finite and positive")
        if not math.isfinite(self.camera_rear_plane_z_mm) or not math.isfinite(
            self.camera_tilt_deg
        ):
            raise ValueError("camera placement must be finite")
        if self.acceptance_color_hfov_deg >= 179.0:
            raise ValueError("acceptance horizontal FOV must be below 179 degrees")

    @property
    def camera_translation_m(self) -> tuple[float, float, float]:
        return (
            self.dorsal_sign * self.camera_center_dorsal_offset_mm * 0.001,
            0.0,
            self.camera_rear_plane_z_mm * 0.001,
        )

    @property
    def camera_rotation_y_deg(self) -> float:
        return -self.dorsal_sign * self.camera_tilt_deg

    @property
    def acceptance_projection_origin_m(self) -> tuple[float, float, float]:
        """Synthetic wide-angle pinhole, placed ahead of the visual housing."""

        x_mm, y_mm, z_mm = self.acceptance_projection_origin_from_rear_mm
        return (
            x_mm * 0.001,
            y_mm * 0.001,
            (self.camera_plate_thickness_mm + z_mm) * 0.001,
        )

    @property
    def acceptance_color_vfov_deg(self) -> float:
        width, height = self.color_render_aspect
        horizontal = math.radians(self.acceptance_color_hfov_deg)
        return math.degrees(2.0 * math.atan((height / width) * math.tan(horizontal / 2.0)))


MOUNT_V1_CONFIG = NeroHand2Gemini305MountConfig()


@dataclass(frozen=True, slots=True)
class StlMesh:
    """STL triangles converted from source millimetres to stage metres."""

    points_m: tuple[tuple[float, float, float], ...]
    normals: tuple[tuple[float, float, float], ...]
    triangle_count: int
    encoding: str
    bounds_mm: tuple[tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True, slots=True)
class NeroHand2Gemini305OverlayHandles:
    """Authored prim paths returned to the runtime composition root."""

    overlay_root_path: str
    mount_mesh_path: str
    camera_root_path: str
    camera_body_path: str | None
    camera_mesh_path: str | None
    color_camera_path: str
    rear_fastener_paths: tuple[str, str]
    visual_aid_paths: tuple[str, ...]
    camera_geometry_kind: str


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _triangle_normal(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    c: tuple[float, float, float],
) -> tuple[float, float, float]:
    ux, uy, uz = (b[index] - a[index] for index in range(3))
    vx, vy, vz = (c[index] - a[index] for index in range(3))
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (0.0, 0.0, 1.0) if norm <= 1e-15 else (nx / norm, ny / norm, nz / norm)


def _mesh_from_triangles(
    triangles_mm: Sequence[
        tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ]
    ],
    *,
    encoding: str,
) -> StlMesh:
    if not triangles_mm:
        raise ValueError("STL contains no triangles")
    points_m: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for triangle in triangles_mm:
        for vertex in triangle:
            if not all(math.isfinite(value) for value in vertex):
                raise ValueError("STL contains a non-finite vertex")
            points_m.append(
                cast(tuple[float, float, float], tuple(value * 0.001 for value in vertex))
            )
            for axis, value in enumerate(vertex):
                lower[axis] = min(lower[axis], value)
                upper[axis] = max(upper[axis], value)
        normals.append(_triangle_normal(*triangle))
    return StlMesh(
        points_m=tuple(points_m),
        normals=tuple(normals),
        triangle_count=len(triangles_mm),
        encoding=encoding,
        bounds_mm=(
            cast(tuple[float, float, float], tuple(lower)),
            cast(tuple[float, float, float], tuple(upper)),
        ),
    )


def _parse_binary_stl(data: bytes, triangle_count: int) -> StlMesh:
    triangles = []
    for index in range(triangle_count):
        values = struct.unpack_from("<12fH", data, 84 + index * 50)
        triangles.append(
            (
                cast(tuple[float, float, float], tuple(values[3:6])),
                cast(tuple[float, float, float], tuple(values[6:9])),
                cast(tuple[float, float, float], tuple(values[9:12])),
            )
        )
    return _mesh_from_triangles(triangles, encoding="binary")


def _parse_ascii_stl(data: bytes) -> StlMesh:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("STL is neither size-valid binary nor UTF-8 ASCII") from exc
    vertices: list[tuple[float, float, float]] = []
    for raw_line in text.splitlines():
        words = raw_line.strip().split()
        if not words or words[0].lower() != "vertex":
            continue
        if len(words) != 4:
            raise ValueError(f"malformed ASCII STL vertex: {raw_line!r}")
        try:
            vertices.append(
                cast(tuple[float, float, float], tuple(float(word) for word in words[1:]))
            )
        except ValueError as exc:
            raise ValueError(f"malformed ASCII STL vertex: {raw_line!r}") from exc
    if len(vertices) % 3:
        raise ValueError("ASCII STL vertex count is not divisible by three")
    triangles = [
        cast(
            tuple[
                tuple[float, float, float],
                tuple[float, float, float],
                tuple[float, float, float],
            ],
            tuple(vertices[index : index + 3]),
        )
        for index in range(0, len(vertices), 3)
    ]
    return _mesh_from_triangles(triangles, encoding="ascii")


def load_stl_mesh_mm(path: str | Path) -> StlMesh:
    """Load ASCII or binary STL coordinates expressed in millimetres."""

    data = Path(path).read_bytes()
    if len(data) < 15:
        raise ValueError("STL is too short")
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + triangle_count * 50:
            return _parse_binary_stl(data, triangle_count)
    return _parse_ascii_stl(data)


def stl_geometry_sha256(mesh: StlMesh) -> str:
    """Hash triangle geometry independent of STL facet and vertex ordering."""

    triangles = sorted(
        tuple(sorted(mesh.points_m[index : index + 3])) for index in range(0, len(mesh.points_m), 3)
    )
    digest = hashlib.sha256()
    for triangle in triangles:
        digest.update(
            struct.pack(
                "<9d",
                *(coordinate for point in triangle for coordinate in point),
            )
        )
    return digest.hexdigest()


def _multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def color_camera_quat_wxyz(dorsal_sign: int) -> tuple[float, float, float, float]:
    """Map USD camera -Z to Gemini +Z and image down toward the hand."""

    if dorsal_sign not in {-1, 1}:
        raise ValueError("dorsal_sign must be -1 or 1")
    roll = math.radians(dorsal_sign * 90.0)
    return _multiply_quaternions(
        (0.0, 0.0, 1.0, 0.0),
        (math.cos(roll / 2.0), 0.0, 0.0, math.sin(roll / 2.0)),
    )


def _set_appearance(
    schema: Any,
    color: tuple[float, float, float],
    *,
    opacity: float = 1.0,
) -> None:
    from pxr import Gf  # type: ignore[import-not-found]

    schema.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    schema.CreateDisplayOpacityAttr([float(opacity)])


def _set_pose(
    prim: Any,
    translation_m: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
) -> None:
    from pxr import Gf, UsdGeom

    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(Gf.Quatd(*quat_wxyz))
    matrix.SetTranslateOnly(Gf.Vec3d(*translation_m))
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble).Set(matrix)


def _author_mesh(
    stage: Any,
    path: str,
    mesh_data: StlMesh,
    color: tuple[float, float, float],
) -> str:
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in mesh_data.points_m])
    mesh.CreateFaceVertexCountsAttr([3] * mesh_data.triangle_count)
    mesh.CreateFaceVertexIndicesAttr(list(range(mesh_data.triangle_count * 3)))
    mesh.CreateNormalsAttr([Gf.Vec3f(*normal) for normal in mesh_data.normals])
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    lower_mm, upper_mm = mesh_data.bounds_mm
    mesh.CreateExtentAttr(
        [
            Gf.Vec3f(*(value * 0.001 for value in lower_mm)),
            Gf.Vec3f(*(value * 0.001 for value in upper_mm)),
        ]
    )
    _set_appearance(mesh, color)
    return path


def _author_cube(
    stage: Any,
    path: str,
    size_m: tuple[float, float, float],
    center_m: tuple[float, float, float],
    color: tuple[float, float, float],
    *,
    opacity: float = 1.0,
) -> str:
    from pxr import Gf, UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xformable = UsdGeom.Xformable(cube.GetPrim())
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*center_m))
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*size_m))
    _set_appearance(cube, color, opacity=opacity)
    return path


def _author_cylinder(
    stage: Any,
    path: str,
    radius_m: float,
    height_m: float,
    center_m: tuple[float, float, float],
    color: tuple[float, float, float],
) -> str:
    from pxr import Gf, UsdGeom

    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(UsdGeom.Tokens.z)
    cylinder.CreateRadiusAttr(radius_m)
    cylinder.CreateHeightAttr(height_m)
    UsdGeom.Xformable(cylinder.GetPrim()).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*center_m)
    )
    _set_appearance(cylinder, color)
    return path


def author_nero_hand2_gemini305_mount_overlay(
    stage: Any,
    *,
    hand_base_path: str,
    config: NeroHand2Gemini305MountConfig,
    mount_mesh: StlMesh,
    camera_mesh: StlMesh | None,
    show_camera_visual_aids: bool,
) -> NeroHand2Gemini305OverlayHandles:
    """Attach a visual-only mount and camera beneath the resolved Hand 2 base."""

    from pxr import Gf, UsdGeom

    hand_base = stage.GetPrimAtPath(hand_base_path)
    if not hand_base_path.startswith("/") or not hand_base.IsValid():
        raise RuntimeError(f"invalid Hand 2 base prim: {hand_base_path}")
    if hand_base.IsInstance() or hand_base.IsInstanceProxy():
        raise RuntimeError(f"Hand 2 base cannot accept a child overlay: {hand_base_path}")
    overlay_root = f"{hand_base_path}/{OVERLAY_NAME}"
    if stage.GetPrimAtPath(overlay_root).IsValid():
        raise RuntimeError(f"inspection overlay already exists: {overlay_root}")
    UsdGeom.Xform.Define(stage, overlay_root)
    mount_path = _author_mesh(
        stage,
        f"{overlay_root}/MountMesh",
        mount_mesh,
        (0.13, 0.36, 0.72),
    )

    camera_root = f"{overlay_root}/Gemini305"
    camera_xform = UsdGeom.Xform.Define(stage, camera_root)
    half_angle = math.radians(config.camera_rotation_y_deg) / 2.0
    _set_pose(
        camera_xform.GetPrim(),
        config.camera_translation_m,
        (math.cos(half_angle), 0.0, math.sin(half_angle), 0.0),
    )
    plate_m = config.camera_plate_thickness_mm * 0.001
    body_size_m = cast(
        tuple[float, float, float],
        tuple(value * 0.001 for value in config.camera_body_size_mm),
    )
    body_path: str | None = None
    camera_mesh_path: str | None = None
    if camera_mesh is None:
        body_path = _author_cube(
            stage,
            f"{camera_root}/Body42x42x23mm",
            body_size_m,
            (0.0, 0.0, plate_m + body_size_m[2] / 2.0),
            (0.055, 0.06, 0.07),
        )
        geometry_kind = "drawing-dimension proxy"
    else:
        mesh_root = UsdGeom.Xform.Define(stage, f"{camera_root}/PrivateAlignedCadMesh")
        UsdGeom.Xformable(mesh_root.GetPrim()).AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(0.0, 0.0, plate_m)
        )
        camera_mesh_path = _author_mesh(
            stage,
            f"{camera_root}/PrivateAlignedCadMesh/Mesh",
            camera_mesh,
            (0.105, 0.115, 0.125),
        )
        geometry_kind = "caller-supplied aligned visual mesh"

    insertion_m = min(config.camera_maximum_screw_insertion_mm, 2.8) * 0.001
    fastener_height_m = plate_m + insertion_m
    rear_fasteners = cast(
        tuple[str, str],
        tuple(
            _author_cylinder(
                stage,
                f"{camera_root}/RearFastener{label}",
                0.0015,
                fastener_height_m,
                (
                    0.0,
                    sign * config.camera_rear_hole_pitch_mm * 0.0005,
                    fastener_height_m / 2.0,
                ),
                (0.58, 0.61, 0.66),
            )
            for label, sign in (("NegativeY", -1), ("PositiveY", 1))
        ),
    )
    visual_aids: list[str] = []
    if show_camera_visual_aids:
        front_z_m = plate_m + body_size_m[2]
        visual_aids.append(
            _author_cube(
                stage,
                f"{camera_root}/FrontGlass",
                (body_size_m[0] - 0.003, body_size_m[1] - 0.003, 0.0008),
                (0.0, 0.0, front_z_m + 0.0004),
                (0.025, 0.045, 0.065),
                opacity=0.92,
            )
        )
        for label, lens_y_m in (("NegativeY", -0.009), ("PositiveY", 0.009)):
            visual_aids.extend(
                (
                    _author_cylinder(
                        stage,
                        f"{camera_root}/Lens{label}Bezel",
                        0.0052,
                        0.0014,
                        (0.0, lens_y_m, front_z_m + 0.0014),
                        (0.008, 0.01, 0.012),
                    ),
                    _author_cylinder(
                        stage,
                        f"{camera_root}/Lens{label}Glass",
                        0.0037,
                        0.0008,
                        (0.0, lens_y_m, front_z_m + 0.0022),
                        (0.025, 0.16, 0.22),
                    ),
                )
            )

    color_camera_path = f"{camera_root}/ColorOpticalFrame"
    color_camera = UsdGeom.Camera.Define(stage, color_camera_path)
    horizontal_aperture_mm = 16.0
    aspect_width, aspect_height = config.color_render_aspect
    vertical_aperture_mm = horizontal_aperture_mm * aspect_height / aspect_width
    focal_length_mm = horizontal_aperture_mm / (
        2.0 * math.tan(math.radians(config.acceptance_color_hfov_deg) / 2.0)
    )
    color_camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    color_camera.CreateHorizontalApertureAttr(horizontal_aperture_mm)
    color_camera.CreateVerticalApertureAttr(vertical_aperture_mm)
    color_camera.CreateFocalLengthAttr(focal_length_mm)
    color_camera.CreateClippingRangeAttr(Gf.Vec2f(0.005, 10.0))
    _set_pose(
        color_camera.GetPrim(),
        config.acceptance_projection_origin_m,
        color_camera_quat_wxyz(config.dorsal_sign),
    )
    return NeroHand2Gemini305OverlayHandles(
        overlay_root_path=overlay_root,
        mount_mesh_path=mount_path,
        camera_root_path=camera_root,
        camera_body_path=body_path,
        camera_mesh_path=camera_mesh_path,
        color_camera_path=color_camera_path,
        rear_fastener_paths=rear_fasteners,
        visual_aid_paths=tuple(visual_aids),
        camera_geometry_kind=geometry_kind,
    )


def author_inspection_lights(stage: Any, overlay_root: str) -> tuple[str, str]:
    """Add two neutral lights without physics or collision APIs."""

    from pxr import Gf, UsdGeom, UsdLux

    root = f"{overlay_root}/InspectionLights"
    UsdGeom.Xform.Define(stage, root)
    dome_path = f"{root}/Dome"
    dome = UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateIntensityAttr(650.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.84, 1.0))
    key_path = f"{root}/Key"
    key = UsdLux.DistantLight.Define(stage, key_path)
    key.CreateIntensityAttr(900.0)
    key.CreateAngleAttr(0.8)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.90, 0.78))
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(-42.0, 18.0, -28.0)
    )
    return (dome_path, key_path)


__all__ = [
    "MOUNT_V1_CONFIG",
    "MOUNT_INSPECTION_CONFIG_ID",
    "NeroHand2Gemini305MountConfig",
    "NeroHand2Gemini305OverlayHandles",
    "StlMesh",
    "author_inspection_lights",
    "author_nero_hand2_gemini305_mount_overlay",
    "color_camera_quat_wxyz",
    "load_stl_mesh_mm",
    "sha256_file",
    "stl_geometry_sha256",
]
