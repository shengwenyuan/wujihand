"""Deterministic mesh and collision-proxy helpers for the D405 wrist rig."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
import struct
from typing import cast


Vector3 = tuple[float, float, float]
Triangle = tuple[Vector3, Vector3, Vector3]
Matrix3 = tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True, slots=True)
class MeshAudit:
    """Topology and bounds derived independently from STL facet metadata."""

    triangle_count: int
    welded_vertex_count: int
    body_count: int
    shared_edge_component_count: int
    watertight: bool
    winding_consistent: bool
    non_manifold_edge_count: int
    degenerate_triangle_count: int
    bounds_mm: tuple[Vector3, Vector3]

    def to_mapping(self) -> dict[str, object]:
        return {
            "triangle_count": self.triangle_count,
            "welded_vertex_count": self.welded_vertex_count,
            "body_count": self.body_count,
            "shared_edge_component_count": self.shared_edge_component_count,
            "watertight": self.watertight,
            "winding_consistent": self.winding_consistent,
            "non_manifold_edge_count": self.non_manifold_edge_count,
            "degenerate_triangle_count": self.degenerate_triangle_count,
            "bounds_mm": [list(self.bounds_mm[0]), list(self.bounds_mm[1])],
        }


@dataclass(frozen=True, slots=True)
class ProxyAudit:
    """Independent visual coverage and intentional-gap checks for a proxy."""

    primitive_count: int
    covered_vertex_fraction: float
    clear_points_preserved: bool
    uncovered_clear_points_mm: tuple[Vector3, ...]

    def to_mapping(self) -> dict[str, object]:
        return {
            "primitive_count": self.primitive_count,
            "covered_vertex_fraction": self.covered_vertex_fraction,
            "clear_points_preserved": self.clear_points_preserved,
            "uncovered_clear_points_mm": [
                list(point) for point in self.uncovered_clear_points_mm
            ],
        }


class _DisjointSet:
    def __init__(self, size: int) -> None:
        self._parent = list(range(size))
        self._rank = [0] * size

    def find(self, item: int) -> int:
        parent = self._parent[item]
        if parent != item:
            self._parent[item] = self.find(parent)
        return self._parent[item]

    def union(self, left: int, right: int) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return
        if self._rank[left_root] < self._rank[right_root]:
            left_root, right_root = right_root, left_root
        self._parent[right_root] = left_root
        if self._rank[left_root] == self._rank[right_root]:
            self._rank[left_root] += 1


def load_stl_triangles(path: str | Path) -> tuple[Triangle, ...]:
    """Load a size-valid binary STL or an OpenSCAD ASCII STL."""

    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + count * 50:
            triangles: list[Triangle] = []
            for index in range(count):
                values = struct.unpack_from("<12fH", data, 84 + index * 50)
                points = tuple(
                    cast(Vector3, tuple(float(value) for value in values[start : start + 3]))
                    for start in (3, 6, 9)
                )
                triangles.append(cast(Triangle, points))
            return tuple(triangles)

    triangles = []
    vertices: list[Vector3] = []
    for raw_line in data.decode("utf-8").splitlines():
        fields = raw_line.strip().split()
        if len(fields) != 4 or fields[0] != "vertex":
            continue
        vertices.append((float(fields[1]), float(fields[2]), float(fields[3])))
        if len(vertices) == 3:
            triangles.append((vertices[0], vertices[1], vertices[2]))
            vertices.clear()
    if vertices or not triangles:
        raise ValueError(f"invalid or empty STL: {path}")
    return tuple(triangles)


def write_binary_stl(
    path: str | Path,
    triangles: Sequence[Triangle],
    *,
    header: str,
) -> None:
    """Write stable binary STL bytes after canonical face ordering."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    ordered = sorted((_canonical_triangle(triangle) for triangle in triangles))
    with destination.open("wb") as stream:
        stream.write(header.encode("ascii", errors="strict")[:80].ljust(80, b"\0"))
        stream.write(struct.pack("<I", len(ordered)))
        for triangle in ordered:
            stream.write(struct.pack("<3f", *_unit_normal(triangle)))
            for point in triangle:
                stream.write(struct.pack("<3f", *point))
            stream.write(struct.pack("<H", 0))


def transform_triangles(
    triangles: Sequence[Triangle],
    *,
    rotation: Matrix3,
    translation_mm: Vector3 = (0.0, 0.0, 0.0),
    reverse_winding: bool = False,
) -> tuple[Triangle, ...]:
    """Apply one affine transform and explicitly repair reflected winding."""

    transformed: list[Triangle] = []
    for triangle in triangles:
        points = tuple(
            _add(_matrix_vector(rotation, point), translation_mm) for point in triangle
        )
        if reverse_winding:
            points = (points[0], points[2], points[1])
        transformed.append(cast(Triangle, points))
    return tuple(transformed)


def audit_mesh(
    triangles: Sequence[Triangle],
    *,
    weld_tolerance_mm: float = 1e-5,
) -> MeshAudit:
    """Cross-check welded-vertex bodies, shared-edge bodies and watertightness."""

    if not triangles:
        raise ValueError("mesh must contain at least one triangle")
    if not math.isfinite(weld_tolerance_mm) or weld_tolerance_mm <= 0.0:
        raise ValueError("weld tolerance must be finite and positive")

    vertex_ids: dict[tuple[int, int, int], int] = {}
    vertices: list[Vector3] = []
    faces: list[tuple[int, int, int]] = []
    degenerate_count = 0
    for triangle in triangles:
        face: list[int] = []
        for point in triangle:
            if not all(math.isfinite(value) for value in point):
                raise ValueError("mesh contains a non-finite vertex")
            key = cast(
                tuple[int, int, int],
                tuple(round(value / weld_tolerance_mm) for value in point),
            )
            vertex_id = vertex_ids.get(key)
            if vertex_id is None:
                vertex_id = len(vertices)
                vertex_ids[key] = vertex_id
                vertices.append(point)
            face.append(vertex_id)
        typed_face = cast(tuple[int, int, int], tuple(face))
        if len(set(typed_face)) < 3 or _triangle_area(triangle) <= 1e-12:
            degenerate_count += 1
        faces.append(typed_face)

    vertex_sets = _DisjointSet(len(vertices))
    edge_to_faces: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(list)
    for face_index, (a, b, c) in enumerate(faces):
        vertex_sets.union(a, b)
        vertex_sets.union(b, c)
        for start, end in ((a, b), (b, c), (c, a)):
            edge_to_faces[min(start, end), max(start, end)].append(
                (face_index, 1 if start < end else -1)
            )

    face_sets = _DisjointSet(len(faces))
    for uses in edge_to_faces.values():
        for (left, _), (right, _) in zip(uses, uses[1:], strict=False):
            face_sets.union(left, right)

    non_manifold = sum(len(uses) != 2 for uses in edge_to_faces.values())
    winding_consistent = all(
        len(uses) == 2 and uses[0][1] != uses[1][1] for uses in edge_to_faces.values()
    )
    lower = cast(
        Vector3,
        tuple(min(point[axis] for point in vertices) for axis in range(3)),
    )
    upper = cast(
        Vector3,
        tuple(max(point[axis] for point in vertices) for axis in range(3)),
    )
    return MeshAudit(
        triangle_count=len(faces),
        welded_vertex_count=len(vertices),
        body_count=len({vertex_sets.find(index) for index in range(len(vertices))}),
        shared_edge_component_count=len(
            {face_sets.find(index) for index in range(len(faces))}
        ),
        watertight=non_manifold == 0,
        winding_consistent=winding_consistent,
        non_manifold_edge_count=non_manifold,
        degenerate_triangle_count=degenerate_count,
        bounds_mm=(lower, upper),
    )


def audit_collision_proxy(
    triangles: Sequence[Triangle],
    proxy: Mapping[str, object],
    *,
    coverage_tolerance_mm: float = 0.75,
) -> ProxyAudit:
    """Measure proxy coverage separately from visual mesh connectivity."""

    raw_primitives = proxy.get("primitives")
    raw_clear_points = proxy.get("required_clear_points_mm", [])
    if not isinstance(raw_primitives, list) or not raw_primitives:
        raise ValueError("collision proxy must contain a non-empty primitives list")
    if not isinstance(raw_clear_points, list):
        raise ValueError("required_clear_points_mm must be a list")
    primitives = tuple(_mapping(value, field="collision primitive") for value in raw_primitives)
    clear_points = tuple(
        _vector3(value, field="required clear point") for value in raw_clear_points
    )
    vertices = {
        (round(point[0], 5), round(point[1], 5), round(point[2], 5))
        for triangle in triangles
        for point in triangle
    }
    covered = sum(
        any(_primitive_contains(point, primitive, coverage_tolerance_mm) for primitive in primitives)
        for point in vertices
    )
    preserved = tuple(
        point
        for point in clear_points
        if not any(_primitive_contains(point, primitive, 0.0) for primitive in primitives)
    )
    return ProxyAudit(
        primitive_count=len(primitives),
        covered_vertex_fraction=covered / len(vertices),
        clear_points_preserved=len(preserved) == len(clear_points),
        uncovered_clear_points_mm=preserved,
    )


def rotation_matrix_z_y(azimuth_deg: float, tilt_deg: float) -> Matrix3:
    """Return the SCAD camera transform ``Rz(azimuth) * Ry(tilt)``."""

    azimuth = math.radians(azimuth_deg)
    tilt = math.radians(tilt_deg)
    cz, sz = math.cos(azimuth), math.sin(azimuth)
    cy, sy = math.cos(tilt), math.sin(tilt)
    return (
        (cz * cy, -sz, cz * sy),
        (sz * cy, cz, sz * sy),
        (-sy, 0.0, cy),
    )


def mirror_proper_body_rotation(rotation: Matrix3) -> Matrix3:
    """Conjugate a body rotation by the Hand2 XZ reflection plane."""

    mirror: Vector3 = (1.0, -1.0, 1.0)
    return cast(
        Matrix3,
        tuple(
            cast(
                Vector3,
                tuple(mirror[row] * rotation[row][column] * mirror[column] for column in range(3)),
            )
            for row in range(3)
        ),
    )


def optical_frame_contract(
    *,
    body_rotation: Matrix3,
    body_translation_mm: Vector3,
    optical_origin_from_rear_mm: Vector3,
) -> dict[str, object]:
    """Build right and mirrored-left optical frames without reflection rotations."""

    local_forward: Vector3 = (0.0, 0.0, 1.0)
    local_up: Vector3 = (-1.0, 0.0, 0.0)
    local_right = _cross(local_forward, local_up)
    right_origin = _add(
        body_translation_mm,
        _matrix_vector(body_rotation, optical_origin_from_rear_mm),
    )
    right_forward = _matrix_vector(body_rotation, local_forward)
    right_up = _matrix_vector(body_rotation, local_up)
    right_right = _matrix_vector(body_rotation, local_right)

    left_body_rotation = mirror_proper_body_rotation(body_rotation)
    left_body_translation = _mirror_y(body_translation_mm)
    left_origin = _mirror_y(right_origin)
    left_forward = _mirror_y(right_forward)
    left_up = _mirror_y(right_up)
    left_right = _scale(_mirror_y(right_right), -1.0)

    right_optical_rotation = _columns(right_right, _scale(right_up, -1.0), right_forward)
    left_optical_rotation = _columns(left_right, _scale(left_up, -1.0), left_forward)
    right_local_rotation = _matrix_multiply(
        _matrix_transpose(body_rotation), right_optical_rotation
    )
    left_local_rotation = _matrix_multiply(
        _matrix_transpose(left_body_rotation), left_optical_rotation
    )
    right_local_origin = _matrix_vector(
        _matrix_transpose(body_rotation), _subtract(right_origin, body_translation_mm)
    )
    left_local_origin = _matrix_vector(
        _matrix_transpose(left_body_rotation),
        _subtract(left_origin, left_body_translation),
    )
    return {
        "right": {
            "body_translation_in_hand_mm": list(body_translation_mm),
            "body_rotation_in_hand": _matrix_lists(body_rotation),
            "optical_origin_in_hand_mm": list(right_origin),
            "optical_rotation_in_hand": _matrix_lists(right_optical_rotation),
            "rear_mount_to_optical": {
                "translation_mm": list(right_local_origin),
                "rotation": _matrix_lists(right_local_rotation),
            },
            "determinants": {
                "body": determinant(body_rotation),
                "optical": determinant(right_optical_rotation),
            },
        },
        "left": {
            "body_translation_in_hand_mm": list(left_body_translation),
            "body_rotation_in_hand": _matrix_lists(left_body_rotation),
            "optical_origin_in_hand_mm": list(left_origin),
            "optical_rotation_in_hand": _matrix_lists(left_optical_rotation),
            "rear_mount_to_optical": {
                "translation_mm": list(left_local_origin),
                "rotation": _matrix_lists(left_local_rotation),
            },
            "determinants": {
                "body": determinant(left_body_rotation),
                "optical": determinant(left_optical_rotation),
            },
        },
    }


def determinant(matrix: Matrix3) -> float:
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def _canonical_triangle(triangle: Triangle) -> Triangle:
    rounded = cast(
        Triangle,
        tuple(cast(Vector3, tuple(round(value, 6) for value in point)) for point in triangle),
    )
    start = min(range(3), key=lambda index: rounded[index])
    return (rounded[start], rounded[(start + 1) % 3], rounded[(start + 2) % 3])


def _unit_normal(triangle: Triangle) -> Vector3:
    normal = _cross(_subtract(triangle[1], triangle[0]), _subtract(triangle[2], triangle[0]))
    magnitude = math.sqrt(sum(value * value for value in normal))
    if magnitude <= 1e-15:
        return (0.0, 0.0, 1.0)
    return cast(Vector3, tuple(value / magnitude for value in normal))


def _triangle_area(triangle: Triangle) -> float:
    cross = _cross(_subtract(triangle[1], triangle[0]), _subtract(triangle[2], triangle[0]))
    return 0.5 * math.sqrt(sum(value * value for value in cross))


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    return cast(Mapping[str, object], value)


def _vector3(value: object, *, field: str) -> Vector3:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{field} must contain three numbers")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} must be finite")
    return cast(Vector3, result)


def _primitive_contains(
    point: Vector3,
    primitive: Mapping[str, object],
    tolerance_mm: float,
) -> bool:
    kind = primitive.get("type")
    if kind == "box":
        center = _vector3(primitive.get("center_mm"), field="box center_mm")
        size = _vector3(primitive.get("size_mm"), field="box size_mm")
        raw_rotation = primitive.get("rotation")
        rotation = (
            _identity_matrix()
            if raw_rotation is None
            else _matrix3(raw_rotation, field="box rotation")
        )
        local = _matrix_vector(_matrix_transpose(rotation), _subtract(point, center))
        return all(
            abs(local[axis]) <= size[axis] / 2.0 + tolerance_mm for axis in range(3)
        )
    if kind == "capsule_segment":
        start = _vector3(primitive.get("start_mm"), field="capsule start_mm")
        end = _vector3(primitive.get("end_mm"), field="capsule end_mm")
        radius = float(cast(float | int | str, primitive.get("radius_mm")))
        return _point_segment_distance(point, start, end) <= radius + tolerance_mm
    raise ValueError(f"unsupported collision primitive type: {kind!r}")


def _matrix3(value: object, *, field: str) -> Matrix3:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{field} must contain three rows")
    rows = tuple(_vector3(row, field=f"{field} row") for row in value)
    return cast(Matrix3, rows)


def _point_segment_distance(point: Vector3, start: Vector3, end: Vector3) -> float:
    segment = _subtract(end, start)
    length_squared = sum(value * value for value in segment)
    if length_squared <= 1e-15:
        delta = _subtract(point, start)
    else:
        projection = sum(
            (point[axis] - start[axis]) * segment[axis] for axis in range(3)
        ) / length_squared
        fraction = min(1.0, max(0.0, projection))
        nearest = _add(start, _scale(segment, fraction))
        delta = _subtract(point, nearest)
    return math.sqrt(sum(value * value for value in delta))


def _identity_matrix() -> Matrix3:
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return cast(
        Vector3,
        tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3)),
    )


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


def _matrix_transpose(matrix: Matrix3) -> Matrix3:
    return cast(
        Matrix3,
        tuple(cast(Vector3, tuple(matrix[column][row] for column in range(3))) for row in range(3)),
    )


def _matrix_lists(matrix: Matrix3) -> list[list[float]]:
    return [list(row) for row in matrix]


def _columns(first: Vector3, second: Vector3, third: Vector3) -> Matrix3:
    return cast(
        Matrix3,
        tuple((first[row], second[row], third[row]) for row in range(3)),
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return cast(Vector3, tuple(left[index] + right[index] for index in range(3)))


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return cast(Vector3, tuple(left[index] - right[index] for index in range(3)))


def _scale(vector: Vector3, scale: float) -> Vector3:
    return cast(Vector3, tuple(value * scale for value in vector))


def _mirror_y(vector: Vector3) -> Vector3:
    return (vector[0], -vector[1], vector[2])


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


__all__ = [
    "Matrix3",
    "MeshAudit",
    "ProxyAudit",
    "Triangle",
    "Vector3",
    "audit_collision_proxy",
    "audit_mesh",
    "determinant",
    "load_stl_triangles",
    "mirror_proper_body_rotation",
    "optical_frame_contract",
    "rotation_matrix_z_y",
    "transform_triangles",
    "write_binary_stl",
]
