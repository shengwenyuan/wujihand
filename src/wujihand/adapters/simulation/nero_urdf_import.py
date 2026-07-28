"""Typed, reproducible NERO URDF-to-USD import recipe.

The recipe and validation helpers are importable in the normal project
environment.  Isaac Sim is imported only inside :func:`import_nero_urdf`.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, cast
import xml.etree.ElementTree as ET

import yaml

from wujihand.integrity import sha256_tree

from .nero_model import NERO_CHILD_LINKS, NERO_JOINT_NAMES, NeroModelProfile


NERO_IMPORT_RECIPE_SCHEMA = "wujihand.nero_urdf_import_recipe.v1"
NERO_IMPORT_RECIPE_ID = "agilex_nero_isaac_6_0_1_import_v1"
NERO_IMPORT_STATUS = "provisional_simulation_only"
NERO_IMPORT_ISAAC_VERSION = "6.0.1"
NERO_IMPORTER_EXTENSION_VERSION = "3.11.2"
NERO_ASSET_TRANSFORMER_EXTENSION_VERSION = "1.2.5"
NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION = "1.7.10"
NERO_SOURCE_LOCK_ID = "agilex-agx-arm-urdf"
NERO_SOURCE_COMMIT = "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
NERO_SOURCE_URDF_PATH = "nero/urdf/nero_description.urdf"
NERO_SOURCE_URDF_SHA256 = "c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278"
NERO_SOURCE_MESH_TREE_PATH = "nero/meshes"
NERO_SOURCE_MESH_TREE_SHA256 = "2323405c805cbc4187f451a22d02954fc952a8a2b67b9602119ad26e1ee5031e"
NERO_ROS_PACKAGE_NAME = "agx_arm_description"
NERO_MESH_URI_PREFIX = f"package://{NERO_ROS_PACKAGE_NAME}/agx_arm_urdf/"
NERO_GENERATED_USDA_PATHS = (
    "nero_description.usda",
    "payloads/Physics/mujoco.usda",
    "payloads/Physics/physics.usda",
    "payloads/Physics/physx.usda",
    "payloads/base.usda",
    "payloads/instances.usda",
    "payloads/materials.usda",
    "payloads/robot.usda",
)
NERO_NORMALIZED_USDA_PATHS = tuple(sorted((*NERO_GENERATED_USDA_PATHS, "payloads/geometries.usda")))
NERO_GENERATED_BINARY_GEOMETRY_PATH = "payloads/geometries.usd"
_GENERATED_DOC_START = '    doc = """'
_GENERATED_DOC_END = '"""\n'
_GENERATED_DOC_MARKER = "Generated from Composed Stage of root layer"
_COLLISION_TYPES = frozenset(
    {"Convex Hull", "Convex Decomposition", "Bounding Sphere", "Bounding Cube"}
)
_ROBOT_TYPES = frozenset(
    {
        "Default",
        "End Effector",
        "Manipulator",
        "Humanoid",
        "Wheeled",
        "Holonomic",
        "Quadruped",
        "Mobile Manipulators",
        "Aerial",
    }
)
_NERO_RIGID_BODY_NAMES = ("base_link", *NERO_CHILD_LINKS)
_VECTOR_ABS_TOL = 1e-6
_QUATERNION_ABS_TOL = 1e-6
_INERTIA_ABS_TOL = 1e-9

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    return cast(Mapping[str, object], value)


def _exact_mapping(value: object, *, expected: frozenset[str], field: str) -> Mapping[str, object]:
    data = _mapping(value, field=field)
    actual = frozenset(data)
    if actual != expected:
        raise ValueError(
            f"{field} keys differ: missing={sorted(expected - actual)}, "
            f"unexpected={sorted(actual - expected)}"
        )
    return data


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _bool(value: object, *, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} must be a boolean")
    return value


def _optional_finite_float(value: object, *, field: str) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric or null")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def _project_path(value: object, *, field: str) -> str:
    text = _string(value, field=field)
    path = Path(text)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != text:
        raise ValueError(f"{field} must be a normalized project-relative path")
    return text


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _xml_child(element: ET.Element, tag: str, *, field: str) -> ET.Element:
    child = element.find(tag)
    if child is None:
        raise RuntimeError(f"pinned NERO URDF has no {field}")
    return child


def _xml_vector(
    element: ET.Element,
    attribute: str,
    *,
    field: str,
) -> Vector3:
    raw = element.attrib.get(attribute)
    if raw is None:
        raise RuntimeError(f"pinned NERO URDF has no {field}")
    values = tuple(float(value) for value in raw.split())
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise RuntimeError(f"pinned NERO URDF has invalid {field}: {raw!r}")
    return values


def _origin(element: ET.Element, *, field: str) -> tuple[Vector3, Vector3]:
    origin = _xml_child(element, "origin", field=f"{field}.origin")
    return (
        _xml_vector(origin, "xyz", field=f"{field}.origin.xyz"),
        _xml_vector(origin, "rpy", field=f"{field}.origin.rpy"),
    )


def _rpy_to_quaternion_wxyz(rpy: Vector3) -> Quaternion:
    roll, pitch, yaw = rpy
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


@dataclass(frozen=True, slots=True)
class NeroUrdfInertial:
    """One link's authored inertial facts in the URDF link frame."""

    link_name: str
    mass_kg: float
    center_of_mass_xyz_m: Vector3
    inertia_kg_m2: Matrix3


@dataclass(frozen=True, slots=True)
class NeroUrdfJoint:
    """One revolute joint's authored URDF kinematic and limit facts."""

    name: str
    parent_link: str
    child_link: str
    origin_xyz_m: Vector3
    origin_quaternion_wxyz: Quaternion
    axis_xyz: Vector3
    lower_rad: float
    upper_rad: float
    effort: float
    velocity_rad_s: float


@dataclass(frozen=True, slots=True)
class NeroUrdfFacts:
    """Exact NERO source facts needed to qualify a derived USD."""

    inertials: tuple[NeroUrdfInertial, ...]
    joints: tuple[NeroUrdfJoint, ...]


def load_nero_urdf_facts(path: str | Path) -> NeroUrdfFacts:
    """Parse the fixed NERO inertial and joint facts without Isaac imports."""

    root = ET.parse(Path(path)).getroot()
    if root.tag != "robot" or root.attrib.get("name") != "nero":
        raise RuntimeError("pinned NERO URDF must declare robot name 'nero'")

    links = {link.attrib.get("name", ""): link for link in root.findall("link")}
    if set(links) != {"world", *_NERO_RIGID_BODY_NAMES}:
        raise RuntimeError("pinned NERO URDF links differ from world + base_link + link1..link7")
    inertials: list[NeroUrdfInertial] = []
    for link_name in _NERO_RIGID_BODY_NAMES:
        inertial = _xml_child(
            links[link_name],
            "inertial",
            field=f"link[{link_name}].inertial",
        )
        center, inertial_rpy = _origin(
            inertial,
            field=f"link[{link_name}].inertial",
        )
        if any(not math.isclose(value, 0.0, abs_tol=1e-12) for value in inertial_rpy):
            raise RuntimeError(f"pinned NERO {link_name} inertial rotation is unsupported")
        mass_element = _xml_child(
            inertial,
            "mass",
            field=f"link[{link_name}].inertial.mass",
        )
        mass = float(mass_element.attrib.get("value", "nan"))
        inertia_element = _xml_child(
            inertial,
            "inertia",
            field=f"link[{link_name}].inertial.inertia",
        )
        components = {
            key: float(inertia_element.attrib.get(key, "nan"))
            for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        }
        if (
            not math.isfinite(mass)
            or mass <= 0.0
            or not all(math.isfinite(value) for value in components.values())
        ):
            raise RuntimeError(f"pinned NERO {link_name} inertia is invalid")
        inertials.append(
            NeroUrdfInertial(
                link_name=link_name,
                mass_kg=mass,
                center_of_mass_xyz_m=center,
                inertia_kg_m2=(
                    (components["ixx"], components["ixy"], components["ixz"]),
                    (components["ixy"], components["iyy"], components["iyz"]),
                    (components["ixz"], components["iyz"], components["izz"]),
                ),
            )
        )

    joints_by_name = {joint.attrib.get("name", ""): joint for joint in root.findall("joint")}
    if set(joints_by_name) != {"world_to_base_link", *NERO_JOINT_NAMES}:
        raise RuntimeError(
            "pinned NERO URDF joints differ from world_to_base_link + joint1..joint7"
        )
    fixed = joints_by_name["world_to_base_link"]
    fixed_xyz, fixed_rpy = _origin(fixed, field="joint[world_to_base_link]")
    fixed_parent = _xml_child(
        fixed,
        "parent",
        field="joint[world_to_base_link].parent",
    ).attrib.get("link")
    fixed_child = _xml_child(
        fixed,
        "child",
        field="joint[world_to_base_link].child",
    ).attrib.get("link")
    if (
        fixed.attrib.get("type") != "fixed"
        or fixed_parent != "world"
        or fixed_child != "base_link"
        or fixed_xyz != (0.0, 0.0, 0.0)
        or fixed_rpy != (0.0, 0.0, 0.0)
    ):
        raise RuntimeError("pinned NERO world_to_base_link joint is unsupported")

    joints: list[NeroUrdfJoint] = []
    for index, name in enumerate(NERO_JOINT_NAMES):
        joint = joints_by_name[name]
        if joint.attrib.get("type") != "revolute":
            raise RuntimeError(f"pinned NERO {name} must be revolute")
        origin_xyz, origin_rpy = _origin(joint, field=f"joint[{name}]")
        parent = _xml_child(
            joint,
            "parent",
            field=f"joint[{name}].parent",
        ).attrib.get("link", "")
        child = _xml_child(
            joint,
            "child",
            field=f"joint[{name}].child",
        ).attrib.get("link", "")
        expected_parent = "base_link" if index == 0 else NERO_CHILD_LINKS[index - 1]
        expected_child = NERO_CHILD_LINKS[index]
        if parent != expected_parent or child != expected_child:
            raise RuntimeError(f"pinned NERO {name} topology is unsupported")
        axis = _xml_vector(
            _xml_child(joint, "axis", field=f"joint[{name}].axis"),
            "xyz",
            field=f"joint[{name}].axis.xyz",
        )
        limit = _xml_child(joint, "limit", field=f"joint[{name}].limit")
        values = {
            key: float(limit.attrib.get(key, "nan"))
            for key in ("lower", "upper", "effort", "velocity")
        }
        if (
            axis != (0.0, 0.0, 1.0)
            or not all(math.isfinite(value) for value in values.values())
            or values["lower"] >= values["upper"]
            or values["effort"] <= 0.0
            or values["velocity"] <= 0.0
        ):
            raise RuntimeError(f"pinned NERO {name} source facts are invalid")
        joints.append(
            NeroUrdfJoint(
                name=name,
                parent_link=parent,
                child_link=child,
                origin_xyz_m=origin_xyz,
                origin_quaternion_wxyz=_rpy_to_quaternion_wxyz(origin_rpy),
                axis_xyz=axis,
                lower_rad=values["lower"],
                upper_rad=values["upper"],
                effort=values["effort"],
                velocity_rad_s=values["velocity"],
            )
        )
    return NeroUrdfFacts(inertials=tuple(inertials), joints=tuple(joints))


@dataclass(frozen=True, slots=True)
class NeroUrdfImportOptions:
    """Isaac Sim 6.0.1 URDF importer options frozen by the recipe."""

    merge_fixed_joints: bool
    merge_mesh: bool
    debug_mode: bool
    collision_from_visuals: bool
    collision_type: str
    allow_self_collision: bool
    robot_type: str
    fix_base: bool
    link_density: float | None
    joint_drive_type: str
    joint_target_type: str
    override_joint_stiffness: float
    override_joint_damping: float
    run_asset_transformer: bool
    run_multi_physics_conversion: bool

    def to_mapping(self) -> dict[str, object]:
        return cast(dict[str, object], asdict(self))


@dataclass(frozen=True, slots=True)
class NeroUrdfImportRecipe:
    """Pinned source, importer version, output identity, and import options."""

    recipe_id: str
    status: str
    isaac_version: str
    importer_extension_version: str
    asset_transformer_extension_version: str
    asset_transformer_rules_extension_version: str
    source_lock_id: str
    source_commit: str
    urdf_path: str
    urdf_sha256: str
    mesh_tree_path: str
    mesh_tree_sha256: str
    ros_package_name: str
    output_root: str
    robot_name: str
    options: NeroUrdfImportOptions
    assumptions: tuple[str, ...]

    @property
    def expected_relative_usd_path(self) -> str:
        return f"{self.output_root}/{self.robot_name}/{self.robot_name}.usda"

    def verify_source(self, source_root: str | Path) -> Path:
        root = Path(source_root).resolve()
        if root.name != "agx_arm_urdf":
            raise ValueError(
                "NERO source root directory must be named agx_arm_urdf for "
                "the upstream package:// paths"
            )
        urdf = (root / self.urdf_path).resolve()
        try:
            urdf.relative_to(root)
        except ValueError as exc:
            raise ValueError("NERO URDF path escapes source root") from exc
        if not urdf.is_file():
            raise FileNotFoundError(f"pinned NERO URDF not found: {urdf}")
        actual = _sha256_file(urdf)
        if actual != self.urdf_sha256:
            raise RuntimeError(
                f"pinned NERO URDF SHA-256 mismatch: expected {self.urdf_sha256}, got {actual}"
            )
        mesh_tree = (root / self.mesh_tree_path).resolve()
        try:
            mesh_tree.relative_to(root)
        except ValueError as exc:
            raise ValueError("NERO mesh tree path escapes source root") from exc
        actual_mesh_tree = sha256_tree(mesh_tree)
        if actual_mesh_tree != self.mesh_tree_sha256:
            raise RuntimeError(
                "pinned NERO mesh tree SHA-256 mismatch: "
                f"expected {self.mesh_tree_sha256}, got {actual_mesh_tree}"
            )
        document = ET.parse(urdf).getroot()
        visual_mesh_uris = tuple(
            mesh.attrib.get("filename", "")
            for mesh in document.findall("./link/visual/geometry/mesh")
        )
        collision_mesh_uris = tuple(
            mesh.attrib.get("filename", "")
            for mesh in document.findall("./link/collision/geometry/mesh")
        )
        if (
            len(visual_mesh_uris) != 8
            or len(collision_mesh_uris) != 8
            or len(set(visual_mesh_uris)) != 8
            or len(set(collision_mesh_uris)) != 8
            or set(visual_mesh_uris) & set(collision_mesh_uris)
            or any(not uri.endswith(".dae") for uri in visual_mesh_uris)
            or any(not uri.endswith(".stl") for uri in collision_mesh_uris)
        ):
            raise RuntimeError(
                "pinned NERO URDF must reference 8 unique visual DAE and "
                "8 distinct collision STL meshes"
            )
        mesh_uris = (*visual_mesh_uris, *collision_mesh_uris)
        for uri in mesh_uris:
            if not uri.startswith(NERO_MESH_URI_PREFIX):
                raise RuntimeError(f"unexpected NERO mesh URI: {uri!r}")
            mesh = (root / uri.removeprefix(NERO_MESH_URI_PREFIX)).resolve()
            try:
                mesh.relative_to(root)
            except ValueError as exc:
                raise RuntimeError(f"NERO mesh URI escapes source root: {uri!r}") from exc
            if not mesh.is_file():
                raise FileNotFoundError(f"pinned NERO mesh not found: {mesh}")
        load_nero_urdf_facts(urdf)
        return urdf


def load_nero_urdf_import_recipe(path: str | Path) -> NeroUrdfImportRecipe:
    """Load the single approved NV-2 NERO import recipe."""

    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "recipe_id",
                "status",
                "isaac",
                "source",
                "output",
                "options",
                "assumptions",
            }
        ),
        field="NERO import recipe",
    )
    if data["schema"] != NERO_IMPORT_RECIPE_SCHEMA:
        raise ValueError(f"unsupported NERO import recipe schema: {data['schema']!r}")
    if data["recipe_id"] != NERO_IMPORT_RECIPE_ID:
        raise ValueError(f"unexpected NERO import recipe ID: {data['recipe_id']!r}")
    if data["status"] != NERO_IMPORT_STATUS:
        raise ValueError(f"unexpected NERO import status: {data['status']!r}")

    isaac = _exact_mapping(
        data["isaac"],
        expected=frozenset(
            {
                "version",
                "urdf_importer_extension_version",
                "asset_transformer_extension_version",
                "asset_transformer_rules_extension_version",
            }
        ),
        field="NERO import recipe.isaac",
    )
    if isaac["version"] != NERO_IMPORT_ISAAC_VERSION:
        raise ValueError("NERO import recipe must target Isaac Sim 6.0.1")
    if isaac["urdf_importer_extension_version"] != NERO_IMPORTER_EXTENSION_VERSION:
        raise ValueError("NERO import recipe has an unapproved importer extension")
    if isaac["asset_transformer_extension_version"] != NERO_ASSET_TRANSFORMER_EXTENSION_VERSION:
        raise ValueError("NERO import recipe has an unapproved asset transformer")
    if (
        isaac["asset_transformer_rules_extension_version"]
        != NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION
    ):
        raise ValueError("NERO import recipe has unapproved asset transformer rules")

    source = _exact_mapping(
        data["source"],
        expected=frozenset(
            {
                "lock_id",
                "commit",
                "urdf_path",
                "urdf_sha256",
                "mesh_tree_path",
                "mesh_tree_sha256",
                "ros_package_name",
            }
        ),
        field="NERO import recipe.source",
    )
    source_expectations = {
        "lock_id": NERO_SOURCE_LOCK_ID,
        "commit": NERO_SOURCE_COMMIT,
        "urdf_path": NERO_SOURCE_URDF_PATH,
        "urdf_sha256": NERO_SOURCE_URDF_SHA256,
        "mesh_tree_path": NERO_SOURCE_MESH_TREE_PATH,
        "mesh_tree_sha256": NERO_SOURCE_MESH_TREE_SHA256,
        "ros_package_name": NERO_ROS_PACKAGE_NAME,
    }
    if any(source[key] != value for key, value in source_expectations.items()):
        raise ValueError("NERO import recipe source differs from the approved pin")

    output = _exact_mapping(
        data["output"],
        expected=frozenset({"root", "robot_name"}),
        field="NERO import recipe.output",
    )
    output_root = _project_path(output["root"], field="NERO import recipe.output.root")
    robot_name = _string(output["robot_name"], field="NERO import recipe.output.robot_name")
    if robot_name != "nero_description":
        raise ValueError("NERO import recipe output robot_name must be nero_description")

    raw_options = _exact_mapping(
        data["options"],
        expected=frozenset(
            {
                "merge_fixed_joints",
                "merge_mesh",
                "debug_mode",
                "collision_from_visuals",
                "collision_type",
                "allow_self_collision",
                "robot_type",
                "fix_base",
                "link_density",
                "joint_drive_type",
                "joint_target_type",
                "override_joint_stiffness",
                "override_joint_damping",
                "run_asset_transformer",
                "run_multi_physics_conversion",
            }
        ),
        field="NERO import recipe.options",
    )
    collision_type = _string(
        raw_options["collision_type"],
        field="NERO import recipe.options.collision_type",
    )
    if collision_type not in _COLLISION_TYPES:
        raise ValueError("NERO import recipe collision_type is unsupported")
    robot_type = _string(raw_options["robot_type"], field="NERO import recipe.options.robot_type")
    if robot_type not in _ROBOT_TYPES:
        raise ValueError("NERO import recipe robot_type is unsupported")
    drive_type = _string(
        raw_options["joint_drive_type"],
        field="NERO import recipe.options.joint_drive_type",
    )
    if drive_type not in {"force", "acceleration"}:
        raise ValueError("NERO import recipe joint_drive_type is unsupported")
    target_type = _string(
        raw_options["joint_target_type"],
        field="NERO import recipe.options.joint_target_type",
    )
    if target_type not in {"none", "position", "velocity"}:
        raise ValueError("NERO import recipe joint_target_type is unsupported")
    stiffness = _optional_finite_float(
        raw_options["override_joint_stiffness"],
        field="NERO import recipe.options.override_joint_stiffness",
    )
    damping = _optional_finite_float(
        raw_options["override_joint_damping"],
        field="NERO import recipe.options.override_joint_damping",
    )
    if stiffness is None or stiffness <= 0.0:
        raise ValueError("NERO import recipe stiffness must be positive")
    if damping is None or damping < 0.0:
        raise ValueError("NERO import recipe damping must be non-negative")
    link_density = _optional_finite_float(
        raw_options["link_density"],
        field="NERO import recipe.options.link_density",
    )
    if link_density is not None and link_density <= 0.0:
        raise ValueError("NERO import recipe link_density must be positive or null")

    options = NeroUrdfImportOptions(
        merge_fixed_joints=_bool(
            raw_options["merge_fixed_joints"],
            field="NERO import recipe.options.merge_fixed_joints",
        ),
        merge_mesh=_bool(raw_options["merge_mesh"], field="NERO import recipe.options.merge_mesh"),
        debug_mode=_bool(raw_options["debug_mode"], field="NERO import recipe.options.debug_mode"),
        collision_from_visuals=_bool(
            raw_options["collision_from_visuals"],
            field="NERO import recipe.options.collision_from_visuals",
        ),
        collision_type=collision_type,
        allow_self_collision=_bool(
            raw_options["allow_self_collision"],
            field="NERO import recipe.options.allow_self_collision",
        ),
        robot_type=robot_type,
        fix_base=_bool(raw_options["fix_base"], field="NERO import recipe.options.fix_base"),
        link_density=link_density,
        joint_drive_type=drive_type,
        joint_target_type=target_type,
        override_joint_stiffness=stiffness,
        override_joint_damping=damping,
        run_asset_transformer=_bool(
            raw_options["run_asset_transformer"],
            field="NERO import recipe.options.run_asset_transformer",
        ),
        run_multi_physics_conversion=_bool(
            raw_options["run_multi_physics_conversion"],
            field="NERO import recipe.options.run_multi_physics_conversion",
        ),
    )
    if options.merge_fixed_joints or options.merge_mesh:
        raise ValueError("NERO import must preserve source links and mesh identities")
    if options.collision_from_visuals:
        raise ValueError("NERO import must preserve authored collision geometry")
    if options.allow_self_collision:
        raise ValueError("NERO self collision remains unqualified in NV-2 bring-up")
    if not options.fix_base:
        raise ValueError("NERO twin must have a fixed base")
    if options.joint_drive_type != "force" or options.joint_target_type != "position":
        raise ValueError("NERO import must use force position drives")
    if not options.run_asset_transformer or not options.run_multi_physics_conversion:
        raise ValueError("NERO import must use the approved Isaac 6 conversion path")

    raw_assumptions = data["assumptions"]
    if not isinstance(raw_assumptions, list):
        raise ValueError("NERO import recipe.assumptions must be a list")
    assumptions = tuple(
        _string(item, field=f"NERO import recipe.assumptions[{index}]")
        for index, item in enumerate(raw_assumptions)
    )
    if not assumptions or len(set(assumptions)) != len(assumptions):
        raise ValueError("NERO import recipe assumptions must be non-empty and unique")

    return NeroUrdfImportRecipe(
        recipe_id=NERO_IMPORT_RECIPE_ID,
        status=NERO_IMPORT_STATUS,
        isaac_version=NERO_IMPORT_ISAAC_VERSION,
        importer_extension_version=NERO_IMPORTER_EXTENSION_VERSION,
        asset_transformer_extension_version=(NERO_ASSET_TRANSFORMER_EXTENSION_VERSION),
        asset_transformer_rules_extension_version=(NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION),
        source_lock_id=NERO_SOURCE_LOCK_ID,
        source_commit=NERO_SOURCE_COMMIT,
        urdf_path=NERO_SOURCE_URDF_PATH,
        urdf_sha256=NERO_SOURCE_URDF_SHA256,
        mesh_tree_path=NERO_SOURCE_MESH_TREE_PATH,
        mesh_tree_sha256=NERO_SOURCE_MESH_TREE_SHA256,
        ros_package_name=NERO_ROS_PACKAGE_NAME,
        output_root=output_root,
        robot_name=robot_name,
        options=options,
        assumptions=assumptions,
    )


def import_nero_urdf(
    recipe: NeroUrdfImportRecipe,
    *,
    source_root: str | Path,
    output_root: str | Path,
) -> Path:
    """Import the pinned NERO URDF in an already-started Isaac Sim process."""

    from isaacsim.asset.importer.urdf import (  # type: ignore[import-not-found]
        URDFImporter,
        URDFImporterConfig,
    )

    urdf = recipe.verify_source(source_root)
    output = Path(output_root).resolve()
    expected_package = output / recipe.robot_name
    if expected_package.exists():
        raise FileExistsError(
            f"refusing a non-deterministic suffixed import; remove or archive "
            f"the existing package first: {expected_package}"
        )
    output.mkdir(parents=True, exist_ok=True)
    config = URDFImporterConfig(
        urdf_path=str(urdf),
        usd_path=str(output),
        ros_package_paths=[
            {
                "name": recipe.ros_package_name,
                "path": str(Path(source_root).resolve().parent),
            }
        ],
        **recipe.options.to_mapping(),
    )
    imported = Path(URDFImporter(config).import_urdf()).resolve()
    expected = expected_package / f"{recipe.robot_name}.usda"
    if imported != expected or not imported.is_file():
        raise RuntimeError(
            f"Isaac importer returned unexpected NERO USD: {imported}; expected {expected}"
        )
    return imported


def _remove_generated_doc(text: str, *, relative: str) -> str:
    start = text.find(_GENERATED_DOC_START)
    if start < 0:
        raise RuntimeError(f"imported NERO USDA has no generated doc block: {relative}")
    end = text.find(_GENERATED_DOC_END, start + len(_GENERATED_DOC_START))
    if end < 0:
        raise RuntimeError(f"imported NERO USDA has an unterminated doc block: {relative}")
    block_end = end + len(_GENERATED_DOC_END)
    block = text[start:block_end]
    if _GENERATED_DOC_MARKER not in block:
        raise RuntimeError(f"refusing to remove non-importer documentation from {relative}")
    if text.find(_GENERATED_DOC_START, block_end) >= 0:
        raise RuntimeError(f"imported NERO USDA has multiple doc blocks: {relative}")
    return text[:start] + text[block_end:]


def normalize_imported_nero_text_layers(
    package_root: str | Path,
) -> tuple[str, ...]:
    """Normalize the fixed nine-layer text package without importing Isaac.

    This is the pure, unit-tested part of the production normalizer.  It
    rewrites the geometry reference after the binary layer has been exported
    to USDA, then removes only asset-transformer ``doc`` metadata containing
    host-specific output and temporary paths.
    """

    root = Path(package_root).resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"imported NERO package not found: {root}")
    actual = tuple(sorted(path.relative_to(root).as_posix() for path in root.rglob("*.usda")))
    if actual != NERO_NORMALIZED_USDA_PATHS:
        raise RuntimeError(
            "normalized NERO USDA layer set differs from the fixed recipe: "
            f"expected={list(NERO_NORMALIZED_USDA_PATHS)}, actual={list(actual)}"
        )

    instances = root / "payloads/instances.usda"
    instances_text = instances.read_text(encoding="utf-8")
    old_reference = "@./geometries.usd@"
    new_reference = "@./geometries.usda@"
    reference_count = instances_text.count(old_reference)
    if reference_count < 1 or new_reference in instances_text:
        raise RuntimeError("generated NERO instances layer has unexpected geometry references")
    instances.write_text(
        instances_text.replace(old_reference, new_reference),
        encoding="utf-8",
    )

    normalized: list[str] = []
    for relative in NERO_NORMALIZED_USDA_PATHS:
        path = root / relative
        text = path.read_text(encoding="utf-8")
        path.write_text(
            _remove_generated_doc(text, relative=relative),
            encoding="utf-8",
        )
        normalized.append(relative)
    return tuple(normalized)


def normalize_imported_nero_package(package_root: str | Path) -> tuple[str, ...]:
    """Normalize the package and explicitly disable PhysX self-collision."""

    from pxr import PhysxSchema, Sdf, Usd  # type: ignore[import-not-found]

    root = Path(package_root).resolve()
    binary_geometry = root / NERO_GENERATED_BINARY_GEOMETRY_PATH
    text_geometry = root / "payloads/geometries.usda"
    if not binary_geometry.is_file() or text_geometry.exists():
        raise RuntimeError(
            "imported NERO package must contain only the generated binary "
            "payloads/geometries.usd layer"
        )
    layer = Sdf.Layer.FindOrOpen(str(binary_geometry))
    if layer is None:
        raise RuntimeError(f"failed to open generated geometry layer: {binary_geometry}")
    if not layer.Export(str(text_geometry)):
        raise RuntimeError(f"failed to export deterministic geometry layer: {text_geometry}")
    binary_geometry.unlink()

    normalized = normalize_imported_nero_text_layers(root)
    root_layer = root / "nero_description.usda"
    stage = Usd.Stage.Open(str(root_layer))
    if stage is None:
        raise RuntimeError(f"failed to open normalized NERO USD: {root_layer}")
    articulation_prim = stage.GetPrimAtPath("/nero/Geometry/world")
    if not articulation_prim.IsValid():
        raise RuntimeError("normalized NERO USD has no articulation root")
    stage.SetEditTarget(stage.GetRootLayer())
    physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(articulation_prim)
    self_collision = physx_articulation.CreateEnabledSelfCollisionsAttr(False)
    if not self_collision.Set(False):
        raise RuntimeError("failed to author explicit PhysX self-collision policy")
    stage.GetRootLayer().Save()
    return normalized


def _usd_vector3(value: object, *, field: str) -> Vector3:
    if value is None:
        raise RuntimeError(f"imported NERO has no {field}")
    values = tuple(float(component) for component in cast(Any, value))
    if len(values) != 3 or not all(math.isfinite(component) for component in values):
        raise RuntimeError(f"imported NERO has invalid {field}")
    return values


def _usd_quaternion_wxyz(value: object, *, field: str) -> Quaternion:
    if value is None:
        raise RuntimeError(f"imported NERO has no {field}")
    quaternion = cast(Any, value)
    imaginary = tuple(float(component) for component in quaternion.GetImaginary())
    values = (float(quaternion.GetReal()), *imaginary)
    if len(values) != 4 or not all(math.isfinite(component) for component in values):
        raise RuntimeError(f"imported NERO has invalid {field}")
    norm = math.sqrt(sum(component * component for component in values))
    if not math.isclose(norm, 1.0, abs_tol=1e-5):
        raise RuntimeError(f"imported NERO has non-unit {field}")
    return values


def _vectors_close(left: Vector3, right: Vector3, *, abs_tol: float) -> bool:
    return all(math.isclose(left[index], right[index], abs_tol=abs_tol) for index in range(3))


def _quaternions_equivalent(left: Quaternion, right: Quaternion) -> bool:
    dot = sum(left[index] * right[index] for index in range(4))
    return math.isclose(abs(dot), 1.0, abs_tol=_QUATERNION_ABS_TOL)


def _quaternion_rotation_matrix(quaternion: Quaternion) -> Matrix3:
    w, x, y, z = quaternion
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def _inertia_from_principal_axes(
    diagonal: Vector3,
    principal_axes: Quaternion,
) -> Matrix3:
    rotation = _quaternion_rotation_matrix(principal_axes)
    return cast(
        Matrix3,
        tuple(
            cast(
                Vector3,
                tuple(
                    sum(rotation[k][i] * diagonal[k] * rotation[k][j] for k in range(3))
                    for j in range(3)
                ),
            )
            for i in range(3)
        ),
    )


def _matrix_max_abs_error(left: Matrix3, right: Matrix3) -> float:
    return max(
        abs(left[row][column] - right[row][column]) for row in range(3) for column in range(3)
    )


def inspect_imported_nero_usd(
    path: str | Path,
    *,
    recipe: NeroUrdfImportRecipe,
    model_profile: NeroModelProfile,
    source_facts: NeroUrdfFacts,
) -> dict[str, object]:
    """Inspect the composed USD and reject topology drift before it is locked."""

    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

    usd_path = Path(path).resolve()
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"failed to open imported NERO USD: {usd_path}")
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isclose(meters_per_unit, 1.0, abs_tol=1e-12):
        raise RuntimeError(f"imported NERO stage must use meters, got {meters_per_unit}")
    default_prim = stage.GetDefaultPrim()
    if not default_prim.IsValid():
        raise RuntimeError("imported NERO stage has no default prim")
    if default_prim.GetName() != "nero":
        raise RuntimeError(
            f"imported NERO default prim must be 'nero', got {default_prim.GetName()!r}"
        )
    source_joints = {joint.name: joint for joint in source_facts.joints}
    source_inertials = {inertial.link_name: inertial for inertial in source_facts.inertials}
    if tuple(source_joints) != NERO_JOINT_NAMES:
        raise RuntimeError("NERO source joint facts differ from joint1..joint7")
    if tuple(source_inertials) != _NERO_RIGID_BODY_NAMES:
        raise RuntimeError("NERO source inertial facts differ from base_link + link1..link7")

    revolute_joints: dict[str, object] = {}
    rigid_body_paths: dict[str, str] = {}
    articulation_roots: list[str] = []
    fixed_joint_paths: list[str] = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if prim.IsA(UsdPhysics.RevoluteJoint):
            if name in revolute_joints:
                raise RuntimeError(f"imported NERO joint name is ambiguous: {name}")
            revolute_joints[name] = UsdPhysics.RevoluteJoint(prim)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if name in rigid_body_paths:
                raise RuntimeError(f"imported NERO rigid body name is ambiguous: {name}")
            rigid_body_paths[name] = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.FixedJoint):
            fixed_joint_paths.append(str(prim.GetPath()))

    if tuple(sorted(revolute_joints)) != tuple(sorted(NERO_JOINT_NAMES)):
        raise RuntimeError(
            f"imported NERO revolute joints differ from joint1..joint7: {sorted(revolute_joints)}"
        )
    required_rigid_bodies = {"base_link", *NERO_CHILD_LINKS}
    if set(rigid_body_paths) != required_rigid_bodies:
        raise RuntimeError(
            "imported NERO rigid bodies differ from base_link + link1..link7: "
            f"missing={sorted(required_rigid_bodies - set(rigid_body_paths))}, "
            f"unexpected={sorted(set(rigid_body_paths) - required_rigid_bodies)}"
        )
    expected_articulation_root = f"{default_prim.GetPath()}/Geometry/world"
    if articulation_roots != [expected_articulation_root]:
        raise RuntimeError(
            "imported NERO articulation root differs from the fixed import topology: "
            f"{articulation_roots}"
        )
    expected_fixed_joint = f"{default_prim.GetPath()}/Physics/world_to_base_link"
    if fixed_joint_paths != [expected_fixed_joint]:
        raise RuntimeError(
            "imported fixed-base NERO must have one world_to_base_link joint, "
            f"got {fixed_joint_paths}"
        )

    joint_axes: dict[str, str] = {}
    joint_limits_deg: dict[str, list[float]] = {}
    joint_drives: dict[str, dict[str, object]] = {}
    joint_origins: dict[str, dict[str, object]] = {}
    for index, name in enumerate(NERO_JOINT_NAMES):
        joint = cast(Any, revolute_joints[name])
        joint_prim = joint.GetPrim()
        source_joint = source_joints[name]
        axis = str(joint.GetAxisAttr().Get())
        if axis != "Z":
            raise RuntimeError(f"imported NERO {name} axis must be Z, got {axis!r}")
        if source_joint.axis_xyz != model_profile.axes_xyz[index]:
            raise RuntimeError(f"NERO source/profile {name} axes differ")
        lower = joint.GetLowerLimitAttr().Get()
        upper = joint.GetUpperLimitAttr().Get()
        if lower is None or upper is None:
            raise RuntimeError(f"imported NERO {name} has no finite angular limits")
        low = float(lower)
        high = float(upper)
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise RuntimeError(f"imported NERO {name} angular limits are invalid")
        if not math.isclose(
            source_joint.lower_rad,
            model_profile.layout.lower[index],
            abs_tol=1e-12,
        ) or not math.isclose(
            source_joint.upper_rad,
            model_profile.layout.upper[index],
            abs_tol=1e-12,
        ):
            raise RuntimeError(f"NERO source/profile {name} limits differ")
        expected_low = math.degrees(source_joint.lower_rad)
        expected_high = math.degrees(source_joint.upper_rad)
        if not math.isclose(low, expected_low, abs_tol=1e-3):
            raise RuntimeError(
                f"imported NERO {name} lower limit differs from q7 profile: "
                f"expected {expected_low}, got {low}"
            )
        if not math.isclose(high, expected_high, abs_tol=1e-3):
            raise RuntimeError(
                f"imported NERO {name} upper limit differs from q7 profile: "
                f"expected {expected_high}, got {high}"
            )

        body0 = [str(target) for target in joint.GetBody0Rel().GetTargets()]
        body1 = [str(target) for target in joint.GetBody1Rel().GetTargets()]
        if body0 != [rigid_body_paths[source_joint.parent_link]]:
            raise RuntimeError(f"imported NERO {name} has unexpected body0: {body0}")
        if body1 != [rigid_body_paths[source_joint.child_link]]:
            raise RuntimeError(f"imported NERO {name} has unexpected body1: {body1}")
        if joint.GetJointEnabledAttr().Get() is False:
            raise RuntimeError(f"imported NERO {name} is disabled")

        local_position_0 = _usd_vector3(
            joint.GetLocalPos0Attr().Get(),
            field=f"{name}.localPos0",
        )
        local_position_1 = _usd_vector3(
            joint.GetLocalPos1Attr().Get(),
            field=f"{name}.localPos1",
        )
        local_rotation_0 = _usd_quaternion_wxyz(
            joint.GetLocalRot0Attr().Get(),
            field=f"{name}.localRot0",
        )
        local_rotation_1 = _usd_quaternion_wxyz(
            joint.GetLocalRot1Attr().Get(),
            field=f"{name}.localRot1",
        )
        if not _vectors_close(
            local_position_0,
            source_joint.origin_xyz_m,
            abs_tol=_VECTOR_ABS_TOL,
        ):
            raise RuntimeError(f"imported NERO {name} localPos0 differs from the URDF origin")
        if not _vectors_close(
            local_position_1,
            (0.0, 0.0, 0.0),
            abs_tol=_VECTOR_ABS_TOL,
        ):
            raise RuntimeError(f"imported NERO {name} localPos1 must be zero")
        if not _quaternions_equivalent(
            local_rotation_0,
            source_joint.origin_quaternion_wxyz,
        ):
            raise RuntimeError(f"imported NERO {name} localRot0 differs from the URDF origin")
        if not _quaternions_equivalent(
            local_rotation_1,
            (1.0, 0.0, 0.0, 0.0),
        ):
            raise RuntimeError(f"imported NERO {name} localRot1 must be identity")

        drive_type = joint_prim.GetAttribute("drive:angular:physics:type").Get()
        stiffness = joint_prim.GetAttribute("drive:angular:physics:stiffness").Get()
        damping = joint_prim.GetAttribute("drive:angular:physics:damping").Get()
        max_force = joint_prim.GetAttribute("drive:angular:physics:maxForce").Get()
        max_velocity = joint_prim.GetAttribute("physxJoint:maxJointVelocity").Get()
        target_position_attribute = joint_prim.GetAttribute("drive:angular:physics:targetPosition")
        target_position = (
            target_position_attribute.Get() if target_position_attribute.IsValid() else None
        )
        target_position_authored = (
            target_position_attribute.IsValid()
            and target_position_attribute.HasAuthoredValueOpinion()
        )
        drive_values = (stiffness, damping, max_force, max_velocity)
        if drive_type != "force" or any(value is None for value in drive_values):
            raise RuntimeError(f"imported NERO {name} drive is incomplete")
        numeric_drive = tuple(float(value) for value in drive_values)
        if not all(math.isfinite(value) for value in numeric_drive):
            raise RuntimeError(f"imported NERO {name} drive contains non-finite values")
        stiffness_value, damping_value, max_force_value, max_velocity_value = numeric_drive
        expected_stiffness = math.radians(recipe.options.override_joint_stiffness)
        expected_damping = math.radians(recipe.options.override_joint_damping)
        if not math.isclose(
            source_joint.velocity_rad_s,
            model_profile.urdf_velocity_rad_s[index],
            abs_tol=1e-12,
        ):
            raise RuntimeError(f"NERO source/profile {name} velocity limits differ")
        expected_max_velocity = math.degrees(source_joint.velocity_rad_s)
        if not math.isclose(stiffness_value, expected_stiffness, abs_tol=1e-5):
            raise RuntimeError(f"imported NERO {name} stiffness differs from recipe")
        if not math.isclose(damping_value, expected_damping, abs_tol=1e-5):
            raise RuntimeError(f"imported NERO {name} damping differs from recipe")
        if not math.isclose(
            max_force_value,
            source_joint.effort,
            abs_tol=1e-6,
        ):
            raise RuntimeError(f"imported NERO {name} max force differs from URDF effort")
        if not math.isclose(
            max_velocity_value,
            expected_max_velocity,
            abs_tol=1e-3,
        ):
            raise RuntimeError(f"imported NERO {name} max velocity differs from the URDF")
        joint_axes[name] = axis
        joint_limits_deg[name] = [low, high]
        joint_origins[name] = {
            "local_position_0_m": list(local_position_0),
            "local_rotation_0_wxyz": list(local_rotation_0),
            "local_position_1_m": list(local_position_1),
            "local_rotation_1_wxyz": list(local_rotation_1),
            "source_origin_position_m": list(source_joint.origin_xyz_m),
            "source_origin_rotation_wxyz": list(source_joint.origin_quaternion_wxyz),
        }
        joint_drives[name] = {
            "type": str(drive_type),
            "stiffness_per_degree": stiffness_value,
            "damping_per_degree": damping_value,
            "max_force_source_urdf_unqualified": max_force_value,
            "max_velocity_deg_s_source_urdf": max_velocity_value,
            "target_position_deg": (
                float(target_position) if target_position is not None else None
            ),
            "target_position_authored": target_position_authored,
        }

    fixed_joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(expected_fixed_joint))
    fixed_body0 = [str(target) for target in fixed_joint.GetBody0Rel().GetTargets()]
    fixed_body1 = [str(target) for target in fixed_joint.GetBody1Rel().GetTargets()]
    if fixed_body0 != [str(default_prim.GetPath())]:
        raise RuntimeError(f"imported fixed joint does not target the robot root: {fixed_body0}")
    if fixed_body1 != [rigid_body_paths["base_link"]]:
        raise RuntimeError(f"imported fixed joint does not target base_link: {fixed_body1}")
    fixed_local_position_0 = _usd_vector3(
        fixed_joint.GetLocalPos0Attr().Get(),
        field="world_to_base_link.localPos0",
    )
    fixed_local_position_1 = _usd_vector3(
        fixed_joint.GetLocalPos1Attr().Get(),
        field="world_to_base_link.localPos1",
    )
    fixed_local_rotation_0 = _usd_quaternion_wxyz(
        fixed_joint.GetLocalRot0Attr().Get(),
        field="world_to_base_link.localRot0",
    )
    fixed_local_rotation_1 = _usd_quaternion_wxyz(
        fixed_joint.GetLocalRot1Attr().Get(),
        field="world_to_base_link.localRot1",
    )
    if (
        not _vectors_close(
            fixed_local_position_0,
            (0.0, 0.0, 0.0),
            abs_tol=_VECTOR_ABS_TOL,
        )
        or not _vectors_close(
            fixed_local_position_1,
            (0.0, 0.0, 0.0),
            abs_tol=_VECTOR_ABS_TOL,
        )
        or not _quaternions_equivalent(
            fixed_local_rotation_0,
            (1.0, 0.0, 0.0, 0.0),
        )
        or not _quaternions_equivalent(
            fixed_local_rotation_1,
            (1.0, 0.0, 0.0, 0.0),
        )
    ):
        raise RuntimeError("imported fixed joint origin differs from the URDF")

    rigid_body_mass_kg: dict[str, float] = {}
    rigid_body_diagonal_inertia: dict[str, list[float]] = {}
    rigid_body_inertial: dict[str, dict[str, object]] = {}
    for name in _NERO_RIGID_BODY_NAMES:
        prim = stage.GetPrimAtPath(rigid_body_paths[name])
        if not prim.HasAPI(UsdPhysics.MassAPI):
            raise RuntimeError(f"imported NERO {name} has no authored MassAPI")
        mass_api = UsdPhysics.MassAPI(prim)
        mass_attribute = mass_api.GetMassAttr()
        center_attribute = mass_api.GetCenterOfMassAttr()
        diagonal_attribute = mass_api.GetDiagonalInertiaAttr()
        axes_attribute = mass_api.GetPrincipalAxesAttr()
        authored_attributes = (
            mass_attribute,
            center_attribute,
            diagonal_attribute,
            axes_attribute,
        )
        if not all(
            attribute.IsValid() and attribute.HasAuthoredValueOpinion()
            for attribute in authored_attributes
        ):
            raise RuntimeError(f"imported NERO {name} inertia is not fully authored")
        mass = mass_attribute.Get()
        center = _usd_vector3(
            center_attribute.Get(),
            field=f"{name}.centerOfMass",
        )
        diagonal = _usd_vector3(
            diagonal_attribute.Get(),
            field=f"{name}.diagonalInertia",
        )
        principal_axes = _usd_quaternion_wxyz(
            axes_attribute.Get(),
            field=f"{name}.principalAxes",
        )
        if mass is None:
            raise RuntimeError(f"imported NERO {name} has no mass")
        mass_value = float(mass)
        if (
            not math.isfinite(mass_value)
            or mass_value <= 0.0
            or not all(math.isfinite(value) and value > 0.0 for value in diagonal)
        ):
            raise RuntimeError(f"imported NERO {name} inertia is invalid")
        source_inertial = source_inertials[name]
        if not math.isclose(
            mass_value,
            source_inertial.mass_kg,
            abs_tol=_VECTOR_ABS_TOL,
        ):
            raise RuntimeError(f"imported NERO {name} mass differs from the URDF")
        if not _vectors_close(
            center,
            source_inertial.center_of_mass_xyz_m,
            abs_tol=_VECTOR_ABS_TOL,
        ):
            raise RuntimeError(f"imported NERO {name} center of mass differs from the URDF")
        reconstructed = _inertia_from_principal_axes(diagonal, principal_axes)
        inertia_error = _matrix_max_abs_error(
            reconstructed,
            source_inertial.inertia_kg_m2,
        )
        if inertia_error > _INERTIA_ABS_TOL:
            raise RuntimeError(
                f"imported NERO {name} inertia tensor differs from the URDF: "
                f"max_abs_error={inertia_error}"
            )
        rigid_body_mass_kg[name] = mass_value
        rigid_body_diagonal_inertia[name] = list(diagonal)
        rigid_body_inertial[name] = {
            "mass_kg": mass_value,
            "center_of_mass_xyz_m": list(center),
            "diagonal_inertia_kg_m2": list(diagonal),
            "principal_axes_wxyz": list(principal_axes),
            "reconstructed_inertia_kg_m2": [list(row) for row in reconstructed],
            "source_inertia_kg_m2": [list(row) for row in source_inertial.inertia_kg_m2],
            "max_abs_inertia_error": inertia_error,
        }

    collision_paths: list[str] = []
    collision_meshes: dict[str, dict[str, object]] = {}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path_string = str(prim.GetPath())
        collision = UsdPhysics.CollisionAPI(prim)
        if collision.GetCollisionEnabledAttr().Get() is False:
            raise RuntimeError(f"imported NERO collision is disabled: {path_string}")
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            raise RuntimeError(f"imported NERO collision is not a mesh: {path_string}")
        approximation = str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())
        if approximation != "convexHull":
            raise RuntimeError(f"imported NERO collision must use convexHull: {path_string}")
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        extent = mesh.GetExtentAttr().Get()
        if (
            points is None
            or face_vertex_counts is None
            or extent is None
            or len(points) < 4
            or len(face_vertex_counts) < 4
            or len(extent) != 2
            or any(int(count) < 3 for count in face_vertex_counts)
        ):
            raise RuntimeError(f"imported NERO collision mesh is incomplete: {path_string}")
        extent_min = _usd_vector3(
            extent[0],
            field=f"{path_string}.extent_min",
        )
        extent_max = _usd_vector3(
            extent[1],
            field=f"{path_string}.extent_max",
        )
        if any(extent_min[index] >= extent_max[index] for index in range(3)):
            raise RuntimeError(f"imported NERO collision mesh has invalid extent: {path_string}")
        collision_paths.append(path_string)
        collision_meshes[path_string] = {
            "source_geometry": "pinned_urdf_collision_stl",
            "approximation": approximation,
            "point_count": len(points),
            "face_count": len(face_vertex_counts),
            "extent_min_m": list(extent_min),
            "extent_max_m": list(extent_max),
        }
    expected_collision_paths = sorted(
        f"{rigid_body_paths[name]}/{name}_1/{name}_1" for name in ("base_link", *NERO_CHILD_LINKS)
    )
    if sorted(collision_paths) != expected_collision_paths:
        raise RuntimeError("imported NERO collisions differ from one convex hull per rigid body")

    articulation_prim = stage.GetPrimAtPath(expected_articulation_root)
    if not articulation_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        raise RuntimeError("imported NERO articulation has no explicit PhysX articulation policy")
    physx_articulation = PhysxSchema.PhysxArticulationAPI(articulation_prim)
    self_collision_attribute = physx_articulation.GetEnabledSelfCollisionsAttr()
    if (
        not self_collision_attribute.IsValid()
        or not self_collision_attribute.HasAuthoredValueOpinion()
        or self_collision_attribute.Get() is not False
    ):
        raise RuntimeError("imported NERO must explicitly disable PhysX self collision")
    newton_self_collision_attribute = articulation_prim.GetAttribute("newton:selfCollisionEnabled")
    newton_self_collision = (
        newton_self_collision_attribute.Get() if newton_self_collision_attribute.IsValid() else None
    )

    return {
        "usd_file_name": usd_path.name,
        "meters_per_unit": meters_per_unit,
        "default_prim_path": str(default_prim.GetPath()),
        "articulation_root_paths": sorted(articulation_roots),
        "fixed_joint_paths": sorted(fixed_joint_paths),
        "rigid_body_paths": {
            name: rigid_body_paths[name] for name in ("base_link", *NERO_CHILD_LINKS)
        },
        "revolute_joint_paths": {
            name: str(cast(Any, revolute_joints[name]).GetPrim().GetPath())
            for name in NERO_JOINT_NAMES
        },
        "joint_axes": joint_axes,
        "joint_limits_deg": joint_limits_deg,
        "joint_origins": joint_origins,
        "joint_drives": joint_drives,
        "rigid_body_mass_kg": rigid_body_mass_kg,
        "rigid_body_diagonal_inertia": rigid_body_diagonal_inertia,
        "rigid_body_inertial": rigid_body_inertial,
        "collision_paths": sorted(collision_paths),
        "collision_approximation": "convexHull",
        "collision_geometry_source": "pinned_urdf_collision_stl_not_visual_dae",
        "collision_meshes": {path: collision_meshes[path] for path in sorted(collision_meshes)},
        "self_collision": {
            "recipe_enabled": recipe.options.allow_self_collision,
            "physx_api_applied": True,
            "physx_authored": True,
            "physx_enabled": False,
            "newton_enabled": newton_self_collision,
            "effective_for_nv2": False,
        },
    }


def recipe_fingerprint(recipe: NeroUrdfImportRecipe) -> str:
    """Return a stable content fingerprint for generated artifact reports."""

    document = {
        "recipe_id": recipe.recipe_id,
        "status": recipe.status,
        "isaac_version": recipe.isaac_version,
        "importer_extension_version": recipe.importer_extension_version,
        "asset_transformer_extension_version": (recipe.asset_transformer_extension_version),
        "asset_transformer_rules_extension_version": (
            recipe.asset_transformer_rules_extension_version
        ),
        "source_lock_id": recipe.source_lock_id,
        "source_commit": recipe.source_commit,
        "urdf_path": recipe.urdf_path,
        "urdf_sha256": recipe.urdf_sha256,
        "mesh_tree_path": recipe.mesh_tree_path,
        "mesh_tree_sha256": recipe.mesh_tree_sha256,
        "ros_package_name": recipe.ros_package_name,
        "output_root": recipe.output_root,
        "robot_name": recipe.robot_name,
        "options": recipe.options.to_mapping(),
        "assumptions": list(recipe.assumptions),
    }
    encoded = json.dumps(
        document,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


__all__ = [
    "NERO_ASSET_TRANSFORMER_EXTENSION_VERSION",
    "NERO_ASSET_TRANSFORMER_RULES_EXTENSION_VERSION",
    "NERO_IMPORTER_EXTENSION_VERSION",
    "NERO_IMPORT_ISAAC_VERSION",
    "NERO_IMPORT_RECIPE_ID",
    "NERO_IMPORT_RECIPE_SCHEMA",
    "NERO_IMPORT_STATUS",
    "NERO_GENERATED_BINARY_GEOMETRY_PATH",
    "NERO_GENERATED_USDA_PATHS",
    "NERO_NORMALIZED_USDA_PATHS",
    "NERO_SOURCE_COMMIT",
    "NERO_SOURCE_MESH_TREE_PATH",
    "NERO_SOURCE_MESH_TREE_SHA256",
    "NERO_ROS_PACKAGE_NAME",
    "NERO_SOURCE_LOCK_ID",
    "NERO_SOURCE_URDF_PATH",
    "NERO_SOURCE_URDF_SHA256",
    "NeroUrdfFacts",
    "NeroUrdfInertial",
    "NeroUrdfImportOptions",
    "NeroUrdfImportRecipe",
    "NeroUrdfJoint",
    "import_nero_urdf",
    "inspect_imported_nero_usd",
    "load_nero_urdf_facts",
    "load_nero_urdf_import_recipe",
    "normalize_imported_nero_package",
    "normalize_imported_nero_text_layers",
    "recipe_fingerprint",
]
