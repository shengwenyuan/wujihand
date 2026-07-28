"""Load and map the pinned, simulation-only AgileX NERO q7 profile."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, cast

import numpy as np
import numpy.typing as npt
import yaml

from wujihand.domain.joints import FloatArray, JointLayout


NERO_PROFILE_SCHEMA = "wujihand.nero_joint_profile.v1"
NERO_PROFILE_ID = "agilex_nero_q7_provisional_v1"
NERO_LAYOUT_ID = "agilex_nero_q7_v1"
NERO_PRODUCT = "agilex_nero"
NERO_PROFILE_STATUS = "provisional_simulation_pending_device_readback"
NERO_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
NERO_CHILD_LINKS = tuple(f"link{index}" for index in range(1, 8))


def _mapping(value: object, *, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    return cast(Mapping[str, object], value)


def _exact_mapping(
    value: object, *, expected: frozenset[str], field: str
) -> Mapping[str, object]:
    data = _mapping(value, field=field)
    actual = frozenset(data)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise ValueError(
            f"{field} keys differ: missing={missing}, unexpected={unexpected}"
        )
    return data


def _string(value: object, *, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-blank string")
    return value


def _finite_float(value: object, *, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


@dataclass(frozen=True, slots=True)
class NeroModelProfile:
    """Backend-neutral q7 meaning plus provisional simulation provenance."""

    profile_id: str
    status: str
    layout_id: str
    layout: JointLayout
    home_position: FloatArray
    base_frame: str
    tool_flange_frame: str
    child_links: tuple[str, ...]
    axes_xyz: tuple[tuple[float, float, float], ...]
    urdf_velocity_rad_s: tuple[float, ...]
    provenance_json: str
    limit_policy: tuple[tuple[str, str], ...]

    def canonical_to_backend(
        self,
        q7: Sequence[float] | npt.NDArray[np.floating[Any]],
        backend_names: Sequence[str],
    ) -> FloatArray:
        """Reorder canonical joint values into an Isaac articulation order."""

        values = self.layout.validate_vector(q7)
        indices = self.layout.indices_for(backend_names)
        return values[np.asarray(indices, dtype=np.int64)]

    def backend_to_canonical(
        self,
        backend_values: Sequence[float] | npt.NDArray[np.floating[Any]],
        backend_names: Sequence[str],
    ) -> FloatArray:
        """Reorder Isaac feedback into the pinned canonical q7 order."""

        values = np.asarray(backend_values, dtype=np.float64)
        if values.shape != (self.layout.size,):
            raise ValueError(
                f"expected backend joint vector shape {(self.layout.size,)}, "
                f"got {values.shape}"
            )
        if not np.isfinite(values).all():
            raise ValueError("backend joint vector contains NaN or infinity")
        indices = np.asarray(self.layout.indices_for(backend_names), dtype=np.int64)
        canonical = np.empty(self.layout.size, dtype=np.float64)
        canonical[indices] = values
        return self.layout.validate_vector(canonical)

    @property
    def provenance(self) -> Mapping[str, object]:
        value = json.loads(self.provenance_json)
        if not isinstance(value, Mapping):
            raise RuntimeError("profile provenance is not a mapping")
        return cast(Mapping[str, object], value)


def load_nero_model_profile(path: str | Path) -> NeroModelProfile:
    """Load a strict NERO profile without importing Isaac or a vendor SDK."""

    raw: Any = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    data = _exact_mapping(
        raw,
        expected=frozenset(
            {
                "schema",
                "profile_id",
                "product",
                "status",
                "layout_id",
                "units",
                "frames",
                "home_position",
                "limit_policy",
                "provenance",
                "joints",
            }
        ),
        field="NERO profile",
    )
    if data["schema"] != NERO_PROFILE_SCHEMA:
        raise ValueError(f"unsupported NERO profile schema: {data['schema']!r}")
    if data["profile_id"] != NERO_PROFILE_ID:
        raise ValueError(f"unexpected NERO profile ID: {data['profile_id']!r}")
    if data["product"] != NERO_PRODUCT:
        raise ValueError(f"unexpected NERO product: {data['product']!r}")
    if data["status"] != NERO_PROFILE_STATUS:
        raise ValueError(f"unexpected NERO profile status: {data['status']!r}")
    if data["layout_id"] != NERO_LAYOUT_ID:
        raise ValueError(f"unexpected NERO layout ID: {data['layout_id']!r}")

    units = _exact_mapping(
        data["units"],
        expected=frozenset({"position", "velocity"}),
        field="NERO profile.units",
    )
    if units != {"position": "rad", "velocity": "rad_s"}:
        raise ValueError("NERO profile units must be rad and rad_s")
    frames = _exact_mapping(
        data["frames"],
        expected=frozenset({"base", "tool_flange"}),
        field="NERO profile.frames",
    )

    raw_joints = data["joints"]
    if not isinstance(raw_joints, list):
        raise ValueError("NERO profile.joints must be a list")
    joints = [
        _exact_mapping(
            value,
            expected=frozenset(
                {
                    "name",
                    "child_link",
                    "axis_xyz",
                    "lower",
                    "upper",
                    "velocity",
                    "urdf_velocity",
                }
            ),
            field=f"NERO profile.joints[{index}]",
        )
        for index, value in enumerate(raw_joints)
    ]
    names = tuple(
        _string(joint["name"], field=f"NERO profile.joints[{index}].name")
        for index, joint in enumerate(joints)
    )
    child_links = tuple(
        _string(
            joint["child_link"],
            field=f"NERO profile.joints[{index}].child_link",
        )
        for index, joint in enumerate(joints)
    )
    if names != NERO_JOINT_NAMES:
        raise ValueError(f"NERO joints must be ordered as {NERO_JOINT_NAMES}")
    if child_links != NERO_CHILD_LINKS:
        raise ValueError(f"NERO child links must be ordered as {NERO_CHILD_LINKS}")

    axes: list[tuple[float, float, float]] = []
    for index, joint in enumerate(joints):
        raw_axis = joint["axis_xyz"]
        if not isinstance(raw_axis, list) or len(raw_axis) != 3:
            raise ValueError(
                f"NERO profile.joints[{index}].axis_xyz must contain 3 values"
            )
        axis = tuple(
            _finite_float(
                component,
                field=f"NERO profile.joints[{index}].axis_xyz[{component_index}]",
            )
            for component_index, component in enumerate(raw_axis)
        )
        if not np.isclose(np.linalg.norm(axis), 1.0, atol=1e-12):
            raise ValueError(
                f"NERO profile.joints[{index}].axis_xyz must be a unit vector"
            )
        axes.append(cast(tuple[float, float, float], axis))

    lower = tuple(
        _finite_float(joint["lower"], field=f"NERO profile.joints[{index}].lower")
        for index, joint in enumerate(joints)
    )
    upper = tuple(
        _finite_float(joint["upper"], field=f"NERO profile.joints[{index}].upper")
        for index, joint in enumerate(joints)
    )
    velocity = tuple(
        _finite_float(
            joint["velocity"], field=f"NERO profile.joints[{index}].velocity"
        )
        for index, joint in enumerate(joints)
    )
    urdf_velocity = tuple(
        _finite_float(
            joint["urdf_velocity"],
            field=f"NERO profile.joints[{index}].urdf_velocity",
        )
        for index, joint in enumerate(joints)
    )
    if any(value <= 0.0 for value in urdf_velocity):
        raise ValueError("NERO URDF velocity values must be positive")
    if any(selected > source for selected, source in zip(velocity, urdf_velocity)):
        raise ValueError("NERO selected velocity must not exceed URDF velocity")

    layout = JointLayout(
        names=names,
        lower=lower,
        upper=upper,
        velocity=velocity,
    )
    raw_home = data["home_position"]
    if not isinstance(raw_home, list):
        raise ValueError("NERO profile.home_position must be a list")
    home = layout.validate_vector(
        [
            _finite_float(value, field=f"NERO profile.home_position[{index}]")
            for index, value in enumerate(raw_home)
        ]
    ).copy()
    if np.any(home < np.asarray(lower)) or np.any(home > np.asarray(upper)):
        raise ValueError("NERO home position must be within joint limits")

    limit_policy = _mapping(
        data["limit_policy"], field="NERO profile.limit_policy"
    )
    policy = tuple(
        sorted(
            (
                _string(key, field="NERO profile.limit_policy key"),
                _string(value, field=f"NERO profile.limit_policy.{key}"),
            )
            for key, value in limit_policy.items()
        )
    )
    provenance = _mapping(data["provenance"], field="NERO profile.provenance")
    provenance_json = json.dumps(
        provenance,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    )
    return NeroModelProfile(
        profile_id=NERO_PROFILE_ID,
        status=NERO_PROFILE_STATUS,
        layout_id=NERO_LAYOUT_ID,
        layout=layout,
        home_position=home,
        base_frame=_string(frames["base"], field="NERO profile.frames.base"),
        tool_flange_frame=_string(
            frames["tool_flange"], field="NERO profile.frames.tool_flange"
        ),
        child_links=child_links,
        axes_xyz=tuple(axes),
        urdf_velocity_rad_s=urdf_velocity,
        provenance_json=provenance_json,
        limit_policy=policy,
    )


__all__ = [
    "NERO_CHILD_LINKS",
    "NERO_JOINT_NAMES",
    "NERO_LAYOUT_ID",
    "NERO_PRODUCT",
    "NERO_PROFILE_ID",
    "NERO_PROFILE_SCHEMA",
    "NERO_PROFILE_STATUS",
    "NeroModelProfile",
    "load_nero_model_profile",
]
