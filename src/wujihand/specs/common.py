"""Shared, simulator-independent values for the five-layer configuration model."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import PurePosixPath
import re
from typing import Self, cast


_IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_.:/+-]{0,127}$")
_QUATERNION_NORM_TOLERANCE = 1e-6


def require_exact_mapping(
    value: object, *, expected: frozenset[str], field: str
) -> Mapping[str, object]:
    """Return a string-keyed mapping only when its keys exactly match a schema."""

    if not isinstance(value, Mapping):
        raise ValueError(f"{field} must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"{field} keys must be strings")
    mapping = cast(Mapping[str, object], value)
    actual = frozenset(mapping)
    if actual != expected:
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        raise ValueError(
            f"{field} keys differ from schema: missing={missing}, unexpected={unexpected}"
        )
    return mapping


def require_sequence(value: object, *, field: str) -> Sequence[object]:
    """Return a non-string sequence used by a schema field."""

    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{field} must be a sequence")
    return cast(Sequence[object], value)


def require_string(value: object, *, field: str) -> str:
    """Return a non-blank string without rewriting it."""

    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-blank string")
    if value != value.strip():
        raise ValueError(f"{field} must not have leading or trailing whitespace")
    return value


def validate_identifier(value: object, *, field: str) -> str:
    """Validate a bounded identifier suitable for stable configuration references."""

    identifier = require_string(value, field=field)
    if _IDENTIFIER.fullmatch(identifier) is None:
        raise ValueError(f"{field} must be a valid identifier")
    return identifier


def validate_project_reference(value: object, *, field: str) -> str:
    """Validate a project-relative path or source-lock identifier.

    Resolution and containment are repository concerns.  This local check rejects
    absolute, parent-relative, home-relative, Windows-style, and ambiguous paths
    before any filesystem operation can occur.
    """

    reference = require_string(value, field=field)
    if "\\" in reference or reference.startswith(("~", "/")):
        raise ValueError(f"{field} must be a safe project-relative reference")
    if any(part in {"", ".", ".."} for part in reference.split("/")):
        raise ValueError(f"{field} must be a safe project-relative reference")
    path = PurePosixPath(reference)
    if path.is_absolute():
        raise ValueError(f"{field} must be a safe project-relative reference")
    if ":" in path.parts[0]:
        raise ValueError(f"{field} must be a safe project-relative reference")
    return reference


def optional_project_reference(value: object, *, field: str) -> str | None:
    """Validate an explicit nullable project reference."""

    if value is None:
        return None
    return validate_project_reference(value, field=field)


def finite_number(value: object, *, field: str) -> float:
    """Return a finite real scalar while rejecting bool and numeric strings."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field} must be a finite number")
    return number


def positive_number(value: object, *, field: str) -> float:
    """Return a finite positive scalar."""

    number = finite_number(value, field=field)
    if number <= 0.0:
        raise ValueError(f"{field} must be positive")
    return number


def finite_vector(value: object, *, size: int, field: str) -> tuple[float, ...]:
    """Return a fixed-size tuple of finite real scalars."""

    items = require_sequence(value, field=field)
    if len(items) != size:
        raise ValueError(f"{field} must contain exactly {size} values")
    return tuple(
        finite_number(item, field=f"{field}[{index}]") for index, item in enumerate(items)
    )


def positive_vector(value: object, *, size: int, field: str) -> tuple[float, ...]:
    """Return a fixed-size tuple whose components are finite and positive."""

    items = finite_vector(value, size=size, field=field)
    if any(item <= 0.0 for item in items):
        raise ValueError(f"{field} values must be positive")
    return items


@dataclass(frozen=True, slots=True)
class ConfigRef:
    """Reference a project configuration by path and expected stable ID."""

    path: str
    expected_id: str

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "ref") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"path", "expected_id"}),
            field=field,
        )
        return cls(
            path=validate_project_reference(data["path"], field=f"{field}.path"),
            expected_id=validate_identifier(
                data["expected_id"], field=f"{field}.expected_id"
            ),
        )

    def to_mapping(self) -> dict[str, object]:
        return {"path": self.path, "expected_id": self.expected_id}


@dataclass(frozen=True, slots=True)
class PoseSpec:
    """Rigid transform using metres and a scalar-first unit quaternion."""

    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]

    @classmethod
    def from_mapping(cls, value: object, *, field: str = "transform") -> Self:
        data = require_exact_mapping(
            value,
            expected=frozenset({"position_m", "quat_wxyz"}),
            field=field,
        )
        position = finite_vector(data["position_m"], size=3, field=f"{field}.position_m")
        quaternion = finite_vector(
            data["quat_wxyz"], size=4, field=f"{field}.quat_wxyz"
        )
        norm = math.sqrt(sum(component * component for component in quaternion))
        if not math.isclose(
            norm,
            1.0,
            rel_tol=0.0,
            abs_tol=_QUATERNION_NORM_TOLERANCE,
        ):
            raise ValueError(f"{field}.quat_wxyz must be unit length")
        return cls(
            position_m=cast(tuple[float, float, float], position),
            quat_wxyz=cast(tuple[float, float, float, float], quaternion),
        )

    @classmethod
    def identity(cls) -> Self:
        return cls(
            position_m=(0.0, 0.0, 0.0),
            quat_wxyz=(1.0, 0.0, 0.0, 0.0),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "position_m": list(self.position_m),
            "quat_wxyz": list(self.quat_wxyz),
        }


__all__ = [
    "ConfigRef",
    "PoseSpec",
    "finite_number",
    "finite_vector",
    "positive_number",
    "positive_vector",
    "optional_project_reference",
    "require_exact_mapping",
    "require_sequence",
    "require_string",
    "validate_identifier",
    "validate_project_reference",
]
