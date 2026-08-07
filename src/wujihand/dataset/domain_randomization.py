"""Versioned, deterministic visual-only domain variants for offline replay."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Final, cast

import yaml

from wujihand.domain.recording import validate_recording_token


VISUAL_DOMAIN_VARIANT_PROFILE_SCHEMA: Final = (
    "wujihand.isaac_visual_domain_variant_profile.v1"
)
VISUAL_DOMAIN_VARIANT_SCOPE: Final = (
    "offline_rgb_only_no_geometry_camera_or_physics_changes"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@dataclass(frozen=True, slots=True)
class VisualDomainVariant:
    variant_id: str
    seed: int
    lighting_intensity_scale: float
    exposure_offset: float
    background_color_rgb: tuple[float, float, float] | None

    def __post_init__(self) -> None:
        validate_recording_token(self.variant_id, field="variant_id")
        if type(self.seed) is not int or self.seed < 0:
            raise ValueError("visual domain variant seed must be non-negative")
        if (
            not math.isfinite(self.lighting_intensity_scale)
            or self.lighting_intensity_scale <= 0.0
            or not math.isfinite(self.exposure_offset)
        ):
            raise ValueError("visual domain variant lighting values differ")
        if self.background_color_rgb is not None and (
            len(self.background_color_rgb) != 3
            or any(
                not math.isfinite(value) or not 0.0 <= value <= 1.0
                for value in self.background_color_rgb
            )
        ):
            raise ValueError("visual domain variant background color differs")

    def to_mapping(self) -> dict[str, object]:
        return {
            "variant_id": self.variant_id,
            "seed": self.seed,
            "lighting_intensity_scale": self.lighting_intensity_scale,
            "exposure_offset": self.exposure_offset,
            "background_color_rgb": (
                None
                if self.background_color_rgb is None
                else list(self.background_color_rgb)
            ),
        }

    @property
    def digest_sha256(self) -> str:
        return hashlib.sha256(
            json.dumps(
                self.to_mapping(),
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        ).hexdigest()


NOMINAL_VISUAL_DOMAIN_VARIANT: Final = VisualDomainVariant(
    variant_id="nominal",
    seed=0,
    lighting_intensity_scale=1.0,
    exposure_offset=0.0,
    background_color_rgb=None,
)


@dataclass(frozen=True, slots=True)
class VisualDomainVariantProfile:
    profile_id: str
    scope: str
    variants: tuple[VisualDomainVariant, ...]
    file_sha256: str

    def __post_init__(self) -> None:
        validate_recording_token(self.profile_id, field="profile_id")
        if self.scope != VISUAL_DOMAIN_VARIANT_SCOPE:
            raise ValueError("visual domain variant scope differs")
        if not self.variants or len({item.variant_id for item in self.variants}) != len(
            self.variants
        ):
            raise ValueError("visual domain variants must be non-empty and unique")
        if self.variants[0].variant_id != "nominal":
            raise ValueError("visual domain variants must begin with nominal")
        if len(self.file_sha256) != 64:
            raise ValueError("visual domain variant profile hash differs")

    def variant(self, variant_id: str) -> VisualDomainVariant:
        selected = tuple(item for item in self.variants if item.variant_id == variant_id)
        if len(selected) != 1:
            raise KeyError(f"unknown visual domain variant: {variant_id}")
        return selected[0]


def load_visual_domain_variant_profile(
    project_root: str | Path,
    profile_path: str | Path,
) -> VisualDomainVariantProfile:
    root = Path(project_root).resolve()
    raw_path = Path(profile_path)
    path = raw_path.resolve() if raw_path.is_absolute() else (root / raw_path).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise ValueError("visual domain variant profile must remain inside project root") from exc
    if path.is_symlink() or not path.is_file():
        raise ValueError("visual domain variant profile is missing or unsafe")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("visual domain variant profile YAML is invalid") from exc
    if not isinstance(value, Mapping) or frozenset(value) != {
        "schema",
        "profile_id",
        "scope",
        "variants",
    }:
        raise ValueError("visual domain variant profile keys differ")
    data = cast(Mapping[str, object], value)
    if data["schema"] != VISUAL_DOMAIN_VARIANT_PROFILE_SCHEMA:
        raise ValueError("visual domain variant profile schema differs")
    if not isinstance(data["profile_id"], str) or not isinstance(data["scope"], str):
        raise ValueError("visual domain variant profile identity differs")
    raw_variants = data["variants"]
    if not isinstance(raw_variants, Sequence) or isinstance(
        raw_variants,
        (str, bytes, bytearray),
    ):
        raise ValueError("visual domain variants must be a sequence")
    variants: list[VisualDomainVariant] = []
    for index, raw in enumerate(raw_variants):
        if not isinstance(raw, Mapping) or frozenset(raw) != {
            "variant_id",
            "seed",
            "lighting_intensity_scale",
            "exposure_offset",
            "background_color_rgb",
        }:
            raise ValueError(f"visual domain variant {index} keys differ")
        item = cast(Mapping[str, object], raw)
        variant_id = item["variant_id"]
        seed = item["seed"]
        if not isinstance(variant_id, str) or type(seed) is not int:
            raise ValueError(f"visual domain variant {index} identity differs")
        numbers: dict[str, float] = {}
        for key in ("lighting_intensity_scale", "exposure_offset"):
            number = item[key]
            if isinstance(number, bool) or not isinstance(number, (int, float)):
                raise ValueError(f"visual domain variant {index}.{key} differs")
            numbers[key] = float(number)
        raw_color = item["background_color_rgb"]
        color: tuple[float, float, float] | None
        if raw_color is None:
            color = None
        elif isinstance(raw_color, Sequence) and not isinstance(
            raw_color,
            (str, bytes, bytearray),
        ) and len(raw_color) == 3 and all(
            not isinstance(number, bool) and isinstance(number, (int, float))
            for number in raw_color
        ):
            color = cast(
                tuple[float, float, float],
                tuple(float(cast(float, number)) for number in raw_color),
            )
        else:
            raise ValueError(f"visual domain variant {index} color differs")
        variants.append(
            VisualDomainVariant(
                variant_id=variant_id,
                seed=seed,
                lighting_intensity_scale=numbers["lighting_intensity_scale"],
                exposure_offset=numbers["exposure_offset"],
                background_color_rgb=color,
            )
        )
    return VisualDomainVariantProfile(
        profile_id=data["profile_id"],
        scope=data["scope"],
        variants=tuple(variants),
        file_sha256=_sha256(path),
    )


__all__ = [
    "VISUAL_DOMAIN_VARIANT_PROFILE_SCHEMA",
    "NOMINAL_VISUAL_DOMAIN_VARIANT",
    "VisualDomainVariant",
    "VisualDomainVariantProfile",
    "load_visual_domain_variant_profile",
]
