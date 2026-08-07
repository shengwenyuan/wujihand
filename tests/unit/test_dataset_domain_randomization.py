from __future__ import annotations

from pathlib import Path

from wujihand.dataset.domain_randomization import (
    VISUAL_DOMAIN_VARIANT_SCOPE,
    load_visual_domain_variant_profile,
)


ROOT = Path(__file__).parents[2]
PROFILE = "configs/profiles/isaac_mini_dataset_visual_domain_variants_v1.yaml"


def test_visual_domain_variant_profile_is_fixed_and_visual_only() -> None:
    profile = load_visual_domain_variant_profile(ROOT, PROFILE)

    assert profile.scope == VISUAL_DOMAIN_VARIANT_SCOPE
    assert tuple(item.variant_id for item in profile.variants) == (
        "nominal",
        "dr_warm_bright",
        "dr_cool_dim",
        "dr_neutral_highkey",
    )
    assert len({item.digest_sha256 for item in profile.variants}) == 4
    assert profile.variant("nominal").background_color_rgb is None
    assert profile.variant("dr_cool_dim").exposure_offset == -0.25
