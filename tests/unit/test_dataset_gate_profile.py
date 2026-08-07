from __future__ import annotations

from pathlib import Path

from wujihand.dataset.gate_profile import load_mini_dataset_gate_profile


ROOT = Path(__file__).parents[2]
PROFILE = "configs/profiles/mini_dataset_integrity_quality_gate_v1.yaml"


def test_gate_profile_freezes_integrity_and_non_blocking_quality_thresholds() -> None:
    profile = load_mini_dataset_gate_profile(ROOT, PROFILE)

    assert profile.integrity.expected_control_hz == 60.0
    assert profile.integrity.expected_physics_hz == 120.0
    assert profile.integrity.physics_grid_time_atol_s == 5e-6
    assert profile.replay_link_position_limit_m == 2e-5
    assert profile.quality.control_hz_lower == (59.5, 58.0, 55.0)
    assert profile.quality.missed_fraction_upper == (0.005, 0.02, 0.03)
