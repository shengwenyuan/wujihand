from __future__ import annotations

import hashlib
from dataclasses import replace

import pytest

from wujihand.dataset.alignment import RawTransition, build_exact_30hz_alignment


def _digest(value: int) -> str:
    return hashlib.sha256(str(value).encode()).hexdigest()


def _transitions(*, first: int = 5, count: int = 5) -> tuple[RawTransition, ...]:
    result = []
    for offset in range(count):
        index = first + offset
        pre = (float(index),) * 54
        post = (float(index + 1),) * 54
        result.append(
            RawTransition(
                run_id="episode-001",
                control_index=index,
                tick_id=index,
                simulation_time_before_s=offset / 60.0,
                simulation_time_after_s=(offset + 1) / 60.0,
                pre_feedback_q54_rad=pre,
                applied_target_q54_rad=(float(index) + 0.5,) * 54,
                post_feedback_q54_rad=post,
                pre_action_state_digest=_digest(index),
            )
        )
    return tuple(result)


def test_alignment_selects_relative_even_ticks_from_nonzero_anchor() -> None:
    alignment = build_exact_30hz_alignment(_transitions())

    assert tuple(frame.source_control_index for frame in alignment.frames) == (5, 7, 9)
    assert alignment.odd_control_indices == (6, 8)
    assert tuple(frame.timestamp_s for frame in alignment.frames) == pytest.approx(
        (0.0, 1.0 / 30.0, 2.0 / 30.0)
    )
    assert alignment.frames[0].observation_q54_rad == (5.0,) * 54
    assert alignment.frames[0].action_q54_rad == (5.5,) * 54
    assert len(alignment.digest_sha256) == 64


def test_alignment_retains_a_trailing_odd_tick_only_in_audit_sidecar() -> None:
    alignment = build_exact_30hz_alignment(_transitions(count=4))

    assert tuple(frame.source_control_index for frame in alignment.frames) == (5, 7)
    assert alignment.odd_control_indices == (6, 8)


def test_alignment_carries_missed_period_mask_without_bridging_segments() -> None:
    alignment = build_exact_30hz_alignment(
        _transitions(),
        missed_control_periods_before_tick={6: 1},
    )

    assert alignment.gap_ticks == ((6, 1),)
    assert tuple(
        (
            frame.temporal_continuity,
            frame.missing_control_periods_before,
            frame.temporal_segment_index,
            frame.gap_before_row,
            frame.transition_valid,
        )
        for frame in alignment.frames
    ) == (
        (True, 0, 0, False, False),
        (False, 1, 1, True, True),
        (True, 0, 1, False, False),
    )


def test_alignment_rejects_gap_incomplete_or_discontinuous_transition() -> None:
    rows = _transitions()
    with pytest.raises(ValueError, match="contiguous"):
        build_exact_30hz_alignment((*rows[:2], *rows[3:]))
    with pytest.raises(ValueError, match="incomplete"):
        build_exact_30hz_alignment((replace(rows[0], complete=False),))


def test_alignment_rejects_q54_post_pre_discontinuity() -> None:
    rows = list(_transitions(count=2))
    second = rows[1]
    rows[1] = RawTransition(
        run_id=second.run_id,
        control_index=second.control_index,
        tick_id=second.tick_id,
        simulation_time_before_s=second.simulation_time_before_s,
        simulation_time_after_s=second.simulation_time_after_s,
        pre_feedback_q54_rad=(99.0,) * 54,
        applied_target_q54_rad=second.applied_target_q54_rad,
        post_feedback_q54_rad=second.post_feedback_q54_rad,
        pre_action_state_digest=second.pre_action_state_digest,
    )

    with pytest.raises(ValueError, match="not continuous"):
        build_exact_30hz_alignment(rows)
