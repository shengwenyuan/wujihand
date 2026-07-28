from __future__ import annotations

import numpy as np
import pytest

from wujihand.application.qualification import (
    build_hand2_qualification_targets,
    partition_hand2_single_digit_indices,
    qualification_gate_exit_code,
)
from wujihand.domain import HandSide, hand2_layout


@pytest.mark.parametrize("side", tuple(HandSide))
def test_targets_cover_five_digits_and_one_combined_flexion_pose(
    side: HandSide,
) -> None:
    layout = hand2_layout(side.value)
    singles, combined = build_hand2_qualification_targets(
        side,
        np.zeros(20),
        amplitude_rad=0.4,
    )

    assert tuple(target.phase_id for target in singles) == (
        f"{side.value}_thumb",
        f"{side.value}_index",
        f"{side.value}_middle",
        f"{side.value}_ring",
        f"{side.value}_pinky",
    )
    assert tuple(target.commanded_joint_names[0] for target in singles) == (
        f"{side.value[0]}_thumb_ip",
        f"{side.value[0]}_index_finger_pip",
        f"{side.value[0]}_middle_finger_pip",
        f"{side.value[0]}_ring_finger_pip",
        f"{side.value[0]}_pinky_pip",
    )
    assert all(target.command_delta_rad == pytest.approx(0.4) for target in singles)
    assert combined.phase_id == f"{side.value}_combined_hand"
    assert len(combined.commanded_joint_names) == 15
    assert combined.command_delta_rad == pytest.approx(0.2)

    for target in (*singles, combined):
        q20 = np.asarray(target.q20_rad)
        assert q20.shape == (20,)
        assert np.isfinite(q20).all()
        assert np.all(q20 >= np.asarray(layout.lower))
        assert np.all(q20 <= np.asarray(layout.upper))


def test_targets_preserve_rest_for_every_uncommanded_joint() -> None:
    layout = hand2_layout("right")
    rest = np.linspace(-0.05, 0.05, 20)
    singles, combined = build_hand2_qualification_targets(
        HandSide.RIGHT,
        rest,
        amplitude_rad=0.2,
    )

    for target in (*singles, combined):
        commanded = {layout.names.index(name) for name in target.commanded_joint_names}
        uncommanded = tuple(index for index in range(20) if index not in commanded)
        np.testing.assert_allclose(
            np.asarray(target.q20_rad)[np.asarray(uncommanded)],
            rest[np.asarray(uncommanded)],
        )


@pytest.mark.parametrize(
    ("side", "rest", "amplitude", "message"),
    (
        ("right", np.zeros(20), 0.2, "HandSide"),
        (HandSide.RIGHT, np.zeros(19), 0.2, "shape"),
        (HandSide.RIGHT, np.zeros(20), 0.0, "positive"),
        (HandSide.RIGHT, np.zeros(20), np.nan, "positive"),
        (HandSide.RIGHT, np.zeros(20), 10.0, "limits"),
    ),
)
def test_targets_reject_ambiguous_or_unsafe_inputs(
    side: object,
    rest: object,
    amplitude: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        build_hand2_qualification_targets(
            side,  # type: ignore[arg-type]
            rest,  # type: ignore[arg-type]
            amplitude_rad=amplitude,  # type: ignore[arg-type]
        )


@pytest.mark.parametrize("side", tuple(HandSide))
@pytest.mark.parametrize("digit", ("thumb", "index", "middle", "ring", "pinky"))
def test_single_digit_partition_excludes_same_digit_linkage_from_crosstalk(
    side: HandSide,
    digit: str,
) -> None:
    layout = hand2_layout(side.value)
    joint_name = next(name for name in layout.names if f"_{digit}_" in name)

    partition = partition_hand2_single_digit_indices(
        layout.names,
        (joint_name,),
    )

    assert partition.commanded_digit == digit
    assert len(partition.same_digit_uncommanded_indices) == 3
    assert len(partition.other_digit_indices) == 16
    assert all(
        f"_{digit}_" not in layout.names[index]
        for index in partition.other_digit_indices
    )


@pytest.mark.parametrize(
    ("commanded", "message"),
    (
        ((), "non-empty"),
        (("r_thumb_ip", "r_index_finger_pip"), "exactly one digit"),
        (("unknown",), "belong"),
    ),
)
def test_single_digit_partition_rejects_ambiguous_commands(
    commanded: tuple[str, ...],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        partition_hand2_single_digit_indices(
            hand2_layout("right").names,
            commanded,
        )


def test_failed_qualification_gate_returns_nonzero_exit_code() -> None:
    assert qualification_gate_exit_code(True) == 0
    assert qualification_gate_exit_code(False) == 2
    with pytest.raises(ValueError, match="bool"):
        qualification_gate_exit_code(1)  # type: ignore[arg-type]
