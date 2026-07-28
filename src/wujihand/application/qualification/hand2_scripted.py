"""Hardware-independent Hand 2 targets for the bounded NV-2 scripted Gate."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from numbers import Real

import numpy as np
import numpy.typing as npt

from wujihand.domain.hand_teleoperation import HandSide
from wujihand.domain.hand2 import hand2_layout


@dataclass(frozen=True, slots=True)
class Hand2QualificationTarget:
    """One explicit, side-specific q20 qualification target."""

    phase_id: str
    side: HandSide
    commanded_joint_names: tuple[str, ...]
    q20_rad: tuple[float, ...]
    command_delta_rad: float


@dataclass(frozen=True, slots=True)
class Hand2SingleDigitPartition:
    """q20 indices split around the digit exercised by one scripted phase."""

    commanded_digit: str
    same_digit_uncommanded_indices: tuple[int, ...]
    other_digit_indices: tuple[int, ...]


_HAND2_DIGITS = ("thumb", "index", "middle", "ring", "pinky")


def build_hand2_qualification_targets(
    side: HandSide,
    rest_position: Sequence[float] | npt.NDArray[np.floating],
    *,
    amplitude_rad: float,
) -> tuple[tuple[Hand2QualificationTarget, ...], Hand2QualificationTarget]:
    """Build five single-finger targets and one conservative combined pose.

    Single-finger phases use thumb IP and the four PIP joints.  The combined
    pose flexes all five digits while leaving abduction at the approved rest
    value.  No value is silently clamped: an incompatible profile fails before
    a simulator command can be authored.
    """

    if type(side) is not HandSide:
        raise ValueError("side must be a HandSide")
    if isinstance(amplitude_rad, bool) or not isinstance(amplitude_rad, Real):
        raise ValueError("amplitude_rad must be a finite positive number")
    amplitude = float(amplitude_rad)
    if not math.isfinite(amplitude) or amplitude <= 0.0:
        raise ValueError("amplitude_rad must be a finite positive number")

    layout = hand2_layout(side.value)
    rest = layout.validate_vector(rest_position)
    if not np.array_equal(rest, layout.clamp(rest)):
        raise ValueError("rest_position must already be inside canonical Hand 2 limits")

    prefix = f"{side.value[0]}_"
    representative = (
        ("thumb", f"{prefix}thumb_ip"),
        ("index", f"{prefix}index_finger_pip"),
        ("middle", f"{prefix}middle_finger_pip"),
        ("ring", f"{prefix}ring_finger_pip"),
        ("pinky", f"{prefix}pinky_pip"),
    )
    singles = tuple(
        _target(
            phase_id=f"{side.value}_{finger}",
            side=side,
            layout_names=layout.names,
            layout_lower=layout.lower,
            layout_upper=layout.upper,
            rest=rest,
            commanded_joint_names=(joint_name,),
            command_delta_rad=amplitude,
        )
        for finger, joint_name in representative
    )

    combined_names = (
        f"{prefix}thumb_cmc_flex",
        f"{prefix}thumb_mcp",
        f"{prefix}thumb_ip",
        *(
            name
            for finger in (
                "index_finger",
                "middle_finger",
                "ring_finger",
                "pinky",
            )
            for name in (
                f"{prefix}{finger}_mcp_flex",
                f"{prefix}{finger}_pip",
                f"{prefix}{finger}_dip",
            )
        ),
    )
    combined = _target(
        phase_id=f"{side.value}_combined_hand",
        side=side,
        layout_names=layout.names,
        layout_lower=layout.lower,
        layout_upper=layout.upper,
        rest=rest,
        commanded_joint_names=combined_names,
        command_delta_rad=amplitude * 0.5,
    )
    return singles, combined


def partition_hand2_single_digit_indices(
    layout_names: Sequence[str],
    commanded_joint_names: Sequence[str],
) -> Hand2SingleDigitPartition:
    """Separate same-digit linkage motion from actual other-finger crosstalk."""

    names = tuple(layout_names)
    commanded = tuple(commanded_joint_names)
    if len(names) != 20 or len(set(names)) != 20:
        raise ValueError("layout_names must contain 20 unique Hand 2 joints")
    if not commanded or len(set(commanded)) != len(commanded):
        raise ValueError("commanded_joint_names must be non-empty and unique")
    if any(name not in names for name in commanded):
        raise ValueError("commanded_joint_names must belong to layout_names")

    commanded_digits = {_hand2_joint_digit(name) for name in commanded}
    if len(commanded_digits) != 1:
        raise ValueError("commanded_joint_names must belong to exactly one digit")
    commanded_digit = commanded_digits.pop()
    commanded_indices = {names.index(name) for name in commanded}
    same_digit_uncommanded = tuple(
        index
        for index, name in enumerate(names)
        if _hand2_joint_digit(name) == commanded_digit
        and index not in commanded_indices
    )
    other_digits = tuple(
        index
        for index, name in enumerate(names)
        if _hand2_joint_digit(name) != commanded_digit
    )
    if len(same_digit_uncommanded) + len(other_digits) + len(commanded_indices) != 20:
        raise ValueError("Hand 2 digit partition is incomplete")
    return Hand2SingleDigitPartition(
        commanded_digit=commanded_digit,
        same_digit_uncommanded_indices=same_digit_uncommanded,
        other_digit_indices=other_digits,
    )


def qualification_gate_exit_code(passed: bool) -> int:
    """Return a process status that cannot silently accept a failed Gate."""

    if type(passed) is not bool:
        raise ValueError("passed must be a bool")
    return 0 if passed else 2


def _hand2_joint_digit(joint_name: str) -> str:
    matches = tuple(
        digit
        for digit in _HAND2_DIGITS
        if f"_{digit}_" in joint_name
    )
    if len(matches) != 1:
        raise ValueError(f"cannot identify exactly one Hand 2 digit for {joint_name!r}")
    return matches[0]


def _target(
    *,
    phase_id: str,
    side: HandSide,
    layout_names: tuple[str, ...],
    layout_lower: tuple[float, ...],
    layout_upper: tuple[float, ...],
    rest: npt.NDArray[np.float64],
    commanded_joint_names: tuple[str, ...],
    command_delta_rad: float,
) -> Hand2QualificationTarget:
    indices = tuple(layout_names.index(name) for name in commanded_joint_names)
    values = rest.copy()
    values[np.asarray(indices, dtype=np.int64)] += command_delta_rad
    lower = np.asarray(layout_lower, dtype=np.float64)
    upper = np.asarray(layout_upper, dtype=np.float64)
    if not (np.all(values >= lower) and np.all(values <= upper)):
        raise ValueError(f"{phase_id} target exceeds canonical Hand 2 limits")
    return Hand2QualificationTarget(
        phase_id=phase_id,
        side=side,
        commanded_joint_names=commanded_joint_names,
        q20_rad=tuple(float(value) for value in values),
        command_delta_rad=command_delta_rad,
    )


__all__ = [
    "Hand2QualificationTarget",
    "Hand2SingleDigitPartition",
    "build_hand2_qualification_targets",
    "partition_hand2_single_digit_indices",
    "qualification_gate_exit_code",
]
