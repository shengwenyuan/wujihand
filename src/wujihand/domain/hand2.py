"""Pinned canonical layouts for Wuji Hand 2 Beta 1."""

from __future__ import annotations

import numpy as np

from .joints import FloatArray, JointLayout


HAND2_RIGHT_LAYOUT = JointLayout(
    names=(
        "r_thumb_cmc_flex",
        "r_thumb_cmc_abd",
        "r_thumb_mcp",
        "r_thumb_ip",
        "r_index_finger_mcp_flex",
        "r_index_finger_mcp_abd",
        "r_index_finger_pip",
        "r_index_finger_dip",
        "r_middle_finger_mcp_flex",
        "r_middle_finger_mcp_abd",
        "r_middle_finger_pip",
        "r_middle_finger_dip",
        "r_ring_finger_mcp_flex",
        "r_ring_finger_mcp_abd",
        "r_ring_finger_pip",
        "r_ring_finger_dip",
        "r_pinky_mcp_flex",
        "r_pinky_mcp_abd",
        "r_pinky_pip",
        "r_pinky_dip",
    ),
    lower=(
        -1.187,
        -1.484,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
        -1.047,
        -0.698,
        -1.047,
        -1.047,
    ),
    upper=(
        1.291,
        0.698,
        1.570,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
        1.570,
        0.698,
        2.094,
        1.570,
    ),
    velocity=(
        8.587,
        11.1,
        12.86,
        13.5,
        8.203,
        8.11,
        12.86,
        13.5,
        8.203,
        8.11,
        12.86,
        13.5,
        8.203,
        8.11,
        12.86,
        13.5,
        8.203,
        8.11,
        12.86,
        13.5,
    ),
)

HAND2_RIGHT_REST = np.zeros(HAND2_RIGHT_LAYOUT.size, dtype=np.float64)

HAND2_LEFT_LAYOUT = JointLayout(
    names=tuple(f"l_{name.removeprefix('r_')}" for name in HAND2_RIGHT_LAYOUT.names),
    lower=HAND2_RIGHT_LAYOUT.lower,
    upper=HAND2_RIGHT_LAYOUT.upper,
    velocity=HAND2_RIGHT_LAYOUT.velocity,
)

HAND2_LEFT_REST = np.zeros(HAND2_LEFT_LAYOUT.size, dtype=np.float64)

HAND2_LAYOUT_IDS = {
    "left": "wuji_hand2_left_firmware_v1",
    "right": "wuji_hand2_right_firmware_v1",
}


def hand2_layout(side: str) -> JointLayout:
    """Return the fixed Beta 1 firmware layout for one explicit side."""

    if side == "left":
        return HAND2_LEFT_LAYOUT
    if side == "right":
        return HAND2_RIGHT_LAYOUT
    raise ValueError("Hand 2 side must be 'left' or 'right'")


def hand2_rest(side: str) -> FloatArray:
    """Return a writable copy of the side-specific zero/rest command."""

    if side == "left":
        return HAND2_LEFT_REST.copy()
    if side == "right":
        return HAND2_RIGHT_REST.copy()
    raise ValueError("Hand 2 side must be 'left' or 'right'")
