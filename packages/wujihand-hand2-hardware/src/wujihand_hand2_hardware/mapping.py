from __future__ import annotations

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
Q20_LABELS = tuple(f"{finger}_S{joint}" for finger in FINGERS for joint in range(1, 5))
Q20_NIDS = tuple(index + 1 + index // 4 for index in range(20))
Q20_LABEL_BY_NID = dict(zip(Q20_NIDS, Q20_LABELS, strict=True))
Q20_INDEX_BY_NID = {nid: index for index, nid in enumerate(Q20_NIDS)}
Q20_INDEX_BY_LABEL = {label: index for index, label in enumerate(Q20_LABELS)}

# wuji-description v2026.8.3, Hand2 Beta1 model revision v2026.7.23.
Q20_DESCRIPTION_NAMES = (
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
)
Q20_LOWER_RAD = (
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
)
Q20_UPPER_RAD = (
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
)

H3_S1_SEQUENCE_LABELS = tuple(f"{finger}_S1" for finger in FINGERS)
H3_SEQUENCE_DEFAULT_DELTA_RAD = 0.12
H3_MAX_DELTA_RAD = 0.15
H3_DESCRIPTION_LIMIT_MARGIN_RAD = 0.10


def validate_q20_layout(labels: tuple[str, ...], observed_nids: set[int]) -> tuple[str, ...]:
    failures: list[str] = []
    if labels != Q20_LABELS:
        failures.append(f"q20 labels mismatch: observed={labels!r}")
    expected_nids = set(Q20_NIDS)
    if observed_nids != expected_nids:
        missing = sorted(expected_nids - observed_nids)
        extra = sorted(observed_nids - expected_nids)
        failures.append(f"q20 NIDs mismatch: missing={missing}, extra={extra}")
    return tuple(failures)
