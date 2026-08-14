from wujihand_hand2_hardware.mapping import (
    H3_MINIMUM_TARGET_FRACTION,
    H3_S1_SEQUENCE_LABELS,
    H4_MINIMUM_TARGET_FRACTION,
    H4_Q20_SEQUENCE_LABELS,
    Q20_DESCRIPTION_NAMES,
    Q20_LABELS,
    Q20_NIDS,
    validate_q20_layout,
)


def test_q20_mapping_is_five_ordered_fingers() -> None:
    assert Q20_LABELS[:4] == ("thumb_S1", "thumb_S2", "thumb_S3", "thumb_S4")
    assert Q20_LABELS[-4:] == ("pinky_S1", "pinky_S2", "pinky_S3", "pinky_S4")
    assert Q20_NIDS == (1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24)
    assert not validate_q20_layout(Q20_LABELS, set(Q20_NIDS))


def test_q20_mapping_rejects_missing_nid() -> None:
    failures = validate_q20_layout(Q20_LABELS, set(Q20_NIDS) - {24})
    assert failures == ("q20 NIDs mismatch: missing=[24], extra=[]",)


def test_h3_s1_sequence_mapping_is_explicit() -> None:
    assert H3_S1_SEQUENCE_LABELS == (
        "thumb_S1",
        "index_S1",
        "middle_S1",
        "ring_S1",
        "pinky_S1",
    )
    indexes = [Q20_LABELS.index(label) for label in H3_S1_SEQUENCE_LABELS]
    assert indexes == [0, 4, 8, 12, 16]
    assert [Q20_NIDS[index] for index in indexes] == [1, 6, 11, 16, 21]
    assert [Q20_DESCRIPTION_NAMES[index] for index in indexes] == [
        "r_thumb_cmc_flex",
        "r_index_finger_mcp_flex",
        "r_middle_finger_mcp_flex",
        "r_ring_finger_mcp_flex",
        "r_pinky_mcp_flex",
    ]


def test_h4_sequence_covers_q20_in_protocol_order() -> None:
    assert H4_Q20_SEQUENCE_LABELS == Q20_LABELS
    assert len(H4_Q20_SEQUENCE_LABELS) == 20
    assert H3_MINIMUM_TARGET_FRACTION == 0.05
    assert H4_MINIMUM_TARGET_FRACTION == 0.01
