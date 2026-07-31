from __future__ import annotations

from wujihand_ros2.inbox import LatestEpochInbox


def test_latest_epoch_inbox_overwrites_without_backlog() -> None:
    inbox = LatestEpochInbox[str]()

    assert inbox.offer(
        "first",
        producer_instance="producer-a",
        transport_epoch=1,
        sequence=0,
    )
    assert inbox.offer(
        "second",
        producer_instance="producer-a",
        transport_epoch=1,
        sequence=1,
    )

    assert inbox.drain() == "second"
    assert inbox.drain() is None
    assert inbox.metrics.overwritten == 1


def test_latest_epoch_inbox_rejects_reorder_and_retired_producer() -> None:
    inbox = LatestEpochInbox[str]()
    assert inbox.offer(
        "a1",
        producer_instance="producer-a",
        transport_epoch=2,
        sequence=4,
    )
    assert not inbox.offer(
        "duplicate",
        producer_instance="producer-a",
        transport_epoch=2,
        sequence=4,
    )
    assert not inbox.offer(
        "old-epoch",
        producer_instance="producer-a",
        transport_epoch=1,
        sequence=100,
    )
    assert inbox.offer(
        "b1",
        producer_instance="producer-b",
        transport_epoch=0,
        sequence=0,
    )
    assert not inbox.offer(
        "late-a",
        producer_instance="producer-a",
        transport_epoch=3,
        sequence=0,
    )

    assert inbox.drain() == "b1"
    assert inbox.metrics.rejected_sequence == 1
    assert inbox.metrics.rejected_old_epoch == 1
    assert inbox.metrics.rejected_old_producer == 1
    assert inbox.metrics.rebinds == 1


def test_new_epoch_clears_pending_old_command() -> None:
    inbox = LatestEpochInbox[str]()
    assert inbox.offer(
        "old-command",
        producer_instance="producer-a",
        transport_epoch=1,
        sequence=20,
    )
    assert inbox.offer(
        "new-command",
        producer_instance="producer-a",
        transport_epoch=2,
        sequence=0,
    )

    assert inbox.drain() == "new-command"
    assert inbox.metrics.rebinds == 1
