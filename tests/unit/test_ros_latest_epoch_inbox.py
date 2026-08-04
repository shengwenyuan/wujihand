from __future__ import annotations

from threading import Event, Thread
import time

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
    assert inbox.metrics.drained == 1


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
    assert inbox.metrics.discarded == 1


def test_clear_accounts_for_a_pending_sample() -> None:
    inbox = LatestEpochInbox[str]()
    assert inbox.offer(
        "pending",
        producer_instance="producer-a",
        transport_epoch=1,
        sequence=0,
    )

    inbox.clear()

    assert inbox.metrics.discarded == 1
    assert inbox.metrics.pending == 0
    assert inbox.metrics.accepted == inbox.metrics.discarded


def test_latest_epoch_inbox_is_safe_for_one_writer_and_one_drainer() -> None:
    inbox = LatestEpochInbox[int]()
    started = Event()
    finished = Event()
    drained: list[int] = []
    sample_count = 2_000

    def produce() -> None:
        started.wait()
        for sequence in range(sample_count):
            assert inbox.offer(
                sequence,
                producer_instance="producer-a",
                transport_epoch=1,
                sequence=sequence,
            )
            if sequence % 17 == 0:
                time.sleep(0)
        finished.set()

    producer = Thread(target=produce)
    producer.start()
    started.set()
    while not finished.is_set():
        item = inbox.drain()
        if item is not None:
            drained.append(item)
        time.sleep(0)
    final = inbox.drain()
    if final is not None:
        drained.append(final)
    producer.join()

    assert drained == sorted(set(drained))
    assert inbox.metrics.accepted == sample_count
    assert inbox.metrics.drained == len(drained)
    assert inbox.metrics.accepted == (
        inbox.metrics.overwritten
        + inbox.metrics.drained
        + inbox.metrics.discarded
        + inbox.metrics.pending
    )
