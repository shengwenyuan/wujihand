"""Latest-only inbox with producer/epoch/sequence rejection."""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Generic, TypeVar


ItemT = TypeVar("ItemT")


@dataclass(frozen=True, slots=True)
class EpochInboxMetrics:
    accepted: int
    drained: int
    discarded: int
    rejected_old_producer: int
    rejected_old_epoch: int
    rejected_sequence: int
    overwritten: int
    rebinds: int
    pending: int


class LatestEpochInbox(Generic[ItemT]):
    """Keep one newest item and retire producer identities after restart."""

    def __init__(self) -> None:
        self._lock = RLock()
        self._producer_instance: str | None = None
        self._transport_epoch: int | None = None
        self._last_sequence: int | None = None
        self._retired_producers: set[str] = set()
        self._latest: ItemT | None = None
        self._accepted = 0
        self._drained = 0
        self._discarded = 0
        self._rejected_old_producer = 0
        self._rejected_old_epoch = 0
        self._rejected_sequence = 0
        self._overwritten = 0
        self._rebinds = 0

    @property
    def metrics(self) -> EpochInboxMetrics:
        with self._lock:
            return EpochInboxMetrics(
                accepted=self._accepted,
                drained=self._drained,
                discarded=self._discarded,
                rejected_old_producer=self._rejected_old_producer,
                rejected_old_epoch=self._rejected_old_epoch,
                rejected_sequence=self._rejected_sequence,
                overwritten=self._overwritten,
                rebinds=self._rebinds,
                pending=int(self._latest is not None),
            )

    def offer(
        self,
        item: ItemT,
        *,
        producer_instance: str,
        transport_epoch: int,
        sequence: int,
    ) -> bool:
        if not producer_instance:
            raise ValueError("producer_instance must not be empty")
        if transport_epoch < 0 or sequence < 0:
            raise ValueError("transport_epoch and sequence must be non-negative")
        with self._lock:
            if producer_instance in self._retired_producers:
                self._rejected_old_producer += 1
                return False
            if self._producer_instance is None:
                self._bind(producer_instance, transport_epoch)
            elif producer_instance != self._producer_instance:
                self._retired_producers.add(self._producer_instance)
                self._bind(producer_instance, transport_epoch)
            elif self._transport_epoch is not None:
                if transport_epoch < self._transport_epoch:
                    self._rejected_old_epoch += 1
                    return False
                if transport_epoch > self._transport_epoch:
                    self._bind(producer_instance, transport_epoch)
            if self._last_sequence is not None and sequence <= self._last_sequence:
                self._rejected_sequence += 1
                return False
            if self._latest is not None:
                self._overwritten += 1
            self._latest = item
            self._last_sequence = sequence
            self._accepted += 1
            return True

    def clear(self) -> None:
        with self._lock:
            self._discard_latest()

    def drain(self) -> ItemT | None:
        with self._lock:
            latest = self._latest
            self._latest = None
            if latest is not None:
                self._drained += 1
            return latest

    def _bind(self, producer_instance: str, transport_epoch: int) -> None:
        if self._producer_instance is not None:
            self._rebinds += 1
        self._discard_latest()
        self._producer_instance = producer_instance
        self._transport_epoch = transport_epoch
        self._last_sequence = None

    def _discard_latest(self) -> None:
        if self._latest is not None:
            self._discarded += 1
            self._latest = None
