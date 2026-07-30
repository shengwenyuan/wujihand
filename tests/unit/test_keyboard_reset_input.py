from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import pytest

from wujihand.adapters.input import KeyboardResetInputAdapter


@dataclass(frozen=True)
class Event:
    type: object
    input: object


class FakeEventSource:
    def __init__(self) -> None:
        self.callback: Callable[[object], bool] | None = None
        self.unsubscribed: tuple[object, int] | None = None

    def subscribe_to_keyboard_events(
        self,
        keyboard: object,
        callback: Callable[[object], bool],
    ) -> int:
        del keyboard
        self.callback = callback
        return 17

    def unsubscribe_to_keyboard_events(
        self,
        keyboard: object,
        subscription_id: int,
    ) -> None:
        self.unsubscribed = (keyboard, subscription_id)

    def emit(self, event: Event) -> bool:
        assert self.callback is not None
        return self.callback(event)


def test_keyboard_reset_input_latches_only_configured_key_press() -> None:
    source = FakeEventSource()
    keyboard = object()
    subject = KeyboardResetInputAdapter(
        event_source=source,
        keyboard=keyboard,
        reset_key="R",
        key_press_type="press",
    )

    subject.start()
    assert source.emit(Event(type="release", input="R"))
    assert source.emit(Event(type="press", input="T"))
    assert not subject.consume_reset_requested()

    assert source.emit(Event(type="press", input="R"))
    assert subject.consume_reset_requested()
    assert not subject.consume_reset_requested()

    with pytest.raises(RuntimeError, match="already started"):
        subject.start()
    subject.close()
    assert source.unsubscribed == (keyboard, 17)
