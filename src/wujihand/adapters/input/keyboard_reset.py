"""Keyboard input adapter for an explicit operator reset request."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol


class KeyboardEventSource(Protocol):
    """Minimal event-source surface supplied by the GUI backend."""

    def subscribe_to_keyboard_events(
        self,
        keyboard: object,
        callback: Callable[[object], bool],
    ) -> int: ...

    def unsubscribe_to_keyboard_events(
        self,
        keyboard: object,
        subscription_id: int,
    ) -> None: ...


class KeyboardResetInputAdapter:
    """Latch one reset request for each configured key-press event."""

    def __init__(
        self,
        *,
        event_source: KeyboardEventSource,
        keyboard: object,
        reset_key: object,
        key_press_type: object,
    ) -> None:
        self._event_source = event_source
        self._keyboard = keyboard
        self._reset_key = reset_key
        self._key_press_type = key_press_type
        self._subscription_id: int | None = None
        self._reset_requested = False

    def start(self) -> None:
        if self._subscription_id is not None:
            raise RuntimeError("keyboard reset input is already started")
        self._subscription_id = (
            self._event_source.subscribe_to_keyboard_events(
                self._keyboard,
                self._on_keyboard_event,
            )
        )

    def consume_reset_requested(self) -> bool:
        requested = self._reset_requested
        self._reset_requested = False
        return requested

    def close(self) -> None:
        subscription_id = self._subscription_id
        if subscription_id is None:
            return
        self._event_source.unsubscribe_to_keyboard_events(
            self._keyboard,
            subscription_id,
        )
        self._subscription_id = None
        self._reset_requested = False

    def _on_keyboard_event(self, event: object) -> bool:
        if (
            getattr(event, "type", None) == self._key_press_type
            and getattr(event, "input", None) == self._reset_key
        ):
            self._reset_requested = True
        return True


__all__ = ["KeyboardEventSource", "KeyboardResetInputAdapter"]
