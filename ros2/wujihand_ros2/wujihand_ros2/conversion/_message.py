"""Small lazy boundary around generated ROS message classes."""

from __future__ import annotations

from collections.abc import Callable
import importlib
from typing import TypeVar, cast


MessageT = TypeVar("MessageT")


def new_message(
    factory: Callable[[], MessageT] | None,
    *,
    class_name: str,
) -> MessageT:
    if factory is not None:
        return factory()
    module = importlib.import_module("wujihand_interfaces.msg")
    message_type = getattr(module, class_name)
    return cast(MessageT, message_type())
