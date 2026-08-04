"""Owned background lifecycle for a ROS executor."""

from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
import time
from typing import Callable, Protocol


class SpinExecutor(Protocol):
    def spin(self) -> None: ...

    def shutdown(self, timeout_sec: float | None = None) -> bool: ...


@dataclass(frozen=True, slots=True)
class ExecutorThreadMetrics:
    started_ns: int | None
    stopped_ns: int | None
    failure: str | None


class RosExecutorThread:
    """Run one executor continuously and make its failure observable."""

    def __init__(
        self,
        executor: SpinExecutor,
        *,
        name: str = "wujihand-ros-executor",
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self._executor = executor
        self._name = name
        self._clock_ns = clock_ns
        self._state_lock = Lock()
        self._entered = Event()
        self._thread: Thread | None = None
        self._stop_requested = False
        self._started_ns: int | None = None
        self._stopped_ns: int | None = None
        self._failure: BaseException | None = None

    @property
    def metrics(self) -> ExecutorThreadMetrics:
        with self._state_lock:
            return ExecutorThreadMetrics(
                started_ns=self._started_ns,
                stopped_ns=self._stopped_ns,
                failure=(
                    None
                    if self._failure is None
                    else f"{type(self._failure).__name__}:{self._failure}"
                ),
            )

    def start(self) -> None:
        with self._state_lock:
            if self._thread is not None:
                raise RuntimeError("ROS executor thread is already started")
            self._started_ns = self._clock_ns()
            self._thread = Thread(target=self._run, name=self._name)
            thread = self._thread
        thread.start()
        if not self._entered.wait(timeout=1.0):
            raise RuntimeError("ROS executor thread did not start")
        self.raise_if_failed()

    def raise_if_failed(self) -> None:
        with self._state_lock:
            failure = self._failure
            thread = self._thread
            stopped_unexpectedly = (
                thread is not None
                and not thread.is_alive()
                and not self._stop_requested
                and failure is None
            )
        if failure is not None:
            raise RuntimeError("ROS executor thread failed") from failure
        if stopped_unexpectedly:
            raise RuntimeError("ROS executor thread stopped unexpectedly")

    def stop(self, *, timeout_s: float = 5.0) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        with self._state_lock:
            thread = self._thread
            if thread is None:
                return
            self._stop_requested = True
        shutdown_complete = self._executor.shutdown(timeout_sec=timeout_s)
        thread.join(timeout=timeout_s)
        if thread.is_alive():
            raise TimeoutError("ROS executor thread did not stop")
        if not shutdown_complete:
            raise TimeoutError("ROS executor did not finish outstanding callbacks")
        self.raise_if_failed()

    def _run(self) -> None:
        self._entered.set()
        try:
            self._executor.spin()
        except BaseException as exc:
            with self._state_lock:
                self._failure = exc
        finally:
            stopped_ns = self._clock_ns()
            with self._state_lock:
                self._stopped_ns = stopped_ns


__all__ = ["ExecutorThreadMetrics", "RosExecutorThread", "SpinExecutor"]
