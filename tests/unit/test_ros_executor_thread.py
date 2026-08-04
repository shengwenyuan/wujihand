from __future__ import annotations

from threading import Event

import pytest

from wujihand_ros2.executor_thread import RosExecutorThread


class BlockingExecutor:
    def __init__(self) -> None:
        self.spinning = Event()
        self.stopped = Event()

    def spin(self) -> None:
        self.spinning.set()
        self.stopped.wait()

    def shutdown(self, timeout_sec: float | None = None) -> bool:
        del timeout_sec
        self.stopped.set()
        return True


class FailingExecutor(BlockingExecutor):
    def __init__(self) -> None:
        super().__init__()
        self.release_failure = Event()
        self.failed = Event()

    def spin(self) -> None:
        self.spinning.set()
        self.release_failure.wait()
        try:
            raise ValueError("fixture failure")
        finally:
            self.failed.set()


def test_executor_thread_runs_until_owned_shutdown() -> None:
    executor = BlockingExecutor()
    worker = RosExecutorThread(executor)

    worker.start()
    assert executor.spinning.wait(timeout=1.0)
    worker.raise_if_failed()
    worker.stop()

    assert worker.metrics.started_ns is not None
    assert worker.metrics.stopped_ns is not None
    assert worker.metrics.failure is None


def test_executor_thread_propagates_background_failure() -> None:
    executor = FailingExecutor()
    worker = RosExecutorThread(executor)
    worker.start()
    assert executor.spinning.wait(timeout=1.0)
    executor.release_failure.set()
    assert executor.failed.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="executor thread failed") as failure:
        worker.raise_if_failed()

    assert isinstance(failure.value.__cause__, ValueError)
    assert worker.metrics.failure == "ValueError:fixture failure"
