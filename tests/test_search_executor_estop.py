from __future__ import annotations

import subprocess
import time

from app.manual_web_demo.search_executor import SubprocessSearchExecutor


class _BlockedProcess:
    def __init__(self) -> None:
        self.terminated = False
        self.killed = False

    def poll(self) -> int | None:
        return -15 if self.terminated or self.killed else None

    def wait(self, timeout: float | None = None) -> int:
        if self.poll() is None:
            raise subprocess.TimeoutExpired("worker", timeout)
        return self.poll() or 0

    def terminate(self) -> None:
        self.terminated = True

    def kill(self) -> None:
        self.killed = True


def test_estop_force_terminates_worker_blocked_outside_ipc() -> None:
    executor = SubprocessSearchExecutor(cmd=("unused",))
    process = _BlockedProcess()
    executor._proc = process  # type: ignore[assignment]
    executor._ESTOP_GRACE_SEC = 0.01
    executor._TERM_GRACE_SEC = 0.01
    sent: list[dict[str, str]] = []
    executor._send = sent.append  # type: ignore[method-assign]

    executor.estop()
    deadline = time.monotonic() + 1.0
    while not process.terminated and time.monotonic() < deadline:
        time.sleep(0.01)

    assert sent == [{"cmd": "estop"}]
    assert process.terminated is True
    assert process.killed is False
