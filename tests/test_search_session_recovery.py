"""WebUI「无法开始: search already active in state STARTING」回归修复测试。

覆盖服务端僵尸会话回收逻辑：
  1) worker 已死（alive()==False）的 STARTING 会话，再次开始前被自动回收，不再 conflict；
  2) state_snapshot 惰性回收僵尸 STARTING；
  3) stop_search 直接复位僵尸 STARTING 为 IDLE，可立即重新开始；
  4) 还活着的 STARTING（正常启动中）不被误回收；
  5) 任务理解被拒（TASK_REJECTED）后允许重新开始。

（不依赖真 worker：直接构造僵尸会话状态来驱动服务层回收。）
"""

from __future__ import annotations

import time

import pytest

from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_session_service import (
    STATUS_STARTING,
    SearchSessionService,
)
from app.task_understanding.search_task_context import SearchTaskContext


class _DeadExecutor:
    """Worker already gone: alive() -> False."""

    def set_on_message(self, cb) -> None: ...
    def start(self, params: dict) -> None: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def stop(self) -> None: ...
    def estop(self) -> None: ...
    def status(self) -> dict: return {"state": "stopped", "session_id": None}
    def alive(self) -> bool: return False
    def shutdown(self) -> None: ...


class _AliveExecutor(_DeadExecutor):
    """Healthy worker, just slow to report readiness."""

    def alive(self) -> bool: return True


_TEST_SESSION_DIR = "outputs/live_runs_test_recovery"


@pytest.fixture(autouse=True)
def _isolate_archives(tmp_path):
    global _TEST_SESSION_DIR
    _TEST_SESSION_DIR = str(tmp_path / "sessions")


def _make_service(executor=None) -> SearchSessionService:
    # backend=go2w_experimental（非 mock）会真正走 executor_factory，因此这里的
    # Dead/Alive executor 才会生效；task 理解用本地 mock_fallback，避免调用网络 LLM。
    return SearchSessionService(
        owner=ControlOwner(),
        executor_factory=lambda: executor or _DeadExecutor(),
        session_dir=_TEST_SESSION_DIR,
        task_understanding_runner=lambda text: SearchTaskContext.mock_fallback(text),
    )


def _req(text: str = "测试目标") -> SearchStartRequest:
    return SearchStartRequest.from_dict({
        "task_text": text,
        "target": text,
        "backend": "go2w_experimental",
    })


def _zombie_starting(svc: SearchSessionService, executor) -> None:
    """手动构造一个卡死在 STARTING 的会话（worker 已死/活，视参数）。"""
    svc._status = STATUS_STARTING
    svc._executor = executor
    svc._started_at = time.time()
    svc._session_id = "stale_session"


def test_stale_starting_auto_recovered_on_restart():
    svc = _make_service(_DeadExecutor())
    _zombie_starting(svc, _DeadExecutor())
    # 直接开始：gate 前的自动回收应把僵尸 STARTING 复位，随后正常开始新会话
    second = svc.start_search(_req())
    assert second["ok"] is True, f"僵尸 STARTING 应被自动回收后允许开始: {second}"
    assert "conflict" not in second


def test_state_snapshot_reaps_stale_starting():
    svc = _make_service(_DeadExecutor())
    _zombie_starting(svc, _DeadExecutor())
    svc.state_snapshot()
    assert svc._status == "FAILED"
    assert svc.state_snapshot()["error"]["code"] == "WORKER_INTERRUPTED"
    assert svc.start_search(_req())["ok"] is True


def test_stop_search_resets_stale_starting():
    svc = _make_service(_DeadExecutor())
    _zombie_starting(svc, _DeadExecutor())
    result = svc.stop_search()
    assert result["ok"] is True
    assert result["status"] == "FAILED"
    assert svc.state_snapshot()["finish_reason"] == "WORKER_INTERRUPTED"
    assert svc.start_search(_req())["ok"] is True


def test_alive_starting_is_not_falsely_reaped():
    svc = _make_service(_AliveExecutor())
    _zombie_starting(svc, _AliveExecutor())
    svc.state_snapshot()
    assert svc._status == STATUS_STARTING, "还活着的启动中会话不能被回收"
    blocked = svc.start_search(_req())
    assert blocked["ok"] is False
    assert "already active" in blocked["error"]


def test_task_rejected_then_retry_allowed():
    svc = _make_service()
    svc._status = "TASK_REJECTED"
    result = svc.start_search(_req("改个目标"))
    assert result["ok"] is True, f"TASK_REJECTED 之后应允许重试: {result}"


def test_alive_but_timedout_starting_is_reaped():
    """worker 进程还活着，但 STARTING 超时（如一直等 RGB-D/ROS preflight）也应收割。"""
    svc = _make_service(_AliveExecutor())
    _zombie_starting(svc, _AliveExecutor())
    svc._started_at = time.time() - 3600  # 早就超过 STARTING_TIMEOUT_SEC
    svc.state_snapshot()
    assert svc._status == "FAILED"
    assert svc.start_search(_req())["ok"] is True


# --------------------------------------------------------------------------- #
# STOPPING 卡死：新开始应能强制覆盖旧停止中的会话                            #
# --------------------------------------------------------------------------- #

def _zombie_stopping(svc, executor) -> None:
    svc._status = "STOPPING"
    svc._executor = executor
    svc._started_at = time.time()
    svc._session_id = "stale_stop_session"


def test_stopping_alive_can_be_overridden():
    """旧 worker 还“活着”但在 STOPPING：新搜索必须能把旧会话停掉并立即开始。"""
    svc = _make_service(_AliveExecutor())
    _zombie_stopping(svc, _AliveExecutor())
    result = svc.start_search(_req("新目标"))
    assert result["ok"] is True, f"STOPPING 不应阻塞新开始: {result}"
    assert svc._status == "STARTING"


def test_stopping_dead_can_be_overridden():
    svc = _make_service(_DeadExecutor())
    _zombie_stopping(svc, _DeadExecutor())
    result = svc.start_search(_req("新目标"))
    assert result["ok"] is True, f"死 worker 的 STOPPING 也不应阻塞: {result}"
    assert svc._status == "STARTING"


# --------------------------------------------------------------------------- #
# 刷新页面后：RUNNING 但 worker 已死 -> 自动清成 IDLE，开始按钮可直接用      #
# --------------------------------------------------------------------------- #

def _zombie_running(svc, executor) -> None:
    svc._status = "RUNNING"
    svc._executor = executor
    svc._started_at = time.time()
    svc._session_id = "stale_running_session"


def test_running_dead_worker_auto_reaped_on_snapshot():
    """页面刷新后只剩 RUNNING 状态但 worker 已死：state_snapshot 应自动回 IDLE。"""
    svc = _make_service(_DeadExecutor())
    _zombie_running(svc, _DeadExecutor())
    svc.state_snapshot()  # 等价于前端刷新拉快照
    assert svc._status == "FAILED"
    state = svc.state_snapshot()
    assert state["error"]["code"] == "WORKER_INTERRUPTED"
    assert state["finish_reason"] == "WORKER_INTERRUPTED"
    assert svc.start_search(_req("新目标"))["ok"] is True


def test_running_alive_worker_not_reaped():
    """正常搜索中的 worker 活着，绝不能因为刷新被误回收。"""
    svc = _make_service(_AliveExecutor())
    _zombie_running(svc, _AliveExecutor())
    svc.state_snapshot()
    assert svc._status == "RUNNING"
    blocked = svc.start_search(_req())
    assert blocked["ok"] is False
    assert "already active" in blocked["error"]
