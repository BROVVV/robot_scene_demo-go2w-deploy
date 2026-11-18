"""§14 Full Semantic 失败必须**非致命**（§30 点名的本文件）。

场景原文（计划书 §14.4）：

    注入：Full Semantic timeout / JSON truncation / parse error
    要求：session 继续 / Quick 仍可规划 / semantic health=DEGRADED / 不出现 WORKER_INTERRUPTED

**§14.4 的第 3 条要求在本仓库里没有对应物，本文件不发明它。**
全仓 `grep DEGRADED` 只有三套，都不是"语义健康"：
`app/spatial/spatial_pose_validator.py:14`（位姿健康）、
`app/navigation/stuck_detector.py:30`（`SENSOR_DEGRADED`）、
`app/spatial/slam_web_map_state.py:20`（`DEGRADED_LIO_DRIFT`）。
真实的语义层健康是**错误码**，住在 `AsyncSemanticObservationManager.last_error_code()`，
取值实测为 `FULL_SEMANTIC_TIMEOUT` / `FULL_SEMANTIC_ERROR` / `VLM_PARSE_ERROR` /
`SEMANTIC_NOT_FRESH` 四个。所以本文件测的是**这四个码 + 对应事件**，并在
`test_semantic_health_is_an_error_code_because_no_degraded_state_exists` 里
把"没有 DEGRADED"这件事本身钉住。

同理 §14.2 的代码样例写的是 `except FullSemanticError`，而
`grep FullSemanticError app/` **零命中** —— 那个异常类不存在；真实实现是
`async_semantic_observer.py:259-290` 的 `subprocess.TimeoutExpired` 分支
加一个兜底 `except Exception`。

**为什么这件事重要。** Full Semantic 跑在后台线程里（`max_inflight=1`）。它一旦抛穿，
按 §14.2 的说法会"在 poll 时重新抛到主循环杀死 worker" —— 而 worker 一死，WebUI 侧
就是 `WORKER_INTERRUPTED` 终态，整场搜索报废。所以这里的每一条都是"让狗继续跑"的
必要条件。

**诚实边界**：本文件直接调 `manager._run_analyze(...)` **同步**跑失败路径，不启线程
（沿用 `tests/test_async_semantic_observer.py` 直接调 `_on_result` 的同一套惯用法）。
它证明"给定失败，管理器的状态与事件是什么"，不证明"真机上 VLM 会这样失败"。
"""

from __future__ import annotations

import json
import subprocess

from app.live_robot.async_semantic_observer import (
    AsyncSemanticObservationManager,
    AsyncSemanticRequest,
)
from app.live_robot.semantic_observer import (
    SEMANTIC_FRESH_STATUSES,
    SEMANTIC_STATUS_ERROR,
    SEMANTIC_STATUS_FRESH_QUICK,
    SEMANTIC_STATUS_PENDING,
    SEMANTIC_STATUS_STALE,
    SEMANTIC_STATUS_TIMEOUT,
    SEMANTIC_STATUS_UNAVAILABLE,
    SemanticObservation,
)

POSE = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}


def _quick(frame_id: str = "quick_0") -> SemanticObservation:
    """上一帧成功的 Quick 结果 —— 它就是"Quick 仍可规划"的那份规划依据。"""
    return SemanticObservation(
        frame_id=frame_id, timestamp_sec=1.0, robot_pose=dict(POSE),
        objects=[], relations=[], source="quick_scene_light", stale=False,
        heading_sector=0, semantic_status=SEMANTIC_STATUS_FRESH_QUICK,
    )


def _request(frame_id: str = "frame_1", sequence: int = 1) -> AsyncSemanticRequest:
    return AsyncSemanticRequest(
        sequence=sequence, image_path="frame.jpg", frame_id=frame_id,
        capture_timestamp=1.0, timestamp_quality=None,
        robot_pose=dict(POSE), target_profile=None,
    )


def _raising(exc: BaseException):
    def _analyze(*_args, **_kwargs):
        raise exc
    return _analyze


def _manager(analyze, events: list[dict], *, seeded: bool = True):
    """造一个已经有过一份可用 Quick 结果的管理器。

    先 `seed` 是刻意的：§3.2 的不变量是"失败**不许覆盖**最后一次成功"，在一张白纸上
    测不出"没覆盖"—— 必须有个东西在那儿才谈得上覆没覆盖。
    """
    manager = AsyncSemanticObservationManager(
        analyze, enabled=True, event_sink=events.append,
    )
    if seeded:
        manager.seed(_quick())
    # 让 `_inflight` 与"真的有一发在跑"一致，`finally` 才会把它减回去。
    manager._inflight = 1
    return manager


def _events_of(events: list[dict]) -> list[str]:
    return [event["event"] for event in events]


# --------------------------------------------------------------------------
# A. 三种注入都必须是「记一笔，然后活着回来」
# --------------------------------------------------------------------------

def test_full_semantic_timeout_is_recorded_and_does_not_raise():
    """**注入 1：Full Semantic timeout。** `analyze` 抛
    `subprocess.TimeoutExpired`（VLM 子进程超时就是这条），管理器必须：
    记下 `FULL_SEMANTIC_TIMEOUT`、发 `semantic_timeout` 事件、**正常返回**。

    "正常返回"不是形式要求：`_run_analyze` 是后台线程的 target，异常逃出去只会
    打到 `threading.excepthook`，而 `finally` 之前就抛穿会让 `_inflight` **永远
    减不回去** —— 之后每一帧都走 coalescing 分支，Full Semantic 就此静默停摆，
    而日志里只有一行线程异常。
    """
    events: list[dict] = []
    manager = _manager(
        _raising(subprocess.TimeoutExpired(cmd="vlm", timeout=65.0)), events
    )

    manager._run_analyze(_request())  # 不许抛

    assert _events_of(events) == ["semantic_timeout"]
    assert events[0]["status"] == "timeout"
    assert events[0]["object_count"] == 0
    assert manager.last_error_code() == "FULL_SEMANTIC_TIMEOUT"
    assert "TimeoutExpired" in (manager.last_error() or "")
    assert manager.has_inflight() is False, (
        "超时之后必须把 inflight 放掉，否则后续每一帧都被 coalescing 吞掉"
    )


def test_truncated_json_from_the_analyzer_is_nonfatal():
    """**注入 2：JSON truncation。** §14.3 说的"65 秒后 JSON 被截断"在真机上表现为
    **`json.loads` 在 `analyze_semantic` 内部抛 `JSONDecodeError`**（不是在这里解析），
    于是它走 `_run_analyze` 的兜底 `except Exception` 分支。

    `JSONDecodeError` 没有 `.code` 属性，所以错误码落到 `FULL_SEMANTIC_ERROR` —— 这条
    同时钉住了那个 `getattr(exc, "code", "") or "FULL_SEMANTIC_ERROR"` 的兜底。
    """
    events: list[dict] = []
    truncated = json.JSONDecodeError("Unterminated string", '{"scene_objects": [{"na', 24)
    manager = _manager(_raising(truncated), events)

    manager._run_analyze(_request())

    assert _events_of(events) == ["semantic_error"]
    assert events[0]["status"] == "error"
    assert events[0]["error_code"] == "FULL_SEMANTIC_ERROR"
    assert manager.last_error_code() == "FULL_SEMANTIC_ERROR"
    assert manager.has_inflight() is False


def test_a_payload_that_parses_but_cannot_be_normalized_is_a_parse_error():
    """**注入 3：parse error（JSON 合法但字段不可用）。** `confidence` 是字符串
    `"很高"` 会让 `_from_payload` 里的 `float(...)` 抛 `ValueError`。

    它与上一条的区别是有意义的：JSON 解析成功、`analyze` 也正常返回，坏在**规范化**
    这一步，所以错误码是 `VLM_PARSE_ERROR` 而不是 `FULL_SEMANTIC_ERROR`。分得开才
    查得出"是模型吐了坏 JSON"还是"模型吐了合法但奇怪的 JSON"。
    """
    events: list[dict] = []
    manager = _manager(
        lambda *_a, **_k: {
            "frame_id": "frame_1",
            "scene_objects": [{"name": "垃圾桶", "confidence": "很高"}],
        },
        events,
    )

    manager._run_analyze(_request())

    assert _events_of(events) == ["semantic_error"]
    assert events[0]["error"] == "semantic payload normalization failed"
    assert manager.last_error_code() == "VLM_PARSE_ERROR"
    assert manager.has_inflight() is False


# --------------------------------------------------------------------------
# B. Quick 仍可规划：失败绝不覆盖最后一次成功（§3.2 强制不变量）
# --------------------------------------------------------------------------

def test_no_failure_kind_clears_the_last_usable_result():
    """**§14.4「Quick 仍可规划」。** 三种注入各来一次，`get_latest_completed()`
    必须**始终**是那份成功的 Quick 结果。

    这是狗还能不能动的分界：`run_semantic_exploration.py:1619` 拿
    `get_latest_completed()` 装配本帧的 `semantic`，再据此生成目标与规划。它一旦被
    失败清空或污染，本帧就没有可用的世界模型 —— 轻则规划退化到"看不见任何东西"，
    重则把一次超时当成"当前场景确认无物体"。
    """
    cases = {
        "timeout": _raising(subprocess.TimeoutExpired(cmd="vlm", timeout=65.0)),
        "truncated_json": _raising(json.JSONDecodeError("x", "{", 1)),
        "bad_field": lambda *_a, **_k: {
            "frame_id": "frame_1",
            "scene_objects": [{"name": "桶", "confidence": "很高"}],
        },
    }
    for label, analyze in cases.items():
        events: list[dict] = []
        manager = _manager(analyze, events)

        manager._run_analyze(_request())

        latest = manager.get_latest_completed()
        assert latest is not None, f"{label}: 失败把最后一次可用结果清空了"
        assert latest.frame_id == "quick_0", f"{label}: latest 被换成了别的东西"
        assert latest.semantic_status == SEMANTIC_STATUS_FRESH_QUICK, label


def test_a_non_fresh_result_is_discarded_instead_of_overwriting_latest():
    """**§3.2 强制不变量本身。** `analyze` 这一次**成功返回**，但结果自带
    `semantic_status="pending"` —— 也就是说它没资格当"当前场景"，必须发
    `semantic_discarded` 并原样留下 `latest`。

    危险形状：只看"没抛异常"就收下这份结果。那样一个"还没有结果的占位"会顶掉
    上一份真结果，latest 的语义从"最后一次成功"退化成"最后一次尝试"。
    """
    events: list[dict] = []
    manager = _manager(
        lambda *_a, **_k: {
            "frame_id": "frame_1", "scene_objects": [],
            "semantic_status": SEMANTIC_STATUS_PENDING,
        },
        events,
    )

    manager._run_analyze(_request())

    assert _events_of(events) == ["semantic_discarded"]
    assert events[0]["status"] == SEMANTIC_STATUS_PENDING
    assert manager.last_error_code() == "SEMANTIC_NOT_FRESH"
    assert manager.get_latest_completed().frame_id == "quick_0"


def test_a_failed_job_releases_the_slot_so_the_session_keeps_going():
    """**§14.4「session 继续」。** 失败之后必须还能提交下一帧。

    `max_inflight=1`：`_inflight` 只要有一次没被 `finally` 减回去，之后**每一帧**
    都落到 `_pending` 合并分支、再也不启新任务。表现是"Full Semantic 突然不再更新"，
    而日志里既没有崩溃也没有报错 —— 正是最难查的那种静默停摆。
    """
    events: list[dict] = []
    manager = _manager(_raising(RuntimeError("boom")), events)
    manager._run_analyze(_request())

    assert manager.has_inflight() is False
    assert manager.submit_if_needed(
        image_path="frame2.jpg", frame_id="frame_2", capture_timestamp=9.0,
        robot_pose={"x": 5.0, "y": 0.0, "yaw_deg": 90.0}, target_profile=None,
    ) is True, "失败之后必须还能开始下一帧的 Full Semantic"

    for thread in list(manager._threads):
        thread.join(timeout=5.0)


# --------------------------------------------------------------------------
# C. 没有 DEGRADED；也没有 WORKER_INTERRUPTED
# --------------------------------------------------------------------------

def test_semantic_health_is_an_error_code_because_no_degraded_state_exists():
    """**诚实记录：§14.4 要的 `semantic health=DEGRADED` 在本仓库里不存在。**

    语义层的状态集合是 `semantic_observer.py:23-29` 那七个
    （`fresh_full` / `fresh_quick_scene` / `pending` / `stale` / `timeout` /
    `error` / `unavailable`）—— **没有 `DEGRADED`**，`"DEGRADED"` 也不在这七个
    里。另外两个集合（位姿健康、LIO 漂移）各有各的 `DEGRADED` 词，混用会让
    "语义失败"和"地图发散"长得一样，而两者的处置完全不同。

    健康的真实表达是错误码。这条把四个码钉住，谁改了就红。
    """
    statuses = {
        SEMANTIC_STATUS_FRESH_QUICK, SEMANTIC_STATUS_PENDING,
        SEMANTIC_STATUS_STALE, SEMANTIC_STATUS_TIMEOUT,
        SEMANTIC_STATUS_ERROR, SEMANTIC_STATUS_UNAVAILABLE,
    }
    assert "DEGRADED" not in statuses, (
        "语义层出现 DEGRADED 了 —— 请先确认它和 spatial_pose_validator / "
        "slam_web_map_state 的同名词是不是同一件事，再决定要不要统一"
    )
    # 只有两个状态算"新鲜"，失败态一个都不在里面 —— 这条是 B 节的底层依据。
    assert SEMANTIC_FRESH_STATUSES == {"fresh_full", "fresh_quick_scene"}
    for failed in (SEMANTIC_STATUS_TIMEOUT, SEMANTIC_STATUS_ERROR,
                   SEMANTIC_STATUS_PENDING, SEMANTIC_STATUS_STALE,
                   SEMANTIC_STATUS_UNAVAILABLE):
        assert failed not in SEMANTIC_FRESH_STATUSES, failed

    # 四个错误码就是这一层全部的"健康降级"表达。
    codes = set()
    for analyze in (
        _raising(subprocess.TimeoutExpired(cmd="v", timeout=1.0)),
        _raising(json.JSONDecodeError("t", "{", 0)),
        lambda *_a, **_k: {"frame_id": "f", "scene_objects": [{"confidence": "高"}]},
        lambda *_a, **_k: {"frame_id": "f", "scene_objects": [],
                           "semantic_status": SEMANTIC_STATUS_PENDING},
    ):
        events: list[dict] = []
        _manager(analyze, events)._run_analyze(_request())
        codes.add(manager_code(events))

    assert codes == {
        "FULL_SEMANTIC_TIMEOUT", "FULL_SEMANTIC_ERROR",
        "VLM_PARSE_ERROR", "SEMANTIC_NOT_FRESH",
    }, f"错误码集合变了：{sorted(codes)}"


def manager_code(events: list[dict]) -> str:
    """从事件里反推错误码 —— 事件是给操作员看的，码是给判据看的，两者都要在。

    这里刻意不直接读 `manager.last_error_code()`（上一条已经读过），而是从**事件流**
    反推：事件是跨进程边界真正落到 JSONL 上的东西，事后复盘只有它。
    """
    event = events[0]
    if event["event"] == "semantic_timeout":
        return "FULL_SEMANTIC_TIMEOUT"
    if event["event"] == "semantic_discarded":
        return "SEMANTIC_NOT_FRESH"
    return event.get("error_code") or "VLM_PARSE_ERROR"


def test_a_full_semantic_failure_never_becomes_worker_interrupted():
    """**§14.4「不出现 WORKER_INTERRUPTED」。**

    `WORKER_INTERRUPTED` 是**另一层**的终态：由 `search_session_service.py:245`
    在搜索 worker 进程死掉时发出，`search_errors.py:72` 把它标成 terminal。它一旦
    出现，整场搜索就结束了 —— 不是"降级"，是"报废"。

    本层的责任就是让 Full Semantic 的失败**传不到那一层**：这条把四种失败各跑一遍，
    断言它们全部止步于 `semantic_*` 事件，一个都没变成 worker 级的终态。这是对
    "非致命"最直接的可观测定义。
    """
    cases = [
        _raising(subprocess.TimeoutExpired(cmd="v", timeout=1.0)),
        _raising(json.JSONDecodeError("t", "{", 0)),
        _raising(RuntimeError("boom")),
        lambda *_a, **_k: {"frame_id": "f", "scene_objects": [{"confidence": "高"}]},
    ]
    for analyze in cases:
        events: list[dict] = []
        manager = _manager(analyze, events)
        manager._run_analyze(_request())  # 不许抛 —— 抛穿就是 worker 的死因

        assert events, "失败必须留下事件，静默失败无法事后复盘"
        assert all(event["event"].startswith("semantic_") for event in events), (
            f"本层只许发 semantic_* 事件，实际：{_events_of(events)}"
        )
        assert not any(
            "WORKER_INTERRUPTED" in json.dumps(event) for event in events
        ), "Full Semantic 的失败绝不许升级成 worker 终态"
