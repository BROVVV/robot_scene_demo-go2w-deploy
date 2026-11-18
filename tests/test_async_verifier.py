"""计划书 §5.1.5 有界异步复核的判据。

**这个文件要钉住的不变量只有一条，但它有四个出口**：复核"没有结论"**不是**
"目标不存在"。超时、worker 抛异常、返回体读不懂、结果晚到被丢弃 —— 这四种
情形在旧代码里都会走到同一个 `confirmed=False`，然后被
`target_possible_resolved_absent` 记成 ABSENT。

**确定性**：所有等待都由注入的 `now`/`sleep` 驱动，真实墙钟只用于证明
"轮询没有阻塞"（那一条本来就是在断言墙钟）。worker 线程一律用
`threading.Event` 卡住/放行，不用 `sleep` 猜时序。
"""

from __future__ import annotations

import ast
import pathlib
import threading
import time

import pytest

from app.live_robot.async_verifier import (
    VERIFY_STATUS_CONFIRMED,
    VERIFY_STATUS_REJECTED,
    VERIFY_STATUS_UNKNOWN,
    VERIFY_UNKNOWN_ERROR,
    VERIFY_UNKNOWN_TIMEOUT,
    AsyncVerifierManager,
    VerificationResult,
    result_matches_submission,
)
from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    SemanticMatch,
    VerificationOutcome,
    verification_is_terminal_negative,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import ExplorationBudget, load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import LiveObservation
from app.perception.target_state import (
    TARGET_ABSENT,
    TARGET_POSSIBLE,
    TARGET_PRESENT,
    TARGET_UNKNOWN,
    normalize_target_state,
    target_state_is_present,
)


class _Clock:
    """注入的假钟：`sleep` 推进时间，不真的睡。"""

    def __init__(self, start: float = 0.0) -> None:
        self.t = float(start)

    def now(self) -> float:
        return self.t

    def sleep(self, seconds: float) -> None:
        self.t += max(0.0, float(seconds))


class _Worker:
    """可控的复核实现：被调用时**卡住**，直到测试放行。

    `started` 让测试确信 worker 真的进了 `verify`（否则"结果被丢弃"可能只是
    "结果压根没产生"）。
    """

    def __init__(self, payload=None, *, error: BaseException | None = None) -> None:
        self.payload = {"is_target": True, "confidence": 0.9, "reason_zh": "看到了"} \
            if payload is None else payload
        self.error = error
        self.started = threading.Event()
        self.release = threading.Event()
        self.calls: list[tuple] = []

    def __call__(self, image_path, bbox, env, *, frame_id="", **_kwargs):
        self.calls.append((image_path, tuple(bbox), frame_id))
        self.started.set()
        # 没被放行就永远不返回 —— 模拟 daemon 那个"取消不了"的慢请求。
        self.release.wait(timeout=10.0)
        if self.error is not None:
            raise self.error
        return dict(self.payload)


def _manager(worker, clock: _Clock, **kwargs) -> AsyncVerifierManager:
    events: list[dict] = []
    kwargs.setdefault("enabled", True)
    manager = AsyncVerifierManager(
        worker,
        now=clock.now,
        sleep=clock.sleep,
        event_sink=events.append,
        **kwargs,
    )
    manager._test_events = events  # noqa: SLF001 - 只给本文件的断言用
    return manager


def _submit(manager, *, candidate_id="det_abc", frame_id="obs_1") -> int:
    """提交一次，返回**序号**（>0 起了线程，0 被合并/未启用）。"""
    return manager.submit(
        frame_id=frame_id,
        candidate_id=candidate_id,
        image_path="/tmp/frame.jpg",
        bbox=[0.1, 0.1, 0.3, 0.3],
        env={},
    )


def _wait_for_submission(manager, sequence: int, timeout: float = 5.0):
    """等某一次提交出结论 —— 判据里不许用 `get_latest`（它跨帧存活）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        manager.poll_completed()
        found = manager.get_result_for(sequence)
        if found is not None:
            return found
        time.sleep(0.005)
    return None


def _events(manager, name: str) -> list[dict]:
    return [item for item in manager._test_events if item["event"] == name]  # noqa: SLF001


# --------------------------------------------------------------------------- #
# 1) 轮询不阻塞；到点出 UNKNOWN
# --------------------------------------------------------------------------- #
def test_poll_returns_immediately_while_the_worker_is_still_stuck():
    """搜索线程调 `poll_completed()` 必须**立刻**返回，哪怕 worker 永远不返回。

    这条是"异步"的字面定义。断言分两半：返回值是空的（没结论），且墙钟没花掉
    —— 后者是因为 worker 被卡住，poll 若等它就一定超时。1.0 s 的宽松上界足够
    区分"立刻"与"等了 12 s 的客户端超时"。
    """
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, client_timeout_s=12.0)
    assert _submit(manager) > 0
    assert worker.started.wait(timeout=5.0), "worker 没被调起来，这条判据就没意义"

    started = time.monotonic()
    drained = manager.poll_completed()
    elapsed = time.monotonic() - started

    assert drained == [], "worker 还没出结论，poll 不该凭空造一条出来"
    assert elapsed < 1.0, f"poll 阻塞了 {elapsed:.2f}s —— 它必须是非阻塞的"
    assert manager.queue_depth()["inflight"] == 1, "请求仍在飞（只是没人等它）"
    worker.release.set()


def test_a_hung_worker_becomes_unknown_at_the_client_deadline():
    """永挂的 worker ⇒ 客户端到点自己判 UNKNOWN，理由必须是 timeout。"""
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, client_timeout_s=12.0)
    _submit(manager)
    assert worker.started.wait(timeout=5.0)

    clock.t = 11.9
    assert manager.poll_completed() == [], "还没到截止线，不许提前判超时"
    clock.t = 12.1
    drained = manager.poll_completed()

    assert len(drained) == 1
    result = drained[0]
    assert result.status == VERIFY_STATUS_UNKNOWN
    assert result.unknown_reason == VERIFY_UNKNOWN_TIMEOUT
    assert result.client_timed_out is True
    assert result.terminal is False, "UNKNOWN 不是终态判定，下游不许拿它改目标态"
    assert result.confirmed is False
    assert _events(manager, "verify_client_timeout"), "超时要留事件，否则事后查不出来"
    worker.release.set()


def test_the_late_result_after_the_client_timeout_is_discarded():
    """客户端已经按超时判过了，worker 后来才返回 ⇒ 一律丢弃。

    这是"daemon 取消不了"的正面处置：**不假装取消**，但也不让它的结论落地。
    这里喂的是 `is_target=True` —— 若没有这道丢弃，就会凭空多出一个"找到了"。
    """
    worker = _Worker({"is_target": True, "confidence": 0.99, "reason_zh": "晚到的确认"})
    clock = _Clock()
    manager = _manager(worker, clock, client_timeout_s=12.0)
    _submit(manager)
    assert worker.started.wait(timeout=5.0)

    clock.t = 12.1
    assert manager.poll_completed()[0].status == VERIFY_STATUS_UNKNOWN

    worker.release.set()
    _wait_until(lambda: not manager.has_inflight())
    # 给 worker 的收尾留一拍（它是另一个线程）。
    _wait_until(lambda: _events(manager, "verify_result_discarded"))

    assert manager.poll_completed() == [], "晚到的结论不许进已完成队列"
    assert manager.get_latest("det_abc").status == VERIFY_STATUS_UNKNOWN
    discarded = _events(manager, "verify_result_discarded")
    assert discarded and discarded[0]["reason"] == "stale_after_client_timeout"


# --------------------------------------------------------------------------- #
# 2) 序号丢弃：乱序不许造出"找到了"
# --------------------------------------------------------------------------- #
def test_an_out_of_order_result_cannot_confirm_an_already_rejected_candidate():
    """同一候选的两条请求：新的先回（否定），旧的后回（确认）⇒ 旧的必须被丢。

    这是"删掉序号丢弃就会红"的那一条：没有它，迟到的旧确认会覆盖新否定，
    于是刚被否掉的候选在下一次 `get_latest` 里变成"确认"。
    """
    first = _Worker({"is_target": True, "confidence": 0.99, "reason_zh": "旧的确认"})
    second = _Worker({"is_target": False, "confidence": 0.1, "reason_zh": "新的否定"})
    gate = {"n": 0}

    def verify(image_path, bbox, env, *, frame_id="", **_kwargs):
        gate["n"] += 1
        return (first if gate["n"] == 1 else second)(
            image_path, bbox, env, frame_id=frame_id
        )

    clock = _Clock()
    manager = _manager(verify, clock, max_inflight=2, client_timeout_s=60.0)
    assert _submit(manager) > 0              # seq 1 — 旧的
    assert first.started.wait(timeout=5.0)
    assert _submit(manager) > 0              # seq 2 — 新的
    assert second.started.wait(timeout=5.0)

    second.release.set()
    _wait_until(lambda: manager.get_latest("det_abc") is not None)
    assert manager.get_latest("det_abc").status == VERIFY_STATUS_REJECTED

    first.release.set()
    _wait_until(lambda: _events(manager, "verify_result_discarded"))

    latest = manager.get_latest("det_abc")
    assert latest.status == VERIFY_STATUS_REJECTED, (
        "迟到的旧确认覆盖了新否定 —— 序号丢弃没了"
    )
    drained = manager.poll_completed()
    assert [item.status for item in drained] == [VERIFY_STATUS_REJECTED], (
        f"迟到的旧结论漏进了已完成队列：{[item.status for item in drained]}"
    )
    assert manager.poll_completed() == []
    assert _events(manager, "verify_result_discarded")[0]["reason"] == "stale_result"


def test_a_result_from_a_previous_generation_is_dropped():
    """换目标（推进代次）之后，上一代的结论不许算到这一代头上。"""
    worker = _Worker({"is_target": True, "confidence": 0.99, "reason_zh": "上一代"})
    clock = _Clock()
    manager = _manager(worker, clock, client_timeout_s=60.0)
    manager.set_task_generation(7)
    manager.submit(
        frame_id="obs_1", candidate_id="det_abc", image_path="/tmp/f.jpg",
        bbox=[0.1, 0.1, 0.2, 0.2], env={}, task_generation=7,
    )
    assert worker.started.wait(timeout=5.0)

    manager.set_task_generation(8)
    worker.release.set()
    _wait_until(lambda: _events(manager, "verify_result_discarded"))

    assert manager.get_latest("det_abc", task_generation=8) is None
    assert manager.get_latest("det_abc", task_generation=7) is None, "旧代的结果不许留"
    assert _events(manager, "verify_generation_advanced")
    assert _events(manager, "verify_result_discarded")[0]["reason"] == "stale_task_generation"


# --------------------------------------------------------------------------- #
# 3) 有界：合并与上限
# --------------------------------------------------------------------------- #
def test_a_client_timeout_does_not_release_the_physical_slot():
    """审查 P1-b：超时只解绑客户端，**不**释放物理在途名额（计划书 §5.1.5）。

    daemon 取消不了，那个请求客观上还占着并发槽。旧代码在超时分支里
    `del self._inflight[...]`，于是 `submit()` 只看 `len(_inflight)` 就会再起一个
    线程 —— 实际在跑两个，`max_inflight=1` 成摆设，而 daemon 是单槽串行的，
    多出来的那个只会把后到的复核排到更后面。
    """
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, max_inflight=1, max_pending=0,
                       client_timeout_s=12.0)
    first = _submit(manager, candidate_id="det_a")
    assert first > 0
    assert worker.started.wait(timeout=5.0)

    clock.t = 12.1
    drained = manager.poll_completed()
    assert drained[0].status == VERIFY_STATUS_UNKNOWN
    depth = manager.queue_depth()
    assert depth["inflight"] == 1, (
        "超时把物理名额放掉了 —— 下一次提交就会再起一个线程，超出发送上限"
    )
    assert depth["client_timed_out"] == 1, "要能看出这个在途请求已经不再被等待"

    # 连续提交都不能再起第二个物理请求。
    assert _submit(manager, candidate_id="det_b") == 0
    assert _submit(manager, candidate_id="det_c") == 0
    assert len(worker.calls) == 1, (
        f"超时后起了第 {len(worker.calls)} 个物理请求 —— max_inflight 没限住"
    )

    worker.release.set()
    _wait_until(lambda: not manager.has_inflight())
    assert manager.queue_depth()["client_timed_out"] == 0, "真返回之后名额才真的释放"


def test_a_second_submission_coalesces_into_pending_instead_of_calling_the_provider():
    """在飞的没回来时，第二次提交**不**打 provider —— 它进 pending。

    这是"max_inflight=1"的实际含义，也是"不设上界就会排队雪崩"的正面形态。
    """
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, max_inflight=1, max_pending=1, client_timeout_s=60.0)
    assert _submit(manager, candidate_id="det_a") > 0
    assert worker.started.wait(timeout=5.0)

    assert _submit(manager, candidate_id="det_b") == 0, "在飞满了就该合并"
    assert len(worker.calls) == 1, "合并掉的请求不许打 provider"
    assert manager.queue_depth()["pending"] == 1

    worker.release.set()
    _wait_until(lambda: len(worker.calls) == 2)
    assert worker.calls[1][2] == "obs_1"


def test_the_pending_queue_is_bounded_and_the_dropped_ones_are_reported():
    """pending 上限保留最新的，被合并掉的要**记一笔**，不能静默丢。"""
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, max_inflight=1, max_pending=1, client_timeout_s=60.0)
    _submit(manager, candidate_id="det_a")
    assert worker.started.wait(timeout=5.0)

    for candidate in ("det_b", "det_c", "det_d"):
        assert _submit(manager, candidate_id=candidate) == 0

    assert manager.queue_depth()["pending"] == 1
    assert manager.queue_depth()["discarded_obsolete"] == 2
    dropped = _events(manager, "verify_request_discarded")
    assert [item["candidate_id"] for item in dropped] == ["det_b", "det_c"]
    worker.release.set()


# --------------------------------------------------------------------------- #
# 4) 没有判定就不是否定
# --------------------------------------------------------------------------- #
def test_a_payload_without_a_verdict_is_unknown_not_rejected():
    """返回体读不懂 ⇒ UNKNOWN。当成 REJECTED 就是拿解析失败去否掉一个候选。"""
    clock = _Clock()
    manager = _manager(lambda *a, **k: {"reason_zh": "模型说了点什么"}, clock)
    _submit(manager)
    _wait_until(lambda: manager.get_latest("det_abc") is not None)
    result = manager.get_latest("det_abc")
    assert result.status == VERIFY_STATUS_UNKNOWN
    assert result.unknown_reason == VERIFY_UNKNOWN_ERROR


def test_a_worker_exception_is_unknown_not_rejected():
    clock = _Clock()

    def boom(*_a, **_k):
        raise RuntimeError("daemon 挂了")

    manager = _manager(boom, clock)
    _submit(manager)
    _wait_until(lambda: manager.get_latest("det_abc") is not None)
    result = manager.get_latest("det_abc")
    assert result.status == VERIFY_STATUS_UNKNOWN
    assert result.unknown_reason == VERIFY_UNKNOWN_ERROR
    assert "RuntimeError" in result.reason_zh


def test_the_three_verify_statuses_are_not_interchangeable():
    """把 UNKNOWN 与两个终态混起来，等于把"没验"当成"验过了"。"""
    clock = _Clock()

    def verdict(payload):
        manager = _manager(lambda *a, **k: payload, _Clock())
        _submit(manager)
        _wait_until(lambda: manager.get_latest("det_abc") is not None)
        return manager.get_latest("det_abc")

    assert verdict({"is_target": True}).status == VERIFY_STATUS_CONFIRMED
    assert verdict({"is_target": False}).status == VERIFY_STATUS_REJECTED
    assert verdict({"is_target": True}).terminal is True
    assert clock.now() == 0.0, "这些判定不该花时间"


# --------------------------------------------------------------------------- #
# 5) await_submission：有上界，到点出 UNKNOWN，且只认本次提交
# --------------------------------------------------------------------------- #
def test_await_submission_returns_the_verdict_as_soon_as_it_lands():
    """答案一到就返回 —— 不为等满而等满。

    这一条刻意用**真钟**：假钟的 `sleep` 不花墙钟，会把整个 12 s 预算在几毫秒内
    跑完（那测的是"到点返回"，不是"早到早返回"）。真钟下 worker 0.05 s 就放行，
    12 s 的预算足够安全。
    """
    events: list[dict] = []
    worker = _Worker({"is_target": True, "confidence": 0.8, "reason_zh": "确认"})
    manager = AsyncVerifierManager(
        worker, enabled=True, client_timeout_s=12.0, event_sink=events.append
    )
    sequence = _submit(manager)
    assert worker.started.wait(timeout=5.0)

    def _release_soon():
        time.sleep(0.05)
        worker.release.set()

    threading.Thread(target=_release_soon, daemon=True).start()
    result = manager.await_submission(sequence)
    assert result is not None and result.status == VERIFY_STATUS_CONFIRMED
    assert result.sequence == sequence, "拿回来的必须就是这一次提交的结论"
    assert result.terminal is True


def test_await_submission_gives_up_at_the_deadline_with_an_unknown():
    """永挂 + 假钟 ⇒ `await_submission` 到点返回 UNKNOWN，而不是把线程钉死。"""
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, client_timeout_s=12.0)
    sequence = _submit(manager)
    assert worker.started.wait(timeout=5.0)

    started = time.monotonic()
    result = manager.await_submission(sequence)
    elapsed = time.monotonic() - started

    assert result is not None
    assert result.status == VERIFY_STATUS_UNKNOWN
    assert result.client_timed_out is True
    assert elapsed < 2.0, "等待走的是注入的假钟，不该真睡 12 秒"
    assert clock.now() >= 12.0, "假钟必须真的走到截止线（否则是在空转）"
    worker.release.set()


def test_a_new_frame_cannot_consume_the_previous_frames_verdict():
    """审查 P1-a：同一候选跨帧，前帧已确认 ⇒ 后帧**不许**拿它当自己的结论。

    框和标签不变时 `entity_id_for()` 两帧算出来是同一个 ID，所以"按候选取最近
    一条结论"（旧 `await_result(candidate_id)`）会在后帧刚提交、复核还没跑完时
    立刻返回前帧的 CONFIRMED —— 生产入口会把它转成 `confirmed=True`，凭空宣布
    找到目标。这里钉住：后帧只拿得到**自己那一次**的结论，拿不到就是 None。
    """
    first = _Worker({"is_target": True, "confidence": 0.99, "reason_zh": "第一帧确认"})
    second = _Worker()  # 第二帧永挂：模拟这一帧的复核还没回来
    gate = {"n": 0}

    def verify(image_path, bbox, env, *, frame_id="", **_kwargs):
        gate["n"] += 1
        worker = first if gate["n"] == 1 else second
        return worker(image_path, bbox, env, frame_id=frame_id)

    clock = _Clock()
    manager = _manager(verify, clock, max_inflight=1, client_timeout_s=60.0)

    seq1 = _submit(manager, candidate_id="det_same", frame_id="frame1")
    assert seq1 > 0
    assert first.started.wait(timeout=5.0)
    first.release.set()
    landed = _wait_for_submission(manager, seq1)
    assert landed is not None and landed.status == VERIFY_STATUS_CONFIRMED
    assert landed.frame_id == "frame1"

    seq2 = _submit(manager, candidate_id="det_same", frame_id="frame2")
    assert seq2 > 0, "第一帧已经结束、槽空着，第二帧该真的起线程"
    assert second.started.wait(timeout=5.0)

    leaked = manager.await_submission(seq2, timeout_s=0.0)
    assert leaked is None, (
        "新帧拿到了上一帧的结论："
        f"status={getattr(leaked, 'status', None)} frame_id={getattr(leaked, 'frame_id', None)}"
    )
    # 旧结论本身还在（只是不许被新帧消费）—— 这正是旧接口的坑。
    assert manager.get_latest("det_same").frame_id == "frame1"
    second.release.set()


def test_the_entry_identity_check_refuses_any_other_submission():
    """生产入口的当面核验：四项里**任何一项**对不上都不算数（审查 P1-a）。

    序号绑定是第一道；这一道防的是"上游把序号接错"——那时第一道会乖乖交出
    那一次的结论，而它属于别的帧 / 别的候选 / 别的代次。放它过去就是把别人的
    结论当自己的判定，也就是假"找到了"。
    """
    result = VerificationResult(
        sequence=7, task_generation=3, frame_id="obs_1",
        candidate_id="det_abc", status=VERIFY_STATUS_CONFIRMED,
    )
    good = dict(sequence=7, frame_id="obs_1", candidate_id="det_abc", task_generation=3)
    assert result_matches_submission(result, **good) is True

    for field, wrong in (
        ("sequence", 8),
        ("frame_id", "obs_2"),
        ("candidate_id", "det_other"),
        ("task_generation", 4),
    ):
        assert result_matches_submission(result, **{**good, field: wrong}) is False, (
            f"{field} 对不上却放行了 —— 别人的结论会被当成这一帧的判定"
        )
    assert result_matches_submission(None, **good) is False, "没有结论就是不匹配"


def test_a_coalesced_submission_has_no_verdict_to_wait_for():
    """被合并进待办的提交返回序号 0 ⇒ `await_submission(0)` 立刻给 None。

    它没有可等的结论。等一个永远不会出现的序号只是白等到截止线，更要紧的是
    不能顺手把这个候选的历史结论交出去。
    """
    worker = _Worker()
    clock = _Clock()
    manager = _manager(worker, clock, max_inflight=1, max_pending=1,
                       client_timeout_s=60.0)
    assert _submit(manager, candidate_id="det_a") > 0
    assert worker.started.wait(timeout=5.0)

    coalesced = _submit(manager, candidate_id="det_b")
    assert coalesced == 0
    assert manager.await_submission(coalesced, timeout_s=0.0) is None
    assert manager.get_result_for(coalesced) is None
    worker.release.set()


# --------------------------------------------------------------------------- #
# 6) explorer 侧：UNKNOWN 不许落 ABSENT
# --------------------------------------------------------------------------- #
def _explorer(observations, policy, verifier):
    cursor = [0]

    def observe() -> LiveObservation:
        index = min(cursor[0], len(observations) - 1)
        cursor[0] += 1
        return observations[index]

    return AutonomousExplorer(
        target="目标",
        observer=observe,
        matcher=lambda observation: SemanticMatch(
            has_candidate=observation.target_state == TARGET_PRESENT),
        verifier=verifier,
        backend=MockBackend(),
        policy=policy,
        graph=ExplorationGraph(session_id="async_verify_test"),
    )


def _observation(index: int, state: str) -> LiveObservation:
    return LiveObservation(
        bundle_id=f"obs_{index}", timestamp=float(index),
        target_match={"target_state": state, "target_present": state == TARGET_PRESENT},
    )


def _possible_policy():
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(max_planning_cycles=1, max_motion_steps=1)
    return policy


def test_exhausted_possible_with_an_unknown_verdict_keeps_possible():
    """复核**没结论**时预算耗尽 ⇒ 发 `target_possible_unresolved`，不落 ABSENT。

    这条就是本任务的核心断言：旧代码在这里无条件降 ABSENT。
    """
    explorer = _explorer(
        [_observation(index, TARGET_POSSIBLE) for index in range(4)],
        _possible_policy(),
        lambda _observation, _match: VerificationOutcome(
            confirmed=False, attempts=1,
            reason_zh="no verification verdict within client timeout",
            status=VERIFY_STATUS_UNKNOWN,
        ),
    )
    explorer.run()

    events = [item["event"] for item in explorer.events]
    assert "target_possible_unresolved" in events
    assert "target_possible_resolved_absent" not in events, (
        "超时被记成了'目标不存在' —— 这正是 §5.1.5 要修的那个假否定"
    )
    unresolved = [item for item in explorer.events
                  if item["event"] == "target_possible_unresolved"][0]
    assert unresolved["target_state"] == TARGET_UNKNOWN
    assert unresolved["status"] == VERIFY_STATUS_UNKNOWN
    # 候选位仍要释放，否则规划会一直围着它转（但释放 ≠ 判它不存在）。
    assert "action_start" in events, "候选位没释放，搜索卡住了"


def test_exhausted_possible_with_a_terminal_rejection_still_goes_absent():
    """对照组：复核**真的跑完并说不是** ⇒ 照旧降 ABSENT。

    没有这一条，上面那条就可能被"什么都不降级"蒙混过去。
    """
    explorer = _explorer(
        [_observation(index, TARGET_POSSIBLE) for index in range(4)],
        _possible_policy(),
        lambda _observation, _match: VerificationOutcome(
            confirmed=False, attempts=1, reason_zh="不是它",
            status=VERIFY_STATUS_REJECTED,
        ),
    )
    explorer.run()

    events = [item["event"] for item in explorer.events]
    assert "target_possible_resolved_absent" in events
    assert "target_possible_unresolved" not in events


def test_a_verifier_that_raises_is_not_a_negative():
    """复核**抛异常**同样不是"目标不存在"的证据。"""
    def boom(_observation, _match):
        raise RuntimeError("verify 挂了")

    explorer = _explorer(
        [_observation(index, TARGET_POSSIBLE) for index in range(4)],
        _possible_policy(),
        boom,
    )
    explorer.run()
    events = [item["event"] for item in explorer.events]
    assert "verification_error" in events
    assert "target_possible_resolved_absent" not in events
    assert "target_possible_unresolved" in events


def test_every_unknown_verdict_leaves_the_target_possible_and_never_found():
    """超时的那一轮**绝不能**以 TARGET_FOUND 收尾 —— 没有假"找到"。"""
    explorer = _explorer(
        [_observation(index, TARGET_POSSIBLE) for index in range(6)],
        _possible_policy(),
        lambda _observation, _match: VerificationOutcome(
            confirmed=False, attempts=1, status=VERIFY_STATUS_UNKNOWN,
            reason_zh="超时",
        ),
    )
    result = explorer.run()
    assert result.result != "TARGET_FOUND"
    assert "target_found" not in [item["event"] for item in explorer.events]


# --------------------------------------------------------------------------- #
# 7) 判据助手本身
# --------------------------------------------------------------------------- #
def test_the_terminal_negative_helper_agrees_with_the_status_constants():
    assert verification_is_terminal_negative(None) is False
    assert verification_is_terminal_negative(
        VerificationOutcome(confirmed=True, attempts=1)
    ) is False
    # `status=""` 是**旧语义**（内联复核，没确认即否定）—— 保留，逐字不变。
    assert verification_is_terminal_negative(
        VerificationOutcome(confirmed=False, attempts=1)
    ) is True
    assert verification_is_terminal_negative(
        VerificationOutcome(confirmed=False, attempts=1, status=VERIFY_STATUS_REJECTED)
    ) is True
    assert verification_is_terminal_negative(
        VerificationOutcome(confirmed=False, attempts=1, status=VERIFY_STATUS_UNKNOWN)
    ) is False


def test_the_new_state_is_not_positive_evidence_and_survives_a_round_trip():
    """`UNKNOWN` 必须能原样往返（否则观测里存的是 POSSIBLE、读回来是 ABSENT）。"""
    assert normalize_target_state(TARGET_UNKNOWN) == TARGET_UNKNOWN
    assert normalize_target_state("unknown") == TARGET_UNKNOWN
    assert normalize_target_state("TIMEOUT") == TARGET_ABSENT, (
        "只有 UNKNOWN 这一个词是状态；别的怪词仍按旧规矩落到默认值"
    )
    assert target_state_is_present(
        {"target_state": TARGET_UNKNOWN, "objects": [{"score": 0.99}]}
    ) is False
    assert target_state_is_present(
        {"target_state": TARGET_POSSIBLE, "objects": [{"score": 0.99}]}
    ) is False
    assert target_state_is_present(
        {"target_state": TARGET_PRESENT, "objects": [{"score": 0.99}]}
    ) is True


# --------------------------------------------------------------------------- #
# 8) 生产入口的接线（AST 守卫）
# --------------------------------------------------------------------------- #
def test_the_web_entry_awaits_by_submission_and_verifies_identity():
    """入口必须用 `await_submission(submit() 的返回值)` + `result_matches_submission`。

    这条防的是"模块改对了、入口没跟着改"：身份绑定做在管理器里，入口却还按
    `candidate_id` 取结论，缺陷照旧 —— 审查 P1-a 点的就是入口那一行。
    """
    source = (
        pathlib.Path(__file__).resolve().parents[1]
        / "scripts" / "go2w" / "run_semantic_exploration.py"
    )
    tree = ast.parse(source.read_text(encoding="utf-8"))
    called_methods = {
        node.func.attr for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "await_submission" in called_methods
    assert "await_result" not in called_methods, (
        "入口还在按候选取结论 —— 跨帧会把上一帧的 CONFIRMED 交还给新帧"
    )
    read_names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    assert "result_matches_submission" in read_names, "入口缺少拿回结论时的当面核验"
    bare_submits = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Expr)
        and isinstance(node.value, ast.Call)
        and isinstance(node.value.func, ast.Attribute)
        and node.value.func.attr == "submit"
    ]
    assert not bare_submits, (
        "submit() 的返回值被丢掉了 —— 没有序号就没法按**本次提交**取结论"
    )


def _wait_until(predicate, timeout: float = 5.0) -> None:
    """等一个线程间条件成立；超时直接失败（不要用 sleep 猜时序）。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    pytest.fail("等待超时：worker 线程没有在预期内推进")
