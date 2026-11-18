"""有界异步目标复核（计划书 §5.1.5）。

缺口：`run_semantic_exploration.verifier` **同步**调 `node._verify_target`，
把搜索线程按在那儿等 VLM —— 默认超时 45 s（`GO2W_VERIFY_CLIENT_TIMEOUT_S`
的旧值），而真实链路上的云端 quick 请求常常漂到 60 s（见
`go2w-cloud-quick-timeout-straddle`）。等不到也不会取消：VLM daemon 是
**单槽串行**的（`go2w-vlm-daemon-single-slot-queueing`），被放弃的慢请求
继续占着 realtime 通道，后到的复核排在它后面，一起被 45 s 闸门判掉。

本模块把复核改成"提交 + 轮询"，并守住三条不变量：

1. **不做负例**。超时、异常、在途 —— 一律 ``UNKNOWN``，**不是** `ABSENT`。
   "没验出来"和"验出来是假的"是两件事，混成一个就会把还没看完的候选
   当成"目标不存在"，正是 §3.2 不许新增的那类错误。
2. **陈旧结果不许覆盖新结果**（序号丢弃）。带外到达的旧响应只能记事件，
   不能改状态 —— 否则乱序一到就凭空造出"找到了"。
3. **有界**：同时最多 ``max_inflight`` 个在飞，最多留 ``max_pending`` 个待办
   （latest-wins 合并）。不设上界就是拿"提交多少跑多少"换一个必然的排队雪崩。

绑定身份是 ``(task_generation, frame_id, candidate_id)``：换目标/重开会话会把
``task_generation`` 推进一格，上一代的结果一律丢弃 —— 报告里"复核确认"的
那一票必须属于当前这一代的目标。

**取结论只能按提交凭据**（``submit()`` 返回的序号，交给
``await_submission()``）。``get_latest()`` 是"这个候选最近一条结论"，它会
**跨帧存活** —— 拿它去判"这一帧找到了没有"，就会把上一帧的 CONFIRMED 抄到
这一帧头上（见 ``await_submission`` 的注释）。

daemon 侧**无法取消**（这是既有事实，本模块不假装能取消）：worker 线程该挂
还是挂着，但**搜索线程不再等它**。客户端超时一到就出 ``UNKNOWN``，那个晚到
的结果会被序号丢弃。注意超时**只解绑客户端，不释放物理名额**：``max_inflight``
数的是"真的还在 daemon 上跑着"的请求，客户端判超时的那一刻它还占着那个槽
（见 ``_client_timed_out``）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time
from typing import Any, Callable

# 复核结论。`CONFIRMED` / `REJECTED` 是**已终结**的判定；`UNKNOWN` 不是判定，
# 它是"没有判定"。下游只许拿前两个去改目标态。
VERIFY_STATUS_CONFIRMED = "CONFIRMED"
VERIFY_STATUS_REJECTED = "REJECTED"
VERIFY_STATUS_UNKNOWN = "UNKNOWN"

# 为什么是 UNKNOWN —— 报表要能分开数，别让三种成因挤在一个字符串里。
VERIFY_UNKNOWN_TIMEOUT = "client_timeout"
VERIFY_UNKNOWN_ERROR = "verify_error"
VERIFY_UNKNOWN_PENDING = "still_in_flight"

TERMINAL_STATUSES = frozenset({VERIFY_STATUS_CONFIRMED, VERIFY_STATUS_REJECTED})


@dataclass
class AsyncVerifyRequest:
    sequence: int
    task_generation: int
    frame_id: str
    candidate_id: str
    image_path: str
    bbox: tuple[float, ...]
    env: dict[str, Any] = field(default_factory=dict)
    submitted_at: float = 0.0
    # 已经出过终态（含 UNKNOWN）就不再出第二次。
    resolved: bool = False


@dataclass
class VerificationResult:
    """一次复核的终态。``status`` 三选一，``confirmed`` 只在 CONFIRMED 时为真。"""

    sequence: int
    task_generation: int
    frame_id: str
    candidate_id: str
    status: str
    reason_zh: str = ""
    latency_ms: float = 0.0
    unknown_reason: str = ""
    details: dict[str, Any] = field(default_factory=dict)
    # 客户端超时那一刻出结论时为真：它回答的是"这条结论是等出来的还是
    # 到点判出来的"，报表要能把两者分开。
    client_timed_out: bool = False

    @property
    def terminal(self) -> bool:
        return self.status in TERMINAL_STATUSES

    @property
    def confirmed(self) -> bool:
        return self.status == VERIFY_STATUS_CONFIRMED


def _candidate_key(task_generation: int, candidate_id: str) -> tuple[int, str]:
    return (int(task_generation), str(candidate_id))


def result_matches_submission(
    result: VerificationResult | None,
    *,
    sequence: int,
    frame_id: str,
    candidate_id: str,
    task_generation: int,
) -> bool:
    """这条结论是不是**这一次提交**的（生产入口的当面核验）。

    `await_submission()` 已经按序号取过一次了，这里是第二道：序号、源帧、候选、
    代次四项全对上才算数。留着它的理由是"哪天上游把序号接错"（把上一次的提交
    凭据传进来），第一道就会乖乖交出那一次的结论 —— 而它的 frame_id 是别的帧。
    对不上时调用方必须按"没判定"（UNKNOWN）处理：**宁可不出结论，也不能拿错的
    结论去宣布找到目标**（审查 P1-a）。
    """
    if result is None:
        return False
    return (
        int(result.sequence) == int(sequence)
        and str(result.frame_id) == str(frame_id)
        and str(result.candidate_id) == str(candidate_id)
        and int(result.task_generation) == int(task_generation)
    )


class AsyncVerifierManager:
    """一次最多一个目标复核在飞；搜索线程只提交与轮询，不阻塞。"""

    def __init__(
        self,
        verify: Callable[..., dict[str, Any]],
        *,
        enabled: bool = False,
        max_inflight: int = 1,
        max_pending: int = 1,
        client_timeout_s: float = 12.0,
        now: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        event_sink: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._verify = verify
        self.enabled = bool(enabled)
        self.max_inflight = max(1, int(max_inflight))
        self.max_pending = max(0, int(max_pending))
        self.client_timeout_s = float(client_timeout_s)
        self._now = now
        # 注入 `now`/`sleep` 是为了让等待循环在测试里**确定性**地瞬间跑完：
        # 真实的 12 s 截止线在单测里既慢又不可复现。
        self._sleep = sleep
        self._event_sink = event_sink
        self._lock = threading.Lock()
        self._sequence = 0
        self._task_generation = 0
        self._inflight: dict[int, AsyncVerifyRequest] = {}
        self._pending: list[AsyncVerifyRequest] = []
        self._completed: list[VerificationResult] = []
        # 每个 (generation, candidate) 已经落地的最新序号 —— 序号丢弃的判据。
        self._applied_sequence: dict[tuple[int, str], int] = {}
        # (generation, candidate) → 最近一条结论。**跨帧存活**，只作报表用；
        # 判"这一帧找到了没有"必须走 `_by_sequence`（按提交凭据取）。
        self._latest: dict[tuple[int, str], VerificationResult] = {}
        # 序号 → 那一次提交的结论。身份绑定就发生在这里：查的是**自己那一次**，
        # 不是"这个候选最近怎么样"。
        self._by_sequence: dict[int, VerificationResult] = {}
        # 客户端已经按超时判过、但物理上**还在 daemon 上跑**的序号。
        # 与 `_inflight` 分开记：前者是"我们不再等它了"，后者是"它还占着槽"。
        self._client_timed_out: set[int] = set()
        self._discarded_stale = 0
        self._discarded_obsolete = 0
        self._threads: list[threading.Thread] = []

    # -- 生命周期 -------------------------------------------------------------

    def set_task_generation(self, generation: int) -> None:
        """换目标/换会话：推进一代，并丢掉上一代的待办。

        在飞的那一个**不取消**（daemon 取消不了），但它的结果会因为代次不匹配
        被丢弃 —— 报出来就成"上一代的目标在这一代被确认"了。
        """
        with self._lock:
            generation = int(generation)
            if generation == self._task_generation:
                return
            dropped = len(self._pending)
            self._pending.clear()
            self._task_generation = generation
        # 代次一变就留痕（哪怕没待办可丢）：事后要能回答"这条结论属于哪一代"，
        # 只在丢东西时才记，会让"换了代"这件事在事件流里根本不存在。
        self._emit({
            "event": "verify_generation_advanced",
            "task_generation": generation,
            "dropped_pending": dropped,
        })

    @property
    def task_generation(self) -> int:
        with self._lock:
            return self._task_generation

    # -- 提交与轮询 -----------------------------------------------------------

    def submit(
        self,
        *,
        frame_id: str,
        candidate_id: str,
        image_path: str,
        bbox: Any,
        env: dict[str, Any] | None = None,
        task_generation: int | None = None,
    ) -> int:
        """提交一次复核，返回**本次提交的序号**。

        序号 >0 = 真的起了线程，把它交给 `await_submission()` 才拿得到**这一次**
        的结论。返回 0 = 没起线程（合并进待办 / 未启用）—— 这一条没有可等的结论，
        调用方必须按"没判定"处理，不许拿这个候选的历史结论顶上。

        （旧签名返回 bool，`if submit(...)` 那种用法仍然成立：0 是假值。）
        """
        if not self.enabled:
            return 0
        with self._lock:
            generation = (
                self._task_generation if task_generation is None
                else int(task_generation)
            )
            request = AsyncVerifyRequest(
                sequence=0,
                task_generation=generation,
                frame_id=str(frame_id),
                candidate_id=str(candidate_id),
                image_path=str(image_path),
                bbox=tuple(float(v) for v in bbox),
                env=dict(env or {}),
                submitted_at=float(self._now()),
            )
            if len(self._inflight) >= self.max_inflight:
                self._pending.append(request)
                # latest-wins：只留最新的 max_pending 条，旧的明确记一笔
                # "被合并掉了"，不是静默丢。
                while len(self._pending) > self.max_pending:
                    obsolete = self._pending.pop(0)
                    self._discarded_obsolete += 1
                    self._emit({
                        "event": "verify_request_discarded",
                        "reason": "coalesced_pending",
                        "frame_id": obsolete.frame_id,
                        "candidate_id": obsolete.candidate_id,
                    })
                if self.max_pending == 0:
                    self._discarded_obsolete += 1
                    self._pending.clear()
                return 0
            self._sequence += 1
            request.sequence = self._sequence
            self._inflight[request.sequence] = request
            sequence = request.sequence
        self._emit({
            "event": "verify_request_started",
            "frame_id": str(frame_id),
            "candidate_id": str(candidate_id),
            "sequence": sequence,
            "task_generation": generation,
        })
        thread = threading.Thread(
            target=self._run_verify,
            args=(request,),
            name=f"async-verify-{sequence}",
            daemon=True,
        )
        with self._lock:
            self._threads.append(thread)
        thread.start()
        return sequence

    def poll_completed(self) -> list[VerificationResult]:
        """取走已终结的结果；顺带把**客户端超时**的那些判成 UNKNOWN。

        超时判在这里而不是 worker 线程里：worker 可能永远不返回（daemon 取消
        不了），把超时挂在它身上等于没有超时。

        **超时不释放物理名额**（计划书 §5.1.5）：`request` 仍留在 `_inflight`
        里，只是被打上 `resolved` + 记进 `_client_timed_out`。daemon 取消不了，
        那个请求客观上还占着并发槽；此时把槽让出来，下一次 `submit()` 就会再起
        一个线程，实际在跑的是两个 —— `max_inflight` 就成了摆设，而 daemon 是
        单槽串行的，多出来的那个只会加剧排队。真正的释放发生在 worker 返回时的
        `_finish()`。
        """
        with self._lock:
            now = float(self._now())
            for request in list(self._inflight.values()):
                if request.resolved:
                    continue
                if now - request.submitted_at < self.client_timeout_s:
                    continue
                request.resolved = True
                self._client_timed_out.add(request.sequence)
                timed_out = VerificationResult(
                    sequence=request.sequence,
                    task_generation=request.task_generation,
                    frame_id=request.frame_id,
                    candidate_id=request.candidate_id,
                    status=VERIFY_STATUS_UNKNOWN,
                    reason_zh="verify client timeout（未出判定，不作为目标不存在的证据）",
                    latency_ms=round((now - request.submitted_at) * 1000.0, 3),
                    unknown_reason=VERIFY_UNKNOWN_TIMEOUT,
                    client_timed_out=True,
                )
                self._by_sequence[request.sequence] = timed_out
                self._latest[
                    _candidate_key(request.task_generation, request.candidate_id)
                ] = timed_out
                self._completed.append(timed_out)
            completed = self._completed
            self._completed = []
        for result in completed:
            self._emit({
                "event": (
                    "verify_client_timeout"
                    if result.client_timed_out else "verify_result_completed"
                ),
                "frame_id": result.frame_id,
                "candidate_id": result.candidate_id,
                "sequence": result.sequence,
                "status": result.status,
                "latency_ms": result.latency_ms,
            })
        return completed

    def await_submission(
        self,
        sequence: int,
        *,
        timeout_s: float | None = None,
        slice_s: float = 0.05,
    ) -> VerificationResult | None:
        """等**那一次提交**（`submit()` 返回的序号）出结论；到点返回 ``None``。

        身份绑定是这里的全部要点：查的是 ``_by_sequence[sequence]``，即
        "你提交的那一次跑完了没有"，**不是** "这个候选最近怎么样"。两者在跨帧
        时不是一回事 —— 框和标签没变时 `entity_id_for()` 两帧算出来是同一个
        ID，按候选查就会把上一帧的 CONFIRMED 立刻交还给这一帧，于是新帧的复核
        还没跑完（甚至还没跑），搜索线程已经拿着旧结论去宣布"找到了"。

        `sequence <= 0` = 那一次没有起线程（被合并进待办/未启用），没有可等的
        结论，直接返回 ``None``（下游按 UNKNOWN 处理），**不去等**：等一个永远
        不会出现的序号只是白等到截止线。

        它与"老的同步复核"的区别**不是**"完全不占时间"，而是三条：

        1. **上限由客户端定**。老路的 45 s 是 daemon 侧的超时，超了抛
           ``PerceptionFailure``，拿不回来也没有上界；这里的截止线是我们
           自己给的（``client_timeout_s``），到点必返回。
        2. **一片一片地轮询**，答案一到就返回，不会为了等满而等满。
        3. **到点出的是 UNKNOWN**，不是否定；晚到的结果被序号丢弃。

        每片都走 ``poll_completed()``，所以**截止线的判定只有一处**
        （``poll_completed`` 里那段），这里不另写一套超时逻辑 —— 两处判超时
        迟早会不一致。
        """
        sequence = int(sequence)
        if sequence <= 0:
            return None
        limit = self.client_timeout_s if timeout_s is None else float(timeout_s)
        deadline = self._now() + max(0.0, limit)
        while True:
            self.poll_completed()
            found = self.get_result_for(sequence)
            if found is not None:
                return found
            remaining = deadline - self._now()
            if remaining <= 0:
                return None
            self._sleep(min(slice_s, remaining))

    def get_result_for(self, sequence: int) -> VerificationResult | None:
        """按序号取结论 —— 身份绑定的取值口。"""
        with self._lock:
            return self._by_sequence.get(int(sequence))

    def get_latest(self, candidate_id: str, task_generation: int | None = None
                   ) -> VerificationResult | None:
        """这个候选**最近一条**结论。**跨帧存活**，所以它不是"本帧的结论"。

        只能用于报表和排查（"这个候选历史上被怎么判过"）。要判"这一帧复核
        出了什么"，必须用 `await_submission()` / `get_result_for()` —— 拿这条
        去等下一条提交，正是审查 P1-a 那个"新帧消费上一帧结论"的缺陷。
        """
        with self._lock:
            generation = (
                self._task_generation if task_generation is None
                else int(task_generation)
            )
            return self._latest.get(_candidate_key(generation, candidate_id))

    def has_inflight(self) -> bool:
        with self._lock:
            return bool(self._inflight)

    def queue_depth(self) -> dict[str, int]:
        """队列可见性（计划书 §5.1.5 第 4 条）。

        只统计**客户端侧**的计数，不动 wire protocol —— 改协议要同时改 daemon，
        那是另一件事，也会把"这次改动只碰客户端"这条边界弄糊。

        `inflight` 数的是**物理**在途（含客户端已判超时、daemon 上还在跑的那些），
        它是 `max_inflight` 真正在限的那个量；`client_timed_out` 是其中"我们不再
        等它"的子集。两者相等 = 槽被超时的请求占满（此时新提交只会进 pending）。
        """
        with self._lock:
            return {
                "inflight": len(self._inflight),
                "client_timed_out": len(self._client_timed_out),
                "pending": len(self._pending),
                "discarded_stale": self._discarded_stale,
                "discarded_obsolete": self._discarded_obsolete,
            }

    def close(self) -> None:
        with self._lock:
            self.enabled = False
            self._pending.clear()

    # -- worker 侧 -----------------------------------------------------------

    def _run_verify(self, request: AsyncVerifyRequest) -> None:
        started = self._now()
        try:
            payload = self._verify(
                request.image_path,
                list(request.bbox),
                dict(request.env),
                frame_id=request.frame_id,
            )
        except Exception as exc:  # noqa: BLE001 - 后台不许把搜索线程带崩
            self._finish(request, None, started, error=exc)
        else:
            self._finish(request, payload, started, error=None)

    def _finish(
        self,
        request: AsyncVerifyRequest,
        payload: dict[str, Any] | None,
        started: float,
        *,
        error: BaseException | None,
    ) -> None:
        now = float(self._now())
        latency_ms = round(max(0.0, now - started) * 1000.0, 3)
        if error is not None:
            result = VerificationResult(
                sequence=request.sequence,
                task_generation=request.task_generation,
                frame_id=request.frame_id,
                candidate_id=request.candidate_id,
                status=VERIFY_STATUS_UNKNOWN,
                reason_zh=(
                    f"verify failed: {type(error).__name__}"
                    "（未出判定，不作为目标不存在的证据）"
                ),
                latency_ms=latency_ms,
                unknown_reason=VERIFY_UNKNOWN_ERROR,
            )
        else:
            result = self._result_from_payload(request, payload or {}, latency_ms)

        with self._lock:
            # 物理名额在这里才真的释放 —— 客户端超时**不**动 `_inflight`
            # （见 `poll_completed`），所以那段时间里它仍计在 `max_inflight` 上。
            self._inflight.pop(request.sequence, None)
            self._client_timed_out.discard(request.sequence)
            if request.resolved:
                # 客户端已经按超时判过了：这一条是**晚到的**，只能记一笔。
                self._discarded_stale += 1
                discard_reason = "stale_after_client_timeout"
            elif request.task_generation != self._task_generation:
                self._discarded_stale += 1
                discard_reason = "stale_task_generation"
            elif result.sequence <= self._applied_sequence.get(
                _candidate_key(request.task_generation, request.candidate_id), 0
            ):
                self._discarded_stale += 1
                discard_reason = "stale_result"
            else:
                discard_reason = ""
                key = _candidate_key(request.task_generation, request.candidate_id)
                self._applied_sequence[key] = result.sequence
                self._latest[key] = result
                # 同一候选只留**最新**那一次的结论：更早的序号已经被序号丢弃
                # 逻辑判过"过期"，留着只会让 `_by_sequence` 随搜索时长无界增长。
                for stale_seq in [
                    seq for seq in self._by_sequence
                    if seq < result.sequence
                    and self._by_sequence[seq].candidate_id == request.candidate_id
                    and self._by_sequence[seq].task_generation == request.task_generation
                ]:
                    del self._by_sequence[stale_seq]
                self._by_sequence[result.sequence] = result
                self._completed.append(result)
            pending = self._next_pending_locked()

        if discard_reason:
            self._emit({
                "event": "verify_result_discarded",
                "frame_id": request.frame_id,
                "candidate_id": request.candidate_id,
                "sequence": request.sequence,
                "reason": discard_reason,
                "status": result.status,
            })
        if pending is not None:
            self.submit(
                frame_id=pending.frame_id,
                candidate_id=pending.candidate_id,
                image_path=pending.image_path,
                bbox=pending.bbox,
                env=pending.env,
                task_generation=pending.task_generation,
            )

    def _next_pending_locked(self) -> AsyncVerifyRequest | None:
        if self._pending and len(self._inflight) < self.max_inflight:
            return self._pending.pop(0)
        return None

    def _result_from_payload(
        self, request: AsyncVerifyRequest, payload: dict[str, Any], latency_ms: float
    ) -> VerificationResult:
        """把 detector 的返回折成三态。**默认 UNKNOWN，不是 REJECTED。**

        返回体缺字段时按"没判定"处理：把"没读懂返回"当成"目标不存在"，
        就是拿一个解析失败去否掉一个候选。
        """
        if not isinstance(payload, dict) or "is_target" not in payload:
            return VerificationResult(
                sequence=request.sequence,
                task_generation=request.task_generation,
                frame_id=request.frame_id,
                candidate_id=request.candidate_id,
                status=VERIFY_STATUS_UNKNOWN,
                reason_zh="verify payload has no usable verdict（未出判定）",
                latency_ms=latency_ms,
                unknown_reason=VERIFY_UNKNOWN_ERROR,
                details=dict(payload or {}),
            )
        confirmed = bool(payload.get("is_target", False))
        return VerificationResult(
            sequence=request.sequence,
            task_generation=request.task_generation,
            frame_id=request.frame_id,
            candidate_id=request.candidate_id,
            status=(
                VERIFY_STATUS_CONFIRMED if confirmed else VERIFY_STATUS_REJECTED
            ),
            reason_zh=str(payload.get("reason_zh", "")),
            latency_ms=latency_ms,
            details=dict(payload),
        )

    # -- 事件 ----------------------------------------------------------------

    def _emit(self, event: dict[str, Any]) -> None:
        if self._event_sink is None:
            return
        try:
            self._event_sink(event)
        except Exception:  # noqa: BLE001 - 记日志不许影响搜索
            pass
