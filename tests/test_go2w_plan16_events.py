"""详细修改计划书 §16 里最后 4 个没有发射点的事件。

审计报告 `reports/go2w_plan_completion_audit_20260915.md:60` 的原话是
「**Quick VLM 失败重试**与**扫描帧被丢弃**这两个最常发生的路径，日志里查不到」。
本文件守的就是这两条路径：

- ``quick_vlm_failed`` / ``quick_vlm_retry`` —— worker（``run_semantic_exploration.py``）
- ``slam_scan_dropped`` / ``slam_scan_accumulated`` —— WebUI 桥（``plain_slam_web_bridge.py``）

**为什么是源码级断言，不是行为级。** 这两个文件都是 ROS 节点：worker 的
``rclpy`` 是函数内延迟 import，但 ``observe()`` 要真节点、真 RGB-D 源、真检测器；
桥的类直接继承 ``rclpy.node.Node``。把发射逻辑拆出来做可注入测试，就是为测试而
重构 —— §10.1 明令禁止。本仓库对 ROS 节点的既有分工是「规则本体住 ``app/``（纯
逻辑，可行为测试），节点只做接线（源码级契约）」：见 ``tests/test_slam_bridge.py``
测的是 ``app/spatial/slam_web_map_state.py``、而 ``slam_web_map_state.py`` 的
docstring 自己写了这条分层理由。这里沿用同一条分工。

**这些断言的边界要说清楚**：源码级断言能证明「发射点写在正确的分支上、顺序对、
限流没有把新事件一起吃回去」，**不能**证明真机上那个分支会被走到。后者靠实机验证，
不靠这里。所以下面每条都钉在「改坏了就会红」的具体位置上，而不是只 grep 一个
字符串存在 —— 只断言字符串存在的测试，在有人把发射点挪到错误分支时仍然会绿。
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "scripts" / "go2w" / "plain_slam_web_bridge.py"
WORKER = ROOT / "scripts" / "go2w" / "run_semantic_exploration.py"


# --- slam_scan_dropped / slam_scan_accumulated（桥） ----------------------

def test_preview_frame_mismatch_is_no_longer_silent() -> None:
    """本次修掉的真问题：这条路径此前**一处日志都没有**。

    ``state.reject("preview_frame_mismatch")`` 只把计数加进 ``rejected_counts``，
    而计数只出现在 snapshot JSON 里 —— 翻 ``slam_web_bridge.log`` 查不到任何东西。
    这是「扫描帧被丢弃…日志里查不到」的最直接一处。
    """
    src = BRIDGE.read_text(encoding="utf-8")
    marker = 'self._state.reject("preview_frame_mismatch")'
    assert src.count(marker) == 1, "预览帧不匹配的分支不止一处，断言前提已变"
    tail = src[src.index(marker):src.index(marker) + 1400]
    # 断言**日志文本本身**，不是裸的 "slam_scan_dropped" —— 那一段的解释性注释里
    # 就写着这个词，只查词的话把日志删掉、注释留着，用例照样绿（实测过）。
    assert "slam_scan_dropped preview: preview_frame_mismatch" in tail, (
        "preview_frame_mismatch 又变回静默丢弃了 —— 这正是审计报告点名的那个洞"
    )
    assert "get_logger().warn" in tail
    # 限流不是可选的：帧号配错是**持续**状态，预览路径按 --preview-rate 跑，
    # 不限流会以 10 Hz 把这个日志刷到没法看。断言条件式本身，不断言变量名 ——
    # 变量还在、条件被改成恒真同样是没限流。
    assert "if now - self._last_preview_reject_log >= 5.0:" in tail, (
        "预览丢帧没有限流 —— 帧号配错时会按预览频率刷屏"
    )
    # 限流会把中间那些帧吃掉，所以累计次数必须跟在同一行里，否则信息净损失。
    assert "rejected_counts.get('preview_frame_mismatch'" in tail


def test_throttled_preview_refresh_is_not_reported_as_a_drop() -> None:
    """**反向断言**：按 ``--preview-rate`` 限流是设计行为，不是丢帧。

    把它也记成 slam_scan_dropped，就等于把一个正常的节流报成故障 —— 那种日志
    比没有日志更坏。断言那一段里没有任何 ``get_logger()`` 调用（而不是断言没有
    ``slam_scan_dropped`` 字样 —— 那一段里有一句解释为什么不记的注释）。
    """
    src = BRIDGE.read_text(encoding="utf-8")
    start = src.index("if now - self._last_preview_monotonic < self._preview_interval:")
    end = src.index("self._last_preview_monotonic = now")
    assert start < end
    assert "get_logger()" not in src[start:end], (
        "按 preview_rate 的正常节流被记成日志了 —— 它会把真正的丢帧淹掉"
    )


def test_map_accept_and_reject_carry_the_plan16_names() -> None:
    src = BRIDGE.read_text(encoding="utf-8")
    accumulated = 'f"slam_scan_accumulated map r{state.map_revision} "'
    dropped = 'f"slam_scan_dropped map: {state.last_rejected_reason} "'
    assert accumulated in src, "永久地图被接受时没有发 §16 的 slam_scan_accumulated"
    assert dropped in src, "永久地图被拒收时没有发 §16 的 slam_scan_dropped"
    # §16 要求「关键日志必须包含 frame/sector/latency」—— 这条路径没有 latency，
    # 但 frame 与 points 必须在，否则查不出是哪一帧被丢。
    tail = src[src.index(dropped):src.index(dropped) + 500]
    assert "frame={frame_id}" in tail and "points={len(points)}" in tail
    # 拒收告警限流 5 s，所以累计次数必须同行给出（审计要的是"查得到"，不是"有这一行"）。
    assert "rejected_counts.get(state.last_rejected_reason" in tail
    # 接受分支必须早于拒收分支返回 —— 否则接受路径会掉进拒收的告警里。
    assert src.index(accumulated) < src.index(dropped)


# --- quick_vlm_failed / quick_vlm_retry（worker） -------------------------

def test_quick_vlm_failed_is_emitted_on_both_failure_branches() -> None:
    """两个 except 都要记。只记一个，另一类失败在日志里仍然是空的。

    ``except PerceptionFailure: raise`` 那条尤其容易漏 —— 它长得像个空分支，
    但它正是"检测器自己抛的结构化错误"，是 quick VLM 最常见的失败形态。
    """
    src = WORKER.read_text(encoding="utf-8")
    # 文件里不止一个 observe()（另有第 800 行那个入口函数），所以 end 必须从
    # 本处往后找，否则切片是空的 —— 空切片会让下面每条断言都变成假绿。
    start = src.index("def _note_quick_vlm_failure")
    body = src[start:src.index("def observe() -> LiveObservation", start)]
    assert body, "切片为空，下面的断言全都会静默通过"
    assert '"event": "quick_vlm_failed"' in body
    assert 'state["quick_vlm_last_failure"] = {' in body
    # 带 latency：§16 原话「关键日志必须包含 frame/sector/latency」
    assert '"latency_ms"' in body and '"code"' in body

    call = "_note_quick_vlm_failure(frame_id,"
    assert src.count(call) == 2, (
        f"quick VLM 的失败记录点有 {src.count(call)} 处，应为 2 —— "
        "少一处就有一种失败在日志里查不到"
    )
    # 两个 except 都必须以 raise 结束：吞掉异常会让 explorer 的重试循环收不到。
    assert "raise failure from exc" in src, "包装分支丢了异常链"
    assert "            raise\n" in src, "结构化错误分支不再原样重抛"


def test_quick_vlm_retry_precedes_the_attempt_it_describes() -> None:
    """retry 是「在补上一次失败」，所以三条都得在：
    条件取自前一次的失败标记、写在这次尝试之前、带上是谁失败了。

    **只断言两次发射的前后顺序是不够的**：把条件整行换成 ``= None`` 而发射点
    不动位置，顺序断言照样绿，retry 却永远不会发。所以这里连条件的数据来源一起断言。
    """
    src = WORKER.read_text(encoding="utf-8")
    feed = 'previous_quick_failure = state.get("quick_vlm_last_failure")'
    assert feed in src, "retry 的条件不再来自上一次失败标记 —— 要么恒发要么永不发"
    gate = src.index("if previous_quick_failure:")
    retry = src.index('"event": "quick_vlm_retry"')
    started = src.index('"event": "quick_vlm_started"')
    assert src.index(feed) < gate < retry < started, (
        "顺序应为 取标记 -> 判标记 -> 发 retry -> 发 started"
    )
    between = src[retry:started]
    assert '"previous_frame_id"' in between, "retry 没带失败那一次的 frame_id，查不出补的是哪一帧"
    assert '"previous_code"' in between


def test_quick_vlm_failure_marker_is_cleared_only_on_success() -> None:
    """标记不清 ⇒ 之后每次 quick VLM 都平白发一条 retry；清早了 ⇒ 失败被自己抹掉。"""
    src = WORKER.read_text(encoding="utf-8")
    clear = src.index('state["quick_vlm_last_failure"] = None')
    last_note = src.rindex("_note_quick_vlm_failure(frame_id,")
    completed = src.index('"event": "quick_vlm_completed"')
    assert clear > last_note, "清理早于记录失败 —— 失败会被自己抹掉，retry 永远不触发"
    assert clear < completed, "清理写在 quick_vlm_completed 之后，语义上不在成功路径上"
    # 「只在成功时清」由两个 except 分支都以 raise 结束保证（上面那条已断言）。
    # 若有人把某个 except 改成 pass/continue，控制流会落到 clear 上，静默清标记。
    for branch in ("            raise\n", "            raise failure from exc\n"):
        assert branch in src, f"失败分支不再以 raise 结束：{branch!r}"
