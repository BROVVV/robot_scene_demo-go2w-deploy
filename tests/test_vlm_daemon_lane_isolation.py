"""守护进程的"通道隔离"：verify 不许排在慢 quick 后面。

钉的是一个实测过的故障：2026-10-12 一次 run 里，一条已被客户端放弃的慢
quick（它仍在烧 openai 的重试）占着唯一的 realtime 通道，把后到的 verify
挡了 **63.2 s**，撞穿搜索侧 45 s 闸门，那次复核等于白做。

根因是两处，缺一不可，**而且两处长得很像"改一处就够了"**：

1. `Handler.handle` 里 quick 与 verify 共用 `realtime_executor`（单 worker）；
2. server 是非线程版 `UnixStreamServer`，`handle()` 是内联跑在 `serve_forever`
   那个线程里的 —— 光拆 executor，排队只是从 executor 挪到 `accept`：
   第二条连接连都连不进来。

所以这里的测试走**真 socket**、真的并发两条连接，而不是直接调 executor。
只测 executor 的话第 2 处永远测不到，而它同样会复现 63.2 s。
"""

from __future__ import annotations

import json
import os
import socket
import threading
import time

import pytest

from app.detectors import siliconflow_vision_daemon as daemon_mod

SLOW_QUICK_S = 1.2


def _roundtrip(socket_path, mode: str, timeout: float = 10.0) -> tuple[float, dict]:
    """走完整协议发一次请求，返回 (墙钟秒数, 响应)。"""
    path = os.fspath(socket_path)  # daemon.socket_path 是 Path，socket 只收 str/bytes
    payload = {"request_id": f"req-{mode}", "mode": mode,
               "image_path": "/tmp/frame.jpg", "target": "绿色垃圾桶"}
    start = time.perf_counter()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(path)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                break
            buf += chunk
    return time.perf_counter() - start, json.loads(buf.decode("utf-8"))


@pytest.fixture
def daemon(tmp_path, monkeypatch):
    """起一个真守护进程，但把真正干活那步换成"按 mode 睡指定秒数"。

    只替换 `daemon_handle_sync`：协议解析、executor 路由、socket 服务
    全是生产代码 —— 被测的就是这三样，替换掉就没意义了。
    """
    delays: dict[str, float] = {}

    def fake_handle(_daemon, request: dict) -> dict:
        mode = str(request.get("mode"))
        time.sleep(delays.get(mode, 0.0))
        return {
            "request_id": str(request.get("request_id", "")),
            "ok": True, "mode": mode, "payload": {}, "error": None,
            "latency_ms": 0.0, "frame_id": "",
        }

    monkeypatch.setattr(daemon_mod, "daemon_handle_sync", fake_handle)
    instance = daemon_mod.VLMDaemon(str(tmp_path / "vlm.sock"))
    instance.start()
    try:
        yield instance, delays
    finally:
        instance.stop()


def test_verify_is_not_queued_behind_a_slow_quick(daemon) -> None:
    """慢 quick 在飞的时候，verify 仍要立刻拿到响应。

    这条同时覆盖上面说的两处根因：verify 既没排进 realtime 通道，
    也没被非线程版 server 挡在 `accept` 外面。
    """
    instance, delays = daemon
    delays["quick"] = SLOW_QUICK_S
    quick_result: dict[str, tuple[float, dict]] = {}

    def _fire_slow_quick() -> None:
        quick_result["quick"] = _roundtrip(instance.socket_path, "quick")

    slow = threading.Thread(target=_fire_slow_quick)
    slow.start()
    try:
        # 等 quick 确实进了池再发 verify，否则测的是"两件事同时到"
        time.sleep(SLOW_QUICK_S / 4)
        verify_elapsed, verify_response = _roundtrip(instance.socket_path, "verify")
    finally:
        slow.join()

    assert verify_response["ok"] is True
    assert quick_result["quick"][0] >= SLOW_QUICK_S * 0.9, "quick 本该睡满，用例前提不成立"
    assert verify_elapsed < SLOW_QUICK_S / 2, (
        f"verify 等了 {verify_elapsed:.2f}s —— 它排到慢 quick 后面去了"
        f"（quick 总共才睡 {SLOW_QUICK_S}s）"
    )


def test_background_label_does_not_block_realtime_verify(daemon) -> None:
    """Unknown-object naming uses the background lane, never the verify lane."""
    instance, delays = daemon
    delays["label"] = SLOW_QUICK_S
    label_result: dict[str, tuple[float, dict]] = {}

    def _fire_slow_label() -> None:
        label_result["label"] = _roundtrip(instance.socket_path, "label")

    slow = threading.Thread(target=_fire_slow_label)
    slow.start()
    try:
        time.sleep(SLOW_QUICK_S / 4)
        verify_elapsed, verify_response = _roundtrip(instance.socket_path, "verify")
    finally:
        slow.join()

    assert verify_response["ok"] is True
    assert label_result["label"][0] >= SLOW_QUICK_S * 0.9
    assert verify_elapsed < SLOW_QUICK_S / 2


def test_two_quick_requests_still_serialize(daemon) -> None:
    """**同一条通道内仍然串行** —— 这是有意为之，不是没修完。

    机器狗上的 llama-server 是 `--parallel 1`，在宿主这边给 quick 多开
    worker 只会把队伍从宿主挪到狗上，还平白多几次等待中的超时。
    没有这条，一个"多开几个 worker 让并发更好看"的改动会全绿通过。
    """
    instance, delays = daemon
    delays["quick"] = 0.5
    elapsed: list[float] = []
    lock = threading.Lock()

    def _fire() -> None:
        seconds, _ = _roundtrip(instance.socket_path, "quick")
        with lock:
            elapsed.append(seconds)

    threads = [threading.Thread(target=_fire) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(elapsed) == 2
    assert max(elapsed) >= 0.9, (
        f"两条 quick 最慢的只用了 {max(elapsed):.2f}s，说明它们并发了；"
        f"llama-server 是 --parallel 1，这里并发没有收益"
    )
