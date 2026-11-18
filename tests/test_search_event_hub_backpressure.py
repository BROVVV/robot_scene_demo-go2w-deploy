"""§2.1 的跨线程推送契约：**慢客户端不许拖慢搜索**。

三条各自独立的性质，都必须在**没有真机、没有 rclpy** 的情况下可证：

1. 执行器读线程只做 ``put_nowait``。队列满了就丢增量 + 标记重同步，
   **绝不阻塞、绝不无界增长**（旧实现是一个会一直长的 pending 列表）。
2. 一个卡住的连接不许拖住别的连接：每批发送 1 秒超时，超时即断开该连接。
3. 重同步时发的是**当前状态快照**，不是补发丢失的增量 —— store / 归档已经在
   入队前更新完，所以丢的只是"推送的及时性"，不是事实。

这三条合起来才是"50 ms 批量 + 有界队列"的意义。只测"消息能发出去"是没有分辨
力的：旧实现在只有一个快客户端时同样能发出去，病是在**并发 + 慢**的时候才发。
"""

from __future__ import annotations

import asyncio
import json
import threading

from app.live_robot.search_event import EventIdAllocator, make_event
from app.manual_web_demo.search_routes import (
    BATCH_INTERVAL_SEC,
    OUTBOX_CAPACITY,
    SEND_TIMEOUT_SEC,
    SearchEventHub,
    create_search_router,
)
from fastapi import WebSocketDisconnect


class FakeService:
    """Hub 真正用到的只有三个方法（订阅 / 读快照 / 读历史事件）。"""

    def __init__(self) -> None:
        self.listeners: list = []
        self.snapshots = 0

    def subscribe_events(self, listener):
        self.listeners.append(listener)
        return lambda: self.listeners.remove(listener)

    def state_snapshot(self) -> dict:
        self.snapshots += 1
        return {"session_id": "search_test", "status": "RUNNING", "graph": None}

    def recent_events(self, limit: int) -> list:
        return []

    def publish(self, event) -> None:
        for listener in list(self.listeners):
            listener(event)


class FakeWebSocket:
    def __init__(self, *, delay: float = 0.0) -> None:
        self.delay = delay
        self.sent: list[str] = []

    async def send_text(self, text: str) -> None:
        if self.delay:
            await asyncio.sleep(self.delay)
        self.sent.append(text)


def _event(index: int):
    return make_event(
        allocator=EventIdAllocator(), session_id="search_test",
        event_type="OBSERVATION_UPDATED", payload={"index": index},
    )


def test_overflow_drops_increments_and_marks_resync_without_blocking():
    """生产者永远不会因为消费不过来而卡住。

    ``put_nowait`` 满了就抛 ``queue.Full``，hub 接住它、清空待发、置重同步位。
    这里从**另一个线程**灌 4 倍容量的增量：只要 ``_on_event`` 里有任何一处阻塞，
    这个循环就会挂住，测试会超时。
    """
    service = FakeService()
    hub = SearchEventHub(service)
    try:
        producer = threading.Thread(
            target=lambda: [service.publish(_event(i))
                            for i in range(OUTBOX_CAPACITY * 4)],
        )
        producer.start()
        producer.join(timeout=10)
        assert not producer.is_alive(), "生产端被阻塞了 —— 队列不是有界的 put_nowait"
        assert hub._outbox.qsize() <= OUTBOX_CAPACITY
        assert hub._resync.is_set(), "溢出后必须标记重同步，否则页面会永久缺一段"
    finally:
        hub.stop()


def test_a_slow_client_is_dropped_instead_of_stalling_the_fast_one():
    """一个 3 秒才发完的连接不许让快连接等它 —— 超时即断开。"""
    service = FakeService()
    hub = SearchEventHub(service)
    slow = FakeWebSocket(delay=SEND_TIMEOUT_SEC * 3)
    fast = FakeWebSocket()
    hub.add(slow)
    hub.add(fast)

    async def run() -> None:
        await hub._broadcast_texts(["a", "b"])

    try:
        asyncio.run(run())
    finally:
        hub.stop()

    assert fast.sent == ["a", "b"], "快连接必须照样收到这批"
    assert slow.sent == [], "慢连接一条都发不出去"
    assert slow not in hub._connections, "超时的连接必须被摘掉"
    assert fast in hub._connections


def test_timeout_applies_to_the_whole_batch():
    service = FakeService()
    hub = SearchEventHub(service)
    slow = FakeWebSocket(delay=SEND_TIMEOUT_SEC * 0.4)
    fast = FakeWebSocket()
    hub.add(slow)
    hub.add(fast)
    try:
        asyncio.run(hub._broadcast_texts(["a", "b", "c"]))
    finally:
        hub.stop()
    assert fast.sent == ["a", "b", "c"]
    assert slow not in hub._connections
    assert len(slow.sent) < 3


def test_websocket_starts_with_one_state_snapshot_and_no_event_replay():
    service = FakeService()
    _router, hub, ws_search = create_search_router(service)

    class NewConnection(FakeWebSocket):
        async def accept(self) -> None:
            pass

        async def receive_text(self) -> str:
            raise WebSocketDisconnect()

    client = NewConnection()
    try:
        asyncio.run(ws_search(client))
    finally:
        hub.stop()
    assert [json.loads(item)["type"] for item in client.sent] == ["snapshot"]


def test_event_during_websocket_join_requests_fresh_snapshot():
    service = FakeService()
    _router, hub, ws_search = create_search_router(service)

    class RacingConnection(FakeWebSocket):
        async def accept(self) -> None:
            pass

        async def send_text(self, text: str) -> None:
            self.sent.append(text)
            if len(self.sent) == 1:
                service.publish(_event(1))

        async def receive_text(self) -> str:
            raise WebSocketDisconnect()

    client = RacingConnection()
    try:
        asyncio.run(ws_search(client))
        assert hub._resync.is_set()
    finally:
        hub.stop()


def test_resync_pushes_a_snapshot_not_the_lost_increments():
    """重同步发的是当前 state 快照 —— 事实在入队前已经落库了。"""
    service = FakeService()
    hub = SearchEventHub(service)
    client = FakeWebSocket()
    hub.add(client)

    for index in range(OUTBOX_CAPACITY + 5):
        service.publish(_event(index))
    assert hub._resync.is_set()

    async def run() -> None:
        task = asyncio.create_task(hub._drain_loop())
        for _ in range(200):
            await asyncio.sleep(BATCH_INTERVAL_SEC)
            if client.sent:
                break
        task.cancel()

    try:
        asyncio.run(run())
    finally:
        hub.stop()

    assert client.sent, "重同步必须真的推出去一条"
    message = json.loads(client.sent[0])
    assert message["type"] == "snapshot", (
        f"重同步应当是快照而不是补发增量，实际是 {message['type']}"
    )
    assert service.snapshots >= 1
    assert not hub._resync.is_set(), "重同步标志用后必须清掉，否则会一直重发快照"
