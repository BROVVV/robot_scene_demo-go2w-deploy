"""REST + WebSocket routes for the autonomous search WebUI
(plan book §10, §21, §22-§24, §29, §51, §98).

Mounts a FastAPI ``APIRouter`` with ``/api/search/*`` and ``/ws/search`` on
top of the existing manual-demo FastAPI server (no second server).

对外搜索状态只有一个读口 ``GET /api/search/state``（小状态 + 一份 ``graph``）；
历史走 ``/api/search/history`` 与 ``/api/search/history/{id}``；三维地图走
``GET /api/slam/map3d``。曾经从 ``state`` 重复投影出来的 ``/map``、
``/place-graph``、``/frontiers``、``/semantic-map``、``/objects``、
``/decisions``、``/history/decisions``、``/events``、``/executor`` 已删除。
"""

from __future__ import annotations

import asyncio
import json
import queue
import re
import threading
import time
from pathlib import Path
from typing import Any, Callable

from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response, StreamingResponse

from app.async_compat import to_thread

from app.live_robot.search_event import SearchEvent
from app.manual_web_demo.config import DISPLAY_ONLY_REASON
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_errors import search_error
from app.manual_web_demo.search_session_service import SearchSessionService

HEARTBEAT_INTERVAL_SEC = 15.0

_RANGE_RE = re.compile(r"^bytes=(\d*)-(\d*)$")
_MIME_TYPES = {
    ".mp4": "video/mp4",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
}


def _parse_range(header: str | None, size: int) -> tuple[int, int] | str | None:
    """Parse one ``bytes=`` range.

    Returns ``(start, end)`` inclusive, ``"invalid"`` for a syntactically valid
    but unsatisfiable range (416), or ``None`` to serve the whole file.  A
    multi-range request is deliberately ignored (200) — the player only sends
    single ranges and multipart/byteranges buys nothing here.
    """
    if not header or size <= 0:
        return None
    match = _RANGE_RE.match(header.strip())
    if not match:
        return None
    first, last = match.group(1), match.group(2)
    if not first and not last:
        return "invalid"
    if not first:
        suffix = int(last)
        if suffix <= 0:
            return "invalid"
        return max(0, size - suffix), size - 1
    start = int(first)
    end = int(last) if last else size - 1
    if start >= size:
        return "invalid"
    end = min(end, size - 1)
    if end < start:
        return "invalid"
    return start, end


def _stream_path(
    path: Path, request: Request, lock: Any, *,
    download_name: str | None = None,
) -> Response:
    """Serve a replay artifact with HTTP Range support and a shared file lock."""
    try:
        size = path.stat().st_size
    except OSError:
        return Response(status_code=404)
    content_type = _MIME_TYPES.get(path.suffix.lower(), "application/octet-stream")
    parsed = _parse_range(request.headers.get("range"), size)
    if parsed == "invalid":
        return Response(
            status_code=416,
            headers={"Content-Range": f"bytes */{size}",
                     "Accept-Ranges": "bytes"},
        )
    if isinstance(parsed, tuple):
        start, end = parsed
        status_code = 206
    else:
        start, end = 0, max(0, size - 1)
        status_code = 200
    length = max(0, end - start + 1)
    headers = {
        "Content-Type": content_type,
        "Content-Length": str(length),
        "Accept-Ranges": "bytes",
        "Cache-Control": "private, no-cache",
        # 回放读的是历史文件，不需要 CORS，也不需要 service worker 缓存。
        "X-Content-Type-Options": "nosniff",
    }
    if status_code == 206:
        headers["Content-Range"] = f"bytes {start}-{end}/{size}"
    if download_name:
        headers["Content-Disposition"] = f'attachment; filename="{download_name}"'

    def body():
        # 共享锁只防"归档清理把正在播放的会话删掉"，拿不到也照常读：
        # 已完成文件被原子替换时 POSIX 仍保留旧 inode。
        held = False
        try:
            held = bool(lock.acquire(shared=True, timeout=5.0))
        except Exception:  # noqa: BLE001
            held = False
        try:
            with path.open("rb") as handle:
                handle.seek(start)
                remaining = length
                while remaining > 0:
                    chunk = handle.read(min(256 * 1024, remaining))
                    if not chunk:
                        break
                    remaining -= len(chunk)
                    yield chunk
        finally:
            if held:
                lock.release()

    return StreamingResponse(body(), status_code=status_code, headers=headers)

# 计划书 §2.1：跨线程入口是有界队列 + 50 ms 批量广播 + 每批 1 秒超时。
OUTBOX_CAPACITY = 256
BATCH_INTERVAL_SEC = 0.05
SEND_TIMEOUT_SEC = 1.0


def _snapshot_message(state: dict[str, Any]) -> dict[str, Any]:
    """The single full-state message shape used for open and for resync."""
    return {"type": "snapshot", "state": state}


# --------------------------------------------------------------------------- #
# WebSocket hub: bridges the SearchEventBus to every /ws/search client         #
# --------------------------------------------------------------------------- #
class SearchEventHub:
    """Subscribes the service event bus once and fans events out to all
    connected ``/ws/search`` sockets (thread-safe).

    The executor read thread only does ``put_nowait`` on a fixed-capacity
    :class:`queue.Queue`; the ASGI loop drains it in batches.  A slow client
    can therefore never block the search, and an overflow degrades to a state
    resync instead of an unbounded pending list.  Archive and store updates
    happen before the enqueue, so a dropped increment never loses a fact.
    """

    def __init__(self, service: SearchSessionService) -> None:
        self._service = service
        self._connections: set[WebSocket] = set()
        self._lock = threading.Lock()
        self._outbox: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=OUTBOX_CAPACITY)
        self._resync = threading.Event()
        self._event_generation = 0
        self._unsubscribe = service.subscribe_events(self._on_event)
        self._task: asyncio.Task | None = None

    def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._drain_loop())

    def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        try:
            self._unsubscribe()
        except Exception:  # noqa: BLE001
            pass

    def add(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.add(websocket)

    def remove(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.discard(websocket)

    def _on_event(self, event: SearchEvent) -> None:
        with self._lock:
            self._event_generation += 1
            try:
                self._outbox.put_nowait({"type": "event", "event": event.to_dict()})
            except queue.Full:
                # 丢掉待发增量并标记重同步。store / 归档已经在入队前更新完，
                # 所以这一步只影响推送的及时性，不影响事实。
                self._drain_pending()
                self._resync.set()

    def event_generation(self) -> int:
        with self._lock:
            return self._event_generation

    def has_pending_events(self) -> bool:
        return not self._outbox.empty()

    def request_resync(self) -> None:
        self._resync.set()

    def _drain_pending(self) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        while len(items) < OUTBOX_CAPACITY:
            try:
                items.append(self._outbox.get_nowait())
            except queue.Empty:
                break
        return items

    async def _drain_loop(self) -> None:
        while True:
            try:
                await asyncio.sleep(BATCH_INTERVAL_SEC)
                if self._resync.is_set():
                    self._resync.clear()
                    self._drain_pending()  # 这批增量已被快照覆盖
                    await self._broadcast_texts([
                        json.dumps(
                            _snapshot_message(self._service.state_snapshot()),
                            ensure_ascii=False,
                        )
                    ])
                    continue
                batch = self._drain_pending()
                if not batch:
                    continue
                await self._broadcast_texts(
                    [json.dumps(item, ensure_ascii=False) for item in batch]
                )
            except asyncio.CancelledError:
                break

    async def _broadcast_texts(self, texts: list[str]) -> None:
        """Send this batch to every connection, then start the next batch.

        One task per connection, each with a per-batch timeout; a connection
        that misses the timeout is dropped instead of holding up the others.
        """
        with self._lock:
            targets = list(self._connections)
        if not targets:
            return

        async def send_one(websocket: WebSocket) -> None:
            for text in texts:
                await websocket.send_text(text)

        results = await asyncio.gather(
            *(asyncio.wait_for(send_one(websocket), timeout=SEND_TIMEOUT_SEC)
              for websocket in targets),
            return_exceptions=True,
        )
        for websocket, result in zip(targets, results):
            if isinstance(result, BaseException):
                self.remove(websocket)


# --------------------------------------------------------------------------- #
# Router factory                                                              #
# --------------------------------------------------------------------------- #
def create_search_router(
    service: SearchSessionService,
    *,
    on_estop: Callable[[], None] | None = None,
    readiness_provider: Callable[[], dict[str, Any]] | None = None,
    display_only_provider: Callable[[], bool] | None = None,
) -> tuple[APIRouter, SearchEventHub, Any]:
    """Returns (api_router, hub, ws_handler).

    The WebSocket handler is returned separately because ``/ws/search`` must
    live outside the ``/api/search`` prefix (plan book §22).

    ``display_only_provider`` 为真时（实体遥控器建图模式，计划书 §6.2 F），
    能起运动或发急停的三个入口被挡住：``/start``、``/resume``、``/estop``。
    只读端点（state / readiness / history / map3d）一律不受影响 —— 页面仍要能看。
    """
    router = APIRouter(prefix="/api/search")
    hub = SearchEventHub(service)

    def _display_only_403() -> JSONResponse | None:
        if display_only_provider is None or not display_only_provider():
            return None
        detail = search_error(
            DISPLAY_ONLY_REASON, source="search_api", stage="DISPLAY_ONLY"
        )
        return JSONResponse(
            {"ok": False, "error": detail["message"], "error_detail": detail},
            status_code=403,
        )

    def _estop() -> None:
        service.estop_search()
        if on_estop is not None:
            on_estop()

    @router.post("/start")
    async def search_start(request: Request) -> JSONResponse:
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        try:
            raw = await request.json()
        except Exception:  # noqa: BLE001
            raw = {}
        if not isinstance(raw, dict):
            raw = {}
        req = SearchStartRequest.from_dict(raw)
        # §10.2：LIO 造出假平移、全局地图已冻结时禁止自主平移搜索。
        # 这里只拦地图健康这一项，遥控建图和三维显示仍然只读可用。
        if readiness_provider is not None:
            readiness = readiness_provider()
            checks = readiness.get("checks") or {}
            if checks.get("slam_map_not_drifting", True) is False:
                detail = search_error(
                    "mapping frozen: "
                    + str(readiness.get("mapping_health_reason")
                          or readiness.get("mapping_health") or ""),
                    source="search_api", stage="START",
                )
                return JSONResponse(
                    {"ok": False, "error": detail["message"],
                     "error_detail": detail},
                    status_code=409,
                )
            if checks.get("navigation_trial_available", True) is False:
                detail = search_error(
                    "navigation trial exhausted or already active",
                    source="search_api", stage="START",
                )
                return JSONResponse(
                    {"ok": False, "error": detail["message"],
                     "error_detail": detail},
                    status_code=409,
                )
        # Task understanding may call the configured LLM and can take
        # seconds (or hit a provider timeout).  Never run it on the ASGI
        # event-loop thread: a synchronous call here makes every fetch,
        # websocket heartbeat, and status poll look like a network failure.
        try:
            loop = asyncio.get_running_loop()
            result = await loop.run_in_executor(None, service.start_search, req)
        except Exception as exc:  # noqa: BLE001 - return a usable API error
            detail = search_error(
                f"search start failed: {type(exc).__name__}: {exc}",
                source="search_api", stage="START",
            )
            return JSONResponse(
                {"ok": False, "error": detail["message"], "error_detail": detail},
                status_code=500,
            )
        if result.get("ok"):
            return JSONResponse(
                {
                    "ok": True,
                    "session_id": result["session_id"],
                    "status": result["status"],
                }
            )
        status_code = 403 if result.get("forbidden") else (409 if result.get("conflict") else 400)
        return JSONResponse(
            {"ok": False, "error": result["error"],
             "error_detail": result.get("error_detail")},
            status_code=status_code,
        )

    @router.post("/pause")
    async def search_pause() -> JSONResponse:
        result = service.pause_search()
        return JSONResponse(result, status_code=200 if result.get("ok") else 409)

    @router.post("/resume")
    async def search_resume() -> JSONResponse:
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        result = service.resume_search()
        return JSONResponse(result, status_code=200 if result.get("ok") else 409)

    @router.post("/stop")
    async def search_stop() -> JSONResponse:
        result = service.stop_search()
        return JSONResponse(result)

    @router.post("/estop")
    async def search_estop() -> JSONResponse:
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        _estop()
        return JSONResponse({"ok": True, "status": "ESTOP"})

    @router.get("/state")
    async def search_state() -> dict[str, Any]:
        return service.state_snapshot()

    @router.get("/history")
    async def search_history(limit: int = 10) -> dict[str, Any]:
        return {"sessions": service.history(min(10, limit)), "max_sessions": 10}

    @router.get("/history/{session_id}")
    async def search_history_session(session_id: str) -> JSONResponse:
        try:
            record = service.history_session(session_id)
        except ValueError:
            record = None
        if record is None:
            detail = search_error(
                "未找到指定的搜索记录，记录可能已超过最近 10 次的滚动保留范围。",
                code="SESSION_NOT_FOUND", source="search_archive", stage="HISTORY",
            )
            return JSONResponse(
                {"ok": False, "error": detail["message"], "error_detail": detail},
                status_code=404,
            )
        return JSONResponse({"ok": True, **record})

    @router.get("/readiness")
    async def search_readiness() -> dict[str, Any]:
        if readiness_provider is not None:
            return readiness_provider()
        return service.executor_state()

    # ------------------------------------------------------------------ #
    # 回放（只读优先；生成只排队，绝不在请求里编码）                       #
    # ------------------------------------------------------------------ #
    def _replay_manager() -> Any | None:
        return getattr(service, "replay_manager", None)

    def _replay_404(message: str) -> JSONResponse:
        detail = search_error(
            message, code="REPLAY_NOT_FOUND", source="search_replay",
            stage="REPLAY",
        )
        return JSONResponse(
            {"ok": False, "error": detail["message"], "error_detail": detail},
            status_code=404,
        )

    @router.get("/history/{session_id}/replay")
    async def replay_status(session_id: str) -> JSONResponse:
        manager = _replay_manager()
        if manager is None:
            return _replay_404("回放功能未启用。")
        # status() verifies the artifact can actually decode (ffprobe +
        # stream decode on first sight); keep that off the event loop.
        status = await to_thread(manager.status, session_id)
        if status is None:
            return _replay_404(
                "未找到指定的搜索记录，记录可能已超过最近 10 次的滚动保留范围。"
            )
        return JSONResponse({"ok": True, **status})

    @router.post("/history/{session_id}/replay/render")
    async def replay_render(session_id: str, request: Request) -> JSONResponse:
        manager = _replay_manager()
        if manager is None:
            return _replay_404("回放功能未启用。")
        try:
            raw = await request.json()
        except Exception:  # noqa: BLE001
            raw = {}
        if not isinstance(raw, dict):
            raw = {}
        result = await to_thread(
            manager.request_render,
            session_id,
            speed=raw.get("speed"),
            force=bool(raw.get("force")),
        )
        if result.get("ok"):
            return JSONResponse(result, status_code=202 if result.get("queued") else 200)
        message = str(result.get("error") or "无法生成回放")
        if "不存在" in message or "未找到" in message:
            return _replay_404(message)
        detail = search_error(
            message, code="REPLAY_RENDER_REFUSED", source="search_replay",
            stage="REPLAY_RENDER",
        )
        return JSONResponse(
            {"ok": False, "error": message, "error_detail": detail},
            status_code=409,
        )

    @router.get("/history/{session_id}/replay/video")
    async def replay_video(session_id: str, request: Request) -> Response:
        # status() 首次见的成品要跑 ffprobe + 解码校验，可能在磁盘冷的时候
        # 花上几秒；播放器发 Range 请求很密，绝不能压在事件循环上。
        return await to_thread(
            _replay_file_response, session_id, request, "video"
        )

    @router.get("/history/{session_id}/replay/download")
    async def replay_download(session_id: str, request: Request) -> Response:
        return await to_thread(
            _replay_file_response, session_id, request, "video", download=True
        )

    @router.get("/history/{session_id}/replay/final-map")
    async def replay_final_map(session_id: str, request: Request) -> Response:
        return await to_thread(
            _replay_file_response, session_id, request, "final_map"
        )

    @router.get("/history/{session_id}/replay/cover")
    async def replay_cover(session_id: str, request: Request) -> Response:
        return await to_thread(
            _replay_file_response, session_id, request, "cover"
        )

    def _replay_file_response(
        session_id: str, request: Request, kind: str, *, download: bool = False,
    ) -> Response:
        manager = _replay_manager()
        if manager is None:
            return _replay_404("回放功能未启用。")
        if manager.status(session_id) is None:
            return _replay_404("未找到指定的搜索记录。")
        path = manager.artifact_path(session_id, kind)
        if path is None:
            return _replay_404(
                "回放成品尚未生成" if kind == "video" else "尚未生成该图片。"
            )
        return _stream_path(
            path, request, manager.stream_lock(session_id),
            download_name=(
                f"go2w_{session_id}_replay.mp4" if download else None
            ),
        )

    async def ws_search(websocket: WebSocket) -> None:
        await websocket.accept()
        # 先送状态再订阅增量。若订阅窗口发生事件或已有事件排队，立刻从
        # 当前 store 重同步一次，避免回放旧事件覆盖刚收到的状态。
        try:
            generation = hub.event_generation()
            pending = hub.has_pending_events()
            await websocket.send_text(
                json.dumps(
                    _snapshot_message(service.state_snapshot()), ensure_ascii=False
                )
            )
            hub.add(websocket)
            if pending or hub.event_generation() != generation:
                hub.request_resync()
            last_heartbeat = time.monotonic()
            while True:
                now = time.monotonic()
                if now - last_heartbeat >= HEARTBEAT_INTERVAL_SEC:
                    last_heartbeat = now
                    await websocket.send_text(
                        json.dumps({"type": "heartbeat"}, ensure_ascii=False)
                    )
                try:
                    await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                except WebSocketDisconnect:
                    break
        except WebSocketDisconnect:
            pass
        finally:
            hub.remove(websocket)

    return router, hub, ws_search
