"""FastAPI web server for the Go2-W manual WASD+QE demo.

Three fully independent chains (plan book §26 / §54):

* camera  : ROS worker -> latest.jpg -> MJPEG endpoint;
* motion  : WebSocket -> ManualDriveController -> ROS worker -> /go2w/motion;
* LLM     : SceneObjectAnalyzer background thread -> SiliconFlow -> table.

The controller runs in an asyncio tick task, the LLM in a daemon thread, and
the ROS worker in a subprocess with its own reader thread. The controller and
the worker talk only through thread-safe queues / the JSONL IPC.
"""

from __future__ import annotations

import asyncio
import json
import queue
import threading
import time
import urllib.request
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Callable

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from app.manual_web_demo.config import ManualDemoSettings, get_manual_demo_settings
from app.manual_web_demo.control_ownership import ControlOwner, OwnerState
from app.manual_web_demo.manual_drive_controller import (
    KEY_MAP,
    ManualDriveController,
)
from app.manual_web_demo.models import CameraStatus, SafetySnapshot
from app.manual_web_demo.ros_worker_client import RosWorkerClient
from app.manual_web_demo.scene_object_analyzer import SceneObjectAnalyzer
from app.manual_web_demo.search_routes import create_search_router
from app.manual_web_demo.search_session_service import SearchSessionService
from app.manual_web_demo.slam_map_snapshot import load_slam_map_snapshot

PACKAGE_DIR = Path(__file__).resolve().parent
INDEX_HTML = PACKAGE_DIR / "templates" / "index.html"
STATIC_DIR = PACKAGE_DIR / "static"
_DEFAULT_SEARCH_SESSION_DIR = "outputs/live_runs"

# The robot is on the directly attached 192.168.123.0/24 network.  The host
# may have a SOCKS proxy configured for general internet access; never route
# the D435 HTTP stream through that proxy.
_DIRECT_HTTP_OPENER = urllib.request.build_opener(
    urllib.request.ProxyHandler({})
)
urllib.request.install_opener(_DIRECT_HTTP_OPENER)


# --------------------------------------------------------------------------- #
# Motion executor that forwards controller intents to the ROS worker          #
# --------------------------------------------------------------------------- #
class RosWorkerMotionExecutor:
    """Adapts the controller's MotionExecutor protocol to the ROS worker."""

    def __init__(self, worker: RosWorkerClient) -> None:
        self._worker = worker
        self._on_result: Callable[[dict[str, Any]], None] | None = None

    def available(self) -> bool:
        return bool(self._worker.status().get("motion_available", False))

    def send_pulse(self, direction: str, on_result: Callable[[dict[str, Any]], None]) -> None:
        self._on_result = on_result
        self._worker.request_pulse(direction)

    def stop(self) -> None:
        self._worker.request_stop()

    def estop(self) -> None:
        self._worker.request_estop()

    def on_worker_message(self, msg_type: str, payload: dict[str, Any]) -> None:
        if msg_type != "motion_finished":
            return
        callback = self._on_result
        self._on_result = None
        if callback is not None:
            callback(payload)


# --------------------------------------------------------------------------- #
# WebSocket broadcast (thread-safe)                                            #
# --------------------------------------------------------------------------- #
class WebSocketBroadcaster:
    def __init__(self) -> None:
        self._connections: set[WebSocket] = set()
        self._lock = threading.Lock()
        self._outbox: queue.Queue[dict[str, Any]] = queue.Queue()

    def add(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.add(websocket)

    def remove(self, websocket: WebSocket) -> None:
        with self._lock:
            self._connections.discard(websocket)

    def publish(self, message: dict[str, Any]) -> None:
        """Thread-safe enqueue; the asyncio drain task actually sends."""
        try:
            self._outbox.put_nowait(message)
        except queue.Full:
            pass

    async def drain(self) -> None:
        """Background task: forward queued messages to every connected client."""
        while True:
            messages: list[dict[str, Any]] = []
            try:
                while True:
                    messages.append(self._outbox.get_nowait())
            except queue.Empty:
                pass
            if messages:
                with self._lock:
                    targets = list(self._connections)
                text = json.dumps(messages, ensure_ascii=False)
                for websocket in targets:
                    try:
                        await websocket.send_text(text)
                    except Exception:  # noqa: BLE001
                        pass
            await asyncio.sleep(0.05)


# --------------------------------------------------------------------------- #
# Runtime wiring                                                               #
# --------------------------------------------------------------------------- #
class DemoRuntime:
    """Owns the worker, controller, analyzer and the camera freshness state."""

    def __init__(
        self,
        config: ManualDemoSettings,
        *,
        worker: RosWorkerClient | None = None,
        analyzer_fn: Callable[[str], dict[str, Any]] | None = None,
        camera_fresh: Callable[[], bool] | None = None,
        search_executor_factory: Callable[[], Any] | None = None,
        search_session_dir: str | None = None,
    ) -> None:
        self.config = config
        self.broadcaster = WebSocketBroadcaster()
        self.owner = ControlOwner()
        self.worker = worker or RosWorkerClient(
            cmd=config.ros_worker_cmd,
            cwd=config.project_root,
            log_path=config.logs_dir_path / "ros_worker.log",
            keepalive_interval_sec=0.1,
        )
        self.executor = RosWorkerMotionExecutor(self.worker)
        self.controller = ManualDriveController(
            executor=self.executor,
            config=config,
            safety_provider=self._safety_snapshot,
            camera_fresh_provider=camera_fresh or self._camera_fresh,
            on_event=self._on_controller_event,
        )
        self.analyzer = SceneObjectAnalyzer(
            config=config,
            frame_provider=self._frame_provider,
            camera_fresh_provider=camera_fresh or self._camera_fresh,
            analyzer_fn=analyzer_fn,
        )
        self.search_service = SearchSessionService(
            owner=self.owner,
            executor_factory=search_executor_factory,
            session_dir=search_session_dir or _DEFAULT_SEARCH_SESSION_DIR,
        )
        self._last_camera_msg_time = 0.0
        self._camera_info: dict[str, Any] = {}
        self._started = False
        self._tasks: list[asyncio.Task] = []

        # Wire worker messages: motion results -> executor, status -> cache.
        self.worker.set_on_message(self._on_worker_message)

    # -- lifecycle ------------------------------------------------------ #
    def start(self) -> None:
        if self._started:
            return
        self.config.runtime_dir_path.mkdir(parents=True, exist_ok=True)
        self.config.logs_dir_path.mkdir(parents=True, exist_ok=True)
        self.worker.start()
        self.worker.request_status()
        self.analyzer.start()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        self.controller.disable(reason="server_shutdown")
        if self.controller.state.motion_in_flight:
            self.worker.request_estop()
        else:
            self.worker.request_stop()
        self.analyzer.stop()
        self.worker.stop()
        for task in self._tasks:
            task.cancel()

    def register_task(self, task: asyncio.Task) -> None:
        self._tasks.append(task)

    # -- providers ------------------------------------------------------- #
    def _camera_fresh(self) -> bool:
        if self._last_camera_msg_time <= 0.0:
            return self._file_camera_fresh()
        memory_fresh = (
            time.monotonic() - self._last_camera_msg_time
            < self.config.camera_stale_seconds
        )
        return memory_fresh or self._file_camera_fresh()

    def _file_camera_fresh(self) -> bool:
        """Accept the robot-local HTTP camera capture as a read-only source."""
        try:
            age = time.time() - self.config.latest_frame_path.stat().st_mtime
        except OSError:
            return False
        return age >= 0.0 and age < self.config.camera_stale_seconds

    def _safety_snapshot(self) -> SafetySnapshot:
        status = self.worker.status()
        pose = status.get("odom_pose")
        return SafetySnapshot(
            robot_mode=status.get("robot_mode"),
            robot_error_code=status.get("robot_error_code"),
            state_fresh=bool(status.get("state_fresh", False)),
            lease_alive=bool(status.get("lease_alive", False)),
            motion_action_available=bool(status.get("motion_available", False)),
            lidar_fresh=status.get("lidar_fresh"),
            front_clearance_m=status.get("front_clearance_m"),
            left_clearance_m=status.get("left_clearance_m"),
            right_clearance_m=status.get("right_clearance_m"),
            rotation_clearance_valid=status.get("rotation_clearance_valid"),
            odom_frame=status.get("odom_frame"),
            odom_pose=tuple(pose) if pose else None,
        )

    def _frame_provider(self) -> str | None:
        path = self.config.latest_frame_path
        return str(path) if path.is_file() else None

    def _on_worker_message(self, msg_type: str, payload: dict[str, Any]) -> None:
        if msg_type == "camera_status":
            self._last_camera_msg_time = time.monotonic()
            self._camera_info = payload
        elif msg_type == "worker_status":
            pass  # RosWorkerClient caches it; snapshot reads on demand.
        elif msg_type == "motion_finished":
            self.executor.on_worker_message(msg_type, payload)
        elif msg_type == "error":
            self.controller.disable(reason="worker_error")

    def _on_controller_event(self, message: dict[str, Any]) -> None:
        """Controller events -> broadcaster, with ControlOwner reconciliation
        (plan book §43): manual ownership ends when control is disabled or
        estop latches."""
        self.broadcaster.publish(message)
        if message.get("type") == "state":
            status = str((message.get("state") or {}).get("status") or "")
            if status in {"DISABLED", "ESTOP"} and self.owner.is_manual():
                self.owner.release(OwnerState.MANUAL)

    # -- snapshots -------------------------------------------------------- #
    def camera_snapshot(self) -> CameraStatus:
        memory_age = (
            time.monotonic() - self._last_camera_msg_time
            if self._last_camera_msg_time > 0.0
            else None
        )
        file_age = None
        file_width = None
        file_height = None
        try:
            file_age = time.time() - self.config.latest_frame_path.stat().st_mtime
            status = json.loads(
                self.config.camera_status_path.read_text(encoding="utf-8")
            )
            file_width = status.get("width")
            file_height = status.get("height")
        except (OSError, json.JSONDecodeError):
            pass
        memory_fresh = (
            memory_age is not None
            and memory_age < self.config.camera_stale_seconds
        )
        file_fresh = (
            file_age is not None
            and 0.0 <= file_age < self.config.camera_stale_seconds
        )
        age_candidates = [
            age for age in (memory_age, file_age)
            if age is not None and age >= 0.0
        ]
        age = min(age_candidates) if age_candidates else None
        return CameraStatus(
            available=memory_age is not None or file_age is not None,
            fresh=memory_fresh or file_fresh,
            age_seconds=age,
            width=self._camera_info.get("width") or file_width,
            height=self._camera_info.get("height") or file_height,
        )

    def status_snapshot(self) -> dict[str, Any]:
        camera = self.camera_snapshot()
        state = self.controller.state
        worker_status = self.worker.status()
        return {
            "camera": camera.to_dict(),
            "motion": {
                "available": bool(worker_status.get("motion_available", False)),
                "control_enabled": state.control_enabled,
                "state": state.status,
                "command": state.command,
                "pressed_key": state.pressed_key,
                "motion_in_flight": state.motion_in_flight,
                "blocked_reason": state.blocked_reason,
                "robot_mode": worker_status.get("robot_mode"),
                "robot_error_code": worker_status.get("robot_error_code"),
                "state_fresh": worker_status.get("state_fresh"),
            },
            "llm": {
                "enabled": self.analyzer.is_enabled(),
                "interval_seconds": self.config.llm_interval_seconds,
                "analysis": self.analyzer.state_dict(),
            },
            "control": {
                "default_disabled": not self.config.control_enabled_default,
                "deadman_ms": self.config.deadman_ms,
            },
            "directions": self.controller.direction_availability(),
            "worker": {
                "state": worker_status.get("state"),
                "alive": self.worker.alive(),
                "last_error": worker_status.get("last_error"),
            },
            "owner": self.owner.snapshot(),
            "search": self.search_service.executor_state(),
        }

    def search_readiness(self) -> dict[str, Any]:
        """ExperimentSearchReadiness (plan book §59): automatic checks only,
        no manual calibration may gate the WebUI.  Camera / worker / motion /
        robot-mode / estop / SLAM drift are blocking; LLM toggle, search worker
        and a not-yet-built map are informational (degraded-only)."""
        status = self.status_snapshot()
        camera = status.get("camera") or {}
        worker = status.get("worker") or {}
        motion = status.get("motion") or {}
        llm = status.get("llm") or {}
        search = status.get("search") or {}
        slam = load_slam_map_snapshot(self.config.slam_map_snapshot_path)
        checks = {
            "camera_fresh": bool(camera.get("fresh", False)),
            "camera_available": bool(camera.get("available", False)),
            "ros_worker_alive": bool(worker.get("alive", False)),
            "motion_action_available": bool(motion.get("available", False)),
            "robot_mode_ok": bool(
                motion.get("state_fresh") is True
                and motion.get("robot_mode") == 1
                and motion.get("robot_error_code") == 0
            ),
            "emergency_stop_available": bool(motion.get("available", False)),
            # §10.2：LIO 正在造假平移时地图已冻结，这时候不许开自主搜索；
            # 地图还没建起来只是降级信息，遥控建图必须仍然可用。
            "slam_map_not_drifting": slam.get("mapping_health") != "DEGRADED_LIO_DRIFT",
            "slam_map_live": bool(slam.get("available")) and bool(slam.get("fresh")),
            "llm_available": bool(llm.get("enabled", False)),
            "search_worker_available": bool(
                search.get("state") not in (None, "stopped") or search.get("alive")
            ),
        }
        blocking = (
            "camera_fresh", "camera_available", "ros_worker_alive",
            "motion_action_available", "robot_mode_ok",
            "emergency_stop_available", "slam_map_not_drifting",
        )
        blocked = [key for key in blocking if not checks[key]]
        degraded = [key for key in checks if not checks[key] and key not in blocking]
        ready = not blocked
        reason = "" if ready else "readiness failed: " + "; ".join(blocked)
        if not checks["slam_map_not_drifting"]:
            reason += " · " + str(slam.get("health_reason") or "")
        return {
            "ready": ready,
            "checks": checks,
            # Return only failed mandatory checks.  Returning the complete
            # list here made a healthy WebUI appear blocked even when
            # ``ready`` was true, which is especially confusing before an
            # operator enables autonomous motion.
            "blocking": blocked,
            "degraded": degraded,
            "reason": reason,
            "mapping_health": slam.get("mapping_health", "UNAVAILABLE"),
            "mapping_health_reason": slam.get("health_reason", ""),
            "owner": self.owner.snapshot(),
        }

    def reset_estop(self) -> dict[str, Any]:
        """Explicitly clear the WebUI estop latch after health checks.

        This only releases application ownership. It does not arm the robot
        or bypass the motion action server; the next search must still arm
        through ``/go2w/arm`` immediately before its first motion step.
        """
        status = self.status_snapshot()
        if not self.owner.is_estop():
            return {
                "ok": True,
                "status": "NOT_LATCHED",
                "owner": self.owner.snapshot(),
            }

        camera = status.get("camera") or {}
        motion = status.get("motion") or {}
        worker_status = self.worker.status()
        search = status.get("search") or {}
        checks = {
            "camera_fresh": camera.get("fresh") is True,
            "worker_alive": self.worker.alive(),
            "motion_available": motion.get("available") is True,
            "robot_mode_ok": (
                motion.get("state_fresh") is True
                and motion.get("robot_mode") == 1
                and motion.get("robot_error_code") == 0
            ),
            "lidar_fresh": worker_status.get("lidar_fresh") is True,
            "motion_idle": motion.get("motion_in_flight") is not True,
            "search_idle": not bool(search.get("alive")),
        }
        failed = [name for name, passed in checks.items() if not passed]
        if failed:
            return {
                "ok": False,
                "status": "ESTOP",
                "error": "estop_reset_blocked: " + ", ".join(failed),
                "checks": checks,
                "owner": self.owner.snapshot(),
            }

        self.controller.disable(reason="estop_reset")
        self.worker.request_stop()
        self.owner.reset_estop()
        self.broadcaster.publish(
            {"type": "estop_reset", "owner": self.owner.snapshot()}
        )
        return {"ok": True, "status": "RESET", "owner": self.owner.snapshot()}


# --------------------------------------------------------------------------- #
# App factory                                                                  #
# --------------------------------------------------------------------------- #
def create_app(
    config: ManualDemoSettings | None = None,
    runtime: DemoRuntime | None = None,
    *,
    search_executor_factory: Callable[[], Any] | None = None,
) -> FastAPI:
    config = config or get_manual_demo_settings()
    runtime = runtime or DemoRuntime(
        config, search_executor_factory=search_executor_factory
    )
    search_router, search_hub, ws_search_handler = create_search_router(
        runtime.search_service,
        on_estop=lambda: runtime.controller.estop(),
        readiness_provider=runtime.search_readiness,
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        runtime.start()
        tick_task = asyncio.create_task(_tick_loop(runtime))
        drain_task = asyncio.create_task(runtime.broadcaster.drain())
        runtime.register_task(tick_task)
        runtime.register_task(drain_task)
        search_hub.start()
        yield
        search_hub.stop()
        runtime.search_service.shutdown()
        for task in (tick_task, drain_task):
            task.cancel()
        runtime.stop()

    app = FastAPI(title="Go2-W Manual + Autonomous Search Console", lifespan=lifespan)

    # Never cache the console page or its static assets: the frontend has been
    # updated repeatedly (semantic topology / viewport gestures) and a stale
    # search_map.js in the browser cache made the new UI look "not installed".
    @app.middleware("http")
    async def _no_store_static(request, call_next):
        response = await call_next(request)
        path = request.url.path
        if path == "/" or path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
            response.headers["Pragma"] = "no-cache"
        return response

    if STATIC_DIR.is_dir():
        app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return INDEX_HTML.read_text(encoding="utf-8")

    @app.get("/api/status")
    async def api_status() -> dict[str, Any]:
        return runtime.status_snapshot()

    @app.get("/api/objects")
    async def api_objects() -> dict[str, Any]:
        return runtime.analyzer.state_dict()

    @app.get("/api/slam/map3d")
    async def api_slam_map3d() -> JSONResponse:
        """Latest decimated mapping-assist cloud; display-only and no-store."""
        response = JSONResponse(
            load_slam_map_snapshot(config.slam_map_snapshot_path)
        )
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    @app.post("/api/llm/enable")
    async def api_llm_enable() -> JSONResponse:
        runtime.analyzer.set_enabled(True)
        return JSONResponse({"ok": True, "enabled": True})

    @app.post("/api/llm/disable")
    async def api_llm_disable() -> JSONResponse:
        runtime.analyzer.set_enabled(False)
        return JSONResponse({"ok": True, "enabled": False})

    @app.post("/api/control/enable")
    async def api_control_enable() -> JSONResponse:
        ok_owner, owner_reason = runtime.owner.try_manual()
        if not ok_owner:
            return JSONResponse({"ok": False, "reason": owner_reason})
        ok = runtime.controller.enable()
        if not ok:
            runtime.owner.release(OwnerState.MANUAL)
        return JSONResponse(
            {
                "ok": ok,
                "reason": (
                    None if ok else runtime.controller.state.blocked_reason
                ),
            }
        )

    @app.post("/api/control/disable")
    async def api_control_disable() -> JSONResponse:
        runtime.controller.disable(reason="user_disabled")
        runtime.owner.release(OwnerState.MANUAL)
        return JSONResponse({"ok": True})

    @app.post("/api/estop")
    async def api_estop() -> JSONResponse:
        # Estop overrides every owner: manual controller AND search session
        # both stop (plan book §42); no new motion may follow.
        runtime.owner.estop()
        runtime.controller.estop()
        runtime.search_service.estop_search()
        return JSONResponse({"ok": True})

    @app.post("/api/estop/reset")
    async def api_estop_reset() -> JSONResponse:
        result = runtime.reset_estop()
        return JSONResponse(result, status_code=200 if result.get("ok") else 409)

    @app.get("/api/camera.mjpeg")
    async def api_camera_mjpeg(request: Request) -> StreamingResponse:
        return StreamingResponse(
            _mjpeg_generator(runtime, request),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    d435_base = getattr(config, "d435_base_url", None) or "http://192.168.123.18:8080"

    @app.get("/api/d435.mjpeg")
    async def api_d435_mjpeg() -> StreamingResponse:
        return StreamingResponse(
            _d435_proxy_mjpeg(d435_base, "/color"),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.get("/api/d435.depth.mjpeg")
    async def api_d435_depth_mjpeg() -> StreamingResponse:
        return StreamingResponse(
            _d435_proxy_mjpeg(d435_base, "/depth"),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    app.include_router(search_router)
    app.websocket("/ws/search")(ws_search_handler)

    @app.websocket("/ws/control")
    async def ws_control(websocket: WebSocket) -> None:
        await websocket.accept()
        runtime.broadcaster.add(websocket)
        await websocket.send_text(
            json.dumps(
                {"type": "state", "state": runtime.controller.state.to_dict()}
            )
        )
        try:
            while True:
                raw = await websocket.receive_text()
                message = _parse_ws_message(raw)
                if message is None:
                    continue
                await _dispatch_ws_message(runtime, message)
        except WebSocketDisconnect:
            runtime.controller.on_ws_disconnect()
        finally:
            runtime.broadcaster.remove(websocket)

    return app


# --------------------------------------------------------------------------- #
# Helpers                                                                      #
# --------------------------------------------------------------------------- #
async def _tick_loop(runtime: DemoRuntime) -> None:
    last_worker_restart = 0.0
    last_status_request = 0.0
    while True:
        try:
            runtime.controller.tick()
            # Keep the ROS worker alive: if it exited (crash / kill), respawn
            # it (throttled) so camera + motion recover without a manual demo
            # restart.
            if runtime._started and not runtime.worker.alive():
                now = time.monotonic()
                if now - last_worker_restart > 2.0:
                    last_worker_restart = now
                    runtime.worker.start()
                    runtime.worker.request_status()
            # Refresh the worker_status snapshot on demand (~every 2.5s) so
            # motion_available / robot mode / safety stay current without a
            # blocking per-second timer inside the worker.
            now = time.monotonic()
            if now - last_status_request > 2.5:
                last_status_request = now
                runtime.worker.request_status()
        except Exception:  # noqa: BLE001
            pass
        await asyncio.sleep(0.05)


async def _mjpeg_generator(runtime: DemoRuntime, request: Request | None = None) -> Any:
    frame_interval = 1.0 / max(1.0, runtime.config.camera_max_fps)
    latest = runtime.config.latest_frame_path
    placeholder: bytes | None = None
    while True:
        # Stop streaming as soon as the client disconnects so the ASGI task
        # (and any TestClient session) can shut down cleanly.
        if request is not None:
            try:
                if await request.is_disconnected():
                    break
            except Exception:  # noqa: BLE001
                break
        data = None
        if latest.is_file():
            try:
                data = latest.read_bytes()
            except OSError:
                data = None
        if data:
            payload = (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(data)).encode("ascii") + b"\r\n\r\n"
                + data
                + b"\r\n"
            )
            yield payload
        else:
            if placeholder is None:
                placeholder = _solid_jpeg()
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(placeholder)).encode("ascii") + b"\r\n\r\n"
                + placeholder
                + b"\r\n"
            )
        await asyncio.sleep(frame_interval)


def _d435_proxy_mjpeg(base_url: str, path: str):
    """Proxy D435 MJPEG stream from the robot's RealSense HTTP service."""
    url = base_url.rstrip("/") + path
    while True:
        try:
            with urllib.request.urlopen(url, timeout=5.0) as resp:
                while True:
                    chunk = resp.read(65536)
                    if not chunk:
                        break
                    yield chunk
        except Exception:  # noqa: BLE001 - keep trying so browser auto-recovers
            time.sleep(0.5)
            continue


_solid_jpeg_cache: bytes | None = None


def _solid_jpeg() -> bytes:
    """A small dark JPEG used while no camera frame has arrived yet."""
    global _solid_jpeg_cache
    if _solid_jpeg_cache is not None:
        return _solid_jpeg_cache
    try:
        from io import BytesIO

        from PIL import Image

        image = Image.new("RGB", (64, 36), (24, 24, 24))
        buffer = BytesIO()
        image.save(buffer, format="JPEG", quality=70)
        _solid_jpeg_cache = buffer.getvalue()
    except Exception:  # noqa: BLE001
        _solid_jpeg_cache = b""
    return _solid_jpeg_cache


def _parse_ws_message(raw: str) -> dict[str, Any] | None:
    text = raw.strip()
    if not text:
        return None
    try:
        message = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(message, dict):
        return None
    return message


async def _dispatch_ws_message(runtime: DemoRuntime, message: dict[str, Any]) -> None:
    msg_type = message.get("type")
    controller = runtime.controller
    if msg_type == "hello":
        runtime.broadcaster.publish(
            {"type": "state", "state": controller.state.to_dict()}
        )
    elif msg_type == "enable_control":
        ok_owner, owner_reason = runtime.owner.try_manual()
        if not ok_owner:
            runtime.broadcaster.publish(
                {
                    "type": "enable_result",
                    "ok": False,
                    "reason": owner_reason,
                }
            )
            return
        ok = controller.enable()
        if not ok:
            runtime.owner.release(OwnerState.MANUAL)
        runtime.broadcaster.publish(
            {
                "type": "enable_result",
                "ok": ok,
                "reason": None if ok else controller.state.blocked_reason,
            }
        )
    elif msg_type == "key_down":
        controller.on_key_down(str(message.get("key") or ""))
    elif msg_type == "key_up":
        controller.on_key_up(str(message.get("key") or ""))
    elif msg_type == "heartbeat":
        controller.on_heartbeat(
            message.get("pressed") or [], seq=message.get("seq")
        )
    elif msg_type == "release_all":
        controller.on_release_all()
    elif msg_type == "estop":
        runtime.owner.estop()
        controller.estop()
        runtime.search_service.estop_search()
    # Unknown message types are ignored.


# Module-level app for ``uvicorn app.manual_web_demo.web_server:app``.
app = create_app()
