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
import os
import queue
import threading
import time
import urllib.request
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Callable

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from app.async_compat import to_thread

from app.manual_web_demo.config import (
    DISPLAY_ONLY_REASON,
    ManualDemoSettings,
    get_manual_demo_settings,
)
from app.manual_web_demo.control_ownership import ControlOwner, OwnerState
from app.manual_web_demo.manual_drive_controller import (
    KEY_MAP,
    ManualDriveController,
)
from app.manual_web_demo.models import CameraStatus, SafetySnapshot
from app.manual_web_demo.ros_worker_client import RosWorkerClient
from app.manual_web_demo.runtime_admin import RuntimeAdminService
from app.manual_web_demo.scene_object_analyzer import SceneObjectAnalyzer
from app.manual_web_demo.search_routes import create_search_router
from app.manual_web_demo.search_session_service import SearchSessionService
from app.manual_web_demo.search_models import load_search_defaults
from app.manual_web_demo.slam_map_snapshot import load_slam_map_snapshot
from app.manual_web_demo.target_watch_service import TargetWatchService
from app.spatial.object_display_snapshot import (
    _load_json_object,
    load_map_meta,
    load_object_snapshot,
)
from app.search_replay.manager import ReplayManager

PACKAGE_DIR = Path(__file__).resolve().parent
INDEX_HTML = PACKAGE_DIR / "templates" / "index.html"
STATIC_DIR = PACKAGE_DIR / "static"
_DEFAULT_SEARCH_SESSION_DIR = "outputs/live_runs"


def _filter_mapping_state_to_display_objects(
    snapshot: dict[str, Any], object_snapshot: dict[str, Any] | None
) -> dict[str, Any]:
    """Project topology through the exact object set used by the metric map."""
    graph_container = dict(snapshot.get("graph") or {})
    topology = dict(graph_container.get("object_topology") or {})
    expected_value = snapshot.get("entity_revision")
    if expected_value is None:
        expected_value = snapshot.get("revision")
    expected_revision = int(expected_value if expected_value is not None else -2)
    object_revision_value = (
        object_snapshot.get("graph_revision") if object_snapshot is not None else None
    )
    if (object_snapshot is None
            or object_snapshot.get("map_epoch") != snapshot.get("map_epoch")
            or int(object_revision_value if object_revision_value is not None else -1)
                != expected_revision):
        graph_container["object_topology"] = {
            **topology,
            "nodes": [],
            "edges": [],
            "stats": {"node_count": 0, "edge_count": 0,
                      "connected_components": 0},
        }
        graph_container["objects"] = []
        return {
            **snapshot,
            "graph": graph_container,
            "semantic_version_mismatch": True,
        }

    allowed = {
        str(item.get("object_id") or "")
        for item in object_snapshot.get("objects", [])
        if item.get("object_id")
    }
    allowed.update(
        str(item.get("node_id") or "")
        for item in topology.get("nodes", [])
        if item.get("target_watch_only") and item.get("is_target_confirmed")
    )
    nodes = [item for item in topology.get("nodes", [])
             if item.get("node_id") in allowed]
    edges = [item for item in topology.get("edges", [])
             if item.get("from") in allowed and item.get("to") in allowed]
    parent = {item["node_id"]: item["node_id"] for item in nodes}

    def find(value: str) -> str:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    for edge in edges:
        left, right = find(edge["from"]), find(edge["to"])
        if left != right:
            parent[right] = left
    stats = dict(topology.get("stats") or {})
    stats.update({
        "node_count": len(nodes),
        "edge_count": len(edges),
        "connected_components": len({find(value) for value in parent}) if parent else 0,
        "confirmed_nodes": sum(
            str(item.get("status") or "").upper() == "CONFIRMED" for item in nodes
        ),
        "confirmed_edges": sum(
            str(item.get("status") or "").upper() == "CONFIRMED" for item in edges
        ),
        "active_nodes": sum(item.get("lifecycle_state") == "active" for item in nodes),
        "stale_nodes": sum(item.get("lifecycle_state") == "stale" for item in nodes),
        "missing_candidate_nodes": sum(
            item.get("lifecycle_state") == "missing_candidate" for item in nodes
        ),
        "located_nodes": len(nodes),
    })
    graph_container["object_topology"] = {
        **topology,
        "nodes": nodes,
        "edges": edges,
        "stats": stats,
    }
    graph_container["objects"] = [
        item for item in graph_container.get("objects", [])
        if item.get("object_id") in allowed
    ]
    return {
        **snapshot,
        "graph": graph_container,
        "semantic_version_mismatch": False,
    }

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
        replay_manager: ReplayManager | None = None,
        enable_replay: bool | None = None,
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
        # 回放录制：真实搜索用独立只读 ROS 采集器；mock 工厂（测试台注入）
        # 默认不录，免得把合成数据写进真实回放目录。
        is_mock_factory = bool(
            getattr(search_executor_factory, "is_mock_factory", False)
        )
        self.replay_manager = replay_manager or ReplayManager(
            sessions_root=search_session_dir or _DEFAULT_SEARCH_SESSION_DIR,
            project_root=config.project_root,
            enabled=enable_replay,
            source="mock" if is_mock_factory else "real",
            config_path=Path(config.project_root) / "configs/go2w/search_replay.yaml"
            if (Path(config.project_root) / "configs/go2w/search_replay.yaml").is_file()
            else None,
        )
        if is_mock_factory and enable_replay is None:
            self.replay_manager.enabled = False
        self.search_service = SearchSessionService(
            owner=self.owner,
            executor_factory=search_executor_factory,
            session_dir=search_session_dir or _DEFAULT_SEARCH_SESSION_DIR,
            # Acceptance sessions can retain existing user evidence without
            # triggering the default rolling-history prune on the first run.
            history_limit=max(1,int(os.getenv('AUTONOMOUS_SEARCH_HISTORY_LIMIT','10'))),
            # §2.3：搜索 worker 把三维对象写到哪、读哪份地图元数据，由 Web 侧这
            # 一份配置说了算；读取端（/api/slam/map3d）用的是同样的两个路径。
            map_paths={
                "map_meta_path": config.slam_map_meta,
                "object_snapshot_path": config.semantic_objects_snapshot,
            },
            replay_manager=self.replay_manager,
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
        # 启动回放恢复检查（崩溃遗留的 QUEUED/RENDERING、未收尾的录制）。
        self.replay_manager.start()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        self.controller.disable(reason="server_shutdown")
        # 只读（实体遥控器）模式下退出时**不得**替操作员发停止命令：计划书
        # §6.2 F 要求后端不发 stop/estop/motion，§9.4 更直接——软件不拥有运动
        # 控制权，退出时不能假设自己有权给机器狗下停止。机器狗由实体遥控器停。
        if not self.config.display_only:
            if self.controller.state.motion_in_flight:
                self.worker.request_estop()
            else:
                self.worker.request_stop()
        self.analyzer.stop()
        self.worker.stop()
        self.replay_manager.shutdown()
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
        nav_profile = os.getenv("GO2W_NAV2_SEARCH_PROFILE", "").strip()
        execution_authorized = os.getenv(
            "GO2W_NAV2_EXECUTION_AUTHORIZED", ""
        ).strip() == "1"
        trial_path = os.getenv("GO2W_NAV2_TRIAL_LEDGER", "").strip()
        trial_status: dict[str, Any] | None = None
        if trial_path:
            try:
                trial = json.loads(Path(trial_path).read_text(encoding="utf-8"))
                maximum_steps = int(trial["maximum_steps"])
                used_steps = len(trial["steps"])
                trial_status = {
                    "policy_version": int(trial["version"]),
                    "maximum_steps": maximum_steps,
                    "used_steps": used_steps,
                    "remaining_steps": max(0, maximum_steps - used_steps),
                    "active": trial.get("active_token") is not None,
                }
            except (OSError, ValueError, KeyError, TypeError):
                trial_status = {"invalid": True}
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
                # 前端据此隐藏 WASD/QE 控制区并显示只读横幅。
                "display_only": self.config.display_only,
                "display_only_reason": (
                    DISPLAY_ONLY_REASON if self.config.display_only else None
                ),
            },
            "web_3d_map_enabled": self.config.web_3d_map_enabled,
            "web_2d_map_enabled": self.config.web_2d_map_enabled,
            "directions": self.controller.direction_availability(),
            "worker": {
                "state": worker_status.get("state"),
                "alive": self.worker.alive(),
                "last_error": worker_status.get("last_error"),
            },
            "owner": self.owner.snapshot(),
            "search": self.search_service.executor_state(),
            # The browser shows the server-selected contract. Search requests
            # cannot widen this profile or create/refund a trial allowance.
            "navigation": {
                "backend": "nav2",
                "profile": nav_profile or "default",
                "execution_authorized": execution_authorized,
                "maximum_goal_travel_m": 3.0 if nav_profile == "field_large" else None,
                "maximum_search_radius_m": 5.0 if nav_profile == "field_large" else None,
                "maximum_linear_speed_mps": 0.30 if nav_profile == "field_large" else None,
                "unlimited_session_steps": (
                    nav_profile == "field_large" and trial_status is None
                ),
                "trial": trial_status,
            },
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
        navigation = status.get("navigation") or {}
        trial = navigation.get("trial")
        checks["navigation_trial_available"] = (
            trial is None
            or (
                trial.get("invalid") is not True
                and int(trial.get("remaining_steps", 0)) > 0
                and trial.get("active") is not True
            )
        )
        blocking = (
            "camera_fresh", "camera_available", "ros_worker_alive",
            "motion_action_available", "robot_mode_ok",
            "emergency_stop_available", "slam_map_not_drifting",
            "navigation_trial_available",
        )
        blocked = [key for key in blocking if not checks[key]]
        if load_search_defaults().get("spatial_provider") == "nav2":
            checks["navigation_2d_localization"] = (
                slam.get("map_kind") == "occupancy_grid_2d"
                and slam.get("frame_id") == "map" and slam.get("fresh") is True
                and slam.get("localization_valid") is True
                and slam.get("mapping_health") == "HEALTHY"
            )
            if not checks["navigation_2d_localization"]:
                blocked.append("navigation_2d_localization")
        degraded = [key for key in checks if not checks[key] and key not in blocked]
        ready = not blocked
        reason = "" if ready else "readiness failed: " + "; ".join(blocked)
        if not checks["slam_map_not_drifting"]:
            reason += " · " + str(slam.get("health_reason") or "")
        return {
            "ready": ready,
            "motion_authorized": os.getenv("GO2W_NAV2_EXECUTION_AUTHORIZED") == "1"
            and os.getenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION") == "1",
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
class _NoStoreStaticHeaders:
    """Add cache headers without wrapping streaming bodies in middleware tasks."""

    def __init__(self, app: Any) -> None:
        self.app = app

    async def __call__(self, scope: Any, receive: Any, send: Any) -> None:
        path = scope.get("path", "")
        no_store = scope.get("type") == "http" and (
            path == "/" or path.startswith("/static/")
            or path.startswith("/api/mapping/target-watch")
            or path.startswith("/api/runtime/"))
        if not no_store:
            await self.app(scope, receive, send)
            return

        async def send_headers(message: Any) -> None:
            if message["type"] == "http.response.start":
                headers = [(key, value) for key, value in message.get("headers", [])
                           if key.lower() not in (b"cache-control", b"pragma")]
                headers.extend([
                    (b"cache-control", b"no-store, no-cache, must-revalidate, max-age=0"),
                    (b"pragma", b"no-cache"),
                ])
                message = {**message, "headers": headers}
            await send(message)

        await self.app(scope, receive, send_headers)


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
        display_only_provider=lambda: config.display_only,
    )
    target_watch = TargetWatchService(
        request_path=config.target_watch_request_path,
        state_path=config.target_watch_state_path,
        evidence_dir=config.target_watch_evidence_dir_path,
        topology_path=config.mapping_topology_snapshot_path,
    )
    runtime_admin = RuntimeAdminService(
        project_root=config.project_root,
        mode=config.webui_mode,
        detector=config.active_detector,
        enabled=config.runtime_admin_enabled,
        supervisor=config.runtime_reset_supervisor,
    )

    def _display_only_403() -> JSONResponse | None:
        """只读硬门。返回 None 表示放行。

        计划书 §6.2 F：实体遥控器模式下"必须确保 DISPLAY_ONLY 下没有任何写
        运动路径"，而且"优先在 API/WS 入口做统一硬门，不要散落很多条件判断"。
        所以判定只有这一处，各路由只需在第一行调用它。``/api/llm/disable``
        是唯一的例外（见该路由的注释）：它只会关掉识别，发不出运动。
        """
        if not config.display_only:
            return None
        return JSONResponse(
            {"ok": False, "error": "display_only", "reason": DISPLAY_ONLY_REASON},
            status_code=403,
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
    app.add_middleware(_NoStoreStaticHeaders)

    if STATIC_DIR.is_dir():
        app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        html = INDEX_HTML.read_text(encoding="utf-8")
        if not config.web_3d_map_enabled:
            start = html.index("<!-- WEB_3D_START -->")
            end = html.index("<!-- WEB_3D_END -->") + len("<!-- WEB_3D_END -->")
            html = html[:start] + html[end:]
            body_class="web-2d-map" if config.web_2d_map_enabled else "web-3d-disabled"
            html = html.replace("<body>", '<body class="'+body_class+'">', 1)
            html = html.replace('<span class="light" id="light-slam"><i></i>3D Map</span>',
                '<span class="light" id="light-slam"><i></i>2D Map</span>' if config.web_2d_map_enabled else "")
            html = html.replace(
                '<script src="/static/slam_map_3d.js?v=go2w-fix-20260911"></script>',
                "",
            )
        if not config.web_2d_map_enabled or config.web_3d_map_enabled:
            start=html.index("<!-- WEB_2D_START -->")
            end=html.index("<!-- WEB_2D_END -->")+len("<!-- WEB_2D_END -->")
            html=html[:start]+html[end:]
            html=html.replace('<script src="/static/slam_map_2d.js?v=20261104"></script>',"")
        return html

    @app.get("/api/status")
    async def api_status() -> dict[str, Any]:
        status = runtime.status_snapshot()
        status["runtime_admin"] = runtime_admin.status()
        return status

    @app.get("/api/runtime/admin")
    async def api_runtime_admin() -> JSONResponse:
        return JSONResponse(runtime_admin.status())

    @app.post("/api/runtime/reset")
    async def api_runtime_reset(request: Request) -> JSONResponse:
        """Archive and restart the current runtime through a detached owner.

        Reset is rejected while any search or WebUI motion is active.  This is
        intentionally stricter than the display-only gate: clearing data is
        allowed in teleoperation mode, but it must never become an implicit
        stop/cancel primitive for an executing robot.
        """
        try:
            body = await request.json()
        except Exception:
            return JSONResponse(
                {"ok": False, "error": "invalid_json"}, status_code=422
            )
        if not isinstance(body, dict):
            return JSONResponse(
                {"ok": False, "error": "invalid_body"}, status_code=422
            )

        status = runtime.status_snapshot()
        search = status.get("search") or {}
        motion = status.get("motion") or {}
        active_search_states = {
            "starting", "running", "paused", "stopping",
            "STARTING", "RUNNING", "PAUSED", "STOPPING",
        }
        if search.get("alive") is True or search.get("state") in active_search_states:
            return JSONResponse(
                {"ok": False, "error": "search_must_be_stopped"}, status_code=409
            )
        if (motion.get("motion_in_flight") is True
                or motion.get("control_enabled") is True):
            return JSONResponse(
                {"ok": False, "error": "motion_must_be_idle"}, status_code=409
            )

        try:
            result = runtime_admin.request_reset(
                action=str(body.get("action") or ""),
                confirmation=str(body.get("confirmation") or ""),
            )
        except ValueError as exc:
            return JSONResponse(
                {"ok": False, "error": str(exc)}, status_code=422
            )
        except RuntimeError as exc:
            return JSONResponse(
                {"ok": False, "error": str(exc)}, status_code=409
            )
        return JSONResponse(result, status_code=202)

    @app.get("/api/objects")
    async def api_objects() -> dict[str, Any]:
        return runtime.analyzer.state_dict()

    @app.get("/api/mapping/state")
    async def api_mapping_state() -> JSONResponse:
        snapshot = _load_json_object(config.mapping_topology_snapshot_path)
        if snapshot is None:
            snapshot = {"session_id": None, "revision": 0,
                        "graph": {"object_topology": {"nodes": [], "edges": []}},
                        "current_detections": [], "hazard_events": [],
                        "generated_at": None}
        meta = load_map_meta(config.slam_map_meta_path)
        if meta and snapshot.get("map_epoch") and snapshot["map_epoch"] != meta["map_epoch"]:
            snapshot = {**snapshot, "graph": {"object_topology": {"nodes": [], "edges": []}},
                        "current_detections": [], "frame_available": False,
                        "map_mismatch": True}
        elif snapshot.get("map_epoch"):
            snapshot = _filter_mapping_state_to_display_objects(
                snapshot,
                load_object_snapshot(config.semantic_objects_snapshot_path),
            )
        snapshot["target_watch"] = target_watch.get_state()
        response = JSONResponse(snapshot)
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    @app.get("/api/mapping/target-watch")
    async def api_mapping_target_watch_get() -> JSONResponse:
        if not config.display_only:
            return JSONResponse(
                {"ok": False, "error": "mapping_mode_required"}, status_code=409
            )
        return JSONResponse(target_watch.get_state())

    @app.post("/api/mapping/target-watch")
    async def api_mapping_target_watch_post(request: Request) -> JSONResponse:
        if not config.display_only:
            return JSONResponse(
                {"ok": False, "error": "mapping_mode_required"}, status_code=409
            )
        try:
            body = await request.json()
        except Exception:
            return JSONResponse({"ok": False, "error": "invalid_json"}, status_code=422)
        target = body.get("target") if isinstance(body, dict) else None
        try:
            payload = await to_thread(target_watch.configure, target or "")
        except ValueError as exc:
            return JSONResponse({"ok": False, "error": str(exc)}, status_code=422)
        except RuntimeError as exc:
            return JSONResponse({"ok": False, "error": str(exc)}, status_code=409)
        return JSONResponse({"ok": True, "request": payload}, status_code=202)

    @app.delete("/api/mapping/target-watch")
    async def api_mapping_target_watch_delete() -> JSONResponse:
        if not config.display_only:
            return JSONResponse(
                {"ok": False, "error": "mapping_mode_required"}, status_code=409
            )
        try:
            payload = target_watch.clear()
        except RuntimeError as exc:
            return JSONResponse({"ok": False, "error": str(exc)}, status_code=409)
        return JSONResponse({"ok": True, "request": payload}, status_code=202)

    @app.get("/api/mapping/target-watch/evidence/{verification_id}")
    async def api_mapping_target_watch_evidence(verification_id: str):
        if not config.display_only:
            return JSONResponse(
                {"ok": False, "error": "mapping_mode_required"}, status_code=409
            )
        path = target_watch.evidence_path(verification_id)
        if path is None:
            return JSONResponse({"error": "evidence_not_found"}, status_code=404)
        response = FileResponse(path, media_type="image/jpeg")
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    @app.get("/api/mapping/evidence/{event_id}")
    async def api_mapping_evidence(event_id: str):
        snapshot = _load_json_object(config.mapping_topology_snapshot_path) or {}
        event = next((item for item in snapshot.get("hazard_events", [])
                      if item.get("hazard_event_id") == event_id), None)
        path = Path(event.get("evidence_path") or "") if event else None
        if (path is None or not path.is_file() or
                path.resolve().parent != config.mapping_topology_snapshot_path.resolve().parent):
            return JSONResponse({"error": "evidence_not_found"}, status_code=404)
        response = FileResponse(path)
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    @app.get("/api/mapping/frame")
    async def api_mapping_frame():
        snapshot = _load_json_object(config.mapping_topology_snapshot_path) or {}
        path = config.mapping_topology_snapshot_path.parent / "semantic_frame.jpg"
        meta = load_map_meta(config.slam_map_meta_path)
        if (not snapshot.get("frame_available") or not path.is_file() or
                (meta and snapshot.get("map_epoch") != meta["map_epoch"])):
            return JSONResponse({"error": "frame_not_found"}, status_code=404)
        response = FileResponse(path, media_type="image/jpeg")
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    def latest_mapping_snapshot():
        snapshot = load_slam_map_snapshot(
            config.slam_map_snapshot_path,
            config.semantic_objects_snapshot_path,
        )
        map_epoch = str(snapshot.get("map_epoch") or "")
        watch = target_watch.get_state()
        for match in watch.get("confirmed_matches", []):
            xyz = match.get("map_xyz")
            provenance = match.get("provenance") or {}
            if (
                not map_epoch
                or str(provenance.get("map_epoch") or "") != map_epoch
                or not isinstance(xyz, list)
                or len(xyz) != 3
            ):
                continue
            snapshot.setdefault("objects", []).append({
                "object_id": str(match.get("verification_id") or match.get("candidate_id")),
                "label_zh": str(
                    match.get("canonical_name_zh")
                    or watch.get("raw_query")
                    or "已确认目标"
                ),
                "x": xyz[0], "y": xyz[1], "z": xyz[2], "size": None,
                "decision": "TARGET_VERIFIED", "is_target_confirmed": True,
                "target_watch_only": True,
                "is_hazard": bool(match.get("automatic_hazard")),
                "hazard_confirmation_state": (
                    "vlm_confirmed" if match.get("automatic_hazard") else None
                ),
                "spatial_quality": match.get("spatial_quality"),
            })
        return snapshot

    @app.get("/api/slam/map3d", include_in_schema=config.web_3d_map_enabled)
    async def api_slam_map3d() -> JSONResponse:
        if not config.web_3d_map_enabled:
            return JSONResponse({"error": "web_3d_map_disabled"}, status_code=404)
        response = JSONResponse(latest_mapping_snapshot())
        response.headers["Cache-Control"] = "no-store, max-age=0"
        return response

    @app.get("/api/slam/map2d", include_in_schema=config.web_2d_map_enabled)
    async def api_slam_map2d() -> JSONResponse:
        if not config.web_2d_map_enabled:
            return JSONResponse({"error":"web_2d_map_disabled"},status_code=404)
        snapshot=latest_mapping_snapshot()
        if snapshot.get("map_kind")!="occupancy_grid_2d":
            return JSONResponse({"available":False,"reason":"waiting_for_2d_map"},
                                headers={"Cache-Control":"no-store, max-age=0"})
        payload={key:snapshot.get(key) for key in (
            "available","map_kind","frame_id","map_id","map_epoch","map_revision",
            "generated_at","health","health_reason","localization_valid","grid","pose")}
        payload["objects"]=[{key:item.get(key) for key in (
            "object_id","label_zh","x","y","decision","is_target_confirmed","is_hazard")}
            for item in snapshot.get("objects",[]) if isinstance(item,dict)]
        return JSONResponse(payload,headers={"Cache-Control":"no-store, max-age=0"})

    @app.post("/api/llm/enable")
    async def api_llm_enable() -> JSONResponse:
        # 只读模式下不许打开：这条 VLM 定时识别会与真正做落图的 RGB-D 语义
        # 侧车抢 GPU（计划书 §2.4 / §6.2 F）。
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        runtime.analyzer.set_enabled(True)
        return JSONResponse({"ok": True, "enabled": True})

    @app.post("/api/llm/disable")
    async def api_llm_disable() -> JSONResponse:
        # 唯一不挡的写入口：它只会让识别停下来，发不出任何运动请求。
        runtime.analyzer.set_enabled(False)
        return JSONResponse({"ok": True, "enabled": False})

    @app.post("/api/control/enable")
    async def api_control_enable() -> JSONResponse:
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
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
        # disable() 会走 _stop_if_motion()，可能发出停止请求 —— 只读模式下一律
        # 不挡进不出去，直接 403。
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        runtime.controller.disable(reason="user_disabled")
        runtime.owner.release(OwnerState.MANUAL)
        return JSONResponse({"ok": True})

    @app.post("/api/estop")
    async def api_estop() -> JSONResponse:
        # Estop overrides every owner: manual controller AND search session
        # both stop (plan book §42); no new motion may follow.
        # 只读模式下后端不代发急停（计划书 §6.2 F）：机器狗的急停握在操作员
        # 手里的实体遥控器上，网页这条链路本来就没接线。
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        runtime.owner.estop()
        runtime.controller.estop()
        runtime.search_service.estop_search()
        return JSONResponse({"ok": True})

    @app.post("/api/estop/reset")
    async def api_estop_reset() -> JSONResponse:
        # reset_estop() 内部会 worker.request_stop()，同样是写运动路径。
        blocked = _display_only_403()
        if blocked is not None:
            return blocked
        result = runtime.reset_estop()
        return JSONResponse(result, status_code=200 if result.get("ok") else 409)

    @app.get("/api/camera.mjpeg")
    async def api_camera_mjpeg(request: Request) -> StreamingResponse:
        # Keep the original receive channel for disconnect polling, including
        # ASGI servers that silently discard sends after a connection closes.
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
    encoded_mtime_ns = -1
    encoded_frame: bytes | None = None
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
                mtime_ns = latest.stat().st_mtime_ns
                if mtime_ns != encoded_mtime_ns:
                    raw = latest.read_bytes()
                    encoded_frame = _web_video_frame(
                        raw,
                        runtime.config.web_video_max_width,
                        runtime.config.web_video_quality,
                    )
                    encoded_mtime_ns = mtime_ns
                data = encoded_frame
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


def _web_video_frame(data: bytes, max_width: int, quality: int) -> bytes:
    """Shrink only the browser JPEG; RGB-D source and ROS frames stay intact."""
    if max_width <= 0:
        return data
    try:
        from io import BytesIO
        from PIL import Image

        image = Image.open(BytesIO(data))
        if image.width <= max_width:
            return data
        height = max(1, round(image.height * max_width / image.width))
        image = image.resize((max_width, height), Image.LANCZOS)
        output = BytesIO()
        image.save(output, format="JPEG", quality=max(30, min(85, quality)))
        return output.getvalue()
    except (OSError, ValueError):
        return data


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


# 只读模式下拒收的控制 WebSocket 消息。``hello`` 不在其中 —— 页面仍要能渲染
# 初始状态。这份集合是**唯一**的判定处，不散落到各分支里（计划书 §6.2 F）。
DISPLAY_ONLY_BLOCKED_WS_TYPES = frozenset({
    "enable_control",
    "key_down",
    "key_up",
    "heartbeat",
    "release_all",
    "estop",
})


async def _dispatch_ws_message(runtime: DemoRuntime, message: dict[str, Any]) -> None:
    msg_type = message.get("type")
    controller = runtime.controller
    if runtime.config.display_only and msg_type in DISPLAY_ONLY_BLOCKED_WS_TYPES:
        runtime.broadcaster.publish(
            {
                "type": "display_only_rejected",
                "message_type": msg_type,
                "reason": DISPLAY_ONLY_REASON,
            }
        )
        return
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
