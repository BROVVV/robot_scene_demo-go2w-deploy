"""Async semantic observation manager for VLM-only low-latency navigation.

The realtime Quick target VLM stays on the motion critical path; the full-scene
semantic VLM is submitted to a background thread (bounded to one in-flight
request) so the robot can keep moving using the latest completed semantic world
model.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import subprocess
import threading
import time
from typing import Any, Callable

from app.live_robot.semantic_observer import (
    SEMANTIC_STATUS_FRESH_FULL,
    SEMANTIC_STATUS_FRESH_QUICK,
    SemanticObservation,
    _from_payload,
)


@dataclass
class AsyncSemanticRequest:
    sequence: int
    image_path: str
    frame_id: str
    capture_timestamp: float
    robot_pose: dict[str, Any] | None
    target_profile: Any
    scene_signature: Any = None


class AsyncSemanticObservationManager:
    """Owns one background full-scene semantic request at a time.

    Rules implemented:
    * max one semantic background job in flight (``max_inflight=1``)
    * latest-wins coalescing: if a new frame arrives while one is running, only
      the newest pending frame is kept for after the current job finishes
    * stale results cannot overwrite newer results
    * results bind to the capture-time robot pose, not the current pose
    * background failures are fail-soft: they log and leave the last valid state
    """

    def __init__(
        self,
        analyze: Callable[..., dict[str, Any]],
        *,
        enabled: bool = True,
        max_inflight: int = 1,
        ttl_seconds: float = 12.0,
        translation_refresh_m: float = 0.30,
        heading_sector_deg: float = 30.0,
        initial_warmup_blocking: bool = True,
        visual_change_enabled: bool = True,
        visual_change_threshold: float | None = None,
        now: Callable[[], float] = time.time,
        event_sink: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._analyze = analyze
        self.enabled = bool(enabled)
        self.max_inflight = max(1, int(max_inflight))
        self.ttl_seconds = float(ttl_seconds)
        self.translation_refresh_m = float(translation_refresh_m)
        self.heading_sector_deg = max(1.0, float(heading_sector_deg))
        self.initial_warmup_blocking = bool(initial_warmup_blocking)
        self.visual_change_enabled = bool(visual_change_enabled)
        self.visual_change_threshold = visual_change_threshold
        self._now = now
        self._event_sink = event_sink
        self._lock = threading.Lock()
        self._latest: SemanticObservation | None = None
        self._latest_sequence = 0
        self._pending: AsyncSemanticRequest | None = None
        self._inflight = 0
        self._sequence = 0
        self._completed: list[SemanticObservation] = []
        self._last_error: str | None = None
        self._last_error_code: str | None = None
        self._last_signature: Any = None
        self._last_profile_hash: str | None = None
        self._last_submit_ts: float | None = None
        self._threads: list[threading.Thread] = []

    # -- public API ----------------------------------------------------------

    def seed(self, semantic: SemanticObservation | None) -> None:
        """Seed the manager with a synchronous first observation (warm-up)."""
        if semantic is None:
            return
        with self._lock:
            if self._latest is None or self._latest_sequence == 0:
                self._latest = semantic
                self._latest_sequence = 0
                self._last_submit_ts = float(semantic.timestamp_sec)

    def submit_if_needed(
        self,
        *,
        image_path: str,
        frame_id: str,
        capture_timestamp: float,
        robot_pose: dict[str, Any] | None,
        target_profile: Any,
        scene_signature: Any = None,
    ) -> bool:
        if not self.enabled:
            return False
        with self._lock:
            if not self._should_refresh_locked(
                robot_pose=robot_pose,
                target_profile=target_profile,
                scene_signature=scene_signature,
            ):
                return False
            request = AsyncSemanticRequest(
                sequence=0,
                image_path=image_path,
                frame_id=str(frame_id),
                capture_timestamp=float(capture_timestamp),
                robot_pose=dict(robot_pose) if robot_pose else None,
                target_profile=target_profile,
                scene_signature=scene_signature,
            )
            if self._inflight >= self.max_inflight:
                # Coalescing: keep only the newest frame worth analysing.
                self._pending = request
                self._emit_locked({
                    "event": "semantic_request_discarded_obsolete",
                    "frame_id": str(frame_id),
                    "reason": "coalesced_pending",
                })
                return False
            self._sequence += 1
            request.sequence = self._sequence
            self._inflight += 1
            self._last_submit_ts = self._now()
            self._last_signature = scene_signature
            sequence = request.sequence
        self._emit({
            "event": "semantic_request_started",
            "frame_id": str(frame_id),
            "sequence": sequence,
            "capture_timestamp": float(capture_timestamp),
        })
        thread = threading.Thread(
            target=self._run_analyze,
            args=(request,),
            name=f"async-semantic-{sequence}",
            daemon=True,
        )
        with self._lock:
            self._threads.append(thread)
        thread.start()
        return True

    def poll_completed(self) -> list[SemanticObservation]:
        with self._lock:
            completed = self._completed
            self._completed = []
            return completed

    def get_latest_completed(self) -> SemanticObservation | None:
        with self._lock:
            return self._latest

    def has_inflight(self) -> bool:
        with self._lock:
            return self._inflight > 0

    def last_error(self) -> str | None:
        with self._lock:
            return self._last_error

    def last_error_code(self) -> str | None:
        with self._lock:
            return self._last_error_code

    def close(self) -> None:
        with self._lock:
            self.enabled = False
            self._pending = None

    # -- lifecycle events (计划书 §16) --------------------------------------

    def _emit(self, event: dict[str, Any]) -> None:
        if self._event_sink is not None:
            try:
                self._event_sink(event)
            except Exception:  # noqa: BLE001 - logging must never break search
                pass

    def _emit_locked(self, event: dict[str, Any]) -> None:
        # Caller holds self._lock; route through the same guarded sink.
        self._emit(event)

    # -- internals -----------------------------------------------------------

    def _should_refresh_locked(
        self,
        *,
        robot_pose: dict[str, Any] | None,
        target_profile: Any,
        scene_signature: Any,
    ) -> bool:
        if self._latest is None:
            # 计划书 §3.4：首帧不再同步阻塞——首个 Full Semantic 也提交后台
            # single-flight，当前轮用 Quick 快路径结果继续。
            return True
        now = float(self._now())
        latest = self._latest
        age = now - float(latest.timestamp_sec)
        yaw = float((robot_pose or {}).get("yaw_deg", 0.0))
        sector = int(round(yaw / self.heading_sector_deg))
        sector_changed = latest.heading_sector is not None and latest.heading_sector != sector
        translation_delta = _translation_delta(robot_pose, latest.robot_pose)
        profile_hash = _hash_profile(target_profile)
        # Use the last submitted profile hash to avoid refresh on every call
        # when target_profile never changes.
        profile_changed = self._last_profile_hash is not None and profile_hash != self._last_profile_hash
        self._last_profile_hash = profile_hash
        visual_changed = (
            self.visual_change_enabled
            and self.visual_change_threshold is not None
            and scene_signature is not None
            and self._last_signature is not None
            and scene_signature != self._last_signature
        )
        return bool(
            age >= self.ttl_seconds
            or sector_changed
            or translation_delta >= self.translation_refresh_m
            or profile_changed
            or visual_changed
        )

    def _run_analyze(self, request: AsyncSemanticRequest) -> None:
        started = self._now()
        try:
            payload = self._analyze(
                request.image_path,
                request.target_profile,
                request_id=f"async_{request.sequence}_{request.frame_id}",
                frame_id=request.frame_id,
                robot_pose=request.robot_pose,
            )
            self._on_result(request, payload, started)
        except subprocess.TimeoutExpired as exc:
            with self._lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._last_error_code = "FULL_SEMANTIC_TIMEOUT"
            self._emit({
                "event": "semantic_timeout",
                "frame_id": str(request.frame_id),
                "sequence": request.sequence,
                "latency_ms": round(max(0.0, self._now() - started) * 1000.0, 3),
                "status": "timeout",
                "object_count": 0,
                "error": str(exc),
            })
        except Exception as exc:  # noqa: BLE001 - background must not kill robot
            error_code = str(getattr(exc, "code", "") or "FULL_SEMANTIC_ERROR")
            is_timeout = bool(
                "TIMEOUT" in error_code
                or isinstance(exc, TimeoutError)
            )
            with self._lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
                self._last_error_code = error_code
            self._emit({
                "event": "semantic_timeout" if is_timeout else "semantic_error",
                "frame_id": str(request.frame_id),
                "sequence": request.sequence,
                "latency_ms": round(max(0.0, self._now() - started) * 1000.0, 3),
                "status": "timeout" if is_timeout else "error",
                "object_count": 0,
                "error": f"{type(exc).__name__}: {exc}",
                "error_code": error_code,
            })
        finally:
            with self._lock:
                self._inflight = max(0, self._inflight - 1)
                pending = self._pending
                self._pending = None
            if pending is not None:
                self.submit_if_needed(
                    image_path=pending.image_path,
                    frame_id=pending.frame_id,
                    capture_timestamp=pending.capture_timestamp,
                    robot_pose=pending.robot_pose,
                    target_profile=pending.target_profile,
                    scene_signature=pending.scene_signature,
                )

    def _on_result(self, request: AsyncSemanticRequest, payload: dict[str, Any],
                   started: float | None = None) -> None:
        yaw = float((request.robot_pose or {}).get("yaw_deg", 0.0))
        sector = int(round(yaw / self.heading_sector_deg))
        now = float(self._now())
        if started is None:
            started = now
        try:
            semantic = _from_payload(
                payload,
                robot_pose=request.robot_pose,
                sector=sector,
                now=float(request.capture_timestamp),
            )
        except Exception:  # noqa: BLE001 - malformed result is not fatal
            with self._lock:
                self._last_error = "semantic payload normalization failed"
                self._last_error_code = "VLM_PARSE_ERROR"
            self._emit({
                "event": "semantic_error",
                "frame_id": str(request.frame_id),
                "sequence": request.sequence,
                "latency_ms": round(max(0.0, now - started) * 1000.0, 3),
                "status": "error",
                "object_count": 0,
                "error": "semantic payload normalization failed",
            })
            return
        # 计划书 §3.2 强制不变量：只有真正成功的 Full Semantic 才能进入 latest；
        # 失败/超时/降级空结果绝不覆盖 latest_success。
        if semantic.semantic_status not in {SEMANTIC_STATUS_FRESH_FULL, SEMANTIC_STATUS_FRESH_QUICK}:
            with self._lock:
                self._last_error = (
                    f"semantic payload rejected (status={semantic.semantic_status})"
                )
                self._last_error_code = semantic.semantic_error_code or "SEMANTIC_NOT_FRESH"
            self._emit({
                "event": "semantic_discarded",
                "frame_id": str(request.frame_id),
                "sequence": request.sequence,
                "latency_ms": round(max(0.0, now - started) * 1000.0, 3),
                "status": semantic.semantic_status,
                "object_count": len(semantic.objects or []),
                "error": semantic.semantic_error_detail,
            })
            return
        semantic.semantic_source_frame_id = str(request.frame_id)
        semantic.semantic_capture_timestamp = float(request.capture_timestamp)
        semantic.semantic_completed_timestamp = now
        semantic.semantic_age_ms = 0.0
        semantic.semantic_source_pose = (
            dict(request.robot_pose) if request.robot_pose else None
        )
        with self._lock:
            if request.sequence <= self._latest_sequence:
                # Stale result; do not overwrite newer semantic state.
                self._emit({
                    "event": "semantic_discarded",
                    "frame_id": str(request.frame_id),
                    "sequence": request.sequence,
                    "latency_ms": round(max(0.0, now - started) * 1000.0, 3),
                    "status": "stale_result",
                    "object_count": len(semantic.objects or []),
                    "error": "newer semantic result already applied",
                })
                return
            self._latest = semantic
            self._latest_sequence = request.sequence
            self._completed.append(semantic)
        self._emit({
            "event": "semantic_result_applied",
            "frame_id": str(request.frame_id),
            "sequence": request.sequence,
            "latency_ms": round(max(0.0, now - started) * 1000.0, 3),
            "status": semantic.semantic_status,
            "object_count": len(semantic.objects or []),
            "source": semantic.source,
        })


def _translation_delta(
    first: dict[str, Any] | None,
    second: dict[str, Any] | None,
) -> float:
    if first is None and second is None:
        return 0.0
    if first is None or second is None:
        return math.inf
    try:
        return math.hypot(
            float(first.get("x", 0.0)) - float(second.get("x", 0.0)),
            float(first.get("y", 0.0)) - float(second.get("y", 0.0)),
        )
    except (TypeError, ValueError):
        return math.inf


def _hash_profile(profile: Any) -> str:
    try:
        if hasattr(profile, "to_dict"):
            value = profile.to_dict()
        else:
            value = repr(profile)
        return hashlib.sha1(repr(value).encode("utf-8")).hexdigest()
    except Exception:
        return ""


def compute_scene_signature(image_path: str) -> str:
    """Very cheap file-level scene signature.

    A real visual-change detector (histogram/SSIM) can replace this later; file
    size + mtime is enough to avoid submitting the exact same image repeatedly.
    """
    try:
        from pathlib import Path

        path = Path(image_path)
        stat = path.stat()
        return f"{stat.st_size}:{stat.st_mtime_ns}"
    except OSError:
        return ""
