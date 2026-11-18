"""Asynchronous VLM naming for detector classes without a Chinese display name.

The detector/inference path must never wait for a cloud or on-robot VLM.  This
module therefore owns a small FIFO, a single background worker and an audit
record for accepted instance-level names.  Results are never reused as a global
raw-class dictionary; the semantic mapper submits evidence and polls results.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from typing import Any, Callable, Mapping


_CJK_RE = re.compile(r"[\u3400-\u9fff]")
_GENERIC_NAMES = frozenset({
    "物体", "目标物体", "未知物体", "不明物体", "未分类物体", "无法判断",
    "无法识别", "其他物体", "物品", "东西", "目标",
})


def normalize_raw_label(value: Any) -> str:
    return " ".join(str(value or "").strip().casefold().replace("_", " ").split())


@dataclass(frozen=True)
class UnknownObjectVLMConfig:
    enabled: bool = True
    socket_path: str = "runtime/go2w/siliconflow_vlm.sock"
    cache_path: str = "runtime/go2w/unknown_object_vlm_labels.json"
    evidence_dir: str = "outputs/remote_control_mapping/runtime/unknown_object_vlm_evidence"
    max_pending: int = 16
    max_calls_per_minute: int = 3
    retry_cooldown_s: float = 120.0
    client_timeout_s: float = 120.0
    min_confidence: float = 0.65
    evidence_max_files: int = 50

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any] | None) -> "UnknownObjectVLMConfig":
        value = dict(raw or {})
        result = cls(
            enabled=bool(value.get("enabled", True)),
            socket_path=str(value.get("socket_path") or cls.socket_path),
            cache_path=str(value.get("cache_path") or cls.cache_path),
            evidence_dir=str(value.get("evidence_dir") or cls.evidence_dir),
            max_pending=int(value.get("max_pending", cls.max_pending)),
            max_calls_per_minute=int(
                value.get("max_calls_per_minute", cls.max_calls_per_minute)
            ),
            retry_cooldown_s=float(
                value.get("retry_cooldown_s", cls.retry_cooldown_s)
            ),
            client_timeout_s=float(
                value.get("client_timeout_s", cls.client_timeout_s)
            ),
            min_confidence=float(value.get("min_confidence", cls.min_confidence)),
            evidence_max_files=int(
                value.get("evidence_max_files", cls.evidence_max_files)
            ),
        )
        result.validate()
        return result

    def validate(self) -> None:
        if not 1 <= self.max_pending <= 100:
            raise ValueError("unknown_object_vlm.max_pending must be 1..100")
        if not 1 <= self.max_calls_per_minute <= 60:
            raise ValueError("unknown_object_vlm.max_calls_per_minute must be 1..60")
        if self.retry_cooldown_s < 0:
            raise ValueError("unknown_object_vlm.retry_cooldown_s must be non-negative")
        if self.client_timeout_s <= 0:
            raise ValueError("unknown_object_vlm.client_timeout_s must be positive")
        if not 0 <= self.min_confidence <= 1:
            raise ValueError("unknown_object_vlm.min_confidence must be in [0,1]")
        if not 1 <= self.evidence_max_files <= 500:
            raise ValueError("unknown_object_vlm.evidence_max_files must be 1..500")


@dataclass(frozen=True)
class UnknownLabelRequest:
    review_key: str
    raw_label: str
    frame_id: str
    object_id: str
    image_path: str
    bbox: tuple[float, float, float, float]
    submitted_at: float


@dataclass(frozen=True)
class UnknownLabelResult:
    review_key: str
    raw_label: str
    frame_id: str
    object_id: str
    label_zh: str | None
    confidence: float
    reason_zh: str
    provider: str
    latency_ms: float
    error: str | None = None

    @property
    def accepted(self) -> bool:
        return bool(self.label_zh) and self.error is None


def accepted_chinese_name(payload: Mapping[str, Any], *, min_confidence: float
                          ) -> tuple[str | None, float, str]:
    """Return a conservative Chinese object name from a VLM payload."""
    name = " ".join(str(payload.get("object_name_zh") or "").strip().split())
    try:
        confidence = max(0.0, min(1.0, float(payload.get("confidence", 0.0))))
    except (TypeError, ValueError):
        confidence = 0.0
    reason = str(payload.get("reason_zh") or "").strip()
    if payload.get("is_target") is not True:
        return None, confidence, reason or "VLM 未确认框内存在可命名物体"
    if confidence < float(min_confidence):
        return None, confidence, reason or "VLM 置信度不足"
    if not name or not _CJK_RE.search(name) or name in _GENERIC_NAMES:
        return None, confidence, reason or "VLM 未给出具体中文物体名"
    if len(name) > 40:
        return None, confidence, reason or "VLM 名称过长"
    return name, confidence, reason


class AsyncUnknownObjectLabeler:
    """Single-worker FIFO with per-map-object deduplication and audit records."""

    def __init__(
        self,
        verify: Callable[[UnknownLabelRequest], Mapping[str, Any]],
        config: UnknownObjectVLMConfig,
        *,
        cache_path: str | Path,
        event_sink: Callable[[dict[str, Any]], None] | None = None,
        now: Callable[[], float] = time.monotonic,
    ) -> None:
        self.verify = verify
        self.config = config
        self.cache_path = Path(cache_path).resolve()
        self.event_sink = event_sink
        self._now = now
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._queue: deque[UnknownLabelRequest] = deque()
        self._pending_labels: set[str] = set()
        self._completed: deque[UnknownLabelResult] = deque()
        self._last_attempt: dict[str, float] = {}
        self._call_times: deque[float] = deque()
        # VLM names are object-instance evidence, never a global raw-class
        # dictionary.  YOLOE can call the same chair "patient" in one frame;
        # reusing patient -> chair across scenes would corrupt real patients.
        self._cache: dict[str, dict[str, Any]] = {}
        self._thread: threading.Thread | None = None
        if config.enabled:
            self._thread = threading.Thread(
                target=self._run, name="unknown-object-vlm", daemon=True
            )
            self._thread.start()

    def cached_object_label(self, review_key: Any) -> str | None:
        key = str(review_key or "").strip()
        with self._lock:
            item = self._cache.get(key) or {}
            value = str(item.get("label_zh") or "").strip()
        return value or None

    def needs_review(self, review_key: Any) -> bool:
        key = str(review_key or "").strip()
        now = float(self._now())
        with self._lock:
            previous = self._last_attempt.get(key)
            return bool(
                self.config.enabled
                and key
                and key not in self._cache
                and key not in self._pending_labels
                and len(self._queue) < self.config.max_pending
                and (
                    previous is None
                    or now - previous >= self.config.retry_cooldown_s
                )
            )

    def submit(
        self,
        *,
        review_key: str,
        raw_label: str,
        frame_id: str,
        object_id: str,
        image_path: str,
        bbox: Any,
    ) -> bool:
        if not self.config.enabled:
            return False
        key = str(review_key or "").strip()
        normalized_raw = normalize_raw_label(raw_label)
        try:
            box = tuple(float(value) for value in bbox)
        except (TypeError, ValueError):
            return False
        if (
            not key or not normalized_raw or len(box) != 4
            or box[2] <= box[0] or box[3] <= box[1]
        ):
            return False
        now = float(self._now())
        with self._lock:
            if key in self._cache or key in self._pending_labels:
                return False
            previous = self._last_attempt.get(key)
            if previous is not None and now - previous < self.config.retry_cooldown_s:
                return False
            if len(self._queue) >= self.config.max_pending:
                return False
            request = UnknownLabelRequest(
                review_key=key,
                raw_label=normalized_raw,
                frame_id=str(frame_id),
                object_id=str(object_id),
                image_path=str(image_path),
                bbox=(box[0], box[1], box[2], box[3]),
                submitted_at=now,
            )
            self._queue.append(request)
            self._pending_labels.add(key)
        self._emit({
            "event": "unknown_object_vlm_queued",
            "review_key": key,
            "raw_label": normalized_raw,
            "frame_id": str(frame_id),
            "object_id": str(object_id),
        })
        self._wake.set()
        return True

    def poll_completed(self) -> list[UnknownLabelResult]:
        with self._lock:
            values = list(self._completed)
            self._completed.clear()
        return values

    def stats(self) -> dict[str, int]:
        with self._lock:
            return {
                "pending": len(self._pending_labels),
                "queued": len(self._queue),
                "cache_entries": len(self._cache),
            }

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=0.2)

    def _run(self) -> None:
        while not self._stop.is_set():
            request = self._next_request()
            if request is None:
                self._wake.wait(0.5)
                self._wake.clear()
                continue
            if not self._consume_rate_slot():
                with self._lock:
                    self._queue.appendleft(request)
                self._stop.wait(1.0)
                continue
            started = float(self._now())
            self._emit({
                "event": "unknown_object_vlm_started",
                "review_key": request.review_key,
                "raw_label": request.raw_label,
                "frame_id": request.frame_id,
                "object_id": request.object_id,
            })
            try:
                payload = dict(self.verify(request) or {})
                name, confidence, reason = accepted_chinese_name(
                    payload, min_confidence=self.config.min_confidence
                )
                error = None
                provider = str(payload.get("provider") or payload.get("_provider") or "")
            except Exception as exc:  # noqa: BLE001 - background errors are data
                name, confidence, reason = None, 0.0, ""
                error = f"{type(exc).__name__}: {exc}"
                provider = ""
            latency_ms = round(max(0.0, float(self._now()) - started) * 1000.0, 3)
            result = UnknownLabelResult(
                review_key=request.review_key,
                raw_label=request.raw_label,
                frame_id=request.frame_id,
                object_id=request.object_id,
                label_zh=name,
                confidence=confidence,
                reason_zh=reason,
                provider=provider,
                latency_ms=latency_ms,
                error=error,
            )
            with self._lock:
                self._pending_labels.discard(request.review_key)
                self._last_attempt[request.review_key] = float(self._now())
                if result.accepted:
                    self._cache[request.review_key] = {
                        "review_key": request.review_key,
                        "raw_label": request.raw_label,
                        "object_id": request.object_id,
                        "label_zh": result.label_zh,
                        "confidence": result.confidence,
                        "provider": result.provider,
                        "updated_at": time.time(),
                    }
                self._completed.append(result)
            if result.accepted:
                self._write_cache()
            self._emit({
                "event": (
                    "unknown_object_vlm_completed"
                    if result.accepted else "unknown_object_vlm_unresolved"
                ),
                "review_key": result.review_key,
                "raw_label": result.raw_label,
                "frame_id": result.frame_id,
                "object_id": result.object_id,
                "label_zh": result.label_zh,
                "confidence": result.confidence,
                "provider": result.provider,
                "latency_ms": result.latency_ms,
                "reason_zh": result.reason_zh,
                "error": result.error,
            })

    def _next_request(self) -> UnknownLabelRequest | None:
        with self._lock:
            return self._queue.popleft() if self._queue else None

    def _consume_rate_slot(self) -> bool:
        now = float(self._now())
        with self._lock:
            while self._call_times and now - self._call_times[0] >= 60.0:
                self._call_times.popleft()
            if len(self._call_times) >= self.config.max_calls_per_minute:
                return False
            self._call_times.append(now)
        return True

    def _write_cache(self) -> None:
        with self._lock:
            payload = {
                "schema_version": 2,
                "scope": "current_mapper_process_only",
                "reviews": dict(self._cache),
            }
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False)
        fd, temporary = tempfile.mkstemp(
            prefix=f".{self.cache_path.name}.", suffix=".tmp", dir=self.cache_path.parent
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self.cache_path)
        finally:
            try:
                Path(temporary).unlink()
            except OSError:
                pass

    def _emit(self, event: dict[str, Any]) -> None:
        if self.event_sink is not None:
            try:
                self.event_sink(dict(event))
            except Exception:
                pass
