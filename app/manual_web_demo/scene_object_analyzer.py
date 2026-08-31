"""Asynchronous SiliconFlow scene-object analyzer for the manual web demo.

Three responsibilities (plan book §23–§29, §45):

* parse the model's JSON into a stable object table (pure, unit-testable);
* reuse the project's existing SiliconFlow client/config — never a second API
  key, endpoint or model account — to list the main visible objects;
* run a single-worker scheduler that never stacks concurrent inferences and
  pauses while the camera is stale.
"""

from __future__ import annotations

import base64
from io import BytesIO
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.models import SceneObject, SceneObjectState

CONFIDENCE_VALUES = ("high", "medium", "low")


# --------------------------------------------------------------------------- #
# Pure parser                                                                  #
# --------------------------------------------------------------------------- #
def normalize_confidence(value: Any) -> str:
    text = str(value or "").strip().lower()
    return text if text in CONFIDENCE_VALUES else "medium"


def _normalize_count(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if value >= 0 else None
    if isinstance(value, float):
        return int(value) if value.is_integer() and value >= 0 else None
    if isinstance(value, str):
        text = value.strip()
        if not text.isdigit():
            return None
        return int(text)
    return None


def _parse_scene_object(item: Any) -> SceneObject | None:
    if not isinstance(item, dict):
        return None
    name_zh = str(item.get("name_zh") or item.get("name") or "").strip()
    name_en = str(item.get("name_en") or "").strip() or None
    if not name_zh and not name_en:
        return None
    position = (
        str(item.get("position") or item.get("position_zh") or "").strip() or None
    )
    return SceneObject(
        name_zh=name_zh,
        name_en=name_en,
        count=_normalize_count(item.get("count")),
        position=position,
        confidence=normalize_confidence(item.get("confidence")),
    )


def _dedup_key(obj: SceneObject) -> str:
    key = (obj.name_zh or "").lower() or (obj.name_en or "").lower()
    return key.strip()


def parse_scene_objects_payload(payload: Any) -> tuple[list[SceneObject], str | None]:
    """Extract a normalized object table from the model's parsed JSON dict.

    Raises ValueError for structurally invalid payloads (plan book §46).
    """
    if not isinstance(payload, dict):
        raise ValueError("scene payload is not a JSON object")
    raw_objects = payload.get("objects")
    if not isinstance(raw_objects, list):
        raise ValueError("scene payload has no objects array")
    scene_summary = str(
        payload.get("scene_summary") or payload.get("scene_summary_zh") or ""
    ).strip() or None
    objects: list[SceneObject] = []
    seen: set[str] = set()
    for item in raw_objects:
        obj = _parse_scene_object(item)
        if obj is None:
            continue
        key = _dedup_key(obj)
        if not key or key in seen:
            continue
        seen.add(key)
        objects.append(obj)
    return objects, scene_summary


# --------------------------------------------------------------------------- #
# SiliconFlow reuse (plan book §23 / §24 / §39)                                #
# --------------------------------------------------------------------------- #
_SCENE_OBJECT_SYSTEM_PROMPT = (
    "你是机器狗第一人称视觉场景识别模块。\n"
    "只识别当前图像中清晰可见的主要物体。\n"
    "不要推测画面外物体，不要根据常识补充不存在的东西。\n"
    "忽略非常小、模糊、无法确认的物体。相同物体尽量合并。\n"
    "只输出严格 JSON，不要 Markdown，不要任何其他文字。"
)

_SCENE_OBJECT_USER_PROMPT = (
    "请列出当前画面中清晰可见的主要物体。返回 JSON：\n"
    "{\n"
    '  "scene_summary": "一句话场景摘要",\n'
    '  "objects": [\n'
    "    {\n"
    '      "name_zh": "椅子",\n'
    '      "name_en": "chair",\n'
    '      "count": 2,\n'
    '      "position": "左侧和中间",\n'
    '      "confidence": "high"\n'
    "    }\n"
    "  ]\n"
    "}\n"
    "约束：\n"
    "- name_zh 用中文，name_en 用英文小写。\n"
    "- count 只写可确认的数量；无法确认时写 null。不要猜数。\n"
    "- position 用中文描述物体在画面中的大致位置（左/中/右/上/下等）。\n"
    "- confidence 只能是 high / medium / low 之一。\n"
    "- 只输出 JSON。"
)


def analyze_scene_objects(
    image_path: str | Path,
    settings: Any | None = None,
) -> dict[str, Any]:
    """Reuse the project SiliconFlow client/config to list main objects.

    Only the prompt is demo-specific (plan book §24). API key, base URL, model,
    timeout and image sizing all come from the existing ``app.config`` /
    ``app.llm_clients.siliconflow_client`` chain.
    """
    from app.llm_clients.siliconflow_client import (
        SiliconFlowVisionClient,
        _load_resized_image_bytes,
    )
    from app.utils.json_utils import extract_json_from_text

    client = SiliconFlowVisionClient(settings=settings)
    demo_settings = client.settings
    image_bytes, mime_type = _load_resized_image_bytes(
        Path(image_path), demo_settings.image_max_side
    )
    image_data_url = (
        f"data:{mime_type};base64,"
        + base64.b64encode(image_bytes).decode("ascii")
    )
    response = client.client.chat.completions.create(
        model=demo_settings.vision_model,
        messages=[
            {"role": "system", "content": _SCENE_OBJECT_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": _SCENE_OBJECT_USER_PROMPT},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": image_data_url,
                            "detail": demo_settings.image_detail,
                        },
                    },
                ],
            },
        ],
        temperature=0.1,
        max_tokens=demo_settings.siliconflow_max_tokens,
    )
    raw_content = client._extract_response_text(response)
    data = extract_json_from_text(raw_content)
    objects, scene_summary = parse_scene_objects_payload(data)
    return {
        "objects": objects,
        "scene_summary": scene_summary,
        "model": demo_settings.vision_model,
    }


# --------------------------------------------------------------------------- #
# Single-worker scheduler                                                      #
# --------------------------------------------------------------------------- #
class SceneObjectAnalyzer:
    """Runs at most one SiliconFlow inference at a time, every N seconds."""

    def __init__(
        self,
        *,
        config: ManualDemoSettings,
        frame_provider: Callable[[], str | None],
        camera_fresh_provider: Callable[[], bool],
        analyzer_fn: Callable[[str], dict[str, Any]] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._config = config
        self._frame_provider = frame_provider
        self._camera_fresh = camera_fresh_provider
        self._analyzer_fn = analyzer_fn or analyze_scene_objects
        self._clock = clock
        self._state = SceneObjectState()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # -inf so the very first cycle is never skipped by the interval gate
        # (real monotonic clocks are already seconds since boot).
        self._last_started_at: float = float("-inf")
        self._enabled: bool = bool(config.llm_enabled)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._loop, name="scene-object-analyzer", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def set_enabled(self, value: bool) -> None:
        """Runtime switch for LLM analysis (UI toggle). Keeps last results."""
        with self._lock:
            self._enabled = bool(value)

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled

    def state_dict(self) -> dict[str, Any]:
        with self._lock:
            result = self._state.to_dict()
            result["enabled"] = self._enabled
            return result

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.run_cycle(now=self._clock())
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._state.status = "error"
                    self._state.error = f"scheduler error: {exc}"
                    # Keep the last successful table.
            self._stop.wait(0.5)

    def run_cycle(self, *, now: float) -> str:
        """One scheduler tick. Returns a short reason string for tests."""
        with self._lock:
            if self._state.status == "running":
                return "skip_running"
            if not self._enabled:
                return "skip_disabled"
        if not self._camera_fresh():
            return "skip_camera_stale"
        if now - self._last_started_at < self._config.llm_interval_seconds:
            return "skip_interval"
        frame_path = self._frame_provider()
        if not frame_path or not os.path.isfile(frame_path):
            return "skip_no_frame"
        try:
            snapshot_bytes = Path(frame_path).read_bytes()
        except OSError:
            return "skip_no_frame"
        if not snapshot_bytes:
            return "skip_no_frame"
        self._start_analysis(snapshot_bytes, now)
        return "started"

    def _start_analysis(self, snapshot_bytes: bytes, started_at: float) -> None:
        with self._lock:
            self._state.status = "running"
            self._state.error = None
            self._state.analysis_started_at = started_at
            self._last_started_at = started_at
        worker = threading.Thread(
            target=self._run_inference,
            args=(snapshot_bytes, started_at),
            daemon=True,
            name="scene-object-inference",
        )
        worker.start()

    def _run_inference(self, snapshot_bytes: bytes, started_at: float) -> None:
        snapshot_path: Path | None = None
        try:
            runtime_dir = self._config.runtime_dir_path
            runtime_dir.mkdir(parents=True, exist_ok=True)
            snapshot_path = runtime_dir / "llm_snapshot.jpg"
            snapshot_path.write_bytes(snapshot_bytes)
            if self._config.save_analysis_frames:
                frames_dir = self._config.analysis_frames_dir_path
                frames_dir.mkdir(parents=True, exist_ok=True)
                (frames_dir / f"frame_{started_at:.3f}.jpg").write_bytes(
                    snapshot_bytes
                )
            result = self._analyzer_fn(str(snapshot_path))
            objects = list(result.get("objects") or [])
            if self._config.llm_hide_low_confidence:
                objects = [
                    obj for obj in objects if obj.confidence != "low"
                ]
            with self._lock:
                self._state.objects = objects
                self._state.scene_summary = result.get("scene_summary")
                self._state.model = result.get("model")
                self._state.frame_timestamp = started_at
                self._state.analysis_finished_at = self._clock()
                self._state.status = "ok"
                self._state.error = None
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._state.status = "error"
                self._state.error = str(exc)
                # Keep the last successful table on failure (plan book §28).
        finally:
            if snapshot_path is not None:
                try:
                    snapshot_path.unlink(missing_ok=True)
                except OSError:
                    pass
