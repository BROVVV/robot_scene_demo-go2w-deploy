"""Versioned data contracts for the search replay recording package.

Everything persisted here is plain JSON / JSONL / NPZ.  Two invariants matter
for evidence quality and are enforced in one place:

* JSON never carries NaN/Infinity (``json_safe`` scrubs and callers count it).
* every timestamp that orders playback is a host ``time.monotonic()`` value
  (``t``); the wall clock is stored next to it as ``wall`` for display and for
  cross-process sanity checks, never for duration arithmetic.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

SCHEMA_VERSION = "go2w_search_replay_v1"
REPLAY_DIRNAME = "replay"

# 回放状态机（独立于搜索状态）：记录 → 收尾 → 排队 → 渲染 → 成品。
DISABLED = "DISABLED"
STARTING = "STARTING"
RECORDING = "RECORDING"
FINALIZING = "FINALIZING"
QUEUED = "QUEUED"
RENDERING = "RENDERING"
READY = "READY"
PARTIAL = "PARTIAL"
FAILED = "FAILED"

TERMINAL_REPLAY_STATUSES = frozenset({READY, PARTIAL, FAILED})
ACTIVE_REPLAY_STATUSES = frozenset(
    {STARTING, RECORDING, FINALIZING, QUEUED, RENDERING}
)
PLAYABLE_REPLAY_STATUSES = frozenset({READY, PARTIAL})

DEFAULT_SPEEDS = (1, 5, 10, 20)

# 判定自由/占用：Nav2 标准 OccupancyGrid 语义（-1 未知，0=自由，100=占用）。
OCCUPIED_THRESHOLD = 50
UNKNOWN_VALUE = -1


def finite_float(value: Any, default: float | None = None) -> float | None:
    """Return a finite float or ``default`` — never NaN/Infinity."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def json_safe(value: Any) -> Any:
    """Recursively replace non-finite floats with ``None``.

    ``json.dump`` would otherwise emit ``NaN``/``Infinity``, which is not JSON
    and breaks every strict reader (browser ``JSON.parse`` included).
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None or not raw.strip():
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    value = finite_float(os.getenv(name), None)
    return default if value is None else value


def _env_int(name: str, default: int) -> int:
    value = finite_float(os.getenv(name), None)
    return default if value is None else int(value)


def _env_str(name: str, default: str) -> str:
    raw = os.getenv(name)
    return raw.strip() if raw and raw.strip() else default


@dataclass(frozen=True)
class ReplayConfig:
    """Defaults and limits for recording/rendering.

    Values come from ``configs/go2w/search_replay.yaml`` first, then explicit
    ``GO2W_SEARCH_REPLAY_*`` environment overrides (the launcher exports those).
    """

    enabled: bool = True
    root: str = "outputs/search_replay"
    width: int = 1920
    height: int = 1080
    fps: int = 30
    default_speed: int = 10
    key_speed: float = 1.0
    speeds: tuple[int, ...] = DEFAULT_SPEEDS
    pose_hz: float = 10.0
    camera_fps: float = 5.0
    map_max_hz: float = 1.0
    camera_url: str = ""
    map_topic: str = "/map"
    map_frame: str = "map"
    base_frame: str = "base_link"
    odom_topic: str = "/go2w/odom/fused"
    localization_topic: str = "/go2w/nav/localization_status"
    session_soft_bytes: int = 8 * 1024**3
    global_soft_bytes: int = 30 * 1024**3
    min_free_bytes: int = 2 * 1024**3
    end_hold_seconds: float = 3.0
    key_pad_seconds: float = 6.0
    chapter_seconds: float = 1800.0
    encoder_threads: int = 2
    encoder_crf: int = 20
    encoder_preset: str = "veryfast"
    capture_ready_timeout: float = 2.0
    capture_stop_timeout: float = 8.0
    render_queue_capacity: int = 16
    event_queue_capacity: int = 4096
    max_pose_jump_m: float = 0.6
    max_pose_jump_window_s: float = 0.5
    max_odom_speed_mps: float = 1.5
    font_path: str = (
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    )

    def with_overrides(self, **changes: Any) -> "ReplayConfig":
        values = {**self.__dict__}
        for key, value in changes.items():
            if value is not None and hasattr(self, key):
                values[key] = value
        return ReplayConfig(**values)


def _config_from_mapping(raw: Mapping[str, Any]) -> dict[str, Any]:
    """Accept only known keys; keep the yaml small and forgiving."""
    allowed = {
        "enabled", "root", "width", "height", "fps", "default_speed",
        "key_speed", "speeds", "pose_hz", "camera_fps", "map_max_hz",
        "camera_url", "map_topic", "map_frame", "base_frame", "odom_topic",
        "localization_topic", "session_soft_bytes", "global_soft_bytes",
        "min_free_bytes", "end_hold_seconds", "key_pad_seconds",
        "chapter_seconds", "encoder_threads", "encoder_crf", "encoder_preset",
        "capture_ready_timeout", "capture_stop_timeout",
        "render_queue_capacity", "event_queue_capacity", "max_pose_jump_m",
        "max_pose_jump_window_s", "max_odom_speed_mps", "font_path",
    }
    return {key: value for key, value in raw.items() if key in allowed}


def load_replay_config(
    project_root: str | Path | None = None,
    config_path: str | Path | None = None,
) -> ReplayConfig:
    """Load yaml defaults, then environment overrides, then explicit disable."""
    root = Path(project_root) if project_root else Path(__file__).resolve().parents[2]
    path = Path(config_path) if config_path else root / "configs/go2w/search_replay.yaml"
    values: dict[str, Any] = {}
    if path.is_file():
        try:
            import yaml

            raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if isinstance(raw, Mapping):
                values.update(_config_from_mapping(raw))
        except (OSError, ValueError):
            # A broken config must not disable replay silently in production;
            # the defaults below are still safe and the collector reports
            # every derived setting in its capture_status.
            pass
    if "speeds" in values and values["speeds"] is not None:
        try:
            values["speeds"] = tuple(
                sorted({int(item) for item in values["speeds"] if int(item) >= 1})
            ) or DEFAULT_SPEEDS
        except (TypeError, ValueError):
            values.pop("speeds", None)
    config = ReplayConfig(**values)
    config = config.with_overrides(
        enabled=_env_bool("GO2W_SEARCH_REPLAY_ENABLED", config.enabled),
        root=_env_str("GO2W_SEARCH_REPLAY_ROOT", config.root),
        camera_url=_env_str("GO2W_SEARCH_REPLAY_CAMERA_URL", config.camera_url),
        width=_env_int("GO2W_SEARCH_REPLAY_WIDTH", config.width),
        height=_env_int("GO2W_SEARCH_REPLAY_HEIGHT", config.height),
        fps=_env_int("GO2W_SEARCH_REPLAY_FPS", config.fps),
        default_speed=_env_int(
            "GO2W_SEARCH_REPLAY_DEFAULT_SPEED", config.default_speed
        ),
        pose_hz=_env_float("GO2W_SEARCH_REPLAY_POSE_HZ", config.pose_hz),
        camera_fps=_env_float("GO2W_SEARCH_REPLAY_CAMERA_FPS", config.camera_fps),
        map_topic=_env_str("GO2W_SEARCH_REPLAY_MAP_TOPIC", config.map_topic),
        map_frame=_env_str("GO2W_SEARCH_REPLAY_MAP_FRAME", config.map_frame),
        base_frame=_env_str("GO2W_SEARCH_REPLAY_BASE_FRAME", config.base_frame),
        odom_topic=_env_str("GO2W_SEARCH_REPLAY_ODOM_TOPIC", config.odom_topic),
        localization_topic=_env_str(
            "GO2W_SEARCH_REPLAY_LOCALIZATION_TOPIC", config.localization_topic
        ),
        global_soft_bytes=_env_int(
            "GO2W_SEARCH_REPLAY_GLOBAL_SOFT_BYTES", config.global_soft_bytes
        ),
    )
    # 硬性边界：尺寸必须是偶数（yuv420p），fps/尺寸过小会让成品不可用。
    width = max(320, int(config.width) // 2 * 2)
    height = max(180, int(config.height) // 2 * 2)
    fps = max(5, int(config.fps))
    return config.with_overrides(width=width, height=height, fps=fps)


@dataclass(frozen=True)
class SegmentInfo:
    """One coordinate/continuity segment of the recorded run."""

    segment_id: str
    start_t: float
    end_t: float | None = None
    reason: str = "start"

    def to_dict(self) -> dict[str, Any]:
        return {
            "segment_id": self.segment_id,
            "start_t": self.start_t,
            "end_t": self.end_t,
            "reason": self.reason,
        }


@dataclass
class MapSnapshot:
    """One unique map version; identical content is stored once."""

    content_hash: str
    path: Path
    t: float
    wall: float
    stamp: float | None
    frame_id: str
    width: int
    height: int
    resolution: float
    origin: tuple[float, float, float, float, float, float, float]
    grid: Any = None  # numpy int8 ndarray, filled lazily

    def meta(self) -> dict[str, Any]:
        return {
            "content_hash": self.content_hash,
            "frame_id": self.frame_id,
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": list(self.origin),
            "stamp": self.stamp,
            "t": self.t,
            "wall": self.wall,
        }


@dataclass
class PoseRow:
    t: float
    wall: float
    x: float
    y: float
    yaw: float
    frame_id: str = "map"
    segment_id: str = "seg0"
    valid: bool = True
    source: str = "tf"
    stamp: float | None = None
    quality: str = "tf_lookup"


@dataclass
class OdomRow:
    t: float
    wall: float
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    stamp: float | None = None
    frame_id: str = "odom_fused"


@dataclass
class FrameRow:
    t: float
    wall: float
    seq: int
    path: str
    width: int | None = None
    height: int | None = None
    quality: str = "http_receive_time"
    gap_s: float | None = None


@dataclass
class EventRow:
    t: float
    wall: float
    event_type: str
    payload: dict[str, Any] = field(default_factory=dict)
    cycle: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "t": self.t,
            "wall": self.wall,
            "event_type": self.event_type,
            "payload": json_safe(self.payload),
            "cycle": self.cycle,
        }


def coerce_speed(value: Any, config: ReplayConfig) -> int:
    """Only the declared playback speeds are accepted."""
    try:
        speed = int(value)
    except (TypeError, ValueError):
        speed = int(config.default_speed)
    allowed = tuple(config.speeds) or DEFAULT_SPEEDS
    if speed in allowed:
        return speed
    # 取最接近的允许值（前端可能传字符串或过期值）。
    return min(allowed, key=lambda item: abs(item - speed))


def normalize_speed_list(values: Iterable[Any]) -> tuple[int, ...]:
    result: set[int] = set()
    for value in values:
        try:
            number = int(value)
        except (TypeError, ValueError):
            continue
        if number >= 1:
            result.add(number)
    return tuple(sorted(result)) or DEFAULT_SPEEDS
