"""RealSense D435 HTTP RGB-D source using the atomic /rgbd endpoints."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

from app.perception.rgbd_source import RGBDFrame, RGBDFrameUnavailable


# Install a proxy-free default opener for this ROS-side sensor process, while
# keeping calls routed through urllib.request.urlopen so tests and operators
# can still inject a transport at the normal seam.
_DIRECT_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
urllib.request.install_opener(_DIRECT_OPENER)

# host_timestamp 合法性窗口（Unix 秒；2026 年 ≈ 1.78e9）。
_HOST_TIMESTAMP_MIN_S = 1.4e9
_HOST_TIMESTAMP_MAX_S = 2.2e9


def _resolve_capture_timestamp(
    host_timestamp: Any, *, default: float
) -> tuple[float, str]:
    """计划书 §11.1：优先使用服务端采集时间；非法/缺失回退接收时间。"""
    try:
        value = float(host_timestamp)
    except (TypeError, ValueError):
        value = 0.0
    if _HOST_TIMESTAMP_MIN_S <= value <= _HOST_TIMESTAMP_MAX_S:
        return value, "host_timestamp"
    return float(default), "receive_time"


class RealSenseHTTPRGBDSource:
    """Reads atomic RGB-D frames from the D435 HTTP stream service.

    The service (``scripts/go2w/realsense_stream.py``) exposes
    ``/rgbd/latest.json`` plus immutable ``/rgbd/frame/<id>/color.jpg`` and
    ``/rgbd/frame/<id>/depth.png`` endpoints.  This source downloads each frame
    once into a local cache directory so OpenCV / vision workers never race the
    remote buffer.
    """

    def __init__(
        self,
        base_url: str = "http://192.168.123.18:8080",
        *,
        cache_dir: str | Path = "runtime/go2w/rgbd_cache",
        max_cached_frames: int = 64,
        request_timeout: float = 5.0,
        max_age_seconds: float = 5.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.max_cached_frames = max(8, int(max_cached_frames))
        self.request_timeout = float(request_timeout)
        self.max_age_seconds = float(max_age_seconds)
    def _url(self, path: str) -> str:
        return urllib.parse.urljoin(self.base_url + "/", path.lstrip("/"))

    def _get_json(self, path: str) -> dict:
        try:
            with urllib.request.urlopen(self._url(path), timeout=self.request_timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
            raise RGBDFrameUnavailable(f"cannot fetch {path}: {exc}") from exc

    def _download(self, url: str, dest: Path) -> None:
        tmp = dest.with_suffix(dest.suffix + ".tmp")
        try:
            with urllib.request.urlopen(url, timeout=self.request_timeout) as resp, tmp.open("wb") as out:
                out.write(resp.read())
            tmp.replace(dest)
        except (urllib.error.URLError, OSError) as exc:
            tmp.unlink(missing_ok=True)
            raise RGBDFrameUnavailable(f"cannot download {url}: {exc}") from exc

    def get_latest(self, timeout_seconds: float = 0.0) -> RGBDFrame:
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        last_error = "no RGB-D frame available"
        while True:
            try:
                return self._get_latest_once()
            except RGBDFrameUnavailable as exc:
                last_error = str(exc)
            if time.monotonic() >= deadline:
                raise RGBDFrameUnavailable(last_error)
            time.sleep(0.05)

    def _get_latest_once(self) -> RGBDFrame:
        meta = self._get_json("/rgbd/latest.json")
        if not meta.get("frame_id"):
            raise RGBDFrameUnavailable("latest.json missing frame_id")
        age = float((meta.get("health") or {}).get("age_s", 0.0) or 0.0)
        if age > self.max_age_seconds:
            raise RGBDFrameUnavailable(f"RGB-D frame stale: age={age:.2f}s")
        intrinsics = meta.get("intrinsics") or {}
        host_ts = meta.get("host_timestamp")
        # 计划书 §11.1：host_timestamp 必须是合法 Unix 秒；非法则回退接收时间
        # 并显式标记 timestamp_quality="receive_time"。
        timestamp, timestamp_quality = _resolve_capture_timestamp(
            host_ts, default=time.time()
        )
        frame = RGBDFrame(
            frame_id=str(meta["frame_id"]),
            timestamp=timestamp,
            color_ref=self._url(str(meta.get("color_url") or "")),
            depth_ref=self._url(str(meta.get("depth_url") or "")),
            width=int(meta.get("width") or 0),
            height=int(meta.get("height") or 0),
            fx=float(intrinsics.get("fx") or 0.0),
            fy=float(intrinsics.get("fy") or 0.0),
            cx=float(intrinsics.get("cx") or 0.0),
            cy=float(intrinsics.get("cy") or 0.0),
            depth_unit_m=float(meta.get("depth_unit_m", 0.001)),
            depth_aligned_to_color=bool(meta.get("depth_aligned_to_color", True)),
            device_timestamp_ms=meta.get("device_timestamp_ms"),
            host_timestamp=host_ts,
            timestamp_quality=timestamp_quality,
            health=dict(meta.get("health") or {}),
            provenance={"source": "realsense_http_atomic", "base_url": self.base_url},
        )
        return self.materialize(frame)

    def materialize(self, frame: RGBDFrame) -> RGBDFrame:
        if not frame.frame_id:
            raise RGBDFrameUnavailable("cannot materialize frame without frame_id")
        color_path = self.cache_dir / f"color_{frame.frame_id}.jpg"
        depth_path = self.cache_dir / f"depth_{frame.frame_id}.png"
        if not color_path.is_file() or color_path.stat().st_size == 0:
            self._download(frame.color_ref, color_path)
        if not depth_path.is_file() or depth_path.stat().st_size == 0:
            self._download(frame.depth_ref, depth_path)
        self._trim_cache()
        return RGBDFrame(
            frame_id=frame.frame_id,
            timestamp=frame.timestamp,
            color_ref=str(color_path),
            depth_ref=str(depth_path),
            width=frame.width,
            height=frame.height,
            fx=frame.fx,
            fy=frame.fy,
            cx=frame.cx,
            cy=frame.cy,
            depth_unit_m=frame.depth_unit_m,
            depth_aligned_to_color=frame.depth_aligned_to_color,
            device_timestamp_ms=frame.device_timestamp_ms,
            host_timestamp=frame.host_timestamp,
            health=frame.health,
            provenance={**frame.provenance, "materialized": True},
        )

    def health(self) -> dict[str, Any]:
        try:
            with urllib.request.urlopen(self._url("/health"), timeout=self.request_timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    def _trim_cache(self) -> None:
        files = sorted(
            self.cache_dir.glob("color_*.jpg"), key=lambda p: p.stat().st_mtime, reverse=True
        )
        for old in files[self.max_cached_frames :]:
            old.unlink(missing_ok=True)
            old.with_name(old.name.replace("color_", "depth_").replace(".jpg", ".png")).unlink(
                missing_ok=True
            )
