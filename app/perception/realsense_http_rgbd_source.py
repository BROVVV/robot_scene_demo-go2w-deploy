"""RealSense D435 HTTP RGB-D source using the atomic /rgbd endpoints."""

from __future__ import annotations

import json
import hashlib
import http.client
import ipaddress
import os
import shutil
import tempfile
import threading
import time
import uuid
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

from app.perception.rgbd_source import RGBDFrame, RGBDFrameUnavailable
from app.perception.remote_capture_clock import align_capture_time, shared_host_capture_time


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
    ``/rgbd/latest.json`` plus color/depth URLs bound to a unique capture token.
    Legacy numeric-ID URLs remain supported with weaker restart guarantees.
    This source downloads each frame
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
        clock_mode: str | None = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.max_cached_frames = max(8, int(max_cached_frames))
        self.request_timeout = float(request_timeout)
        self.max_age_seconds = float(max_age_seconds)
        self._materialize_lock = threading.Lock()
        self.clock_mode = clock_mode or os.environ.get('D435_CAPTURE_CLOCK_MODE', 'remote')
        if self.clock_mode not in {'remote', 'shared_host'}:
            raise ValueError('D435_CAPTURE_CLOCK_MODE must be remote or shared_host')
        if self.clock_mode == 'shared_host':
            try:
                loopback = ipaddress.ip_address(urllib.parse.urlparse(self.base_url).hostname).is_loopback
            except ValueError:
                loopback = False
            if not loopback:
                raise ValueError('Shared-host camera clock requires a literal loopback URL')
    def _url(self, path: str) -> str:
        return urllib.parse.urljoin(self.base_url + "/", path.lstrip("/"))

    def _get_json(self, path: str) -> dict:
        try:
            with urllib.request.urlopen(self._url(path), timeout=self.request_timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, http.client.HTTPException, json.JSONDecodeError) as exc:
            raise RGBDFrameUnavailable(f"cannot fetch {path}: {exc}") from exc

    def _download(self, url: str, dest: Path) -> None:
        tmp = dest.with_suffix(dest.suffix + ".tmp")
        try:
            with urllib.request.urlopen(url, timeout=self.request_timeout) as resp, tmp.open("wb") as out:
                out.write(resp.read())
            tmp.replace(dest)
        except (urllib.error.URLError, OSError, http.client.HTTPException) as exc:
            tmp.unlink(missing_ok=True)
            raise RGBDFrameUnavailable(f"cannot download {url}: {exc}") from exc

    def get_latest(self, timeout_seconds: float = 0.0) -> RGBDFrame:
        return self._latest_with_retry(timeout_seconds, materialize_images=True)

    def get_latest_metadata(self, timeout_seconds: float = 0.0) -> RGBDFrame:
        """Same validated capture metadata; leave image references as URLs."""
        return self._latest_with_retry(timeout_seconds, materialize_images=False)

    def _latest_with_retry(self, timeout_seconds: float, *, materialize_images: bool) -> RGBDFrame:
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        last_error = "no RGB-D frame available"
        while True:
            try:
                return (self._get_latest_once() if materialize_images
                        else self._get_latest_once(materialize_images=False))
            except RGBDFrameUnavailable as exc:
                last_error = str(exc)
            if time.monotonic() >= deadline:
                raise RGBDFrameUnavailable(last_error)
            time.sleep(0.05)

    def _get_latest_once(self, *, materialize_images: bool = True) -> RGBDFrame:
        started, monotonic_started = time.time(), time.monotonic()
        meta = self._get_json("/rgbd/latest.json")
        received, elapsed = time.time(), time.monotonic()-monotonic_started
        if not meta.get("frame_id"):
            raise RGBDFrameUnavailable("latest.json missing frame_id")
        if meta.get('capture_id'):
            for field in ('color_url', 'depth_url'):
                token = urllib.parse.parse_qs(urllib.parse.urlsplit(
                    str(meta.get(field) or '')).query).get('capture_id')
                if token != [str(meta['capture_id'])]:
                    raise RGBDFrameUnavailable(f'{field} does not bind capture_id')
        age = float((meta.get("health") or {}).get("age_s", 0.0) or 0.0)
        if age > self.max_age_seconds:
            raise RGBDFrameUnavailable(f"RGB-D frame stale: age={age:.2f}s")
        intrinsics = meta.get("intrinsics") or {}
        host_ts = meta.get("host_timestamp")
        # 计划书 §11.1：host_timestamp 必须是合法 Unix 秒；非法则回退接收时间
        # 并显式标记 timestamp_quality="receive_time"。
        timestamp, timestamp_quality = _resolve_capture_timestamp(
            host_ts, default=received
        )
        clock = {}
        if self.clock_mode == 'shared_host':
            if timestamp_quality != 'host_timestamp' or 'age_s' not in (meta.get('health') or {}):
                raise RGBDFrameUnavailable('shared-host capture clock metadata missing')
            timestamp, clock = shared_host_capture_time(host_ts, age, started=started,
                received=received, elapsed=elapsed, max_age_seconds=self.max_age_seconds)
            timestamp_quality = 'shared_host_capture'
        elif timestamp_quality == 'host_timestamp' and 'age_s' in (meta.get('health') or {}):
            timestamp, clock = align_capture_time(host_ts, age, started=started,
                received=received, elapsed=elapsed)
            timestamp_quality = 'host_clock_aligned'
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
            provenance={"source": "realsense_http_atomic", "base_url": self.base_url,
                        "capture_clock": clock, "capture_id": meta.get("capture_id"),
                        "cache_nonce": uuid.uuid4().hex if not meta.get("capture_id")
                        and timestamp_quality == 'receive_time' else None},
        )
        if materialize_images:
            return self.materialize(frame)
        return frame

    def materialize(self, frame: RGBDFrame) -> RGBDFrame:
        with self._materialize_lock:
            return self._materialize_frame(frame)

    def _materialize_frame(self, frame: RGBDFrame) -> RGBDFrame:
        if not frame.frame_id:
            raise RGBDFrameUnavailable("cannot materialize frame without frame_id")
        if frame.provenance.get('materialized'):
            if all(Path(ref).is_file() and Path(ref).stat().st_size > 0
                   for ref in (frame.color_ref, frame.depth_ref)):
                return frame
            raise RGBDFrameUnavailable('materialized RGB-D pair is no longer cached')
        # Keep references from older captures immutable, even when a producer
        # resets its numeric counter or this client is restarted. Never reuse
        # the unverified flat files left by the older cache implementation.
        host_stamp, host_quality = _resolve_capture_timestamp(frame.host_timestamp, default=0.)
        valid_host = host_quality == 'host_timestamp'
        identity = {"base_url": self.base_url, "frame_id": frame.frame_id,
            "capture_id": frame.provenance.get("capture_id"),
            "host_timestamp": host_stamp if valid_host else None,
            "device_timestamp_ms": frame.device_timestamp_ms,
            "color_url": frame.color_ref, "depth_url": frame.depth_ref,
            "geometry": [frame.width, frame.height, frame.fx, frame.fy,
                frame.cx, frame.cy, frame.depth_unit_m, frame.depth_aligned_to_color]}
        if not identity['capture_id'] and not valid_host:
            identity['nonce'] = frame.provenance.get('cache_nonce') or uuid.uuid4().hex
        try:
            key = hashlib.sha256(json.dumps(identity, sort_keys=True,
                allow_nan=False).encode()).hexdigest()
        except (TypeError, ValueError) as exc:
            raise RGBDFrameUnavailable(f"invalid capture identity: {exc}") from exc
        pair_dir = self.cache_dir / ('capture_' + key)
        safe_id = urllib.parse.quote(str(frame.frame_id), safe='')
        color_path = pair_dir / f"color_{safe_id}.jpg"
        depth_path = pair_dir / f"depth_{safe_id}.png"
        if not (color_path.is_file() and color_path.stat().st_size > 0
                and depth_path.is_file() and depth_path.stat().st_size > 0):
            staging = Path(tempfile.mkdtemp(prefix='.rgbd-', dir=self.cache_dir))
            try:
                self._download(frame.color_ref, staging / color_path.name)
                self._download(frame.depth_ref, staging / depth_path.name)
                if any(p.stat().st_size == 0 for p in staging.iterdir()):
                    raise RGBDFrameUnavailable('empty RGB-D download')
                # Both transfers must succeed before either file is visible
                # through a returned RGBDFrame. Failed staging is discarded.
                try:
                    staging.rename(pair_dir)
                except OSError as exc:
                    # Another source instance may have committed this exact
                    # identity while the downloads were in flight. Reuse its
                    # complete pair without deleting published references.
                    if not (color_path.is_file() and color_path.stat().st_size > 0
                            and depth_path.is_file() and depth_path.stat().st_size > 0):
                        raise RGBDFrameUnavailable(f"cannot commit RGB-D pair: {exc}") from exc
            finally:
                shutil.rmtree(staging, ignore_errors=True)
        self._trim_cache(keep=color_path)
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
            timestamp_quality=frame.timestamp_quality,
            health=frame.health,
            provenance={**frame.provenance, "materialized": True},
        )

    def health(self) -> dict[str, Any]:
        try:
            with urllib.request.urlopen(self._url("/health"), timeout=self.request_timeout) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    def _trim_cache(self, *, keep: Path) -> None:
        files = sorted([*self.cache_dir.glob("color_*.jpg"),
            *self.cache_dir.glob("capture_*/color_*.jpg")],
            key=lambda p: p.stat().st_mtime, reverse=True)
        # Filesystem timestamps can tie or go backwards. The pair being
        # returned must survive pruning regardless of its mtime/order.
        files = [keep] + [path for path in files if path != keep]
        for old in files[self.max_cached_frames :]:
            old.unlink(missing_ok=True)
            old.with_name(old.name.replace("color_", "depth_").replace(".jpg", ".png")).unlink(
                missing_ok=True
            )
            if old.parent != self.cache_dir:
                old.parent.rmdir()
