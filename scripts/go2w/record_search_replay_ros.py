#!/usr/bin/python3
"""只读 ROS 2 采集器：为一次真实搜索会话录制回放素材。

严格只读 —— 本进程只创建订阅、TF 监听和 HTTP 相机读取；不创建运动客户端、
不发布 cmd_vel、不发导航目标、不启动第二套建图、不改变任何运动授权。

数据写到 ``<session-dir>/replay/``：

* ``timeline.jsonl``  地图版本引用、坐标系分段、定位有效性、map→odom 证据
* ``poses.jsonl``     map→base_link（约 10 Hz）与 /go2w/odom/fused
* ``frames.jsonl``    相机 JPEG 记录（默认 ≤5 fps）
* ``maps/<hash>.npz`` 去重后的 OccupancyGrid（内容 hash 覆盖栅格与位姿元数据）
* ``frames/<seq>.jpg``
* ``capture_status.json`` 运行健康状态（原子写）

控制通道是 stdin 上的 JSON 行；stdin EOF（Web 崩溃）或 SIGTERM 都会收尾。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Any
from urllib.request import ProxyHandler, Request, build_opener

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402

from app.search_replay import storage  # noqa: E402
from app.search_replay.models import ReplayConfig, json_safe, load_replay_config  # noqa: E402

COLLECTOR_VERSION = "go2w_search_replay_ros_v1"
DEFAULT_CAMERA_URL = "http://192.168.123.18:8080/color"
MAP_SAVE_INTERVAL_S = 1.0
POSE_HZ = 10.0
ODOM_HZ = 10.0
EVIDENCE_HZ = 1.0
STATUS_HZ = 1.0
BUDGET_INTERVAL_S = 1.0
CAMERA_MIN_BYTES = 2000
CAMERA_MAX_BYTES = 8 * 1024 * 1024


def log(message: str) -> None:
    print(f"[replay-recorder] {message}", file=sys.stderr, flush=True)


def _now() -> tuple[float, float]:
    return time.monotonic(), time.time()


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    index = 2
    size = len(data)
    while index + 9 < size:
        if data[index] != 0xFF:
            index += 1
            continue
        marker = data[index + 1]
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            index += 2
            continue
        length = int.from_bytes(data[index + 2:index + 4], "big")
        if length < 2:
            return None
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height = int.from_bytes(data[index + 5:index + 7], "big")
            width = int.from_bytes(data[index + 7:index + 9], "big")
            return (width, height) if width and height else None
        index += 2 + length
    return None


class CameraReader:
    """Independent MJPEG client; bypasses the host HTTP proxy."""

    def __init__(self, recorder: "Recorder", url: str, fps: float) -> None:
        self.recorder = recorder
        self.url = url
        self.fps = max(0.2, float(fps))
        self.effective_fps = self.fps
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_emit = 0.0
        self._opener = build_opener(ProxyHandler({}))

    def start(self) -> None:
        if not self.url:
            return
        self._thread = threading.Thread(
            target=self._run, name="replay-camera", daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.recorder._camera_enabled = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _emit(self, jpeg: bytes) -> None:
        stamp, wall = _now()
        interval = 1.0 / max(0.1, self.effective_fps)
        if stamp - self._last_emit < interval:
            self.recorder.note_camera_skipped()
            return
        if not (CAMERA_MIN_BYTES <= len(jpeg) <= CAMERA_MAX_BYTES):
            self.recorder.note_camera_bad("尺寸异常")
            return
        dimensions = _jpeg_dimensions(jpeg)
        if dimensions is None:
            self.recorder.note_camera_bad("无法解析 JPEG 尺寸")
            return
        if self.recorder.write_frame(jpeg, dimensions, stamp, wall):
            self._last_emit = stamp

    def _run(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            if not self.recorder.camera_allowed():
                if self._stop.wait(0.5):
                    break
                continue
            try:
                request = Request(self.url, headers={
                    "User-Agent": "go2w-search-replay/1.0",
                    "Connection": "close",
                })
                with self._opener.open(request, timeout=5.0) as response:
                    backoff = 1.0
                    self.recorder.note_camera_connected()
                    self._pump(response)
            except Exception as exc:  # noqa: BLE001 - camera failure is non-fatal
                self.recorder.note_camera_error(f"{type(exc).__name__}: {exc}")
                if self._stop.wait(backoff):
                    break
                backoff = min(backoff * 2.0, 5.0)
            else:
                if self._stop.wait(0.2):
                    break

    def _pump(self, response) -> None:
        buffer = b""
        while not self._stop.is_set() and self.recorder.camera_allowed():
            chunk = response.read(4096)
            if not chunk:
                return
            buffer += chunk
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    buffer = buffer[-2:]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    buffer = buffer[start:]
                    break
                jpeg = buffer[start:end + 2]
                buffer = buffer[end + 2:]
                self._emit(jpeg)
            if len(buffer) > 4 * 1024 * 1024:
                buffer = b""


class Recorder:
    """Process-local state shared by ROS callbacks, camera thread and main loop."""

    def __init__(self, session_dir: Path, config: ReplayConfig,
                 camera_url: str, camera_fps: float) -> None:
        self.session_dir = session_dir
        self.replay_dir = storage.ensure_replay_layout(session_dir)
        self.config = config
        self.started_t, self.started_wall = _now()
        self.stop_event = threading.Event()
        self.lock = storage.FileLock(self.replay_dir / storage.LOCK)
        self.camera = CameraReader(self, camera_url, camera_fps)
        self.degraded: list[str] = []
        self.notes: list[str] = []
        self.ready = False
        self.status = "STARTING"
        self._seq = 0
        self._frame_seq = 0
        self._bytes_written = 0
        self._last_map_save = 0.0
        self._pending_map: dict[str, Any] | None = None
        self._last_map_hash = ""
        self._last_pose_t = 0.0
        self._last_odom_t = 0.0
        self._last_frame_t = 0.0
        self._last_map_t = 0.0
        self._tf_gap_start: float | None = None
        self._tf_gap_noted = False
        self._loc_valid = True
        self._topic_valid = True
        self._source_valid = True
        self._tf_lookup_valid = True
        self._last_pose_source_stamp: float | None = None
        self._source_stale_count = 0
        self._source_stale_noted = False
        self._clock_step_noted = False
        self._last_evidence: dict[str, float] | None = None
        self._odom_discontinuity_noted = False
        self._camera_state = "idle"
        self._camera_error = ""
        self._camera_connected = False
        self._camera_skip = 0
        self._camera_bad = 0
        self._map_locked = False
        self.counts = {
            "poses": 0, "valid_poses": 0, "odom": 0, "frames": 0, "events": 0,
            "map_messages": 0, "maps_stored": 0, "maps_merged": 0,
            "tf_failures": 0, "camera_errors": 0, "camera_skipped": 0,
            "camera_bad_frames": 0, "localization_invalid": 0,
            "pose_stale": 0, "source_clock_steps": 0,
            "map_odom_discontinuities": 0,
        }
        self._pose_buffer: list[dict[str, Any]] = []
        self._odom_buffer: list[dict[str, Any]] = []
        self._timeline_buffer: list[dict[str, Any]] = []
        self._camera_enabled = bool(camera_url)
        self._map_interval = 1.0 / max(0.1, float(config.map_max_hz))

    # -- lifecycle ------------------------------------------------------ #
    def acquire(self) -> bool:
        return self.lock.acquire()

    def mark_ready(self) -> None:
        if self.ready:
            return
        self.ready = True
        self.status = "RECORDING"
        self.write_status()

    def request_stop(self, reason: str) -> None:
        if not self.stop_event.is_set():
            self.add_note(f"停止原因：{reason}")
            self.stop_event.set()

    def add_note(self, note: str) -> None:
        if note and note not in self.notes:
            self.notes.append(note)

    def degrade(self, reason: str) -> None:
        if reason not in self.degraded:
            self.degraded.append(reason)
            self.add_note(f"降级：{reason}")
            log(f"degraded: {reason}")

    # -- writes --------------------------------------------------------- #
    def write_frame(self, jpeg: bytes, dimensions: tuple[int, int],
                    stamp: float, wall: float) -> bool:
        if not self._camera_enabled:
            return False
        self._frame_seq += 1
        name = f"{self._frame_seq:06d}.jpg"
        target = self.replay_dir / storage.FRAMES_DIR / name
        temporary = target.with_name(target.name + ".tmp")
        try:
            temporary.write_bytes(jpeg)
            os.replace(temporary, target)
        except OSError as exc:
            self.note_camera_bad(f"写入失败: {exc}")
            return False
        self._bytes_written += len(jpeg)
        gap = stamp - self._last_frame_t if self.counts["frames"] else None
        self._last_frame_t = stamp
        self._camera_state = "streaming"
        self._camera_error = ""
        self._camera_connected = True
        self.append_frames([{
            "kind": "frame",
            "session_id": self.session_id,
            "recording_id": self.recording_id,
            "segment_id": self.segment_id,
            "seq": self._frame_seq,
            "t": stamp,
            "wall": wall,
            "path": name,
            "width": dimensions[0],
            "height": dimensions[1],
            "quality": "http_receive_time",
            "gap_s": round(gap, 3) if gap is not None else None,
        }])
        self.counts["frames"] += 1
        return True

    def note_camera_connected(self) -> None:
        self._camera_connected = True
        self._camera_state = "streaming"

    def note_camera_error(self, message: str) -> None:
        self._camera_error = message
        self._camera_state = "error"
        self.counts["camera_errors"] += 1

    def note_camera_skipped(self) -> None:
        self._camera_skip += 1
        self.counts["camera_skipped"] += 1

    def note_camera_bad(self, message: str) -> None:
        self._camera_bad += 1
        self.counts["camera_bad_frames"] += 1
        self._camera_error = message

    def camera_allowed(self) -> bool:
        return self._camera_enabled and not self.stop_event.is_set()

    # -- dtypes --------------------------------------------------------- #
    @property
    def session_id(self) -> str:
        return self._session_id

    @property
    def recording_id(self) -> str:
        return self._recording_id

    @property
    def segment_id(self) -> str:
        return self._segment_id

    def configure_session(self, session_id: str, recording_id: str) -> None:
        self._session_id = session_id
        self._recording_id = recording_id
        self._segment_id = "seg0"

    def start_segment(self, reason: str, stamp: float, wall: float) -> None:
        index = len([item for item in self._timeline_buffer
                     if item.get("kind") == "segment"]) + 1
        self._segment_id = f"seg{index}"
        row = {"kind": "segment", "t": stamp, "wall": wall,
               "reason": reason, "segment_id": self._segment_id}
        self._timeline_buffer.append(row)
        self._pose_buffer.append(dict(row))
        log(f"segment {self._segment_id}: {reason}")

    # -- buffers -------------------------------------------------------- #
    def append_frames(self, rows: list[dict[str, Any]]) -> None:
        storage.append_jsonl(self.replay_dir / storage.FRAMES, rows)

    def flush(self) -> None:
        if self._pose_buffer:
            storage.append_jsonl(self.replay_dir / storage.POSES,
                                 self._pose_buffer)
            self._pose_buffer = []
        if self._odom_buffer:
            storage.append_jsonl(self.replay_dir / storage.POSES,
                                 self._odom_buffer)
            self._odom_buffer = []
        if self._timeline_buffer:
            storage.append_jsonl(self.replay_dir / storage.TIMELINE,
                                 self._timeline_buffer)
            self._timeline_buffer = []

    def add_pose(self, stamp: float, wall: float, transform) -> None:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        header = transform.header
        source_stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        delta = (None if self._last_pose_source_stamp is None
                 else source_stamp - self._last_pose_source_stamp)
        if delta is not None and delta < -0.5:
            # 源时钟回退：宿主 monotonic 仍然单调，只有源时间戳能证明换了
            # 一个时间轴。必须在这里切段，否则前后两段会被连成一条线。
            self.counts["source_clock_steps"] += 1
            if not self._clock_step_noted:
                self._clock_step_noted = True
                self.degrade(
                    f"TF 源时钟回退 {delta:.1f}s，已在回退处切段"
                )
            if self._source_valid is False:
                self._set_source_valid(True, stamp, wall)
            self.start_segment("source_clock_step", stamp, wall)
            self._source_stale_count = 0
        elif delta is not None and delta <= 0.0:
            # 缓存 TF：源时间戳没有推进。它可以保留为诊断，但绝不能算作
            # 一次新的有效定位，更不能成为轨迹点。
            self.counts["pose_stale"] += 1
            self._source_stale_count += 1
            if self._source_stale_count >= 2 and self._source_valid:
                self._set_source_valid(False, stamp, wall)
            return
        else:
            if not self._source_valid:
                self._set_source_valid(True, stamp, wall)
            self._source_stale_count = 0
        self._last_pose_source_stamp = source_stamp
        valid = bool(self._loc_valid)
        self._pose_buffer.append({
            "kind": "pose",
            "session_id": self.session_id,
            "recording_id": self.recording_id,
            "segment_id": self.segment_id,
            "seq": self.counts["poses"] + 1,
            "t": stamp,
            "wall": wall,
            "stamp": source_stamp,
            "frame_id": header.frame_id or "map",
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": yaw,
            "valid": valid,
            "source": "tf",
            "quality": "tf_lookup" if valid else "localization_invalid",
        })
        self.counts["poses"] += 1
        self.counts["valid_poses"] += int(valid)
        self._last_pose_t = stamp

    def add_odom(self, stamp: float, wall: float, message) -> None:
        pose = message.pose.pose
        yaw = _yaw_from_quaternion(pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w)
        header = message.header
        source_stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        self._odom_buffer.append({
            "kind": "odom",
            "session_id": self.session_id,
            "recording_id": self.recording_id,
            "segment_id": self.segment_id,
            "seq": self.counts["odom"] + 1,
            "t": stamp,
            "wall": wall,
            "stamp": source_stamp,
            "frame_id": header.frame_id or "odom_fused",
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw,
            "vx": float(message.twist.twist.linear.x),
        })
        self.counts["odom"] += 1
        self._last_odom_t = stamp

    def add_localization(self, stamp: float, wall: float,
                         payload: dict[str, Any], source: str) -> None:
        valid = bool(payload.get("valid", True))
        self._topic_valid = valid
        self._emit_localization(stamp, wall, payload, source)

    def _set_source_valid(self, valid: bool, stamp: float, wall: float) -> None:
        """TF source freshness is independent of the localization topic."""
        self._source_valid = valid
        if not valid and not self._source_stale_noted:
            self._source_stale_noted = True
            self.degrade("TF 源时间戳连续未推进，缓存位姿已标为无效")
        self._emit_localization(stamp, wall, {}, "tf_source")

    def set_tf_lookup_valid(self, valid: bool, stamp: float, wall: float) -> None:
        """A TF lookup never overwrites the last localization-topic verdict."""
        if self._tf_lookup_valid == valid:
            return
        self._tf_lookup_valid = valid
        self._emit_localization(stamp, wall, {}, "tf_lookup")

    def _emit_localization(self, stamp: float, wall: float,
                           payload: dict[str, Any], source: str) -> None:
        # 有效定位 = 定位话题认为有效 且 TF 源时间戳在推进。两者必须分开
        # 记录，否则一次 TF 缓存会把话题的 valid=true 覆盖掉。
        valid = bool(self._topic_valid and self._source_valid and self._tf_lookup_valid)
        evidence = payload.get("evidence") if isinstance(payload, dict) else None
        pose = evidence.get("pose") if isinstance(evidence, dict) else None
        self._timeline_buffer.append({
            "kind": "localization",
            "t": stamp,
            "wall": wall,
            "valid": valid,
            "blocking": bool(payload.get("blocking") or False),
            "pose_jump": bool(payload.get("pose_jump") or False),
            "frame": (pose or {}).get("frame") if isinstance(pose, dict) else None,
            "x": (pose or {}).get("x") if isinstance(pose, dict) else None,
            "y": (pose or {}).get("y") if isinstance(pose, dict) else None,
            "source": source,
        })
        if self._loc_valid and not valid:
            self.counts["localization_invalid"] += 1
            self.start_segment("localization_invalid", stamp, wall)
        self._loc_valid = valid

    # -- maps ----------------------------------------------------------- #
    @staticmethod
    def map_hash(message) -> tuple[str, np.ndarray]:
        grid = np.asarray(message.data, dtype=np.int8).reshape(
            (int(message.info.height), int(message.info.width))
        )
        origin = message.info.origin
        orientation = origin.orientation
        meta = struct.pack(
            "<7d", float(origin.position.x), float(origin.position.y),
            float(origin.position.z), float(orientation.x),
            float(orientation.y), float(orientation.z), float(orientation.w),
        ) + struct.pack("<2id", int(message.info.width),
                        int(message.info.height),
                        float(message.info.resolution))
        digest = hashlib.sha256(meta + struct.pack("<i", len(message.data)))
        digest.update(grid.tobytes())
        return digest.hexdigest()[:32], grid

    def on_map(self, message) -> None:
        stamp, wall = _now()
        if self._map_locked:
            self.counts["map_messages"] += 1
            return
        try:
            digest, grid = self.map_hash(message)
        except (ValueError, TypeError) as exc:
            self.add_note(f"/map 解析失败: {exc}")
            return
        self.counts["map_messages"] += 1
        self._last_map_t = stamp
        info = message.info
        origin = info.origin
        orientation = origin.orientation
        row = {
            "kind": "map",
            "t": stamp,
            "wall": wall,
            "stamp": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            "frame_id": message.header.frame_id or "map",
            "width": int(info.width),
            "height": int(info.height),
            "resolution": float(info.resolution),
            "origin": [
                float(origin.position.x), float(origin.position.y),
                float(origin.position.z), float(orientation.x),
                float(orientation.y), float(orientation.z),
                float(orientation.w),
            ],
            "hash": digest,
            "new_content": digest != self._last_map_hash,
            # 只有真正落盘的版本才能被回放；被限频合并掉的中间版本保持
            # false，读取端会跳过它并回退到上一可用地图。
            "stored": False,
        }
        if digest == self._last_map_hash:
            # The source has returned to the persisted version before the
            # pending version was saved. That pending map is now obsolete.
            self._pending_map = None
            self._timeline_buffer.append(row)
            return
        self._timeline_buffer.append(row)
        self._pending_map = {
            "hash": digest,
            "grid": grid,
            "row": row,
            "frame_id": row["frame_id"],
        }
        # 首图立即落盘；后续最多每秒一次，被合并的计数。
        if self._last_map_hash == "" or stamp - self._last_map_save >= self._map_interval:
            self.store_pending_map(stamp)
        else:
            self.counts["maps_merged"] += 1

    def store_pending_map(self, stamp: float | None = None) -> None:
        pending = self._pending_map
        if pending is None:
            return
        digest = pending["hash"]
        target = self.replay_dir / storage.MAPS_DIR / f"{digest}.npz"
        if not target.exists():
            temporary = target.with_name(f".{digest}.{os.getpid()}.tmp.npz")
            try:
                np.savez_compressed(temporary, grid=pending["grid"])
                try:
                    with temporary.open("rb") as handle:
                        os.fsync(handle.fileno())
                except OSError:
                    pass
                os.replace(temporary, target)
                self._bytes_written += target.stat().st_size
            except OSError as exc:
                self.add_note(f"地图落盘失败: {exc}")
                try:
                    temporary.unlink()
                except OSError:
                    pass
                return
        self._last_map_hash = digest
        self._last_map_save = stamp if stamp is not None else _now()[0]
        row = pending["row"]
        row["stored"] = True
        # timeline 可能已经把该行按 stored=false 刷到磁盘了（pending 等待
        # 限频期间），再补一条显式落盘标记，读取端以标记为准。
        self._timeline_buffer.append({
            "kind": "map_stored",
            "t": row.get("t", self._last_map_save),
            "wall": row.get("wall"),
            "hash": digest,
            "stored": True,
        })
        self._pending_map = None
        self.counts["maps_stored"] += 1

    # -- status --------------------------------------------------------- #
    def write_status(self) -> None:
        stamp, wall = _now()
        payload = {
            "schema_version": "go2w_search_replay_v1",
            "collector": COLLECTOR_VERSION,
            "session_id": self.session_id,
            "recording_id": self.recording_id,
            "status": self.status,
            "ready": self.ready,
            "read_only": True,
            "pid": os.getpid(),
            "started_t": self.started_t,
            "updated_t": stamp,
            "updated_wall": wall,
            "segment_id": self.segment_id,
            "counts": dict(self.counts),
            "last": {
                "pose_t": self._last_pose_t,
                "odom_t": self._last_odom_t,
                "frame_t": self._last_frame_t,
                "map_t": self._last_map_t,
            },
            "gaps": {
                "tf_active": self._tf_gap_start is not None,
                "tf_gap_started_t": self._tf_gap_start,
                "map_age_s": round(stamp - self._last_map_t, 3)
                if self._last_map_t else None,
                "pose_age_s": round(stamp - self._last_pose_t, 3)
                if self._last_pose_t else None,
                "frame_age_s": round(stamp - self._last_frame_t, 3)
                if self._last_frame_t else None,
            },
            "camera": {
                "url": self.camera.url,
                "enabled": self._camera_enabled,
                "state": self._camera_state,
                "connected": self._camera_connected,
                "target_fps": self.camera.fps,
                "effective_fps": self.camera.effective_fps,
                "last_error": self._camera_error,
                "skipped": self._camera_skip,
                "bad_frames": self._camera_bad,
            },
            "degraded": list(self.degraded),
            "notes": list(self.notes),
            "bytes_written": self._bytes_written,
            "disk_free_bytes": storage.free_bytes(self.replay_dir),
            "map_interval_s": self._map_interval,
        }
        storage.atomic_write_json(self.replay_dir / storage.CAPTURE_STATUS,
                                  json_safe(payload))

    # -- budgets -------------------------------------------------------- #
    def check_budget(self) -> None:
        free = storage.free_bytes(self.replay_dir)
        soft = self.config.session_soft_bytes
        if self._camera_enabled and self._bytes_written > soft \
                and self.camera.effective_fps > 1.0:
            self.camera.effective_fps = 1.0
            self.degrade(f"会话素材超过软预算 {soft} 字节，相机降到 1fps")
        if self._camera_enabled and (
            self._bytes_written > soft * 1.5
            or free < self.config.min_free_bytes
        ):
            self._camera_enabled = False
            self.degrade("磁盘预算不足，已停止相机录制（地图/位姿继续）")
        if not self._map_locked and (
            self._bytes_written > soft * 3
            or free < self.config.min_free_bytes / 2
        ):
            self._map_locked = True
            self.degrade("磁盘接近硬限制，停止地图落盘（轨迹/事件继续）")

    def flush_tick(self) -> None:
        # 先落盘待存地图，再刷 timeline：这样绝大多数地图行写盘时已经
        # 带上 stored=true（等待限频、已提前刷出的少数版本由 map_stored
        # 标记兜底）。
        if self._pending_map is not None and \
                _now()[0] - self._last_map_save >= self._map_interval:
            self.store_pending_map()
        self.flush()


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="只读采集一次搜索会话的回放素材（不控制机器人）"
    )
    parser.add_argument("--session-dir", required=True,
                        help="现有搜索会话目录（outputs/live_runs/search_*）")
    parser.add_argument("--session-id", default="",
                        help="会话 id；默认取目录名")
    parser.add_argument("--recording-id", default="",
                        help="录制 UUID；默认新生成")
    parser.add_argument("--config", default="",
                        help="回放 YAML 配置路径（默认 configs/go2w/search_replay.yaml）")
    parser.add_argument("--camera-url", default="",
                        help="MJPEG 相机地址；传空字符串可禁用相机")
    parser.add_argument("--camera-fps", type=float, default=0.0,
                        help="相机采样上限（默认取配置）")
    parser.add_argument("--stdin-control", action="store_true",
                        help="从 stdin 读取 JSON 控制行（EOF 表示收尾）")
    parser.add_argument("--version", action="version", version=COLLECTOR_VERSION)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    session_dir = Path(args.session_dir).resolve()
    if not session_dir.is_dir():
        log(f"会话目录不存在: {session_dir}")
        return 2
    session_id = args.session_id or session_dir.name
    if not storage.valid_session_id(session_id):
        log(f"非法 session_id: {session_id}")
        return 2
    recording_id = args.recording_id or f"rec_{os.urandom(6).hex()}"
    if not storage.valid_recording_id(recording_id):
        log(f"非法 recording_id: {recording_id}")
        return 2

    config = load_replay_config(PROJECT_ROOT,
                                Path(args.config) if args.config else None)
    camera_url = args.camera_url or config.camera_url
    if camera_url == "-" or camera_url.lower() in {"none", "off"}:
        camera_url = ""
    if not camera_url and args.camera_url == "":
        camera_url = DEFAULT_CAMERA_URL
    camera_fps = args.camera_fps or config.camera_fps

    recorder = Recorder(session_dir, config, camera_url, camera_fps)
    recorder.configure_session(session_id, recording_id)
    recorder._timeline_buffer.append({
        "kind": "start",
        "t": recorder.started_t,
        "wall": recorder.started_wall,
        "collector": COLLECTOR_VERSION,
        "read_only": True,
        "session_id": session_id,
        "recording_id": recording_id,
        "camera_url": camera_url,
        "map_topic": config.map_topic,
        "odom_topic": config.odom_topic,
        "localization_topic": config.localization_topic,
    })
    if not recorder.acquire():
        log("另一个采集器已持有该会话的 replay/.lock")
        recorder.status = "FAILED"
        recorder.add_note("采集器锁被占用，可能有另一份录制在运行")
        recorder.write_status()
        return 3

    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener, LookupException, \
        ConnectivityException, ExtrapolationException

    rclpy.init(args=None)
    node = Node("go2w_search_replay_recorder")

    map_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    odom_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    status_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
    TransformListener(tf_buffer, node, spin_thread=False,
                      qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_odom(message: Odometry) -> None:
        stamp, wall = _now()
        if stamp - recorder._last_odom_t < 1.0 / ODOM_HZ:
            return
        recorder.add_odom(stamp, wall, message)

    def on_localization(message: String) -> None:
        stamp, wall = _now()
        try:
            payload = json.loads(message.data or "{}")
        except json.JSONDecodeError:
            payload = {"valid": True, "parse_error": True}
        if not isinstance(payload, dict):
            payload = {"valid": True}
        recorder.add_localization(stamp, wall, payload, "topic")

    def on_map(message: OccupancyGrid) -> None:
        recorder.on_map(message)

    subscriptions = [
        node.create_subscription(OccupancyGrid, config.map_topic, on_map, map_qos),
        node.create_subscription(Odometry, config.odom_topic, on_odom, odom_qos),
        node.create_subscription(String, config.localization_topic,
                                 on_localization, status_qos),
    ]

    control_thread = None
    if args.stdin_control:
        control_thread = threading.Thread(
            target=_control_loop, args=(recorder,), name="replay-control",
            daemon=True,
        )
        control_thread.start()

    def _signal_handler(signum, _frame) -> None:
        recorder.request_stop(f"signal {signum}")

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    recorder.status = "RECORDING"
    recorder.write_status()
    recorder.camera.start()
    log(f"recording session={session_id} recording={recording_id} "
        f"dir={recorder.replay_dir} camera={'on' if camera_url else 'off'}")

    last_pose = 0.0
    last_evidence = 0.0
    last_status = 0.0
    last_budget = 0.0
    start_monotonic = recorder.started_t
    pose_period = 1.0 / POSE_HZ

    try:
        while not recorder.stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
            stamp, wall = _now()
            if not recorder.ready and (stamp - start_monotonic >= 0.5):
                recorder.mark_ready()

            if stamp - last_pose >= pose_period:
                last_pose = stamp
                _sample_pose(recorder, tf_buffer, Time, Duration,
                             LookupException, ConnectivityException,
                             ExtrapolationException, stamp, wall)

            if stamp - last_evidence >= 1.0 / EVIDENCE_HZ:
                last_evidence = stamp
                _sample_evidence(recorder, tf_buffer, Time, Duration,
                                 LookupException, ConnectivityException,
                                 ExtrapolationException, stamp, wall)

            if stamp - last_status >= 1.0 / STATUS_HZ:
                last_status = stamp
                recorder.flush_tick()
                recorder.write_status()

            if stamp - last_budget >= BUDGET_INTERVAL_S:
                last_budget = stamp
                recorder.check_budget()
    except KeyboardInterrupt:
        recorder.request_stop("KeyboardInterrupt")
    finally:
        recorder.status = "FINALIZING"
        recorder.write_status()
        recorder.camera.stop()
        for subscription in subscriptions:
            try:
                node.destroy_subscription(subscription)
            except Exception:  # noqa: BLE001
                pass
        recorder.store_pending_map()
        recorder.flush()
        _write_final_manifest(recorder, session_dir)
        recorder.status = "FINALIZED"
        recorder.write_status()
        recorder.flush()
        log(f"finalized counts={recorder.counts} degraded={recorder.degraded}")
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()
        recorder.lock.release()
    return 0


def _control_loop(recorder: Recorder) -> None:
    """Read JSON control lines; EOF means the Web process is gone."""
    try:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(payload, dict):
                continue
            command = str(payload.get("cmd") or "")
            if command in {"stop", "finalize"}:
                recorder.request_stop(str(payload.get("reason") or "control stop"))
                return
            if command == "mark":
                stamp, wall = _now()
                reason = str(payload.get("reason") or "mark")
                recorder.start_segment(reason, stamp, wall)
                reporter = payload.get("event")
                if isinstance(reporter, dict):
                    recorder._timeline_buffer.append({
                        "kind": "search_event", "t": stamp, "wall": wall,
                        "event": reporter,
                    })
    except (OSError, ValueError):
        pass
    recorder.request_stop("control channel closed")


def _sample_pose(recorder: Recorder, buffer, Time, Duration,
                 LookupException, ConnectivityException, ExtrapolationException,
                 stamp: float, wall: float) -> None:
    try:
        transform = buffer.lookup_transform(
            recorder.config.map_frame, recorder.config.base_frame,
            Time(), timeout=Duration(seconds=0.05),
        )
    except (LookupException, ConnectivityException, ExtrapolationException) as exc:
        recorder.counts["tf_failures"] += 1
        if recorder._tf_gap_start is None:
            recorder._tf_gap_start = stamp
            recorder._tf_gap_noted = False
            recorder.set_tf_lookup_valid(False, stamp, wall)
        if stamp - recorder._tf_gap_start > 1.0 and not recorder._tf_gap_noted:
            # The live gap start/duration and failure count remain in status.
            # A changing duration in every note made a single prolonged gap
            # append thousands of unique strings and reserialize them at 1Hz.
            recorder.add_note(f"TF 中断超过 1s: {type(exc).__name__}")
            recorder._tf_gap_noted = True
        return
    except Exception as exc:  # noqa: BLE001 - never kill the recorder loop
        recorder.counts["tf_failures"] += 1
        recorder.set_tf_lookup_valid(False, stamp, wall)
        recorder.add_note(f"TF 查询异常: {type(exc).__name__}: {exc}")
        return
    if recorder._tf_gap_start is not None:
        gap = stamp - recorder._tf_gap_start
        recorder._tf_gap_start = None
        recorder._tf_gap_noted = False
        recorder.start_segment("tf_resume", stamp, wall)
        recorder.add_note(f"TF 恢复（中断 {gap:.1f}s）")
    recorder.set_tf_lookup_valid(True, stamp, wall)
    try:
        recorder.add_pose(stamp, wall, transform)
    except (ValueError, AttributeError) as exc:
        recorder.add_note(f"位姿写入失败: {exc}")


def _sample_evidence(recorder: Recorder, buffer, Time, Duration,
                     LookupException, ConnectivityException,
                     ExtrapolationException, stamp: float, wall: float) -> None:
    try:
        transform = buffer.lookup_transform(
            recorder.config.map_frame, "odom_fused",
            Time(), timeout=Duration(seconds=0.05),
        )
    except (LookupException, ConnectivityException, ExtrapolationException):
        return
    except Exception:  # noqa: BLE001 - evidence is optional
        return
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    header = transform.header
    recording = recorder
    recording._timeline_buffer.append({
        "kind": "evidence",
        "t": stamp,
        "wall": wall,
        "stamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
        "frame": f"{recorder.config.map_frame}->odom_fused",
        "x": float(translation.x),
        "y": float(translation.y),
        "yaw": _yaw_from_quaternion(rotation.x, rotation.y, rotation.z,
                                    rotation.w),
    })
    # map→odom_fused 跳变说明坐标/定位被重置过。此时位姿源时间戳可能仍在
    # 前进，只有这条变换能看出来；轨迹必须在这里切断，不能画成一条直线。
    current = {"t": stamp, "x": float(translation.x), "y": float(translation.y),
               "yaw": _yaw_from_quaternion(rotation.x, rotation.y, rotation.z,
                                           rotation.w)}
    previous = recorder._last_evidence
    recorder._last_evidence = current
    if previous is None:
        return
    gap = current["t"] - previous["t"]
    if gap > 3.0:
        return
    jump = math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
    yaw_jump = abs(math.atan2(math.sin(current["yaw"] - previous["yaw"]),
                              math.cos(current["yaw"] - previous["yaw"])))
    if jump <= 1.0 and yaw_jump <= math.pi / 4:
        return
    recorder.counts["map_odom_discontinuities"] += 1
    if not recorder._odom_discontinuity_noted:
        recorder._odom_discontinuity_noted = True
        recorder.degrade("map→odom_fused 出现跳变，坐标可能被重置，已在跳变处切段")
    recorder.start_segment("map_odom_discontinuity", stamp, wall)


def _write_final_manifest(recorder: Recorder, session_dir: Path) -> None:
    stamp, wall = _now()
    path = recorder.replay_dir / storage.MANIFEST
    manifest = storage.read_json(path) or {}
    manifest.setdefault("schema_version", "go2w_search_replay_v1")
    manifest["session_id"] = recorder.session_id
    manifest["recording_id"] = recorder.recording_id
    quality = "FULL"
    if (recorder.degraded or recorder.counts["valid_poses"] < recorder.counts["poses"]
            or recorder.counts["valid_poses"] == 0
            or recorder.counts["frames"] == 0
            or recorder.counts["maps_stored"] == 0):
        quality = "DEGRADED"
    if quality == "DEGRADED":
        if recorder.counts["valid_poses"] == 0:
            recorder.add_note("未记录到有效位姿（TF map→base_link）")
        elif recorder.counts["valid_poses"] < recorder.counts["poses"]:
            recorder.add_note("部分定位记录无效，已从实测轨迹中排除")
        if recorder.counts["maps_stored"] == 0:
            recorder.add_note("未记录到 /map 地图版本")
        if recorder.counts["frames"] == 0 and recorder.camera.url:
            recorder.add_note("未记录到相机帧")
    manifest["recording"] = {
        "collector": COLLECTOR_VERSION,
        "read_only": True,
        "pid": os.getpid(),
        "started_t": recorder.started_t,
        "started_wall": recorder.started_wall,
        "ended_t": stamp,
        "ended_wall": wall,
        "counts": dict(recorder.counts),
        "degraded": list(recorder.degraded),
        "notes": list(recorder.notes),
        "camera": {
            "url": recorder.camera.url,
            "target_fps": recorder.camera.fps,
            "effective_fps": recorder.camera.effective_fps,
            "quality": "http_receive_time",
        },
        "map_interval_s": recorder._map_interval,
        "sampling": {
            "pose_hz": POSE_HZ,
            "odom_hz": ODOM_HZ,
            "evidence_hz": EVIDENCE_HZ,
            "camera_target_fps": recorder.camera.fps,
            "map_max_hz": round(1.0 / recorder._map_interval, 3),
        },
        "topics": {
            "map": recorder.config.map_topic,
            "odom": recorder.config.odom_topic,
            "localization": recorder.config.localization_topic,
            "map_frame": recorder.config.map_frame,
            "base_frame": recorder.config.base_frame,
        },
        "quality": quality,
    }
    manifest["data_range"] = {
        "first_t": min(recorder.started_t, recorder._last_pose_t or recorder.started_t),
        "last_t": stamp,
        "poses": recorder.counts["poses"],
        "map_versions": recorder.counts["maps_stored"],
        "frames": recorder.counts["frames"],
    }
    storage.atomic_write_json(path, json_safe(manifest))


if __name__ == "__main__":
    sys.exit(main())
