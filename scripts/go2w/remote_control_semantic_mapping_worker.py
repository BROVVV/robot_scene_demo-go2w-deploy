#!/usr/bin/env python3
"""实体遥控器建图的语义侧车：周期采 RGB-D 关键帧并把物体落到 pslam_map。

计划书：docs/GO2W_遥控主动建图与周期物体落图_MVP实施计划_20260918.md §6.1

这个进程**只读**：订阅 SLAM 位姿 / TF，读 D435 的 HTTP RGB-D，写自己的
events.jsonl 与 status.json。它没有任何 motion client、Action client、速度
发布器或遥控器发布器 —— 机器狗怎么动完全由 Unitree 实体遥控器决定。

两个执行单元，刻意不写成一个 ``sleep(5)`` 串行循环：

* **关键帧采集器**（主线程）：持续 ``spin_once()`` 喂 ROS 位姿缓存，按
  「位移 / 转角 / 最长等待」三选一判断是否留帧，留帧后立刻返回继续 spin。
* **单实例推理消费者**（后台线程）：始终只有一个 GroundedSAM 在跑，上一帧
  处理完立刻取下一帧；队列空时才等待。

采帧与推理解耦保证的是**路径视觉覆盖**：GPU 忙时地图上的物体标记会滞后，
但已经采下来的中间关键帧仍会随后被处理，不会因为检测器忙就丢掉整段路。
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# ------------------------------------------------------------------ 状态常量
STATE_WAITING = "waiting"
STATE_LOADING = "loading"
STATE_RUNNING = "running"
STATE_DEGRADED = "degraded"
STATE_STOPPED = "stopped"

# 连续失败多少轮进入 degraded。取 3 是因为单帧 RGB-D 抖动、单次 GPU OOM 都
# 不该让页面显示成故障 —— 但连续三轮都失败就不是抖动了。
DEGRADED_AFTER_CONSECUTIVE_FAILURES = 3

# 采集时间戳可信度。D435 由机器狗侧服务打 `time.time()`，宿主侧 SLAM 位姿走
# ROS 时基 —— 两机时钟若不一致，SLAMBridge 的最近邻查找会**静默**取到错误的
# 位姿（本仓实测过一次 738.4s 的两机时钟差）。这个门限就是防那条假绿：
# 偏差超过它就不用拍摄时间戳，退回接收时刻（只有 HTTP 往返的毫秒级误差）。
DEFAULT_MAX_CLOCK_SKEW_S = 5.0

# RAM++ 场景词缓存时长，与 run_autonomous_loop 的取值一致。
DEFAULT_RAM_SCENE_TAG_INTERVAL_S = 8.0

# 「运动过快就跳过识别」的门限。只跳过识别，绝不干涉运动。取的是一眼能看出
# 「这不是在正常走」的量级，正常步行/转向不会触发。
DEFAULT_MAX_LINEAR_SPEED_MPS = 2.0
DEFAULT_MAX_ANGULAR_SPEED_DPS = 120.0

# 语义合并半径与确认观测数。与 run_semantic_exploration.py 里构造
# SemanticObjectMap 用的值保持一致 —— 计划书 §6.4 要求沿用现有语义合并半径，
# 不另起一套配置。
SEMANTIC_MAP_MERGE_DISTANCE_M = 0.4
SEMANTIC_MAP_CONFIRM_MIN_OBSERVATIONS = 2

# YOLOE prompt-free 的原始类别只是待复核证据，不能在中文 UI 里把所有未命中
# 词表的框都压成同一个「未分类物体」。VLM 尚未返回时保留原始类别，既能看出
# 后台正在工作，也能区分队列中的不同候选。
UNKNOWN_LABEL_REVIEW_PENDING = "待VLM复核"


def _extract_teleop_geometric_relations(
    scene_objects: list[dict[str, Any]], relation_config: dict[str, Any]
) -> list[dict[str, Any]]:
    """提取遥控关键帧中的方向关系，邻近关系仍由地图距离统一维护。

    ``scene_objects`` 已经由 RGB-D 定位器补齐相机坐标，因此这里沿用自主建图
    使用的同帧几何提取器。过滤 ``near`` 是为了避免同一帧既按相机距离、又按
    持久地图距离给同一条边重复计数。
    """
    from app.spatial.geometric_relation_extractor import extract_geometric_relations

    relations = extract_geometric_relations(
        scene_objects,
        min_horizontal_separation=float(
            relation_config.get("min_horizontal_separation", 0.10)
        ),
        min_vertical_separation=float(
            relation_config.get("min_vertical_separation", 0.10)
        ),
        min_depth_separation_m=float(
            relation_config.get("min_depth_separation_m", 0.25)
        ),
        near_distance_m=float(relation_config.get("camera_near_distance_m", 0.90)),
    )
    return [item for item in relations if item.get("relation") != "near"]


def _wrap_pi(delta: float) -> float:
    """把角差（弧度）折到 (-pi, pi]。

    存在的理由：位姿里的 yaw 是弧度（SpatialPose 由 atan2 产出），而关键帧
    门限是按**度**配的。混用不会报错，只会让 15° 的门变成 15 rad —— 于是
    每一帧都被判成"转过了一个门限"。所以两条换算各自只走自己的入口。
    """
    return (float(delta) + math.pi) % (2.0 * math.pi) - math.pi


def _interpreter_has_torch(python: str) -> bool:
    """解释器是不是真的能 import torch（检测器子进程的最低要求）。

    只查文件存在是不够的：本机的 .venv/bin/python 存在但没有 torch，选中它
    会让 GroundedSAM 子进程起不来，而且失败面在子进程里，看起来像"识别不出
    东西"而不是"解释器选错了"。
    """
    try:
        completed = subprocess.run(
            [python, "-c", "import torch"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=60,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return completed.returncode == 0


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _merge_discovered_scene_terms(
    scheduled_terms: list[str],
    discovered_terms: list[str],
    resident_terms: list[str],
    max_terms: int,
) -> list[str]:
    """Keep safety prompts resident and prefer RAM++ scene discoveries.

    The rotating schedule remains the deterministic fallback, but it must not
    consume the whole GroundingDINO prompt budget before the current frame's
    RAM++ vocabulary is considered.  YOLOE and disabled/unavailable RAM++
    naturally pass an empty ``discovered_terms`` list and retain the previous
    schedule.
    """
    merged: list[str] = []
    for value in [*resident_terms, *discovered_terms, *scheduled_terms]:
        term = " ".join(str(value).strip().split())
        if term and term not in merged:
            merged.append(term)
    return merged[: max(1, int(max_terms))]


def _bbox_area_ratio(bbox: Any) -> float:
    try:
        x1, y1, x2, y2 = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return 0.0
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def _bbox_center_distance_ratio(left: Any, right: Any) -> float:
    try:
        lx1, ly1, lx2, ly2 = (float(value) for value in left)
        rx1, ry1, rx2, ry2 = (float(value) for value in right)
    except (TypeError, ValueError):
        return float("inf")
    return math.hypot(
        (lx1 + lx2 - rx1 - rx2) * 0.5,
        (ly1 + ly2 - ry1 - ry2) * 0.5,
    )


def _bbox_union(
    left: Any, right: Any, margin: float = 0.0
) -> tuple[float, float, float, float]:
    lx1, ly1, lx2, ly2 = (float(value) for value in left)
    rx1, ry1, rx2, ry2 = (float(value) for value in right)
    return (
        max(0.0, min(lx1, rx1) - margin),
        max(0.0, min(ly1, ry1) - margin),
        min(1.0, max(lx2, rx2) + margin),
        min(1.0, max(ly2, ry2) + margin),
    )


def _host_local_anchor(candidate_bbox: Any, host_bbox: Any) -> list[float] | None:
    """Normalized anomaly centre on its host bbox; separates two pipe spots."""
    try:
        cx1, cy1, cx2, cy2 = (float(value) for value in candidate_bbox)
        hx1, hy1, hx2, hy2 = (float(value) for value in host_bbox)
    except (TypeError, ValueError):
        return None
    if hx2 <= hx1 or hy2 <= hy1:
        return None
    return [
        round(((cx1 + cx2) * 0.5 - hx1) / (hx2 - hx1), 4),
        round(((cy1 + cy2) * 0.5 - hy1) / (hy2 - hy1), 4),
    ]


@dataclass
class Keyframe:
    """一张待推理的关键帧。RGB / depth / intrinsics / 位姿在采帧瞬间一起冻结。"""

    frame_id: str
    capture_timestamp: float
    transform_timestamp: float
    timestamp_source: str
    clock_skew_seconds: float | None
    rgbd_frame: Any
    session_id: int
    robot_pose: dict[str, Any] = field(default_factory=dict)
    # 采帧时刻的 RGB-D 源健康快照，只作诊断。
    source_health: dict[str, Any] = field(default_factory=dict)

    def pose_xyz_yaw(self) -> tuple[float, float, float] | None:
        pose = self.robot_pose or {}
        if not all(_finite(pose.get(key)) for key in ("x", "y", "yaw")):
            return None
        return (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))


class RemoteControlSemanticMapper:
    """采帧 + 推理 + 落图。所有对外可见的状态都写在 status.json 里。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.events_path = Path(args.events).resolve()
        self.status_path = Path(args.status).resolve()
        self.topology_path = Path(args.topology_snapshot_path).resolve()
        self.target_watch_request_path = Path(args.target_watch_request).resolve()
        self.target_watch_state_path = Path(args.target_watch_state).resolve()
        self.target_watch_evidence_dir = Path(args.target_watch_evidence_dir).resolve()
        self.topology_path.parent.mkdir(parents=True, exist_ok=True)
        self._frame_path = self.topology_path.parent / "semantic_frame.jpg"
        self._frame_available = False
        import yaml
        from app.spatial.hazard_rules import HazardMonitor
        from app.spatial.object_mobility import ObjectMobilityTracker
        from app.spatial.place_graph import PlaceGraph
        from app.perception.pipe_context_tracker import PipeContextTracker
        from app.perception.term_rotation import RotatingTermScheduler
        self.mapping_config = yaml.safe_load(Path(args.mapping_config).read_text(encoding="utf-8")) or {}
        from app.perception.passive_target_watch import (
            TargetWatchConfig,
            empty_state,
        )
        from app.perception.unknown_object_vlm import UnknownObjectVLMConfig
        self._target_watch_config = TargetWatchConfig.from_mapping(
            self.mapping_config.get("target_watch")
        )
        self._unknown_label_config = UnknownObjectVLMConfig.from_mapping(
            self.mapping_config.get("unknown_object_vlm")
        )
        configured_detector = str(
            (self.mapping_config.get("perception") or {}).get("detector")
            or "grounded_sam"
        )
        self.detector_backend = str(
            args.detector
            or os.environ.get("REMOTE_MAPPING_DETECTOR")
            or configured_detector
        ).strip().lower()
        if self.detector_backend not in {"grounded_sam", "yoloe_seg"}:
            raise ValueError(f"unsupported remote mapping detector: {self.detector_backend}")
        self._mobility = ObjectMobilityTracker(self.mapping_config.get("mobility"))
        self._hazards = HazardMonitor(self.mapping_config.get("hazard"))
        visual = self.mapping_config.get("hazard", {}).get("visual_enhancement", {})
        self._pipe_tracker = PipeContextTracker(
            max_age_s=float(visual.get("pipe_context_max_age_s", 8.0)),
            min_quality=float(visual.get("pipe_tracking_min_quality", 0.65)),
        )
        self._term_scheduler = RotatingTermScheduler(
            self.mapping_config.get("detection_schedule"),
            self.mapping_config.get("detection_terms", []),
        )
        self._relations = None
        topology_cfg = self.mapping_config.get("topology", {})
        self._place_graph = PlaceGraph(
            merge_distance_m=float(topology_cfg.get("place_merge_distance_m", 2.0)),
            relocation_min_displacement_m=float(topology_cfg.get("place_sample_distance_m", 8.0)),
        )
        self._last_place_sample_pose: tuple[float, float, float] | None = None
        self._last_place_pose_at: float | None = None
        self._map_epoch = ""
        self._topology_revision = 0
        self._current_detections: list[dict[str, Any]] = []
        self._last_current_capture = 0.0
        self._last_detection_capture = 0.0
        self._last_current_latency = 0.0
        self._dequeue_old_next = False
        self._last_map_odom_pose: tuple[float, float, float] | None = None
        self.events_path.parent.mkdir(parents=True, exist_ok=True)
        self.status_path.parent.mkdir(parents=True, exist_ok=True)

        self._stop = threading.Event()
        # ROS 侧状态（provider 的 _map/_pose/_map_bridge）由采集线程独占更新，
        # 推理线程只在调用 camera_point_to_map 的瞬间借用。临界区必须短 ——
        # 推理线程持有它 1.9 秒会把 spin 饿死，反之会让变换用到半更新的位姿。
        self._provider_lock = threading.Lock()
        self._queue_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._events_lock = threading.Lock()
        self._target_watch_lock = threading.RLock()
        self._target_watch_request_lock = threading.Lock()
        # 状态文件的临时文件序号（见 _publish_status：多线程写同一个 tmp 名会撞）。
        self._status_write_seq = 0

        self._queue: deque[Keyframe] = deque()
        self._session_id = 0
        self._last_keyframe_pose: tuple[float, float, float] | None = None
        self._accumulated_yaw_deg = 0.0
        self._last_keyframe_host_s = 0.0
        self._last_yaw_sample: float | None = None
        self._prev_pose_sample: tuple[float, tuple[float, float, float]] | None = None
        self._speed_warned = False

        self._events_handle = None
        self._inference_thread: threading.Thread | None = None
        self._target_watch_thread: threading.Thread | None = None

        self._provider = None
        self._rgbd_source = None
        self._detector = None
        self._localizer = None
        self._semantic_map = None
        self._settings = None
        # §2.3：三维对象的上屏发布器。会话轮换时把 last_revision 清掉，新的
        # （空的）实体表才会被写出去，覆盖掉上一会话留下的对象快照。
        self._object_publisher: Any = None
        self._object_revision = 0
        self._cached_ram_terms: list[str] = []
        self._last_ram_tag_monotonic = 0.0
        self._detect_lock = threading.Lock()
        self._target_watch_spec: Any = None
        self._target_watch_generation = 0
        self._target_watch_last_bad_request = ""
        self._target_watch_revision = 0
        self._target_watch_verifier: Any = None
        self._target_watch_candidates: dict[tuple[int, str], dict[str, Any]] = {}
        self._target_watch_evidence_seq = 0
        self._target_watch_cooldown: Any = None
        self._target_watch_budget: Any = None
        # Object targets configured as "stop after first confirmation" use a
        # strict one-request gate.  The generic verifier supports one pending
        # latest-wins request for hazards, but queueing a second ordinary
        # object here could let it overwrite the first confirmed verdict.
        self._target_watch_object_verify_open_generation: int | None = None
        self._target_watch_state = empty_state(
            "teleop_0", detector_backend=self.detector_backend
        )
        self._target_watch_state["automatic_hazard_enabled"] = bool(
            self._target_watch_config.auto_hazard_enabled
        )
        self._unknown_labeler: Any = None
        self._unknown_label_evidence_seq = 0
        self._unknown_label_reviewed = 0
        self._unknown_label_relabeled = 0
        self._unknown_label_unresolved = 0

        self._launch_log_path = PROJECT_ROOT / "runtime/go2w/sessions/plain_slam_go2w.launch.log"
        self._launch_log_size: int | None = None

        self._status: dict[str, Any] = {
            "state": STATE_LOADING,
            "session_id": 0,
            "pid": os.getpid(),
            "events_path": str(self.events_path),
            "started_at": time.time(),
            "last_attempt_at": None,
            "last_success_at": None,
            "last_frame_id": None,
            "last_detection_count": 0,
            "last_detection_terms": [],
            "last_detection_group": None,
            "last_localized_count": 0,
            "persistent_object_count": 0,
            "queue_depth": 0,
            "captured_keyframes": 0,
            "dropped_keyframes": 0,
            "dropped_redundant_keyframes": 0,
            "processed_keyframes": 0,
            "failed_keyframes": 0,
            "consecutive_failures": 0,
            "skipped_robot_moving_fast": 0,
            "inference_latency_seconds": None,
            "slam_ready": False,
            "quality": None,
            "map_frame": None,
            "clock_skew_seconds": None,
            "last_error": None,
            "last_error_at": None,
            "detector_backend": self.detector_backend,
            "fallback_used": False,
            "unknown_label_vlm_enabled": self._unknown_label_config.enabled,
            "unknown_label_vlm_pending": 0,
            "unknown_label_vlm_cache_entries": 0,
            "unknown_label_vlm_reviewed": 0,
            "unknown_label_vlm_relabeled": 0,
            "unknown_label_vlm_unresolved": 0,
        }

    # ------------------------------------------------------------------ 日志
    def _log(self, message: str) -> None:
        print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)

    def _host_s(self) -> float:
        return round(time.monotonic(), 6)

    # ------------------------------------------------------------ 事件落盘
    def _write_event(self, event: dict[str, Any]) -> None:
        if self._events_handle is None:
            return
        try:
            row = json.loads(json.dumps(event, ensure_ascii=False, allow_nan=False))
        except (TypeError, ValueError) as exc:
            self._log(f"WARNING: 事件含非法值，已丢弃: {exc}")
            return
        with self._events_lock:
            self._events_handle.write(json.dumps(row, ensure_ascii=False) + "\n")
            self._events_handle.flush()

    # -------------------------------------------------------------- 状态文件
    def _publish_status(self, **updates: Any) -> None:
        with self._status_lock:
            self._status.update(updates)
            payload = json.loads(
                json.dumps(self._status, ensure_ascii=False, allow_nan=False)
            )
        # 临时文件名必须**每次都不一样**，不能只用 pid：采集线程和推理线程都会
        # 发状态，两者共用同一个 tmp 名时，先跑完的那个 os.replace 会把文件改走，
        # 后一个就 ENOENT —— 表现为"状态偶尔写不进去"，而 _status_lock 只护住了
        # 内存里的 dict，护不住这两行文件操作。加序号即可，不需要再加一把锁。
        with self._status_lock:
            self._status_write_seq += 1
            seq = self._status_write_seq
        tmp_path = self.status_path.with_suffix(f".json.tmp.{os.getpid()}.{seq}")
        try:
            tmp_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            os.replace(tmp_path, self.status_path)
        except OSError as exc:
            self._log(f"WARNING: 状态写入失败: {exc}")
            try:
                tmp_path.unlink()
            except OSError:
                pass

    def _bump(self, key: str, delta: int = 1) -> None:
        with self._status_lock:
            self._status[key] = int(self._status.get(key, 0)) + delta

    def _fail(self, message: str) -> None:
        with self._status_lock:
            self._status["last_error"] = str(message)[:2000]
            self._status["last_error_at"] = time.time()
            self._status["consecutive_failures"] = (
                int(self._status.get("consecutive_failures", 0)) + 1
            )
            failures = int(self._status["consecutive_failures"])
            self._status["state"] = (
                STATE_DEGRADED
                if failures >= DEGRADED_AFTER_CONSECUTIVE_FAILURES
                else self._status.get("state", STATE_LOADING)
            )
        self._log(f"WARNING: {message}")

    # ---------------------------------------------------------------- 初始化
    def setup(self) -> None:
        from app.config import get_settings
        from app.detectors.factory import create_local_detector
        from app.perception.depth_object_localizer import DepthObjectLocalizer
        from app.perception.realsense_http_rgbd_source import (
            RealSenseHTTPRGBDSource,
        )
        from app.spatial.semantic_object_map import SemanticObjectMap

        self._publish_status(state=STATE_LOADING)
        settings = get_settings()
        self._settings = settings

        self._prepare_detector_env(settings)

        self._rgbd_source = RealSenseHTTPRGBDSource(
            self.args.rgbd_base_url,
            cache_dir=str(PROJECT_ROOT / "runtime/go2w/rgbd_cache"),
        )
        depth_cfg = self.mapping_config.get("hazard", {}).get("depth_localization", {})
        self._localizer = DepthObjectLocalizer(
            pipe_surface_radius_px=int(depth_cfg.get("pipe_surface_radius_px", 18)),
            pipe_surface_min_points=int(depth_cfg.get("pipe_surface_min_points", 8)),
            pipe_surface_max_mad_m=float(depth_cfg.get("pipe_surface_max_mad_m", 0.12)),
        )
        self._semantic_map = self._new_semantic_map()
        # 只订阅 map_2d / odom_base / /tf。enable_ros=True 会在内部建自己的
        # 节点，所以这个进程不再另建第二个节点。
        if self.args.map_mode == "2d":
            from app.spatial.nav2_spatial_provider import Nav2SpatialProvider

            # This worker is a read-only observer and never authorizes motion.
            # Sparse outdoor scenes may fail only the scan-return-count checks;
            # still permit map projection when every map/odom/TF integrity check
            # is green.  The global Nav2 localization gate remains unchanged.
            self._provider = Nav2SpatialProvider(
                enable_ros=True,
                allow_passive_mapping_sparse_scans=True,
            )
        else:
            from app.spatial.plain_slam_spatial_provider import PlainSlamSpatialProvider

            self._provider = PlainSlamSpatialProvider(
                enable_ros=True,
                map_topic="/go2w/slam/map_2d",
                odom_topic="/go2w/slam/odom_base",
            )
        self._events_handle = self.events_path.open("a", encoding="utf-8")
        self._launch_log_size = self._read_launch_log_size()

        self._detector = create_local_detector(self.detector_backend, settings)
        # YOLOE 必须在侧车宣告 ready 前完成进程/CUDA/模型核验；失败即显式
        # 退出，不允许看起来像空场景，也不允许切回 Grounded-SAM。
        if self.detector_backend == "yoloe_seg":
            health = self._detector.health()
            effective_backend = str(health.get("backend") or "")
            if effective_backend != self.detector_backend:
                raise RuntimeError(
                    "detector backend mismatch: "
                    f"requested={self.detector_backend} "
                    f"effective={effective_backend or '<missing>'}"
                )
            self._publish_status(
                detector_backend=effective_backend,
                detector_model_id=health.get("model_id"),
                detector_model_sha256=health.get("model_sha256"),
                detector_library_version=health.get("library_version"),
                detector_device=health.get("device"),
                detector_precision=health.get("precision"),
                detector_prompt_mode=health.get("prompt_mode"),
                detector_class_count=health.get("class_count"),
                fallback_used=False,
            )
        self._setup_target_watch()
        self._setup_unknown_object_vlm()
        self._log(
            "语义侧车就绪："
            f"detector={self.detector_backend} rgbd={self.args.rgbd_base_url} "
            f"events={self.events_path}"
        )

    def _prepare_detector_env(self, settings: Any) -> None:
        """让常驻 GroundedSAM 子进程能 import ``app.*`` 并找到真解释器。

        detector 直接 ``os.environ.copy()`` 给子进程，而它是按绝对路径启动的，
        Python 不会自动把仓库根放进 sys.path —— 不补这一下，子进程里的
        ``from app.perception...`` 会 ImportError。
        """
        parts = [str(PROJECT_ROOT)]
        if self.detector_backend == "yoloe_seg":
            existing = os.environ.get("PYTHONPATH")
            if existing:
                parts.append(existing)
            os.environ["PYTHONPATH"] = os.pathsep.join(
                dict.fromkeys(part for part in parts if part)
            )
            python = Path(str(settings.yoloe_python or "")).expanduser()
            model = Path(str(settings.yoloe_model or "")).expanduser()
            if not python.is_file() or not os.access(python, os.X_OK):
                raise RuntimeError(f"YOLOE_PYTHON 不可执行: {python}")
            if not model.is_file():
                raise RuntimeError(f"YOLOE_MODEL 不存在: {model}")
            prompt_free = model.name.endswith("-seg-pf.pt")
            if not (prompt_free or model.name.endswith("-seg.pt")):
                raise RuntimeError("YOLOE_MODEL 必须是 *-seg.pt 或 *-seg-pf.pt")
            if not prompt_free:
                text_encoder = Path(str(settings.yoloe_text_encoder or "")).expanduser()
                if not text_encoder.is_file():
                    raise RuntimeError(f"YOLOE_TEXT_ENCODER 不存在: {text_encoder}")
            return
        if settings.grounded_sam_pythonpath:
            parts.append(str(settings.grounded_sam_pythonpath))
        existing = os.environ.get("PYTHONPATH")
        if existing:
            parts.append(existing)
        os.environ["PYTHONPATH"] = os.pathsep.join(
            dict.fromkeys(part for part in parts if part)
        )
        python = str(settings.grounded_sam_python or "")
        if not python or not Path(python).is_file():
            # 回退链只收**真的有 torch** 的解释器。以前只判 ``is_file()``，
            # 于是在系统 Python 下会挑中同样存在的 .venv/bin/python —— 那个
            # 环境没有 torch，检测器子进程根本起不来，每一帧都失败：进程活着、
            # 状态显示 degraded、一个物体都不会有，日志里却没有任何一处写着
            # "解释器选错了"。宁可回退到没有候选、让 doctor 明确报错。
            checked: list[str] = []
            for candidate in (
                os.environ.get("GO2W_CONDA_PYTHON"),
                str(PROJECT_ROOT / ".runtime_venv/bin/python"),
                str(PROJECT_ROOT / ".venv/bin/python"),
                sys.executable,
            ):
                if not candidate or not Path(candidate).is_file():
                    continue
                if not _interpreter_has_torch(candidate):
                    checked.append(candidate)
                    continue
                os.environ["GROUNDED_SAM_PYTHON"] = candidate
                settings.grounded_sam_python = candidate
                self._log(f"检测器解释器回退到 {candidate}")
                break
            else:
                raise RuntimeError(
                    "GROUNDED_SAM_PYTHON 无效，且没有可用的回退解释器。"
                    f"已排除（没有 torch）：{checked or '无候选文件'}。"
                    "检查 .env 的 GROUNDED_SAM_PYTHON，"
                    "或确认启动脚本已经把 .env 灌进环境变量。"
                )

    # ------------------------------------------------------ 被动目标监看
    def _setup_target_watch(self) -> None:
        from app.live_robot.async_verifier import AsyncVerifierManager
        from app.perception.passive_target_watch import CandidateCooldown, MinuteBudget

        self.target_watch_request_path.parent.mkdir(parents=True, exist_ok=True)
        self.target_watch_state_path.parent.mkdir(parents=True, exist_ok=True)
        self.target_watch_evidence_dir.mkdir(parents=True, exist_ok=True)
        cfg = self._target_watch_config
        self._target_watch_cooldown = CandidateCooldown(cfg.same_candidate_cooldown_s)
        self._target_watch_budget = MinuteBudget(cfg.max_calls_per_minute)
        self._target_watch_verifier = AsyncVerifierManager(
            self._verify_target_watch,
            enabled=bool(cfg.enabled and cfg.verify_enabled),
            max_inflight=cfg.max_inflight,
            max_pending=cfg.max_pending,
            client_timeout_s=cfg.client_timeout_s,
            event_sink=self._target_verify_event,
        )
        self._target_watch_verifier.set_task_generation(0)
        self._publish_target_watch_state()
        if self.args.smoke_target:
            from app.perception.passive_target_watch import deterministic_spec

            self._apply_target_watch_spec(
                deterministic_spec(self.args.smoke_target), generation=1,
                submitted_at=time.time(), source="smoke_target",
            )

    def _setup_unknown_object_vlm(self) -> None:
        """Start the non-blocking, background-only unknown-object name queue."""
        from app.perception.unknown_object_vlm import AsyncUnknownObjectLabeler

        cfg = self._unknown_label_config
        cache_path = Path(cfg.cache_path)
        if not cache_path.is_absolute():
            cache_path = PROJECT_ROOT / cache_path
        evidence_dir = Path(cfg.evidence_dir)
        if not evidence_dir.is_absolute():
            evidence_dir = PROJECT_ROOT / evidence_dir
        evidence_dir.mkdir(parents=True, exist_ok=True)
        self._unknown_label_evidence_dir = evidence_dir
        self._unknown_labeler = AsyncUnknownObjectLabeler(
            self._verify_unknown_object,
            cfg,
            cache_path=cache_path,
            event_sink=self._unknown_label_event,
        )
        stats = self._unknown_labeler.stats()
        self._publish_status(
            unknown_label_vlm_enabled=cfg.enabled,
            unknown_label_vlm_pending=stats["pending"],
            unknown_label_vlm_cache_entries=stats["cache_entries"],
        )

    def _verify_unknown_object(self, request: Any) -> dict[str, Any]:
        """Use the daemon's background lane to name one detector crop."""
        from app.detectors.siliconflow_vision_protocol import (
            MODE_LABEL,
            PRIORITY_BACKGROUND,
            SiliconFlowDaemonClient,
            VLMRequest,
        )

        cfg = self._unknown_label_config
        socket_path = Path(cfg.socket_path)
        if not socket_path.is_absolute():
            socket_path = PROJECT_ROOT / socket_path
        client = SiliconFlowDaemonClient(
            str(socket_path), timeout=cfg.client_timeout_s
        )
        if not client.available():
            raise ConnectionError(f"VLM daemon socket unavailable: {socket_path}")
        response = client.request(VLMRequest(
            request_id=f"unknown_label_{os.getpid()}_{time.time_ns()}",
            mode=MODE_LABEL,
            image_path=request.image_path,
            frame_id=request.frame_id,
            target="框选区域内的主要实体物体",
            bbox=list(request.bbox),
            priority=PRIORITY_BACKGROUND,
            extra_instructions=(
                "这是异步地图补标任务。请只识别框内主要实体物体，给出简短、"
                "具体、可作为地图节点名称的中文类别名；不要输出颜色、动作、"
                "场景、背景、人体部位或门把手等零部件。无法可靠识别时令 "
                "is_target=false，不要猜测。检测器原始类别仅供弱参考："
                f"{request.raw_label!r}。"
            ),
        ))
        if not response.ok:
            raise RuntimeError(response.error or "VLM daemon returned failure")
        if response.frame_id and str(response.frame_id) != str(request.frame_id):
            raise RuntimeError("VLM response frame_id mismatch")
        payload = dict(response.payload or {})
        payload.setdefault("provider", payload.get("_provider") or "vlm_background")
        payload.setdefault("latency_ms", response.latency_ms)
        return payload

    def _unknown_label_event(self, event: dict[str, Any]) -> None:
        self._write_event({
            **event,
            "session_id": self._mapping_session_id(),
            "backend": self.detector_backend,
            "host_s": self._host_s(),
        })

    def _save_unknown_label_evidence(
        self, source: Path, *, frame_id: str, raw_label: str
    ) -> Path | None:
        if not source.is_file():
            return None
        self._unknown_label_evidence_seq += 1
        sequence = self._unknown_label_evidence_seq
        safe_frame = "".join(ch for ch in str(frame_id) if ch.isalnum())[:24] or "frame"
        destination = self._unknown_label_evidence_dir / (
            f"uvl_{self._session_id}_{safe_frame}_{sequence}.jpg"
        )
        temporary = destination.with_suffix(".jpg.tmp")
        try:
            shutil.copyfile(source, temporary)
            os.replace(temporary, destination)
        except OSError as exc:
            self._log(f"WARNING: 未分类物体 VLM 证据帧保存失败: {exc}")
            temporary.unlink(missing_ok=True)
            return None
        self._prune_unknown_label_evidence()
        return destination

    def _prune_unknown_label_evidence(self) -> None:
        files = sorted(
            self._unknown_label_evidence_dir.glob("uvl_*.jpg"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        for path in files[self._unknown_label_config.evidence_max_files:]:
            try:
                path.unlink()
            except OSError:
                pass

    def _queue_unknown_object_labels(
        self,
        *,
        frame_path: Path,
        frame_id: str,
        scene_objects: list[dict[str, Any]],
        persistent_by_source_id: dict[str, Any],
    ) -> None:
        from app.detectors.vocabulary import LABEL_ZH_FALLBACK

        labeler = self._unknown_labeler
        if labeler is None:
            return
        queued = 0
        for item in scene_objects:
            if str(item.get("label_zh") or "").strip() != LABEL_ZH_FALLBACK:
                continue
            raw_label = str(item.get("label") or "").strip()
            source_id = str(item.get("source_object_id") or item.get("id") or "")
            association = persistent_by_source_id.get(source_id)
            if association is None:
                continue
            review_key = (
                f"{self._mapping_session_id()}:{self._map_epoch}:"
                f"{association.persistent_object_id}"
            )
            if not labeler.needs_review(review_key):
                continue
            bbox = item.get("bbox_2d") or item.get("bbox")
            evidence = self._save_unknown_label_evidence(
                frame_path, frame_id=frame_id, raw_label=raw_label
            )
            if evidence is None:
                continue
            if labeler.submit(
                review_key=review_key,
                raw_label=raw_label,
                frame_id=frame_id,
                object_id=association.persistent_object_id,
                image_path=str(evidence),
                bbox=bbox,
            ):
                queued += 1
            else:
                evidence.unlink(missing_ok=True)
        if queued:
            stats = labeler.stats()
            self._publish_status(
                unknown_label_vlm_pending=stats["pending"],
                unknown_label_vlm_cache_entries=stats["cache_entries"],
            )

    def _bind_current_detection_labels(
        self,
        current: list[dict[str, Any]],
        persistent_by_source_id: dict[str, Any],
    ) -> None:
        """Bind live boxes to map identities and reuse reviewed display names.

        ``current`` and ``SemanticObjectMap`` are deliberately separate output
        models.  The VLM reviewer updates the latter, so without this bridge the
        video overlay keeps showing the detector's fallback forever even though
        the matching map node already has a reviewed Chinese name.
        """
        from app.detectors.vocabulary import LABEL_ZH_FALLBACK

        review_enabled = self._unknown_labeler is not None
        for item in current:
            source_id = str(item.get("source_object_id") or "")
            association = persistent_by_source_id.get(source_id)
            if association is None:
                continue
            persistent_id = str(association.persistent_object_id)
            item["persistent_object_id"] = persistent_id
            entry = self._semantic_map.objects.get(persistent_id)
            if entry is None:
                continue

            reviewed_label = str(getattr(entry, "label_zh", "") or "").strip()
            if reviewed_label and reviewed_label != LABEL_ZH_FALLBACK:
                provenance = dict(getattr(entry, "provenance", None) or {})
                item["label_zh"] = reviewed_label
                item["label_review_state"] = "resolved"
                item["label_source"] = str(
                    provenance.get("label_source") or "semantic_map"
                )
                if provenance.get("label_vlm_confidence") is not None:
                    item["label_vlm_confidence"] = provenance[
                        "label_vlm_confidence"
                    ]
                continue

            if (
                review_enabled
                and str(item.get("label_zh") or "").strip()
                == LABEL_ZH_FALLBACK
            ):
                raw_label = " ".join(str(item.get("label") or "").split())
                item["raw_label"] = raw_label
                item["label_zh"] = (
                    f"{UNKNOWN_LABEL_REVIEW_PENDING} · {raw_label}"
                    if raw_label else UNKNOWN_LABEL_REVIEW_PENDING
                )
                item["label_review_state"] = "pending"
                item["label_source"] = "yoloe_prompt_free_pending_vlm"

    def _apply_unknown_label_results(self) -> None:
        """Apply completed VLM names on the inference thread, never concurrently."""
        from app.detectors.vocabulary import LABEL_ZH_FALLBACK
        if self._unknown_labeler is None:
            return
        results = self._unknown_labeler.poll_completed()
        if not results:
            return
        changed = 0
        for result in results:
            self._unknown_label_reviewed += 1
            if not result.accepted:
                self._unknown_label_unresolved += 1
                continue
            current_review_key = (
                f"{self._mapping_session_id()}:{self._map_epoch}:{result.object_id}"
            )
            entry = self._semantic_map.objects.get(result.object_id)
            if result.review_key != current_review_key or entry is None:
                self._write_event({
                    "event": "unknown_object_vlm_stale_result",
                    "review_key": result.review_key,
                    "object_id": result.object_id,
                    "frame_id": result.frame_id,
                    "host_s": self._host_s(),
                })
                continue
            if str(entry.label_zh or "").strip() != LABEL_ZH_FALLBACK:
                continue
            entry.label_zh = str(result.label_zh)
            entry.provenance = {
                **dict(entry.provenance or {}),
                "label_source": "vlm_background_review",
                "label_vlm_confidence": result.confidence,
                "label_vlm_provider": result.provider,
                "label_vlm_frame_id": result.frame_id,
                "label_vlm_raw_label": result.raw_label,
            }
            # The latest camera overlay is a detached snapshot, not a view of
            # ``entry``.  Patch every box bound to this persistent identity so
            # the reviewed name appears immediately, even while the robot and
            # camera remain stationary and no new detection frame is produced.
            for item in self._current_detections:
                if str(item.get("persistent_object_id") or "") != result.object_id:
                    continue
                item["label_zh"] = str(result.label_zh)
                item["label_review_state"] = "resolved"
                item["label_source"] = "vlm_background_review"
                item["label_vlm_confidence"] = result.confidence
            changed += 1
            self._write_event({
                "event": "semantic_object_vlm_relabeled",
                "object_id": entry.object_id,
                "raw_label": entry.label,
                "label_zh": entry.label_zh,
                "confidence": result.confidence,
                "provider": result.provider,
                "frame_id": result.frame_id,
                "reason_zh": result.reason_zh,
                "host_s": self._host_s(),
            })
        self._unknown_label_relabeled += changed
        if changed:
            self._semantic_map.revision += 1
            self._object_revision += 1
            self._publish_display_objects(self._object_revision)
            self._publish_topology(
                self._current_detections, self._last_current_latency
            )
        stats = self._unknown_labeler.stats()
        self._publish_status(
            unknown_label_vlm_pending=stats["pending"],
            unknown_label_vlm_cache_entries=stats["cache_entries"],
            unknown_label_vlm_reviewed=self._unknown_label_reviewed,
            unknown_label_vlm_relabeled=self._unknown_label_relabeled,
            unknown_label_vlm_unresolved=self._unknown_label_unresolved,
        )

    def _mapping_session_id(self) -> str:
        return f"teleop_{self._session_id}"

    def _publish_target_watch_state(self, **updates: Any) -> None:
        from app.perception.passive_target_watch import atomic_write_json

        with self._target_watch_lock:
            self._target_watch_state.update(updates)
            self._target_watch_revision += 1
            self._target_watch_state.update({
                "schema_version": 1,
                "mapping_session_id": self._mapping_session_id(),
                "generation": self._target_watch_generation,
                "revision": self._target_watch_revision,
                "detector_backend": self.detector_backend,
                "updated_at": time.time(),
            })
            payload = copy.deepcopy(self._target_watch_state)
        try:
            atomic_write_json(self.target_watch_state_path, payload)
        except OSError as exc:
            self._log(f"WARNING: 目标监看状态写入失败: {exc}")

    def _target_watch_snapshot(self) -> dict[str, Any]:
        with self._target_watch_lock:
            return copy.deepcopy(self._target_watch_state)

    def _target_stat(self, key: str, delta: int = 1) -> None:
        with self._target_watch_lock:
            stats = self._target_watch_state.setdefault("statistics", {})
            stats[key] = int(stats.get(key, 0)) + int(delta)

    def _sync_target_watch_request(self) -> None:
        if not self._target_watch_request_lock.acquire(blocking=False):
            return
        try:
            self._sync_target_watch_request_locked()
        finally:
            self._target_watch_request_lock.release()

    def _sync_target_watch_request_locked(self) -> None:
        from app.perception.passive_target_watch import (
            TargetWatchSpec,
            load_json_object,
            validate_request,
        )

        if not self._target_watch_config.enabled:
            return
        raw = load_json_object(self.target_watch_request_path)
        if raw is None:
            return
        try:
            request = validate_request(raw, session_id=self._mapping_session_id())
        except (TypeError, ValueError) as exc:
            fingerprint = f"{type(exc).__name__}:{exc}:{raw.get('generation')}"
            if fingerprint != self._target_watch_last_bad_request:
                self._target_watch_last_bad_request = fingerprint
                self._write_event({
                    "event": "target_watch_degraded",
                    "reason": "invalid_request",
                    "error": str(exc),
                    "session_id": self._mapping_session_id(),
                    "generation": self._target_watch_generation,
                    "backend": self.detector_backend,
                    "host_s": self._host_s(),
                })
                self._publish_target_watch_state(last_error=f"request invalid: {exc}")
            return
        generation = int(request["generation"])
        if generation <= self._target_watch_generation:
            return
        if request["command"] == "clear":
            self._clear_target_watch(generation=generation, source="request")
            return
        spec = TargetWatchSpec.from_dict(request["profile"])
        self._apply_target_watch_spec(
            spec, generation=generation,
            submitted_at=float(request.get("submitted_at") or time.time()),
            source="request",
        )

    def _apply_target_watch_spec(
        self, spec: Any, *, generation: int, submitted_at: float, source: str
    ) -> None:
        from app.perception.passive_target_watch import (
            TargetWatchSpec,
            target_profile_from_spec,
        )

        cfg = self._target_watch_config
        runtime_spec = TargetWatchSpec.from_dict({
            **spec.to_dict(),
            "target_terms_en": list(spec.target_terms_en[: cfg.max_target_terms]),
            "anchor_terms_en": list(spec.anchor_terms_en[: cfg.max_anchor_terms]),
        })
        with self._detect_lock:
            self._detector.configure_target(target_profile_from_spec(runtime_spec))
        with self._target_watch_lock:
            self._target_watch_spec = runtime_spec
            self._target_watch_generation = int(generation)
            self._target_watch_object_verify_open_generation = None
            self._target_watch_candidates.clear()
            self._target_watch_state = {
                **self._target_watch_state,
                "enabled": True,
                "state": "ARMED",
                "raw_query": spec.raw_query,
                "profile": spec.to_dict(),
                "candidate_count": 0,
                "last_candidate": None,
                "verification": None,
                "confirmed_matches": [],
                "last_error": None,
                "configured_at": submitted_at,
            }
        self._target_watch_cooldown.clear()
        self._target_watch_verifier.set_task_generation(generation)
        self._publish_target_watch_state()
        self._write_event({
            "event": "target_watch_configured",
            "source": source,
            "session_id": self._mapping_session_id(),
            "generation": generation,
            "raw_query": spec.raw_query,
            "profile": spec.to_dict(),
            "backend": self.detector_backend,
            "host_s": self._host_s(),
        })

    def _clear_target_watch(self, *, generation: int, source: str) -> None:
        with self._detect_lock:
            self._detector.configure_target(None)
        with self._target_watch_lock:
            self._target_watch_spec = None
            self._target_watch_generation = int(generation)
            self._target_watch_object_verify_open_generation = None
            self._target_watch_candidates.clear()
            stats = copy.deepcopy(self._target_watch_state.get("statistics") or {})
            self._target_watch_state = {
                "enabled": False,
                "automatic_hazard_enabled": bool(
                    self._target_watch_config.auto_hazard_enabled
                ),
                "state": "IDLE",
                "raw_query": "",
                "profile": None,
                "candidate_count": 0,
                "last_candidate": None,
                "verification": None,
                "confirmed_matches": [],
                "statistics": stats,
                "last_error": None,
            }
        self._target_watch_cooldown.clear()
        self._target_watch_verifier.set_task_generation(generation)
        self._publish_target_watch_state()
        self._write_event({
            "event": "target_watch_cleared", "source": source,
            "session_id": self._mapping_session_id(), "generation": generation,
            "backend": self.detector_backend, "host_s": self._host_s(),
        })

    def _verify_target_watch(
        self, image_path: str, bbox: list[float], env: dict[str, Any], *, frame_id: str
    ) -> dict[str, Any]:
        from app.detectors.siliconflow_vision_protocol import (
            MODE_VERIFY,
            PRIORITY_REALTIME,
            SiliconFlowDaemonClient,
            VLMRequest,
        )

        socket_path = Path(self._target_watch_config.socket_path)
        if not socket_path.is_absolute():
            socket_path = PROJECT_ROOT / socket_path
        client = SiliconFlowDaemonClient(
            str(socket_path), timeout=self._target_watch_config.client_timeout_s
        )
        if not client.available():
            raise ConnectionError(f"VLM daemon socket unavailable: {socket_path}")
        profile = env.get("profile") or {}
        request = VLMRequest(
            request_id=f"target_watch_{os.getpid()}_{time.time_ns()}",
            mode=MODE_VERIFY,
            image_path=image_path,
            frame_id=str(frame_id),
            target=str(env.get("raw_query") or ""),
            bbox=list(bbox),
            priority=PRIORITY_REALTIME,
            extra_instructions=(
                "严格同时核验核心类别、属性、关系和否定约束。"
                f"目标画像={json.dumps(profile, ensure_ascii=False)}。"
                + (
                    "这是自动危险源复核：重点判断框内黏稠/荧光物是否附着在管道"
                    "表面或呈现从管道渗漏、污染的空间关系；普通绿色物体、植物、"
                    "墙面或反光必须判为非目标。"
                    if env.get("automatic_hazard") else ""
                )
                + "不确定时不要给肯定结论。"
            ),
            robot_pose=env.get("robot_pose"),
        )
        response = client.request(request)
        if not response.ok:
            raise RuntimeError(response.error or "VLM daemon returned failure")
        if response.frame_id and str(response.frame_id) != str(frame_id):
            raise RuntimeError("VLM response frame_id mismatch")
        payload = dict(response.payload or {})
        payload.setdefault(
            "provider", str(payload.get("_provider") or "siliconflow_daemon")
        )
        payload.setdefault("latency_ms", response.latency_ms)
        confidence = payload.get("confidence")
        try:
            confidence_value = float(confidence)
        except (TypeError, ValueError):
            confidence_value = 0.0
        if payload.get("is_target") is True and confidence_value < self._target_watch_config.confirmation_min_confidence:
            payload.pop("is_target", None)
            payload["reason_zh"] = (
                str(payload.get("reason_zh") or "")
                + "；置信度低于确认门槛，按 UNKNOWN 处理"
            ).strip("；")
        return payload

    def _target_verify_event(self, event: dict[str, Any]) -> None:
        mapped = {
            "verify_request_started": "target_watch_verification_started",
            "verify_request_discarded": "target_watch_candidate_suppressed",
            "verify_result_discarded": "target_watch_stale_result_discarded",
            "verify_client_timeout": "target_watch_unknown",
            "verify_result_completed": "target_watch_verification_finished",
            "verify_generation_advanced": "target_watch_generation_advanced",
        }.get(str(event.get("event")), str(event.get("event")))
        enriched = {
            **event, "event": mapped,
            "session_id": self._mapping_session_id(),
            "generation": event.get("task_generation", self._target_watch_generation),
            "backend": self.detector_backend, "host_s": self._host_s(),
        }
        if mapped == "target_watch_verification_started":
            key = (int(enriched["generation"]), str(event.get("candidate_id") or ""))
            with self._target_watch_lock:
                candidate = copy.deepcopy(self._target_watch_candidates.get(key) or {})
                self._target_stat("verify_started")
                self._target_watch_state["state"] = "VERIFYING"
                self._target_watch_state["verification"] = {
                    "verification_id": candidate.get("verification_id"),
                    "sequence": event.get("sequence"),
                    "status": "PENDING",
                    "automatic_hazard": bool(
                        candidate.get("automatic_hazard")
                    ),
                    "canonical_name_zh": candidate.get("canonical_name_zh"),
                    "provider": None, "confidence": None, "reason_zh": "",
                    "latency_ms": None, "unknown_reason": None,
                    "evidence_path": candidate.get("evidence_path"),
                }
                for item in self._current_detections:
                    if item.get("candidate_id") == event.get("candidate_id"):
                        item["target_watch_status"] = "VERIFYING"
            self._publish_target_watch_state()
        elif mapped == "target_watch_stale_result_discarded":
            self._target_stat("stale_results_discarded")
            self._publish_target_watch_state()
        elif mapped == "target_watch_candidate_suppressed" and event.get("candidate_id"):
            with self._target_watch_lock:
                self._target_watch_candidates.pop(
                    (int(enriched["generation"]), str(event["candidate_id"])), None
                )
        self._write_event(enriched)

    def _target_watch_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._sync_target_watch_request()
                for result in self._target_watch_verifier.poll_completed():
                    self._apply_target_watch_result(result)
            except Exception as exc:  # noqa: BLE001
                self._log(f"WARNING: 目标监看轮询失败: {exc}")
            self._stop.wait(0.10)

    def _apply_target_watch_result(self, result: Any) -> None:
        generation = int(result.task_generation)
        key = (generation, str(result.candidate_id))
        with self._target_watch_lock:
            if generation != self._target_watch_generation:
                self._target_stat("stale_results_discarded")
                return
            candidate = copy.deepcopy(self._target_watch_candidates.get(key) or {})
            if (
                self._target_watch_object_verify_open_generation == generation
                and not candidate.get("automatic_hazard")
            ):
                self._target_watch_object_verify_open_generation = None
            details = dict(result.details or {})
            verification = {
                "verification_id": candidate.get("verification_id"),
                "sequence": result.sequence,
                "status": result.status,
                "automatic_hazard": bool(candidate.get("automatic_hazard")),
                "canonical_name_zh": candidate.get("canonical_name_zh"),
                "provider": details.get("provider"),
                "confidence": details.get("confidence"),
                "object_name_zh": details.get("object_name_zh"),
                "reason_zh": result.reason_zh,
                "latency_ms": details.get("latency_ms", result.latency_ms),
                "unknown_reason": result.unknown_reason or None,
                "evidence_path": candidate.get("evidence_path"),
            }
            self._target_watch_state["verification"] = verification
            self._target_watch_state["state"] = result.status
            self._target_watch_state["automatic_hazard_active"] = bool(
                candidate.get("automatic_hazard")
            )
            stat_key = {
                "CONFIRMED": "verify_confirmed",
                "REJECTED": "verify_rejected",
                "UNKNOWN": "verify_unknown",
            }[result.status]
            self._target_stat(stat_key)
            if result.status == "CONFIRMED":
                match = {**candidate, **verification}
                matches = self._target_watch_state.setdefault("confirmed_matches", [])
                matches.append(match)
                keep = min(20, self._target_watch_config.evidence_max_files)
                del matches[:-keep]
            self._target_watch_candidates.pop(key, None)
            for item in self._current_detections:
                if item.get("candidate_id") == result.candidate_id:
                    item["target_watch_status"] = result.status
                    item["is_target_confirmed"] = result.status == "CONFIRMED"
        self._publish_target_watch_state()
        self._prune_target_evidence()
        event_name = {
            "CONFIRMED": "target_watch_confirmed",
            "REJECTED": "target_watch_rejected",
            "UNKNOWN": "target_watch_unknown",
        }[result.status]
        self._write_event({
            "event": event_name, "session_id": self._mapping_session_id(),
            "generation": generation, "frame_id": result.frame_id,
            "candidate_id": result.candidate_id, "sequence": result.sequence,
            "status": result.status, "verification": verification,
            "automatic_hazard": bool(candidate.get("automatic_hazard")),
            "backend": self.detector_backend, "host_s": self._host_s(),
        })

    def _new_semantic_map(self) -> Any:
        from app.spatial.semantic_object_map import SemanticObjectMap

        return SemanticObjectMap(
            merge_distance_m=SEMANTIC_MAP_MERGE_DISTANCE_M,
            confirm_min_observations=SEMANTIC_MAP_CONFIRM_MIN_OBSERVATIONS,
            stale_after_seconds=float(self.mapping_config.get("mobility", {}).get("stale_after_s", 300)),
        )

    # ------------------------------------------------------------ 会话轮换
    def _read_launch_log_size(self) -> int | None:
        try:
            return self._launch_log_path.stat().st_size
        except OSError:
            return None

    def _check_session_rotation(self) -> None:
        """LIO launch 日志被截断 == ``start_plain_slam_mapping.sh`` 换会话。

        pslam_map 在新会话里可能重新定义，旧坐标系的物体会全部错位，所以必须
        清空实体图而不是继续合并。三态：文件读不到就**不轮换**（不动），
        只把大小记成 None —— 不能把"读不到"当成"轮换了"，那会在 SLAM 日志
        权限出问题时把整张图反复清空。
        """
        if self.args.map_mode != "3d":
            return
        size = self._read_launch_log_size()
        if size is None:
            return
        previous = self._launch_log_size
        self._launch_log_size = size
        if previous is None or size >= previous:
            return
        self._rotate_session(reason="lio_launch_log_truncated")

    def _publish_display_objects(self, revision: int) -> None:
        """§2.3：实体图更新后把内存对象投影上屏（与自主搜索同一份投影函数）。

        拿不到地图桥的 ``map_epoch`` 时什么都不做 —— 对象坐标属于哪张地图无从
        声明就不该画，这是正确行为而不是失败。
        """
        if self._semantic_map is None:
            return
        if self._object_publisher is None:
            from app.spatial.object_display_snapshot import ObjectDisplayPublisher

            self._object_publisher = ObjectDisplayPublisher(
                meta_path=PROJECT_ROOT / str(self.args.map_meta_path),
                snapshot_path=PROJECT_ROOT / str(self.args.object_snapshot_path),
                producer_session_id=f"teleop_{self._session_id}",
                confirmed_min_confidence=(
                    self._settings.semantic_map_min_object_confidence
                    if self.args.map_mode == "2d" else None
                ),
                confirm_min_observations=(
                    SEMANTIC_MAP_CONFIRM_MIN_OBSERVATIONS
                    if self.args.map_mode == "2d" else None
                ),
                topology_accepted_only=self.args.map_mode == "2d",
            )
        self._object_publisher.publish(
            (entry.to_dict() for entry in self._semantic_map.objects.values()),
            graph_revision=revision,
        )

    def _update_place_graph(self, keyframe: Keyframe, update_result: Any) -> None:
        """Sample a truthful, sparse trajectory skeleton for the integrated view."""
        if not self.mapping_config.get("topology", {}).get("integrated_place_view", True):
            return
        pose_value = keyframe.robot_pose or {}
        if not all(_finite(pose_value.get(name)) for name in ("x", "y", "yaw")):
            return
        pose_tuple = (
            float(pose_value["x"]), float(pose_value["y"]), float(pose_value["yaw"])
        )
        previous = self._last_place_sample_pose
        distance = (
            math.hypot(pose_tuple[0] - previous[0], pose_tuple[1] - previous[1])
            if previous else 0.0
        )
        turn_deg = (
            abs(math.degrees(_wrap_pi(pose_tuple[2] - previous[2]))) if previous else 0.0
        )
        topology = self.mapping_config.get("topology", {})
        if (
            self._last_place_pose_at is not None
            and keyframe.capture_timestamp - self._last_place_pose_at
            > float(topology.get("place_max_continuity_gap_s", 20.0))
        ):
            self._place_graph.break_continuity()
            self._last_place_sample_pose = None
            previous = None
            distance = 0.0
            turn_deg = 0.0
        self._last_place_pose_at = keyframe.capture_timestamp
        sample = (
            previous is None
            or distance >= float(topology.get("place_sample_distance_m", 8.0))
            or turn_deg >= float(topology.get("place_turn_sample_deg", 35.0))
        )
        if sample:
            from app.spatial.models import SpatialPose
            place_id, _ = self._place_graph.register_observation(
                observation_id=keyframe.frame_id,
                heading_sector=int(round(math.degrees(pose_tuple[2]) / 30.0)) % 12,
                objects=[],
                rgbd_frame_id=keyframe.frame_id,
                pose=SpatialPose(
                    x=pose_tuple[0], y=pose_tuple[1], yaw=pose_tuple[2],
                    frame_id=str(self._map_epoch or "map"),
                    quality="METRIC_RGBD", source="robot_pose_at_capture",
                ),
                observed_displacement_m=distance if previous is not None else None,
                timestamp=keyframe.capture_timestamp,
            )
            self._last_place_sample_pose = pose_tuple
        else:
            place = self._place_graph.current_place()
            place_id = place.place_id if place is not None else None
        if place_id:
            self._place_graph.attach_objects(
                place_id,
                [assoc.persistent_object_id for assoc in update_result.associations],
            )

    def _integrated_topology_snapshot(self, object_graph: dict[str, Any]) -> dict[str, Any]:
        """Add PLACE nodes without changing the pure object-topology contract."""
        nodes = [dict(item) for item in object_graph.get("nodes", [])]
        edges = [dict(item) for item in object_graph.get("edges", [])]
        object_ids = {str(item.get("node_id") or "") for item in nodes}
        for place in self._place_graph.places.values():
            value = place.to_dict()
            pose = value.get("pose") or {}
            nodes.append({
                "node_id": place.place_id,
                "node_type": "PLACE",
                "label": "观察点",
                "label_zh": "观察点",
                "x": pose.get("x"),
                "y": pose.get("y"),
                "pose": pose,
                "visit_count": place.visit_count,
                "revisited": place.revisited,
            })
            for object_id in place.observed_object_ids:
                if object_id in object_ids:
                    edges.append({
                        "edge_id": f"{place.place_id}__observed_from__{object_id}",
                        "source": place.place_id,
                        "target": object_id,
                        "from": place.place_id,
                        "to": object_id,
                        "relation": "OBSERVED_FROM",
                        "traversable": False,
                    })
        for edge in self._place_graph.edges:
            edges.append({
                "edge_id": edge.edge_id,
                "source": edge.from_place,
                "target": edge.to_place,
                "from": edge.from_place,
                "to": edge.to_place,
                "relation": "CONNECTED_TO",
                "traversable": edge.status not in {"BLOCKED", "STALE"},
                "observed_displacement_m": edge.observed_displacement_m,
            })
        return {
            "schema_version": "integrated_place_object_topology_v1",
            "nodes": nodes,
            "edges": edges,
            "stats": {"node_count": len(nodes), "edge_count": len(edges)},
        }

    def _publish_topology(self, detections: list[dict[str, Any]], latency: float) -> None:
        from app.spatial.object_display_snapshot import (
            load_object_snapshot,
            write_object_snapshot,
        )
        from app.spatial.object_relation_store import ObjectRelationStore
        if self._relations is None:
            self._relations = ObjectRelationStore()
        # Both object views carry the same semantic/entity revision.  Polling,
        # status refreshes and failed inference must not create a topology-only
        # version that never existed in the object overlay.
        self._topology_revision = self._object_revision
        display_snapshot = load_object_snapshot(self.args.object_snapshot_path)
        display_ids = set()
        if (display_snapshot
                and display_snapshot.get("map_epoch") == self._map_epoch
                and int(display_snapshot.get("graph_revision") or -1) == self._object_revision):
            display_ids = {
                str(item.get("object_id") or "")
                for item in display_snapshot.get("objects", [])
                if item.get("object_id")
            }
        topology_objects = {
            object_id: entry
            for object_id, entry in (
                self._semantic_map.objects.items()
                if self._semantic_map and self._map_epoch else []
            )
            if object_id in display_ids
        }
        graph = self._relations.object_topology_snapshot(
            topology_objects,
            revision=self._topology_revision,
            near_distance_m=float(self.mapping_config.get("topology", {}).get("near_distance_m", 3.0)))
        graph["generated_at"] = time.time()
        watch_snapshot = self._target_watch_snapshot()
        for match in watch_snapshot.get("confirmed_matches", []):
            xyz = match.get("map_xyz")
            provenance = match.get("provenance") or {}
            if (
                not isinstance(xyz, list)
                or len(xyz) != 3
                or not all(_finite(value) for value in xyz)
                or not self._map_epoch
                or str(provenance.get("map_epoch") or "") != self._map_epoch
            ):
                continue
            graph["nodes"].append({
                "node_id": str(match.get("verification_id") or match.get("candidate_id")),
                "node_type": "OBJECT",
                "label": str(watch_snapshot.get("raw_query") or match.get("matched_term") or "target"),
                "label_zh": str(watch_snapshot.get("raw_query") or "已确认目标"),
                "x": float(xyz[0]), "y": float(xyz[1]), "z": float(xyz[2]),
                "map_xyz": list(xyz),
                "status": "CONFIRMED", "lifecycle_state": "active",
                "mobility_class": "fixed", "is_target_confirmed": True,
                "target_watch_only": True,
                "confidence": match.get("confidence"),
                "spatial_quality": match.get("spatial_quality"),
            })
        if watch_snapshot.get("confirmed_matches"):
            graph["stats"]["node_count"] = len(graph["nodes"])
        evidence_events = {event["hazard_event_id"]: event for event in self._hazards.events.values()}
        for node in graph["nodes"]:
            evidence = evidence_events.get(node.get("hazard_event_id"), {}).get("evidence_path")
            node["hazard_evidence_available"] = bool(evidence and Path(evidence).is_file())
        with self._queue_lock:
            queued = len(self._queue)
        integrated = self._integrated_topology_snapshot(graph)
        snapshot = {
            "session_id": f"teleop_{self._session_id}",
            "map_epoch": self._map_epoch,
            "generated_at": time.time(),
            "revision": self._topology_revision,
            "entity_revision": self._object_revision,
            "graph": {"object_topology": graph,
                      "integrated_topology": integrated,
                      "objects": [entry.to_dict() for entry in topology_objects.values()]},
            "current_detections": detections,
            "last_detection_at": self._last_detection_capture or None,
            "frame_available": self._frame_available,
            "hazard_events": sorted(self._hazards.events.values(), key=lambda e: e["last_seen"], reverse=True)[:30],
            "processing_latency_seconds": round(latency, 3),
            "sound_cooldown_s": float(self.mapping_config.get("hazard", {}).get("repeat_sound_cooldown_s", 30)),
            "queue_depth": queued,
            "last_error": self._status.get("last_error"),
            "target_watch": watch_snapshot,
        }
        write_object_snapshot(self.topology_path, snapshot)

    def _rotate_session(self, *, reason: str) -> None:
        self._session_id += 1
        self._reset_target_watch_session(reason=reason)
        if self._object_publisher is not None:
            # 换会话后实体表是空的，版本号回到 0；不清掉上次的版本号，空表不会
            # 被写出去，页面上会继续显示上一张地图的物体。
            self._object_publisher.last_revision = None
        self._object_revision = 0
        with self._queue_lock:
            dropped = len(self._queue)
            self._queue.clear()
        self._semantic_map = self._new_semantic_map()
        from app.spatial.object_relation_store import ObjectRelationStore
        from app.spatial.place_graph import PlaceGraph
        self._relations = ObjectRelationStore()
        topology_cfg = self.mapping_config.get("topology", {})
        self._place_graph = PlaceGraph(
            merge_distance_m=float(topology_cfg.get("place_merge_distance_m", 2.0)),
            relocation_min_displacement_m=float(topology_cfg.get("place_sample_distance_m", 8.0)),
        )
        self._last_place_sample_pose = None
        self._last_place_pose_at = None
        self._mobility.reset()
        self._hazards.reset()
        self._pipe_tracker.reset()
        self._current_detections = []
        self._map_epoch = ""
        self._last_current_capture = 0.0
        self._last_detection_capture = 0.0
        self._frame_available = False
        self._frame_path.unlink(missing_ok=True)
        self._last_current_latency = 0.0
        self._dequeue_old_next = False
        self._last_map_odom_pose = None
        self._last_keyframe_pose = None
        self._accumulated_yaw_deg = 0.0
        self._last_keyframe_host_s = 0.0
        self._last_yaw_sample = None
        self._prev_pose_sample = None
        self._write_event({
            "event": "semantic_session_reset",
            "reason": reason,
            "session_id": self._session_id,
            "discarded_keyframes": dropped,
            "host_s": self._host_s(),
        })
        self._publish_status(
            session_id=self._session_id,
            persistent_object_count=0,
            queue_depth=0,
        )
        self._publish_topology([], 0.0)
        self._log(f"SLAM 换会话 → 语义实体图已清空（丢弃 {dropped} 张待推理关键帧）")

    def _reset_target_watch_session(self, *, reason: str) -> None:
        from app.perception.passive_target_watch import empty_state

        if self._detector is not None:
            with self._detect_lock:
                self._detector.configure_target(None)
        with self._target_watch_lock:
            self._target_watch_spec = None
            self._target_watch_generation = 0
            self._target_watch_object_verify_open_generation = None
            self._target_watch_candidates.clear()
            self._target_watch_state = empty_state(
                self._mapping_session_id(), detector_backend=self.detector_backend
            )
            self._target_watch_state["automatic_hazard_enabled"] = bool(
                self._target_watch_config.auto_hazard_enabled
            )
        if self._target_watch_cooldown is not None:
            self._target_watch_cooldown.clear()
        if self._target_watch_verifier is not None:
            self._target_watch_verifier.set_task_generation(0)
        self._publish_target_watch_state(last_error=None)
        self._write_event({
            "event": "target_watch_cleared", "reason": f"session_reset:{reason}",
            "session_id": self._mapping_session_id(), "generation": 0,
            "backend": self.detector_backend, "host_s": self._host_s(),
        })

    # ---------------------------------------------------------- 采帧判定
    def _current_speed(self, pose_xyz_yaw: tuple[float, float, float]) -> tuple[float, float]:
        """用相邻两个 poll 周期的位姿差估速度（m/s, deg/s）。

        provider 的 SpatialPose 不带 twist，为这个可选门再加一条 odom 订阅
        不值得 —— 位姿差分对"是不是在正常走"这个粗判够用。

        窗口固定成一个 poll 周期（而不是"上次位姿变化至今"）：后者在静止一段
        时间后突然起步时会把 dt 算成整段静止时长，从而**低估**速度、放行本该
        跳过的高速帧。位姿没变时距离≈0，下面的门限自然不会触发。
        """
        now = time.monotonic()
        previous = self._prev_pose_sample
        self._prev_pose_sample = (now, pose_xyz_yaw)
        if previous is None:
            return (0.0, 0.0)
        prev_t, (px, py, pyaw) = previous
        dt = now - prev_t
        if dt <= 1e-3:
            return (0.0, 0.0)
        distance = math.hypot(pose_xyz_yaw[0] - px, pose_xyz_yaw[1] - py)
        turned_deg = abs(math.degrees(_wrap_pi(pose_xyz_yaw[2] - pyaw)))
        if distance < 1e-4 and turned_deg < 0.5:
            return (0.0, 0.0)
        return (distance / dt, turned_deg / dt)

    def _should_capture(self, pose_xyz_yaw: tuple[float, float, float]) -> bool:
        if self._last_keyframe_pose is None:
            return True
        previous = self._last_keyframe_pose
        distance = math.hypot(pose_xyz_yaw[0] - previous[0], pose_xyz_yaw[1] - previous[1])
        if distance >= self.args.keyframe_distance_m:
            return True
        if self._accumulated_yaw_deg >= self.args.keyframe_yaw_deg:
            return True
        return (
            time.monotonic() - self._last_keyframe_host_s
            >= self.args.keyframe_max_seconds
        )

    # ---------------------------------------------------------- 采集线程
    def _capture_loop(self) -> None:
        poll = max(0.02, float(self.args.capture_poll_seconds))
        consecutive_rgbd_failures = 0
        while not self._stop.is_set():
            started = time.monotonic()
            # spin 里是 8 × 10ms 的有界批次，正常情况下远小于 poll 周期。
            with self._provider_lock:
                self._provider.spin_once()
            self._check_session_rotation()

            pose = self._provider.get_pose()
            quality = self._provider.quality()
            if pose is not None:
                pose_xyz_yaw = (float(pose.x), float(pose.y), float(pose.yaw))
                self._accumulated_yaw_deg += (
                    abs(math.degrees(_wrap_pi(pose_xyz_yaw[2] - self._last_yaw_sample)))
                    if self._last_yaw_sample is not None
                    else 0.0
                )
                self._last_yaw_sample = pose_xyz_yaw[2]
                linear, angular = self._current_speed(pose_xyz_yaw)
                if (
                    linear > self.args.max_linear_speed_mps
                    or angular > self.args.max_angular_speed_dps
                ):
                    self._bump("skipped_robot_moving_fast")
                    if not self._speed_warned:
                        self._speed_warned = True
                        self._log(
                            "WARNING: 运动速度超过识别门限 "
                            f"({linear:.2f} m/s, {angular:.1f} deg/s)，本帧跳过识别"
                            "（只跳过识别，不干涉运动）"
                        )
                    self._publish_status(**_status_counter_view(self))
                    self._sleep_remaining(started, poll)
                    continue
                self._publish_status(
                    quality=quality,
                    slam_ready=quality != "CAMERA_LOCAL",
                )
                if not self._should_capture(pose_xyz_yaw):
                    self._sleep_remaining(started, poll)
                    continue
            else:
                self._publish_status(quality=quality, slam_ready=False)
                self._sleep_remaining(started, poll)
                continue

            try:
                keyframe = self._capture_keyframe(pose_xyz_yaw)
                consecutive_rgbd_failures = 0
            except Exception as exc:  # noqa: BLE001 - 采帧失败不能拖垮侧车
                consecutive_rgbd_failures += 1
                if consecutive_rgbd_failures in (1, 10) or consecutive_rgbd_failures % 50 == 0:
                    self._log(f"WARNING: 采帧失败×{consecutive_rgbd_failures}: {exc}")
                self._fail(f"capture failed: {exc}")
                self._sleep_remaining(started, poll)
                continue
            if keyframe is not None:
                self._enqueue(keyframe)
            self._sleep_remaining(started, poll)

    def _sleep_remaining(self, started: float, poll: float) -> None:
        remaining = poll - (time.monotonic() - started)
        if remaining > 0:
            self._stop.wait(remaining)

    def _capture_keyframe(
        self, pose_xyz_yaw: tuple[float, float, float]
    ) -> Keyframe | None:
        """取一帧 RGB-D 并冻结成关键帧。失败抛异常，由调用方计数。"""
        frame = self._rgbd_source.get_latest(timeout_seconds=0.0)
        received_host_s = time.time()
        frame = self._rgbd_source.materialize(frame)

        # 时钟一致性门：拍摄时间戳只有在和宿主时钟对得上时才敢拿去查位姿。
        skew = float(frame.timestamp) - received_host_s
        if abs(skew) <= self.args.max_clock_skew_seconds:
            transform_timestamp = float(frame.timestamp)
            timestamp_source = "capture_timestamp"
        else:
            transform_timestamp = received_host_s
            timestamp_source = "receive_time_clock_skew"
            if self._status.get("clock_skew_seconds") is None:
                self._log(
                    "WARNING: D435 时间戳与宿主时钟相差 "
                    f"{skew:.1f}s（>{self.args.max_clock_skew_seconds}s），"
                    "改用接收时刻做位姿查找"
                )

        pose = self._provider.get_pose()
        robot_pose = {
            "x": round(pose_xyz_yaw[0], 4),
            "y": round(pose_xyz_yaw[1], 4),
            "yaw": round(pose_xyz_yaw[2], 4),
            "yaw_deg": round(math.degrees(pose_xyz_yaw[2]), 3),
            "source": getattr(pose, "source", None),
            "frame_id": getattr(pose, "frame_id", None),
        }
        keyframe = Keyframe(
            frame_id=frame.evidence_id,
            capture_timestamp=received_host_s,
            transform_timestamp=transform_timestamp,
            timestamp_source=timestamp_source,
            clock_skew_seconds=round(skew, 3),
            rgbd_frame=frame,
            session_id=self._session_id,
            robot_pose=robot_pose,
            source_health=dict(self._rgbd_source.health() or {}),
        )

        self._last_keyframe_pose = pose_xyz_yaw
        self._accumulated_yaw_deg = 0.0
        self._last_keyframe_host_s = time.monotonic()
        self._bump("captured_keyframes")
        self._publish_status(
            last_frame_id=keyframe.frame_id,
            clock_skew_seconds=keyframe.clock_skew_seconds,
        )
        return keyframe

    # ---------------------------------------------------------- 有界队列
    def _enqueue(self, keyframe: Keyframe) -> None:
        limit = max(1, int(self.args.queue_size))
        with self._queue_lock:
            while len(self._queue) >= limit:
                self._evict_one_locked()
            self._queue.append(keyframe)
            depth = len(self._queue)
        self._publish_status(queue_depth=depth, **_status_counter_view(self))

    def _evict_one_locked(self) -> None:
        """队列满时优先删冗余帧，而不是无条件"只留最新帧"。

        「冗余」= 与相邻关键帧的归一化位姿差 < 1，也就是**它俩之间本来就不该
        各留一帧**（静止不动时每秒一帧的堆积就是这种）。删掉这一对里较旧的那
        张：新的那张已经站在几乎同一个位置，代表的是更近的状态。若 20 帧真的
        代表 20 个不同位置，才丢最旧帧并计入 dropped_keyframes。
        """
        if not self._queue:
            return
        best_index = -1
        best_score = None
        for index in range(len(self._queue) - 1):
            score = self._redundancy(self._queue[index], self._queue[index + 1])
            if best_score is None or score < best_score:
                best_score = score
                best_index = index
        if best_index >= 0 and best_score is not None and best_score < 1.0:
            del self._queue[best_index]
            self._bump("dropped_redundant_keyframes")
            return
        self._queue.popleft()
        self._bump("dropped_keyframes")

    def _redundancy(self, older: Keyframe, newer: Keyframe) -> float:
        a = older.pose_xyz_yaw()
        b = newer.pose_xyz_yaw()
        if a is None or b is None:
            # 没有位姿就没有判据 —— 当作**不冗余**，让调用方走"丢最旧帧"那条
            # 确定性的路，而不是凭一个猜出来的分数去删中间帧。
            return float("inf")
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        yaw_deg = abs(math.degrees(_wrap_pi(b[2] - a[2])))
        return (
            distance / max(1e-6, self.args.keyframe_distance_m)
            + yaw_deg / max(1e-6, self.args.keyframe_yaw_deg)
        )

    def _dequeue(self, timeout: float) -> Keyframe | None:
        deadline = time.monotonic() + timeout
        while not self._stop.is_set():
            with self._queue_lock:
                if self._queue:
                    if self._dequeue_old_next and len(self._queue) > 1:
                        keyframe = self._queue.popleft()
                        self._dequeue_old_next = False
                    else:
                        keyframe = self._queue.pop()
                        self._dequeue_old_next = True
                    while self._queue and time.time() - self._queue[0].capture_timestamp > 15:
                        self._queue.popleft()
                        self._bump("dropped_keyframes")
                    depth = len(self._queue)
                    break
            if time.monotonic() >= deadline:
                return None
            self._stop.wait(0.05)
        else:
            return None
        self._publish_status(queue_depth=depth)
        return keyframe

    # ---------------------------------------------------------- 推理线程
    def _inference_loop(self) -> None:
        while not self._stop.is_set():
            self._apply_unknown_label_results()
            keyframe = self._dequeue(timeout=0.2)
            if keyframe is None:
                continue
            if keyframe.session_id != self._session_id:
                # 采帧之后 SLAM 换了会话：这张帧的位姿属于旧坐标系，不能落图。
                self._log(f"丢弃跨会话关键帧 {keyframe.frame_id}")
                continue
            try:
                self._process_keyframe(keyframe)
            except Exception as exc:  # noqa: BLE001 - 单帧失败不能停掉侧车
                # SIGTERM closes the persistent detector while a keyframe may
                # still be inside detect().  That expected shutdown race must
                # not overwrite an otherwise healthy final status with a fake
                # "worker exited unexpectedly" inference failure.
                if self._stop.is_set():
                    self._log(f"停机中止在途关键帧 {keyframe.frame_id}")
                    break
                self._bump("failed_keyframes")
                self._fail(f"inference failed on {keyframe.frame_id}: {exc}")
                self._publish_topology(self._current_detections, self._last_current_latency)
                self._publish_status(**_status_counter_view(self))
            finally:
                self._apply_unknown_label_results()

    def _process_keyframe(self, keyframe: Keyframe) -> None:
        from app.perception.depth_object_localizer import (
            enrich_scene_objects_with_spatial,
        )
        from app.spatial.object_display_snapshot import load_map_meta
        from app.spatial.semantic_object_gate import filter_persistent_observations

        started = time.monotonic()
        if keyframe.capture_timestamp <= self._last_current_capture:
            self._process_historical_keyframe(keyframe)
            return
        self._last_current_capture = keyframe.capture_timestamp
        self._publish_status(last_attempt_at=time.time(), state=STATE_RUNNING)

        frame = keyframe.rgbd_frame
        detection_batch = self._detect_batch(frame)
        detections = list(detection_batch.mapping_detections)

        scene_objects: list[dict[str, Any]] = []
        for index, obj in enumerate(detections, start=1):
            # detect_with_dynamic_terms 返回的是 DetectedObject **dataclass**，
            # `dict(obj)` 会抛 "'DetectedObject' object is not iterable" —— 必须是
            # asdict。参考实现 run_autonomous_loop._detect 之所以能写 dict(obj)，
            # 是因为它在上一层已经用 _gsam_to_dict 转过一次了。
            # 这里用 asdict 而不是照抄 _gsam_to_dict：后者只挑 9 个字段，会把
            # color / attributes / text_score 丢掉，而它们在 semantic_observer
            # 那条路上是真被读的；多带的键下游会忽略，少带的键查不出来。
            item = asdict(obj)
            # frame_object_id 与 id 同时写：逐帧事件是原始取证，而显示门的
            # raw_label 配对按 frame_object_id 查（见 semantic_object_gate）。
            item["id"] = f"gsam_obj_{index:03d}"
            item["frame_object_id"] = item["id"]
            if not item.get("category"):
                item["category"] = "object"
            scene_objects.append(item)
        for item in scene_objects:
            host_index = item.get("host_detection_index")
            if isinstance(host_index, int) and 0 <= host_index < len(scene_objects):
                host = scene_objects[host_index]
                item["host_source_object_id"] = host["id"]
                item["host_local_anchor"] = _host_local_anchor(
                    item.get("bbox_2d") or item.get("bbox"),
                    host.get("bbox_2d") or host.get("bbox"),
                )
        try:
            localized = self._localizer.localize(scene_objects, frame)
        except (ValueError, TypeError, RuntimeError) as exc:
            self._log(f"WARNING: 深度定位失败，仍保留当前检测和危险提示: {exc}")
            localized = []

        # §8.2：必须用**拍摄时刻**的位姿，绝不用"最新位姿"硬凑。变换失败
        # （map→odom 还没收到、位姿过期）就这一条不落图，而不是塞个近似值。
        with self._provider_lock:
            provenance = dict(self._provider.transform_provenance())
            map_projector = getattr(self._provider, "camera_point_to_map", None)
            localized_kept = []
            for obs_item in localized:
                if getattr(obs_item, "camera_xyz", None) is None:
                    continue
                if not callable(map_projector):
                    continue
                mapped = map_projector(obs_item.camera_xyz, keyframe.transform_timestamp)
                if mapped is None:
                    continue
                anchor = None
                anchor_projector = getattr(self._provider, "anchor_map_point", None)
                if callable(anchor_projector):
                    anchor = anchor_projector(mapped, keyframe.transform_timestamp)
                obs_item.map_xyz = mapped
                # ``mapped`` is already the transformed 3-D box centre.  The
                # previous assignment copied camera optical coordinates into
                # a field explicitly labelled as map coordinates, which the
                # live run exposed as a frame mismatch in event evidence.
                obs_item.box_center_map = mapped
                obs_item.box_size_map = obs_item.box_size_camera
                obs_item.spatial_quality = provenance.get("transform_quality")
                obs_item.provenance = {
                    **dict(obs_item.provenance or {}),
                    **provenance,
                    **dict(anchor or {}),
                    "map_frame": mapped_frame(provenance),
                    "timestamp": keyframe.transform_timestamp,
                    "capture_timestamp": keyframe.capture_timestamp,
                    "timestamp_source": keyframe.timestamp_source,
                    "clock_skew_seconds": keyframe.clock_skew_seconds,
                    "session_id": keyframe.session_id,
                }
                if _finite(mapped[0]) and _finite(mapped[1]) and _finite(mapped[2]):
                    localized_kept.append(obs_item)
            localized = localized_kept
            map_frame_value = mapped_frame(provenance)

        meta = load_map_meta(self.args.map_meta_path)
        epoch = str((meta or {}).get("map_epoch") or "")
        if self.args.map_mode == "2d" and not epoch:
            # The 2-D provider and display bridge consume the same latched
            # keyframe graph.  Do not discard early semantic frames merely
            # because the file bridge has not written its first heartbeat yet.
            epoch = str(provenance.get("map_epoch") or "")
        if self._map_epoch and epoch and epoch != self._map_epoch:
            self._rotate_session(reason="map_epoch_changed")
            return
        if epoch and not self._map_epoch:
            self._mobility.reset()
        if epoch:
            self._map_epoch = epoch
        if keyframe.session_id != self._session_id:
            return
        map_odom_pose = provenance.get("map_odom_pose")
        if map_odom_pose is not None:
            map_odom_pose = tuple(float(value) for value in map_odom_pose)
            prior = self._last_map_odom_pose
            if prior is not None and (
                math.hypot(map_odom_pose[0] - prior[0], map_odom_pose[1] - prior[1]) >
                    float(self.mapping_config.get("mobility", {}).get("map_jump_m", .15))
                or abs(_wrap_pi(map_odom_pose[2] - prior[2])) >
                    float(self.mapping_config.get("mobility", {}).get("map_jump_yaw_rad", .08))
            ):
                self._mobility.reset()
                reproject = getattr(self._provider, "reproject_camera_point_to_map", None)
                if callable(reproject) and self._semantic_map is not None:
                    with self._provider_lock:
                        self._semantic_map.reproject_observations(
                            reproject,
                            anchor_lookup=getattr(
                                self._provider, "reproject_anchor_point", None
                            ),
                        )
            self._last_map_odom_pose = map_odom_pose

        scene_objects = enrich_scene_objects_with_spatial(
            scene_objects, localized, image_timestamp=keyframe.capture_timestamp
        )

        by_source = {str(obs.object_id): obs for obs in localized}
        tracking_inputs = []
        for item in scene_objects:
            obs = by_source.get(str(item["id"]))
            tracking_inputs.append(obs if obs is not None else SimpleNamespace(
                label=item.get("label"), label_zh=item.get("label_zh"),
                bbox=item.get("bbox_2d") or item.get("bbox"), map_xyz=None))
        tracks = self._mobility.update(
            tracking_inputs, frame_id=keyframe.frame_id,
            timestamp=keyframe.capture_timestamp, map_epoch=self._map_epoch)
        self._hazards.begin_frame(
            frame_id=keyframe.frame_id,
            timestamp=keyframe.capture_timestamp,
            evaluated=True,
        )
        all_source_tracks = {
            str(item["id"]): track for item, track in zip(scene_objects, tracks)
        }
        current: list[dict[str, Any]] = []
        by_source_track = {}
        for item, track in zip(scene_objects, tracks):
            source_id = str(item["id"])
            obs = by_source.get(source_id)
            event, is_new = self._hazards.observe(
                item, track_id=track.track_id, frame_id=keyframe.frame_id,
                timestamp=keyframe.capture_timestamp,
                has_position=obs is not None and obs.map_xyz is not None,
                location_mode=(obs.provenance or {}).get("location_mode") if obs else None,
                host_object_id=item.get("host_source_object_id"),
                host_local_anchor=item.get("host_local_anchor"),
            )
            fields = track.fields()
            if event:
                host_track = all_source_tracks.get(str(item.get("host_source_object_id") or ""))
                if host_track is not None:
                    event["host_track_id"] = host_track.track_id
                fields.update(is_hazard=True, hazard_rule_id=event["hazard_rule_id"],
                              hazard_label=event["hazard_label"],
                              hazard_event_id=event["hazard_event_id"],
                              hazard_confirmation_state=event.get("status"),
                              hazard_location_mode=event.get("location_mode"),
                              hazard_position_status=event.get("position_status"))
                status = str(event.get("status") or "suspected")
                if status != event.get("last_announced_status") and status in {"suspected", "confirmed"}:
                    event["last_announced_status"] = status
                    if not event.get("evidence_path") and Path(frame.color_ref).is_file():
                        evidence = self.events_path.parent / f"{event['hazard_event_id']}.jpg"
                        try:
                            shutil.copyfile(frame.color_ref, evidence)
                            event["evidence_path"] = str(evidence)
                        except OSError as exc:
                            self._log(f"WARNING: 危险截图保存失败: {exc}")
                    self._write_event({
                        "event": "hazard_alert" if status == "confirmed" else "hazard_suspected",
                        **event,
                    })
            if obs is not None:
                for name, value in fields.items():
                    setattr(obs, name, value)
                by_source_track[source_id] = track
            current.append({"source_object_id": source_id,
                            "label": item.get("label"), "label_zh": item.get("label_zh"),
                            "bbox_2d": item.get("bbox_2d") or item.get("bbox"),
                            "capture_timestamp": keyframe.capture_timestamp,
                            "has_position": obs is not None and obs.map_xyz is not None,
                            "location_mode": ((obs.provenance or {}).get("location_mode")
                                              if obs is not None else "unavailable"),
                            **fields})
            if track.mobility_class == "dynamic" and track.persistent_id:
                old = self._semantic_map.objects.get(track.persistent_id)
                if old is not None:
                    old.topology_eligible = False
                    old.mobility_class = "dynamic"
                    old.mobility_state = track.mobility_state
                    old.mobility_reason = track.reason

        # 连续短期轨迹提供了比外观类别更强的身份依据：可移动物体完成一次
        # moving -> stationary 后，把原 ID 开启新位置段，再让常规一对一关联更新
        # 属性。这样旧位置不会与新位置无限平均，也不会同时留下两个当前节点。
        for obs in localized:
            track = by_source_track.get(str(obs.object_id))
            if (track is None or not track.persistent_id
                    or not track.position_change_pending
                    or track.mobility_class != "semi_static"
                    or track.mobility_state != "stationary" or obs.map_xyz is None):
                continue
            old = self._semantic_map.objects.get(track.persistent_id)
            if old is None or old.map_xyz is None:
                continue
            displacement = math.dist(old.map_xyz, obs.map_xyz)
            if displacement <= float(self.mapping_config.get("mobility", {}).get("moving_distance_m", .25)):
                track.position_change_pending = False
                continue
            old.mobility_class = "semi_static"
            old.mobility_state = "stationary"
            old.mobility_reason = track.reason
            changed = self._semantic_map.start_position_epoch(
                old.object_id, obs.map_xyz, timestamp=keyframe.capture_timestamp,
                reason="continuous_track_restable_at_new_position",
            )
            if changed:
                track.position_change_pending = False

        # 仅证据充分的第一、第二档进入现有持久地图。
        persistent = [
            obs for obs in localized
            if self._map_epoch
            and getattr(obs, "topology_eligible", False)
            and getattr(obs, "mobility_state", "") == "stationary"
            and getattr(obs, "category", "") != "review"
            and (
                not getattr(obs, "is_hazard", False)
                or getattr(obs, "hazard_confirmation_state", None) == "confirmed"
            )
        ]
        persistent, ingest_dropped = filter_persistent_observations(
            persistent,
            min_confidence=self._settings.semantic_map_min_object_confidence,
        )
        self._write_event({
            "event": "semantic_object_ingest_filter",
            "frame_id": keyframe.frame_id,
            "input": len(localized),
            "kept": len(persistent),
            "dropped": ingest_dropped,
            "min_confidence": self._settings.semantic_map_min_object_confidence,
            "host_s": self._host_s(),
        })

        if keyframe.session_id != self._session_id:
            return
        update_result = self._semantic_map.update_with_associations(
            persistent,
            place_id=None,
            now=keyframe.capture_timestamp,
            frame_id=keyframe.frame_id,
        )
        self._semantic_map._reconcile_statuses(now=keyframe.capture_timestamp)
        for assoc in update_result.associations:
            track = by_source_track.get(str(assoc.source_object_id))
            if track is None:
                continue
            if track.persistent_id and track.persistent_id != assoc.persistent_object_id:
                old = self._semantic_map.objects.get(track.persistent_id)
                if old is not None:
                    old.topology_eligible = False
            track.persistent_id = assoc.persistent_object_id
            for event in self._hazards.events.values():
                if event["track_id"] == track.track_id:
                    event["persistent_object_id"] = track.persistent_id

        persistent_by_track = {
            track.track_id: track.persistent_id
            for track in by_source_track.values()
            if track.persistent_id
        }
        for event in list(self._hazards.events.values()):
            host_persistent = persistent_by_track.get(str(event.get("host_track_id") or ""))
            if host_persistent:
                bound = self._hazards.bind_host(
                    track_id=str(event["track_id"]),
                    host_persistent_object_id=host_persistent,
                )
                host_entry = self._semantic_map.objects.get(host_persistent)
                if bound is not None and host_entry is not None:
                    host_entry.is_hazard = True
                    host_entry.hazard_rule_id = bound.get("hazard_rule_id")
                    host_entry.hazard_label = bound.get("hazard_label")
                    host_entry.hazard_event_id = bound.get("hazard_event_id")
                    host_entry.hazard_location_mode = bound.get("location_mode")
                    host_entry.hazard_position_status = bound.get("position_status")

        association_by_index = {
            assoc.observation_index: assoc for assoc in update_result.associations
        }
        persistent_by_source_id = {
            str(assoc.source_object_id): assoc
            for assoc in update_result.associations
            if assoc.source_object_id
        }

        current_by_source_id = {
            str(item.get("source_object_id") or ""): item for item in current
        }
        for assoc in update_result.associations:
            track = by_source_track.get(str(assoc.source_object_id))
            if track is None:
                continue
            event = self._hazards.bind_persistent(
                track_id=track.track_id,
                persistent_object_id=assoc.persistent_object_id,
            )
            if event is None:
                continue
            event_id = event["hazard_event_id"]
            item = current_by_source_id.get(str(assoc.source_object_id))
            if item is not None:
                item["hazard_event_id"] = event_id
            entry = self._semantic_map.objects.get(assoc.persistent_object_id)
            if entry is not None:
                entry.hazard_event_id = event_id

        # The association result is the only reliable bridge from a detector
        # track to a persistent map identity.  Attach it before publishing or
        # queueing VLM work, then prefer any name already reviewed on that node.
        self._bind_current_detection_labels(current, persistent_by_source_id)

        self._update_place_graph(keyframe, update_result)

        self._update_revisit_lifecycle(
            keyframe=keyframe,
            update_result=update_result,
        )

        frame_objects: list[dict[str, Any]] = []
        for scene_object in scene_objects:
            event_item = dict(scene_object)
            assoc = persistent_by_source_id.get(
                str(event_item.get("source_object_id") or "")
            )
            if assoc is not None:
                event_item.update({
                    "persistent_object_id": assoc.persistent_object_id,
                    "association_score": assoc.association_score,
                    "association_action": assoc.action,
                })
            frame_objects.append(event_item)

        self._write_event({
            "event": "semantic_frame_objects_localized",
            "frame_id": keyframe.frame_id,
            "image_timestamp": keyframe.capture_timestamp,
            "timestamp_quality": frame.timestamp_quality,
            "transform_timestamp": keyframe.transform_timestamp,
            "timestamp_source": keyframe.timestamp_source,
            "robot_pose": keyframe.robot_pose,
            "map_frame": map_frame_value,
            "objects": frame_objects,
            "host_s": self._host_s(),
        })

        self._queue_unknown_object_labels(
            frame_path=Path(frame.color_ref),
            frame_id=keyframe.frame_id,
            scene_objects=scene_objects,
            persistent_by_source_id=persistent_by_source_id,
        )

        written_entities = 0
        for index, obs_item in enumerate(persistent):
            if getattr(obs_item, "map_xyz", None) is None:
                continue
            assoc = association_by_index.get(index)
            if assoc is None:
                # 本帧没建立可靠身份就不写实体事件 —— 绝不按 label 猜（两个
                # 同标签的物体必须保持各自的持久 ID）。
                continue
            entity = self._semantic_map.objects.get(assoc.persistent_object_id)
            event_object = entity.to_dict() if entity is not None else {}
            event_object.update({
                "object_id": assoc.persistent_object_id,
                "label": obs_item.label,
                "camera_xyz": (
                    list(obs_item.camera_xyz) if obs_item.camera_xyz else None
                ),
                "map_xyz": list(obs_item.map_xyz),
                "spatial_quality": obs_item.spatial_quality,
                "source_object_id": assoc.source_object_id,
                "association_score": assoc.association_score,
                "association_action": assoc.action,
                "map_frame": map_frame_value,
                "session_id": keyframe.session_id,
            })
            self._write_event({
                "event": "semantic_object_localized",
                "object": event_object,
                "host_s": self._host_s(),
            })
            written_entities += 1

        # §2.3：实体图刚更新完，把新版本投影到 WebUI 的显示对象快照。
        self._object_revision += 1
        self._publish_display_objects(self._object_revision)

        latency = time.monotonic() - started
        self._last_current_latency = latency
        self._current_detections = current
        self._last_detection_capture = keyframe.capture_timestamp
        self._frame_available = False
        try:
            temporary_frame = self._frame_path.with_suffix(".jpg.tmp")
            shutil.copyfile(frame.color_ref, temporary_frame)
            os.replace(temporary_frame, self._frame_path)
            self._frame_available = True
        except OSError as exc:
            self._log(f"WARNING: 当前识别帧保存失败: {exc}")
        self._process_target_candidates(
            keyframe=keyframe,
            frame=frame,
            batch=detection_batch,
            map_frame=map_frame_value,
        )
        if self._relations is None:
            from app.spatial.object_relation_store import ObjectRelationStore
            self._relations = ObjectRelationStore()
        relation_config = self.mapping_config.get("topology", {})
        # 遥控建图以前只写 near/co_visible，虽然几何提取器仍存在，左右、前后、
        # 上下关系却没有进入持久关系库。此处恢复与自主建图相同的关系链：
        # frame_object_id -> association -> persistent obj_xxx。
        self._relations.sync(
            relations=_extract_teleop_geometric_relations(scene_objects, relation_config),
            update_result=update_result,
            observation_id=keyframe.frame_id,
            timestamp=keyframe.capture_timestamp,
        )
        self._relations.mark_stale(now=keyframe.capture_timestamp)
        self._relations.observe_teleop_pairs(
            self._semantic_map.objects,
            {assoc.persistent_object_id for assoc in update_result.associations},
            frame_id=keyframe.frame_id, timestamp=keyframe.capture_timestamp,
            near_distance_m=float(relation_config.get("near_distance_m", 3.0)),
            near_exit_distance_m=float(relation_config.get("near_exit_distance_m", 3.4)),
            spatial_neighbor_distance_m=float(
                relation_config.get("spatial_neighbor_distance_m", 4.5)
            ),
            max_near_edges=int(relation_config.get("max_near_edges", 4)),
            bridge_extra_edges_per_node=int(
                relation_config.get("bridge_extra_edges_per_node", 2)
            ),
            max_co_visible_edges=int(relation_config.get("max_co_visible_edges", 4)),
            pair_evidence=getattr(self._provider, "spatial_pair_evidence", None))
        with self._status_lock:
            self._status["last_error"] = None
        self._publish_topology(current, latency)
        self._bump("processed_keyframes")
        # 成功的这一轮把连续失败计数**在内存里**也清零，不只是写进状态文件：
        # _fail() 是拿 self._status 里的旧值 +1 的，只改落盘值的话，下一次失败
        # 会接着很久以前的计数继续涨，degraded 会比实际早到。
        with self._status_lock:
            self._status["consecutive_failures"] = 0
        self._publish_status(
            state=STATE_RUNNING,
            last_success_at=time.time(),
            last_frame_id=keyframe.frame_id,
            last_detection_count=len(detections),
            last_localized_count=len(localized),
            persistent_object_count=len(self._semantic_map.objects),
            inference_latency_seconds=round(latency, 3),
            map_frame=map_frame_value,
            last_error=None,
            **_status_counter_view(self, consecutive_failures=0),
        )
        self._log(
            f"{keyframe.frame_id}: 检测 {len(detections)} → 落图候选 {len(persistent)} "
            f"→ 实体事件 {written_entities}，实体总数 {len(self._semantic_map.objects)}，"
            f"耗时 {latency:.2f}s"
        )

    def _update_revisit_lifecycle(self, *, keyframe: Keyframe, update_result: Any) -> None:
        """Apply conservative negative evidence after positive associations."""
        if self.args.map_mode != "2d" or self._semantic_map is None:
            return
        projector = getattr(self._provider, "map_point_to_camera", None)
        if not callable(projector):
            return
        try:
            import cv2
            from app.detectors.vocabulary import resolve_label_zh
            from app.spatial.revisit_visibility import evaluate_revisit_visibility

            depth = cv2.imread(str(keyframe.rgbd_frame.depth_ref), cv2.IMREAD_UNCHANGED)
        except (ImportError, OSError, ValueError):
            return
        if depth is None:
            return
        matched = {assoc.persistent_object_id for assoc in update_result.associations}
        scheduled = {
            resolve_label_zh(term)
            for term in self._term_scheduler.last_terms
        }
        revisit = self.mapping_config.get("revisit", {})
        min_interval = float(revisit.get("negative_min_interval_s", 1.0))
        frame = keyframe.rgbd_frame
        observer = keyframe.robot_pose or {}
        observer_xy = (
            (float(observer["x"]), float(observer["y"]))
            if _finite(observer.get("x")) and _finite(observer.get("y")) else None
        )
        for object_id, entry in list(self._semantic_map.objects.items()):
            if object_id in matched or getattr(entry, "map_xyz", None) is None:
                continue
            if getattr(entry, "lifecycle_state", "active") == "removed":
                continue
            if getattr(entry, "mobility_class", None) not in {"fixed", "semi_static"}:
                continue
            if keyframe.capture_timestamp - float(getattr(entry, "last_checked_at", 0.0) or 0.0) < min_interval:
                continue
            with self._provider_lock:
                camera_xyz = projector(entry.map_xyz, keyframe.transform_timestamp)
            decision = evaluate_revisit_visibility(
                camera_xyz=camera_xyz,
                depth_image=depth,
                width=int(frame.width), height=int(frame.height),
                fx=float(frame.fx), fy=float(frame.fy),
                cx=float(frame.cx), cy=float(frame.cy),
                depth_unit_m=float(frame.depth_unit_m),
                localization_valid=bool(self._map_epoch),
                detector_covered=(entry.label_zh in scheduled),
                frame_reliable=keyframe.timestamp_source == "capture_timestamp",
                background_clearance_m=float(revisit.get("background_clearance_m", 0.25)),
                max_detection_distance_m=float(revisit.get("max_detection_distance_m", 8.0)),
            )
            entry.last_checked_at = keyframe.capture_timestamp
            if not decision.counts_as_missing:
                continue
            state = self._semantic_map.record_missing(
                object_id,
                observation_id=keyframe.frame_id,
                timestamp=keyframe.capture_timestamp,
                observer_xy=observer_xy,
                semi_static_required=int(revisit.get("semi_static_missing_observations", 3)),
                semi_static_span_s=float(revisit.get("semi_static_missing_span_s", 2.0)),
                fixed_required=int(revisit.get("fixed_missing_observations", 5)),
                fixed_min_viewpoints=int(revisit.get("fixed_missing_min_viewpoints", 2)),
            )
            self._write_event({
                "event": "semantic_object_absence_evidence",
                "object_id": object_id,
                "frame_id": keyframe.frame_id,
                "reason": decision.reason,
                "lifecycle_state": state,
                "expected_depth_m": decision.expected_depth_m,
                "observed_depth_m": decision.observed_depth_m,
                "host_s": self._host_s(),
            })
            if state == "removed":
                for event in self._hazards.events.values():
                    if event.get("persistent_object_id") == object_id:
                        event["location_state"] = "原位置已失效/现场需复核"

    def _process_historical_keyframe(self, keyframe: Keyframe) -> None:
        """旧帧只补已有稳定节点的证据，不推动实时运动或重复危险告警。"""
        from app.spatial.object_display_snapshot import load_map_meta
        from app.spatial.semantic_object_gate import filter_persistent_observations
        if keyframe.session_id != self._session_id:
            return
        meta = load_map_meta(self.args.map_meta_path)
        if not meta or meta["map_epoch"] != self._map_epoch:
            return
        scene_objects = []
        for index, obj in enumerate(self._detect(keyframe.rgbd_frame), start=1):
            item = asdict(obj)
            item["id"] = f"historical_obj_{index:03d}"
            scene_objects.append(item)
        localized = self._localizer.localize(scene_objects, keyframe.rgbd_frame)
        kept = []
        matched_ids: set[str] = set()
        with self._provider_lock:
            projector = getattr(self._provider, "camera_point_to_map", None)
            provenance = dict(self._provider.transform_provenance())
            for obs in localized:
                if obs.camera_xyz is None or not callable(projector):
                    continue
                mapped = projector(obs.camera_xyz, keyframe.transform_timestamp)
                if mapped is None:
                    continue
                anchor = None
                anchor_projector = getattr(self._provider, "anchor_map_point", None)
                if callable(anchor_projector):
                    anchor = anchor_projector(mapped, keyframe.transform_timestamp)
                obs.map_xyz = mapped
                obs.provenance = {**dict(obs.provenance or {}), **provenance,
                                  **dict(anchor or {}),
                                  "map_frame": mapped_frame(provenance),
                                  "timestamp": keyframe.transform_timestamp,
                                  "capture_timestamp": keyframe.capture_timestamp}
                candidate = self._semantic_map._best_candidate(
                    obs, now=keyframe.capture_timestamp, observation_index=len(kept))
                if (candidate is None or candidate[0].topology_eligible is not True
                        or candidate[0].object_id in matched_ids):
                    continue
                entry = candidate[0]
                matched_ids.add(entry.object_id)
                for name in ("mobility_class", "mobility_state", "mobility_reason",
                             "mobility_confidence", "topology_eligible", "is_hazard",
                             "hazard_rule_id", "hazard_label", "hazard_event_id",
                             "hazard_location_mode", "hazard_position_status"):
                    setattr(obs, name, getattr(entry, name))
                kept.append(obs)
        kept, _ = filter_persistent_observations(
            kept, min_confidence=self._settings.semantic_map_min_object_confidence)
        if keyframe.session_id != self._session_id:
            return
        # A historical frame may enrich the current position epoch with an
        # observation anchor.  It must not roll back a live position, revive a
        # removed object, clear accumulated absence evidence, or mutate hazard
        # state.  Keep those fields byte-for-byte stable around association.
        protected_fields = (
            "map_xyz", "position_variance_xyz", "observation_count", "last_seen",
            "status", "topology_eligible", "lifecycle_state", "last_checked_at",
            "missing_observation_count", "missing_first_at",
            "missing_last_observation_id", "missing_viewpoints", "position_epoch",
            "position_changed_at", "is_hazard", "hazard_rule_id", "hazard_label",
            "hazard_event_id", "hazard_location_mode", "hazard_position_status",
        )
        protected = {
            key: {name: copy.deepcopy(getattr(entry, name)) for name in protected_fields}
            for key, entry in self._semantic_map.objects.items()
        }
        update = self._semantic_map.update_with_associations(
            kept, now=keyframe.capture_timestamp, frame_id=keyframe.frame_id)
        for key, values in protected.items():
            entry = self._semantic_map.objects.get(key)
            if entry is None:
                continue
            for name, value in values.items():
                setattr(entry, name, value)
        self._semantic_map._reconcile_statuses(now=self._last_current_capture)
        if update.created_ids:
            # 地图状态变化可令候选失效，仍不允许历史帧产生有效新节点。
            for key in update.created_ids:
                self._semantic_map.objects.pop(key, None)
        if update.updated_ids:
            self._object_revision += 1
            self._publish_display_objects(self._object_revision)
            self._publish_topology(self._current_detections, self._last_current_latency)
        self._write_event({"event": "historical_frame_processed",
                           "frame_id": keyframe.frame_id,
                           "image_timestamp": keyframe.capture_timestamp,
                           "updated_ids": update.updated_ids})

    # ---------------------------------------------------------------- 检测
    def _ram_scene_terms(self, image_path: str) -> list[str]:
        from app.detectors.backend import detector_capabilities

        if not detector_capabilities(self.detector_backend).ram_scene_terms:
            return []
        settings = self._settings
        perception = self.mapping_config.get("perception", {})
        if (not settings.enable_ram_plus
                or not bool(perception.get("use_ram_scene_terms", True))):
            return []
        now = time.monotonic()
        interval = max(
            DEFAULT_RAM_SCENE_TAG_INTERVAL_S,
            float(getattr(settings, "ram_scene_tag_interval_seconds", 0.0) or 0.0),
        )
        if now - self._last_ram_tag_monotonic < interval:
            return self._cached_ram_terms
        try:
            self._cached_ram_terms = list(
                self._detector.discover_scene_terms(image_path)
            )
        except Exception as exc:  # noqa: BLE001 - RAM++ 失败不该终止识别
            self._log(f"WARNING: RAM++ 场景词获取失败，退到基础词表: {exc}")
            self._cached_ram_terms = []
        self._last_ram_tag_monotonic = now
        return self._cached_ram_terms

    def _detect(self, frame: Any) -> list[Any]:
        """Compatibility view used by historical frames: mapping gate only."""
        return list(self._detect_batch(frame).mapping_detections)

    def _detect_batch(self, frame: Any) -> Any:
        """Run one detector request and split mapping evidence from watch candidates."""
        from app.perception.passive_target_watch import (
            DetectionBatch,
            active_verification_specs,
            matches_terms,
            schedule_terms,
        )

        self._sync_target_watch_request()
        profile = str(self.mapping_config.get("scene_profile") or "indoor")
        scene_round = self._term_scheduler.next_terms(
            profile, extra_priority=self._hazards.prompt_terms()
        )
        resident_scene_terms = [
            *self._hazards.prompt_terms(), *self._term_scheduler.priority
        ]
        scene_round = _merge_discovered_scene_terms(
            scene_round,
            self._ram_scene_terms(frame.color_ref),
            resident_scene_terms,
            self._term_scheduler.max_terms,
        )
        with self._target_watch_lock:
            watch_spec = self._target_watch_spec
            generation = self._target_watch_generation
        verification_specs = active_verification_specs(
            watch_spec,
            auto_hazard_enabled=bool(
                self._target_watch_config.enabled
                and self._target_watch_config.auto_hazard_enabled
            ),
        )
        # With no operator-entered target, promote the built-in leak profile to
        # the detector schedule.  When an operator target exists, global hazard
        # terms remain resident and both profiles are still candidate-gated
        # below, so an unrelated target never disables automatic leak review.
        scheduled_spec = watch_spec or (
            verification_specs[0][0] if verification_specs else None
        )
        if scheduled_spec is not None:
            terms, slot_terms, truncated = schedule_terms(
                scheduled_spec,
                scene_round,
                [*self._hazards.prompt_terms(), *self._term_scheduler.priority],
                max_terms=self._term_scheduler.max_terms,
                max_target_terms=self._target_watch_config.max_target_terms,
                max_anchor_terms=self._target_watch_config.max_anchor_terms,
                reserve_scene_terms=self._target_watch_config.reserve_scene_terms,
            )
        else:
            terms = list(scene_round)
            slot_terms = {"target": (), "anchor": (), "scene": tuple(terms)}
            truncated = []
        scheduled_terms = list(terms)
        prompt_free = bool(getattr(self._detector, "prompt_free", False))
        if prompt_free:
            # The fused PF checkpoint owns its vocabulary.  Keep the scheduler
            # out of both the request and the public status so an ignored
            # rotating list cannot be mistaken for an active model prompt.
            terms = []
            slot_terms = {}
            truncated = []
            enabled_slots = None
        else:
            enabled_slots = frozenset(
                slot for slot, values in slot_terms.items() if values
            )
        with self._detect_lock:
            detections = self._detector.detect_with_dynamic_terms(
                frame.color_ref,
                scheduled_spec.raw_query if scheduled_spec is not None else "",
                list(slot_terms.get("target") or ()),
                list(slot_terms.get("scene") or ()),
                frame_id=str(getattr(frame, "frame_id", "")),
                slots=enabled_slots,
            )
        diagnostics = dict(getattr(self._detector, "last_diagnostics", {}) or {})
        self._publish_status(
            last_detection_terms=terms,
            last_detection_group=self._term_scheduler.last_group_index,
            scene_profile=profile,
            detector_backend=self.detector_backend,
            detector_model_id=diagnostics.get("model_id"),
            detector_model_sha256=diagnostics.get("model_sha256"),
            detector_library_version=diagnostics.get("library_version"),
            detector_device=diagnostics.get("device"),
            detector_precision=diagnostics.get("precision"),
            detector_prompt_mode=diagnostics.get("prompt_mode"),
            detector_class_count=diagnostics.get("class_count"),
            detector_confidence=diagnostics.get("confidence"),
            detector_iou=diagnostics.get("iou_threshold"),
            detector_image_size=diagnostics.get("image_size"),
            prompt_hash=diagnostics.get("prompt_hash"),
            prompt_term_count=diagnostics.get("prompt_term_count"),
            inference_latency_ms=diagnostics.get("inference_latency_ms"),
            masks_valid=diagnostics.get("masks_valid"),
            masks_invalid=diagnostics.get("masks_invalid"),
            fallback_used=False,
        )
        self._write_event({
            "event": "semantic_detection_terms",
            "frame_id": getattr(frame, "frame_id", None),
            "scene_profile": profile,
            "group_index": self._term_scheduler.last_group_index,
            "terms": terms,
            "slot_terms": {key: list(value) for key, value in slot_terms.items()},
            "scheduled_terms_ignored": scheduled_terms if prompt_free else [],
            "truncated_terms": truncated,
            "target_generation": generation if verification_specs else None,
            "automatic_hazard_enabled": bool(
                self._target_watch_config.auto_hazard_enabled
            ),
            "detector_backend": self.detector_backend,
            "detector_diagnostics": diagnostics or None,
            "host_s": self._host_s(),
        })
        threshold = float(
            self.mapping_config.get("perception", {}).get(
                "min_detection_confidence", 0.0
            )
        )
        max_bbox_area = float(
            self.mapping_config.get("perception", {}).get(
                "max_bbox_area_ratio", 1.0
            )
        )
        filtered = [
            item for item in detections
            if float(getattr(item, "score", 0.0) or 0.0) >= threshold
            and _bbox_area_ratio(
                getattr(item, "bbox_2d", getattr(item, "bbox", None))
            ) <= max_bbox_area
        ]
        from app.perception.slime_leak_enhancer import (
            enhance_slime_leak_detections,
            select_low_confidence_pipe_contexts,
        )

        visual = self.mapping_config.get("hazard", {}).get("visual_enhancement", {})
        pipe_terms = {
            str(value).strip().casefold()
            for value in visual.get(
                "pipe_labels", ["pipe", "pipeline", "industrial pipe", "管道"]
            )
        }
        context_only_pipes = select_low_confidence_pipe_contexts(
            detections,
            visual,
            persistent_min_confidence=threshold,
        )
        tracked_contexts = self._pipe_tracker.contexts(
            frame.color_ref,
            [*filtered, *context_only_pipes],
            timestamp=float(getattr(frame, "timestamp", time.time())),
            pipe_terms=pipe_terms,
        )
        mapping_detections = enhance_slime_leak_detections(
            frame.color_ref,
            filtered,
            visual,
            context_detections=context_only_pipes,
            tracked_pipe_contexts=tracked_contexts,
            display_language="zh",
        )

        target_candidates: list[dict[str, Any]] = []
        anchors: list[tuple[int, Any]] = []
        seen_anchor_objects: set[int] = set()
        seen_candidate_objects: set[int] = set()
        if verification_specs:
            pool = list(detections)
            pool.extend(item for item in mapping_detections if item not in pool)
            minimum = self._target_watch_config.candidate_score(self.detector_backend)
            for candidate_spec, automatic_hazard in verification_specs:
                spec_anchors: list[tuple[int, Any]] = []
                for anchor_index, anchor in enumerate(detections):
                    anchor_label = (
                        getattr(anchor, "source_prompt_term", None)
                        or getattr(anchor, "label", "")
                    )
                    anchor_slots = set(
                        getattr(anchor, "detection_slots", ()) or ()
                    )
                    if (
                        (not automatic_hazard and "anchor" in anchor_slots)
                        or matches_terms(anchor_label, candidate_spec.anchor_terms_en)
                    ):
                        spec_anchors.append((anchor_index, anchor))
                        if id(anchor) not in seen_anchor_objects:
                            anchors.append((anchor_index, anchor))
                            seen_anchor_objects.add(id(anchor))
                for index, item in enumerate(pool):
                    label = (
                        getattr(item, "source_prompt_term", None)
                        or getattr(item, "label", "")
                    )
                    slots = set(getattr(item, "detection_slots", ()) or ())
                    bbox = getattr(item, "bbox_2d", getattr(item, "bbox", None))
                    if not (
                        matches_terms(label, candidate_spec.target_terms_en)
                        or (not automatic_hazard and "target" in slots)
                    ):
                        continue
                    if id(item) in seen_candidate_objects:
                        continue
                    if matches_terms(
                        label,
                        (
                            *candidate_spec.negative_terms,
                            *candidate_spec.possible_confusions,
                        ),
                    ):
                        continue
                    if float(getattr(item, "score", 0.0) or 0.0) < minimum:
                        continue
                    if (
                        _bbox_area_ratio(bbox)
                        > self._target_watch_config.max_bbox_area_ratio
                    ):
                        continue
                    nearby = sorted(
                        (
                            (
                                _bbox_center_distance_ratio(
                                    bbox, getattr(anchor, "bbox_2d", None)
                                ),
                                anchor_index,
                                anchor,
                            )
                            for anchor_index, anchor in spec_anchors
                            if anchor is not item
                        ),
                        key=lambda value: value[0],
                    )
                    nearby = [
                        value for value in nearby
                        if value[0]
                        <= self._target_watch_config.relation_max_center_distance_ratio
                    ]
                    if candidate_spec.verification_mode == "relation" and not nearby:
                        continue
                    verification_bbox = tuple(float(value) for value in bbox)
                    anchor_indexes: list[int] = []
                    if nearby:
                        _, anchor_index, anchor = nearby[0]
                        anchor_indexes.append(anchor_index)
                        verification_bbox = _bbox_union(
                            bbox,
                            getattr(anchor, "bbox_2d", None),
                            self._target_watch_config.relation_crop_margin_ratio,
                        )
                    target_candidates.append({
                        "detection": item,
                        "detection_index": index,
                        "target_generation": generation,
                        "matched_term": str(label),
                        "anchor_indexes": anchor_indexes,
                        "verification_bbox": verification_bbox,
                        "verification_spec": candidate_spec.to_dict(),
                        "automatic_hazard": bool(automatic_hazard),
                    })
                    seen_candidate_objects.add(id(item))
            target_candidates.sort(
                key=lambda value: (
                    bool(value.get("automatic_hazard")),
                    float(getattr(value["detection"], "score", 0.0) or 0.0),
                ),
                reverse=True,
            )
            target_candidates = target_candidates[
                : self._target_watch_config.max_per_frame
            ]
            self._target_stat("frames_evaluated")
            self._publish_target_watch_state()
            self._write_event({
                "event": "target_watch_terms_scheduled",
                "session_id": self._mapping_session_id(),
                "generation": generation,
                "frame_id": str(getattr(frame, "frame_id", "")),
                "backend": self.detector_backend,
                "slot_terms": {
                    key: list(value) for key, value in slot_terms.items()
                },
                "truncated_terms": truncated,
                "automatic_hazard_enabled": bool(
                    self._target_watch_config.auto_hazard_enabled
                ),
                "host_s": self._host_s(),
            })
        return DetectionBatch(
            raw_detections=tuple(detections),
            mapping_detections=tuple(mapping_detections),
            target_candidates=tuple(target_candidates),
            target_anchors=tuple(item for _, item in anchors),
            prompt_terms=tuple(terms),
            slot_terms={key: tuple(value) for key, value in slot_terms.items()},
            diagnostics=diagnostics,
        )

    def _process_target_candidates(
        self, *, keyframe: Keyframe, frame: Any, batch: Any, map_frame: str
    ) -> None:
        """Localize and asynchronously verify only live-frame gated candidates."""
        from app.perception.passive_target_watch import (
            TargetCandidate,
            TargetWatchSpec,
            candidate_id,
            candidate_key,
        )

        candidates = list(batch.target_candidates or ())
        if self.args.smoke_target:
            candidates = candidates[:1]
        if not candidates:
            return
        with self._target_watch_lock:
            generation = self._target_watch_generation
            operator_object_confirmed = any(
                not bool(item.get("automatic_hazard"))
                for item in self._target_watch_state.get("confirmed_matches", [])
            )

        scene: list[dict[str, Any]] = []
        for offset, value in enumerate(candidates):
            if int(value.get("target_generation", -1)) != generation:
                continue
            item = asdict(value["detection"])
            item["id"] = f"target_watch_obj_{offset:03d}"
            item["frame_object_id"] = item["id"]
            scene.append(item)
        localized_by_id: dict[str, Any] = {}
        if scene:
            try:
                localized = self._localizer.localize(scene, frame)
            except (ValueError, TypeError, RuntimeError) as exc:
                localized = []
                self._log(f"WARNING: 目标候选深度定位失败（仍可视觉复核）: {exc}")
            with self._provider_lock:
                provenance = dict(self._provider.transform_provenance())
                projector = getattr(self._provider, "camera_point_to_map", None)
                for observation in localized:
                    if observation.camera_xyz is not None and callable(projector):
                        mapped = projector(
                            observation.camera_xyz, keyframe.transform_timestamp
                        )
                        if (
                            self._map_epoch
                            and mapped is not None
                            and all(_finite(value) for value in mapped)
                        ):
                            observation.map_xyz = mapped
                            observation.spatial_quality = provenance.get(
                                "transform_quality"
                            )
                            observation.provenance = {
                                **dict(observation.provenance or {}),
                                **provenance,
                                "map_frame": map_frame,
                                "map_epoch": self._map_epoch or None,
                                "timestamp": keyframe.transform_timestamp,
                                "capture_timestamp": keyframe.capture_timestamp,
                                "session_id": keyframe.session_id,
                            }
                    localized_by_id[str(observation.object_id)] = observation

        for offset, value in enumerate(candidates):
            if int(value.get("target_generation", -1)) != generation:
                self._target_stat("stale_results_discarded")
                continue
            spec_value = value.get("verification_spec")
            if not isinstance(spec_value, dict):
                self._write_event({
                    "event": "target_watch_candidate_suppressed",
                    "reason": "missing_verification_spec",
                    "session_id": self._mapping_session_id(),
                    "generation": generation,
                    "frame_id": keyframe.frame_id,
                    "backend": self.detector_backend,
                    "host_s": self._host_s(),
                })
                continue
            spec = TargetWatchSpec.from_dict(spec_value)
            automatic_hazard = bool(value.get("automatic_hazard"))
            if (
                not automatic_hazard
                and operator_object_confirmed
                and spec.verification_mode != "diffuse_hazard"
                and self._target_watch_config.stop_after_first_object_confirmation
            ):
                continue
            detection = value["detection"]
            source_id = f"target_watch_obj_{offset:03d}"
            observation = localized_by_id.get(source_id)
            prompt_term = str(value.get("matched_term") or getattr(detection, "label", ""))
            candidate_identifier = candidate_id(
                keyframe.frame_id, int(value.get("detection_index") or offset), prompt_term
            )
            bbox = tuple(float(item) for item in getattr(detection, "bbox_2d"))
            mask_available = bool(
                getattr(detection, "mask_rle", None)
                and getattr(detection, "mask_size", None)
            )
            camera_xyz = (
                tuple(float(item) for item in observation.camera_xyz)
                if observation is not None and observation.camera_xyz is not None
                else None
            )
            map_xyz = (
                tuple(float(item) for item in observation.map_xyz)
                if observation is not None and observation.map_xyz is not None
                else None
            )
            # Diffuse phenomena do not get a metric centre from a loose box.
            # A valid instance mask is the minimum requirement for publishing
            # their own location; otherwise the visual verdict remains useful
            # but map_xyz stays explicitly unknown.
            if spec.verification_mode == "diffuse_hazard" and not mask_available:
                map_xyz = None
            record = TargetCandidate(
                candidate_id=candidate_identifier,
                target_generation=generation,
                frame_id=keyframe.frame_id,
                source_object_id=source_id,
                matched_term=prompt_term,
                detection_slots=tuple(getattr(detection, "detection_slots", ()) or ()),
                detector_score=float(getattr(detection, "score", 0.0) or 0.0),
                bbox_2d=bbox,
                mask_available=mask_available,
                anchor_candidate_ids=tuple(
                    f"anchor_{index}" for index in value.get("anchor_indexes", ())
                ),
                verification_bbox=tuple(value.get("verification_bbox") or bbox),
                camera_xyz=camera_xyz,
                map_xyz=map_xyz,
                spatial_quality=(
                    str(getattr(observation, "spatial_quality", "") or "") or None
                ),
                provenance=(
                    dict(getattr(observation, "provenance", {}) or {})
                    if observation is not None else {
                        "map_frame": map_frame,
                        "map_epoch": self._map_epoch or None,
                        "capture_timestamp": keyframe.capture_timestamp,
                        "position_status": "unavailable",
                    }
                ),
                capture_timestamp=keyframe.capture_timestamp,
            ).to_dict()
            record.update({
                "automatic_hazard": automatic_hazard,
                "canonical_name_zh": spec.canonical_name_zh,
                "verification_profile": spec.to_dict(),
            })
            self._current_detections.append({
                "source_object_id": source_id,
                "candidate_id": candidate_identifier,
                "label": prompt_term,
                "label_zh": spec.canonical_name_zh,
                "score": record["detector_score"],
                "bbox_2d": list(bbox),
                "is_target_candidate": True,
                "target_watch_status": "CANDIDATE",
                "automatic_hazard": automatic_hazard,
                "capture_timestamp": keyframe.capture_timestamp,
                "camera_xyz": record.get("camera_xyz"),
                "map_xyz": record.get("map_xyz"),
            })
            self._target_stat("candidates_generated")
            with self._target_watch_lock:
                self._target_watch_state["state"] = "CANDIDATE"
                self._target_watch_state["automatic_hazard_active"] = (
                    automatic_hazard
                )
                self._target_watch_state["candidate_count"] = int(
                    self._target_watch_state.get("candidate_count") or 0
                ) + 1
                self._target_watch_state["last_candidate"] = copy.deepcopy(record)
            self._publish_target_watch_state()
            self._write_event({
                "event": "target_watch_candidate",
                "session_id": self._mapping_session_id(), "generation": generation,
                "frame_id": keyframe.frame_id, "candidate_id": candidate_identifier,
                "candidate": record,
                "automatic_hazard": automatic_hazard,
                "backend": self.detector_backend,
                "host_s": self._host_s(),
            })

            socket_path = Path(self._target_watch_config.socket_path)
            if not socket_path.is_absolute():
                socket_path = PROJECT_ROOT / socket_path
            if not self._target_watch_config.verify_enabled or not socket_path.exists():
                with self._target_watch_lock:
                    self._target_watch_state["state"] = "DEGRADED"
                    self._target_watch_state["last_error"] = (
                        f"VLM daemon unavailable: {socket_path}"
                    )
                self._publish_target_watch_state()
                self._write_event({
                    "event": "target_watch_degraded", "reason": "vlm_unavailable",
                    "session_id": self._mapping_session_id(), "generation": generation,
                    "frame_id": keyframe.frame_id, "candidate_id": candidate_identifier,
                    "backend": self.detector_backend, "host_s": self._host_s(),
                })
                continue
            if (
                spec.verification_mode != "diffuse_hazard"
                and self._target_watch_config.stop_after_first_object_confirmation
            ):
                with self._target_watch_lock:
                    object_verify_open = (
                        self._target_watch_object_verify_open_generation == generation
                    )
                if object_verify_open:
                    self._write_target_candidate_suppressed(
                        record, reason="object_verification_inflight"
                    )
                    continue
            # The daemon-health gate precedes cooldown and rate accounting:
            # a degraded candidate is still shown to the operator, but a
            # missing verifier must not consume the retry window or call
            # budget.  Once the daemon recovers the next observation can be
            # verified immediately.
            if not self._target_watch_cooldown.permits(
                candidate_key(record, generation)
            ):
                self._write_target_candidate_suppressed(
                    record, reason="candidate_cooldown"
                )
                continue
            if not self._target_watch_budget.permits():
                self._target_stat("verify_rate_limited")
                self._publish_target_watch_state()
                self._write_event({
                    "event": "target_watch_rate_limited",
                    "session_id": self._mapping_session_id(),
                    "generation": generation, "frame_id": keyframe.frame_id,
                    "candidate_id": candidate_identifier,
                    "backend": self.detector_backend, "host_s": self._host_s(),
                })
                continue
            evidence = self._save_target_evidence(
                Path(frame.color_ref), generation=generation,
                frame_id=keyframe.frame_id,
            )
            if evidence is None:
                self._write_target_candidate_suppressed(
                    record, reason="evidence_copy_failed"
                )
                continue
            verification_id, evidence_path = evidence
            record.update({
                "verification_id": verification_id,
                "evidence_path": str(evidence_path),
                "evidence_url": (
                    "/api/mapping/target-watch/evidence/" + verification_id
                ),
            })
            with self._target_watch_lock:
                self._target_watch_candidates[(generation, candidate_identifier)] = copy.deepcopy(record)
                self._target_watch_state["last_candidate"] = copy.deepcopy(record)
            self._publish_target_watch_state()
            sequence = self._target_watch_verifier.submit(
                frame_id=keyframe.frame_id,
                candidate_id=candidate_identifier,
                image_path=str(evidence_path),
                bbox=record["verification_bbox"],
                env={
                    "raw_query": spec.raw_query,
                    "profile": spec.to_dict(),
                    "automatic_hazard": automatic_hazard,
                    "robot_pose": copy.deepcopy(keyframe.robot_pose),
                    "candidate": copy.deepcopy(record),
                    "verification_id": verification_id,
                },
                task_generation=generation,
            )
            if sequence > 0 and (
                spec.verification_mode != "diffuse_hazard"
                and self._target_watch_config.stop_after_first_object_confirmation
            ):
                with self._target_watch_lock:
                    self._target_watch_object_verify_open_generation = generation
            elif sequence <= 0:
                self._write_target_candidate_suppressed(
                    record, reason="queued_latest_wins"
                )

    def _write_target_candidate_suppressed(
        self, record: dict[str, Any], *, reason: str
    ) -> None:
        self._write_event({
            "event": "target_watch_candidate_suppressed",
            "reason": reason,
            "session_id": self._mapping_session_id(),
            "generation": self._target_watch_generation,
            "frame_id": record.get("frame_id"),
            "candidate_id": record.get("candidate_id"),
            "backend": self.detector_backend, "host_s": self._host_s(),
        })

    def _save_target_evidence(
        self, source: Path, *, generation: int, frame_id: str
    ) -> tuple[str, Path] | None:
        if not source.is_file():
            return None
        with self._target_watch_lock:
            self._target_watch_evidence_seq += 1
            sequence = self._target_watch_evidence_seq
        safe_frame = "".join(ch for ch in str(frame_id) if ch.isalnum())[:32] or "frame"
        verification_id = f"twv_{self._session_id}_{generation}_{sequence}"
        destination = self.target_watch_evidence_dir / (
            f"tw_{self._session_id}_{generation}_{safe_frame}_{sequence}.jpg"
        )
        temporary = destination.with_suffix(".jpg.tmp")
        try:
            shutil.copyfile(source, temporary)
            os.replace(temporary, destination)
        except OSError as exc:
            self._log(f"WARNING: 目标证据帧保存失败: {exc}")
            temporary.unlink(missing_ok=True)
            return None
        self._prune_target_evidence()
        return verification_id, destination

    def _prune_target_evidence(self) -> None:
        limit = self._target_watch_config.evidence_max_files
        files = sorted(
            self.target_watch_evidence_dir.glob("tw_*.jpg"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        with self._target_watch_lock:
            protected = {
                str(item.get("evidence_path") or "")
                for item in self._target_watch_candidates.values()
            }
            protected.update(
                str(item.get("evidence_path") or "")
                for item in self._target_watch_state.get("confirmed_matches", [])
            )
        retained = 0
        for path in files:
            if retained < limit or str(path) in protected:
                retained += 1
                continue
            try:
                path.unlink()
            except OSError:
                pass

    # ---------------------------------------------------------------- 生命周期
    def run(self) -> int:
        self._target_watch_thread = threading.Thread(
            target=self._target_watch_loop, name="target-watch", daemon=True
        )
        self._target_watch_thread.start()
        self._inference_thread = threading.Thread(
            target=self._inference_loop, name="semantic-inference", daemon=True
        )
        self._inference_thread.start()
        self._publish_status(state=STATE_WAITING)
        self._capture_loop()
        self._inference_thread.join(timeout=5.0)
        self._target_watch_thread.join(timeout=2.0)
        return 0

    def run_once(self, timeout_seconds: float) -> int:
        """无运动烟雾检查：等一张有效 RGB-D + SLAM 位姿，处理一轮就退出。"""
        deadline = time.monotonic() + timeout_seconds
        pose_deadline = (
            min(deadline, time.monotonic() + 3.0)
            if self.args.camera_local_smoke else deadline
        )
        pose_xyz_yaw = None
        while time.monotonic() < pose_deadline and not self._stop.is_set():
            with self._provider_lock:
                self._provider.spin_once()
            pose = self._provider.get_pose()
            if pose is not None:
                pose_xyz_yaw = (float(pose.x), float(pose.y), float(pose.yaw))
                break
            self._stop.wait(0.2)
        if pose_xyz_yaw is None:
            if self.args.camera_local_smoke and not self._stop.is_set():
                # `--smoke` intentionally does not start SLAM.  Validate the
                # detector -> mask -> RGB-D camera_xyz contract without
                # inventing a map pose or map_xyz.
                self._publish_status(
                    state=STATE_RUNNING, slam_ready=False, quality="CAMERA_LOCAL"
                )
                keyframe = self._capture_keyframe((0.0, 0.0, 0.0))
                if keyframe is None:
                    self._fail("--once 未能采到关键帧")
                    return 4
                return self._process_camera_local_smoke(keyframe)
            self._fail(f"--once 超时：{timeout_seconds}s 内没有拿到 plain_slam 位姿")
            self._publish_status(**_status_counter_view(self))
            return 3

        # 位姿到手还不够：按 §8.2，落图要求 map→odom（只从 /tf 来）也已经在
        # bridge 里，否则 camera_point_to_map 一律返回 None，这一轮会"检测到了
        # 但一个都落不下来"。
        #
        # 正常模式下 TF 不是问题：采帧很快，但处理要等模型冷启动十几秒，这期间
        # 采集线程一直在 spin，TF 早到了。run_once 是同步的，所以这里显式等一次。
        # tf_ready 必须先在循环外初始化：等位姿那一步可能已经耗光 deadline，
        # 这个循环一次都不执行，直接落到下面的 if 上会 NameError。
        tf_ready = False
        tf_deadline = deadline
        while time.monotonic() < tf_deadline and not self._stop.is_set():
            with self._provider_lock:
                self._provider.spin_once()
                tf_ready = bool(
                    self._provider.transform_provenance().get("tf_available")
                )
            if tf_ready:
                break
            self._stop.wait(0.2)
        if not tf_ready:
            self._fail(
                f"--once 超时：{timeout_seconds}s 内没有收到 map→odom（/tf）。"
                "落图需要用拍摄时刻的 map 系位姿，拿不到就不落图 —— "
                "确认 plain_slam 的 slam_3d_node 在广播 pslam_map→pslam_odom。"
            )
            self._publish_status(**_status_counter_view(self))
            return 6

        self._publish_status(state=STATE_RUNNING, slam_ready=True)
        self._launch_log_size = self._read_launch_log_size()
        keyframe = self._capture_keyframe(pose_xyz_yaw)
        if keyframe is None:
            self._fail("--once 未能采到关键帧")
            return 4

        # 处理这一帧期间必须**有人继续 spin**，否则位姿在 pose_stale_s(3s) 之后就
        # 过期，PlainSlamSpatialProvider.camera_point_to_map 开头的
        # `if self.get_pose() is None: return None` 会直接短路 —— 连 bridge 都不进，
        # 于是"检测到了但一个都落不下来"，冒烟永远测不到 §10.2 的第三项判据。
        # 正常模式天然满足这个不变量（采集线程一直在 spin），run_once 是单线程，
        # 得自己把它补上。冷启动一轮推理十几秒，远大于 3 秒。
        #
        # 辅助线程只做 spin；_process_keyframe 里取 _provider_lock 的那一小段
        # （位姿快照，毫秒级）会被它短暂挡住，而十几秒的推理发生在取锁之前，
        # 所以不冲突。
        spin_stop = threading.Event()

        def _keep_spinning() -> None:
            while not spin_stop.is_set():
                with self._provider_lock:
                    self._provider.spin_once()
                spin_stop.wait(0.05)

        spinner = threading.Thread(target=_keep_spinning, name="once-spin", daemon=True)
        spinner.start()
        try:
            self._process_keyframe(keyframe)
        finally:
            spin_stop.set()
            spinner.join(timeout=2.0)
        status = self._status
        if status.get("last_error"):
            return 5
        self._log(
            f"--once 完成：{keyframe.frame_id} 检测 {status['last_detection_count']}，"
            f"定位 {status['last_localized_count']}，"
            f"实体总数 {status['persistent_object_count']}"
        )
        return 0

    def _process_camera_local_smoke(self, keyframe: Keyframe) -> int:
        """One-frame, no-SLAM validation used by the launcher's safe smoke."""
        started = time.monotonic()
        detection_batch = self._detect_batch(keyframe.rgbd_frame)
        detections = list(detection_batch.mapping_detections)
        if not detections and self.detector_backend == "yoloe_seg":
            # The industrial schedule's first rotation deliberately contains
            # only landmarks/hazards.  An indoor stationary smoke frame can
            # legitimately contain none of them, which would leave mask/depth
            # untested.  Retry the same frame with common geometry-bearing
            # objects; this is smoke-only and does not alter production cadence.
            smoke_terms = [
                "person", "chair", "table", "door", "cabinet", "trash bin", "pipe"
            ]
            with self._detect_lock:
                detections = self._detector.detect_with_dynamic_terms(
                    keyframe.rgbd_frame.color_ref, "", [], smoke_terms,
                    frame_id=str(keyframe.frame_id), slots=frozenset({"scene"}),
                )
            threshold = float(
                self.mapping_config.get("perception", {}).get(
                    "min_detection_confidence", 0.0
                )
            )
            max_bbox_area = float(
                self.mapping_config.get("perception", {}).get(
                    "max_bbox_area_ratio", 1.0
                )
            )
            detections = [
                item for item in detections
                if float(getattr(item, "score", 0.0) or 0.0) >= threshold
                and _bbox_area_ratio(getattr(item, "bbox_2d", None)) <= max_bbox_area
            ]
            self._write_event({
                "event": "camera_local_smoke_retry_terms",
                "frame_id": keyframe.frame_id,
                "terms": smoke_terms,
                "detection_count": len(detections),
                "detector_diagnostics": dict(
                    getattr(self._detector, "last_diagnostics", {}) or {}
                ),
            })
            # The retry validates detector geometry only. It intentionally
            # does not fabricate target candidates from generic smoke terms.
        scene_objects: list[dict[str, Any]] = []
        for index, obj in enumerate(detections, start=1):
            item = asdict(obj)
            item["id"] = f"smoke_obj_{index:03d}"
            item["frame_object_id"] = item["id"]
            scene_objects.append(item)
        localized = self._localizer.localize(scene_objects, keyframe.rgbd_frame)
        valid = [item for item in localized if getattr(item, "camera_xyz", None) is not None]
        if self._target_watch_spec is not None:
            self._process_target_candidates(
                keyframe=keyframe,
                frame=keyframe.rgbd_frame,
                batch=detection_batch,
                map_frame="camera",
            )
            deadline = time.monotonic() + self._target_watch_config.client_timeout_s + 0.5
            while time.monotonic() < deadline and self._target_watch_verifier.has_inflight():
                for result in self._target_watch_verifier.poll_completed():
                    self._apply_target_watch_result(result)
                self._stop.wait(0.05)
            for result in self._target_watch_verifier.poll_completed():
                self._apply_target_watch_result(result)
            watch = self._target_watch_snapshot()
            if int((watch.get("statistics") or {}).get("candidates_generated") or 0) == 0:
                self._log("--smoke-target: NO_CANDIDATE（VLM 调用数保持 0）")
        latency = time.monotonic() - started
        masks_valid = sum(1 for item in detections if getattr(item, "mask_rle", None))
        self._write_event({
            "event": "camera_local_perception_smoke",
            "frame_id": keyframe.frame_id,
            "detector_backend": self.detector_backend,
            "detection_count": len(detections),
            "masks_valid": masks_valid,
            "camera_localized_count": len(valid),
            "fallback_used": False,
            "inference_latency_seconds": latency,
            "host_s": self._host_s(),
        })
        self._publish_status(
            state=STATE_RUNNING,
            slam_ready=False,
            quality="CAMERA_LOCAL",
            last_success_at=time.time(),
            last_frame_id=keyframe.frame_id,
            last_detection_count=len(detections),
            last_localized_count=len(valid),
            masks_valid=masks_valid,
            inference_latency_seconds=latency,
            **_status_counter_view(self, processed_keyframes=1, consecutive_failures=0),
        )
        self._log(
            f"--once CAMERA_LOCAL 完成：{keyframe.frame_id} 检测 {len(detections)}，"
            f"mask {masks_valid}，camera_xyz {len(valid)}"
        )
        return 0

    def close(self) -> None:
        self._stop.set()
        if self._target_watch_verifier is not None:
            self._target_watch_verifier.close()
        if self._unknown_labeler is not None:
            self._unknown_labeler.close()
        self._publish_status(state=STATE_STOPPED, stopped_at=time.time())
        for name, closer in (
            ("detector", lambda: self._detector and self._detector.close()),
            ("provider", lambda: self._provider and self._provider.close()),
        ):
            try:
                closer()
            except Exception as exc:  # noqa: BLE001
                self._log(f"WARNING: 关闭 {name} 失败: {exc}")
        if self._events_handle is not None:
            try:
                self._events_handle.close()
            except OSError:
                pass
            self._events_handle = None


def mapped_frame(provenance: dict[str, Any]) -> str:
    return str(provenance.get("target_frame") or "unknown")


def _status_counter_view(
    mapper: RemoteControlSemanticMapper, **overrides: Any
) -> dict[str, Any]:
    """把计数器一次性推给状态文件，省得每个调用点各写一遍。

    ``overrides`` 是给调用点显式覆盖用的。没有它的话，调用点写
    ``_publish_status(consecutive_failures=0, **_status_counter_view(self))``
    会直接 TypeError（同一个关键字传了两次）—— 而那是在推理成功路径上，
    异常被推理循环兜住，于是"识别成功"被计成"识别失败"，一个物体都落不下来。
    走 overrides 之后调用点怎么写都不会撞。
    """
    with mapper._status_lock:
        keys = (
            "captured_keyframes",
            "dropped_keyframes",
            "dropped_redundant_keyframes",
            "processed_keyframes",
            "failed_keyframes",
            "consecutive_failures",
            "skipped_robot_moving_fast",
        )
        view = {key: mapper._status.get(key, 0) for key in keys}
    view.update(overrides)
    return view


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实体遥控器建图的语义侧车（只读位姿，只写自己的事件与状态）"
    )
    parser.add_argument("--events", required=True, help="本次运行事件文件 (jsonl)")
    parser.add_argument("--status", required=True, help="当前状态 JSON")
    # 计划书 §2.3：本进程是遥控建图模式的三维对象生产者。它按地图桥写的
    # map_epoch 声明自己属于哪张地图，写到同一套显式路径上，Web 端才可能配对。
    parser.add_argument(
        "--map-meta-path",
        default="outputs/autonomous_search/runtime/slam_map_meta.json",
        help="point-cloud-free map metadata written by the map bridge",
    )
    parser.add_argument(
        "--object-snapshot-path",
        default="outputs/autonomous_search/runtime/semantic_objects.json",
        help="where this process publishes its display objects",
    )
    parser.add_argument("--topology-snapshot-path", default="outputs/autonomous_search/runtime/semantic_topology.json")
    parser.add_argument(
        "--target-watch-request",
        default=os.environ.get(
            "GO2W_TARGET_WATCH_REQUEST",
            "outputs/remote_control_mapping/runtime/target_watch_request.json",
        ),
    )
    parser.add_argument(
        "--target-watch-state",
        default=os.environ.get(
            "GO2W_TARGET_WATCH_STATE",
            "outputs/remote_control_mapping/runtime/target_watch_state.json",
        ),
    )
    parser.add_argument(
        "--target-watch-evidence-dir",
        default=os.environ.get(
            "GO2W_TARGET_WATCH_EVIDENCE_DIR",
            "outputs/remote_control_mapping/runtime/target_watch_evidence",
        ),
    )
    parser.add_argument("--mapping-config", default=str(PROJECT_ROOT / "configs/go2w/semantic_mapping.yaml"))
    parser.add_argument(
        "--detector", choices=("grounded_sam", "yoloe_seg"), default=None,
        help="perception backend; CLI > REMOTE_MAPPING_DETECTOR > YAML > grounded_sam",
    )
    parser.add_argument("--map-mode", choices=("2d", "3d"), default="2d")
    parser.add_argument("--capture-poll-seconds", type=float, default=0.2)
    parser.add_argument("--keyframe-max-seconds", type=float, default=1.0)
    parser.add_argument("--keyframe-distance-m", type=float, default=0.75)
    parser.add_argument("--keyframe-yaw-deg", type=float, default=15.0)
    parser.add_argument("--queue-size", type=int, default=20)
    parser.add_argument("--rgbd-base-url", default="http://192.168.123.18:8080")
    parser.add_argument("--max-clock-skew-seconds", type=float, default=DEFAULT_MAX_CLOCK_SKEW_S)
    parser.add_argument(
        "--max-linear-speed-mps", type=float, default=DEFAULT_MAX_LINEAR_SPEED_MPS
    )
    parser.add_argument(
        "--max-angular-speed-dps", type=float, default=DEFAULT_MAX_ANGULAR_SPEED_DPS
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="只跑一轮（供无运动烟雾检查）；配合 --once-timeout-seconds",
    )
    parser.add_argument("--once-timeout-seconds", type=float, default=30.0)
    parser.add_argument(
        "--camera-local-smoke", action="store_true",
        help="when --once has no SLAM, validate mask + camera_xyz without faking map_xyz",
    )
    parser.add_argument(
        "--smoke-target", default="",
        help="with --once, configure one passive target and run at most one VLM verify",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    mapper = RemoteControlSemanticMapper(args)

    def _handle_signal(signum: int, _frame: Any) -> None:
        mapper._log(f"收到信号 {signum}，准备退出")
        mapper._stop.set()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    exit_code = 0
    try:
        mapper.setup()
        if args.once:
            exit_code = mapper.run_once(args.once_timeout_seconds)
        else:
            exit_code = mapper.run()
    except Exception as exc:  # noqa: BLE001 - 把失败写进状态文件再退
        mapper._fail(f"fatal: {exc}")
        mapper._publish_status(state=STATE_DEGRADED)
        exit_code = 1
    finally:
        mapper.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
