#!/usr/bin/env python3
"""High-level autonomous semantic exploration entry point (plan section 23).

Runs the platform-independent AutonomousExplorer against a chosen backend:

* ``--backend nav2`` (default, 计划书 §6.6-1): real Go2-W through the Nav2
  action chain (``ComputePathToPose`` + ``NavigateToPose``); requires the
  perception / odometry / motion stacks and the Nav2 bringup running.  There
  is no fallback: if Nav2, the map, TF or localization is not ready the
  explorer waits or fails, it never drops back to relative motion.
* ``--backend go2w_experimental``: legacy relative-motion executor from
  ``run_autonomous_loop.py``.  Kept only for standalone diagnostics (§4.3);
  not a production choice for autonomous search.
* ``--backend mock`` / ``--backend mock_metric``: offline E2E with scripted
  vision and simulated motion (no robot needed).
* ``--replay <session.jsonl>``: deterministic replay of a previous session's
  observations with the mock backend.

Outputs: session JSONL events, ``outputs/live_runs/<session_id>/``
(exploration_graph.json + summary.json).
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from app.config import get_settings
from app.detectors.backend import is_local_structured_backend
from app.detectors.siliconflow_vision_worker import quick_target_present, quick_target_state
from app.live_robot.autonomous_explorer import (
    AutonomousExplorer,
    PerceptionFailure,
    SemanticMatch,
    VerificationOutcome,
)
from app.live_robot.experiment_readiness import compute_experiment_readiness
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    scenario_anchor_then_target,
    scenario_no_target,
    scenario_target_appears_after,
)
from app.live_robot.async_semantic_observer import AsyncSemanticObservationManager, compute_scene_signature
from app.live_robot.latency_profiler import LatencyProfiler
from app.live_robot.verify_cache import VerificationCache, VerificationCacheEntry
from app.perception.target_spatial import target_identity
from app.live_robot.semantic_observer import (
    SEMANTIC_STATUS_FRESH_FULL,
    SEMANTIC_STATUS_FRESH_QUICK,
    SEMANTIC_STATUS_PENDING,
    SEMANTIC_STATUS_UNAVAILABLE,
    LiveSemanticObserver,
    SemanticObservation,
    _from_payload,
    semantic_observation_to_live,
    semantic_payload_from_quick_target_absence,
    local_detection_scene_objects,
)
from app.memory.observation_memory_store import ObservationMemoryStore
from app.navigation.backend_factory import create_backend
from app.navigation.robot_backend import TERMINAL_NAVIGATION_STATUSES
from app.navigation.topology_route_executor import (
    TopologyRouteExecutor,
    TopologyRoutePlanner,
)
from app.reasoning.llm_prior_generator import LLMPriorGenerator, LLMPriorInput
from app.navigation.exploration_config import (
    load_exploration_policy,
    load_go2w_experiment_profile,
)
from app.navigation.search_policy_config import load_search_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.local_scan import LocalScanState, select_local_scan_goal
from app.navigation.nav2_candidate_geometry import scan_is_clear
from app.navigation.semantic_frontier_relevance import score_frontier_semantics
from app.navigation.models import (
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
    LiveObservation,
)
from app.navigation.safety_models import (
    evaluate_perception_quality,
    local_safety_from_clearance,
)
from app.reasoning.semantic_navigation.models import SearchReasoningContext
from app.reasoning.semantic_navigation.task_agent import (
    attach_trace,
    enrich_context,
    retrieve_knowledge,
    summarize_topology,
)
from app.reasoning.semantic_navigation.router import SemanticSearchController
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory
from app.reasoning.target_profile import TargetProfileResolver
from app.spatial.models import FrontierCandidate, SpatialFrameMismatch
from app.spatial.world_state_snapshot import (
    WorldRevisions,
    build_world_state_snapshot,
)
from app.spatial.world_bundle import write_world_manifest
from app.perception.detection_scheduler import (
    ALL_SLOTS,
    DetectionCadence,
    DetectionScheduler,
    SLOT_ANCHOR,
    SLOT_SCENE,
    SLOT_TARGET,
    SlotAssembly,
    SlotCacheEntry,
    assemble_slot_objects,
    decide_detection_refresh,
    put_slot_entry,
)
from app.perception.entity_roles import (
    classify_roles,
    derive_target_state,
    entity_id_for,
    select_candidate,
)
from app.perception.target_state import TARGET_ABSENT, TARGET_PRESENT
from app.live_robot.async_verifier import (
    VERIFY_STATUS_CONFIRMED,
    VERIFY_STATUS_UNKNOWN,
    AsyncVerifierManager,
    result_matches_submission,
)
from app.spatial.dynamic_labels import drop_dynamic_objects
from app.spatial.object_mobility import ObjectMobilityTracker
from app.spatial.semantic_object_gate import filter_persistent_observations
from app.spatial.geometric_relation_extractor import extract_geometric_relations
from app.spatial.spatial_pose_validator import MotionEvidence, SpatialPoseValidator


class _Nav2MotionEvidenceRecorder:
    """给 Nav2 后端补上「这条指令要求了多少位移」的运动证据。

    pose validator 用 ``MotionEvidence`` 判断位姿变化是否与指令相称。实验后端是在
    ``execute_step`` 里逐条记账的，Nav2 没有那个入口，所以在后端边界上补一次。

    它只记账：所有调用原样委派，不改任何一条指令的内容，也不改拒绝行为。这不是
    第二个后端，是包在唯一后端外面的一层观测。
    """

    def __init__(self, inner: Any, *, node: Any, motion_state: dict[str, Any]) -> None:
        self._inner = inner
        self._node = node
        self._motion_state = motion_state
        self._odom_before: tuple[float, float, float] | None = None

    # ---- RobotBackend 接口：逐条委派 ----
    def capabilities(self) -> Any:
        return self._inner.capabilities()

    @property
    def occupancy_grid(self) -> Any:
        # 计划书 §7.5：候选评分要用后端订阅到的占据栅格。这层只观测，原样透传。
        return getattr(self._inner, "occupancy_grid", None)

    def get_pose(self) -> Any:
        return self._inner.get_pose()

    def health(self) -> Any:
        return self._inner.health()

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        """把内层后端依赖的 ROS 订阅推进一轮。

        nav2 后端的 ``localization_valid`` 与 ``/map`` 都靠订阅回调写进缓存，初值
        是「未收到」。explorer 的 bootstrap 重试要在这个节点上转，否则重试期间
        回调不执行，三次都判 ``NO_LOCALIZATION``。这层只透传一次 spin，不改任何
        指令内容与拒绝行为。

        生产入口会把同一个节点加入专用 ``SingleThreadedExecutor`` 并在后台线程
        持续 spin。此时不能再把节点临时加入 ``rclpy`` 的全局 executor；双 executor
        抢同一节点会令订阅回调停止推进，现场表现正是启动检查通过、任务 bootstrap
        随后连续报 ``NO_LOCALIZATION``。已有 executor 时这里只让出时间给它处理，
        仅在没有专用 executor 的独立调用路径中才主动 ``spin_once``。
        """
        if self._node is None:
            if timeout_sec > 0:
                time.sleep(timeout_sec)
            return
        if getattr(self._node, "executor", None) is not None:
            if timeout_sec > 0:
                time.sleep(timeout_sec)
            return
        import rclpy  # noqa: PLC0415 - 与模块内其他 ROS 依赖一样按需导入

        rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def cancel(self, handle: Any = None) -> bool:
        return self._inner.cancel(handle)

    def stop(self) -> None:
        self._inner.stop()

    def execute_goal(self, goal: Any) -> Any:
        if str(getattr(goal, "goal_type", "")).upper() == "NAVIGATE_POSE":
            self._odom_before = self._odom()
            self._note_requested(goal)
        return self._inner.execute_goal(goal)

    def get_navigation_status(self, handle: Any) -> Any:
        result = self._inner.get_navigation_status(handle)
        if result.status in TERMINAL_NAVIGATION_STATUSES:
            self._note_completed()
        return result

    def navigation_timeout_sec(self) -> float:
        return self._inner.navigation_timeout_sec()

    # ---- 记账 ----
    def _odom(self) -> tuple[float, float, float] | None:
        try:
            snapshot = self._node._odom_snapshot()
        except Exception:  # noqa: BLE001 - 记不上账不能让搜索停下来
            return None
        return (float(snapshot[0]), float(snapshot[1]), float(snapshot[2]))

    def _note_requested(self, goal: Any) -> None:
        map_pose = self._inner.get_pose()
        if goal.position is None or map_pose is None or map_pose.frame_id != "map":
            return
        try:
            target_yaw = math.radians(float(goal.yaw or 0.0))
            turn_deg = math.degrees(_wrap_pi(target_yaw - map_pose.yaw))
            forward_m = math.hypot(
                float(goal.position[0]) - map_pose.x,
                float(goal.position[1]) - map_pose.y,
            )
        except Exception:  # noqa: BLE001
            return
        self._motion_state["pending_motion_evidence"] = MotionEvidence(
            command_type="ROTATE" if forward_m <= 0.03 else "TRANSLATE",
            requested_turn_deg=turn_deg,
            requested_forward_m=forward_m,
        )

    def _note_completed(self) -> None:
        before = self._odom_before
        self._odom_before = None
        evidence = self._motion_state.get("pending_motion_evidence")
        if before is None or not isinstance(evidence, MotionEvidence):
            return
        after = self._odom()
        if after is None:
            return
        # 重建而不是原地改字段：MotionEvidence 是不是 frozen 不该由这里决定。
        self._motion_state["pending_motion_evidence"] = MotionEvidence(
            command_type=evidence.command_type,
            requested_turn_deg=evidence.requested_turn_deg,
            requested_forward_m=evidence.requested_forward_m,
            wheel_delta_xy_m=math.hypot(after[0] - before[0], after[1] - before[1]),
            wheel_delta_yaw_deg=math.degrees(_wrap_pi(after[2] - before[2])),
            motion_completed_at=time.time(),
        )


def _navigation_sector(*, yaw_rad: float, sectors: int) -> int:
    """把当前真实 yaw 映射到 0..sectors-1 的导航 heading sector。

    计划书 §6.2 / 不变量 2：导航朝向来自运动位姿事实，与语义状态无关。
    """
    sectors = max(1, int(sectors))
    sector_deg = 360.0 / sectors
    return int(round(math.degrees(float(yaw_rad)) / sector_deg)) % sectors


def _requested_dyaw_deg(step: str) -> float:
    """l30 -> +30, r30 -> -30, f/b -> 0（运动请求的标称转角）。"""
    if not step:
        return 0.0
    if step.startswith("l"):
        try:
            return float(step[1:])
        except ValueError:
            return 0.0
    if step.startswith("r"):
        try:
            return -float(step[1:])
        except ValueError:
            return 0.0
    return 0.0


def _requested_forward_m(step: str) -> float:
    """Parse the local primitive's signed forward/backward request."""
    if not step or step[0] not in {"f", "b"}:
        return 0.0
    try:
        distance = float(step[1:])
    except (TypeError, ValueError):
        return 0.0
    return distance if step[0] == "f" else -distance


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _classify_perception_error(exc: Exception) -> str:
    """计划书 §8.2：把底层异常映射成结构化错误码（Quick/RGB-D 感知链）。

    **顺序即语义，先认"哪一段"再认"什么毛病"。** 旧写法把 ``timeout`` 判在最
    前面，于是 ``RGB-D capture failed: ... timed out`` 会被记成
    ``QUICK_VLM_TIMEOUT`` —— 审计报告点名的正是这类"现场定位走弯路"：拿着
    QUICK_VLM_TIMEOUT 去查 VLM，实际是相机那条链断的。

    ``RGB_IMAGE_ERROR`` 只对**非超时**的取帧失败（相机断流、解码失败）。注意
    取帧失败的文本一律以 ``RGB-D capture failed: `` 开头，所以"是图像还是深度"
    必须在 ``rgb-d`` 这个分支**内部**再判一次，放到外面判永远够不着。

    **诚实边界**：真机上到 2026-09-16 为止只出现过两类失败 ——
    ``SiliconFlow vision API timed out``（22 次，→ QUICK_VLM_TIMEOUT，分类正确）
    与 ``UnboundLocalError: image_stamp``（1 次，→ UNKNOWN，分类正确）。
    ``RGB_IMAGE_ERROR`` / ``DEPTH_ERROR`` **一次都没在真机上出现过**，下面的关键词
    没有被真机数据校准过 —— 这次补的是**可达性**（此前它们除了 docstring 之外
    根本产生不出来），不是精度。

    关键词刻意收得**窄**：只用 ``rgb-d``/``rgbd``/``camera``，不用裸的 ``image``。
    原因是真机数据当场否掉了宽版本 —— ``image_stamp`` 那个变量名会让
    ``UnboundLocalError: local variable 'image_stamp' ``（一个代码 bug，真机上出现过）
    被误判成 ``RGB_IMAGE_ERROR``，于是现场去查相机。窄版本下它正确落到 UNKNOWN。
    """
    code = str(getattr(exc, "code", "") or "").strip()
    if code:
        return code
    text = f"{type(exc).__name__}: {exc}".lower()
    timed_out = "timed out" in text or "timeout" in text
    if "rgb-d" in text or "rgbd" in text or "camera" in text:
        if timed_out or "stale" in text or "unavailable" in text:
            return "RGBD_TIMEOUT"
        if "depth" in text:
            return "DEPTH_ERROR"
        return "RGB_IMAGE_ERROR"
    if "depth" in text:
        return "DEPTH_ERROR"
    if timed_out:
        return "QUICK_VLM_TIMEOUT"
    if "stale" in text:
        return "FRAME_STALE"
    if "tf" in text or "transform" in text:
        return "TF_UNAVAILABLE"
    return "UNKNOWN_PERCEPTION_ERROR"


def _motion_action_server_count() -> tuple[int | None, str]:
    """Count /go2w/motion servers so duplicate action owners fail closed."""
    try:
        completed = subprocess.run(
            ["ros2", "action", "info", "/go2w/motion"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f"action graph probe failed: {type(exc).__name__}: {exc}"
    output = (completed.stdout or "") + (completed.stderr or "")
    match = re.search(r"Action servers:\s*(\d+)", output)
    if completed.returncode != 0 or match is None:
        return None, output.strip()[-500:] or "action graph count unavailable"
    return int(match.group(1)), output.strip()[-500:]


def _motion_action_server_process_count() -> tuple[int | None, list[int]]:
    """Count real server executables; duplicate ROS node names can collapse."""
    pids: list[int] = []
    try:
        proc_root = Path("/proc")
        for entry in proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv0 = (entry / "cmdline").read_bytes().split(b"\0", 1)[0]
            except (OSError, PermissionError):
                continue
            if Path(os.fsdecode(argv0)).name == "go2w_motion_action_server":
                pids.append(int(entry.name))
    except OSError:
        return None, []
    return len(pids), sorted(pids)


def _topic_publisher_count(topic: str) -> tuple[int | None, str]:
    """Return the ROS publisher count for a safety-critical topic."""

    try:
        completed = subprocess.run(
            ["ros2", "topic", "info", topic, "-v"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f"topic graph probe failed: {type(exc).__name__}: {exc}"
    output = (completed.stdout or "") + (completed.stderr or "")
    match = re.search(r"Publisher count:\s*(\d+)", output)
    if completed.returncode != 0 or match is None:
        return None, output.strip()[-500:] or "publisher count unavailable"
    return int(match.group(1)), output.strip()[-500:]


def _wheel_odom_process_count() -> tuple[int | None, list[int]]:
    """Count the single project-authoritative odom process.

    This is a local-process diagnostic only.  The WebUI may run on the robot
    while the one canonical ``go2w_wheel_odom`` publisher runs on the control
    host, so a zero local count is valid when the ROS graph reports exactly one
    odom publisher.  The backend applies that network-authority rule.
    """

    pids: list[int] = []
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                argv = [
                    os.fsdecode(item)
                    for item in (entry / "cmdline").read_bytes().split(b"\0")
                    if item
                ]
            except (OSError, PermissionError):
                continue
            if (
                any(Path(item).name == "go2w_wheel_odom" for item in argv)
                or any("sport_odom_bridge.py" in item for item in argv)
            ):
                pids.append(int(entry.name))
    except OSError:
        return None, []
    return len(pids), sorted(pids)


def count_cloud_api_call(profiler, node: Any, counter: str) -> bool:
    """按后端决定这次检测算不算一次云端调用；算就给计数桶 +1，返回是否计入。

    `--detector`（网页默认 `grounded_sam`）决定用哪条检测链：`llm` 走云端
    SiliconFlow，`grounded_sam` 走本机常驻的 GroundingDINO+SAM2 —— 后者一个
    云端请求都不发。`quick_api_calls` / `semantic_api_calls` 两个桶是给
    「今天打了几次云端」用的，本地检测加进去就是虚高，而且**不会报错**，
    只会让"云端调用量/耗时来源"这类判断一直偏。

    之所以把判断和 +1 放在同一个函数里（而不是在调用点写 `if ...: incr(...)`）：
    "本地检测不计入云端桶"这件事因此是个**可直接跑的行为**，而不是两行源码的
    先后顺序 —— 后者正是独立审查 R1 指出的、把错误固化下来的那种测试。

    拿不到 `_detector` 时按云端算（`--detector` 的默认值就是 `llm`）：漏记一次
    真实云端调用比多记一次更难发现。
    """
    if getattr(node, "_detector", "llm") != "llm":
        return False
    profiler.incr(counter)
    return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Operator-supervised high-level autonomous semantic exploration"
    )
    parser.add_argument("--target", required=True, help="自然语言搜索目标，如 饮水机旁边的蓝色垃圾桶")
    parser.add_argument("--task-context-json", default="")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--executor-id", default="")
    parser.add_argument("--worker-generation", type=int, default=None)
    parser.add_argument(
        "--backend", choices=("nav2", "go2w_experimental", "mock", "mock_metric"),
        default="nav2",
        help=(
            "robot backend: nav2 (real Go2-W, the only production backend) / "
            "go2w_experimental (legacy relative motion, not for autonomous search) / "
            "offline mocks"
        ),
    )
    parser.add_argument(
        "--reasoner", choices=("legacy", "semantic", "semantic_navigation", "hybrid"),
        default="semantic_navigation",
    )
    parser.add_argument(
        "--operator-supervised-experiment", action="store_true",
        help="operator present with remote; enables the experiment profile "
             "(authorizes <=30deg turns without a rotation lease; all other "
             "safety gates stay active)",
    )
    parser.add_argument(
        "--operator-authorized-rotation", action="store_true",
        help="legacy alias of --operator-supervised-experiment for the motion gate",
    )
    parser.add_argument("--max-seconds", type=float, default=600.0,
                        help="session duration limit in seconds; 0 = unlimited")
    parser.add_argument("--max-planning-cycles", type=int, default=100)
    parser.add_argument("--max-motion-steps", type=int, default=50)
    parser.add_argument("--finish-on-visual-confirmation", action="store_true",
                        default=True,
                        help="2D visual confirmation + relation evidence + verify PASS "
                             "ends the task (no metric 3D required)")
    parser.add_argument("--no-finish-on-visual-confirmation", action="store_true")
    parser.add_argument("--turn-only", action="store_true",
                        help="reject forward steps at the motion executor")
    parser.add_argument("--dry-run-motion", action="store_true",
                        help="run the full real pipeline (camera/LLM/SemanticNavigation/"
                             "planner) but never send motion commands "
                             "(REOBSERVE-only backend)")
    parser.add_argument("--output", default="outputs/live_sessions/semantic_exploration.jsonl")
    parser.add_argument("--session-dir", default="outputs/live_runs")
    # 计划书 §2.3：三维对象的上屏通路。本进程把内存里的持久实体投影成显示对象，
    # 声明自己属于哪张地图；地图桥写的那份轻量元数据是唯一的 epoch/坐标系来源。
    # 两者路径都由 Web 侧显式传给 worker，再透传到这里，不靠 glob 猜。
    parser.add_argument("--map-meta-path",
                        default="outputs/autonomous_search/runtime/slam_map_meta.json",
                        help="point-cloud-free map metadata written by the map bridge")
    parser.add_argument("--object-snapshot-path",
                        default="outputs/autonomous_search/runtime/semantic_objects.json",
                        help="where this process publishes its display objects")
    parser.add_argument("--record-video", default="",
                        help="record camera stream to MP4 (go2w backend only)")
    parser.add_argument("--dry-run", action="store_true",
                        help="offline run with scripted vision (alias of --backend mock)")
    parser.add_argument("--replay", default="",
                        help="replay observations from a previous session JSONL")
    parser.add_argument("--mock-scenario",
                        choices=("target_appears_after_n", "no_target", "anchor_then_target",
                                 "semantic_topology", "green_bin"),
                        default="anchor_then_target")
    parser.add_argument("--mock-target-after", type=int, default=3,
                        help="observations before the target appears (mock scenario)")
    parser.add_argument("--mock-confirm-after-seen", type=int, default=1)
    parser.add_argument("--verify-min-confidence", type=float, default=0.6)
    parser.add_argument(
        "--detector", choices=("llm", "grounded_sam", "yoloe_seg"), default="llm"
    )
    parser.add_argument("--llm-model", default="")
    parser.add_argument("--spool-root", default="runtime/go2w/spool")
    parser.add_argument("--rgbd-source", action="store_true",
                        help="use the D435 atomic RGB-D HTTP source as the primary "
                             "camera for observation (color+depth attached to LiveObservation)")
    parser.add_argument("--rgbd-base-url", default="http://192.168.123.18:8080")
    parser.add_argument("--spatial-v2", action="store_true",
                        help="enable SemanticNavigation V2 spatial exploration loop: PlaceGraph, "
                             "frontier selection, LongTermGoalSelector and LocalGoalExecutor")
    parser.add_argument("--spatial-provider",
                        choices=("camera", "rtabmap", "plain_slam", "nav2"),
                        default="camera",
                        help="metric spatial provider for the V2 loop: "
                             "camera (default, D435 BEV/camera-local), "
                             "rtabmap (/rtabmap/map + /rtabmap/odom), "
                             "plain_slam (/go2w/slam/map_2d + /go2w/slam/odom_base)")
    parser.add_argument("--rtabmap", action="store_true",
                        help="legacy alias of --spatial-provider rtabmap: use RTAB-Map "
                             "ROS2 topics (/rtabmap/map, /rtabmap/odom) as the "
                             "SpatialProvider; requires the D435 RGB-D bridge "
                             "and rtabmap_slam to be running")
    parser.add_argument("--plain-slam", action="store_true",
                        help="legacy alias of --spatial-provider plain_slam: use the "
                             "plain_slam PandarXT-16 mapping pipeline "
                             "(/go2w/slam/map_2d, /go2w/slam/odom_base)")
    parser.add_argument("--max-local-rotations", type=int, default=3,
                        help="bounded LOCAL_SCAN rotation quota per Place (plan §57)")
    parser.add_argument("--odom-topic", default="/go2w/odom/fused")
    parser.add_argument("--max-radius", type=float, default=0.0,
                        help="search radius limit in metres; 0 = unlimited")
    parser.add_argument("--target-score-min", type=float, default=0.15)
    parser.add_argument("--reach-area-ratio", type=float, default=0.15)
    parser.add_argument("--max-turn-deg", type=float, default=30.0)
    parser.add_argument("--forward-step-m", type=float, default=1.5)
    parser.add_argument("--max-observed-forward-m", type=float, default=None,
                        help="optional trial ceiling for measured displacement per forward goal")
    parser.add_argument("--max-navigation-goals", type=int, default=0,
                        help="high-level navigation goal attempt cap (0 = unrestricted)")
    parser.add_argument("--max-motion-commands", type=int, default=0,
                        help="physical primitive attempt cap, including corrections (0 = unrestricted)")
    parser.add_argument("--semantic-allow-forward", action="store_true")
    parser.add_argument("--allow-degraded", action="store_true",
                        help="continue when non-critical readiness checks degrade")
    parser.add_argument("--exploration-config", default="configs/exploration/default.yaml")
    parser.add_argument("--profile-config", default="configs/go2w/high_level_experiment.yaml")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.reasoner == "semantic":
        args.reasoner = "semantic_navigation"
    if args.no_finish_on_visual_confirmation:
        args.finish_on_visual_confirmation = False
    if args.dry_run and args.backend == "go2w_experimental":
        args.backend = "mock"
    # Compatible aliases: --rtabmap / --plain-slam map onto --spatial-provider.
    if args.rtabmap:
        args.spatial_provider = "rtabmap"
    if args.plain_slam:
        args.spatial_provider = "plain_slam"
    if args.replay:
        return run_replay(args)
    if args.backend in {"nav2", "go2w_experimental"}:
        return run_go2w(args)
    return _run_offline(args)


# ---------------------------------------------------------------------------
# shared artifacts
# ---------------------------------------------------------------------------


def _publish_display_objects(state: dict[str, Any], args, node,
                             semantic_map, revision: int) -> None:
    """§2.3：实体图更新后，把内存里的持久实体投影成 WebUI 的显示对象。

    `AutonomousExplorer.semantic_graph.object_map` 就是这里的 ``semantic_map``
    （run_go2w 把它注入 explorer），所以自主搜索与遥控语义建图走的是同一个投影
    函数。发布会话号取本进程的 ``--session-id``，用来在现场核验"页面上的物体是
    哪一场搜索写的"。

    拿不到地图桥的 ``map_epoch`` 时整个函数悄然不做任何事 —— 不上屏是正确行为，
    不是失败：地图桥可能还没起，而对象坐标属于哪张地图无从声明就不该画。
    """
    if semantic_map is None:
        return
    publisher = state.get("object_display_publisher")
    if publisher is None:
        from app.spatial.object_display_snapshot import ObjectDisplayPublisher

        publisher = ObjectDisplayPublisher(
            meta_path=PROJECT_ROOT / str(args.map_meta_path),
            snapshot_path=PROJECT_ROOT / str(args.object_snapshot_path),
            producer_session_id=str(getattr(args, "session_id", "") or ""),
        )
        state["object_display_publisher"] = publisher
    if publisher.publish(
            (entry.to_dict() for entry in semantic_map.objects.values()),
            graph_revision=revision):
        node.get_logger().debug(
            f"display objects published: revision={revision} "
            f"epoch={publisher.epoch} -> {publisher.snapshot_path}")


def _write_session_artifacts(args, explorer: AutonomousExplorer,
                             result, events: list[dict[str, Any]]) -> Path:
    session_id = explorer.session_id
    run_dir = Path(args.session_dir) / session_id
    run_dir.mkdir(parents=True, exist_ok=True)
    graph_path = explorer.save_artifacts(Path(args.session_dir))
    summary_path = run_dir / "summary.json"
    summary_path.write_text(
        json.dumps(result.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    if explorer.task_context is not None:
        (run_dir / "task.json").write_text(
            json.dumps(explorer.task_context.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
    (run_dir / "decisions.jsonl").write_text(
        "".join(json.dumps(item.to_dict(), ensure_ascii=False) + "\n" for item in explorer.decision_records),
        encoding="utf-8",
    )
    (run_dir / "semantic_map.json").write_text(
        json.dumps(explorer.semantic_graph.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    (run_dir / "final_state.json").write_text(
        json.dumps(
            {
                "schema_version": "search_final_state_v1",
                "session_id": session_id,
                "state": explorer.state,
                "result": result.to_dict(),
                "task": explorer.task_context.to_dict() if explorer.task_context else {},
                "current_place_id": explorer.semantic_graph.current_place_id,
                "map_revision": explorer.semantic_graph.revision,
                "last_decision": (
                    explorer.decision_records[-1].to_dict()
                    if explorer.decision_records else None
                ),
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ) + "\n",
        encoding="utf-8",
    )
    jsonl_path = Path(args.output)
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    with jsonl_path.open("a", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event, ensure_ascii=False) + "\n")
        handle.write(
            json.dumps(
                {"event": "session_summary", "session_id": session_id,
                 **result.to_dict()},
                ensure_ascii=False,
            )
            + "\n"
        )
    # Offline/replay runs do not have a metric map provider, but still persist
    # an explicit session epoch and bind their artifacts.  The live path
    # rewrites this manifest later with real map/graph/grid revisions.
    offline_revisions = WorldRevisions(
        map_id=f"session_map:{session_id}",
        map_epoch=f"session_epoch:{session_id}",
        graph_revision=int(getattr(explorer.semantic_graph, "revision", 0) or 0),
        grid_revision=int(getattr(explorer.semantic_graph, "revision", 0) or 0),
        semantic_revision=int(getattr(explorer.semantic_graph, "revision", 0) or 0),
        frame_id="offline_relative",
        sensor_profile_id="offline",
    )
    map_reference = run_dir / "map_reference.json"
    map_reference.write_text(
        json.dumps({
            "schema_version": "go2w_map_reference_v1",
            **offline_revisions.to_dict(),
            "map": {},
            "source": "offline_session",
        }, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    components = {
        name: run_dir / filename
        for name, filename in {
            "map_reference": "map_reference.json",
            "task": "task.json",
            "decisions": "decisions.jsonl",
            "summary": "summary.json",
            "final_state": "final_state.json",
            "exploration_graph": "exploration_graph.json",
            "semantic_map": "semantic_map.json",
        }.items()
        if (run_dir / filename).is_file()
    }
    write_world_manifest(
        run_dir,
        revisions=offline_revisions,
        components=components,
        metadata={
            "session_id": session_id,
            "run_mode": "offline_or_unversioned",
            "requires_relocalization_before_navigation": True,
            "motion_authority_restored": False,
        },
    )
    print(json.dumps(result.to_dict(), ensure_ascii=False))
    print(f"session_dir={run_dir}")
    print(f"graph={graph_path}")
    print(f"jsonl={jsonl_path}")
    return run_dir


def _build_offline_components(args):
    if args.mock_scenario == "semantic_topology":
        from app.live_robot.mock_observation_scene import scenario_semantic_topology

        return scenario_semantic_topology()
    if args.mock_scenario == "green_bin":
        from app.live_robot.mock_observation_scene import scenario_green_bin

        return scenario_green_bin()
    if args.mock_scenario == "target_appears_after_n":
        scene = scenario_target_appears_after(max(1, args.mock_target_after))
    elif args.mock_scenario == "no_target":
        scene = scenario_no_target()
    else:
        scene = scenario_anchor_then_target()
    scene.confirm_after_seen = max(1, args.mock_confirm_after_seen)
    return scene


# Explorer events that carry map meaning; the live WebUI worker augments
# these with the full exploration-graph snapshot for the SVG map.


def _run_offline(args, event_hook=None) -> int:
    policy = load_exploration_policy(args.exploration_config, overrides={
        "exploration": {
            "budget": {
                "max_search_seconds": args.max_seconds,
                "max_planning_cycles": args.max_planning_cycles,
                "max_motion_steps": args.max_motion_steps,
            },
        },
    })
    scene = _build_offline_components(args)
    backend_kind = "mock_metric" if args.backend == "mock_metric" else "mock"
    backend = create_backend(backend_kind)
    task_context = _task_context_from_args(args)
    graph = ExplorationGraph(session_id=args.session_id or time.strftime("explore_offline_%Y%m%d_%H%M%S"))
    holder: dict[str, Any] = {"explorer": None}

    def on_event(event: dict[str, Any]) -> None:
        if event_hook is not None:
            # 计划书 §3A：图不再挂在普通事件上。worker 的 event_hook 会在
            # revision 变化时单独发一条 graph_snapshot。
            event = event_hook(event, holder)
        if event is None:
            return
        explorer = holder.get("explorer")
        if explorer is not None:
            explorer.events.append(event)

    explorer = AutonomousExplorer(
        target=task_context.canonical_target,
        task_context=task_context,
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=backend,
        policy=policy,
        graph=graph,
        negative_target_key=task_context.canonical_target,
        finish_on_visual_confirmation=args.finish_on_visual_confirmation,
        turn_only=args.turn_only,
        session_id=graph.session_id,
        executor_id=args.executor_id or None,
        worker_generation=args.worker_generation,
        on_event=on_event,
    )
    holder["explorer"] = explorer
    result = explorer.run()
    _write_session_artifacts(args, explorer, result, explorer.events)
    return 0 if result.result == "TARGET_FOUND" else 3


def run_replay(args, event_hook=None) -> int:
    """Deterministic replay of a previous session's observation events."""
    jsonl_path = Path(args.replay)
    if not jsonl_path.is_file():
        print(json.dumps({"status": "failed", "error": f"replay file missing: {jsonl_path}"},
                         ensure_ascii=False))
        return 2
    observations: list[LiveObservation] = []
    for line in jsonl_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if event.get("event") == "observation":
            observation = LiveObservation.from_dict(event)
            observation.target_match = {
                "target_present": bool(event.get("target_present", False)),
                "target_state": str(
                    event.get("target_state")
                    or (event.get("target_match") or {}).get("target_state")
                    or ("PRESENT" if event.get("target_present") else "ABSENT")
                ),
                "score": (
                    float((event.get("target_match") or {}).get("score", 0.0))
                    if isinstance(event.get("target_match"), dict) else 0.0
                ),
            }
            observations.append(observation)
    if not observations:
        print(json.dumps({"status": "failed", "error": "no observation events in replay file"},
                         ensure_ascii=False))
        return 2
    policy = load_exploration_policy(args.exploration_config, overrides={
        "exploration": {
            "budget": {
                "max_search_seconds": args.max_seconds,
                "max_planning_cycles": args.max_planning_cycles,
                "max_motion_steps": args.max_motion_steps,
            },
        },
    })
    backend = create_backend("mock")
    index = [0]

    def observe() -> LiveObservation:
        obs = observations[min(index[0], len(observations) - 1)]
        index[0] += 1
        return obs

    def matcher(observation: LiveObservation) -> SemanticMatch:
        return SemanticMatch(
            has_candidate=bool(observation.target_present),
            target_match=observation.target_match,
            target_score=float((observation.target_match or {}).get("score", 0.0)),
            target_match_level="candidate" if observation.target_present else "none",
            target_state=observation.target_state,
            provenance={"source": "replay"},
        )

    def verifier(observation: LiveObservation,
                 match: SemanticMatch) -> VerificationOutcome:
        return VerificationOutcome(
            confirmed=bool(observation.target_present),
            attempts=1,
            reason_zh="replay verify",
        )

    replay_session_id = (
        args.session_id or f"replay_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    graph = ExplorationGraph(session_id=replay_session_id)
    holder: dict[str, Any] = {"explorer": None}

    def on_event(event: dict[str, Any]) -> None:
        if event_hook is not None:
            # 计划书 §3A：图不再挂在普通事件上。worker 的 event_hook 会在
            # revision 变化时单独发一条 graph_snapshot。
            event = event_hook(event, holder)
        if event is None:
            return
        explorer = holder.get("explorer")
        if explorer is not None:
            explorer.events.append(event)

    explorer = AutonomousExplorer(
        target=args.target,
        observer=observe,
        matcher=matcher,
        verifier=verifier,
        backend=backend,
        policy=policy,
        graph=graph,
        negative_target_key=args.target,
        finish_on_visual_confirmation=args.finish_on_visual_confirmation,
        turn_only=args.turn_only,
        session_id=graph.session_id,
        dry_run=bool(args.dry_run_motion),
        on_event=on_event,
    )
    holder["explorer"] = explorer
    result = explorer.run()
    _write_session_artifacts(args, explorer, result, explorer.events)
    return 0 if result.result == "TARGET_FOUND" else 3


# ---------------------------------------------------------------------------
# real Go2-W
# ---------------------------------------------------------------------------


#: 单个前进目标允许申请的最大距离。这是程序里的绝对天花板，档位配置只能比它
#: 更小（``limits.max_forward_step_m``）。3 m 是用户 2026-10-14 给的大场地口径；
#: 它不会变成一条 3 m 的物理指令 —— ``Go2WExperimentalBackend._execute_forward``
#: 仍把它拆成 ≤0.30 m 的短段，每段单独停下、过闸门、校验里程计。
MAX_FORWARD_STEP_M_CEILING = 3.0


def _runner_start_safety(args, node):
    """Robot/sensor health at startup; Nav2 checks geometry per planned path."""
    if args.backend == "nav2":
        return node._safety_ok(require_front_clearance=False)
    return node._safety_ok()


def _forward_duration_scale_for_run(
    profile_execution: dict[str, Any], max_navigation_goals: int,
) -> float:
    scale = float(profile_execution.get("forward_command_duration_scale", 1.0))
    # The historical 1.5x compensation drove a 0.15 m request to almost
    # 0.20 m in the supervised short-range trial.  Keep that calibration for
    # other profiles; this bounded trial must use the conservative duration.
    if max_navigation_goals > 0:
        scale = min(scale, 1.0)
    return max(0.5, min(2.0, scale))


def run_go2w(args, event_hook=None) -> int:
    if not (args.operator_supervised_experiment or args.operator_authorized_rotation):
        if args.dry_run_motion:
            # WebUI dry-run (plan book §60): the full real perception /
            # reasoning pipeline runs but no motion command is ever sent, so
            # operator rotation authorization is not required.
            print(json.dumps({"status": "dry_run_motion", "note": "no motion commands will be sent"},
                             ensure_ascii=False))
        else:
            print(
                json.dumps(
                    {
                        "status": "blocked",
                        "reason": (
                            "go2w_experimental backend requires "
                            "--operator-supervised-experiment (operator with remote "
                            "present); turns are blocked without operator authorization "
                            "because rotation clearance is fail-closed"
                        ),
                    },
                    ensure_ascii=False,
                )
            )
            return 3
    try:
        import rclpy
    except ImportError as exc:
        print(json.dumps({"status": "failed", "error": f"rclpy unavailable: {exc}"}))
        return 2
    from run_autonomous_loop import AutonomousLoop  # noqa: E402
    vision_model = str(args.llm_model or get_settings().vision_model)

    # Load the repository-owned operator-supervised profile at the real
    # backend boundary.  This does not invoke Stage-2/Nav2 readiness gates.
    experiment_profile = load_go2w_experiment_profile(args.profile_config)
    profile_name = str(
        experiment_profile.get("profile") or "operator_supervised_experiment"
    )
    profile_limits = experiment_profile.get("limits") or {}
    profile_motion = experiment_profile.get("motion_primitives") or {}
    profile_execution = experiment_profile.get("execution") or {}
    max_turn_deg = min(
        abs(float(args.max_turn_deg)),
        abs(float(profile_limits.get("max_turn_deg", args.max_turn_deg))),
        30.0,
    )
    forward_step_m = min(
        max(0.0, float(args.forward_step_m)),
        max(0.0, float(profile_limits.get("max_forward_step_m", 1.5))),
        MAX_FORWARD_STEP_M_CEILING,
    )
    trial_ledger_path=os.getenv("GO2W_NAV2_TRIAL_LEDGER", "").strip()
    selected_trial=None
    if trial_ledger_path:
        from app.navigation.trial_budget import NavigationTrialBudget
        selected_trial=NavigationTrialBudget(trial_ledger_path)
        forward_step_m = min(forward_step_m, 3. if selected_trial.field_search else
                             .45 if selected_trial.bounded_search else .12)
        max_turn_deg = min(max_turn_deg, 20. if selected_trial.field_search else 10.)

    policy = load_exploration_policy(args.exploration_config, overrides={
        "exploration": {
            "budget": {
                "max_search_seconds": args.max_seconds,
                "max_planning_cycles": args.max_planning_cycles,
                "max_motion_steps": args.max_motion_steps,
            },
            # Use the profile-clamped value so candidate generation cannot
            # propose a turn larger than the operator-supervised contract.
            "candidates": {"fallback_turn_deg": max_turn_deg},
        },
    })
    search_policy = load_search_policy()
    # There is one request-level compatibility switch for the terminal
    # behaviour.  Mirror it into the typed target-approach policy so the
    # explorer cannot accidentally use a default-only policy that contradicts
    # the CLI/WebUI request.
    search_policy.target_approach.finish_on_visual_confirmation = bool(
        args.finish_on_visual_confirmation
    )
    # 保留 Python 的 KeyboardInterrupt，别让 rclpy 的默认 SIGINT handler 先把
    # context 关掉。否则异常清理再调用 cancel/stop/disarm 时服务和 ActionClient
    # 已经失效，现场会留下 software_arm=true（2026-10-23 实测反例）。
    from app.ros_compat import init_preserving_python_signals
    init_preserving_python_signals()
    output_path = str(Path(args.output))
    node = AutonomousLoop(
        pattern=["f"], output=output_path, forward_vx=0.12,
        forward_seconds=2.0, max_yaw_rate=1.0, min_clearance_m=0.30,
        mode="state_machine_search", max_seconds=args.max_seconds,
        wander_front_go_m=0.45, wander_turn_deg=max_turn_deg,
        max_radius_m=args.max_radius, scan_turn_deg=max_turn_deg, scan_span=3,
        pre_scan_turns=0, record_video=args.record_video,
        video_fps=15.0, video_scale=0.4, scan360_steps=8,
        scan360_turn_deg=45.0, odom_topic=args.odom_topic,
    )
    node._target = args.target
    node._detector = args.detector
    node._llm_model = vision_model
    node._spool_root = args.spool_root
    node._target_score_min = args.target_score_min
    node._align_threshold = 0.08
    node._align_yaw_max_deg = 25.0
    node._reach_area_ratio = args.reach_area_ratio
    node._semantic_reasoning = True
    node._search_reasoner = args.reasoner
    node._search_reasoner_mode = "active"
    node._semantic_allow_forward = bool(args.semantic_allow_forward)
    node._front_half_plane_only = False
    node._turn_only = bool(args.turn_only)
    node._max_motion_steps = 0
    node._min_rotation_clearance_m = 0.0
    node._operator_authorized_rotation = bool(
        args.operator_supervised_experiment or args.operator_authorized_rotation
    )
    node._rotation_lease_path = ""
    node._rotation_lease_error = ""
    node._experiment_profile_name = profile_name
    node._experiment_motion_primitives = {
        str(name): bool(enabled) for name, enabled in profile_motion.items()
    }
    node._experiment_max_turn_deg = max_turn_deg
    node._experiment_forward_step_m = forward_step_m
    node._experiment_field_search = bool(selected_trial and selected_trial.field_search)
    # A long semantic move is executed as independently stopped and verified
    # chunks.  Each chunk re-enters the LiDAR/mode/arm safety gates.
    node._experiment_forward_segment_m = min(0.30, forward_step_m)
    # 失败后的自动倒车（``_maybe_autonomous_reverse``）走的是 ``b<metres>`` 原语，
    # 所以它的开关必须跟档位自己声明的 REVERSE 一致，不能再挂在"有没有目标次数
    # 上限"上：大场地档 ``max_navigation_goals=0``，按旧口径会把倒车打开，而操作员
    # 在确认横幅上读到的是 ``REVERSE: false``。两处必须同源。
    node._disable_autonomous_reverse = not bool(
        node._experiment_motion_primitives.get("REVERSE", False)
    )
    node._forward_duration_scale = _forward_duration_scale_for_run(
        profile_execution, args.max_navigation_goals,
    )

    try:
        return _run_go2w_explorer(args, node, policy, event_hook,
                                  task_context=_task_context_from_args(args))
    except KeyboardInterrupt:
        # 先解除物理授权，再结算 Action/账本。即使账本落盘或 Action cancel 失败，
        # 也不能让它阻止 disarm；若 disarm 服务本身失败，再升级成硬停止。
        disarmed = not bool(node._armed_by_runner)
        if not disarmed:
            try:
                node._arm(False)
                disarmed = True
            except Exception:
                pass
        active_backend = getattr(node, "_search_nav2_backend", None)
        if active_backend is not None:
            try:
                active_backend.stop()
            except Exception:
                pass
        if not disarmed:
            try:
                node._emergency_stop()
            except Exception:
                pass
        try:
            node.get_logger().warn("operator interrupted semantic exploration; motion stopped")
        except Exception:
            pass
        return 130
    except Exception as exc:
        # Keep the worker protocol concise, but retain the complete traceback
        # in the ROS log.  A plain errno (especially from a relative runtime
        # path) is otherwise impossible to distinguish from a sensor/API
        # failure when the WebUI only receives the terminal exit code.
        import traceback

        node.get_logger().error(
            f"semantic exploration failed: {exc}\n{traceback.format_exc()}"
        )
        active_backend = getattr(node, "_search_nav2_backend", None)
        if active_backend is not None:
            active_backend.stop()
        if node._armed_by_runner:
            try:
                node._emergency_stop()
            except Exception:
                pass
            try:
                node._arm(False)
            except Exception:
                pass
        return 4
    finally:
        # nav2 的 IO 节点 + 两个 ActionClient + 走它的线程。**这一段单独成函数**
        # 是为了让它自己吞掉失败（见 _teardown_nav2_io 的注释）：它原本是裸的
        # 属性链，一旦哪一步抛出来，后面的 provider/manager/output/video/
        # destroy_node 全部不再执行，而且异常顶掉本函数真正的返回码。
        failures = _teardown_nav2_io(node)
        if failures:
            try:
                node.get_logger().warn(
                    "nav2 IO 收尾未干净（其余清理照常进行）：" + "、".join(failures))
            except Exception:
                pass
        provider = getattr(node, "_search_spatial_provider", None)
        if provider is not None:
            try:
                provider.close()
            except Exception:
                pass
        # Disable background Full Semantic submissions before closing the
        # event stream they use.  In-flight calls are daemon threads and may
        # finish later, but they no longer schedule follow-up work.
        try:
            manager = getattr(node, "_semantic_manager", None)
            if manager is not None:
                manager.close()
        except Exception:
            pass
        # §5.1.5：关掉异步复核的提交入口。**在飞的那条不会取消**（daemon 取消
        # 不了），它晚到的结果会因为 manager 已关而没人再读 —— 这正是设计。
        try:
            verifier_manager = getattr(node, "_async_verifier", None)
            if verifier_manager is not None:
                verifier_manager.close()
        except Exception:
            pass
        try:
            node._output.close()
        except Exception:
            pass
        if args.record_video and node._video is not None:
            try:
                node._video.stop()
            except Exception:
                pass
        # rclpy Node.destroy_node() does not own all ActionClient entities on
        # every supported ROS 2 build.  Destroy the client first so its late
        # goal/result handles cannot touch an already-invalid context during
        # shutdown.  The semantic manager is daemon-threaded, but disable it
        # before the ROS context goes away so no late event is emitted into a
        # closed output stream.
        try:
            node._client.destroy()
        except Exception:
            pass
        # 同一条规矩：这是 finally 里的最后一步，抛出去等于 `rclpy.shutdown()`
        # 永远不执行，而它上面每一处清理都写了 try/except。
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


def _teardown_nav2_io(node: Any) -> list[str]:
    """收掉 nav2 的 IO 节点、它上面挂的两个 ActionClient，以及走它的线程（§P3-1）。

    **它自己不抛，只把没干净收掉的步骤名报回来。** 调用点在 ``finally`` 里，而
    ``finally`` 里抛出去有两重后果，都不是"多一条报错"那么轻：

    * 后面**所有**清理都不再执行 —— ``provider.close()`` / ``manager.close()`` /
      ``node._output.close()`` / 视频停止 / ``node.destroy_node()``。这个块里每一
      步都写了 try/except，唯独这一段没有，于是一次收尾异常会让整场运行的产物
      （``node._output`` 那条流）留在没关的状态；
    * 异常顶掉 ``_run_go2w_explorer`` 真正的返回值（``0/2/3/4``）。操作员读到的
      是一次"收尾崩了"，而运行**为什么**结束（真正的原因在 ``except`` 那一支里
      已经记进日志了）被换成了别的。

    写成一串各自独立的 try 而不是一个大 try：一处的失败不该让后面的资源泄漏。
    每条失败都记名返回，由调用方决定怎么报 —— 不静默。

    **幂等**：先把 ``_nav2_io_executor`` / ``_nav2_io_node`` 摘掉再拆。这是本仓
    已有的写法（``Nav2SpatialProvider.close()`` 先把 ``self._executor`` 置 None，
    ``PlainSlam`` / ``Rtabmap`` provider 先把 ``self._node`` 置 None），第二次
    调用是空操作，而不是对着已经拆掉的 executor / node 再拆一次。

    属性一律走 ``getattr(..., None)``：这些对象是**分四行**依次挂到 ``node`` 上的
    （见 ``run_go2w`` 里创建 nav2 后端那一段：先 backend、再 io_node、再 executor、
    最后才 thread 并 start），中间任何一行失败都会留下"executor 有了、thread 还
    没有"这种半成品状态。原来的收尾直接写 ``node._nav2_io_thread`` /
    ``node._search_nav2_backend._inner``，用的是裸属性链，赌的是"executor 在
    ⇒ 后面几样都在"——那个不变量没人保证，而它一旦不成立，异常正好发生在最该
    安静的那一段里。
    """
    executor = getattr(node, "_nav2_io_executor", None)
    io_node = getattr(node, "_nav2_io_node", None)
    thread = getattr(node, "_nav2_io_thread", None)
    if executor is None and io_node is None:
        return []
    node._nav2_io_executor = None
    node._nav2_io_node = None
    node._nav2_io_thread = None
    failures: list[str] = []

    if executor is not None:
        try:
            executor.shutdown(timeout_sec=2.)
        except Exception:  # noqa: BLE001
            failures.append("io_executor.shutdown")

    if thread is not None:
        try:
            thread.join(timeout=2.)
        except Exception:  # noqa: BLE001
            failures.append("io_thread.join")

    # 顺序是 executor → thread → clients → node：client 长在 io_node 上，必须
    # 先于它销毁（rclpy 的 destroy_node 不接管所有 ActionClient 实体，晚到的
    # goal/result 句柄会碰已经失效的 context）。
    inner = getattr(getattr(node, "_search_nav2_backend", None), "_inner", None)
    for name in ("_path_client", "_navigate_client", "_spin_client"):
        client = getattr(getattr(inner, name, None), "_client", None)
        if client is None:
            continue
        try:
            client.destroy()
        except Exception:  # noqa: BLE001
            failures.append(name + ".destroy")

    if io_node is not None:
        try:
            io_node.destroy_node()
        except Exception:  # noqa: BLE001
            failures.append("io_node.destroy_node")
    return failures


_FRONTIER_REACH_RADIUS_M = 1.0
_FRONTIER_NO_MOTION_MAX_M = 0.05
_FRONTIER_FAIL_ATTEMPTS = 3


def _settle_frontier_selection(
    state: dict[str, Any],
    spatial_memory: Any,
    frontiers: list[Any],
    current_position: tuple[float, float] | None,
) -> None:
    """结算「上一次选中的前沿」这次算走到、还是走不到。

    ``SpatialMemory.mark_frontier_visited`` 在整份代码里原本**零调用点** —— 被
    选中的前沿永远只加 ``selected_count``，``visit_count`` / ``fail_count`` 永远
    是 0，"试过但到不了"的前沿永远进不了黑名单。这里是它唯一的写入口。

    判据全部要求**正向证据**，拿不到就不写（fail-closed）：
    * 前沿还在当前地图的前沿集里、且当前位置已进入到达半径 ⇒ visit；
    * 否则累计尝试次数；连续 ``_FRONTIER_FAIL_ATTEMPTS`` 次规划之后机器人
      **一点没挪** ⇒ fail；
    * 有位移但没到 ⇒ 什么都不写（既不是"到过"，也不是"到不了"）；
    * 前沿从地图上消失了 ⇒ 不写。那通常是走穿了，但也可能只是地图重建，
      两者分不清，所以不给证据就不记账。

    已知的误伤面：机器人自己卡住时位移也是 0，失败会被错记到前沿头上。缓解不是
    "不触发"（那会退回静默 no-op），而是下游把黑名单当**扣分**而不是**剔除** ——
    误伤最多让一个前沿降权，绝不把它硬排除、更不会据此判 SEARCH_EXHAUSTED。
    """
    if spatial_memory is None:
        return
    settle = state.get("last_frontier_selection")
    if not isinstance(settle, dict):
        return
    frontier_id = str(settle.get("frontier_id") or "")
    if not frontier_id:
        return
    current = next((f for f in frontiers if f.frontier_id == frontier_id), None)
    if current is None or current_position is None:
        return
    target = current.position
    if target is None:
        return
    distance = math.hypot(
        float(current_position[0]) - float(target[0]),
        float(current_position[1]) - float(target[1]),
    )
    if distance <= _FRONTIER_REACH_RADIUS_M:
        spatial_memory.mark_frontier_visited(frontier_id, failed=False)
        state.pop("last_frontier_selection", None)
        return
    settle["attempts"] = int(settle.get("attempts") or 0) + 1
    pose = settle.get("pose")
    moved = (
        math.hypot(float(current_position[0]) - float(pose[0]),
                   float(current_position[1]) - float(pose[1]))
        if pose is not None else None
    )
    if settle["attempts"] >= _FRONTIER_FAIL_ATTEMPTS and moved is not None \
            and moved < _FRONTIER_NO_MOTION_MAX_M:
        spatial_memory.mark_frontier_visited(frontier_id, failed=True)
        # 记满一笔之后把窗口清零、并以当前位姿为新基准。**不能**用一个
        # "已经记过就不再记"的闩锁收尾 —— ``SpatialMemory.blacklisted`` 要
        # ``fail_count >= 2`` 才置位，闩锁会让它永远是 False，黑名单这条链
        # 表面接上了、实际一次都不会触发（正是这次要修的静默 no-op 反过来
        # 长在我自己手上）。清零而不是累积，也避免了"同一段停滞被反复记账"。
        settle["attempts"] = 0
        settle["pose"] = current_position


def _run_go2w_explorer(args, node, policy, event_hook=None,
                     task_context=None) -> int:
    import rclpy  # noqa: F401  (used by the observer closure; rclpy.init already done)
    from run_autonomous_loop import target_prompt  # noqa: E402
    settings = get_settings()
    # §15 第四条：启动时把 12 个阈值的**生效值**打出来。这是范围校验的唯一
    # 出口 —— 越界时 app.config 会在 stderr 上出声并回落默认值，但只有把这
    # 一行和 .env 对着看，才知道自己设的数到底有没有生效。
    print(
        "[GO2W-CONFIG] "
        + " ".join(
            f"{name}={getattr(settings, attr)}"
            for name, attr in (
                ("LIO_ROTATION_WHEEL_MAX_TRANSLATION_M",
                 "go2w_lio_rotation_wheel_max_translation_m"),
                ("LIO_ROTATION_PSLAM_MAX_TRANSLATION_M",
                 "go2w_lio_rotation_pslam_max_translation_m"),
                ("LIO_MAX_POSE_SPEED_MPS", "go2w_lio_max_pose_speed_mps"),
                ("LIO_RECOVERY_VALID_SAMPLES", "go2w_lio_recovery_valid_samples"),
                ("PLACE_MERGE_DISTANCE_M", "go2w_place_merge_distance_m"),
                ("PLACE_RELOCATION_MIN_M", "go2w_place_relocation_min_m"),
                ("PERCEPTION_SETTLE_SECONDS", "go2w_perception_settle_seconds"),
                ("TARGET_POSSIBLE_VERIFY_ATTEMPTS",
                 "go2w_target_possible_verify_attempts"),
                ("TARGET_POSSIBLE_REOBSERVE_DELAY_SECONDS",
                 "go2w_target_possible_reobserve_delay_seconds"),
                ("RELATION_LEFT_RIGHT_MIN_NORM", "go2w_relation_left_right_min_norm"),
                ("RELATION_DEPTH_ORDER_MIN_M", "go2w_relation_depth_order_min_m"),
                ("RELATION_NEAR_MAX_M", "go2w_relation_near_max_m"),
                # 2026-09-17：持久对象图的入口门。它决定"哪些观测能变成实体"，
                # 调错了在页面上看不出来（少几个框），所以必须和 .env 对着看。
                ("SEMANTIC_MAP_MIN_OBJECT_CONFIDENCE",
                 "semantic_map_min_object_confidence"),
            )
        ),
        flush=True,
    )
    search_policy = load_search_policy()
    # The CLI/WebUI flag is the per-run authority for whether confirmation is
    # legacy-terminal. Keep the typed approach policy's observable contract in
    # sync with that request.
    search_policy.target_approach.finish_on_visual_confirmation = bool(
        args.finish_on_visual_confirmation
    )
    env = node._load_detector_env()
    # `vision_model` is used by the nested ``analyze_semantic`` observer.  It
    # is a local variable of the caller ``run_go2w`` and is never passed into
    # this function, so re-derive it here (fixes NameError in the worker).
    vision_model = str(args.llm_model or settings.vision_model or "Qwen/Qwen3-VL-8B-Instruct")
    # Keep proxy variables in the child env; the robot/developer host may
    # require the local forward proxy for external SiliconFlow API access.
    # Ensure the detector subprocess uses a real interpreter on this machine
    # instead of the author's hard-coded path.
    if not env.get("SILICONFLOW_PYTHON") and not env.get("GROUNDED_SAM_PYTHON"):
        for candidate in (
            os.environ.get("GO2W_CONDA_PYTHON"),
            str(PROJECT_ROOT / ".runtime_venv/bin/python"),
            str(PROJECT_ROOT / ".venv/bin/python"),
            sys.executable,
        ):
            if candidate and Path(candidate).is_file():
                env["SILICONFLOW_PYTHON"] = candidate
                break
    spool_root = args.spool_root
    state: dict[str, Any] = {
        "image_path": None,
        "semantic": None,
        "common_sense_pending": False,
        "common_sense_updated_ts": 0.0,
        "local_detection_by_frame": {},
    }
    # 计划书 §5.1/§14：RGB-D 帧缓存（有界）与运动后新帧门控状态。
    rgbd_frame_cache: dict[str, Any] = {}
    motion_state: dict[str, Any] = {
        "motion_end_timestamp": 0.0,
        "last_frame_id_before_motion": None,
        "last_yaw_before": None,
        "pending_motion_evidence": None,
        # 计划书 §16.1 的 mapping_health_changed 要的是"变化"，不是每帧一条。
        # 放这里而不是闭包外，是为了跟 pending_motion_evidence 同一个生命周期。
        "last_mapping_health": None,
    }

    def _last_success_age_s() -> float | None:
        ts = state.get("last_success_ts")
        if ts is None:
            return None
        return max(0.0, time.time() - float(ts))
    operator_authorized = bool(
        args.operator_supervised_experiment or args.operator_authorized_rotation
    )

    # ---- pre-run sensor waits (mirrors the audited runner) ------------------
    if not node._wait_for(lambda: node._sport is not None and node._odom is not None,
                          10.0, "sport/odom"):
        print(json.dumps({"status": "failed", "error": "sport/odom unavailable"},
                         ensure_ascii=False))
        return 2
    if not node._wait_for(
        lambda: node._lidar_fresh is True and node._clearance is not None,
        5.0, "fresh LiDAR clearance",
    ):
        print(json.dumps({"status": "failed", "error": "LiDAR clearance unavailable"},
                         ensure_ascii=False))
        return 2
    node._motion_origin = node._odom_snapshot()
    ready, reason = _runner_start_safety(args, node)
    if not ready:
        node._write({"event": "pre_arm_safety_reject", "reason": reason,
                     "host_s": node._host_s()})
        print(json.dumps({"status": "blocked", "reason": reason}, ensure_ascii=False))
        return 2

    # ---- automatic experiment readiness ------------------------------------
    readiness = _probe_readiness(args, node)
    print(json.dumps(readiness.to_dict(), ensure_ascii=False))
    node._write({"event": "experiment_readiness", "host_s": node._host_s(),
                 **readiness.to_dict()})
    if not readiness.ready and not args.allow_degraded:
        print(json.dumps({"status": "blocked", "reason": readiness.reason},
                         ensure_ascii=False))
        return 3

    # ---- semantic stack -----------------------------------------------------
    semantic_memory = SemanticSearchMemory(
        default_ttl_sec=policy.budget.negative_memory_ttl_seconds,
        observation_store=(
            ObservationMemoryStore(settings=settings)
            if settings.live_search_reasoner_use_observation_memory else None
        ),
    )
    llm_prior_generator = LLMPriorGenerator(settings=settings)
    # 计划书 §5.1.1：画像**只在会话启动时解析一次**，不逐帧。`use_llm` 由开关
    # 决定（默认 False = 此前网页路径的硬编码值）。CLI 路径用 True，网页此前
    # 恒 False，所以 LLM 派生的上下文词/关系锚点词从未进过网页的检测词表。
    profile = TargetProfileResolver().resolve(
        args.target, use_llm=bool(settings.go2w_web_target_profile_use_llm)
    )
    controller = SemanticSearchController(
        profile,
        backend=args.reasoner,
        partial_threshold=settings.live_search_graph_match_partial_threshold,
        strong_threshold=settings.live_search_graph_match_strong_threshold,
    )
    # §5.1.1 接线：把画像交给 node，检测器构造时才会拿到它（`_ensure_gsam_detector`
    # 读的是 `node._target_profile`）。**此前这里没有这一行** —— 赋值只发生在
    # run_autonomous_loop.py 的 `_run_state_machine_search` 里，而网页路径
    # run_go2w → _run_go2w_explorer → AutonomousExplorer.run() 从不经过那个函数，
    # 于是 `GroundedSAMSubprocessDetector(target_profile=None)`，
    # `_build_prompts` 丢掉 detector_terms()/context_labels_en/relation_anchor_labels_en。
    #
    # `_close_detector()` 是**目标切换失效**：`_ensure_gsam_detector` 首次调用即
    # 缓存实例，不在这里关掉的话，换目标会继续用上一轮那个词表构造的 worker。
    if settings.go2w_web_target_profile_wiring:
        node._target_profile = profile
        node._close_detector()
    if getattr(node, "_detector", "llm") == "yoloe_seg":
        # Fail before the explorer can submit any Nav2 goal.  There is no
        # silent fallback: model/CUDA/protocol health is a startup gate.
        detector_health = node._ensure_local_detector(env).health()
        effective_backend = str(detector_health.get("backend") or "")
        if effective_backend != "yoloe_seg":
            raise RuntimeError(
                "YOLOE worker backend mismatch: "
                f"requested=yoloe_seg effective={effective_backend or '<missing>'}"
            )
        node._write({
            "event": "detector_startup_health",
            "detector_backend": effective_backend,
            "fallback_used": False,
            "health": detector_health,
            "host_s": node._host_s(),
        })
    prompt = target_prompt(args.target)
    target_contract_enabled = bool(
        args.backend == 'nav2' and settings.go2w_result_role_separation
        and args.rgbd_source and args.spatial_v2
    )

    def analyze_semantic(
        image_path: object,
        _profile: object,
        *,
        request_id: str | None = None,
        frame_id: str | None = None,
        robot_pose: dict[str, Any] | None = None,
    ) -> dict:
        if not isinstance(image_path, str):
            raise RuntimeError("semantic observation has no stable image")
        if is_local_structured_backend(getattr(node, "_detector", "llm")):
            # Local structured backends are inferred exactly once by observe().
            # The same full-frame result feeds target candidates, scene memory,
            # geometry and visualization; this background adapter is cache-only.
            frame_id_text = str(frame_id or state.get("frame_id", "semantic_live"))
            cached = (state.get("local_detection_by_frame") or {}).get(frame_id_text)
            if cached is None:
                raise PerceptionFailure(
                    "LOCAL_DETECTION_CACHE_MISS: semantic analysis requested before "
                    "the scheduled full-frame detection completed",
                    code="LOCAL_DETECTION_CACHE_MISS",
                    recoverable=True,
                )
            scene_objects = []
            context_objects = cached.get("objects") or []
            if target_contract_enabled:
                context_objects = local_detection_scene_objects(
                    context_objects, _profile,
                    source_frame_id=str(cached.get("source_frame_id") or ''),
                    current_frame_id=frame_id_text,
                )
            for index, obj in enumerate(context_objects, start=1):
                item = dict(obj)
                item["id"] = f"local_obj_{index:03d}"
                # 只在检测器没给类别时才补默认值。此前这里是无条件硬赋值，
                # 把 detector 已经算好的 container / furniture / electronics
                # 一律抹成 "object"，下游再也分不出「地面」和「垃圾桶」
                # （计划书 §2.2 第 5 条）。
                item.setdefault("category", "object")
                if not item.get("category"):
                    item["category"] = "object"
                scene_objects.append(item)
            return {
                "image_path": image_path,
                "frame_id": str(cached.get("source_frame_id") or frame_id_text),
                "robot_pose": cached.get("robot_pose") or robot_pose,
                "scene_objects": scene_objects,
                "scene_relations": [],
                "source": f"{getattr(node, '_detector', 'local')}_subprocess",
                "semantic_status": SEMANTIC_STATUS_FRESH_FULL,
                "semantic_quality": "full",
                "semantic_source_frame_id": str(
                    cached.get("source_frame_id") or frame_id_text
                ),
                "semantic_capture_timestamp": float(
                    cached.get("capture_timestamp") or time.time()
                ),
                "semantic_completed_timestamp": time.time(),
                "semantic_age_ms": 0.0,
                "semantic_source_pose": cached.get("robot_pose") or robot_pose,
            }
        # 每次请求使用独立 output 文件，避免并发 Full Semantic 互相覆盖。
        request_id = request_id or f"{time.time_ns()}_{os.getpid()}"
        request_dir = PROJECT_ROOT / "runtime/go2w/vlm_requests"
        request_dir.mkdir(parents=True, exist_ok=True)
        output_path = request_dir / f"semantic_{request_id}.json"
        # 优先走长驻 VLM daemon；不可用时再回退到 subprocess（legacy fallback）。
        try:
            from app.detectors.siliconflow_vision_protocol import (
                SiliconFlowDaemonClient,
                VLMRequest,
            )

            daemon_client = SiliconFlowDaemonClient(
                str(PROJECT_ROOT / "runtime/go2w/siliconflow_vlm.sock"),
                timeout=max(45.0, float(settings.siliconflow_timeout_seconds) + 15.0),
            )
            if daemon_client.available():
                daemon_request = VLMRequest(
                    request_id=f"semantic_{request_id}",
                    mode="semantic",
                    image_path=image_path,
                    frame_id=str(frame_id or state.get("frame_id", "semantic_live")),
                    target=args.target,
                    priority="background",
                    extra_instructions=(
                        "完整列出当前画面的可见物体与关系，供下一视角选择；不要确认目标。"
                    ),
                    model=vision_model,
                    robot_pose=robot_pose,
                )
                daemon_response = daemon_client.request(daemon_request)
                if daemon_response.ok and isinstance(daemon_response.payload, dict):
                    daemon_payload = dict(daemon_response.payload)
                    daemon_payload.update({
                        "image_path": image_path,
                        "frame_id": str(frame_id or state.get("frame_id", "semantic_live")),
                        "robot_pose": robot_pose,
                        "source": "siliconflow_full_scene_vlm_daemon",
                        "semantic_status": SEMANTIC_STATUS_FRESH_FULL,
                        "semantic_quality": "full",
                        "semantic_source_frame_id": str(
                            frame_id or state.get("frame_id", "semantic_live")
                        ),
                        "semantic_capture_timestamp": time.time(),
                        "semantic_completed_timestamp": time.time(),
                        "semantic_age_ms": 0.0,
                        "semantic_source_pose": robot_pose,
                    })
                    return daemon_payload
        except Exception:
            # 任何 daemon IPC 问题都走 subprocess fallback，不中断搜索。
            pass
        # 始终走全场景分析（objects + relations 一起拿）。快速“无目标”捷径只返回
        # objects、不返回 relations，会导致拓扑只有孤立节点；全场景更长但能
        # 既建节点又连边（用 max_tokens=2048，不截断）。
        python = env.get(
            "SILICONFLOW_PYTHON",
            env.get(
                "GROUNDED_SAM_PYTHON",
                sys.executable,
            ),
        )
        worker = PROJECT_ROOT / "app/detectors/siliconflow_vision_worker.py"
        command = [
            python, str(worker), "--image", image_path,
            "--output", str(output_path), "--target", args.target,
            "--extra-instructions",
            "完整列出当前画面的可见物体与关系，供下一视角选择；不要确认目标。",
            "--model", vision_model,
        ]
        last_err = ""
        completed = None
        semantic_timeout = max(
            35.0,
            min(90.0, float(settings.siliconflow_timeout_seconds) + 15.0),
        )
        for attempt in range(2):
            try:
                completed = subprocess.run(
                    command, cwd=str(PROJECT_ROOT), env=env, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    timeout=semantic_timeout, check=False,
                )
            except subprocess.TimeoutExpired as exc:
                # 计划书 §3.2 不变量 1：timeout ≠ empty scene。必须抛结构化
                # 异常让后台管理器标记语义不可用，绝不返回“空场景成功”。
                raise PerceptionFailure(
                    "FULL_SEMANTIC_TIMEOUT: full-scene analysis exceeded "
                    f"{semantic_timeout:.0f}s",
                    code="FULL_SEMANTIC_TIMEOUT",
                    recoverable=True,
                    detail=str(exc),
                ) from exc
            if completed.returncode == 0:
                break
            last_err = completed.stderr[-600:]
        if completed is None or completed.returncode != 0:
            # 单帧视觉解析失败绝不中断整场搜索：后台管理器保持最近一次成功
            # 的 latest_success，当前帧用 Quick 快路径结果继续（计划书 §8.4）。
            node.get_logger().warn(
                f"semantic observer failed (2 tries): {last_err}"
            )
            raise PerceptionFailure(
                f"FULL_SEMANTIC_ERROR: {last_err}",
                code="FULL_SEMANTIC_ERROR",
                recoverable=True,
                detail=last_err,
            )
        if not output_path.is_file():
            raise PerceptionFailure(
                f"semantic output missing: {output_path}",
                code="VLM_PARSE_ERROR",
                recoverable=True,
            )
        try:
            payload = json.loads(output_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            raise PerceptionFailure(
                f"semantic output unreadable: {exc}",
                code="VLM_PARSE_ERROR",
                recoverable=True,
            ) from exc
        payload.update({
            "image_path": image_path,
            "frame_id": str(frame_id or state.get("frame_id", "semantic_live")),
            "robot_pose": robot_pose,
            "source": "siliconflow_full_scene_existing_pipeline",
            "semantic_status": SEMANTIC_STATUS_FRESH_FULL,
            "semantic_quality": "full",
            "semantic_source_frame_id": str(frame_id or state.get("frame_id", "semantic_live")),
            "semantic_capture_timestamp": time.time(),
            "semantic_completed_timestamp": time.time(),
            "semantic_age_ms": 0.0,
            "semantic_source_pose": robot_pose,
        })
        return payload

    semantic_observer = LiveSemanticObserver(
        analyze_semantic,
        ttl_seconds=settings.live_search_reasoner_scene_ttl_seconds,
    )
    # 后台 Full Semantic 管理器：普通帧不再同步等待全场景分析。
    semantic_manager = AsyncSemanticObservationManager(
        analyze_semantic,
        enabled=settings.vlm_runtime_semantic_background_enabled,
        max_inflight=settings.vlm_runtime_semantic_max_inflight,
        ttl_seconds=max(5.0, settings.vlm_runtime_semantic_ttl_seconds),
        translation_refresh_m=settings.vlm_runtime_semantic_translation_refresh_m,
        heading_sector_deg=settings.vlm_runtime_semantic_heading_sector_deg,
        initial_warmup_blocking=settings.vlm_runtime_semantic_initial_warmup_blocking,
        visual_change_enabled=settings.vlm_runtime_semantic_visual_change_enabled,
        visual_change_threshold=None,
        event_sink=node._write,
    )
    # Keep the manager reachable from the outer lifecycle cleanup.  Its
    # request threads are daemonized, but they must be disabled before the
    # ROS node/output stream is torn down.
    node._semantic_manager = semantic_manager
    observed_sectors: set[int] = set()

    # Optional D435 atomic RGB-D source (plan §17/§22).
    rgbd_source = None
    depth_localizer = None
    if args.rgbd_source:
        from app.perception.depth_object_localizer import DepthObjectLocalizer
        from app.perception.realsense_http_rgbd_source import RealSenseHTTPRGBDSource

        rgbd_source = RealSenseHTTPRGBDSource(
            args.rgbd_base_url,
            cache_dir=str(PROJECT_ROOT / "runtime/go2w/rgbd_cache"),
        )
        depth_localizer = DepthObjectLocalizer()

    # Optional SemanticNavigation V2 spatial exploration state (plan §62-§90).
    place_graph = None
    semantic_map = None
    semantic_mobility_tracker = None
    target_candidate_mapper = None
    last_semantic_map_odom_pose = None
    entity_graph = None
    route_planner = None
    spatial_memory = None
    camera_provider = None
    bev_mapper = None
    pose_validator = None
    frontier_extractor = None
    psg_provider = None
    spatial_reasoner = None
    local_executor = None
    if args.spatial_v2:
        from app.navigation.decision_record import build_decision_record
        from app.navigation.local_goal_executor import LocalGoalExecutor
        from app.navigation.long_term_goal_selector import LongTermGoalSelector
        from app.navigation.semantic_route_planner import SemanticRoutePlanner
        from app.reasoning.semantic_navigation.semantic_prior_provider import RuleSemanticPriorProvider
        from app.reasoning.semantic_navigation.spatial_reasoner import SpatialSearchReasoner
        from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
        from app.spatial.frontier_extractor import FrontierExtractor
        from app.spatial.lightweight_depth_bev import LightweightDepthBEVMapper
        from app.spatial.place_graph import PlaceGraph
        from app.spatial.semantic_entity_graph import SemanticEntityGraph
        from app.spatial.semantic_object_map import SemanticMapUpdateResult, SemanticObjectMap
        from app.spatial.spatial_memory import SpatialMemory

        # §15：这几个阈值以前是写死在这里的字面量，而 PlaceGraph /
        # SpatialPoseValidator 的模块默认值是另一套数（0.6 / 0.10）
        # 从来没生效过 —— 典型的"默认值 ≠ 生效值"。现在统一由 settings 出，
        # 不设环境变量时取的就是这里原来的字面量，行为不变。
        place_graph = PlaceGraph(
            merge_distance_m=settings.go2w_place_merge_distance_m,
            relocation_min_displacement_m=settings.go2w_place_relocation_min_m,
        )
        pose_validator = SpatialPoseValidator(
            rotation_wheel_translation_max_m=(
                settings.go2w_lio_rotation_wheel_max_translation_m
            ),
            rotation_pslam_translation_max_m=(
                settings.go2w_lio_rotation_pslam_max_translation_m
            ),
            max_pslam_speed_mps=settings.go2w_lio_max_pose_speed_mps,
            recovery_valid_samples=settings.go2w_lio_recovery_valid_samples,
        )
        semantic_map = SemanticObjectMap(
            merge_distance_m=0.4,
            confirm_min_observations=2,
        )
        import yaml
        mobility_config_path = PROJECT_ROOT / "configs/go2w/semantic_mapping.yaml"
        mobility_config = yaml.safe_load(mobility_config_path.read_text(encoding="utf-8")) if mobility_config_path.is_file() else {}
        semantic_mobility_tracker = ObjectMobilityTracker((mobility_config or {}).get("mobility"))
        from app.spatial.target_candidate_map import TargetCandidateMapper
        target_candidate_mapper = TargetCandidateMapper((mobility_config or {}).get("mobility"))
        entity_graph = SemanticEntityGraph(
            place_graph=place_graph, object_map=semantic_map, frame_id="map"
        )
        route_planner = SemanticRoutePlanner(
            inflation_radius_m=0.25,
            allow_unknown=False,
            max_waypoints=32,
        )
        spatial_memory = SpatialMemory()
        camera_provider = None
        if args.spatial_provider == "nav2":
            from app.spatial.nav2_spatial_provider import Nav2SpatialProvider
            camera_provider = Nav2SpatialProvider(enable_ros=True)
        elif args.spatial_provider == "plain_slam":
            from app.spatial.plain_slam_spatial_provider import PlainSlamSpatialProvider

            camera_provider = PlainSlamSpatialProvider(
                enable_ros=True,
                map_topic="/go2w/slam/map_2d",
                odom_topic="/go2w/slam/odom_base",
                fallback=CameraLocalSpatialProvider(
                    relocate_distance_m=args.forward_step_m or 0.25,
                ),
            )
        elif args.spatial_provider == "rtabmap":
            from app.spatial.rtabmap_spatial_provider import RtabmapSpatialProvider

            camera_provider = RtabmapSpatialProvider(
                enable_ros=True,
                map_topic="/rtabmap/map",
                odom_topic="/rtabmap/odom",
            )
        else:
            camera_provider = CameraLocalSpatialProvider(
                relocate_distance_m=args.forward_step_m or 0.25,
            )
        node.get_logger().info(f"spatial provider = {args.spatial_provider}")
        node._search_spatial_provider = camera_provider
        # 2026-09-09 P2 修复：先等 provider 真正拿到地图，再进入观测循环。
        # 否则首次观测会早于 DDS 首批 map/pose 到达，quality() 降级成 CAMERA_LOCAL，
        # map_xyz 与 map_frame 都不会写进 semantic_object_localized（实测于
        # search_20260909_103523：5 个物体全部 CAMERA_LOCAL、map_xyz=None）。
        if camera_provider is not None and hasattr(camera_provider, "get_map"):
            # 条件必须同时满足「有地图」和「map→odom TF 已到达」：实测 TF 比 map 晚约 1 秒出现，
            # 只等 get_map() 会让循环提前退出，随后 camera_point_to_map() 因 tf_available=false
            # 恒返回 None（P2 第三处断点的真正原因）。
            _deadline = time.time() + 25.0
            _probe = None
            while time.time() < _deadline:
                _spin = getattr(camera_provider, "spin_once", None)
                if _spin is None:
                    break
                _spin()
                time.sleep(0.05)
                metric_map = camera_provider.get_map()
                if metric_map is not None:
                    try:
                        if args.spatial_provider == "nav2":
                            # Nav2 deliberately rejects timestamp=None for
                            # camera projection: accepting it would silently
                            # replace the acquisition-time transform with the
                            # latest pose.  Readiness is instead the real map
                            # pose plus the version identity carried by the
                            # latched keyframe graph.
                            provenance = dict(metric_map.provenance or {})
                            pose_probe = camera_provider.get_pose()
                            if (pose_probe is not None
                                    and provenance.get("map_epoch")
                                    and provenance.get("sensor_profile_id") not in {None, "", "unknown"}):
                                _probe = pose_probe
                        else:
                            _probe = camera_provider.camera_point_to_map(
                                (0.0, 0.0, 2.0), time.time()
                            )
                    except Exception:  # noqa: BLE001
                        _probe = None
                    if _probe is not None:
                        break
            node.get_logger().info(
                "spatial provider ready: map="
                + ("yes" if camera_provider.get_map() is not None else "no")
                + ", map_pose_or_point=" + ("yes" if _probe is not None else "no")
                + ", quality="
                + str(getattr(camera_provider, "quality", lambda: "?")())
            )
        bev_mapper = LightweightDepthBEVMapper()
        frontier_extractor = FrontierExtractor(
            min_component_size=1,
            max_distance_m=math.inf if args.spatial_provider == 'nav2' else 20.,
        )
        psg_provider = RuleSemanticPriorProvider()
        spatial_reasoner = SpatialSearchReasoner(
            LongTermGoalSelector(
                psg_zero_weight=1.0,
                psg_partial_weight=0.7,
                psg_strong_weight=0.2,
                psg_verify_weight=0.0,
            )
        )
        local_executor = LocalGoalExecutor(
            # No range evidence: take a short observation step. Known frontier
            # ranges may produce a longer goal, capped by the selected profile.
            forward_step_m=min(0.30, float(
                getattr(node, "_experiment_forward_step_m", None)
                or args.forward_step_m or 0.25)),
            max_forward_step_m=getattr(node, "_experiment_forward_step_m", None)
            or args.forward_step_m or 0.25,
            max_turn_deg=getattr(node, "_experiment_max_turn_deg", None)
            or args.max_turn_deg,
            turn_only=bool(args.turn_only),
        )
        state["spatial_v2"] = {
            "place_graph": place_graph,
            "semantic_map": semantic_map,
            "entity_graph": entity_graph,
            "route_planner": route_planner,
            "spatial_memory": spatial_memory,
            "local_executor": local_executor,
        }

    # ---- observer -----------------------------------------------------------
    observe_cache_dir = PROJECT_ROOT / "runtime/go2w/semantic_observe_cache"
    observe_cache_dir.mkdir(parents=True, exist_ok=True)

    def _cached_image(image_path: str, frame_id: Any) -> str:
        """Copy the bundle image to a stable path so later LLM steps (which can
        take minutes) never lose it to spool rotation."""
        import shutil

        target = observe_cache_dir / f"bundle_{frame_id}.jpg"
        try:
            shutil.copy2(image_path, target)
        except OSError:
            return image_path
        # keep only the newest 60 cached frames
        cached = sorted(observe_cache_dir.glob("bundle_*.jpg"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
        for old in cached[60:]:
            try:
                old.unlink()
            except OSError:
                pass
        return str(target)

    def _note_quick_vlm_failure(frame_id: Any, elapsed_s: float,
                                failure: PerceptionFailure) -> None:
        """§16 ``quick_vlm_failed``：记下失败，并留给下一次调用发 retry。

        **为什么 retry 判定用 worker 自己的 state，而不是 explorer 手里的
        ``exc.code``。** 重试循环在 explorer（另一个进程）里，它只拿到一个
        ``PerceptionFailure``；而 ``_classify_perception_error`` 把任何带
        "timeout" 的文本都归成 ``QUICK_VLM_TIMEOUT`` —— RGB-D 取帧超时也算 ——
        所以按 code 判"这是不是 quick VLM 的重试"会误判。给 ``PerceptionFailure``
        加 stage 字段能解决，但那要动一个搜索状态存储也在消费的错误契约；worker
        本来就知道自己是哪一段失败的，所以标记留在这里，契约不动。
        """
        state["quick_vlm_last_failure"] = {
            "frame_id": str(frame_id),
            "code": str(failure.code),
            "error": str(failure),
        }
        node._write({
            "event": "quick_vlm_failed",
            "frame_id": str(frame_id),
            "latency_ms": round(float(elapsed_s) * 1000.0, 3),
            "code": str(failure.code),
            "recoverable": bool(failure.recoverable),
            "error": str(failure),
            "host_s": node._host_s(),
        })

    # ---- 计划书 §5.1.4：检测分频的会话级状态 --------------------------------
    # 三次"最近刷新"记的是**检测真的跑了**的时刻（monotonic），不是"这一轮到了"
    # 的时刻 —— 记后者会让跳过的那几轮把时间戳一路推着走，TTL 永远不到期。
    cadence_state: dict[str, Any] = {
        "target_s": None,
        "anchor_s": None,
        "scene_s": None,
        "sector": None,
        "place_id": None,
        "x": None,
        "y": None,
    }
    # 一次检测的产物缓存：**按产生它的那次检测**存（哪些槽 + 源帧 + 时刻），
    # 不是一个大 `objects` 列表。整体覆盖会让"只刷 target"的那一轮把场景物体
    # 丢掉、"只刷 scene"的那一轮把目标候选丢掉（审查 P2）。
    cadence_entries: list[SlotCacheEntry] = []
    cadence_ttl_by_slot = {
        SLOT_TARGET: float(settings.go2w_target_refresh_interval_s),
        SLOT_ANCHOR: float(settings.go2w_anchor_refresh_interval_s),
        SLOT_SCENE: float(settings.go2w_scene_refresh_interval_s),
    }
    detection_scheduler = DetectionScheduler(
        interval_s=float(settings.go2w_target_refresh_interval_s),
        translation_refresh_m=float(
            settings.go2w_detection_translation_refresh_m
        ),
        rotation_refresh_deg=float(
            settings.go2w_detection_rotation_refresh_deg
        ),
        min_interval_s=float(settings.go2w_detection_min_interval_s),
    )

    def _cadence_translation(pose_now: dict[str, Any] | None) -> float:
        if not pose_now or cadence_state["x"] is None:
            return float("inf")  # 没有上一次的位置 ⇒ 当作"视图变了"，宁多刷
        try:
            return math.hypot(
                float(pose_now.get("x", 0.0)) - float(cadence_state["x"]),
                float(pose_now.get("y", 0.0)) - float(cadence_state["y"]),
            )
        except (TypeError, ValueError):
            return float("inf")

    def observe() -> LiveObservation:
        # Shared session cache: this observer both reads and replaces it, and
        # the navigation-time target monitor updates the same enclosing list.
        nonlocal cadence_entries, last_semantic_map_odom_pose
        state["planning_cycle"] = int(state.get("planning_cycle", 0)) + 1
        profiler = LatencyProfiler(
            planning_cycle=int(state.get("planning_cycle", 0)),
            frame_id=str(state.get("frame_id", "")),
        )
        profiler.mark("capture_start")
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.05)
        rgbd_frame = None
        try:
            if rgbd_source is not None:
                rgbd_frame = rgbd_source.get_latest(timeout_seconds=5.0)
                frame_id = rgbd_frame.evidence_id
                stable_image = _cached_image(rgbd_frame.color_ref, frame_id)
                state["image_path"] = stable_image
                state["frame_id"] = frame_id
                profiler.profile.frame_id = str(frame_id)
                # 计划书 §5.1：RGB-D 帧缓存（有界 60 帧），供“语义来自旧帧但
                # 仍要按 source frame 做 depth localization”的场景使用。
                rgbd_frame_cache[str(frame_id)] = rgbd_frame
                if len(rgbd_frame_cache) > 60:
                    for stale_id in list(rgbd_frame_cache)[: len(rgbd_frame_cache) - 60]:
                        rgbd_frame_cache.pop(stale_id, None)
                node._write({"event": "camera_frame_selected", "frame_id": frame_id,
                             "source": "d435", "capture_timestamp": rgbd_frame.timestamp,
                             "host_s": node._host_s()})
            else:
                image_path, frame_id = node._latest_bundle_image(spool_root)
                stable_image = _cached_image(image_path, frame_id)
                state["image_path"] = stable_image
                state["frame_id"] = frame_id
                profiler.profile.frame_id = str(frame_id)
                node._write({"event": "camera_frame_selected", "frame_id": frame_id,
                             "source": "spool", "host_s": node._host_s()})
        except Exception as exc:  # noqa: BLE001 - 感知失败可恢复，交给 explorer retry
            raise PerceptionFailure(
                f"RGB-D capture failed: {type(exc).__name__}: {exc}",
                code=_classify_perception_error(exc),
                recoverable=True,
                detail=str(exc),
            ) from exc
        # 计划书 §14：运动后必须拿到新帧才允许规划下一步。运动刚结束时旧帧
        # 短暂可接受（相机流有延迟），超过 3 秒仍无新帧 -> FRAME_STALE retry。
        motion_end_ts = float(motion_state.get("motion_end_timestamp") or 0.0)
        last_motion_frame = motion_state.get("last_frame_id_before_motion")
        if motion_end_ts > 0.0 and last_motion_frame is not None:
            if str(frame_id) == str(last_motion_frame):
                waited = time.time() - motion_end_ts
                if waited > 3.0:
                    raise PerceptionFailure(
                        "FRAME_STALE: no new camera frame after motion ended "
                        f"(frame={frame_id}, waited={waited:.1f}s)",
                        code="FRAME_STALE",
                        recoverable=True,
                        detail="motion ended but camera frame did not advance",
                    )
                # 短时间内再取一次，期望拿到运动后的新帧。
                for _ in range(2):
                    time.sleep(0.5)
                    rclpy.spin_once(node, timeout_sec=0.1)
                    try:
                        if rgbd_source is not None:
                            rgbd_frame = rgbd_source.get_latest(timeout_seconds=2.0)
                            frame_id = rgbd_frame.evidence_id
                            stable_image = _cached_image(rgbd_frame.color_ref, frame_id)
                            state["image_path"] = stable_image
                            state["frame_id"] = frame_id
                            rgbd_frame_cache[str(frame_id)] = rgbd_frame
                        else:
                            image_path, frame_id = node._latest_bundle_image(
                                spool_root, retries=0
                            )
                            stable_image = _cached_image(image_path, frame_id)
                            state["image_path"] = stable_image
                            state["frame_id"] = frame_id
                    except Exception:  # noqa: BLE001 - 保留旧帧信息，由上层判断
                        pass
                    if str(frame_id) != str(last_motion_frame):
                        break
        profiler.record("capture_ms", "capture_start")
        x, y, yaw = node._odom_snapshot()
        pose = {"x": x, "y": y, "yaw_rad": yaw,
                "yaw_deg": math.degrees(yaw)}
        capture_ts = (
            float(rgbd_frame.timestamp)
            if rgbd_frame is not None
            else time.time()
        )
        capture_timestamp_quality = (
            str(rgbd_frame.timestamp_quality)
            if rgbd_frame is not None
            else "receive_time"
        )
        # 计划书 §6.2：navigation heading sector 是运动状态，永远由当前
        # capture pose 计算，与语义是否 stale 无关。
        navigation_sector = _navigation_sector(
            yaw_rad=yaw,
            sectors=max(1, int(policy.candidates.heading_sectors)),
        )
        state["navigation_heading_sector"] = navigation_sector
        node._write({
            "event": "navigation_sector_observed",
            "frame_id": str(frame_id),
            "navigation_heading_sector": navigation_sector,
            "yaw_deg": round(math.degrees(yaw), 3),
            "image_timestamp": capture_ts,
            "timestamp_quality": capture_timestamp_quality,
            "host_s": node._host_s(),
        })
        # 后台 Full Semantic single-flight（计划书 §3.5）：首帧也提交后台，
        # 当前轮不再同步等待 75 秒；失败/超时绝不进入 latest_success。
        # Cloud VLM semantic enumeration remains asynchronous.  Local
        # structured backends use the scheduled full-frame result below; they
        # must not launch a second detector pass for this same frame.
        if not is_local_structured_backend(getattr(node, "_detector", "llm")):
            semantic_manager.submit_if_needed(
                image_path=stable_image,
                frame_id=str(frame_id),
                capture_timestamp=capture_ts,
                timestamp_quality=capture_timestamp_quality,
                robot_pose=pose,
                target_profile=controller.target_profile,
                scene_signature=compute_scene_signature(stable_image),
            )
        # ---- 计划书 §5.1.4：这一轮跑哪些词槽 --------------------------------
        # 决策刻意放在 `quick_start` 之前：它决定的是"quick 段该怎么跑"，
        # 属于 quick 段的**前置**而不是段内一步（也顺带让帧号测试切出来的
        # quick 段只依赖 `slots`/`reuse_detection`/`cadence_entries` 三个名字）。
        # 关着开关时 `slots = None` ⇒ 与改造前逐字一致（一次全量检测）。
        detection_event = state.pop("detection_event", None)
        trigger = detection_scheduler.evaluate(
            now=time.monotonic(),
            pose=(x, y, yaw),
            event=detection_event,
            place_id=state.get("place_id"),
        )
        if trigger.debounced and detection_event:
            state["detection_event"] = detection_event
        cadence = DetectionCadence(
            refresh_target=trigger.should_detect,
            refresh_anchor=trigger.should_detect,
            refresh_scene=trigger.should_detect,
            reasons=trigger.reasons,
        )
        slots = cadence.slots if settings.go2w_detection_cadence_enabled else None
        if settings.go2w_detection_cadence_enabled and cadence.forced_scene:
            # §5.2 要求这条**可见**：场景词被硬顶逼出来时要记下实际等了多久，
            # 否则"场景刷新是不是正常"只能靠推断。
            node._write({
                "event": "scene_refresh_max_wait_s",
                "frame_id": str(frame_id),
                "waited_s": round(float(cadence.scene_wait_s or 0.0), 3),
                "scene_max_stale_s": float(settings.go2w_scene_max_stale_s),
                "host_s": node._host_s(),
            })
        # 没有任何槽到期时，用**已经攒下的**结果组装这一轮观测（一次前向都不发）。
        # 判据是"组装得出来东西"，不是"缓存里有东西" —— 后者会让冷启动那几轮
        # 交出一个空观测。
        stale_assembly = assemble_slot_objects(
            cadence_entries,
            now=time.monotonic(),
            ttl_by_slot=cadence_ttl_by_slot,
        ) if settings.go2w_detection_cadence_enabled else None
        reuse_detection = bool(
            settings.go2w_detection_cadence_enabled
            and not slots
            and stale_assembly is not None
            and bool(cadence_entries)
        )
        if settings.go2w_detection_cadence_enabled and not slots and not reuse_detection:
            # 冷启动理论上到不了这里（`last_*_s is None` 会强制全刷）。真到了
            # 就是判据漏了一条：退回全量跑，宁可多花一次前向也不能交空观测。
            slots = None
        node._write({
            "event": "detection_cadence_decided",
            "frame_id": str(frame_id),
            "cadence_enabled": bool(settings.go2w_detection_cadence_enabled),
            "slots": None if slots is None else sorted(slots),
            "reused": reuse_detection,
            "forced_scene": bool(cadence.forced_scene),
            "reasons": list(cadence.reasons),
            "trigger_reason": trigger.reason,
            "elapsed_s": trigger.elapsed_s,
            "translation_m": round(trigger.translation_m, 4),
            "rotation_deg": round(trigger.rotation_deg, 3),
            "debounced": trigger.debounced,
            "detector_backend": getattr(node, "_detector", "llm"),
            "host_s": node._host_s(),
        })
        profiler.mark("quick_start")
        # §16 quick_vlm_retry：上一次 quick VLM 失败过 ⇒ 本次调用是在补它。
        # 带上失败那一次的 frame_id/code，光有一个 retry 记号查不出补的是哪一帧。
        previous_quick_failure = state.get("quick_vlm_last_failure")
        if previous_quick_failure:
            node._write({
                "event": "quick_vlm_retry",
                "frame_id": str(frame_id),
                "previous_frame_id": str(previous_quick_failure.get("frame_id", "")),
                "previous_code": str(previous_quick_failure.get("code", "")),
                "previous_error": str(previous_quick_failure.get("error", "")),
                "host_s": node._host_s(),
            })
        if reuse_detection:
            # 全槽都没到期：这一轮**不**发前向，复用已经攒下的检测产物。
            node._write({
                "event": "quick_vlm_cache_reused",
                "frame_id": str(frame_id),
                "objects": len(stale_assembly.objects),
                "host_s": node._host_s(),
            })
        else:
            node._write({"event": "quick_vlm_started", "frame_id": str(frame_id),
                         "host_s": node._host_s()})
        quick_start_perf = time.perf_counter()
        try:
            # 帧号取自**本次观测**（`observe` 手里的 `frame_id`），不取"最近一帧"
            # 那个共享可变字段：异步下它未必还是这一帧（独立复审 F1）。
            if reuse_detection:
                assembly = stale_assembly
            else:
                fresh = node._detect(stable_image, prompt, env,
                                     frame_id=frame_id, slots=slots)
                if settings.go2w_detection_cadence_enabled:
                    # 只把**这次跑过的槽**记进去，别的槽上次的结果原样留着 ——
                    # 组装才是本轮观测，`fresh` 只是它的一个子集（审查 P2）。
                    cadence_entries = put_slot_entry(
                        cadence_entries,
                        SlotCacheEntry(
                            slots=frozenset(slots) if slots else frozenset(ALL_SLOTS),
                            objects=deepcopy(fresh),
                            frame_id=str(frame_id),
                            at=time.monotonic(),
                        ),
                    )
                    assembly = assemble_slot_objects(
                        cadence_entries,
                        now=time.monotonic(),
                        ttl_by_slot=cadence_ttl_by_slot,
                    )
                else:
                    # 分频关着时**不进**缓存那条路：行为与加这个功能之前逐字一致。
                    assembly = SlotAssembly(
                        objects=list(fresh),
                        entries=(),
                        frame_id=str(frame_id or ""),
                    )
            objects = deepcopy(assembly.objects)
            if settings.go2w_detection_cadence_enabled:
                # 组装结果**逐槽**可见：哪个槽的物体是几秒前的、来自哪一帧。
                # §5.2 要的是"过期结果不许冒充当前帧事实"，光说"复用了缓存"证不了。
                node._write({
                    "event": "detection_cadence_assembled",
                    "frame_id": str(frame_id),
                    "objects": len(objects),
                    "mixed_frames": bool(assembly.mixed_frames),
                    "views_source_frame_id": str(assembly.frame_id),
                    "sources": assembly.provenance(
                        now=time.monotonic(), ttl_by_slot=cadence_ttl_by_slot
                    ),
                    "host_s": node._host_s(),
                })
        except PerceptionFailure as exc:
            _note_quick_vlm_failure(frame_id,
                                    time.perf_counter() - quick_start_perf, exc)
            raise
        except Exception as exc:  # noqa: BLE001 - 结构化错误交给 explorer retry
            failure = PerceptionFailure(
                f"Quick VLM failed: {type(exc).__name__}: {exc}",
                code=_classify_perception_error(exc),
                recoverable=True,
                detail=str(exc),
                last_success_age_s=_last_success_age_s(),
            )
            _note_quick_vlm_failure(frame_id,
                                    time.perf_counter() - quick_start_perf, failure)
            raise failure from exc
        quick_latency_ms = (time.perf_counter() - quick_start_perf) * 1000.0
        profiler.record("quick_vlm_ms", "quick_start")
        # 时间戳只在**真的跑了**之后推进（复用那一轮不推），否则跳过的轮次会把
        # 时间戳一路推着走、TTL 永远不到期 —— 分频就变成"永远只刷一次"。
        if not reuse_detection:
            refreshed_at = time.monotonic()
            detection_scheduler.mark_detected(
                now=refreshed_at,
                pose=(x, y, yaw),
                place_id=state.get("place_id"),
            )
            if slots is None or "target" in slots:
                cadence_state["target_s"] = refreshed_at
            if slots is None or "anchor" in slots:
                cadence_state["anchor_s"] = refreshed_at
            if slots is None or "scene" in slots:
                cadence_state["scene_s"] = refreshed_at
            cadence_state["sector"] = navigation_sector
            cadence_state["place_id"] = state.get("place_id")
            cadence_state["x"] = pose.get("x")
            cadence_state["y"] = pose.get("y")
        if is_local_structured_backend(getattr(node, "_detector", "llm")):
            local_cache = state.setdefault("local_detection_by_frame", {})
            local_cache[str(frame_id)] = {
                "objects": deepcopy(objects),
                "source_frame_id": str(assembly.frame_id or frame_id),
                "capture_timestamp": capture_ts,
                "robot_pose": dict(pose),
            }
            while len(local_cache) > 8:
                local_cache.pop(next(iter(local_cache)))
        # 只在真打了云端时才计入云端桶。`grounded_sam` 下这一段的耗时是本地
        # 检测（`quick_vlm_ms` 事件名照旧，因为下游读取方按名字取），但**不是**
        # 一次云端调用 —— 记进去会让云端调用量虚高（独立审查 R1）。
        count_cloud_api_call(profiler, node, "quick_api_calls")
        # 成功才清。留着它，下一次调用会平白发一条 quick_vlm_retry。
        state["quick_vlm_last_failure"] = None
        node._write({"event": "quick_vlm_completed", "frame_id": str(frame_id),
                     "latency_ms": round(float(quick_latency_ms), 3),
                     "objects": len(objects),
                     "host_s": node._host_s()})
        # Quick VLM 新契约：objects 只含真正目标候选；target_decision 是目标 gate。
        quick_payload = None
        if getattr(node, "_detector", "llm") == "llm":
            quick_payload = getattr(node, "_last_llm_detection_payload", None)
        # §5.1.2：一次检测的产物按职责拆成三份视图。默认空表，两个分支各自填。
        target_candidates: list[dict[str, Any]] = []
        relation_anchors: list[dict[str, Any]] = []
        # 三份视图的来源帧。分频开着时**不**无条件写当前帧号：本轮观测是若干次
        # 检测拼起来的，若拼进来的物体来自不止一帧，这里如实给空串（"不知道"），
        # 而不是把当前帧号按在旧框上 —— 旧框的 `entity_source_frame_id` 才是
        # 它真正的来路，判同一实体要用那个（审查 P2 / P1-c）。
        views_source_frame_id = (
            str(assembly.frame_id or "")
            if settings.go2w_detection_cadence_enabled
            else str(frame_id or "")
        )
        if quick_payload is not None:
            decision = quick_payload.get("target_decision") or {}
            target_objects = list(
                quick_payload.get("target_objects")
                or quick_payload.get("objects")
                or []
            )
            detections = []
            for item in target_objects:
                raw_bbox = item.get("bbox_2d") or item.get("bbox")
                if not isinstance(raw_bbox, (list, tuple)) or len(raw_bbox) != 4:
                    # POSSIBLE candidates are still carried by target_state,
                    # but an invalid box must never be used as a full-frame
                    # verification crop.
                    continue
                bbox = [float(v) for v in raw_bbox]
                detections.append({
                    "label": str(item.get("label", "object")),
                    "score": float(item.get("score", 0.0)),
                    "bbox_2d": bbox,
                })
            target_present = quick_target_present(quick_payload, args.target_score_min)
            target_state = quick_target_state(quick_payload, args.target_score_min)
            # quick 契约上 `objects`/`target_objects` **本来就只含目标候选**
            # （普通物体在 scene_objects_light 里），所以这里直接就是候选视图，
            # 不需要再分类。
            target_candidates = list(detections)
        else:
            profile_now = controller.target_profile
            if settings.go2w_result_role_separation and profile_now is not None:
                # 旧行为 `bool(detections)` 是**错的**：`objects` 里除了目标词
                # 还有 RAM 场景词（`_detect_grounded_sam` 把场景词拼进了同一次
                # 请求），所以画面里随便有件东西，目标就算"存在"。改成只在
                # **目标词候选**里判定。
                #
                # 分类用的是 `objects` 而不是上面重建的 `detections`：后者只留了
                # label/score/bbox，把判断职责要用的 `source_prompt_term` 丢了。
                # `source_frame_id` 一起传下去：三份视图的 `entity_id` 只有在
                # **同一帧内**才表示同一个实体，装配点上不写清楚，下游就只剩
                # "bbox 哈希"可猜了（审查 P1-c）。
                partition = classify_roles(
                    objects, profile_now, source_frame_id=views_source_frame_id
                )
                # `detections` 用**这一批带 ID 的 dict**，不再从 `objects` 重建。
                # 旧的重建只留 label/score/bbox，把 `entity_id` 和
                # `source_prompt_term` 一起丢了 —— 于是 `detections` 与两份候选
                # 视图虽然源自同一次检测，却对不上号（这正是审查 P1-c 说的
                # "只在纯函数单测里成立"）。同帧同源的三份视图必须共用同一批 dict。
                detections = partition.detections
                target_candidates = partition.target_candidates
                relation_anchors = partition.relation_anchors
                target_state = derive_target_state(
                    objects, profile_now, args.target_score_min
                )
                target_present = target_state == TARGET_PRESENT
            else:
                # 旧路径（回退用）：把 `objects` 削成 label/score/bbox 三件套。
                # 这一削就把 `entity_id` 与 `source_prompt_term` 丢了 —— 所以
                # 只在职责分离**关着**时才走这里。
                #
                # 但**分频开着**时不能照单全收（第三次补充复审·必修）：装配层
                # 为了保住 `scene` 会把旧的全量条目一并选进来，它里面那个过期的
                # 目标框也就躺在 `objects` 里；职责分离关着**没有任何人看**
                # `detection_fresh_slots` 那枚标记，于是旧帧的高分框从这条默认
                # 路径原样回到选框池 —— `select_candidate` 取最高分，锁框与复核
                # 都指着它，却报"目标确认为真"。单独开任务 4、任务 2 保持默认，
                # 走的正是这条路，所以必修 1 的修复此前只在开任务 2 时才生效。
                #
                # 这里先过一遍**同一套新鲜度过滤**再削三件套。用的是
                # `classify_roles` 那把尺子、不另写一套（两套判据必然会漂）。
                # 分频关着时物体身上没有那枚标记 ⇒ `classify_roles` 一条都不丢，
                # 行为与加这个功能之前逐字一致。
                pool = objects
                if settings.go2w_detection_cadence_enabled:
                    pool = classify_roles(
                        objects, profile_now, source_frame_id=views_source_frame_id
                    ).detections
                detections = []
                for item in pool:
                    bbox = [float(v) for v in item.get("bbox_2d", [0.0, 0.0, 1.0, 1.0])]
                    detections.append({
                        "label": str(item.get("label", "object")),
                        "score": float(item.get("score", 0.0)),
                        "bbox_2d": bbox,
                    })
                target_present = bool(detections)
                target_state = TARGET_PRESENT if target_present else TARGET_ABSENT
        # 计划书 §5.1.3：派给 `_feed_detection` 的那个框（相机画面上锁的那一个）
        # 也只在**目标候选**里挑。旧代码在全量 detections 上取最高分，于是锁框
        # 常常锁在场景物体上 —— 画面看着"锁住了"，锁的不是要找的东西。
        best_feed, _ = select_candidate(
            target_candidates,
            detections,
            scope=settings.go2w_verify_candidate_scope,
            candidates_authoritative=settings.go2w_result_role_separation,
        )
        node._write({
            "event": "target_candidate_selection",
            "frame_id": str(frame_id),
            "candidates_authoritative": settings.go2w_result_role_separation,
            "target_candidate_count": len(target_candidates),
            "detection_count": len(detections),
            "selected": ({key: best_feed.get(key) for key in (
                "entity_id", "entity_source_frame_id", "label", "source_prompt_term", "score", "bbox_2d"
            )} if best_feed is not None else None),
            "host_s": node._host_s(),
        })
        if best_feed is not None:
            node._feed_detection(
                str(best_feed.get("label", "object")),
                float(best_feed.get("score", 0.0) or 0.0),
                best_feed.get("bbox_2d") or best_feed.get("bbox"),
            )
        # ---- 计划书 §4.1：Quick 快路径场景物体（轻量 fallback）-------------
        # Quick 一次调用既给目标判断，又给少量显著普通物体；Full Semantic
        # 成功后再用完整 object list/relations 增强。绝不把普通物体混进
        # target_objects。
        quick_scene_objects: list[dict[str, Any]] = []
        if quick_payload is not None:
            quick_scene_objects = list(
                quick_payload.get("scene_objects_light")
                or quick_payload.get("scene_objects")
                or []
            )
        elif is_local_structured_backend(getattr(node, '_detector', 'llm')):
            quick_scene_objects = local_detection_scene_objects(
                objects, controller.target_profile, source_frame_id=views_source_frame_id,
                current_frame_id=str(frame_id))
        quick_semantic_payload = {
            "scene_objects": quick_scene_objects,
            "scene_relations": [],
            "scene_summary_zh": str(
                (quick_payload or {}).get("scene_summary_zh") or ""
            ),
            "image_path": stable_image,
            "frame_id": str(frame_id),
            "source": (f"{getattr(node, '_detector', 'local')}_scene_detections" if quick_payload is None
                       else "siliconflow_quick_scene_light"),
            "semantic_status": SEMANTIC_STATUS_FRESH_QUICK,
            "semantic_quality": "quick_scene_light",
            "semantic_source_frame_id": str(frame_id),
            "semantic_capture_timestamp": capture_ts,
            "timestamp_quality": capture_timestamp_quality,
            "semantic_completed_timestamp": capture_ts,
            "semantic_age_ms": 0.0,
            "semantic_source_pose": pose,
            "target_state": target_state,
            "target_decision": (quick_payload or {}).get("target_decision", {}),
        }
        profiler.mark("semantic_start")
        # 先收割后台已完成结果；未完成时继续用 latest completed。
        for completed_semantic in semantic_manager.poll_completed():
            node._write({
                "event": "semantic_background_applied",
                "frame_id": str(getattr(completed_semantic, "frame_id", "")),
                "result_age_ms": round(
                    max(0.0, (time.time() - float(
                        getattr(completed_semantic, "semantic_completed_timestamp", 0.0)
                        or completed_semantic.timestamp_sec)) * 1000.0),
                    3,
                ),
                "objects": len(completed_semantic.objects or []),
                "relations": len(completed_semantic.relations or []),
                "status": getattr(completed_semantic, "semantic_status", "fresh_full"),
                "host_s": node._host_s(),
            })
        latest_semantic = semantic_manager.get_latest_completed()
        # 计划书 §4.2 语义优先级：
        #   当前帧 Full Semantic 成功 > 当前帧 scene_objects_light > 当前帧无语义
        # 禁止：111 秒前的 Full Semantic 冒充“当前场景确认无物体”。
        semantic: SemanticObservation | None = None
        if latest_semantic is not None and (
            latest_semantic.semantic_source_frame_id == str(frame_id)
            or latest_semantic.frame_id == str(frame_id)
        ):
            semantic = latest_semantic
            profiler.set_counter("semantic_cache_hit", True)
        elif latest_semantic is not None:
            # 旧帧 Full Semantic 在很短的新鲜度窗口内可用，这里**不要求 quick 没看见物体**。
            # Full 是异步提交的，结果几乎总是晚一拍到达（真机实测 4.8~23.0 s），而
            # observation 在 quick 返回后立刻装配——若仍要求 quick 为空，本分支永不进入
            # （真机 6 个 run 里 full_older_frame 命中 0 次），Full 独有的 scene_relations
            # 会被整批丢掉，物体拓扑只剩孤点。
            # 目标存在性判定不受影响：target_present/target_state 由 quick 路径独立给出，
            # 本分支不参与，绝不拿旧 Full 冒充当前帧“确认无物体”。
            # 明确标记 age/status，绝不假装是当前帧事实。
            age_s = max(
                0.0,
                time.time() - float(
                    latest_semantic.semantic_completed_timestamp
                    or latest_semantic.timestamp_sec
                ),
            )
            soft_stale_s = max(
                0.0, float(settings.vlm_runtime_planner_semantic_soft_stale_seconds)
            )
            if age_s <= soft_stale_s:
                semantic = latest_semantic
                semantic.semantic_age_ms = age_s * 1000.0
                if semantic.semantic_status in {"fresh_full", "fresh_quick_scene"}:
                    semantic.semantic_status = "stale"
                    semantic.semantic_quality = "full_older_frame"
                profiler.set_counter("semantic_cache_hit", True)
            else:
                node._write({"event": "semantic_stale", "frame_id": str(frame_id),
                             "source_frame_id": str(latest_semantic.frame_id),
                             "age_ms": round(age_s * 1000.0, 3),
                             "host_s": node._host_s()})
        if semantic is None:
            # 当前帧 Quick 快路径（scene_objects_light）；没有普通物体时也
            # 构造“当前帧无语义”状态，而不是拿旧语义顶替。
            quick_semantic = _from_payload(
                quick_semantic_payload,
                robot_pose=pose,
                sector=navigation_sector,
                now=capture_ts,
            )
            if not quick_scene_objects:
                quick_semantic.semantic_status = SEMANTIC_STATUS_PENDING
                quick_semantic.semantic_quality = "unavailable"
                node._write({
                    "event": "semantic_unavailable",
                    "frame_id": str(frame_id),
                    "detail": "Full Semantic pending and quick scene empty",
                    "host_s": node._host_s(),
                })
            semantic = quick_semantic
            # 同上：本地语义链不产生云端调用。`semantic_cache_hit` 记的是
            # 「这帧没命中后台缓存」这件事本身，与后端无关，照旧。
            count_cloud_api_call(profiler, node, "semantic_api_calls")
            profiler.set_counter("semantic_cache_hit", False)
        if semantic.semantic_age_ms is None:
            semantic.semantic_age_ms = max(
                0.0,
                (time.time() - float(semantic.semantic_capture_timestamp
                                     or semantic.timestamp_sec)) * 1000.0,
            )
        semantic_frame_id = str(
            getattr(semantic, "semantic_source_frame_id", None)
            or semantic.frame_id
        )
        from app.perception.depth_object_localizer import resolve_depth_frame

        timestamp_frame = resolve_depth_frame(semantic_frame_id, rgbd_frame, rgbd_frame_cache)
        image_timestamp = float(
            timestamp_frame.timestamp
            if timestamp_frame is not None
            else semantic.semantic_capture_timestamp or semantic.timestamp_sec
        )
        image_timestamp_quality = str(
            timestamp_frame.timestamp_quality
            if timestamp_frame is not None
            else (
                capture_timestamp_quality
                if semantic_frame_id == str(frame_id)
                else semantic.timestamp_quality or "semantic_capture_timestamp"
            )
        )
        state["semantic"] = semantic
        profiler.set_counter("semantic_source_frame_id", str(getattr(semantic, "frame_id", "")))
        profiler.set_counter("semantic_result_age_ms", max(0.0, float(semantic.semantic_age_ms or 0.0)))
        profiler.set_counter("semantic_status", str(semantic.semantic_status))
        # 计划书 §6.4：所有“机器人看过哪些方向”的记录都使用 navigation sector。
        observed_sectors.add(navigation_sector)
        node._write({
            "event": "semantic_status",
            "frame_id": str(frame_id),
            "semantic_status": semantic.semantic_status,
            "semantic_quality": semantic.semantic_quality,
            "semantic_source_frame_id": semantic.semantic_source_frame_id,
            "semantic_age_ms": round(float(semantic.semantic_age_ms or 0.0), 3),
            "semantic_object_count": len(semantic.objects or []),
            "semantic_error_code": semantic.semantic_error_code,
            "image_timestamp": image_timestamp,
            "timestamp_quality": image_timestamp_quality,
            "host_s": node._host_s(),
        })

        spatial_quality = "RGB_ONLY"
        camera_xyz = None
        depth_ref = None
        intrinsics = None
        depth_scale = None
        localized_frame_id: str | None = None
        localized: list[Any] = []
        if depth_localizer is not None:
            # 计划书 §5.2 / 不变量：old semantic bbox 严禁配 current depth。
            # 只能使用语义 source frame 对应的深度帧（当前帧或 RGB-D 缓存）。
            depth_frame = resolve_depth_frame(
                semantic_frame_id, rgbd_frame, rgbd_frame_cache
            )
            if depth_frame is None and rgbd_frame is not None:
                node._write({
                    "event": "semantic_depth_frame_mismatch",
                    "semantic_frame_id": semantic_frame_id,
                    "current_frame_id": rgbd_frame.evidence_id,
                    "sensor_frame_id": str(rgbd_frame.frame_id),
                    "action": "SEMANTIC_2D_ONLY",
                    "host_s": node._host_s(),
                })
                spatial_quality = "SEMANTIC_2D_ONLY"
            if depth_frame is not None:
                image_timestamp = float(depth_frame.timestamp)
                image_timestamp_quality = str(depth_frame.timestamp_quality)
                localized_frame_id = depth_frame.evidence_id
                localized = depth_localizer.localize(semantic.objects, depth_frame)
                # Enrich object dicts with spatial fields and remember the best
                # camera-local 3D position for the LiveObservation payload.
                enriched_objects: list[dict[str, Any]] = []
                for index, obj in enumerate(semantic.objects):
                    item = dict(obj)
                    if index < len(localized):
                        spatial = localized[index]
                        item["depth_m"] = spatial.depth_m
                        item["bearing_deg"] = spatial.bearing_deg
                        item["camera_xyz"] = list(spatial.camera_xyz) if spatial.camera_xyz else None
                        item["spatial_quality"] = spatial.spatial_quality
                    enriched_objects.append(item)
                semantic.objects = enriched_objects
                # Also publish 3D fields into the observed SceneGraph node
                # attributes so PSG can bind hypotheses to real anchors.
                if semantic.scene_graph is not None:
                    for sg_node in semantic.scene_graph.nodes:
                        label = str(getattr(sg_node, "label_zh", None) or getattr(sg_node, "label", None) or "")
                        for obj in enriched_objects:
                            if str(obj.get("label_zh") or obj.get("label") or "") == label:
                                attrs = dict(getattr(sg_node, "attributes", {}) or {})
                                for key in ("depth_m", "bearing_deg", "camera_xyz", "spatial_quality"):
                                    if obj.get(key) is not None:
                                        attrs[key] = obj[key]
                                sg_node.attributes = attrs
                                break
                localized_with_xyz = [item for item in localized if item.camera_xyz is not None]
                if localized_with_xyz:
                    best_spatial = max(localized_with_xyz, key=lambda item: item.confidence)
                    camera_xyz = list(best_spatial.camera_xyz)
                    spatial_quality = best_spatial.spatial_quality
                depth_ref = depth_frame.depth_ref
                intrinsics = {
                    "fx": depth_frame.fx, "fy": depth_frame.fy,
                    "cx": depth_frame.cx, "cy": depth_frame.cy,
                }
                depth_scale = depth_frame.depth_unit_m

        # Quick and Full are both allowed to omit basic geometric relations.
        # Derive conservative same-frame edges after RGB-D enrichment and
        # before persistent association; endpoint ids remain frame-local here.
        # §15：三个关系阈值走 settings。比的是 bbox_2d 的中心差，而 bbox_2d
        # 是归一化到 0~1 的图像坐标（app/prompts.py:47,103），所以 _MIN_NORM
        # 里的 NORM 名副其实，单位不是米。
        geometric_relations = extract_geometric_relations(
            semantic.objects,
            min_horizontal_separation=settings.go2w_relation_left_right_min_norm,
            min_depth_separation_m=settings.go2w_relation_depth_order_min_m,
            near_distance_m=settings.go2w_relation_near_max_m,
        )
        if geometric_relations:
            existing_relation_keys = {
                (
                    str(item.get("subject_id") or item.get("source_id") or ""),
                    str(item.get("object_id") or item.get("target_id") or ""),
                    str(item.get("relation") or item.get("relation_type") or "").lower(),
                )
                for item in semantic.relations
                if isinstance(item, dict)
            }
            semantic.relations = list(semantic.relations) + [
                relation for relation in geometric_relations
                if (
                    relation["subject_id"], relation["object_id"], relation["relation"]
                ) not in existing_relation_keys
            ]
            node._write({
                "event": "geometric_relations_extracted",
                "frame_id": str(frame_id),
                "relation_count": len(geometric_relations),
                "host_s": node._host_s(),
            })

        # The target view is deliberately absent from ordinary scene objects.
        # Measure each candidate against its own capture, independently of a
        # potentially older asynchronous scene result. Never borrow the scene
        # aggregate's depth or best-object coordinates for the selected target.
        from app.perception.target_spatial import localize_target_candidates, project_target_candidates, bind_target_measurements
        measured_targets = localize_target_candidates(
            target_candidates, source_frame_id=views_source_frame_id,
            current_frame=rgbd_frame, cache=rgbd_frame_cache,
            localizer=depth_localizer,
        )
        target_candidates = bind_target_measurements(target_candidates, measured_targets,
                                                      source_frame_id=views_source_frame_id)
        observation = semantic_observation_to_live(
            semantic,
            bundle_id=f"bundle_{frame_id}",
            detections=detections,
            target_present=target_present,
            target_state=target_state,
            target_candidates=(target_candidates if settings.go2w_result_role_separation else None),
            relation_anchors=relation_anchors,
            # 三份视图的来源帧（本次前景检测）；`scene_objects` 的来源帧由
            # semantic_observation_to_live 从 semantic 自己取 —— 两者不同就说明
            # 它们是两次检测的产物，下游**不能**按 ID 直接对齐（审查 P1-c）。
            views_source_frame_id=views_source_frame_id,
            pose=pose,
            image_ref=stable_image,
            depth_ref=depth_ref,
            rgbd_frame_id=localized_frame_id,
            intrinsics=intrinsics,
            depth_scale=depth_scale,
            spatial_quality=spatial_quality,
            camera_xyz=camera_xyz,
            navigation_heading_sector=navigation_sector,
            timestamp=image_timestamp,
            sensor_health={
                "camera": True,
                "lidar": node._lidar_fresh is True,
                "sport_mode_ok": (
                    node._sport is not None
                    and int(node._sport.mode) == 1
                    and int(node._sport.error_code) == 0
                ),
            },
        )
        observation.provenance.update({
            "image_timestamp": image_timestamp,
            "timestamp_quality": image_timestamp_quality,
        })
        if target_contract_enabled:
            observation.provenance.update(target_spatial_contract='capture_v1',
                                          current_capture_id=str(frame_id))
        if semantic_map is not None:
            # This map is shared with explorer.semantic_graph. The observer is
            # the sole writer; explorer reuses this exact association result.
            observation.object_update_result = SemanticMapUpdateResult()

        # ---- SemanticNavigation V2 spatial state update -------------------------------
        if place_graph is not None and pose is not None:
            from app.spatial.camera_local_spatial_provider import CameraLocalSpatialProvider
            from app.spatial.models import SpatialPose

            # 计划书 §9.2 / 不变量 3：空间地图位姿必须来自 SpatialProvider 自己的
            # 世界坐标系（plain_slam -> pslam_odom）。wheel odom 只用于相对运动/
            # 当前 yaw/运动验证，绝不冒充 pslam pose。
            spatial_pose = None
            raw_spatial_pose = None
            spatial_pose_diagnostics = None
            provider_quality = "CAMERA_LOCAL"
            if camera_provider is not None:
                spin = getattr(camera_provider, "spin_once", None)
                if spin is not None:
                    # Preserve the bounded input wait while both legacy
                    # CAMERA_LOCAL and Nav2 NO_GLOBAL_SPATIAL_POSE recover.
                    # A map alone does not prove a valid current global pose.
                    from app.spatial.provider_readiness import wait_for_spatial_inputs
                    wait_for_spatial_inputs(camera_provider)
                try:
                    spatial_pose = camera_provider.get_pose()
                except Exception:  # noqa: BLE001 - degraded pose never crashes
                    spatial_pose = None
                if spatial_pose is None:
                    diagnostic = getattr(camera_provider, 'pose_diagnostics', None)
                    if callable(diagnostic):
                        spatial_pose_diagnostics = diagnostic()
                provider_quality = getattr(
                    camera_provider, "quality", lambda: "CAMERA_LOCAL"
                )()
                # 2026-09-09 P2 诊断：确认 worker 内 provider 的 _pose 是否被赋值
                # （quality() 返回 CAMERA_LOCAL 只可能是 _pose is None）。
                try:
                    _dbg_pose = camera_provider.get_pose()
                    node.get_logger().info(
                        "spatial probe: quality=" + str(provider_quality)
                        + " internal_pose="
                        + ("yes" if getattr(camera_provider, "_pose", None) is not None else "no")
                        + " get_pose=" + ("yes" if _dbg_pose is not None else "no")
                        + " pose_frame=" + str(getattr(_dbg_pose, "frame_id", None))
                        + " map=" + ("yes" if camera_provider.get_map() is not None else "no")
                    )
                except Exception:  # noqa: BLE001
                    pass
                if isinstance(camera_provider, CameraLocalSpatialProvider):
                    # 仅 camera-local 提供者接受 wheel odom 相对位姿（无全局地图）。
                    # ⚠ 必须每轮都注入，不能加"只在 _pose is None 时注入一次"的条件：
                    # 这个 provider 没有别的位姿源（_pose 只由 set_pose 写、get_pose
                    # 读，见 camera_local_spatial_provider.py:38-42），一旦注入过一次，
                    # get_pose() 就恒返回注入那一刻的快照 —— 位姿冻结会让
                    # pslam_delta_xy 恒 0，使 LIO_DRIFT_DURING_ROTATION 判据永不触发。
                    # 2026-09-16 §22 实测：同段旋转 73°，外部 /go2w/slam/odom_base 漂
                    # 0.204 m，worker 内 raw_pose 16 位小数逐帧完全相同、delta 报 0.0。
                    spatial_pose = SpatialPose(
                        x=float(pose["x"]),
                        y=float(pose["y"]),
                        yaw=float(pose["yaw_rad"]),
                        frame_id="odom",
                        quality="relative",
                        source="go2w_wheel_odom",
                    )
                    camera_provider.set_pose(spatial_pose)
                elif spatial_pose is None:
                    node._write({
                        "event": "spatial_frame_mismatch",
                        "detail": "NO_GLOBAL_SPATIAL_POSE: wheel odom not injected "
                                  "into metric provider",
                        "provider": type(camera_provider).__name__,
                        "pose_diagnostics": spatial_pose_diagnostics,
                        "host_s": node._host_s(),
                    })
            raw_spatial_pose = spatial_pose
            motion_evidence = motion_state.get("pending_motion_evidence")
            # 不传 timestamp：validator 算的是"两次校验之间位姿变了多少"，
            # 时间基准必须是校验发生的时刻（其默认 time.monotonic()）。
            # 曾传 observation.timestamp，而那是**语义帧**的时间戳
            # （semantic_observer.py:319 的 semantic.timestamp_sec）—— 语义结果
            # 常常是缓存旧帧（semantic_status=stale），于是 elapsed 变成负值，
            # 位姿被误判成 POSE_TIMESTAMP_NONMONOTONIC 而拒掉。
            # 2026-09-16 §22 实测：4 次校验中 1 次这样被拒。
            validation = (
                pose_validator.validate(spatial_pose, motion_evidence)
                if pose_validator is not None else None
            )
            if validation is not None:
                if validation.accepted:
                    spatial_pose = validation.accepted_pose
                else:
                    # Preserve only the last good pose for diagnostics; do not
                    # write the rejected raw pose into metric map/place data.
                    spatial_pose = None
                    provider_quality = "DEGRADED_LIO"
                node._write({
                    "event": "spatial_pose_validation",
                    "frame_id": str(frame_id),
                    "accepted": validation.accepted,
                    "health": validation.health,
                    "reason_code": validation.reason_code,
                    "pslam_delta_xy_m": validation.pslam_delta_xy_m,
                    "pslam_delta_yaw_deg": validation.pslam_delta_yaw_deg,
                    "wheel_delta_xy_m": validation.wheel_delta_xy_m,
                    "raw_pose": raw_spatial_pose.to_dict() if raw_spatial_pose else None,
                    "accepted_pose": validation.accepted_pose.to_dict() if validation.accepted_pose else None,
                    "host_s": node._host_s(),
                })
                observation.sensor_health.update({
                    "spatial_pose_health": validation.health,
                    "spatial_pose_reason": validation.reason_code,
                    "spatial_pose_accepted": validation.accepted,
                })
                # 计划书 §16.2：每次 motion 后一条统一记录，事后排查主要靠它。
                # motion_evidence 只在 :2723 一处被赋值、:2098 一处被清空，而
                # 这两处和本块同属 `if place_graph is not None and pose is not None`，
                # 所以一次运动恰好消费一条、不会重复发。字段名照抄 §16.2 的样例，
                # analyze_search_run.py:141 已经在读 wheel.delta_xy_m/pslam.delta_xy_m。
                if motion_evidence is not None:
                    node._write({
                        "event": "motion_observation_summary",
                        "frame_id": str(frame_id),
                        "requested": {
                            "turn_deg": motion_evidence.requested_turn_deg,
                            "forward_m": motion_evidence.requested_forward_m,
                        },
                        "wheel": {
                            "delta_xy_m": motion_evidence.wheel_delta_xy_m,
                            "delta_yaw_deg": motion_evidence.wheel_delta_yaw_deg,
                        },
                        "pslam": {
                            "delta_xy_m": validation.pslam_delta_xy_m,
                            "delta_yaw_deg": validation.pslam_delta_yaw_deg,
                            "accepted": validation.accepted,
                            "reason": validation.reason_code,
                        },
                        "host_s": node._host_s(),
                    })
                # ⚠ 这是 SpatialPoseValidator 的命名空间（HEALTHY/DEGRADED/
                # RECOVERING/REJECTED，见 spatial_pose_validator.py 顶部），
                # **不是**建图写端 webui_map_snapshot_writer.py 那道"激光修正
                # 停摆"闸门的词表（HEALTHY/DEGRADED_LIO_DRIFT）。两个健康概念
                # 数据源不同，不要为了好看统一它们。键名用 mapping_health 是
                # 因为 analyze_search_run.py:138 先读它。
                if validation.health != motion_state["last_mapping_health"]:
                    node._write({
                        "event": "mapping_health_changed",
                        "frame_id": str(frame_id),
                        "previous_mapping_health": motion_state["last_mapping_health"],
                        "mapping_health": validation.health,
                        "reason_code": validation.reason_code,
                        "host_s": node._host_s(),
                    })
                    motion_state["last_mapping_health"] = validation.health
                # 只发 LIO 漂移这一种拒绝。四种 reason_code 里
                # POSE_INVALID / POSE_TIMESTAMP_NONMONOTONIC / PSLAM_POSE_JUMP /
                # PSLAM_YAW_JUMP 都不是 LIO 漂移，套这个名字会说谎；它们本来
                # 就带在同帧的 spatial_pose_validation 里，没有信息损失。
                if validation.reason_code == "LIO_DRIFT_DURING_ROTATION":
                    node._write({
                        "event": "scan_rejected_lio_drift",
                        "frame_id": str(frame_id),
                        "reason_code": validation.reason_code,
                        "pslam_delta_xy_m": validation.pslam_delta_xy_m,
                        "pslam_delta_yaw_deg": validation.pslam_delta_yaw_deg,
                        "wheel_delta_xy_m": validation.wheel_delta_xy_m,
                        "host_s": node._host_s(),
                    })
            spatial_provenance = {}
            if camera_provider is not None and hasattr(camera_provider, "transform_provenance"):
                try:
                    spatial_provenance = camera_provider.transform_provenance()
                except Exception:  # noqa: BLE001
                    spatial_provenance = {}
            # image_stamp 在下面的 map 投影和末尾的 enrich 两处都要用。投影块会
            # 因 spatial_pose 缺失而整体跳过（运动后位姿校验失败时就是这样），
            # 所以必须在这里先定值，否则 enrich 会撞 UnboundLocalError。
            image_stamp = getattr(observation, "timestamp", None)
            projected_targets = project_target_candidates(
                observation.target_candidates, camera_provider,
                pose_accepted=spatial_pose is not None,
            )
            bind_target_measurements(observation.target_candidates, projected_targets,
                                     source_frame_id=views_source_frame_id)
            # Phase 2: write map_xyz into each localized observation BEFORE
            # entity association so cross-view fusion can use world coords.
            if camera_provider is not None and spatial_pose is not None:
                map_projector = getattr(camera_provider, "camera_point_to_map", None)
                for obs_item in localized:
                    if getattr(obs_item, "camera_xyz", None) is None:
                        continue
                    mapped = None
                    mapped_frame = spatial_provenance.get("target_frame", "map")
                    # 计划书 §10：优先 T_map_camera(t_image)，输出 pslam_map 系。
                    if callable(map_projector):
                        try:
                            mapped = map_projector(obs_item.camera_xyz, image_stamp)
                        except Exception:  # noqa: BLE001 - 投影失败不阻断探索
                            mapped = None
                        if mapped is not None:
                            mapped_frame = getattr(
                                getattr(camera_provider, "_map_bridge", None),
                                "map_frame", getattr(camera_provider, "map_frame", "map"),
                            )
                    if mapped is None:
                        # 回退：pslam_odom 系 + 标称外参（旧行为，如实标注）。
                        try:
                            mapped = camera_provider.camera_point_to_spatial(
                                obs_item.camera_xyz, pose=spatial_pose
                            )
                        except SpatialFrameMismatch as exc:
                            node._write({
                                "event": "spatial_frame_mismatch",
                                "error": str(exc),
                                "host_s": node._host_s(),
                            })
                            mapped = None
                        except Exception:  # noqa: BLE001 - degraded mapping must not crash
                            mapped = None
                    if mapped is not None:
                        obs_item.map_xyz = mapped
                        obs_item.spatial_quality = provider_quality
                        # The provider records the successful TF path while
                        # projecting.  Refresh provenance afterwards so an
                        # earlier "unavailable" sample cannot contradict a
                        # valid map_xyz in the same observation.
                        if hasattr(camera_provider, "transform_provenance"):
                            try:
                                spatial_provenance = (
                                    camera_provider.transform_provenance()
                                )
                            except Exception:  # noqa: BLE001
                                pass
                        anchor_info = None
                        anchor_factory = getattr(
                            camera_provider, "anchor_map_point", None
                        )
                        if callable(anchor_factory):
                            try:
                                anchor_info = anchor_factory(mapped, image_stamp)
                            except Exception:  # noqa: BLE001 - anchor absence degrades honestly
                                anchor_info = None
                        if getattr(obs_item, "box_center_camera", None) is not None:
                            try:
                                box_center = map_projector(
                                    obs_item.box_center_camera, image_stamp
                                )
                            except Exception:  # noqa: BLE001
                                box_center = None
                            obs_item.box_center_map = box_center
                            # Camera-axis AABB is not a map-axis AABB after
                            # rotation.  Keep its frame explicit rather than
                            # copying the dimensions into map coordinates.
                            obs_item.box_size_map = None
                        # 2026-09-09 P2 诊断：物体侧看到的 quality 到底是什么。
                        node.get_logger().info(
                            "object quality: label="
                            + str(getattr(obs_item, "label", "?"))
                            + " provider_quality=" + str(provider_quality)
                            + " map_xyz=" + str(tuple(round(float(v), 3) for v in mapped))
                        )
                        obs_item.provenance = {
                            **dict(obs_item.provenance or {}),
                            **spatial_provenance,
                            "map_frame": mapped_frame,
                            "timestamp": image_stamp,
                            "timestamp_quality": observation.provenance.get(
                                "timestamp_quality"
                            ),
                            "source_sensor": "rgbd",
                            "depth_method": (obs_item.provenance or {}).get(
                                "depth_source", "unknown"
                            ),
                            "box_size_frame": (
                                "camera_optical"
                                if getattr(obs_item, "box_size_camera", None) is not None
                                else None
                            ),
                            "anchor_status": (
                                "anchored" if anchor_info is not None else "unavailable"
                            ),
                            **dict(anchor_info or {}),
                        }
            # 把最终的逐帧原始坐标写回 LiveObservation。匹配只认同帧
            # frame_object_id/source id，不按 label 或下标猜测。
            from app.perception.depth_object_localizer import enrich_scene_objects_with_spatial

            observation.scene_objects = enrich_scene_objects_with_spatial(
                observation.scene_objects,
                localized,
                image_timestamp=float(image_stamp or observation.timestamp),
            )

            place_id, created = place_graph.register_observation(
                observation_id=observation.bundle_id,
                heading_sector=(
                    observation.navigation_heading_sector
                    or observation.heading_sector
                ),
                objects=observation.object_labels,
                rgbd_frame_id=observation.rgbd_frame_id,
                pose=spatial_pose,
                observed_displacement_m=(
                    motion_evidence.wheel_delta_xy_m
                    if motion_evidence is not None else None
                ),
                timestamp=observation.timestamp,
                target_candidate=observation.target_present,
            )
            state["place_id"] = place_id
            state["created_place"] = created
            update_result = None
            if semantic_map is not None:
                correction_revision = spatial_provenance.get(
                    "map_correction_revision"
                )
                graph_revision = spatial_provenance.get("graph_revision")
                reprojection_revision = (
                    f"graph:{graph_revision}"
                    if graph_revision is not None
                    else (
                        f"correction:{correction_revision}"
                        if correction_revision is not None else None
                    )
                )
                previous_revision = state.get("semantic_reprojection_revision")
                reprojector = getattr(
                    camera_provider, "reproject_camera_point_to_map", None
                )
                if (
                    callable(reprojector)
                    and previous_revision is not None
                    and reprojection_revision is not None
                    and reprojection_revision != previous_revision
                ):
                    reprojected = semantic_map.reproject_observations(
                        reprojector,
                        anchor_lookup=getattr(
                            camera_provider, "reproject_anchor_point", None
                        ),
                    )
                    if reprojected["observations"]:
                        node._write({
                            "event": "semantic_reprojected",
                            "from_revision": previous_revision,
                            "to_revision": reprojection_revision,
                            "graph_revision": graph_revision,
                            "map_correction_revision": correction_revision,
                            **reprojected,
                            "host_s": node._host_s(),
                        })
                if reprojection_revision is not None:
                    state["semantic_reprojection_revision"] = reprojection_revision
                # 2026-09-09 P2 诊断：把交给语义图的 localized 整体打出来，
                # 看"打印的那条"是否就是"写进实体的那条"。
                node.get_logger().info(
                    "localized dump: " + str([
                        (getattr(o, "label", "?"),
                         getattr(o, "spatial_quality", "?"),
                         id(o),
                         tuple(round(float(v), 3) for v in (getattr(o, "map_xyz", None) or ())))
                        for o in localized
                    ])
                )
                # §26：人/狗/猫这类动态物体不写进持久地图（否则下次走到同一
                # 地点，地图上还站着一个早就不在的人）。就地重置 localized，
                # 让下游 :2161 的 enumerate(localized) 用的是同一张表 ——
                # association_by_index 的下标就是这张表里的位置，两张表一旦
                # 分叉，persistent_object_id 会串到别的物体上。
                # 显示不受影响：WebUI 的当前检测走 observation.scene_objects。
                current_map_odom_pose = spatial_provenance.get("map_odom_pose")
                if current_map_odom_pose is not None:
                    current_map_odom_pose = tuple(float(value) for value in current_map_odom_pose)
                    prior_pose = last_semantic_map_odom_pose
                    mobility_limits = (mobility_config or {}).get("mobility", {})
                    if prior_pose is not None and (
                        math.hypot(current_map_odom_pose[0] - prior_pose[0],
                                   current_map_odom_pose[1] - prior_pose[1]) >
                            float(mobility_limits.get("map_jump_m", .15))
                        or abs(current_map_odom_pose[2] - prior_pose[2]) >
                            float(mobility_limits.get("map_jump_yaw_rad", .08))
                    ):
                        semantic_mobility_tracker.reset()
                        if target_candidate_mapper is not None:target_candidate_mapper.reset()
                    last_semantic_map_odom_pose = current_map_odom_pose
                tracks = semantic_mobility_tracker.update(
                    localized,
                    frame_id=str(observation.rgbd_frame_id or observation.bundle_id),
                    timestamp=float(observation.timestamp),
                    map_epoch=str(spatial_provenance.get("map_epoch") or ""),
                )
                track_by_source = {str(obs.object_id): track for obs, track in zip(localized, tracks)}
                for obs, track in zip(localized, tracks):
                    for name, value in track.fields().items():
                        setattr(obs, name, value)
                    if track.mobility_class == "dynamic" and track.persistent_id:
                        old = semantic_map.objects.get(track.persistent_id)
                        if old is not None:
                            old.topology_eligible = False
                            old.mobility_class = "dynamic"
                localized = [obs for obs in drop_dynamic_objects(localized)
                             if getattr(obs, "topology_eligible", False)]
                # 2026-09-17 整改 §5.1：持久地图入口门。检测端跑在
                # box_threshold=0.12 这种偏召回的阈值上，碎片框（0.12~0.33、
                # 2.7cm 宽、悬空、同帧分裂）全部会建出持久实体 —— 实测 41 个
                # 持久 ID 里 30 个分数低于 0.35。这道门只挡"入图"，逐帧原始
                # 事件（上面的 semantic_frame_objects_localized）一条不少。
                # `localized` 从此是**入口门之后**的那张表：update_with_
                # associations、association_by_index、semantic_object_localized
                # 的 enumerate 全部共用它，下标不会错位。
                localized, ingest_dropped = filter_persistent_observations(
                    localized,
                    min_confidence=settings.semantic_map_min_object_confidence,
                )
                node._write({
                    "event": "semantic_object_ingest_filter",
                    "input": len(localized) + sum(ingest_dropped.values()),
                    "kept": len(localized),
                    "dropped": ingest_dropped,
                    "min_confidence": settings.semantic_map_min_object_confidence,
                    "host_s": node._host_s(),
                })
                update_result = semantic_map.update_with_associations(
                    localized,
                    place_id=place_id,
                    now=observation.timestamp,
                    frame_id=observation.rgbd_frame_id,
                )
                for association in update_result.associations:
                    track = track_by_source.get(str(association.source_object_id))
                    if track is not None:
                        if track.persistent_id and track.persistent_id != association.persistent_object_id:
                            old = semantic_map.objects.get(track.persistent_id)
                            if old is not None:
                                old.topology_eligible = False
                        track.persistent_id = association.persistent_object_id
                observation.object_update_result = update_result
            if bev_mapper is not None and rgbd_frame is not None and spatial_pose is not None:
                bev_mapper.update(rgbd_frame, spatial_pose)
            state["localized"] = localized

            # Identity bridge (plan §5): frame_object_id -> entity association
            # -> persistent_object_id.  The WebUI object events and the object
            # relation graph must NEVER use label-based identity, so that two
            # chairs / bins keep distinct persistent ids.
            association_by_index: dict[int, Any] = {}
            persistent_by_source_id: dict[str, Any] = {}
            if update_result is not None:
                association_by_index = {
                    assoc.observation_index: assoc
                    for assoc in update_result.associations
                }
                persistent_by_source_id = {
                    str(assoc.source_object_id): assoc
                    for assoc in update_result.associations
                    if assoc.source_object_id
                }

            # 独立记录每帧原始观测；即使本帧没有成功关联持久实体也照常落盘。
            # persistent_object_id 仅来自 source id 的严格关联，绝不按 label 补猜。
            frame_objects: list[dict[str, Any]] = []
            for scene_object in observation.scene_objects:
                event_item = dict(scene_object)
                source_object_id = event_item.get("source_object_id")
                assoc = persistent_by_source_id.get(str(source_object_id or ""))
                if assoc is not None:
                    event_item.update({
                        "persistent_object_id": assoc.persistent_object_id,
                        "association_score": assoc.association_score,
                        "association_action": assoc.action,
                    })
                frame_objects.append(event_item)
            node._write({
                "event": "semantic_frame_objects_localized",
                "frame_id": observation.rgbd_frame_id or semantic_frame_id,
                "image_timestamp": observation.provenance.get("image_timestamp"),
                "timestamp_quality": observation.provenance.get("timestamp_quality"),
                "objects": frame_objects,
                "host_s": node._host_s(),
            })

            # Phase 3: publish spatial events for the WebUI, including the real
            # map_xyz on every localized object.
            if camera_provider is not None and observation.rgbd_frame_id:
                node._write({
                    "event": "rgbd_frame_updated",
                    "frame_id": observation.rgbd_frame_id,
                    "depth_ref": observation.depth_ref,
                    "intrinsics": observation.intrinsics,
                    "depth_scale": observation.depth_scale,
                    "spatial_quality": getattr(camera_provider, "quality", lambda: "CAMERA_LOCAL")(),
                    "host_s": node._host_s(),
                })
            node._write({
                "event": "spatial_pose_updated",
                "pose": spatial_pose.to_dict() if spatial_pose is not None else None,
                "quality": (
                    getattr(camera_provider, "quality", lambda: "CAMERA_LOCAL")()
                    if camera_provider is not None else "CAMERA_LOCAL"
                ),
                "host_s": node._host_s(),
            })
            for index, obs_item in enumerate(localized):
                if getattr(obs_item, "map_xyz", None) is None:
                    continue
                assoc = association_by_index.get(index)
                if assoc is None:
                    # No reliable persistent identity this frame; do not guess
                    # from the label (two same-label objects must stay distinct).
                    continue
                entity = None
                if semantic_map is not None:
                    entity = semantic_map.objects.get(assoc.persistent_object_id)
                event_object = {}
                if entity is not None:
                    event_object = entity.to_dict()
                event_object.update({
                    "object_id": assoc.persistent_object_id,
                    "label": obs_item.label,
                    "camera_xyz": list(obs_item.camera_xyz) if obs_item.camera_xyz else None,
                    "map_xyz": list(obs_item.map_xyz),
                    "spatial_quality": obs_item.spatial_quality,
                    "source_object_id": assoc.source_object_id,
                    "association_score": assoc.association_score,
                    "association_action": assoc.action,
                    # §10 取证：map_xyz 到底落在哪个 frame（pslam_map 还是 pslam_odom）。
                    "map_frame": (obs_item.provenance or {}).get("map_frame", "unknown"),
                })
                node._write({
                    "event": "semantic_object_localized",
                    "object": event_object,
                    "host_s": node._host_s(),
                })
            node._write({
                "event": "place_updated",
                "place": place_graph.places[place_id].to_dict(),
                "host_s": node._host_s(),
            })
            if update_result is not None and entity_graph is not None:
                entity_graph.sync_from_observation(
                    observation_id=observation.bundle_id,
                    heading_sector=(
                        observation.navigation_heading_sector
                        or observation.heading_sector
                    ),
                    labels=observation.object_labels,
                    spatial_objects=localized,
                    pose=spatial_pose,
                    timestamp=observation.timestamp,
                    place_id=place_id,
                    update_result=update_result,
                    relations=list(getattr(semantic, "relations", None) or []),
                )
            # One motion action is consumed by exactly one post-motion
            # observation.  Leaving it pending would double-count a later
            # stationary frame as a relocation.
            motion_state["pending_motion_evidence"] = None

        if (observation.provenance.get('target_spatial_contract') == 'capture_v1'
                and target_candidate_mapper is not None and semantic_map is not None):
            from app.spatial.target_candidate_map import merge_updates
            persisted_targets, target_update, target_diagnostics = target_candidate_mapper.ingest(
                observation.target_candidates, semantic_map,
                place_id=state.get('place_id'),
                min_confidence=settings.semantic_map_min_object_confidence,
            )
            bind_target_measurements(observation.target_candidates, persisted_targets,
                                     source_frame_id=views_source_frame_id)
            observation.object_update_result = merge_updates(observation.object_update_result, target_update)
            node._write({'event':'target_object_ingest','groups':target_diagnostics,
                         'motion_authorized':False,'host_s':node._host_s()})
        node._write({
            "event": "target_candidates_localized",
            "frame_id": str(frame_id),
            "candidates": [{key: item.get(key) for key in (
                "entity_id", "entity_source_frame_id", "label", "bbox_2d", "score",
                "depth_m", "camera_xyz", "bearing_deg", "map_xyz", "spatial_quality",
                "spatial_provenance", "persistent_object_id", "target_persistence",
            )} for item in observation.target_candidates],
            "motion_authorized": False,
            "host_s": node._host_s(),
        })
        profiler.record("blocking_decision_ms", "quick_start")
        profiler.record("cycle_total_ms", "capture_start")
        state["last_success_ts"] = time.time()
        node._write({
            "event": "latency_profile",
            "planning_cycle": profiler.profile.planning_cycle,
            "frame_id": profiler.profile.frame_id,
            "timings_ms": profiler.profile.timings_ms,
            "api_calls": profiler.profile.api_calls,
            "counters": profiler.profile.counters,
            "host_s": node._host_s(),
        })
        return observation

    def navigation_target_monitor() -> bool:
        """Low-frequency local detection while Nav2 owns the control loop."""
        nonlocal cadence_entries
        if not is_local_structured_backend(getattr(node, "_detector", "llm")):
            return False
        if rgbd_source is not None:
            frame = rgbd_source.get_latest(timeout_seconds=2.0)
            frame_id = frame.evidence_id
            image_path = _cached_image(frame.color_ref, frame_id)
            capture_timestamp = float(frame.timestamp)
        else:
            raw_image, frame_id = node._latest_bundle_image(spool_root, retries=0)
            image_path = _cached_image(raw_image, frame_id)
            capture_timestamp = time.time()
        x_now, y_now, yaw_now = node._odom_snapshot()
        now = time.monotonic()
        trigger = detection_scheduler.evaluate(
            now=now,
            pose=(x_now, y_now, yaw_now),
            event=state.get("detection_event"),
            place_id=state.get("place_id"),
        )
        if not trigger.should_detect:
            return False
        objects = node._detect(
            image_path,
            prompt,
            env,
            frame_id=frame_id,
            slots=frozenset(ALL_SLOTS),
        )
        detection_scheduler.mark_detected(
            now=time.monotonic(),
            pose=(x_now, y_now, yaw_now),
            place_id=state.get("place_id"),
        )
        state.pop("detection_event", None)
        cadence_entries = put_slot_entry(
            cadence_entries,
            SlotCacheEntry(
                slots=frozenset(ALL_SLOTS),
                objects=deepcopy(objects),
                frame_id=str(frame_id),
                at=time.monotonic(),
            ),
        )
        local_cache = state.setdefault("local_detection_by_frame", {})
        local_cache[str(frame_id)] = {
            "objects": deepcopy(objects),
            "source_frame_id": str(frame_id),
            "capture_timestamp": capture_timestamp,
            "robot_pose": {
                "x": x_now, "y": y_now, "yaw_rad": yaw_now,
                "yaw_deg": math.degrees(yaw_now),
            },
        }
        roles = classify_roles(
            objects,
            controller.target_profile,
            source_frame_id=str(frame_id),
        )
        best_score = max(
            (float(item.get("score", 0.0) or 0.0)
             for item in roles.target_candidates),
            default=0.0,
        )
        interrupt_threshold = max(
            float(args.target_score_min),
            float(settings.semantic_map_min_object_confidence),
        )
        interrupt = best_score >= interrupt_threshold
        node._write({
            "event": "navigation_detection",
            "frame_id": str(frame_id),
            "detector_backend": getattr(node, "_detector", "llm"),
            "detection_trigger": trigger.reason,
            "translation_m": round(trigger.translation_m, 4),
            "rotation_deg": round(trigger.rotation_deg, 3),
            "candidate_count": len(roles.target_candidates),
            "best_target_score": round(best_score, 4),
            "interrupt_threshold": interrupt_threshold,
            "interrupt": interrupt,
            "host_s": node._host_s(),
        })
        if interrupt:
            state["detection_event"] = "target_candidate"
        return interrupt

    # ---- matcher -------------------------------------------------------------
    # 计划书 §4.1：把已有任务、本地知识和只读拓扑摘要送进推理上下文。
    # 这三项只在解释与审计里出现，**不参与方向排序**（§4.4 / §4.5）。
    # 任务与知识是整个会话的常量，所以在闭包外算一次 —— matcher 每帧都跑，
    # 每次进去读一遍知识库文件是纯浪费。拓扑摘要**不在此列**，见下面 matcher 内。
    agent_task_context = (
        task_context.to_dict() if hasattr(task_context, "to_dict") else task_context
    )
    agent_knowledge_items = retrieve_knowledge(
        str(getattr(task_context, "canonical_target", "") or "")
    )

    def _refresh_reasoning_world_snapshot(observation: LiveObservation):
        """Build the current measured world before Task Agent reasoning.

        The goal generator later replaces this with a candidate-complete
        snapshot.  Building this pre-reasoning envelope prevents the first
        cycle from receiving ``world_state=None`` while preserving the rule
        that only checked candidates enter the final decision snapshot.
        """
        if camera_provider is None or not hasattr(camera_provider, "get_map"):
            return state.get("world_state_snapshot")
        metric_map = camera_provider.get_map()
        if metric_map is None:
            return state.get("world_state_snapshot")
        map_provenance = dict(getattr(metric_map, "provenance", {}) or {})
        runtime_epoch = state.setdefault(
            "map_epoch", f"runtime_{int(time.time() * 1000)}"
        )
        revisions = WorldRevisions(
            map_id=str(map_provenance.get("map_id") or "runtime_map"),
            map_epoch=str(map_provenance.get("map_epoch") or runtime_epoch),
            graph_revision=int(map_provenance.get("graph_revision") or 0),
            grid_revision=int(getattr(metric_map, "revision", 0) or 0),
            semantic_revision=int(getattr(semantic_map, "revision", 0) or 0),
            frame_id=str(map_provenance.get("frame_id") or "map"),
            sensor_profile_id=str(
                map_provenance.get("sensor_profile_id")
                or os.getenv("GO2W_GEOMETRY_PROFILE_SHA256", "unknown")
            ),
        )
        provider_health = (
            camera_provider.health()
            if hasattr(camera_provider, "health") else {}
        )
        pose = camera_provider.get_pose() if hasattr(camera_provider, "get_pose") else None
        pose_value = pose.to_dict() if hasattr(pose, "to_dict") else observation.pose
        snapshot = build_world_state_snapshot(
            revisions=revisions,
            task=dict(agent_task_context or {}),
            robot={
                "pose": pose_value,
                "spatial_quality": getattr(
                    camera_provider, "quality", lambda: "unavailable"
                )(),
            },
            map_summary={
                "source": getattr(metric_map, "source", None),
                "resolution_m": getattr(metric_map, "resolution_m", None),
                "width": getattr(metric_map, "width", None),
                "height": getattr(metric_map, "height", None),
                "free_cells": len(getattr(metric_map, "free", []) or []),
                "occupied_cells": len(getattr(metric_map, "occupied", []) or []),
                "unknown_cells": len(getattr(metric_map, "unknown", []) or []),
                "quality": provider_health.get("mapping_quality"),
            },
            objects=(
                [entry.to_dict() for entry in semantic_map.objects.values()]
                if semantic_map is not None else []
            ),
            topology=place_graph.to_dict() if place_graph is not None else {},
            search_memory=(
                spatial_memory.to_dict() if spatial_memory is not None else {}
            ),
            candidates=list(state.get("candidate_audit") or []),
            feedback={"last_decision": state.get("last_decision")},
            health={
                "input_health": provider_health,
                "geometry_health": observation.sensor_health,
                "localization_health": provider_health.get("localization_valid"),
                "mapping_quality": provider_health.get("mapping_quality"),
                "motion_authorized": False,
            },
        )
        value = snapshot.to_dict()
        state["world_state_snapshot"] = value
        node._write({
            "event": "world_state_snapshot_pre_reasoning",
            "snapshot": value,
            "source_observation_id": str(observation.bundle_id),
            "host_s": node._host_s(),
        })
        return value

    def matcher(observation: LiveObservation) -> SemanticMatch:
        context_start_perf = time.perf_counter()
        semantic = state.get("semantic")
        scene_graph = getattr(semantic, "scene_graph", None)
        current_world_snapshot = _refresh_reasoning_world_snapshot(observation)
        context = SearchReasoningContext(
            target_profile=controller.target_profile,
            scene_graph=scene_graph,
            negative_memory=semantic_memory,
            robot_pose=observation.pose,
            robot_yaw_deg=float((observation.pose or {}).get("yaw_deg", 0.0)),
            observed_heading_sectors=sorted(observed_sectors),
            world_state_snapshot=current_world_snapshot,
        )
        # 拓扑摘要在**闭包内逐帧**算：place_graph 要到 explorer.run() 之后才开始
        # 装观测，在外面只算一次，就会把一张刚 new 出来的空图冻结成整个会话的
        # "当前拓扑"，而离线评估器读的是收尾才写的终局图 —— 两条路口径不一致。
        # 逐帧读它是 O(1)（只读 places/edges 的长度，不走 to_dict()），没有成本问题。
        enrich_context(
            context,
            task_context=agent_task_context,
            knowledge_items=agent_knowledge_items,
            topology_summary=summarize_topology(place_graph),
            run_mode="live",
        )
        context_end_perf = time.perf_counter()
        directive = controller.propose(context)
        reasoner_end_perf = time.perf_counter()
        directive = attach_trace(context, directive)
        trace_end_perf = time.perf_counter()
        node._write({
            "event": "task_agent_timing",
            "frame_id": str(observation.bundle_id),
            "context_ms": round((context_end_perf - context_start_perf) * 1000, 3),
            "reasoner_ms": round((reasoner_end_perf - context_end_perf) * 1000, 3),
            "trace_ms": round((trace_end_perf - reasoner_end_perf) * 1000, 3),
            "host_s": node._host_s(),
        })
        graph_match = context.graph_match
        anchor_labels: list[str] = []
        if graph_match is not None:
            anchor_ids = set(graph_match.supporting_anchor_scene_node_ids)
            nodes = (
                scene_graph.nodes if scene_graph is not None
                and hasattr(scene_graph, "nodes") else []
            )
            for scene_node in nodes:
                node_id = getattr(scene_node, "node_id", None)
                label = getattr(scene_node, "label_zh", None) or getattr(scene_node, "label", None)
                if node_id in anchor_ids and label:
                    anchor_labels.append(str(label))
        match = SemanticMatch(
            has_candidate=bool(observation.target_present),
            graph_match=graph_match,
            directive=directive,
            target_profile=controller.target_profile,
            anchor_labels=anchor_labels,
            target_score=graph_match.score if graph_match is not None else 0.0,
            target_match_level=(
                graph_match.state.value if graph_match is not None else "none"
            ),
            target_state=observation.target_state,
            provenance={"source": "semantic_navigation_matcher"},
        )
        state["match"] = match
        return match

    # ---- verifier -------------------------------------------------------------
    # 同一 frame_id + bbox + target 最多调一次 Verify API；跨 fresh frame 可重试。
    verify_cache = VerificationCache()

    # 计划书 §5.1.5：有界异步复核。`GO2W_ASYNC_VERIFY_ENABLED=false`（默认）
    # 时这个对象**不建**，`verifier` 逐字走旧的同步路径。
    async_verifier: AsyncVerifierManager | None = None
    if settings.go2w_async_verify_enabled:
        async_verifier = AsyncVerifierManager(
            node._verify_target,
            enabled=True,
            max_inflight=settings.go2w_verify_max_inflight,
            max_pending=settings.go2w_verify_max_pending,
            client_timeout_s=settings.go2w_verify_client_timeout_s,
            event_sink=node._write,
        )
        # 代次 = 本次 worker 代。换目标/重启会话会换一个 worker，上一代的
        # 复核结论一律不许算到这一代头上。
        async_verifier.set_task_generation(int(getattr(args, "worker_generation", 0) or 0))
    # 挂在节点上，好让外层那个 finally 能像收 `_semantic_manager` 一样收它。
    # 两处都显式赋值：关着开关时留一个上一轮的旧对象在那儿，收尾会去关一个
    # 早就该扔掉的东西。
    node._async_verifier = async_verifier

    def _async_verify(observation: LiveObservation,
                      best: dict[str, Any]) -> VerificationOutcome:
        """提交一条复核并**有界**等它 —— 等不到就出 UNKNOWN（§5.1.5）。

        与旧路径的三处不同，逐条写清楚免得后人以为是"同一件事换个写法"：

        1. **上限归客户端**。旧路等的是 daemon 自己的 45 s 超时，超了抛
           ``PerceptionFailure``；这里的截止线是 ``GO2W_VERIFY_CLIENT_TIMEOUT_S``，
           到点一定返回。
        2. **没有判定就不写缓存**。只有 CONFIRMED/REJECTED 才进 `verify_cache`；
           把一条 UNKNOWN 缓存成"这一帧问过了"，下次同一帧就不会再问 —— 而它
           其实什么都没答。
        3. **候选身份用 `entity_id`**。异步的序号丢弃与 latest-wins 合并都绑在
           `(代次, 帧, 候选)` 上；没有实体 ID 就只能退化成"按帧"，而同帧里
           换一个候选会被当成同一条请求。
        4. **只等本次提交的序号**。框和标签不变时 `entity_id_for()` 跨帧算出来是
           同一个 ID，所以"按候选取最近的结论"会在新帧上立刻命中上一帧那一条
           （审查 P1-a：新帧消费旧结论 ⇒ 假"找到了"）。这里拿 `submit()` 的返回
           值去 `await_submission()`，并当面对一遍帧/候选/代次。
        """
        # 候选身份**现算**，不去读 `best["entity_id"]`：职责分离关着时
        # （`GO2W_RESULT_ROLE_SEPARATION=false` + scope=all_detections）复核池是
        # 裸 detections，没人给它打过 ID。`entity_id_for` 与任务 2 的
        # `assign_entity_ids` 是同一个函数，所以两处算出来的 ID 一定相同 ——
        # 这正是"序号丢弃"需要的那种稳定身份。
        candidate_id = entity_id_for(best)

        def _report_queue(outcome_status: str) -> None:
            """队列可见性（§5.1.5 第 4 条）：在飞/待办/丢弃各多少，一条事件说清。

            只统计**客户端侧**计数，不动 wire protocol —— 改协议要同时改 daemon。
            """
            node._write({
                "event": "verify_queue_depth",
                "candidate_id": candidate_id,
                "outcome_status": outcome_status,
                "host_s": node._host_s(),
                **async_verifier.queue_depth(),
            })

        if not best.get("bbox_2d"):
            # 没有框就没法裁图复核。这也是"没有判定"（而不是"目标不存在"）。
            return VerificationOutcome(
                confirmed=False,
                attempts=1,
                reason_zh="candidate has no bbox_2d; nothing to crop for verification",
                status=VERIFY_STATUS_UNKNOWN,
            )
        submission = async_verifier.submit(
            frame_id=observation.bundle_id,
            candidate_id=candidate_id,
            image_path=observation.image_ref,
            bbox=list(best["bbox_2d"]),
            env=env,
        )
        if submission == 0:
            # 没起线程：在飞的占满了 / 被合并进待办。这一条**没有可等的结论**，
            # 不许拿这个候选的历史结论顶上（那正是审查 P1-a 的形态）。
            node._write({
                "event": "verify_submission_coalesced",
                "frame_id": str(observation.bundle_id),
                "candidate_id": candidate_id,
                "host_s": node._host_s(),
            })
        # 只等**本次提交**（序号）的结论 —— 不是"这个候选最近怎么样"。
        outcome = async_verifier.await_submission(submission)
        # 第二道身份核验。绑定已经发生在 `await_submission`（按序号取）里，
        # 这里再把帧/候选/代次当面对一遍：哪天上游把序号接错，也不会把别人的
        # 结论当成这一帧的判定。对不上就按"没判定"办 —— 宁可 UNKNOWN，也不能
        # 拿错结论去宣布找到目标。
        if outcome is not None and not result_matches_submission(
            outcome,
            sequence=submission,
            frame_id=str(observation.bundle_id),
            candidate_id=candidate_id,
            task_generation=async_verifier.task_generation,
        ):
            node._write({
                "event": "verify_identity_mismatch",
                "submitted_sequence": submission,
                "returned_sequence": outcome.sequence,
                "expected_frame_id": str(observation.bundle_id),
                "returned_frame_id": outcome.frame_id,
                "expected_candidate_id": candidate_id,
                "returned_candidate_id": outcome.candidate_id,
                "host_s": node._host_s(),
            })
            outcome = None
        if outcome is None or not outcome.terminal:
            _report_queue(VERIFY_STATUS_UNKNOWN)
            return VerificationOutcome(
                confirmed=False,
                attempts=1,
                reason_zh=(
                    outcome.reason_zh if outcome is not None
                    else "no verification verdict within client timeout"
                         "（未出判定，不作为目标不存在的证据）"
                ),
                details=dict(outcome.details) if outcome is not None else {},
                status=VERIFY_STATUS_UNKNOWN,
            )
        _report_queue(outcome.status)
        # 只有**终结**的结论才允许进入"目标存不存在"的判定，也才值得缓存。
        confidence = float(outcome.details.get("confidence", 0.0))
        confirmed = (
            outcome.status == VERIFY_STATUS_CONFIRMED
            and confidence >= args.verify_min_confidence
        )
        verify_cache.put(
            verify_cache.make_key(
                observation.bundle_id, args.target, list(best["bbox_2d"]),
                candidate_identity=(target_identity(best) if observation.provenance.get(
                    'target_spatial_contract') == 'capture_v1' else None),
            ),
            VerificationCacheEntry(
                confirmed=confirmed,
                confidence=confidence,
                reason=outcome.reason_zh,
                details=dict(outcome.details),
            ),
        )
        return VerificationOutcome(
            confirmed=confirmed,
            attempts=1,
            reason_zh=outcome.reason_zh,
            details=dict(outcome.details),
            status=outcome.status,
        )

    def _verify_candidate_unbound(observation: LiveObservation,
                 match: SemanticMatch) -> VerificationOutcome:
        image_path = observation.image_ref
        if not isinstance(image_path, str):
            return VerificationOutcome(confirmed=False, attempts=1,
                                       reason_zh="no image for verification")
        # 计划书 §5.1.3：复核的是**目标候选**，不是"画面里最显眼的那件东西"。
        # 旧代码在全量 detections 上取最高分，一把 0.95 的椅子会把 0.32 的目标桶
        # 挤出复核位 ⇒ 复核的是椅子，报出来的却是"目标确认为真"。
        best, authoritative = select_candidate(
            observation.target_candidates,
            observation.detections,
            scope=settings.go2w_verify_candidate_scope,
            candidates_authoritative=settings.go2w_result_role_separation,
        )
        if best is None:
            return VerificationOutcome(
                confirmed=False,
                attempts=1,
                reason_zh=(
                    "no target candidate to verify"
                    "（目标候选为空；场景物体不作为复核对象，"
                    "不因此判目标存在）"
                    if authoritative
                    else "no detection to verify"
                ),
            )
        key = verify_cache.make_key(
            observation.bundle_id, args.target, list(best["bbox_2d"]),
            candidate_identity=(target_identity(best) if observation.provenance.get(
                'target_spatial_contract') == 'capture_v1' else None),
        )
        cached = verify_cache.get(key)
        if cached is not None:
            return VerificationOutcome(
                confirmed=cached.confirmed,
                attempts=1,
                reason_zh=(cached.reason or "cached same-frame verification"),
                details=dict(cached.details or {}),
            )
        if async_verifier is not None:
            return _async_verify(observation, best)
        # 帧号取**本次观测自己的身份**（`bundle_id`，同时也是上面 verify_cache
        # 的键），不取"最近一帧"那个共享可变字段：异步下它未必还是这一帧
        # （独立复审 F1：确需帧号的 Verify/事件优先用本次请求上下文）。
        result = node._verify_target(
            image_path, list(best["bbox_2d"]), env,
            frame_id=observation.bundle_id,
        )
        confirmed = bool(result.get("is_target", False)) and float(
            result.get("confidence", 0.0)
        ) >= args.verify_min_confidence
        outcome = VerificationOutcome(
            confirmed=confirmed,
            attempts=1,
            reason_zh=str(result.get("reason_zh", "")),
            details=result,
        )
        verify_cache.put(
            key,
            VerificationCacheEntry(
                confirmed=confirmed,
                confidence=float(result.get("confidence", 0.0)),
                reason=str(result.get("reason_zh", "")),
                details=result,
            ),
        )
        return outcome

    def verifier(observation: LiveObservation, match: SemanticMatch) -> VerificationOutcome:
        if observation.provenance.get('target_spatial_contract') != 'capture_v1':
            return _verify_candidate_unbound(observation, match)
        from dataclasses import replace
        from app.perception.target_spatial import target_identity
        best,_ = select_candidate(observation.target_candidates, observation.detections,
            scope=settings.go2w_verify_candidate_scope,
            candidates_authoritative=settings.go2w_result_role_separation)
        identity = target_identity(best)
        if identity is None or identity['source_frame_id'] != observation.provenance.get('current_capture_id'):
            return VerificationOutcome(confirmed=False,attempts=0,status=VERIFY_STATUS_UNKNOWN,
                reason_zh='目标框与当前复核图像的采集身份不一致，保持停车重观测')
        outcome = _verify_candidate_unbound(observation, match)
        if (target_identity(best) != identity
                or (outcome.details.get('target_identity') is not None
                    and outcome.details['target_identity'] != identity)):
            return VerificationOutcome(confirmed=False,attempts=outcome.attempts,status=VERIFY_STATUS_UNKNOWN,
                reason_zh='复核期间目标候选身份改变，保持停车重观测')
        node._write({'event':'target_verification_binding','identity':identity,
                     'confirmed':outcome.confirmed,'status':outcome.status,
                     'host_s':node._host_s()})
        return replace(outcome,details={**outcome.details,'target_identity':identity})

    # ---- SemanticNavigation V2 spatial candidate generator / planner ------------------
    def _scan_info_marker(place) -> tuple:
        """计划书 §7.4“新信息”定义：新 navigation sector / 新 object / 新 label。"""
        return (
            sorted(observed_sectors),
            len(semantic_map.objects) if semantic_map is not None else 0,
            sorted(place.observed_object_labels) if place is not None else [],
        )

    def _place_scan_has_new_info(place_id: str, place) -> bool:
        marker_key = f"scan_info_marker_{place_id}"
        previous = state.get(marker_key)
        current = _scan_info_marker(place)
        state[marker_key] = current
        return previous is not None and current != previous

    def spatial_candidate_generator(**kwargs: Any) -> list[Any]:
        """V2 candidate generator: selects a long-term spatial intent and
        returns the next local primitive for the current intent."""
        observation = kwargs.get("observation")
        capabilities = kwargs.get("capabilities")
        current_yaw_deg = float(kwargs.get("current_yaw_deg") or 0.0)
        # 计划书 §6.7：地图系后端要有当前位置才能把「观察转向」表达成
        # NAVIGATE_POSE(x, y, yaw)。拿不到位姿时保持 None，让下游显式放弃。
        raw_pose = kwargs.get("robot_pose")
        current_position: tuple[float, float] | None = None
        if raw_pose is not None and len(raw_pose) >= 2:
            current_position = (float(raw_pose[0]), float(raw_pose[1]))
        # §6.3：``robot_pose`` 的两种坏法得分得开——key 根本没传（explorer 的
        # plan_context 提前 return 了）与 key 传了但值为 None（``_has_xy`` 判定这帧
        # 没有坐标）。两者的下游症状完全一样，只报一个 bool 会把定位方向带偏。
        robot_pose_source = (
            "absent" if "robot_pose" not in kwargs
            else ("none" if raw_pose is None else f"ok:{len(raw_pose)}")
        )
        plan_context_keys = sorted(
            key for key in (
                "clearance_map", "frontiers", "viewpoint_planner",
                "robot_pose", "occupancy_grid",
            )
            if key in kwargs
        )
        # 计划书 §6.3：每次生成都记录真实的地图/frontier 证据，供探索器区分
        # “等地图 / 规划缺陷 / 真穷尽”，不再让空列表自己去猜原因。
        diagnostics: dict[str, Any] = {
            "map_fresh": None,
            "map_source": None,
            "map_revision": 0,
            "frontier_count": 0,
            "reachable_frontier_count": 0,
            "visited_frontier_count": 0,
            "unreachable_frontier_count": 0,
            "local_scan_quota_exhausted": False,
            "empty_reason": "",
            # §6.3：地图系后端要「当前位置」才表达得出 NAVIGATE_POSE，而这份
            # 位置来自 explorer 的 plan_context。缺了它就会退到 ``local_executor``
            # 里无原语可发、只报一个 ``local_executor_no_primitive``——现场分不清
            # 是「位姿没传进来」还是「候选真穷尽」。这两条把分叉点直接记下来。
            "robot_pose_present": False,
            "capabilities_metric_navigation": False,
            "robot_pose_source": "",
            "plan_context_keys": [],
        }
        diagnostics["robot_pose_present"] = current_position is not None
        diagnostics["robot_pose_source"] = robot_pose_source
        diagnostics["plan_context_keys"] = plan_context_keys
        diagnostics["capabilities_metric_navigation"] = bool(
            getattr(capabilities, "supports_metric_navigation", False)
        )
        state["last_candidate_diagnostics"] = diagnostics

        # Continue an active local intent if it still has primitives.
        if local_executor is not None and local_executor.active:
            goal = local_executor.next_goal(
                current_yaw_deg=current_yaw_deg,
                current_position=current_position,
                capabilities=capabilities,
            )
            if goal is not None:
                return [goal]
            local_executor.finish()

        if place_graph is None or spatial_reasoner is None:
            diagnostics["empty_reason"] = "spatial_provider_missing"
            return []

        match = state.get("match")
        graph_match = getattr(match, "graph_match", None) if match is not None else None
        match_state = (
            graph_match.state.value if graph_match is not None else "zero_match"
        )

        # ---- bounded LOCAL_SCAN (计划书 §7 / §57-§58) ----------------------
        # 配额语义：同一个 Place 最多执行 max_local_rotations 次 LOCAL_SCAN
        # 旋转动作（local_scan_steps），与 heading coverage 覆盖数完全解耦。
        if (
            args.backend != "nav2"
            and match_state in {"zero_match", "partial_match"}
            and place_graph is not None
        ):
            current_place = place_graph.current_place()
            if current_place is not None:
                max_local_rotations = max(0, int(args.max_local_rotations))
                scan_states = state.setdefault("local_scan_states", {})
                scan_state = LocalScanState.from_dict(
                    scan_states.get(current_place.place_id)
                )
                has_new_info = _place_scan_has_new_info(
                    current_place.place_id, current_place
                )
                scan_map = camera_provider.get_map() if camera_provider is not None and args.backend=='nav2' else None
                goal_fields, next_state = select_local_scan_goal(
                    current_yaw_deg=current_yaw_deg,
                    heading_coverage=current_place.heading_coverage,
                    max_local_rotations=max_local_rotations,
                    state=scan_state,
                    sector_deg=30.0,
                    sectors=12,
                    new_information=has_new_info,
                    max_turn_deg=min(10., float(getattr(node, '_experiment_max_turn_deg', args.max_turn_deg)))
                        if os.getenv('GO2W_NAV2_TRIAL_LEDGER', '').strip() else None,
                    goal_is_allowed=(
                        lambda delta: scan_is_clear(scan_map,current_position,current_yaw_deg,delta)
                    ) if args.backend=='nav2' else None,
                )
                scan_states[current_place.place_id] = next_state.to_dict()
                state["local_scan_states"] = scan_states
                if goal_fields is None:
                    if next_state.steps >= max_local_rotations > 0:
                        diagnostics["local_scan_quota_exhausted"] = True
                        node._write({
                            "event": "local_scan_quota_exhausted",
                            "place_id": current_place.place_id,
                            "steps": next_state.steps,
                            "max_local_rotations": max_local_rotations,
                            "host_s": node._host_s(),
                        })
                    # 无合法候选/配额耗尽：落到下方长期目标规划分支。
                else:
                    node._write({
                        "event": "local_scan_candidate",
                        "place_id": current_place.place_id,
                        "candidates": [
                            {"delta_sector": delta, "sector": (int(round(current_yaw_deg / 30.0)) + delta) % 12}
                            for delta in (1, -1, 2, -2)
                        ],
                        "host_s": node._host_s(),
                    })
                    goal = ExplorationGoal(
                        **{**goal_fields,
                           "goal_id": f"local_scan_{next_state.steps:03d}",
                           "provenance": {
                               **goal_fields["provenance"],
                               "place_id": current_place.place_id,
                           }},
                    )
                    # 计划书 §6.7 / §12.4：Nav2 模式下 LOCAL_SCAN 的观察转向
                    # 也必须是「当前位置 + 目标朝向」的地图系 NAVIGATE_POSE。
                    # 拿不到当前位置就放弃这个候选，落到下面的长期目标分支。
                    if not bool(getattr(capabilities, "supports_metric_navigation", False)):
                        scanned_goal = goal
                    elif current_position is None:
                        diagnostics["empty_reason"] = "local_scan_no_current_position"
                        scanned_goal = None
                    else:
                        scanned_goal = ExplorationGoal(
                            goal_id=goal.goal_id,
                            goal_type="NAVIGATE_POSE",
                            position=current_position,
                            yaw=current_yaw_deg + float(goal.relative_dyaw or 0.0),
                            frame="map",
                            semantic_reason=goal.semantic_reason,
                            heading_sector=goal.heading_sector,
                            expected_information_gain=goal.expected_information_gain,
                            provenance=goal.provenance,
                        )
                    if scanned_goal is not None:
                        node._write({
                            "event": "local_scan_selected",
                            "goal_id": scanned_goal.goal_id,
                            "goal_type": scanned_goal.goal_type,
                            "relative_dyaw": goal.relative_dyaw,
                            "goal_yaw_deg": scanned_goal.yaw,
                            "position": (
                                list(scanned_goal.position)
                                if scanned_goal.position is not None else None
                            ),
                            "heading_sector": goal.heading_sector,
                            "steps": next_state.steps,
                            "max_local_rotations": max_local_rotations,
                            "same_direction_count": next_state.same_direction_count,
                            "host_s": node._host_s(),
                        })
                        return [scanned_goal]

        # Metric map first (plain_slam / RTAB-Map), then BEV fallback,
        # otherwise relative frontier.
        frontiers: list[Any] = []
        map_snap = None
        pose = None
        if camera_provider is not None and hasattr(camera_provider, "get_map"):
            spin = getattr(camera_provider, "spin_once", None)
            if spin is not None:
                spin()
            map_snap = camera_provider.get_map()
            if map_snap is not None and map_snap.revision > 0:
                pose = camera_provider.get_pose()
                frontiers = frontier_extractor.extract(map_snap, pose) if frontier_extractor else []
        if not frontiers and bev_mapper is not None:
            map_snap = bev_mapper.get_map()
            if map_snap is not None and map_snap.revision > 0:
                pose = camera_provider.get_pose() if camera_provider is not None else None
                frontiers = frontier_extractor.extract(map_snap, pose) if frontier_extractor else []
        if not frontiers and camera_provider is not None:
            frontiers = camera_provider.get_frontiers()

        # Acceptance diagnostics (plan §28): log which metric map fed the
        # frontier extraction so the data chain is auditable.
        # Keep the map/pose pair that passed the provider health gate at the
        # start of this planning cycle.  Re-reading a 10 Hz localization gate
        # here can turn one internally consistent cycle into "frontiers from a
        # map that is now None" when validity toggles between two adjacent
        # callbacks.  Execution still performs its own fresh Nav2 preflight.
        metric_map = map_snap
        if (
            metric_map is None
            and camera_provider is not None
            and hasattr(camera_provider, "get_map")
        ):
            metric_map = camera_provider.get_map()
        if (
            metric_map is not None
            and metric_map.revision > 0
            and camera_provider is not None
        ):
            from app.spatial.junction_extractor import extract_junction_candidates

            junctions = extract_junction_candidates(
                metric_map, camera_provider.get_pose()
            )
            known_ids = {item.frontier_id for item in frontiers}
            frontiers.extend(
                item for item in junctions if item.frontier_id not in known_ids
            )
        diagnostics["map_fresh"] = bool(
            metric_map is not None and metric_map.revision > 0
        )
        diagnostics["map_source"] = (
            getattr(metric_map, "source", None) if metric_map is not None else None
        )
        diagnostics["map_revision"] = int(
            getattr(metric_map, "revision", 0) if metric_map is not None else 0
        )
        diagnostics["frontier_count"] = len(frontiers)
        # 结算上一次选中的前沿（唯一的 ``mark_frontier_visited`` 写入口，
        # 详见 ``_settle_frontier_selection``）。放在前沿集构建之后、选择之前：
        # 这一轮的选择会覆盖 ``state["last_frontier_selection"]``，所以必须先结账。
        _settle_frontier_selection(state, spatial_memory, frontiers, current_position)

        # A frontier the route planner can reach is what makes further search
        # possible; visited/unreachable ones are already spent (计划书 §6.3).
        #
        # 2026-10-14 修正：``FrontierMemoryEntry`` **没有** ``status`` 字段，
        # 原来那句 ``getattr(value, "status", "OPEN")`` 于是恒返回 "OPEN" ——
        # visited/unreachable 永远是 0、reachable 恒等于前沿总数，上面那句注释
        # 描述的行为从未发生过。改成只由**正向证据**推导，且刻意不把
        # fail/blacklist 算成 UNREACHABLE：那是"我们试过但没成功"，不是"地图上
        # 不可达"；把它算成不可达会让 ``reachable_frontier_count`` 提前掉到 0，
        # 而那个数喂给 ``_classify_empty_candidates``，会把"卡住"报成"穷尽"。
        memory_status: dict[str, str] = {}
        if spatial_memory is not None:
            for key, entry in spatial_memory.frontiers.items():
                # ``mark_frontier_visited(failed=True)`` **也会**给 visit_count 加一，
                # 所以只认 visit_count>0 会把"试过但到不了"的前沿也算成 VISITED。
                # VISITED 必须同时要求 fail_count == 0（正向证据）。宁可少算
                # VISITED —— 那会让 reachable 偏大，偏差方向是"别过早宣布穷尽"。
                if int(getattr(entry, "visit_count", 0) or 0) > 0 and \
                        int(getattr(entry, "fail_count", 0) or 0) == 0:
                    memory_status[key] = "VISITED"
        diagnostics["visited_frontier_count"] = sum(
            1 for value in memory_status.values() if value == "VISITED"
        )
        diagnostics["unreachable_frontier_count"] = sum(
            1 for value in memory_status.values()
            if value in {"UNREACHABLE", "FAILED"}
        )
        diagnostics["reachable_frontier_count"] = sum(
            1 for item in frontiers
            if memory_status.get(item.frontier_id, "OPEN")
            not in {"VISITED", "UNREACHABLE", "FAILED"}
        )
        if metric_map is not None and metric_map.revision > 0:
            pose_source = getattr(camera_provider.get_pose(), "source", "none")
            node.get_logger().info(
                f"map source = {metric_map.source}, "
                f"map revision = {metric_map.revision}, "
                f"pose source = {pose_source}, "
                f"frontier count = {len(frontiers)}"
            )
            node.get_logger().info(
                f"route planner consumes {metric_map.source} "
                f"(revision {metric_map.revision})"
            )
        elif camera_provider is not None:
            # ROS 2 Foxy rclpy loggers take a single message string; printf-style
            # extra positional args raise TypeError (计划书 §6.1).
            node.get_logger().warning(
                f"metric map not fresh; falling back "
                f"(provider={args.spatial_provider}, "
                f"frontier count={len(frontiers)})"
            )

        semantic = state.get("semantic")
        scene_graph = getattr(semantic, "scene_graph", None) if semantic is not None else None
        psg_prior = (
            psg_provider.predict(
                controller.goal_graph,
                scene_graph,
                camera_provider,
                semantic_map,
            )
            if psg_provider is not None else None
        )
        if psg_prior is not None:
            for region in psg_prior.region_hypotheses:
                if (
                    region.state == "REJECTED"
                    or not region.metric_claim
                    or region.center is None
                ):
                    continue
                dx = region.center[0] - current_position[0] if current_position else 0.0
                dy = region.center[1] - current_position[1] if current_position else 0.0
                frontiers.append(FrontierCandidate(
                    frontier_id=f"semantic_{region.region_id}",
                    position=tuple(region.center),
                    frame="map",
                    bearing_deg=(
                        math.degrees(math.atan2(dy, dx))
                        if current_position is not None else None
                    ),
                    distance_m=(
                        math.hypot(dx, dy) if current_position is not None else None
                    ),
                    size_score=float(region.confidence),
                    spatial_information_gain=0.35,
                    reachable=True,
                    nearby_semantics=[region.relation, region.anchor_object_id],
                    provenance={
                        "source": "semantic_region",
                        "region": region.to_dict(),
                        "reachability_checked": False,
                    },
                ))
        diagnostics["frontier_count"] = len(frontiers)
        frontier_memory = (
            {key: value.to_dict() for key, value in spatial_memory.frontiers.items()}
            if spatial_memory is not None else {}
        )
        if args.backend == 'nav2' and os.getenv('GO2W_NAV2_TRIAL_LEDGER', '').strip():
            from app.navigation.nav2_candidate_geometry import filter_frontiers_for_heading
            heading_limit = min(20. if getattr(node,'_experiment_field_search',False)
                                else 10., float(node._experiment_max_turn_deg))
            frontiers, heading_rejected = filter_frontiers_for_heading(
                frontiers, current_position, current_yaw_deg,
                maximum_turn_deg=heading_limit)
            # Keep the original discovered/reachable counts above: a frontier
            # outside this trial's scope is not explored or unreachable.
            diagnostics['trial_eligible_frontier_count'] = len(frontiers)
            diagnostics['trial_heading_rejected'] = heading_rejected
            node._write({'event': 'trial_frontier_heading_filter',
                'maximum_turn_deg': heading_limit,
                'eligible_ids': [f.frontier_id for f in frontiers],
                'rejected': heading_rejected, 'host_s': node._host_s()})
            if not frontiers:
                diagnostics['empty_reason'] = 'no_frontier_within_trial_heading'
                return []
        # Build route costs for every frontier through the shared route
        # planner so the LongTermGoalSelector sees real path cost / reachability.
        route_costs: dict[str, dict[str, Any]] = {}
        if route_planner is not None and camera_provider is not None:
            rt_pose = pose or camera_provider.get_pose() or state.get("spatial_pose")
            for f in frontiers:
                rp = route_planner.plan(
                    start_pose=rt_pose if rt_pose is not None else None,
                    target_type="FRONTIER_CANDIDATE",
                    target_id=f.frontier_id,
                    target_position=f.position,
                    map_snapshot=(
                        camera_provider.get_map()
                        if hasattr(camera_provider, "get_map") else None
                    ),
                    place_graph=place_graph,
                    object_map=semantic_map,
                    frame_id="map",
                )
                if rp is not None:
                    route_plan_dict = rp.to_dict()
                    # Raw frontiers normally sit on the known/unknown boundary.
                    # A full-grid route to that boundary may be unavailable even
                    # though a 1.5--3 m observation prefix in the same direction
                    # is entirely inside observed free space.  Candidate
                    # admission must describe the strategic sub-goal that Nav2
                    # will actually receive, not the distant boundary cell.
                    if (
                        not route_plan_dict.get("reachable")
                        and args.backend == "nav2"
                        and rt_pose is not None
                        and metric_map is not None
                    ):
                        from app.navigation.nav2_candidate_geometry import (
                            nearer_frontier_goal,
                        )

                        source = str(
                            (f.provenance or {}).get("source") or "frontier"
                        )
                        probe = ExplorationGoal(
                            goal_id=f"reachability_{f.frontier_id}",
                            goal_type="NAVIGATE_POSE",
                            position=f.position,
                            yaw=f.bearing_deg,
                            frame="map",
                            provenance={
                                "intent": {
                                    "intent_type": "EXPLORE_FRONTIER",
                                    "preferred_position": list(f.position),
                                    "provenance": {"source": source},
                                }
                            },
                        )
                        safe_prefix = nearer_frontier_goal(
                            probe,
                            None,
                            metric_map,
                            maximum_step_m=float(
                                getattr(
                                    node,
                                    "_experiment_forward_step_m",
                                    args.forward_step_m,
                                )
                            ),
                            current_position=(rt_pose.x, rt_pose.y),
                            current_yaw_deg=current_yaw_deg,
                            maximum_turn_deg=float(
                                getattr(
                                    node,
                                    "_experiment_max_turn_deg",
                                    args.max_turn_deg,
                                )
                            ),
                        )
                        prefix_proof = safe_prefix.provenance.get(
                            "nearer_frontier_goal"
                        )
                        if prefix_proof and safe_prefix.position is not None:
                            prefix_length = math.dist(
                                (rt_pose.x, rt_pose.y), safe_prefix.position
                            )
                            route_plan_dict = {
                                **route_plan_dict,
                                "waypoints": [],
                                "path_length_m": round(prefix_length, 3),
                                "reachable": True,
                                "planner_source": "observed_map_safe_prefix",
                                "cost_components": {
                                    "path_length_m": round(prefix_length, 3),
                                    "frontier_distance_m": round(
                                        math.dist(
                                            (rt_pose.x, rt_pose.y), f.position
                                        ),
                                        3,
                                    ),
                                },
                                "provenance": {
                                    **dict(route_plan_dict.get("provenance") or {}),
                                    "candidate_reachability": (
                                        "observed_free_safe_prefix"
                                    ),
                                    "safe_prefix_position": list(
                                        safe_prefix.position
                                    ),
                                    "safe_prefix_proof": prefix_proof,
                                },
                            }
                    route_costs[f.frontier_id] = route_plan_dict
                    if entity_graph is not None:
                        entity_graph.set_route_plan(route_plan_dict)

        # Semantic relevance is measured from the current persistent object
        # map, explicit relation anchors and place observations.  The legacy
        # placeholder passed 0.0 for every frontier, so topology could never
        # affect the actual selector even though the UI displayed it.
        semantic_relevance, semantic_relevance_evidence = score_frontier_semantics(
            frontiers,
            target_profile=controller.target_profile,
            semantic_map=semantic_map,
            place_graph=place_graph,
            route_costs=route_costs,
            current_position=current_position,
        )
        diagnostics["semantic_relevance"] = {
            frontier_id: evidence.to_dict()
            for frontier_id, evidence in semantic_relevance_evidence.items()
        }
        node._write({
            "event": "frontier_semantic_relevance",
            "scores": semantic_relevance,
            "evidence": diagnostics["semantic_relevance"],
            "map_revision": diagnostics.get("map_revision"),
            "host_s": node._host_s(),
        })

        map_provenance = dict(getattr(metric_map, "provenance", {}) or {})
        runtime_epoch = state.setdefault(
            "map_epoch", f"runtime_{int(time.time() * 1000)}"
        )
        revisions = WorldRevisions(
            map_id=str(map_provenance.get("map_id") or "runtime_map"),
            map_epoch=str(map_provenance.get("map_epoch") or runtime_epoch),
            graph_revision=int(map_provenance.get("graph_revision") or 0),
            grid_revision=int(getattr(metric_map, "revision", 0) or 0),
            semantic_revision=int(getattr(semantic_map, "revision", 0) or 0),
            frame_id=str(map_provenance.get("frame_id") or "map"),
            sensor_profile_id=str(
                map_provenance.get("sensor_profile_id")
                or os.getenv("GO2W_GEOMETRY_PROFILE_SHA256", "unknown")
            ),
        )
        candidate_audit = []
        for frontier in frontiers:
            frontier_id = str(frontier.frontier_id)
            candidate_audit.append({
                "id": frontier_id,
                "type": str(
                    (frontier.provenance or {}).get("source") or "frontier"
                ).upper(),
                "frontier": frontier.to_dict(),
                "score_components": semantic_relevance_evidence[frontier_id].to_dict(),
                "planned_path_cost": (route_costs.get(frontier_id) or {}).get(
                    "path_length_m"
                ),
                "reachability_status": (
                    "REACHABLE"
                    if (route_costs.get(frontier_id) or {}).get("reachable") is True
                    else (
                        "UNREACHABLE"
                        if route_costs.get(frontier_id) else "UNKNOWN"
                    )
                ),
                "rejected_reasons": (
                    [] if (route_costs.get(frontier_id) or {}).get("reachable") is not False
                    else ["route_planner_unreachable"]
                ),
            })
        provider_health = (
            camera_provider.health()
            if camera_provider is not None and hasattr(camera_provider, "health")
            else {}
        )
        world_snapshot = build_world_state_snapshot(
            revisions=revisions,
            task=dict(agent_task_context or {}),
            robot={
                "position": list(current_position) if current_position is not None else None,
                "yaw_deg": current_yaw_deg,
                "spatial_quality": getattr(
                    camera_provider, "quality", lambda: "unavailable"
                )(),
            },
            map_summary={
                "source": getattr(metric_map, "source", None),
                "resolution_m": getattr(metric_map, "resolution_m", None),
                "width": getattr(metric_map, "width", None),
                "height": getattr(metric_map, "height", None),
                "free_cells": len(getattr(metric_map, "free", []) or []),
                "occupied_cells": len(getattr(metric_map, "occupied", []) or []),
                "unknown_cells": len(getattr(metric_map, "unknown", []) or []),
                "quality": provider_health.get("mapping_quality"),
            },
            objects=(
                [entry.to_dict() for entry in semantic_map.objects.values()]
                if semantic_map is not None else []
            ),
            topology=place_graph.to_dict() if place_graph is not None else {},
            search_memory={"frontiers": frontier_memory},
            candidates=candidate_audit,
            feedback={"last_decision": state.get("last_decision")},
            health={
                "input_health": provider_health,
                "geometry_health": observation.sensor_health if observation is not None else {},
                "localization_health": provider_health.get("localization_valid"),
                "mapping_quality": provider_health.get(
                    "mapping_quality",
                    {"state": "UNAVAILABLE", "map_fresh": diagnostics.get("map_fresh")},
                ),
                "motion_authorized": os.getenv(
                    "GO2W_NAV2_EXECUTION_AUTHORIZED", "0"
                ) == "1",
            },
        )
        state["world_state_snapshot"] = world_snapshot.to_dict()
        state["candidate_audit"] = candidate_audit
        node._write({
            "event": "world_state_snapshot",
            "snapshot": world_snapshot.to_dict(),
            "host_s": node._host_s(),
        })

        # Memory context: observation memory + in-session frontier memory.
        memory_context: dict[str, Any] = {"frontier_priors": {}, "observation_priors": {}}
        for fid, fm in frontier_memory.items():
            # 键名修正 2026-10-14：``frontier_memory`` 来自
            # ``SpatialMemory.frontiers[*].to_dict()``，里面写的是
            # ``semantic_gain_after_visit``；原来读的 ``semantic_prior`` /
            # ``semantic_score`` 两个键**在 to_dict 里都不存在**，于是这个
            # "长期记忆先验"恒为 0（静默 no-op：写端写了、读端读了个空键）。
            #
            # **但键名修对之后它仍然是恒 0 的**：全仓库两个
            # ``mark_frontier_visited`` 调用点（``_settle_frontier_selection``
            # 里那两处）都**不传 gain**，于是 ``semantic_gain_after_visit``
            # 只剩 dataclass 默认值 0.0 —— 产物里的实证是 64 处记录**全是**
            # 0.0。这一项要么有个真值、要么该删；现在两不靠，先如实记在这里。
            # 要让它活起来得先在结算时攒出"这次尝试换到了多少新语义"，那是新
            # 功能（而且"全局新增物体数"那种代理量会把这个增量错记到当前前沿
            # 头上），**没做**。
            memory_context["frontier_priors"][fid] = float(
                fm.get("semantic_gain_after_visit", 0.0) or 0.0
            )
        if semantic_memory is not None:
            long_term = semantic_memory.retrieve_long_term(args.target, top_k=5)
            for item in long_term:
                # Observation memory does not know frontier ids; expose a
                # conservative place-level prior for the current Place.
                memory_context["observation_priors"] = {
                    fid: min(1.0, float(item.get("target_related", False) or 0.0) * 0.5)
                    for fid in (f.frontier_id for f in frontiers)
                }

        # Structured common-sense prior; never allowed to confirm targets or
        # command motion.  Refresh in background (non-blocking) with a TTL so
        # it does not sit on the motion critical path.
        common_sense: dict[str, Any] = state.get("common_sense") or {}
        prior_ttl = 30.0
        now_ts = time.time()
        if (
            settings.llm_commonsense_prior_enabled
            and (
                not common_sense
                or (now_ts - float(state.get("common_sense_updated_ts", 0.0))) >= prior_ttl
            )
            and not bool(state.get("common_sense_pending"))
        ):
            state["common_sense_pending"] = True
            _semantic_summary = (
                semantic.get("scene_summary", "")
                if isinstance(semantic, dict)
                else str(getattr(semantic, "scene_summary", "") or "")
            )
            prior_input = LLMPriorInput(
                target=args.target,
                scene_summary=str(_semantic_summary),
                observed_objects=(
                    observation.scene_objects if observation is not None else []
                ),
                observed_relations=(
                    observation.scene_relations if observation is not None else []
                ),
                robot_capabilities={
                    "motion_primitives": list(getattr(capabilities, "allowed_motion_primitives", []))
                },
            )

            def _refresh_prior() -> None:
                try:
                    new_prior = llm_prior_generator.generate(prior_input)
                    state["common_sense"] = new_prior
                    state["common_sense_updated_ts"] = time.time()
                except Exception as exc:  # noqa: BLE001 - prior failure is non-fatal
                    state["common_sense_error"] = f"{type(exc).__name__}: {exc}"
                finally:
                    state["common_sense_pending"] = False

            threading.Thread(target=_refresh_prior, daemon=True).start()

        selector_kwargs = {
            "match_state": str(match_state).replace("_match", "").upper(),
            "place_graph": place_graph,
            "psg_prior": psg_prior,
            "frontier_memory": frontier_memory,
            "route_costs": route_costs,
            "semantic_relevance": semantic_relevance,
            "current_yaw_deg": current_yaw_deg,
            "memory_context": memory_context,
            "common_sense": common_sense,
        }
        ranked_candidates = spatial_reasoner.selector.rank_frontiers(
            frontiers=[
                item for item in frontiers
                if item.reachable
                and (route_costs.get(item.frontier_id) or {}).get(
                    "reachable", True
                ) is not False
            ],
            **selector_kwargs,
        )
        ordered_candidate_ids = [
            item.intent.target_frontier_id for item in ranked_candidates
            if item.intent.target_frontier_id
        ]
        active_candidate_id = state.get("active_strategic_candidate_id")
        strategic_candidate_id = (
            str(active_candidate_id)
            if active_candidate_id in ordered_candidate_ids else None
        )
        if strategic_candidate_id is None and ordered_candidate_ids:
            from app.navigation.strategic_vlm import constrain_strategic_decision

            detection_summaries = []
            for item in list(getattr(observation, "target_candidates", None) or []):
                detection_summaries.append({
                    "id": entity_id_for(item),
                    "label": item.get("label"),
                    "score": item.get("score"),
                })
            by_id = {item.frontier_id: item for item in frontiers}
            candidate_summaries = []
            for candidate_id in ordered_candidate_ids:
                candidate = by_id[candidate_id]
                route_info = route_costs.get(candidate_id) or {}
                candidate_summaries.append({
                    "id": candidate_id,
                    "source": (candidate.provenance or {}).get(
                        "source", "frontier"
                    ),
                    "relative_direction_deg": (
                        None if candidate.bearing_deg is None else round(
                            math.remainder(
                                candidate.bearing_deg - current_yaw_deg, 360.0
                            ), 1
                        )
                    ),
                    "distance_m": candidate.distance_m,
                    "path_length_m": route_info.get("path_length_m"),
                })
            strategic_payload = node._daemon_vlm_request(
                "strategic",
                str(
                    getattr(observation, "image_ref", "")
                    or state.get("image_path") or ""
                ),
                args.target,
                frame_id=str(getattr(observation, "bundle_id", "")),
                model=vision_model,
                timeout=min(
                    45.0, float(settings.vlm_runtime_semantic_timeout_seconds)
                ),
                extra_instructions=json.dumps({
                    "target_description": controller.target_profile.prompt_context(),
                    "local_detections": detection_summaries,
                    "reachable_candidates": candidate_summaries,
                    "visited_or_failed": [
                        {
                            "id": key,
                            "visit_count": value.get("visit_count", 0),
                            "fail_count": value.get("fail_count", 0),
                        }
                        for key, value in frontier_memory.items()
                        if value.get("visit_count") or value.get("fail_count")
                    ][:8],
                }, ensure_ascii=False),
            )
            strategic = constrain_strategic_decision(
                strategic_payload,
                detection_ids=[item["id"] for item in detection_summaries],
                candidate_ids=ordered_candidate_ids,
            )
            node._write({
                "event": "strategic_vlm_decision",
                **strategic.to_dict(),
                "candidate_count": len(ordered_candidate_ids),
                "detector_backend": getattr(node, "_detector", "llm"),
                "host_s": node._host_s(),
            })
            if strategic.target_state in {"present", "possible"}:
                state["detection_event"] = "target_candidate"
                diagnostics["empty_reason"] = (
                    "strategic_target_possible_reobserve"
                )
                return []
            strategic_candidate_id = strategic.selected_search_candidate_id
            if strategic_candidate_id is not None:
                state["active_strategic_candidate_id"] = strategic_candidate_id

        scored = spatial_reasoner.propose(
            graph_match=graph_match,
            frontiers=frontiers,
            place_graph=place_graph,
            semantic_map=semantic_map,
            target_profile=controller.target_profile,
            observation_id=getattr(observation, "rgbd_frame_id", None),
            psg_prior=psg_prior,
            frontier_memory=frontier_memory,
            route_costs=route_costs,
            semantic_relevance=semantic_relevance,
            current_yaw_deg=current_yaw_deg,
            current_position=current_position,
            memory_context=memory_context,
            common_sense=common_sense,
            strategic_candidate_id=strategic_candidate_id,
        )
        if scored is None:
            diagnostics["empty_reason"] = "spatial_reasoner_no_intent"
            return []
        intent = scored.intent
        if intent.target_frontier_id and spatial_memory is not None:
            spatial_memory.mark_frontier_selected(intent.target_frontier_id)
            # 记下这次选择，供**下一次**生成器调用开头的
            # ``_settle_frontier_selection`` 结账（那时才知道这次到底走到没有）。
            # 同一条前沿连续被选中时把尝试计数接续上来，否则"规划了三次还是
            # 一点没挪"永远攒不满。
            previous = state.get("last_frontier_selection")
            same = (
                isinstance(previous, dict)
                and previous.get("frontier_id") == intent.target_frontier_id
            )
            state["last_frontier_selection"] = {
                "frontier_id": intent.target_frontier_id,
                "pose": current_position,
                "attempts": int(previous.get("attempts") or 0) if same else 0,
            }
        goal = None
        route = None
        current_place = place_graph.current_place()
        if intent.target_frontier_id and current_place is not None:
            from app.spatial.topological_frontier import TopologicalFrontier

            current_place_id = current_place.place_id
            tf = next(
                (
                    TopologicalFrontier(
                        frontier_id=f.frontier_id,
                        parent_place_id=current_place_id,
                        bearing_deg=float(f.bearing_deg or 0.0),
                        local_distance_hint_m=f.distance_m,
                        information_gain=float(f.spatial_information_gain or 0.0),
                    )
                    for f in frontiers
                    if f.frontier_id == intent.target_frontier_id
                ),
                None,
            )
            if tf is not None:
                route = TopologyRoutePlanner().plan(
                    place_graph=place_graph,
                    current_place_id=current_place_id,
                    frontier=tf,
                )
            if route is not None:
                goal = TopologyRouteExecutor(
                    max_metric_step_m=float(getattr(node, "_experiment_forward_step_m", args.forward_step_m)),
                    forward_step_m=float(
                        getattr(node, "_experiment_forward_step_m", 0.30)
                    )
                ).next_goal(
                    route=route,
                    current_place_id=current_place_id,
                    place_graph=place_graph,
                    current_yaw_deg=current_yaw_deg,
                    capabilities=capabilities,
                    frontier=tf,
                    current_position=current_position,
                )
        if goal is None:
            local_executor.begin(intent)
            goal = local_executor.next_goal(
                current_yaw_deg=current_yaw_deg,
                current_position=current_position,
                capabilities=capabilities,
            )
            if goal is None:
                local_executor.finish()
                diagnostics["empty_reason"] = "local_executor_no_primitive"
                diagnostics["intent_type"] = str(getattr(intent, "intent_type", ""))
                diagnostics["intent_had_preferred_position"] = (
                    getattr(intent, "preferred_position", None) is not None
                )
                return []
        goal.semantic_relevance = max(0.0, min(1.0, scored.score))
        goal.expected_information_gain = intent.spatial_gain
        goal.provenance = {
            **goal.provenance,
            "intent": intent.to_dict(),
            "scored_intent": scored.to_dict(),
            "route": route.to_dict() if route is not None else None,
            "route_plan": route_costs.get(intent.target_frontier_id)
            if intent.target_frontier_id else None,
            "snapshot_id": world_snapshot.snapshot_id,
            **revisions.to_dict(),
        }
        if args.backend == 'nav2' and camera_provider is not None:
            from app.navigation.nav2_candidate_geometry import nearer_frontier_goal
            candidate_pose = camera_provider.get_pose()
            goal = nearer_frontier_goal(goal, goal.provenance.get('route_plan'),
                camera_provider.get_map(), maximum_step_m=float(
                    getattr(node, '_experiment_forward_step_m', args.forward_step_m)),
                current_position=((candidate_pose.x,candidate_pose.y) if candidate_pose else None),
                current_yaw_deg=current_yaw_deg,
                maximum_turn_deg=float(
                    getattr(node, '_experiment_max_turn_deg', args.max_turn_deg)))
        strategic_goal_distance_m = None
        if current_position is not None and goal.position is not None:
            strategic_goal_distance_m = math.dist(current_position, goal.position)
        goal.provenance = {
            **goal.provenance,
            "strategic_candidate_id": intent.target_frontier_id,
            "strategic_candidate_source": (
                intent.provenance.get("source")
                if isinstance(intent.provenance, dict) else "frontier"
            ),
            "strategic_goal_distance_m": strategic_goal_distance_m,
            "vlm_reason": (
                strategic.reason
                if "strategic" in locals() else "deterministic retained intent"
            ),
        }
        # Publish real long-term goal selection + decision record.
        if entity_graph is not None:
            selected_route = route_costs.get(intent.target_frontier_id)
            if selected_route:
                entity_graph.set_route_plan(selected_route)
        _publish_decision_record(
            cycle=state.get("cycle", 0),
            match_state=match_state,
            scored=scored,
            goal=goal,
            intent=intent,
            observation=observation,
            route_plan=route_costs.get(intent.target_frontier_id),
        )
        return [goal]

    def _publish_decision_record(
        *,
        cycle: int,
        match_state: str,
        scored: Any,
        goal: Any,
        intent: Any,
        observation: Any,
        route_plan: dict[str, Any] | None,
    ) -> None:
        """Build and emit a real structured DecisionRecord (plan §10)."""
        from app.navigation.decision_record import (
            alternative_from_candidate,
            build_decision_record,
        )

        breakdown = dict(scored.components or {})
        matched_normalized = {
            "ZERO": "ZERO", "PARTIAL": "PARTIAL", "STRONG": "STRONG",
            "VERIFY": "VERIFY",
        }.get(str(match_state).upper(), str(match_state).upper())
        record = build_decision_record(
            cycle=int(cycle),
            match_state=matched_normalized,
            selected_intent=intent.to_dict(),
            selected_goal=goal.to_dict(),
            next_motion_command={"instruction_zh": goal.semantic_reason},
            score=float(scored.score),
            score_breakdown=breakdown,
            evidence={
                "anchor_object_ids": [
                    entry.object_id for entry in (semantic_map.objects.values() if semantic_map else [])
                    if getattr(entry, "status", "") == "CONFIRMED"
                ][:3],
                "anchor_labels": getattr(observation, "scene_objects", None)
                and [
                    str(item.get("label_zh") or item.get("label") or "")
                    for item in observation.scene_objects[:3]
                ]
                or [],
                "current_place_id": place_graph.current_place().place_id
                if place_graph and place_graph.current_place() else None,
                "spatial_quality": getattr(camera_provider, "quality", lambda: "CAMERA_LOCAL")(),
                "route_id": (route_plan or {}).get("route_id"),
            },
            alternatives=[],
            map_revision=entity_graph.revision if entity_graph is not None else 0,
            canonical_target=args.target,
            task_text=args.target,
            current_place_id=place_graph.current_place().place_id
            if place_graph and place_graph.current_place() else None,
            session_id=args.session_id or "",
            task_id=str((agent_task_context or {}).get("task_id") or "") or None,
            snapshot_id=(state.get("world_state_snapshot") or {}).get("snapshot_id"),
            map_id=(state.get("world_state_snapshot") or {}).get("map_id"),
            map_epoch=(state.get("world_state_snapshot") or {}).get("map_epoch"),
            graph_revision=(state.get("world_state_snapshot") or {}).get("graph_revision"),
            grid_revision=(state.get("world_state_snapshot") or {}).get("grid_revision"),
            semantic_revision=(state.get("world_state_snapshot") or {}).get("semantic_revision"),
            sensor_profile_id=(state.get("world_state_snapshot") or {}).get(
                "sensor_profile_id"
            ),
            source_observation_ids=[str(getattr(observation, "bundle_id", ""))],
            evidence_age_ms=getattr(observation, "semantic_age_ms", None),
            candidates=list(state.get("candidate_audit") or []),
            selected_candidate_id=getattr(intent, "target_frontier_id", None),
            downstream_selected_id=getattr(goal, "goal_id", None),
        )
        state["last_decision"] = record.to_dict()
        node._write({
            "event": "long_term_goal_selected",
            "intent": intent.to_dict(),
            "route_plan": route_plan,
            "scored": scored.to_dict(),
            "host_s": node._host_s(),
        })
        node._write({
            "event": "decision_recorded",
            "decision": record.to_dict(),
            "host_s": node._host_s(),
        })
        node._write({
            "event": "local_goal_progress",
            "progress": {
                "goal_id": goal.goal_id,
                "goal_type": goal.goal_type,
                "decision_id": record.decision_id,
            },
            "host_s": node._host_s(),
        })

    def spatial_planner(candidates: list[Any], **kwargs: Any) -> Any:
        """Real explainable planner: score the produced goal from the intent.

        The score and components come from the LongTermGoalSelector's real
        ScoredIntent rather than a hard-coded ``spatial_v2=1.0``.
        """
        from app.navigation.exploration_planner import ScoredGoal

        if not candidates:
            return None
        goal = candidates[0]
        prov = dict(goal.provenance or {})
        scored_intent = prov.get("scored_intent") or {}
        score = float(scored_intent.get("score", 0.0) or 0.0)
        components = dict(scored_intent.get("components", {}) or {})
        reasons = list(scored_intent.get("reasons", []) or [])
        if not reasons:
            reasons = [goal.semantic_reason or "semantic navigation intent"]
        return ScoredGoal(
            goal=goal,
            score=score if score > 0 else 0.5,
            components=components if components else {"semantic_relevance": score or 0.5},
            reasons=reasons,
        )

    # ---- backend ---------------------------------------------------------------
    from app.navigation.go2w_experimental_backend import (
        Go2WBackendConfig,
        Go2WExperimentalBackend,
    )
    step_index = [0]

    def execute_step(step: str) -> tuple[bool, str, dict[str, Any]]:
        if args.dry_run_motion:
            # WebUI dry-run: run the full real perception / reasoning pipeline
            # but never send a motion command (plan book §60).
            return True, "dry_run_motion", {"step": step, "dry_run": True}
        index = step_index[0]
        step_index[0] += 1
        before = node._odom_snapshot()
        ok, reason = node._execute_step(
            index, step, attempts=1 if (
                args.max_navigation_goals > 0 or args.max_motion_commands > 0
            ) else 2,
        )
        detail = {"step": step, "index": index}
        reason_code = str(reason or "").split(":", 1)[0].strip().upper()
        if reason_code in {"MOTION_ACCEPT_TIMEOUT", "MOTION_RESULT_TIMEOUT"}:
            detail["error_type"] = reason_code
            detail["non_retryable"] = True
        if ok:
            after = node._odom_snapshot()
            wheel_delta_xy = math.hypot(after[0] - before[0], after[1] - before[1])
            requested_dyaw = _requested_dyaw_deg(step)
            motion_state["pending_motion_evidence"] = MotionEvidence(
                command_type=("ROTATE" if abs(requested_dyaw) > 0.0 else "TRANSLATE"),
                requested_turn_deg=requested_dyaw,
                requested_forward_m=_requested_forward_m(step),
                wheel_delta_xy_m=wheel_delta_xy,
                wheel_delta_yaw_deg=math.degrees(_wrap_pi(after[2] - before[2])),
                motion_completed_at=time.time(),
            )
            motion_state["motion_end_timestamp"] = time.time()
            motion_state["last_frame_id_before_motion"] = str(
                state.get("frame_id") or ""
            )
            motion_state["last_yaw_before"] = float(before[2])
            # 计划书 §7.5：用运动后的真实 odom 计算 reached sector，绝不拿
            # “计划想去的 sector”冒充“实际到达 sector”。
            sectors = max(1, int(policy.candidates.heading_sectors))
            observed_dyaw = math.degrees(_wrap_pi(after[2] - before[2]))
            sector_before = _navigation_sector(yaw_rad=before[2], sectors=sectors)
            sector_reached = _navigation_sector(yaw_rad=after[2], sectors=sectors)
            current_place = (
                place_graph.current_place() if place_graph is not None else None
            )
            node._write({
                "event": "motion_sector_reached",
                "step": step,
                "requested_dyaw_deg": round(requested_dyaw, 3),
                "observed_dyaw_deg": round(observed_dyaw, 3),
                "wheel_delta_xy_m": round(wheel_delta_xy, 4),
                "sector_before": sector_before,
                "sector_requested": _navigation_sector(
                    yaw_rad=before[2] + math.radians(requested_dyaw),
                    sectors=sectors,
                ),
                "sector_reached": sector_reached,
                "coverage_before": (
                    dict(current_place.heading_coverage or {})
                    if current_place is not None else {}
                ),
                "coverage_after": "next_observation",
                "host_s": node._host_s(),
            })
        return ok, reason, detail

    def stop() -> None:
        node._emergency_stop()

    def cancel() -> None:
        node._emergency_stop()

    def health_probe() -> dict[str, Any]:
        try:
            motion_ready = bool(node._client.server_is_ready())
        except Exception:
            motion_ready = False
        try:
            arm_ready = bool(node._arm_client.service_is_ready())
            stop_ready = bool(node._stop_srv.service_is_ready())
        except Exception:
            arm_ready = stop_ready = False
        mode_ok = bool(
            node._sport is not None
            and int(node._sport.mode) == 1
            and int(node._sport.error_code) == 0
        )
        action_server_count, action_graph_detail = _motion_action_server_count()
        action_process_count, action_process_pids = (
            _motion_action_server_process_count()
        )
        odom_publisher_count, odom_graph_detail = _topic_publisher_count(
            args.odom_topic
        )
        odom_process_count, odom_process_pids = _wheel_odom_process_count()
        return {
            "motion_action_available": motion_ready,
            "motion_action_server_count": action_server_count,
            "motion_action_graph_detail": action_graph_detail,
            "motion_action_server_process_count": action_process_count,
            "motion_action_server_pids": action_process_pids,
            "odom_topic": args.odom_topic,
            "odom_publisher_count": odom_publisher_count,
            "odom_graph_detail": odom_graph_detail,
            "wheel_odom_process_count": odom_process_count,
            "wheel_odom_process_pids": odom_process_pids,
            "arm_service_available": arm_ready,
            "emergency_stop_service_available": stop_ready,
            "robot_mode_error": not mode_ok,
            "lidar_fresh": node._lidar_fresh is True,
            "operator_authorized_rotation": operator_authorized,
            "experiment_profile": getattr(
                node, "_experiment_profile_name", "operator_supervised_experiment"
            ),
            "allowed_motion_primitives": [
                name for name, enabled in getattr(
                    node, "_experiment_motion_primitives", {}
                ).items() if enabled
            ],
        }

    if args.backend == "nav2":
        from app.navigation.nav2_search_backend import (
            REJECT_NO_LOCALIZATION,
            REJECT_NO_MAP,
            REJECT_NAV2_NOT_READY,
            create_nav2_search_backend,
        )
        import threading
        from rclpy.executors import SingleThreadedExecutor
        nav2_io_node = rclpy.create_node("go2w_nav2_search_io")
        from std_srvs.srv import Trigger
        from app.navigation.motion_authorization import arm_with_arbiter_reset
        arbiter_reset_client=nav2_io_node.create_client(Trigger,'/go2w/control_arbiter/reset_emergency_stop')

        def reset_arbiter_after_arm():
            deadline=time.monotonic()+1.5
            while time.monotonic()<deadline:
                future=arbiter_reset_client.call_async(Trigger.Request())
                while not future.done() and time.monotonic()<deadline:
                    time.sleep(.02)
                if not future.done():
                    future.cancel()
                    return False
                response=future.result()
                if response is not None and response.success:
                    return True
                time.sleep(.05)
            return False

        # 计划书 §6.1 / §6.6：真机自主搜索的唯一后端。运动全部由 Nav2 的
        # ComputePathToPose + NavigateToPose 产生，本脚本不再自己拼前进/转向。
        def nav2_motion_preflight():
            # Launcher authorization is process-owned; request JSON cannot supply it.
            if (args.dry_run_motion or os.getenv("GO2W_NAV2_EXECUTION_AUTHORIZED") != "1"
                    or not args.operator_supervised_experiment):
                return False
            ok, _ = _runner_start_safety(args, node)
            return bool(ok and arbiter_reset_client.service_is_ready())

        def authorize_nav2_motion():
            if not nav2_motion_preflight():
                return False
            return arm_with_arbiter_reset(arm=lambda:node._arm(True),
                is_armed=lambda:node._armed_by_runner, reset=reset_arbiter_after_arm,
                disarm=lambda:node._arm(False))

        def release_nav2_motion():
            """authorize 的对称半边（计划书 §P3-3）。取消/失败/超时之后没人收回授权。

            目标必须拥有这次 arm；trial 和累计行程 guard 的目标成功后也会
            调用，使下一目标重新握手。``node._armed_by_runner`` 为假时不发
            解除，避免解除其他操作员拥有的授权。

            **不抛**：调用点在"目标结束、准备结算结果"的路径上。这里抛出去会把一次
            正常的导航结算变成异常，而 ARM 是租约、自己会过期，收不回来的代价有界。
            """
            try:
                if node._armed_by_runner:
                    node._arm(False)
            except Exception as error:  # noqa: BLE001
                node.get_logger().warn(
                    "disarm after non-successful goal failed (arm is a lease, it "
                    "expires on its own): " + str(error))

        from app.navigation.trial_budget import NavigationTrialBudget
        trial_ledger = os.getenv("GO2W_NAV2_TRIAL_LEDGER", "").strip()
        trial_budget = NavigationTrialBudget(trial_ledger) if trial_ledger else None

        backend = _Nav2MotionEvidenceRecorder(
            create_nav2_search_backend(nav2_io_node, dry_run=bool(args.dry_run_motion),
                                       motion_authorizer=authorize_nav2_motion,
                                       motion_preflight=nav2_motion_preflight,
                                       motion_releaser=release_nav2_motion,
                                       trial_budget=trial_budget,
                                       # The selected trial envelope includes
                                       # NavFn's starting-cell quantization.
                                       # Capping path length at the requested
                                       # straight displacement rejects a valid
                                       # 0.45m goal whose actual path is 0.47m.
                                       max_path_length_m=(trial_budget.MAX_TRANSLATION_M-trial_budget.STOP_RESERVE_M
                                           if trial_budget else min(3., float(getattr(
                                               node, "_experiment_forward_step_m", args.forward_step_m))+.05))),
            node=node, motion_state=motion_state
        )
        node._search_nav2_backend = backend
        node._nav2_io_node = nav2_io_node
        node._nav2_io_executor = SingleThreadedExecutor()
        node._nav2_io_executor.add_node(nav2_io_node)
        node._nav2_io_thread = threading.Thread(target=node._nav2_io_executor.spin,
                                                daemon=True, name="nav2-search-io")
        node._nav2_io_thread.start()
        if not node._wait_for(lambda: backend.health().ready and backend.get_pose() is not None,
                             10., "Nav2 live map/TF/health inputs"):
            raise RuntimeError("Nav2 map/TF/health inputs did not become current")
        node._write({
            "event": "nav2_backend_selected",
            "detail": "自主搜索后端为 nav2；相对前进与实验后端不在本路径上",
            "reject_reasons": [
                REJECT_NAV2_NOT_READY,
                REJECT_NO_MAP,
                REJECT_NO_LOCALIZATION,
            ],
            "host_s": node._host_s(),
        })
    else:
        backend = Go2WExperimentalBackend(
            execute_step=execute_step,
            odometry=node._odom_snapshot,
            stop=stop,
            cancel=cancel,
            health_probe=health_probe,
            config=Go2WBackendConfig(
                dry_run=bool(args.dry_run_motion),
                max_turn_deg_per_action=getattr(
                    node, "_experiment_max_turn_deg", args.max_turn_deg
                ),
                forward_step_m=getattr(
                    node, "_experiment_forward_segment_m", 0.30
                ),
                max_forward_step_m=max(
                    0.01,
                    float(getattr(
                        node, "_experiment_forward_step_m", args.forward_step_m
                    )),
                ),
                max_observed_forward_m=args.max_observed_forward_m,
                max_motion_commands=args.max_motion_commands,
            ),
        )

    if args.record_video:
        try:
            from run_autonomous_loop import BundleVideoRecorder
            node._video = BundleVideoRecorder(args.record_video, 15.0, 0.4)
            node._video.start()
            node.get_logger().info(f"recording camera to {args.record_video}")
        except RuntimeError as exc:
            node.get_logger().warn(f"video recording disabled: {exc}")

    # ---- explorer ---------------------------------------------------------------
    from app.task_understanding.search_task_context import SearchTaskContext
    task_context = _task_context_from_args(args)
    graph = ExplorationGraph(session_id=args.session_id or time.strftime("explore_go2w_%Y%m%d_%H%M%S"))
    events: list[dict[str, Any]] = []
    holder: dict[str, Any] = {"explorer": None}

    def on_event(event: dict[str, Any]) -> None:
        if event.get("event") == "navigation_result" and not event.get("dry_run"):
            status = str(event.get("status") or "").upper()
            state["detection_event"] = (
                "arrival" if status == "SUCCEEDED" else "nav_failure"
            )
            state.pop("active_strategic_candidate_id", None)
        explorer = holder.get("explorer")
        if explorer is not None:
            # 与发给 Web 的语义图使用同一 revision。目标确认会在观察之后
            # 更新实体 provenance；若只在 observe() 里写，最终目标标记
            # 永远停留在确认前的快照。
            _publish_display_objects(
                state, args, node, explorer.semantic_graph.object_map,
                explorer.semantic_graph.revision,
            )
        if event_hook is not None:
            # 计划书 §3A：图不再挂在普通事件上。worker 的 event_hook 会在
            # revision 变化时单独发一条 graph_snapshot。
            event = event_hook(event, holder)
        if event is None:
            return
        events.append(event)
        node._write(event)

    def exhaustion_probe() -> dict[str, Any]:
        """Read-only map/frontier evidence for empty-candidate classification.

        计划书 §6.3：复用已有 frontier_extractor / spatial_reasoner /
        TopologyRoutePlanner 的结果，不建立第二套候选系统。
        """
        return dict(state.get("last_candidate_diagnostics") or {})

    explorer_kwargs: dict[str, Any] = {}

    def local_safety_provider(observation: LiveObservation):
        """Adapt the existing live LiDAR gate into the explorer safety view.

        The runner already subscribes to the validated preprocessor outputs;
        this adapter deliberately consumes only those values.  It does not
        infer rear/side geometry, so unknown sectors remain fail-closed.
        """
        front = node._clearance
        if front is None or not math.isfinite(float(front)):
            front_status = "no_return" if front is not None else "unknown"
            front_value = None
        else:
            front_status = "measured"
            front_value = float(front)
        clearance = {
            "front_status": front_status,
            "front_m": front_value,
            "left_status": "unknown",
            "right_status": "unknown",
            "lidar_fresh": node._lidar_fresh is True,
        }
        return local_safety_from_clearance(
            clearance,
            timestamp=float(observation.timestamp or time.time()),
            hard_clearance_m=search_policy.clearance_navigation.hard_clearance_m,
            rotation_clearance_m=search_policy.clearance_navigation.hard_clearance_m,
            geometry_validated=(
                node._lidar_fresh is True and front is not None
            ),
            pose_quality="relative",
            source_health={
                "source": "go2w_lidar_preprocessor",
                "rotation_clearance_valid": node._rotation_clearance_valid is True,
            },
        )

    def perception_quality_provider(
        observation: LiveObservation, information_gain: float
    ):
        target_item = next(
            (
                item for item in observation.scene_objects
                if isinstance(item, dict)
                and str(item.get("category") or "").lower() == "target"
            ),
            None,
        )
        measured_objects = list(observation.scene_objects)
        stable_count = len(observation.scene_objects)
        if observation.provenance.get('target_spatial_contract') == 'capture_v1':
            from app.perception.target_spatial import selected_current_target, validated_target_binding
            target_item = selected_current_target(observation)
            current_targets = [item for item in observation.target_candidates
                if item.get('entity_source_frame_id') == observation.provenance.get('current_capture_id')]
            measured_objects += current_targets
            if semantic_map is not None:
                stable_count += len({value for item in current_targets
                    if (value := validated_target_binding(item,semantic_map)) is not None})
        target_depth = (
            target_item.get("depth_m") if isinstance(target_item, dict) else None
        )
        target_depth_valid = isinstance(target_depth, (int, float)) and float(
            target_depth
        ) > 0.0
        depth_count = sum(
            1
            for item in measured_objects
            if isinstance(item, dict)
            and isinstance(item.get("depth_m"), (int, float))
            and float(item["depth_m"]) > 0.0
        )
        return evaluate_perception_quality(
            depth_valid_ratio=1.0 if depth_count else 0.0,
            visual_feature_count=len(measured_objects),
            stable_object_count=stable_count,
            estimated_information_gain=float(information_gain),
            front_clearance_m=(
                None
                if node._clearance is None or not math.isfinite(float(node._clearance))
                else float(node._clearance)
            ),
            target_visible=target_item is not None,
            target_depth_valid=target_depth_valid,
            camera_blocked_front_m=search_policy.camera_view.camera_blocked_front_m,
            near_field_ratio_threshold=(
                search_policy.camera_view.near_field_ratio_threshold
            ),
            low_information_threshold=search_policy.camera_view.low_information_threshold,
            near_field_threshold_m=search_policy.camera_view.near_field_threshold_m,
        )

    explorer_kwargs["local_safety_provider"] = local_safety_provider
    explorer_kwargs["perception_quality_provider"] = perception_quality_provider
    explorer_kwargs["decision_context_provider"] = lambda: state.get(
        "world_state_snapshot"
    )
    if (
        args.backend == "nav2"
        and is_local_structured_backend(getattr(node, "_detector", "llm"))
    ):
        explorer_kwargs["navigation_target_monitor"] = navigation_target_monitor
    if args.spatial_v2:
        explorer_kwargs["candidate_generator"] = spatial_candidate_generator
        explorer_kwargs["planner"] = spatial_planner
        explorer_kwargs["exhaustion_probe"] = exhaustion_probe

        def preflight_rejection_handler(goal, result) -> None:
            """Feed a concrete Nav2 route rejection back into frontier memory."""
            # Infrastructure/admission failures say nothing about the chosen
            # frontier.  Only concrete route geometry is valid negative memory.
            if (result.provenance or {}).get("reject_reason") not in {
                "TOO_NARROW", "NO_PATH", "GOAL_OCCUPIED"
            }:
                return
            provenance = goal.provenance if isinstance(goal.provenance, dict) else {}
            intent = provenance.get("intent")
            frontier_id = provenance.get("frontier_id")
            if frontier_id is None and isinstance(intent, dict):
                frontier_id = intent.get("target_frontier_id")
            if not frontier_id or spatial_memory is None:
                return
            frontier_id = str(frontier_id)
            spatial_memory.mark_frontier_preflight_rejected(frontier_id)
            previous = state.get("last_frontier_selection")
            if (
                isinstance(previous, dict)
                and str(previous.get("frontier_id")) == frontier_id
            ):
                # Explicit Nav2 evidence supersedes the delayed three-cycle
                # zero-motion heuristic; do not count the same rejection twice.
                state.pop("last_frontier_selection", None)
            state.setdefault("preflight_rejected_frontiers", []).append({
                "frontier_id": frontier_id,
                "goal_id": goal.goal_id,
                "reject_reason": (result.provenance or {}).get("reject_reason"),
            })

        explorer_kwargs["preflight_rejection_handler"] = (
            preflight_rejection_handler
        )
        # 2026-09-09 P2 修复：AutonomousExplorer 默认自建 SemanticNavigationGraph
        # （其 object_map 与 worker 的 semantic_map 是两套），于是 MAP_UPDATED 展示的
        # 实体永远停在 CAMERA_LOCAL，而 worker 那套其实已收到 METRIC_LIDAR 观测。
        # 这里把 worker 的 semantic_map 注入进去，让两者共用同一张对象表。
        # 2026-09-16 补上 place_graph（计划书 §7 的 P0）：当时只注入了 object_map，
        # 于是 Place 仍然是两套 —— explorer 自建的那张用 place_graph.py:49-50 的
        # 默认值（merge 0.6 / relocation 0.10，比 worker 的 0.35 / 0.50 松 5 倍）
        # 且拿不到 observed_displacement_m，而 WebUI 展示的 Place 正是从 explorer
        # 这张图来的（memory_update 的 semantic_navigation_graph 字段），planner 读的
        # 却是 worker 那张。同一个"当前所在的地点"因此有两个身份。
        _spatial_state = state.get("spatial_v2") or {}
        if _spatial_state.get("semantic_map") is not None:
            from app.spatial.semantic_navigation_graph import SemanticNavigationGraph

            explorer_kwargs["semantic_graph"] = SemanticNavigationGraph(
                object_map=_spatial_state["semantic_map"],
                place_graph=_spatial_state["place_graph"],
            )
    explorer = AutonomousExplorer(
        target=task_context.canonical_target,
        task_context=task_context,
        observer=observe,
        matcher=matcher,
        verifier=verifier,
        backend=backend,
        policy=policy,
        graph=graph,
        negative_memory=semantic_memory,
        negative_target_key=profile.canonical_name_zh,
        on_event=on_event,
        finish_on_visual_confirmation=args.finish_on_visual_confirmation,
        search_policy=search_policy,
        turn_only=bool(args.turn_only),
        # 干跑整场不发运动指令，motion_steps 必须如实为 0（计划书 §60）。
        dry_run=bool(args.dry_run_motion),
        session_id=graph.session_id,
        executor_id=args.executor_id or None,
        worker_generation=args.worker_generation,
        max_navigation_goal_attempts=args.max_navigation_goals,
        **explorer_kwargs,
    )
    holder["explorer"] = explorer
    node.get_logger().info(
        f"starting semantic exploration target={args.target} "
        f"session={graph.session_id}"
    )
    search_limit = float(policy.budget.max_search_seconds)
    node._semantic_session_deadline_monotonic = (
        time.monotonic() + search_limit if search_limit > 0.0 else None
    )
    try:
        result = explorer.run()
    finally:
        node._semantic_session_deadline_monotonic = None
    _publish_display_objects(
        state, args, node, explorer.semantic_graph.object_map,
        explorer.semantic_graph.revision,
    )
    if node._armed_by_runner:
        if args.backend == "nav2":
            backend.stop()
        else:
            node._emergency_stop()
        try:
            node._arm(False)
        except RuntimeError as exc:
            node.get_logger().error(str(exc))
    if camera_provider is not None:
        close = getattr(camera_provider, "close", None)
        if close is not None:
            close()
    # on_event 已经通过 node._write 把每条事件实时写进 args.output，这里再传
    # events 会以 "a" 模式把同一批事件追加第二遍：实测
    # search_20260902_211913 的 events.jsonl 里每条事件都出现两次，按事件计数
    # 的取证（decision_recorded 8 条 vs decisions.jsonl 4 行）会直接翻倍。
    _write_session_artifacts(args, explorer, result, [])
    if place_graph is not None:
        run_dir = Path(args.session_dir) / explorer.session_id
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / "place_graph.json").write_text(
            json.dumps(place_graph.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        if semantic_map is not None:
            (run_dir / "semantic_map.json").write_text(
                json.dumps(semantic_map.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
        if spatial_memory is not None:
            (run_dir / "spatial_memory.json").write_text(
                json.dumps(spatial_memory.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
    # Bind the map identity, revisions, task, decisions, topology and semantic
    # payloads into one integrity-checked bundle.  Loading code can now reject
    # an A-map/B-object mixture instead of trusting matching frame names.
    run_dir = Path(args.session_dir) / explorer.session_id
    snapshot_value = state.get("world_state_snapshot") or {}
    bundle_revisions = WorldRevisions(
        map_id=str(snapshot_value.get("map_id") or "runtime_map"),
        map_epoch=str(
            snapshot_value.get("map_epoch")
            or state.get("map_epoch")
            or f"session_{explorer.session_id}"
        ),
        graph_revision=int(snapshot_value.get("graph_revision") or 0),
        grid_revision=int(snapshot_value.get("grid_revision") or 0),
        semantic_revision=int(
            snapshot_value.get("semantic_revision")
            or getattr(semantic_map, "revision", 0)
            or 0
        ),
        frame_id=str(snapshot_value.get("frame_id") or "map"),
        sensor_profile_id=str(snapshot_value.get("sensor_profile_id") or "unknown"),
    )
    map_reference = run_dir / "map_reference.json"
    map_reference.write_text(
        json.dumps({
            "schema_version": "go2w_map_reference_v1",
            **bundle_revisions.to_dict(),
            "map": snapshot_value.get("map") or {},
            "source": "world_state_snapshot",
        }, ensure_ascii=False, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    component_names = {
        "map_reference": "map_reference.json",
        "task": "task.json",
        "decisions": "decisions.jsonl",
        "summary": "summary.json",
        "exploration_graph": "exploration_graph.json",
        "semantic_map": "semantic_map.json",
        "place_graph": "place_graph.json",
        "spatial_memory": "spatial_memory.json",
    }
    replay_manifest = run_dir / "replay/manifest.json"
    write_world_manifest(
        run_dir,
        revisions=bundle_revisions,
        components={
            name: run_dir / relative
            for name, relative in component_names.items()
            if (run_dir / relative).is_file()
        },
        mutable_components=(
            {"replay_manifest": replay_manifest}
            if replay_manifest.is_file() else {}
        ),
        metadata={
            "session_id": explorer.session_id,
            "requires_relocalization_before_navigation": True,
            "motion_authority_restored": False,
        },
    )
    return 0 if result.result == "TARGET_FOUND" else 3


def _task_context_from_args(args: argparse.Namespace):
    from app.task_understanding.search_task_context import SearchTaskContext

    if getattr(args, "task_context_json", ""):
        try:
            value = json.loads(args.task_context_json)
            if isinstance(value, dict):
                return SearchTaskContext.from_dict(value)
        except (TypeError, ValueError, json.JSONDecodeError):
            pass
    return SearchTaskContext.mock_fallback(args.target)


def _probe_readiness(args, node):
    """Automatic health probe for the experiment profile (plan section 17)."""
    from app.live_robot.frame_bundle_reader import FrameBundleReader
    bundle_ok = False
    camera_ok = False
    try:
        reader = FrameBundleReader(args.spool_root)
        bundle = reader.read_latest(timeout_seconds=2.0)
        bundle_ok = bundle is not None
        camera_ok = bool((bundle.payload.get("sensor_health") or {}).get("camera"))
    except Exception:
        pass
    try:
        motion_ready = bool(node._client.server_is_ready())
    except Exception:
        motion_ready = False
    try:
        arm_ready = bool(node._arm_client.service_is_ready())
        stop_ready = bool(node._stop_srv.service_is_ready())
    except Exception:
        arm_ready = stop_ready = False
    mode_ok = bool(
        node._sport is not None
        and int(node._sport.mode) == 1
        and int(node._sport.error_code) == 0
    )
    pose_fresh = bool(node._odom is not None)
    # 报告里的 backend 与 capabilities 必须是**同一个**后端的。
    #
    # 这里曾经写死一张相对后端的能力表（supports_relative_translation=True、
    # supports_metric_navigation 不声明），而 backend 字段照实参填 nav2 ——
    # 于是 2026-10-15 11:05 那一场里，同一次会话的 `backend_health` 报
    # `supports_metric_navigation: true` / `allowed_motion_primitives:
    # ["NAVIGATE_POSE","STOP"]`，`experiment_readiness` 报 false /
    # `["FORWARD","ROTATE_LEFT","ROTATE_RIGHT"]`。两份报告互相打脸，而读的人
    # 只会以为"这个后端没有地图导航能力"。
    #
    # 修法是按后端**种类**取真值，而不是在这里再抄一张表：本函数跑在后端实例
    # 建起来之前（readiness 是执行前的闸门，nav2 后端要到主循环前才创建），
    # 那时手上只有 `args.backend` 这个字符串，所以两张能力表都放在各自的后端
    # 模块里当模块级常量，`capabilities()` 也返回同一份。
    backend_kind = str(getattr(args, "backend", "") or "nav2").strip().lower()
    if backend_kind == "nav2":
        from app.navigation.nav2_search_backend import NAV2_CAPABILITIES

        capabilities = NAV2_CAPABILITIES
    else:
        from app.navigation.go2w_experimental_backend import (
            GO2W_EXPERIMENTAL_CAPABILITIES,
        )

        capabilities = GO2W_EXPERIMENTAL_CAPABILITIES
    # The production path uses the long-running VLM daemon.  In that mode the
    # ROS worker intentionally does not need to inherit the provider key: the
    # daemon owns the API client and may have loaded the key from .env before
    # it was started.  Treat a live daemon socket as an available LLM runtime;
    # otherwise retain the legacy direct-key check.
    llm_daemon_available = False
    try:
        from app.detectors.siliconflow_vision_protocol import SiliconFlowDaemonClient

        llm_daemon_available = SiliconFlowDaemonClient(
            str(PROJECT_ROOT / "runtime/go2w/siliconflow_vlm.sock")
        ).available()
    except Exception:
        llm_daemon_available = False
    llm_key_configured = bool(os.getenv("SILICONFLOW_API_KEY"))
    llm_runtime_available = llm_key_configured or llm_daemon_available
    return compute_experiment_readiness(
        # 报告里的 backend 必须是这一趟真正在跑的那个（§4.3：不让
        # go2w_experimental 的名字继续出现在生产报告里）。用上面判定能力表用的
        # 同一个 backend_kind，否则「名字」和「能力」又会有两个来源。
        backend=backend_kind,
        camera_fresh=camera_ok,
        bundle_fresh=bundle_ok,
        llm_available=llm_runtime_available,
        motion_action_available=motion_ready,
        robot_mode_ok=mode_ok,
        emergency_stop_available=stop_ready,
        # 后端在这里还没被建起来（readiness 是执行前的闸门，nav2 后端要到主循环
        # 之前才创建），所以这一项不是**观测**、是**延迟断言**：两条路径后面都会
        # 硬核对 —— nav2 在创建后 _wait_for(backend.health().ready) 不成立就抛
        # RuntimeError，相对后端在每次 execute_goal 里 health() 不就绪就返回
        # BACKEND_UNAVAILABLE。也就是说报 True 不会放过一个坏后端，只是不在这
        # 一刻下结论。真正**会被读错**的是能力表，那项已按后端实取（见上）。
        backend_healthy=True,
        pose_freshness_if_available=pose_fresh,
        capabilities=capabilities,
        check_llm_key=not llm_daemon_available,
    )


if __name__ == "__main__":
    raise SystemExit(main())
