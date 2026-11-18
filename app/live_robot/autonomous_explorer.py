"""AutonomousExplorer: the high-level long-cycle orchestrator.

It owns the continuous loop

    OBSERVE -> MATCH -> VERIFY -> UPDATE_MEMORY -> PLAN -> EXECUTE
    -> WAIT_RESULT -> RECOVER/REPLAN -> OBSERVE -> ... -> TARGET_FOUND

The explorer only knows the platform-independent contracts
(``RobotBackend``, ``ExplorationGoal``, ``ExplorationGraph``,
``LiveObservation``); perception details and robot specifics are injected as
callables, so the same class runs on the Go2-W experimental backend, mock
backends and a future production metric backend.
"""

from __future__ import annotations

import json
import math
import threading
import time
import traceback
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from app.config import get_settings
from app.navigation.approach_controller import (
    ACTION_APPROACH_BLOCKED,
    ACTION_APPROACH_COMPLETE,
    ACTION_APPROACH_FORWARD,
    ACTION_ROTATE_SMALL,
    ACTION_STOP_REOBSERVE,
    ACTION_TARGET_REACHED_OR_TOO_CLOSE,
    decide_approach,
    evaluate_target_reached,
    robot_bearing_from_camera,
)
from app.navigation.candidate_goal_generator import (
    # 私有名，但它是这条纪律唯一的真源：兜底前进被哪个安全条件挡下。
    # 候选生成器把它算出来只当布尔用，出了函数就丢，所以这里重算一次
    # 留下来当证据（计划书 §1.5）。改那边必须同时看这里。
    _forward_fallback_block_reason,
    generate_live_exploration_candidates,
)
from app.navigation.clearance_map import ClearanceMap, clearance_map_from_local_safety
from app.navigation.exploration_config import (
    CandidateConfig,
    ExplorationBudget,
    ExplorationPolicy,
    MemoryConfig,
    RecoveryConfig,
    ScoringWeights,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph, ObservationNode
from app.navigation.exploration_planner import (
    ScoredGoal,
    score_exploration_goal,
    select_exploration_goal,
)
from app.navigation.models import (
    GOAL_NAVIGATE_POSE,
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    ExplorationGoal,
    LiveObservation,
)
from app.navigation.target_pose_generator import generate_target_approach_viewpoints
from app.navigation.viewpoint_models import TargetApproachContext
from app.navigation.decision_models import DecisionRecord, make_motion_command
from app.navigation.live_graph_path_planner import plan_live_graph_path
from app.navigation.recovery_manager import (
    RecoveryManager,
    RecoveryManagerConfig,
    select_escape_rotation,
)
from app.navigation.safety_models import LocalSafetySnapshot, PerceptionQuality
from app.navigation.search_policy_config import SearchPolicy
from app.navigation.stuck_detector import (
    StuckDetectorInput,
    StuckState,
    detect_stuck,
    requires_recovery,
)
from app.navigation.viewpoint_planner import ViewpointPlanner, ViewpointPlannerConfig
from app.reasoning.embodiment_context import (
    build_local_safety_context,
    build_perception_quality_context,
    build_recovery_context,
    build_topology_context,
)
from app.navigation.robot_backend import (
    NavigationResult,
    NavigationStatus,
    RobotBackend,
    TERMINAL_NAVIGATION_STATUSES,
)
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph
from app.perception.target_state import TARGET_ABSENT, TARGET_POSSIBLE, TARGET_UNKNOWN
from app.live_robot.async_verifier import VERIFY_STATUS_REJECTED, VERIFY_STATUS_UNKNOWN
from app.task_understanding.search_task_context import SearchTaskContext


def _json_context(context: dict[str, Any]) -> dict[str, Any]:
    """Render the pydantic embodiment context for the event stream."""
    return {
        key: value.model_dump(mode="json") if hasattr(value, "model_dump") else value
        for key, value in context.items()
    }


def _pose_xy_yaw(pose: dict[str, Any] | None) -> tuple[float, float, float]:
    """Read ``(x, y, yaw_rad)`` from an observation pose dict.

    The key priority matches ``SemanticNavigationGraph`` frontier placement so
    frontier positions, the clearance map and viewpoint poses share one frame.
    """
    data = pose or {}
    yaw = data.get("yaw_rad")
    if yaw is None:
        yaw = data.get("yaw")
    if yaw is None and data.get("yaw_deg") is not None:
        yaw = math.radians(float(data["yaw_deg"]))
    return float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(yaw or 0.0)


def _has_xy(pose: dict[str, Any] | None) -> bool:
    """这一帧的位姿里到底有没有坐标（计划书 §3.2）。

    ``_pose_xy_yaw`` 对缺失的字段一律回落到 0，所以「机器人在原点」和「这帧
    根本没有位姿」在它的返回值上长得一模一样。凡是拿这个结果去**发目标**的
    地方，都必须先在这里问一句，否则位姿一丢就会把地图原点当成机器人所在。
    """
    if not isinstance(pose, Mapping):
        return False
    return pose.get("x") is not None and pose.get("y") is not None


# 观测到的平移量小于这个值就当作"没动"（2 cm；真机一步最小 0.10~0.30 m，
# 差着一个数量级，不会因为里程计抖动把"动了"误判成"没动"）。
_MIN_OBSERVED_TRANSLATION_M = 0.02

# 各后端 observed_motion 里表示"真的挪了多少米"的键。legacy 后端与 mock 写
# ``displacement_m``；nav2 后端的 observed_motion 里**根本没有位移键**（只有
# path_length_m / goal_x 这些计划量），所以不能只认键名——见
# ``_goal_commands_translation`` 的兜底。
_OBSERVED_TRANSLATION_KEYS = ("displacement_m", "observed_displacement_m")


def _observed_translation_m(observed_motion: Any) -> float | None:
    """从 observed_motion 里读出观测到的平移量（米）；读不到返回 None。

    None 的语义是"**没有这条证据**"，不是"位移是 0"——两者在下游的处置相反：
    前者要 fail-open（保持旧行为），后者才能判"没动"。
    """
    if not isinstance(observed_motion, Mapping):
        return None
    for key in _OBSERVED_TRANSLATION_KEYS:
        if observed_motion.get(key) is not None:
            return abs(float(observed_motion[key]))
    return None


# 命令要求的平移量小于这个值就算"原地"（2 cm）。取这个数量级有两个理由：
# 真机最小步幅实测约 0.10 m，差着一个数量级；而地图系目标的坐标来自
# ``plan_context`` 里那份 robot_pose，与执行时这次 ``get_pose()`` **不是同一个
# 采样**，原地转向的目标点因此不会逐位相等，判据不能写成 ``== 0.0``。
_MIN_COMMANDED_TRANSLATION_M = 0.02


def _commanded_translation_m(goal: Any, pose: Any) -> float:
    """这条命令要求机器人平移多远（米）。

    **两种后端写在两处不同的字段上**，只认一处就会在另一种后端上失灵：

    * 相对后端（``go2w_experimental``）：``relative_dx`` / ``relative_dy``；
    * 地图系后端（``nav2``，**默认**）：三个 ``relative_*`` **全是 None**，
      ``local_goal_executor._metric_goal`` 写的是绝对 ``position`` + ``yaw``
      （计划书 §6.7）。位移只能从"目标点与当前位姿的距离"算——原地转向的地图系
      目标就是「当前坐标 + 换朝向」（拿不到偏好位置时 ``target =
      current_position``），距离因此约等于 0。

    **``pose`` 必须是"下发命令之前"那一次采样的位姿。**它换来的是"这条命令要求
    从哪儿挪到哪儿"；换成执行之后的位姿就变成"最后停在离目标多远"，走到位了恒
    得 0（默认路径上 ``_did_translate`` 与 ``_last_command_was_in_place_turn``
    会一起失灵，见 EXECUTE 段注释）。
    """
    dx = abs(float(getattr(goal, "relative_dx", 0.0) or 0.0))
    dy = abs(float(getattr(goal, "relative_dy", 0.0) or 0.0))
    if dx + dy > 0.0:
        return dx + dy
    target = getattr(goal, "position", None)
    if target is None or pose is None:
        return 0.0
    return math.hypot(
        float(target[0]) - float(pose.x), float(target[1]) - float(pose.y)
    )


def _commanded_turn_deg(goal: Any, pose: Any) -> float:
    """这条命令要求机器人转多少度（带符号）。单位换算在两个后端间是反的：
    ``ExplorationGoal.yaw`` 是**度**，而 ``RobotPose.yaw`` 是**弧度**。

    ``pose`` 同上：必须是**下发命令之前**那一次采样。用执行之后的位姿，一次
    已经转到位的地图系转向会被算成 dyaw=0 ⇒ ``_last_command_was_in_place_turn``
    判"不是原地转向"（实测：90° 转向在 ``MockMetricBackend`` 上得 0.000）。
    """
    dyaw = getattr(goal, "relative_dyaw", None)
    if dyaw is not None:
        return float(dyaw)
    yaw = getattr(goal, "yaw", None)
    if yaw is None or pose is None:
        return 0.0
    delta = float(yaw) - math.degrees(float(pose.yaw))
    return (delta + 180.0) % 360.0 - 180.0


def _goal_commands_translation(goal: Any, pose: Any) -> bool:
    """这条命令本身要不要机器人平移。

    原地转向类原语（ROTATE_VIEW / INSPECT_ANCHOR / REVISIT_NODE，以及由
    LOCAL_SCAN 转成的地图系 NAVIGATE_POSE）的平移量恒为 0 是**构造事实**，
    不是"没有证据"——所以可以据此判"没动"。

    2026-10-14 修正：原来只看 ``relative_dx``/``relative_dy``，而**默认的 nav2
    后端一个 relative_* 都不写**，于是"成功走了 3 米"也被判成"没打算平移"，
    ``_did_translate`` 落到最后一支返回 False —— 比改之前的
    ``bool(result.succeeded)`` 更差。现在两种表示都认。
    """
    return _commanded_translation_m(goal, pose) > _MIN_COMMANDED_TRANSLATION_M


class ExplorerState(str, Enum):
    BOOTSTRAP = "BOOTSTRAP"
    OBSERVE = "OBSERVE"
    MATCH = "MATCH"
    VERIFY = "VERIFY"
    UPDATE_MEMORY = "UPDATE_MEMORY"
    PLAN = "PLAN"
    EXECUTE = "EXECUTE"
    WAIT_RESULT = "WAIT_RESULT"
    RECOVER = "RECOVER"
    PAUSED = "PAUSED"
    # 计划书 §6.3：地图暂时不新鲜是一种等待状态，绝不是搜索穷尽。
    WAITING_FOR_MAP = "WAITING_FOR_MAP"
    # 计划书 §18.1：视觉确认之后不再直接成功，必须走完接近闭环。
    TARGET_CONFIRMED = "TARGET_CONFIRMED"
    TARGET_LOCALIZE = "TARGET_LOCALIZE"
    APPROACH_PLAN = "APPROACH_PLAN"
    APPROACH_EXECUTE = "APPROACH_EXECUTE"
    FINAL_OBSERVE = "FINAL_OBSERVE"
    FINAL_VERIFY = "FINAL_VERIFY"
    TARGET_REACHED = "TARGET_REACHED"
    TARGET_FOUND = "TARGET_FOUND"
    SEARCH_EXHAUSTED = "SEARCH_EXHAUSTED"
    OPERATOR_STOP = "OPERATOR_STOP"
    FAILED = "FAILED"
    FINISHED = "FINISHED"


EXPLORER_FINISH_REASONS = (
    "TARGET_FOUND",
    "TIMEOUT",
    "SEARCH_EXHAUSTED",
    "OPERATOR_STOP",
    "BACKEND_FAILURE",
    "PERCEPTION_FAILURE",
    "MAX_STEPS_REACHED",
    # 计划书 §6.2：候选生成异常和长时间等不到地图是程序/数据问题，用独立
    # finish reason 表达，不允许伪装成 SEARCH_EXHAUSTED。
    "PLANNING_ERROR",
    "MAP_UNAVAILABLE",
    "BUDGET_EXHAUSTED",
)


@dataclass
class SemanticMatch:
    """Matcher output handed to the verifier / memory / planner."""

    has_candidate: bool
    graph_match: Any | None = None
    target_match: dict[str, Any] | None = None
    target_profile: Any | None = None
    anchor_labels: list[str] = field(default_factory=list)
    directive: Any | None = None
    target_score: float = 0.0
    target_match_level: str = "none"
    target_state: str = "ABSENT"
    provenance: dict[str, Any] = field(default_factory=dict)


@dataclass
class VerificationOutcome:
    confirmed: bool
    attempts: int
    reason_zh: str = ""
    details: dict[str, Any] = field(default_factory=dict)
    # 计划书 §5.1.5：``""`` = 旧语义（内联复核，"没确认"即否定），保留给
    # 同步路径与既有调用方，逐字不变；``UNKNOWN`` = **没有判定**（超时/报错/
    # 还在飞），下游不许拿它当"目标不存在"。取值见
    # `app/live_robot/async_verifier.py`。
    status: str = ""


def verification_is_terminal_negative(verification: "VerificationOutcome | None") -> bool:
    """这条复核结论是不是一个**已终结的否定**。

    只有"复核真的跑完了、并且说不是"才允许把候选降成 ABSENT。
    没跑完（超时/异常/在途）与"没结论"都不是证据 —— 把两者混起来，就会
    把一次没做完的复核记成"目标不存在"。
    """
    if verification is None or verification.confirmed:
        return False
    return verification.status in ("", VERIFY_STATUS_REJECTED)


@dataclass
class SessionResult:
    result: str
    target: str = ""
    duration_s: float = 0.0
    planning_cycles: int = 0
    motion_steps: int = 0
    observations: int = 0
    unique_nodes: int = 0
    unique_places: int = 0
    unique_objects: int = 0
    frontiers_discovered: int = 0
    map_nodes_total: int = 0
    replans: int = 0
    navigation_failures: int = 0
    verify_attempts: int = 0
    semantic_goal_selection_count: int = 0
    fallback_goal_selection_count: int = 0
    finish_reason: str = ""
    session_id: str = ""
    summary: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "result": self.result,
            "target": self.target,
            "duration_s": round(self.duration_s, 3),
            "planning_cycles": self.planning_cycles,
            "motion_steps": self.motion_steps,
            "observations": self.observations,
            "unique_nodes": self.unique_nodes,
            "unique_places": self.unique_places,
            "unique_objects": self.unique_objects,
            "frontiers_discovered": self.frontiers_discovered,
            "map_nodes_total": self.map_nodes_total,
            "replans": self.replans,
            "navigation_failures": self.navigation_failures,
            "verify_attempts": self.verify_attempts,
            "semantic_goal_selection_count": self.semantic_goal_selection_count,
            "fallback_goal_selection_count": self.fallback_goal_selection_count,
            "finish_reason": self.finish_reason,
            "session_id": self.session_id,
            "summary": self.summary,
        }


class PerceptionFailure(RuntimeError):
    """Observer could not produce a valid observation.

    Structured error contract (计划书 §8.2/§8.5):

    * ``code``: machine-readable cause (QUICK_VLM_TIMEOUT, RGBD_TIMEOUT,
      RGB_IMAGE_ERROR, DEPTH_ERROR, VLM_PARSE_ERROR, TF_UNAVAILABLE,
      FRAME_STALE, FULL_SEMANTIC_TIMEOUT, UNKNOWN_PERCEPTION_ERROR, ...)
    * ``recoverable``: transient errors may be retried with a fresh
      observation; non-recoverable ones fail the session immediately.

    ``FRAME_MISMATCH`` 曾经列在这里，2026-09-16 删掉：**语义/深度帧对不上不是一种
    失败**。计划书 §5.2 规定找不到同一 frame 时"不做 3D localization、不用 current
    depth 顶替，只保留 2D semantic" —— 那是一次**降级**（``spatial_quality=
    SEMANTIC_2D_ONLY``），运行照常继续，记录走 ``semantic_depth_frame_mismatch``
    事件（``run_semantic_exploration.py`` 里那条），不该抛成 PerceptionFailure。
    留一个永远不会被 raise 的码在这里，只会让人以为"帧不匹配会自动重试"。
    """

    def __init__(
        self,
        message: str,
        *,
        code: str = "UNKNOWN_PERCEPTION_ERROR",
        recoverable: bool = True,
        detail: str = "",
        last_success_age_s: float | None = None,
    ) -> None:
        super().__init__(message)
        self.code = str(code)
        self.recoverable = bool(recoverable)
        self.detail = str(detail)
        self.last_success_age_s = last_success_age_s

    def to_dict(self) -> dict[str, Any]:
        return {
            "error_type": "PERCEPTION_ERROR",
            "code": self.code,
            "message": str(self),
            "recoverable": self.recoverable,
            "detail": self.detail,
            "last_success_age_s": self.last_success_age_s,
        }


class AutonomousExplorer:
    """Long-cycle operator-supervised semantic exploration orchestrator."""

    def __init__(
        self,
        *,
        target: str = "",
        task_context: SearchTaskContext | None = None,
        observer: Callable[[], LiveObservation | None],
        matcher: Callable[[LiveObservation], SemanticMatch],
        verifier: Callable[[LiveObservation, SemanticMatch], VerificationOutcome],
        backend: RobotBackend,
        policy: ExplorationPolicy | None = None,
        graph: ExplorationGraph | None = None,
        negative_memory: Any = None,
        negative_target_key: str = "target",
        candidate_generator: Callable[..., list[Any]] | None = None,
        planner: Callable[..., ScoredGoal | None] | None = None,
        exhaustion_probe: Callable[[], dict[str, Any]] | None = None,
        on_event: Callable[[dict[str, Any]], None] | None = None,
        finish_on_visual_confirmation: bool = True,
        turn_only: bool = False,
        dry_run: bool = False,
        max_navigation_goal_attempts: int = 0,
        session_id: str | None = None,
        max_perception_retries: int = 2,
        max_waiting_for_map_cycles: int = 20,
        now: Callable[[], float] = time.time,
        semantic_graph: SemanticNavigationGraph | None = None,
        executor_id: str | None = None,
        worker_generation: int | None = None,
        local_safety_provider: Callable[[LiveObservation], LocalSafetySnapshot | None] | None = None,
        perception_quality_provider: Callable[[LiveObservation, float], PerceptionQuality | None] | None = None,
        breadcrumb_provider: Callable[[], Any] | None = None,
        recovery_manager: RecoveryManager | None = None,
        search_policy: SearchPolicy | None = None,
        clearance_map_provider: Callable[
            [LiveObservation, LocalSafetySnapshot | None], ClearanceMap | None
        ] | None = None,
        viewpoint_planner: ViewpointPlanner | None = None,
        decision_context_provider: Callable[[], dict[str, Any] | None] | None = None,
        preflight_rejection_handler: Callable[
            [ExplorationGoal, NavigationResult], None
        ] | None = None,
        navigation_target_monitor: Callable[[], bool] | None = None,
    ) -> None:
        self.task_context = task_context
        self.target = task_context.canonical_target if task_context else target
        self._observer = observer
        self._matcher = matcher
        self._verifier = verifier
        self._backend = backend
        self.policy = policy or load_exploration_policy()
        self.graph = graph or ExplorationGraph(session_id=session_id or "exploration_session")
        self.semantic_graph = semantic_graph or SemanticNavigationGraph()
        self.negative_memory = negative_memory
        self.negative_target_key = negative_target_key
        self._candidate_generator = candidate_generator or generate_live_exploration_candidates
        self._planner = planner or select_exploration_goal
        # 计划书 §6.3：空候选必须由真实地图/frontier 证据分类，探索器自己不再
        # 猜测原因。探针只读，不产生第二套候选。
        self._exhaustion_probe = exhaustion_probe
        self._on_event = on_event or (lambda event: None)
        self._decision_context_provider = decision_context_provider
        self._preflight_rejection_handler = preflight_rejection_handler
        self._navigation_target_monitor = navigation_target_monitor
        self.finish_on_visual_confirmation = bool(finish_on_visual_confirmation)
        self.turn_only = bool(turn_only)
        # 干跑整场不发运动指令。这是个**会话级**事实（不是逐条结果的事），
        # 所以由调用方一次告知，不去猜每条结果里的记号。
        self.dry_run = bool(dry_run)
        self._max_navigation_goal_attempts = max(0, int(max_navigation_goal_attempts))
        self._navigation_goal_attempts = 0
        self.session_id = session_id or time.strftime("explore_%Y%m%d_%H%M%S")
        self.max_perception_retries = max(0, int(max_perception_retries))
        self.max_waiting_for_map_cycles = max(1, int(max_waiting_for_map_cycles))
        self._now = now
        self._run_started_at: float | None = None
        self._state = ExplorerState.BOOTSTRAP
        self._operator_stop_requested = False
        self._pause_requested = False
        self._paused = False
        self._control_lock = threading.RLock()
        self._control_generation = 0
        self._observation_generation = 0
        self.events: list[dict[str, Any]] = []
        self._seen_labels: set[str] = set()
        self._no_information_cycles = 0
        # True when the previous EXECUTE step actually TRANSLATED the robot.
        # As long as it keeps moving we keep exploring and do not count "no new
        # information" against the exhaust budget (a sofa may just not be
        # visible from this office corner yet).
        #
        # 2026-10-14：这一位原来写的是 ``bool(result.succeeded)``——名字说"动了"，
        # 值是"后端报成功"。真机上两者不等价：09-15 那轮 12 条
        # ``dry-run turn ... validated`` 全是 SUCCEEDED，而
        # ``observed_motion.yaw_delta_deg`` 恒为 0.0。后端一直报成功 ⇒ 这一位
        # 一直是 True ⇒ ``_no_information_cycles`` 每轮被清零 ⇒「无信息」闸门
        # 永远不开 ⇒ ``_classify_no_information`` / ``_stuck_state`` 整条卡死
        # 判据一次都跑不到（那两轮语料里 ``no_information_classified`` 与
        # ``stuck_detected`` 都是 0 次）。现在改由观测到的平移量判定，见
        # ``_did_translate``。
        self._last_execution_moved = False
        # 连续多少轮"只转不平移、且一点新信息都没换到"——这是"原地打转"的机械
        # 定义。原本喂给 ``StuckDetectorInput.repeated_heading_count`` 的是
        # ``graph.sector_visited_count()``，它只返回 0/1，而阈值是 3，
        # 所以那半个判据在数学上不可能成立。
        self._in_place_turn_streak = 0
        # 第二个消费者闸门的迟滞位：这一**段"没进步"**（从上一次换到新信息算起）
        # 已经问过一次卡死分类了。它同时是循环边界的保证：连跳条件在"一直原地转"
        # 时每轮都成立，而闸门里那条脱困支路是 ``continue``（不涨
        # ``planning_cycles``、也未必涨 ``motion_steps``）—— 不设迟滞的话闸门每轮
        # 都开、每轮都脱困，整个循环只剩 ``max_search_seconds`` 一个边界
        # （夹具实测：200 轮里问了 186 次、脱困 186 次）。
        # **只有换到新信息才复位**，不是"不再原地转"就复位：脱困动作本身会写进
        # 命令窗口（倒车是平移），跳数会被它清掉、迟滞跟着复位，于是"转 3 轮 →
        # 脱困 → 再转 3 轮"能永远循环下去 —— 那是个新的饿死支路。
        self._spin_gate_latched = False
        self._last_goal_source: str | None = None
        # 计划书 §6.2/§6.3：候选生成异常与等待地图各自独立计数，异常第二次
        # 复现才判 FAILED，地图长时间不新鲜才判 MAP_UNAVAILABLE。
        self._candidate_error_signatures: list[str] = []
        self._waiting_for_map_cycles = 0
        self._current_place_id: str | None = None
        self._perception_failure_detail: dict[str, Any] | None = None
        self.executor_id = executor_id
        self.worker_generation = worker_generation
        self.decision_records: list[DecisionRecord] = []
        # POSSIBLE target evidence gets a bounded fresh-frame verification
        # loop.  It is never allowed to fall directly through to motion.
        self._possible_reobserve_count = 0
        # 计划书 §11/§12/§13：局部安全快照 + 卡死判定 + 脱困。
        # ``local_safety_provider is None`` 时保持旧行为（无 LiDAR 证据就不
        # 允许任何自主脱困动作，fail-closed）。
        self._local_safety_provider = local_safety_provider
        self._perception_quality_provider = perception_quality_provider
        self._breadcrumb_provider = breadcrumb_provider
        self.search_policy = search_policy or SearchPolicy()
        self._recovery_manager = recovery_manager or RecoveryManager(
            RecoveryManagerConfig(
                enabled=self.search_policy.recovery.enabled,
                max_backward_step_m=self.search_policy.recovery.max_backward_step_m,
                min_backward_step_m=self.search_policy.recovery.min_backward_step_m,
                max_age_sec=self.search_policy.recovery.max_age_sec,
                heading_tolerance_deg=self.search_policy.recovery.heading_tolerance_deg,
                max_consecutive=self.search_policy.recovery.max_consecutive,
                max_total_m=self.search_policy.recovery.max_total_m,
            )
        )
        # 计划书 §5/§7/§10：净空地图 + 视点规划。没有 clearance 证据时
        # (provider 与 local_safety_provider 都为 None) 保持旧候选逻辑。
        self._clearance_map_provider = clearance_map_provider
        clearance = self.search_policy.clearance_navigation
        self._viewpoint_planner = viewpoint_planner or ViewpointPlanner(
            ViewpointPlannerConfig(
                hard_clearance_m=clearance.hard_clearance_m,
                preferred_clearance_m=clearance.preferred_clearance_m,
                footprint_radius_m=clearance.footprint_radius_m,
            ),
            weights=self.policy.scoring,
        )
        self._last_clearance_map: ClearanceMap | None = None
        self._last_embodiment_context: dict[str, Any] = {}
        self._last_safety_rejections: list[dict[str, Any]] = []
        self._pose_history: list[tuple[float, float, float]] = []
        self._command_history: list[dict[str, Any]] = []
        self._result_history: list[str] = []
        self._last_stuck_state: StuckState | None = None
        self._no_information_replans = 0
        self._recovery_count = 0

    # ---- operator interface ----------------------------------------------

    def request_stop(self) -> None:
        """Operator stop: halt motion and finish with OPERATOR_STOP."""
        with self._control_lock:
            self._operator_stop_requested = True
            self._pause_requested = False
            self._control_generation += 1
            try:
                self._backend.stop()
            except Exception:
                pass

    def request_pause(self) -> None:
        """Operator pause (plan book §40): stop generating new goals and stop
        the current motion; memory and the exploration graph are kept."""
        with self._control_lock:
            if self._operator_stop_requested:
                return
            self._pause_requested = True
            self._control_generation += 1
            try:
                self._backend.stop()
            except Exception:
                pass

    def request_resume(self) -> None:
        """Operator resume: continue the loop; the next cycle re-observes and
        replans (no stale instruction is restored)."""
        with self._control_lock:
            self._pause_requested = False

    def _control_interrupted(self, generation: int) -> bool:
        with self._control_lock:
            return (self._operator_stop_requested or self._pause_requested
                    or generation != self._control_generation)

    def _dispatch_goal(self, goal: ExplorationGoal, generation: int) -> Any:
        # Serialize dispatch with stop/pause: either the goal precedes the
        # operator command and is stopped, or it is never submitted. An epoch
        # also invalidates evidence when pause/resume both occur during VLM.
        with self._control_lock:
            if self._control_interrupted(generation):
                return None
            if not self._claim_navigation_goal(goal.goal_id):
                return None
            return self._backend.execute_goal(goal)

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def state(self) -> str:
        """Public terminal/current state used by session artifact writers."""
        return self._state.value

    @property
    def operator_stop_requested(self) -> bool:
        return self._operator_stop_requested

    # ---- main loop ---------------------------------------------------------

    def _pump_backend(self, seconds: float = 0.0) -> None:
        """重试等待期间把后端的 ROS 订阅转起来。

        nav2 后端的就绪判据（``/go2w/nav/localization_valid``、``/map``）由订阅
        回调喂，缓存初值是「未收到」。原来的重试只在两次 ``health()`` 之间
        ``time.sleep``，而 ``run()`` 是阻塞调用、bootstrap 期间没有任何人在 spin，
        回调永不执行 ⇒ 无论重试几次都判 ``NO_LOCALIZATION``，搜索连第一轮规划都
        进不去。后端不提供 ``spin_once`` 时（离线 mock、实验后端）退回 sleep，
        语义与原来一致。
        """
        spin = getattr(self._backend, "spin_once", None)
        if spin is None:
            if seconds > 0:
                time.sleep(seconds)
            return
        deadline = time.monotonic() + max(0.0, seconds)
        while True:
            spin()
            if time.monotonic() >= deadline:
                return

    def run(self) -> SessionResult:
        started = float(self._now())
        self._run_started_at = started
        self._navigation_goal_attempts = 0
        budget = self.policy.budget
        planning_cycles = 0
        motion_steps = 0
        observations = 0
        replans = 0
        navigation_failures = 0
        verify_attempts = 0
        semantic_goal_selection_count = 0
        fallback_goal_selection_count = 0
        finish_reason = ""

        self._state = ExplorerState.BOOTSTRAP
        self._emit("session_start", target=self.target, session_id=self.session_id,
                   task_id=self.task_context.task_id if self.task_context else None,
                   task_context=self.task_context.to_dict() if self.task_context else None,
                   budget=budget.to_dict(),
                   # Keep the terminal-behaviour switch in the first durable
                   # event.  A deployed run can then prove whether a visual
                   # confirmation was intentionally legacy-terminal or was
                   # required to enter the target-approach state machine.
                   finish_on_visual_confirmation=self.finish_on_visual_confirmation,
                   target_approach_enabled=self.search_policy.target_approach.enabled,
                   policy=asdict(self.policy) if hasattr(self.policy, "__dataclass_fields__") else self.policy.to_dict())

        # ---- BOOTSTRAP ------------------------------------------------------
        backend_ready = False
        for attempt in range(1, max(1, self.policy.recovery.backend_reconnect_attempts) + 1):
            self._pump_backend()
            health = self._backend.health()
            self._emit("backend_health", attempt=attempt, ready=health.ready,
                       degraded=health.degraded, health=health.to_dict())
            if health.ready:
                backend_ready = True
                break
            if attempt < self.policy.recovery.backend_reconnect_attempts:
                self._pump_backend(
                    self.policy.recovery.backend_reconnect_delay_seconds
                )
        if not backend_ready:
            finish_reason = "BACKEND_FAILURE"
            self._state = ExplorerState.FAILED
            self._emit("session_finish", result=finish_reason,
                       reason="backend unhealthy after reconnect attempts")
            return self._result(finish_reason, started, planning_cycles, motion_steps,
                                observations, replans, navigation_failures,
                                verify_attempts, semantic_goal_selection_count,
                                fallback_goal_selection_count, finish_reason)

        self._state = ExplorerState.OBSERVE
        elapsed = float(self._now()) - started
        while budget.remaining(
            elapsed_sec=float(self._now()) - started, planning_cycles=planning_cycles,
            motion_steps=motion_steps,
        ):
            if self._operator_stop_requested:
                finish_reason = "OPERATOR_STOP"
                self._state = ExplorerState.OPERATOR_STOP
                try:
                    self._backend.stop()
                except Exception:
                    pass
                self._emit("session_finish", result=finish_reason, reason="operator stop")
                return self._result(finish_reason, started, planning_cycles, motion_steps,
                                    observations, replans, navigation_failures,
                                    verify_attempts, semantic_goal_selection_count,
                                    fallback_goal_selection_count, finish_reason)

            # ---- PAUSE gate (plan book §40) -------------------------------
            # While paused the loop parks without generating new goals or
            # motion; memory and the graph stay intact. Resume re-enters the
            # normal OBSERVE -> ... -> PLAN pipeline, so the next decision is
            # always a fresh replan, never a restored instruction.
            if self._pause_requested:
                self._state = ExplorerState.PAUSED
                self._paused = True
                try:
                    self._backend.stop()
                except Exception:
                    pass
                self._emit("paused")
                while self._pause_requested and not self._operator_stop_requested:
                    time.sleep(0.2)
                self._paused = False
                if not self._operator_stop_requested:
                    self._emit("resumed")
                continue

            # ---- OBSERVE ----------------------------------------------------
            with self._control_lock:
                generation = self._control_generation
                self._observation_generation = generation
            self._state = ExplorerState.OBSERVE
            self._emit(
                "phase_progress",
                phase="OBSERVE",
                operation="capture_and_analyze",
                detail_zh="正在获取最新画面并分析目标与场景；机器狗会原地等待分析完成",
            )
            perception_retries = 0
            last_perception_error: PerceptionFailure | None = None
            observation: LiveObservation | None = None
            pipeline_start_perf = time.perf_counter()
            while observation is None and not self._control_interrupted(generation):
                try:
                    observation = self._observer()
                except PerceptionFailure as exc:
                    last_perception_error = exc
                    self._emit("perception_failure", error=str(exc),
                               code=exc.code, recoverable=exc.recoverable,
                               detail=exc.detail,
                               last_success_age_s=exc.last_success_age_s)
                    # 计划书 §8.3：只要错误可恢复就 retry，无论之前是否已经
                    # 成功观察过；机器人保持 stop，不允许拿 stale 帧继续发运动。
                    if exc.recoverable and perception_retries < self.max_perception_retries:
                        perception_retries += 1
                        backoff = float(perception_retries)  # retry1=1s, retry2=2s
                        self._emit("perception_retry", attempt=perception_retries,
                                   reason=str(exc), code=exc.code,
                                   backoff_seconds=backoff,
                                   observations_so_far=observations)
                        time.sleep(backoff)
                        continue
                    finish_reason = "PERCEPTION_FAILURE"
                    self._state = ExplorerState.FAILED
                    break
                except Exception as exc:
                    last_perception_error = PerceptionFailure(
                        f"{type(exc).__name__}: {exc}",
                        code="UNKNOWN_PERCEPTION_ERROR",
                        recoverable=True,
                        detail=traceback.format_exc(),
                    )
                    self._emit("observer_error", error=f"{type(exc).__name__}: {exc}",
                               traceback=traceback.format_exc())
                    if perception_retries < self.max_perception_retries:
                        perception_retries += 1
                        backoff = float(perception_retries)
                        self._emit("observer_retry", attempt=perception_retries,
                                   error=f"{type(exc).__name__}: {exc}",
                                   backoff_seconds=backoff,
                                   observations_so_far=observations)
                        time.sleep(backoff)
                        continue
                    finish_reason = "PERCEPTION_FAILURE"
                    self._state = ExplorerState.FAILED
                    break
            if self._control_interrupted(generation):
                continue
            if finish_reason:
                # 计划书 §8.5：最终失败信息必须精准，不再只显示“未分类异常”。
                # （session_finish 由循环出口统一发射，这里只保存详情。）
                if last_perception_error is not None:
                    self._perception_failure_detail = {
                        **last_perception_error.to_dict(),
                        "attempts": perception_retries + 1,
                    }
                break
            observation_done_perf = time.perf_counter()
            observations += 1
            if observation.target_state != TARGET_POSSIBLE:
                self._possible_reobserve_count = 0
            pose = self._backend.get_pose()
            observation.pose = pose.to_dict() if pose is not None else None
            # 计划书 §6.3 / 不变量 2：navigation heading sector 永远由当前
            # capture pose 计算；不能因为 observer 塞了一个 stale sector 就跳过。
            if pose is not None:
                sector_deg = 360.0 / max(1, self.policy.candidates.heading_sectors)
                navigation_sector = int(
                    round(pose.yaw * 180.0 / 3.141592653589793 / sector_deg)
                ) % max(1, self.policy.candidates.heading_sectors)
                observation.navigation_heading_sector = navigation_sector
                observation.heading_sector = navigation_sector
            self._emit("observation", bundle_id=observation.bundle_id,
                       objects=observation.object_labels,
                       scene_objects=observation.scene_objects,
                       scene_relations=observation.scene_relations,
                       target_present=observation.target_present,
                       target_state=observation.target_state,
                       heading_sector=observation.heading_sector,
                       navigation_heading_sector=observation.navigation_heading_sector,
                       semantic_status=observation.semantic_status,
                       semantic_quality=observation.semantic_quality,
                       semantic_source_frame_id=observation.semantic_source_frame_id,
                       semantic_age_ms=observation.semantic_age_ms,
                       spatial_quality=observation.spatial_quality,
                       pose=observation.pose,
                       sensor_health=observation.sensor_health,
                       image_ref=observation.image_ref)

            # ---- MATCH ------------------------------------------------------
            self._state = ExplorerState.MATCH
            match_start_perf = time.perf_counter()
            try:
                match = self._matcher(observation)
            except Exception as exc:
                self._emit("matcher_error", error=f"{type(exc).__name__}: {exc}")
                match = SemanticMatch(has_candidate=False)
            match_done_perf = time.perf_counter()
            if self._control_interrupted(generation):
                continue
            if not isinstance(match, SemanticMatch):
                match = SemanticMatch(has_candidate=bool(getattr(match, "has_candidate", False)))
            if observation.target_state == TARGET_POSSIBLE:
                # The observer's tri-state is the hard safety boundary even
                # when a legacy matcher only understands target_present.
                match.has_candidate = True
                match.target_state = TARGET_POSSIBLE
                match.target_match_level = "possible"
            self._emit("match", has_candidate=match.has_candidate,
                       target_state=observation.target_state,
                       target_match_level=match.target_match_level,
                       target_score=match.target_score,
                       anchor_labels=match.anchor_labels,
                       directive=match.directive.to_dict() if match.directive is not None and hasattr(match.directive, "to_dict") else None,
                       graph_match=match.graph_match.to_dict() if match.graph_match is not None and hasattr(match.graph_match, "to_dict") else None)

            # ---- VERIFY -----------------------------------------------------
            verification: VerificationOutcome | None = None
            if match.has_candidate:
                self._state = ExplorerState.VERIFY
                for attempt in range(1, max(1, budget.verify_attempts) + 1):
                    try:
                        verification = self._verifier(observation, match)
                    except Exception as exc:
                        # A verification failure must never kill the session:
                        # record it as unconfirmed and keep exploring.
                        verification = VerificationOutcome(
                            confirmed=False, attempts=attempt,
                            reason_zh=f"verifier error: {type(exc).__name__}: {exc}",
                            details={"error": traceback.format_exc()},
                            # 复核**抛异常**不是"目标不存在"的证据（§5.1.5）。
                            status=VERIFY_STATUS_UNKNOWN,
                        )
                        self._emit("verification_error", attempt=attempt,
                                   error=f"{type(exc).__name__}: {exc}")
                    verify_attempts += 1
                    self._emit("verification", attempt=attempt,
                               confirmed=verification.confirmed,
                               reason_zh=verification.reason_zh)
                    if verification.confirmed:
                        break
                    if attempt < budget.verify_attempts:
                        # small re-observe between verify attempts
                        time.sleep(1.0)
                    if self._control_interrupted(generation):
                        break
                if self._control_interrupted(generation):
                    continue
                if (verification is not None and verification.confirmed
                        and observation.provenance.get('target_spatial_contract') == 'capture_v1'):
                    from app.perception.target_spatial import selected_current_target, target_identity
                    expected = target_identity(selected_current_target(observation))
                    if expected is None or verification.details.get('target_identity') != expected:
                        self._backend.stop()
                        self._emit('verification_identity_mismatch',
                            reason='复核结果未绑定当前目标采集，保持停车重观测')
                        self._state = ExplorerState.OBSERVE
                        continue
                if verification is not None and verification.confirmed:
                    # A target can be confirmed on the first frame, before the
                    # normal UPDATE_MEMORY phase.  Persist that observation in
                    # the semantic graph first so the terminal state still
                    # contains a Place, Object evidence and map provenance.
                    spatial_update = self.semantic_graph.update_observation(
                        observation_id=observation.bundle_id,
                        heading_sector=observation.heading_sector,
                        scene_objects=observation.scene_objects,
                        scene_relations=observation.scene_relations,
                        object_update_result=observation.object_update_result,
                        pose=observation.pose,
                        timestamp=observation.timestamp,
                        target_candidate=True,
                    )
                    self._current_place_id = spatial_update["place_id"]
                    # Stamp the confirmation before the snapshot; otherwise the
                    # only semantic graph the WebUI ever receives is the
                    # pre-confirmation one and its topology shows no confirmed
                    # target at all.
                    self.graph.mark_target_confirmed(self._current_node_id(observation))
                    confirmed_object_id = self._confirmed_target_object_id(
                        observation, spatial_update, verification=verification
                    )
                    self.semantic_graph.mark_target_confirmed(
                        object_id=confirmed_object_id,
                        observation_id=observation.bundle_id,
                    )
                    # 计划书 §2.1：普通事件不再携带整张图。图由 worker 在
                    # revision 变化时单独发一次 graph_snapshot；这里只报版本号，
                    # 上面两步 mark_target_confirmed 已经把 revision 推进了。
                    self._emit(
                        "memory_update",
                        node_id=self._current_node_id(observation),
                        place_id=self._current_place_id,
                        new_labels=observation.object_labels,
                        new_relations=[],
                        new_sector=True,
                        information_gain=1.0,
                        no_information_cycles=0,
                        graph_revision=self.semantic_graph.revision,
                        unique_nodes=len(self.graph.nodes),
                        unique_places=len(self.semantic_graph.place_graph.places),
                        unique_objects=len(self.semantic_graph.object_map.objects),
                        frontiers_discovered=len(self.semantic_graph.frontiers),
                        map_nodes_total=self.semantic_graph.node_count,
                    )
                    # 计划书 §18.2：确认只是接近的开始。只有 legacy 开关打开
                    # 时才允许"远处看到就结束"。
                    approach_detail: dict[str, Any] = {}
                    reached = True
                    if not self.finish_on_visual_confirmation and self.search_policy.target_approach.enabled:
                        self._state = ExplorerState.TARGET_CONFIRMED
                        self._emit("target_confirmed",
                                   reason_zh=verification.reason_zh,
                                   attempts=verification.attempts,
                                   place_id=self._current_place_id,
                                   object_id=confirmed_object_id,
                                   observation_id=observation.bundle_id)
                        if (observation.provenance.get('target_spatial_contract') == 'capture_v1'
                                and confirmed_object_id is None):
                            # A visual verdict is not a stable metric identity.
                            # Collect fresh captures under the existing time
                            # budget before considering any approach command.
                            self._backend.stop()
                            self._emit('target_position_pending',
                                reason='目标持久身份或稳定定位尚未通过，保持停车重观测')
                            self._state = ExplorerState.OBSERVE
                            continue
                        reached, approach_detail = self._run_target_approach(
                            observation, match, confirmed_object_id
                        )
                    if self._control_interrupted(generation):
                        continue
                    if reached:
                        self._state = ExplorerState.TARGET_FOUND
                        finish_reason = "TARGET_FOUND"
                        self._emit("target_found",
                                   reason_zh=verification.reason_zh,
                                   attempts=verification.attempts,
                                   place_id=self._current_place_id,
                                   object_id=confirmed_object_id,
                                   observation_id=observation.bundle_id,
                                   approach=approach_detail)
                        try:
                            self._backend.stop()
                        except Exception:
                            pass
                        break
                    # 没走到 standoff 就不算成功：继续搜索，由预算决定何时收尾。
                    self._emit("approach_aborted", **approach_detail)
                if verification is not None and not verification.confirmed:
                    self._emit("verification_rejected", reason_zh=verification.reason_zh)
                if observation.target_state == TARGET_POSSIBLE:
                    # Re-enter OBSERVE with the robot stopped.  Once the
                    # bounded fresh-frame budget is spent, downgrade this
                    # candidate to ABSENT for this planning cycle so no stale
                    # uncertainty can authorize a motion command.
                    # §15：重观测上限与间隔走 settings（现值就是原来的 2 / 0.2）。
                    _max_possible_reobserve = (
                        get_settings().go2w_target_possible_verify_attempts
                    )
                    if self._possible_reobserve_count < _max_possible_reobserve:
                        self._possible_reobserve_count += 1
                        try:
                            self._backend.stop()
                        except Exception:
                            pass
                        self._emit(
                            "target_possible_reobserve",
                            attempt=self._possible_reobserve_count,
                            max_attempts=_max_possible_reobserve,
                            reason="POSSIBLE requires a fresh stopped frame before motion",
                        )
                        time.sleep(
                            get_settings().go2w_target_possible_reobserve_delay_seconds
                        )
                        continue
                    # 计划书 §5.1.5：预算耗尽之后怎么落地，取决于**为什么**没确认。
                    #   已终结的否定（复核真的跑完并说"不是"）⇒ 可以降 ABSENT；
                    #   超时 / 报错 / 在途 ⇒ **不许降 ABSENT**（保持 POSSIBLE）——
                    #   那是把"没验出来"当成"验出来是假的"，正是 §3.2 禁止新增的
                    #   那类错误。这两种情形**都要释放本轮的候选位**：释放的是
                    #   "这一轮要不要为它停"，不是"它到底存不存在"；不释放，
                    #   规划会一直围着同一个候选转，搜索再也不前进。
                    terminal_negative = verification_is_terminal_negative(verification)
                    # 观测里的**证据**照旧是 POSSIBLE：那个候选还在，只是没被
                    # 验证过。变化的是**本轮 match 的结论** —— 没结论就是
                    # TARGET_UNKNOWN，不是"不存在"。
                    observation.target_match = {
                        **dict(observation.target_match or {}),
                        "target_present": False,
                        "target_state": (
                            TARGET_ABSENT if terminal_negative else TARGET_POSSIBLE
                        ),
                        "resolution": (
                            "possible_verification_exhausted" if terminal_negative
                            else "possible_verification_inconclusive"
                        ),
                    }
                    match.has_candidate = False
                    match.target_state = (
                        TARGET_ABSENT if terminal_negative else TARGET_UNKNOWN
                    )
                    match.target_match_level = "none"
                    self._emit(
                        "target_possible_resolved_absent" if terminal_negative
                        else "target_possible_unresolved",
                        attempts=self._possible_reobserve_count + 1,
                        target_state=match.target_state,
                        status=(verification.status if verification is not None else ""),
                        reason=(
                            "fresh verification did not confirm target"
                            if terminal_negative
                            else "verification produced no verdict"
                                 "（超时/报错/在途：目标既不确认也不否定）"
                        ),
                    )

            verification_done_perf = time.perf_counter()
            # ---- UPDATE_MEMORY ----------------------------------------------
            self._state = ExplorerState.UPDATE_MEMORY
            info_gain, new_labels, new_relations, new_sector = self._update_memory(
                observation, match, pose
            )
            if match.has_candidate and not (verification and verification.confirmed):
                node = self.graph.get_node(self._current_node_id(observation))
                if node is not None:
                    self.graph.mark_target_candidate(node.node_id)
            if new_labels or new_relations or new_sector or self._last_execution_moved:
                self._no_information_cycles = 0
            else:
                self._no_information_cycles += 1
            # 「原地打转」计数：上一轮的命令是原地转向（按构造不含平移），
            # 而这一轮一点新信息都没换到。连转 3 轮（= ``repeated_heading_limit``）
            # 就是卡死判据里的 NO_PROGRESS。注意这里刻意**不**用
            # ``_no_information_cycles`` 代替：那个计数把"被墙挡住动不了"和
            # "在原地转圈"混在一起，而这两种要的脱困动作不一样。
            learned_something = bool(new_labels or new_relations or new_sector)
            if self._last_command_was_in_place_turn() and not learned_something:
                self._in_place_turn_streak += 1
            else:
                self._in_place_turn_streak = 0
            if learned_something:
                # 这一轮换到了新信息 ⇒ 上一段"没进步"结束，第二个闸门重新武装。
                self._spin_gate_latched = False
            # Per-cycle flag: the next cycle's UPDATE_MEMORY only sees a
            # movement that happened in the cycle just before it.
            self._last_execution_moved = False
            self._emit("memory_update", node_id=self._current_node_id(observation),
                       place_id=self._current_place_id,
                       new_labels=new_labels, new_relations=new_relations,
                       new_sector=new_sector,
                       information_gain=info_gain,
                       no_information_cycles=self._no_information_cycles,
                       # 现场取证要用的第二个计数：``no_information_cycles`` 说
                       # "多少轮没换到新信息"，它说"其中连续几轮是原地转向"。
                       # 少了它，"闸门没开"就无法区分"真没在转"和"判据没长在现场上"。
                       in_place_turn_streak=self._in_place_turn_streak,
                       graph_revision=self.semantic_graph.revision,
                       unique_nodes=len(self.graph.nodes),
                       unique_places=len(self.semantic_graph.place_graph.places),
                       unique_objects=len(self.semantic_graph.object_map.objects),
                       frontiers_discovered=len(self.semantic_graph.frontiers),
                       map_nodes_total=self.semantic_graph.node_count)
            memory_done_perf = time.perf_counter()
            # **2026-10-14（第二个消费者闸门）**：上面那个连跳计数算出来、也喂进了
            # ``_stuck_state``，但整个卡死分类只挂在"连续 N 轮无信息"这一个闸门后面
            # （``max_consecutive_no_information_cycles``，默认 20）。连跳阈值是 3，
            # 也就是说要**先连转 3 次、再一路转到第 20 轮**判据才第一次被问到 ——
            # 真机五条会话最长只跑到 3 轮（``no_information_classified`` 0 次发射）。
            # 连跳自己已经含了"这一轮没有新信息"，到阈值就该问一次，不必再等 17 轮。
            spin_limit = max(
                1, int(self.search_policy.stuck_detection.repeated_heading_limit))
            no_information_gate = (
                self._no_information_cycles
                >= max(1, budget.max_consecutive_no_information_cycles))
            spin_condition = (
                not self.dry_run and self._in_place_turn_streak >= spin_limit)
            spin_gate = spin_condition and not self._spin_gate_latched
            # 只由连跳推开的这一次咨询：``decision`` 是照"连续 20 轮无信息"那个
            # 问题算的，对"为什么在原地转圈"只有 ``stuck_state`` 有意义 ——
            # 所以它只放行「脱困」与「等地图」两个出口，其余结论一律**不采信**
            # 也不消耗重规划预算，落到下面的正常规划（``planning_cycles`` 照常
            # 递增，循环边界不会被绕开）。连转几圈 ≠ 探索穷尽；把它读成
            # "提前收工"是另一种假绿。
            spin_only = spin_gate and not no_information_gate
            if spin_condition:
                # 迟滞：这一段"没进步"问过就不再问，直到换到新信息（见 __init__ 的
                # 注释：没有它，闸门每轮都开，循环只剩 wall-clock 一个边界）。
                self._spin_gate_latched = True
            if no_information_gate or spin_gate:
                # 计划书 §12：低信息先排除卡死/传感器退化/仍有 frontier，
                # 只有真正无可达 frontier 才允许判 SEARCH_EXHAUSTED。
                low_info = self._classify_no_information(
                    observation, pose, navigation_failures)
                stuck_state = low_info.get("stuck_state")
                stuck_value = stuck_state.value if stuck_state is not None else None
                self._emit("no_information_classified",
                           no_information_cycles=self._no_information_cycles,
                           consultation=("spin_gate" if spin_only
                                         else "no_information_cycles"),
                           **{**low_info, "stuck_state": stuck_value})
                if low_info["decision"] == "RECOVERY_REQUIRED":
                    self._run_recovery(observation=observation, pose=pose,
                                       stuck_state=stuck_state,
                                       goal_id=f"recovery_low_info_{planning_cycles}")
                    self._no_information_cycles = 0
                    replans += 1
                    continue
                if low_info["decision"] == "WAITING_FOR_MAP":
                    self._state = ExplorerState.WAITING_FOR_MAP
                    self._waiting_for_map_cycles += 1
                    self._backend.stop()
                    if self._waiting_for_map_cycles > self.max_waiting_for_map_cycles:
                        finish_reason = "MAP_UNAVAILABLE"
                        self._state = ExplorerState.FAILED
                        self._emit("map_unavailable",
                                   waiting_cycles=self._waiting_for_map_cycles,
                                   reason="local safety evidence degraded")
                        break
                    self._emit("waiting_for_map",
                               waiting_cycles=self._waiting_for_map_cycles,
                               reason="local safety evidence degraded",
                               detail_zh="LiDAR 证据不新鲜或几何未校验，机器狗停稳等待")
                    self._no_information_cycles = 0
                    continue
                if not spin_only:
                    if low_info["decision"] == "REPLAN":
                        self._no_information_cycles = 0
                        self._no_information_replans += 1
                        replans += 1
                        continue
                    finish_reason = "SEARCH_EXHAUSTED"
                    self._state = ExplorerState.SEARCH_EXHAUSTED
                    self._emit("search_exhausted",
                               reason=f"{self._no_information_cycles} consecutive no-information cycles",
                               stuck_state=stuck_value,
                               open_frontiers=low_info.get("open_frontiers"))
                    break

            # ---- PLAN --------------------------------------------------------
            self._state = ExplorerState.PLAN
            planning_cycles += 1
            current_yaw_deg = (pose.yaw * 180.0 / 3.141592653589793) if pose is not None else 0.0
            anchor_labels = match.anchor_labels
            # 计划书 §5/§7/§11：候选生成前先取局部安全快照、感知质量与净空
            # 地图。没有注入 provider 时三者均为 None，候选逻辑保持旧行为。
            safety = self._local_safety(observation)
            perception = self._perception_quality(observation, info_gain)
            if safety is not None:
                self._emit("local_safety_snapshot", local_safety=safety.to_dict(),
                           planning_cycles=planning_cycles)
            if perception is not None:
                self._emit("perception_quality",
                           perception_quality=perception.to_dict(),
                           planning_cycles=planning_cycles)
            clearance_map = self._clearance_map(observation, safety)
            self._last_clearance_map = clearance_map
            safety_rejections: list[dict[str, Any]] = []
            self._last_safety_rejections = safety_rejections
            plan_context = self._candidate_plan_context(
                observation=observation,
                safety=safety,
                perception=perception,
                clearance_map=clearance_map,
                safety_rejections=safety_rejections,
            )
            try:
                candidates = self._candidate_generator(
                    observation=observation,
                    graph=self.graph,
                    directive=getattr(match, "directive", None),
                    anchor_labels=anchor_labels,
                    negative_memory=self.negative_memory,
                    negative_target_key=self.negative_target_key,
                    capabilities=self._backend.capabilities(),
                    config=self.policy.candidates,
                    current_yaw_deg=current_yaw_deg,
                    turn_only=self.turn_only,
                    max_candidates=self.policy.candidates.max_candidates,
                    **plan_context,
                )
            except Exception as exc:
                # 计划书 §6.2：候选生成异常是 PLANNING_ERROR，不是搜索穷尽。
                # 第一次：停稳、记录、允许一次新观测后重新规划；同一异常第二次
                # 复现才判 FAILED。
                signature = f"{type(exc).__name__}: {exc}"
                repeated = signature in self._candidate_error_signatures
                self._candidate_error_signatures.append(signature)
                self._emit("candidate_generator_error", error=signature,
                           error_type="PLANNING_ERROR",
                           source="candidate_generator",
                           traceback=traceback.format_exc(),
                           repeated=repeated,
                           action="fail" if repeated else "replan_after_new_observation")
                try:
                    self._backend.stop()
                except Exception:
                    pass
                if repeated:
                    finish_reason = "PLANNING_ERROR"
                    self._state = ExplorerState.FAILED
                    break
                replans += 1
                continue
            # 计划书 §1.5：安全条件造成的拒绝也必须可解释。兜底前进那道闸
            # （`_forward_fallback_block_reason`）算出的原因串在候选生成器
            # 内部只被当布尔用，出了函数就没了 —— 于是"为什么没往前走"在
            # 整条链路上没有一处留得下话。这里用**同一份输入**重算一次
            # （纯函数，同输入同输出，不会与那边漂移），塞进 §29 已有的
            # safety_rejections 通道：它会同时进 candidate_rejected_safety
            # 事件和决策记录的 hard_safety_rejections，WebUI 直接可读。
            forward_blocked = _forward_fallback_block_reason(safety, perception)
            if forward_blocked and self._forward_fallback_could_apply():
                safety_rejections.append({
                    "source": "forward_fallback",
                    "reason": forward_blocked,
                })
            if safety_rejections:
                # 计划书 §29：被硬安全拒掉的 frontier 必须留下证据，绝不能
                # 静默消失成 "没有候选"。
                self._emit("candidate_rejected_safety",
                           rejections=safety_rejections,
                           planning_cycles=planning_cycles)
            viewpoint_provenance = [
                item.provenance for item in candidates
                if isinstance(item.provenance, dict)
                and item.provenance.get("source") == "frontier_viewpoint"
            ]
            if viewpoint_provenance:
                self._emit("viewpoint_candidates",
                           viewpoints=viewpoint_provenance,
                           planning_cycles=planning_cycles)
            if safety is not None:
                # 计划书 §15：LLM 与人一样只看结构化摘要，绝不看原始点云。
                self._last_embodiment_context = self.llm_embodiment_context(
                    safety=safety, perception=perception,
                    viewpoint_provenance=viewpoint_provenance,
                    navigation_failures=navigation_failures,
                )
                self._emit("llm_embodiment_context",
                           **_json_context(self._last_embodiment_context))
            if not candidates:
                decision = self._classify_empty_candidates()
                self._emit("candidates_empty", **decision)
                if decision["classification"] == "WAITING_FOR_MAP":
                    # 计划书 §6.3：地图不新鲜时停稳等待新地图，绝不盲目前进，
                    # 也绝不宣告搜索穷尽。
                    self._state = ExplorerState.WAITING_FOR_MAP
                    self._waiting_for_map_cycles += 1
                    try:
                        self._backend.stop()
                    except Exception:
                        pass
                    if self._waiting_for_map_cycles > self.max_waiting_for_map_cycles:
                        finish_reason = "MAP_UNAVAILABLE"
                        self._state = ExplorerState.FAILED
                        self._emit("map_unavailable",
                                   waiting_cycles=self._waiting_for_map_cycles,
                                   reason=decision["reason"])
                        break
                    self._emit("waiting_for_map",
                               waiting_cycles=self._waiting_for_map_cycles,
                               reason=decision["reason"],
                               detail_zh="地图暂时不新鲜，机器狗停稳等待新地图后重新规划")
                    time.sleep(1.0)
                    continue
                if decision["classification"] == "PLANNING_ERROR":
                    # 有可达 frontier 却拿不到候选：这是规划缺陷，不是穷尽。
                    signature = f"empty_candidates_with_reachable_frontiers:{decision['reason']}"
                    repeated = signature in self._candidate_error_signatures
                    self._candidate_error_signatures.append(signature)
                    try:
                        self._backend.stop()
                    except Exception:
                        pass
                    if repeated:
                        finish_reason = "PLANNING_ERROR"
                        self._state = ExplorerState.FAILED
                        break
                    replans += 1
                    continue
                self._waiting_for_map_cycles = 0
                finish_reason = "SEARCH_EXHAUSTED"
                self._state = ExplorerState.SEARCH_EXHAUSTED
                self._emit("search_exhausted", reason=decision["reason"],
                           **{key: value for key, value in decision.items()
                              if key not in {"reason", "classification"}})
                break
            self._waiting_for_map_cycles = 0
            # Failed goals: exclude sectors that failed >= N times (section
            # 10.5) while alternatives exist; recent-goal tabu avoids
            # immediate repeats.
            recent = self.graph.recent_goal_sequence()
            tabu_sectors = {
                item.get("heading_sector") for item in recent[-2:]
                if item.get("heading_sector") is not None
            }
            tabu_nodes = {
                item.get("target_node_id") for item in recent[-2:]
                if item.get("target_node_id") is not None
            }
            max_failures = max(1, budget.max_navigation_failures_per_goal)
            failed_sectors = {
                sector for sector in range(self.policy.candidates.heading_sectors)
                if self.graph.sector_failure_count(sector) >= max_failures
            }
            exclude_sectors = (tabu_sectors | failed_sectors) if len(candidates) > 2 else set()
            exclude_nodes = tabu_nodes if len(candidates) > 2 else set()
            scored = self._planner(
                candidates,
                graph=self.graph,
                weights=self.policy.scoring,
                current_yaw_deg=current_yaw_deg,
                exclude_node_ids=exclude_nodes,
                exclude_sectors=exclude_sectors,
            )
            if scored is None:
                # 计划书 §6.2：planner 拿到候选却选不出目标是规划缺陷；只有在
                # 真正没有可达区域时才允许判定穷尽。
                decision = self._classify_empty_candidates()
                self._emit("planner_returned_no_goal",
                           candidate_count=len(candidates), **decision)
                if decision["classification"] == "SEARCH_EXHAUSTED":
                    finish_reason = "SEARCH_EXHAUSTED"
                    self._state = ExplorerState.SEARCH_EXHAUSTED
                    self._emit("search_exhausted", reason=decision["reason"],
                               **{key: value for key, value in decision.items()
                                  if key not in {"reason", "classification"}})
                    break
                signature = f"planner_returned_no_goal:{decision['classification']}"
                repeated = signature in self._candidate_error_signatures
                self._candidate_error_signatures.append(signature)
                try:
                    self._backend.stop()
                except Exception:
                    pass
                if repeated:
                    finish_reason = "PLANNING_ERROR"
                    self._state = ExplorerState.FAILED
                    break
                replans += 1
                continue
            goal = scored.goal
            # Emit the full scored ranking (plan book §33-§34) so the WebUI
            # can render the candidate list with per-component scores.
            all_scored: list[ScoredGoal] = []
            for candidate in candidates:
                if candidate.target_node_id in exclude_nodes:
                    continue
                if candidate.heading_sector in exclude_sectors:
                    continue
                try:
                    all_scored.append(
                        score_exploration_goal(
                            candidate,
                            graph=self.graph,
                            weights=self.policy.scoring,
                            current_yaw_deg=current_yaw_deg,
                        )
                    )
                except Exception:  # noqa: BLE001 - one bad goal never kills PLAN
                    continue
            self._emit("candidates",
                       candidates=[item.to_dict() for item in all_scored],
                       selected_goal_id=scored.goal.goal_id,
                       planning_cycles=planning_cycles)
            source = str(goal.provenance.get("source", "unknown"))
            # --spatial-v2 下候选由 spatial_candidate_generator 产出，V1 的 source
            # 白名单匹配不上：source 是 local_goal_executor，路由 goal 甚至没写
            # source。语义与否要看长期意图 intent_type。
            # 形态有两种：run_semantic_exploration.py 的 provenance 覆盖写的是
            # intent.to_dict()，而 LocalGoalExecutor 早退分支保留的是裸字符串
            # （local_goal_executor.py 的 "intent": intent.intent_type）。
            intent = goal.provenance.get("intent")
            intent_type = intent.get("intent_type") if isinstance(intent, dict) else str(intent or "")
            # EXPLORE_FRONTIER 是前沿几何+代价打分，仍是兜底；其余三种（接近/复核
            # 目标、检查语义锚点）都是语义驱动。
            semantic_intent = intent_type in {
                "APPROACH_TARGET", "VERIFY_TARGET", "INSPECT_ANCHOR_REGION"}
            if (source in {"semantic_navigation_directive", "semantic_anchor"}
                    or semantic_intent):
                semantic_goal_selection_count += 1
            else:
                fallback_goal_selection_count += 1
            self._emit("selected_goal", goal=goal.to_dict(),
                       score=round(scored.score, 4),
                       components=scored.components,
                       reasons=scored.reasons,
                       planning_cycles=planning_cycles)
            instruction_done_perf = time.perf_counter()
            self._emit(
                "decision_pipeline_timing",
                planning_cycles=planning_cycles,
                bundle_id=observation.bundle_id,
                observer_ms=round((observation_done_perf - pipeline_start_perf) * 1000, 3),
                observation_handoff_ms=round((match_start_perf - observation_done_perf) * 1000, 3),
                matcher_ms=round((match_done_perf - match_start_perf) * 1000, 3),
                verification_ms=round((verification_done_perf - match_done_perf) * 1000, 3),
                memory_ms=round((memory_done_perf - verification_done_perf) * 1000, 3),
                planning_ms=round((instruction_done_perf - memory_done_perf) * 1000, 3),
                total_ms=round((instruction_done_perf - pipeline_start_perf) * 1000, 3),
                goal_id=goal.goal_id,
            )
            goal_provenance = goal.provenance if isinstance(goal.provenance, dict) else {}
            # 计划书 §16.1 的 place_identity_mismatch。两侧来源本来就不同：
            # scan_place_id 是**目标锚点**记录在 provenance 里的观测地，而
            # _current_place_id 是本周期决策侧算出来的当前 place。不一致意味着
            # 目标锚在另一个 Place 上，先只记录、不停栈 —— 没有真机触发频率就
            # 定阈值，会让搜索被频繁打断（§7.3 后半本次不做）。
            scan_place_id = goal_provenance.get("place_id")
            if (scan_place_id and self._current_place_id
                    and str(scan_place_id) != str(self._current_place_id)):
                self._emit("place_identity_mismatch",
                           decision_place_id=self._current_place_id,
                           local_scan_place_id=scan_place_id,
                           goal_id=goal.goal_id)
            if goal_provenance.get("source") == "frontier_viewpoint":
                self._emit("viewpoint_selected",
                           frontier_id=goal_provenance.get("frontier_id"),
                           viewpoint_id=goal_provenance.get("viewpoint_id"),
                           viewpoint=goal_provenance,
                           planning_cycles=planning_cycles)

            decision = self._make_decision_record(
                observation=observation,
                match=match,
                goal=goal,
                scored=scored,
                candidates=all_scored,
                cycle=planning_cycles,
                safety=safety,
                perception=perception,
                safety_rejections=safety_rejections,
            )
            self.decision_records.append(decision)
            self._emit("decision_recorded", decision=decision.to_dict(),
                       next_motion_command=decision.next_motion_command)

            # ---- EXECUTE -----------------------------------------------------
            # A slow detector or VLM can consume the entire session budget
            # after the while-condition was checked. Recheck immediately
            # before issuing a real goal: the 2026-10-14 supervised trial
            # otherwise sent its third move about 60 s after a 120 s budget.
            if self._motion_budget_expired(goal.goal_id, planning_cycles):
                finish_reason = "TIMEOUT"
                break
            if self._operator_stop_requested:
                finish_reason = "OPERATOR_STOP"
                self._state = ExplorerState.OPERATOR_STOP
                try:
                    self._backend.stop()
                except Exception:
                    pass
                break
            self._state = ExplorerState.EXECUTE
            self._emit("action_start", goal=goal.to_dict(),
                       planning_cycles=planning_cycles,
                       decision_id=decision.decision_id,
                       next_motion_command=decision.next_motion_command)
            # **位姿必须取在下发命令之前**（2026-10-14 修正）。地图系后端的
            # ``_commanded_translation_m`` / ``_commanded_turn_deg`` 都是
            # 「目标点 − 当前位姿」，取**执行之后**的位姿等于在问"机器人最后停
            # 在离目标多远的地方"：走到位了就是 0。实测（``MockMetricBackend``，
            # 成功时把位姿搬到目标点）：走完 3 m 的导航算出 distance=0.000
            # ⇒ ``_did_translate`` 判"没动"；一次 90° 原地转向算出 dyaw=0.000
            # ⇒ ``_last_command_was_in_place_turn`` 判"不是原地转向"，
            # 两条判据在默认路径上**同时变成空操作**。真机 Nav2 只是因为控制器
            # 有到位容差（xy/yaw 未必精确到 0）才偶然还能动，判据的含义则是反的。
            command_pose = self._backend.get_pose()
            handle = self._dispatch_goal(goal, generation)
            if handle is None:
                if self._control_interrupted(generation):
                    continue
                finish_reason = "MAX_STEPS_REACHED"
                break
            self._state = ExplorerState.WAIT_RESULT
            result = self._wait_result(handle, timeout_sec=max(10.0, self.policy.recovery.timeout_retry_count * 15.0),
                                       control_generation=generation)
            planned_only = (self.dry_run and result.status == NavigationStatus.CANCELLED
                and (result.provenance or {}).get('dry_run') is True
                and ((result.provenance or {}).get('candidate_decision') or {}).get('accepted') is True)
            self._emit("navigation_result", goal_id=goal.goal_id,
                       status=result.status.value, message=result.message,
                       dry_run=planned_only, path_checked=planned_only,
                       requested_motion=result.requested_motion,
                       observed_motion=result.observed_motion,
                       wheel_travel=(result.provenance or {}).get("wheel_travel"),
                       elapsed_sec=result.elapsed_sec,
                       # 计划书 §14：每个候选决策一行结构化记录（走廊宽度、最小
                       # 侧向净空、判定、速度上限）。后端已经把它放进了
                       # provenance，这里必须往上带一层——否则它只活在
                       # BackendHealth.details 里，现场没有任何可查的日志。
                       candidate_decision=(result.provenance or {}).get(
                           "candidate_decision"))
            # If this goal physically moved the robot (even without new visual
            # info), it is still active exploration - don't penalize it.
            #
            # "moved" 必须由**观测到的平移**判定：后端报 SUCCEEDED 只说明命令
            # 走完了链路，不说明机器人挪了地方。
            # 只取**一次**位姿（上面 ``command_pose``，下发命令之前那次），判
            # "动没动"和记命令窗口都用它：它回答的是"这条命令要求机器人从哪儿
            # 挪到哪儿"，两次采样不一致会让原地转向的残差非零。
            self._last_execution_moved = self._did_translate(
                goal, result, command_pose)
            self._record_execution_history(goal, command_pose, result)
            updated_decision = decision.with_execution(
                status=(
                    "PLANNED" if planned_only else
                    "SUCCEEDED" if result.succeeded else
                    "REJECTED" if result.status == NavigationStatus.REJECTED else
                    "FAILED"
                ),
                message=result.message,
                requested_motion=result.requested_motion,
                observed_motion=result.observed_motion,
                replan_reason=None if result.succeeded or planned_only else result.message,
                arrival_verified=bool(
                    (result.provenance or {}).get("arrival_verified") is True
                ),
            )
            self.decision_records.append(updated_decision)
            self._emit("decision_recorded", decision=updated_decision.to_dict())
            if self._control_interrupted(generation):
                continue
            if planned_only:
                # A checked plan neither moved nor visited the destination.
                # Do not turn its intentional cancellation into recovery.
                continue
            if (result.provenance or {}).get("target_candidate_interrupt") is True:
                replans += 1
                self._emit(
                    "replan",
                    goal_id=goal.goal_id,
                    status=result.status.value,
                    navigation_failures=navigation_failures,
                    recovery=None,
                    target_candidate_interrupt=True,
                )
                continue
            preflight_rejected = (
                result.status == NavigationStatus.REJECTED
                and (result.provenance or {}).get("motion_action_sent") is False
            )
            if preflight_rejected:
                # No NavigateToPose/Spin Action was sent.  This is a candidate
                # planning outcome, not a physical navigation failure: never
                # command a recovery movement or contaminate stuck counters.
                if self._preflight_rejection_handler is not None:
                    try:
                        self._preflight_rejection_handler(goal, result)
                    except Exception as exc:  # noqa: BLE001 - advisory memory
                        self._emit(
                            "preflight_rejection_handler_error",
                            goal_id=goal.goal_id,
                            error=f"{type(exc).__name__}: {exc}",
                        )
                replans += 1
                self._emit(
                    "candidate_preflight_rejected",
                    goal_id=goal.goal_id,
                    status=result.status.value,
                    reject_reason=(result.provenance or {}).get("reject_reason"),
                    candidate_decision=(result.provenance or {}).get(
                        "candidate_decision"
                    ),
                    navigation_failures=navigation_failures,
                )
                self._emit(
                    "replan",
                    goal_id=goal.goal_id,
                    status=result.status.value,
                    navigation_failures=navigation_failures,
                    recovery=None,
                    preflight_rejected=True,
                )
                continue
            self.graph.record_navigation(
                result,
                goal_type=goal.goal_type,
                requested_motion=result.requested_motion,
                observed_motion=result.observed_motion,
                target_node_id=goal.target_node_id or self._current_node_id(observation),
                source_node_id=self._current_node_id(observation),
                heading_sector=goal.heading_sector,
            )
            if result.succeeded:
                # motion_steps 是"机器狗真的走了几步"。--dry-run-motion 下
                # 运动指令一条都没发（backend 的 dry_run 分支返回 SUCCEEDED
                # 只为把推理链跑完），把它计进去会让报表在机器人一步没动时
                # 报出"走了 N 步" —— 2026-09-15、09-17 两次实测都栽在这。
                # 代价：干跑不再消耗 max_motion_steps，由 max_seconds /
                # max_planning_cycles 兜底（这两条本来就管着干跑的长度）。
                if not self.dry_run:
                    motion_steps += 1
                node_id = goal.target_node_id or self._current_node_id(observation)
                node = self.graph.get_node(node_id)
                if node is not None:
                    self.graph.mark_visited(node.node_id)
                continue
            if result.status == NavigationStatus.OPERATOR_STOP:
                finish_reason = "OPERATOR_STOP"
                self._state = ExplorerState.OPERATOR_STOP
                self._emit("session_finish", result=finish_reason, reason=result.message)
                return self._result(finish_reason, started, planning_cycles, motion_steps,
                                    observations, replans, navigation_failures,
                                    verify_attempts, semantic_goal_selection_count,
                                    fallback_goal_selection_count, finish_reason)
            if result.status == NavigationStatus.TIMEOUT:
                self._backend.cancel(handle)
                if result.attempt <= self.policy.recovery.timeout_retry_count:
                    replans += 1
                    continue
            if result.failed:
                navigation_failures += 1
                node_id = goal.target_node_id or self._current_node_id(observation)
                node = self.graph.get_node(node_id)
                if node is not None and node.navigation_fail_count >= budget.max_navigation_failures_per_goal:
                    self.graph.mark_unreachable(node_id, reason=result.message)
            if self._state != ExplorerState.OPERATOR_STOP:
                self._state = ExplorerState.RECOVER
                # 计划书 §13.2：导航失败必须先停稳、刷新局部安全证据并决策
                # 脱困，再进入下一轮规划；没有 LiDAR 证据时保持旧行为。
                recovery: dict[str, Any] | None = None
                if self._local_safety_provider is not None:
                    safety = self._local_safety(observation)
                    perception = self._perception_quality(observation, 0.0)
                    stuck_state = self._stuck_state(
                        observation, pose, safety, perception, navigation_failures)
                    recovery = self._run_recovery(
                        observation=observation, pose=pose, result=result,
                        stuck_state=stuck_state,
                        goal_id=f"recovery_{planning_cycles}")
                replans += 1
                self._emit("replan", goal_id=goal.goal_id,
                           status=result.status.value,
                           navigation_failures=navigation_failures,
                           recovery=recovery)
        else:
            elapsed = float(self._now()) - started
            if finish_reason == "":
                if budget.time_expired(elapsed):
                    finish_reason = "TIMEOUT"
                elif budget.max_motion_steps > 0 and motion_steps >= budget.max_motion_steps:
                    finish_reason = "MAX_STEPS_REACHED"
                else:
                    finish_reason = "MAX_PLANNING_CYCLES_REACHED"

        if self._operator_stop_requested:
            finish_reason = "OPERATOR_STOP"
            self._state = ExplorerState.OPERATOR_STOP
        # 计划书 §6.2：失败终局（PLANNING_ERROR / MAP_UNAVAILABLE / 感知失败）
        # 不许被压成 FINISHED，否则 WebUI 会把缺陷显示成正常结束。
        if self._state not in {ExplorerState.TARGET_FOUND, ExplorerState.OPERATOR_STOP,
                               ExplorerState.FAILED}:
            self._state = ExplorerState.FINISHED
        if finish_reason == "":
            # 计划书 §6.2：预算耗尽时不能冒充 SEARCH_EXHAUSTED。
            finish_reason = "BUDGET_EXHAUSTED"
        finish_extra: dict[str, Any] = {}
        if self._perception_failure_detail is not None:
            detail = self._perception_failure_detail
            finish_extra = {
                "cause": detail.get("code"),
                "attempts": detail.get("attempts"),
                "last_success_age_s": detail.get("last_success_age_s"),
                "recoverable": detail.get("recoverable"),
                "error": detail.get("message"),
                "error_detail": detail,
            }
        self._emit("session_finish", result=finish_reason,
                   planning_cycles=planning_cycles, motion_steps=motion_steps,
                   observations=observations, unique_nodes=len(self.graph.nodes),
                   replans=replans, navigation_failures=navigation_failures,
                   verify_attempts=verify_attempts, **finish_extra)
        return self._result(finish_reason, started, planning_cycles, motion_steps,
                            observations, replans, navigation_failures,
                            verify_attempts, semantic_goal_selection_count,
                            fallback_goal_selection_count, finish_reason)

    # ---- internals ---------------------------------------------------------

    def _motion_budget_expired(self, goal_id: str, planning_cycles: int | None = None) -> bool:
        """Check the wall-clock deadline at every goal dispatch, including recovery."""
        if self._run_started_at is None:
            return False
        elapsed = float(self._now()) - self._run_started_at
        limit = self.policy.budget.max_search_seconds
        if not self.policy.budget.time_expired(elapsed):
            return False
        self._emit("motion_budget_expired",
                   elapsed_sec=round(elapsed, 3), limit_sec=limit,
                   goal_id=goal_id, planning_cycles=planning_cycles)
        return True

    def _claim_navigation_goal(self, goal_id: str) -> bool:
        """Count attempted goals across search, approach and recovery paths."""
        cap = self._max_navigation_goal_attempts
        if cap > 0 and self._navigation_goal_attempts >= cap:
            self._emit("navigation_goal_budget_reached",
                       goal_id=goal_id, attempts=self._navigation_goal_attempts,
                       limit=cap)
            return False
        self._navigation_goal_attempts += 1
        return True

    def _wait_result(self, handle: Any, *, timeout_sec: float,
                     control_generation: int | None = None) -> NavigationResult:
        timeout_provider = getattr(self._backend,"navigation_timeout_sec",None)
        if callable(timeout_provider):
            recommended = float(timeout_provider())
            if math.isfinite(recommended) and recommended > 0:
                timeout_sec = max(timeout_sec,recommended)
        deadline = float(self._now()) + timeout_sec
        monitor_state: dict[str, Any] = {
            "thread": None,
            "result": False,
            "error": None,
            "last_started": 0.0,
        }

        def run_monitor() -> None:
            try:
                monitor_state["result"] = bool(self._navigation_target_monitor())
            except Exception as exc:  # noqa: BLE001 - monitoring is advisory
                monitor_state["error"] = f"{type(exc).__name__}: {exc}"

        while float(self._now()) < deadline:
            if self._operator_stop_requested:
                return NavigationResult(
                    goal_id=getattr(handle, "goal_id", "goal"),
                    status=NavigationStatus.OPERATOR_STOP,
                    message="operator stop during navigation",
                )
            if self._pause_requested or (control_generation is not None
                    and self._control_interrupted(control_generation)):
                # Pause halts the wait; the loop parks at the pause gate.
                return NavigationResult(
                    goal_id=getattr(handle, "goal_id", "goal"),
                    status=NavigationStatus.CANCELLED,
                    message="operator pause during navigation",
                )
            if control_generation is None:
                result = self._backend.get_navigation_status(handle)
            else:
                # Status polling can itself dispatch NavigateToPose after
                # planning. Keep that transition ordered with operator stop.
                with self._control_lock:
                    if self._control_interrupted(control_generation):
                        continue
                    result = self._backend.get_navigation_status(handle)
            if result.status in TERMINAL_NAVIGATION_STATUSES:
                return result
            if self._navigation_target_monitor is not None:
                monitor_thread = monitor_state.get("thread")
                if isinstance(monitor_thread, threading.Thread) and not monitor_thread.is_alive():
                    if monitor_state.get("error"):
                        self._emit(
                            "navigation_target_monitor_error",
                            error=monitor_state.pop("error"),
                        )
                    if monitor_state.get("result"):
                        self._backend.cancel(handle)
                        self._backend.stop()
                        self._emit(
                            "navigation_target_interrupt",
                            goal_id=getattr(handle, "goal_id", "goal"),
                        )
                        return NavigationResult(
                            goal_id=getattr(handle, "goal_id", "goal"),
                            status=NavigationStatus.CANCELLED,
                            message="high-confidence target candidate detected",
                            provenance={
                                "target_candidate_interrupt": True,
                                "motion_action_sent": True,
                            },
                        )
                    monitor_state["thread"] = None
                now = float(self._now())
                if monitor_state.get("thread") is None and (
                    now - float(monitor_state.get("last_started") or 0.0) >= 1.0
                ):
                    monitor_state["result"] = False
                    monitor_state["error"] = None
                    monitor_state["last_started"] = now
                    monitor_state["thread"] = threading.Thread(
                        target=run_monitor,
                        name="navigation-target-monitor",
                        daemon=True,
                    )
                    monitor_state["thread"].start()
            # nav2 后端的 Action 状态是回调写进去的（RosActionClientPort.poll 的
            # 文档：必须有人持续 spin，否则状态永远停在 UNKNOWN）。这里原来是裸
            # ``time.sleep``，等待窗口内没人 spin ⇒ ``ComputePathToPose`` 的回执
            # 永远读不到，10 s 后按规划超时判 ``NO_PATH``（现场只看到一句
            # "compute path timeout"，看不出是"没人 spin"）。不提供 ``spin_once``
            # 的后端（离线 mock、实验后端）退回 sleep，与原来逐字等价。
            self._pump_backend(0.2)
        return NavigationResult(
            goal_id=getattr(handle, "goal_id", "goal"),
            status=NavigationStatus.TIMEOUT,
            message=f"navigation wait timeout after {timeout_sec:.0f}s",
        )

    # ---- local safety / stuck / recovery (计划书 §11-§14) ------------------

    def _local_safety(self, observation: LiveObservation) -> LocalSafetySnapshot | None:
        if self._local_safety_provider is None:
            return None
        return self._local_safety_provider(observation)

    def _perception_quality(
        self, observation: LiveObservation, information_gain: float
    ) -> PerceptionQuality | None:
        if self._perception_quality_provider is None:
            return None
        return self._perception_quality_provider(observation, float(information_gain))

    def _breadcrumb(self) -> Any:
        if self._breadcrumb_provider is None:
            return None
        return self._breadcrumb_provider()

    def _did_translate(
        self, goal: ExplorationGoal, result: NavigationResult, pose: Any
    ) -> bool:
        """上一轮执行有没有让机器人真的**平移**（不是"转了点角度"）。

        ``pose`` 是**下发这条命令时**的位姿（调用方在 ``execute_goal`` 之前取的
        那一次）。地图系后端要靠它把绝对目标换算成位移，传 None 会让判据退化；
        换成执行之后的位姿则会让"走到了"和"没打算走"这两件事在下游无法区分。

        三段判据，从强到弱：
        0. 命令失败 ⇒ 没挪窝。有些后端（mock）在**失败**结果里照样带上它
           "本来打算走"的位移；把那个当观测值会把"每次运动都失败"读成
           "一直在移动"，`test_no_information_exhausts_only_when_stuck`
           就是这么红掉的。所以先按状态把失败排掉。
        1. ``observed_motion`` 里有位移遥测 ⇒ 以它为准（legacy 后端与 mock 走这里）。
        2. 没有位移遥测，而且这条命令本来就要平移（或整场是干跑，一条运动指令
           都没发）⇒ **没有证据**，保持旧语义。干跑必须走这一支，否则干跑会被
           自己的零位移判成"原地不动"，20 轮后把一次链路自检报成
           SEARCH_EXHAUSTED。
        3. 没有位移遥测，而且这条命令按构造就是原地转向（ROTATE_VIEW /
           INSPECT_ANCHOR，以及地图系后端里"当前坐标 + 换朝向"那种
           NAVIGATE_POSE）⇒ 平移量恒为 0 是构造事实，判"没动"。**这一支就是
           "原地打转"能被判出来的前提。**

        2026-10-14 修正（**默认路径上的一次误判**）：第 2/3 支的分界原本只看
        ``relative_dx``/``relative_dy``，而默认的 nav2 后端
        （``supports_metric_navigation=True``）一个 relative_* 都不写，于是
        **每一条**命令都被算成"按构造原地转向"——一次成功走了 3 米的导航也被判
        "没动"，``_no_information_cycles`` 会在机器人持续探索时照样涨到阈值。
        改之前这里是 ``bool(result.succeeded)``，所以那一版**反而没有**这个毛病。
        现在两种表示都算（见 ``_commanded_translation_m``）。
        """
        if not result.succeeded:
            return False
        displacement = _observed_translation_m(result.observed_motion)
        if displacement is not None:
            return displacement > _MIN_OBSERVED_TRANSLATION_M
        if self.dry_run or _goal_commands_translation(goal, pose):
            return True
        return False

    def _last_command_was_in_place_turn(self) -> bool:
        """上一条已执行的命令是不是"原地转向"（含 dyaw、不含平移）。

        2026-10-14 修正：这里原来要求 ``distance_m == 0.0`` **精确相等**。地图系
        后端的位移是"目标点与当前位姿的距离"，而目标点来自 ``plan_context`` 里
        那份 robot_pose、当前位姿来自执行时另一次 ``get_pose()``，两次采样不同 ⇒
        一次货真价实的原地转向也会剩下 1e-9 量级的残差 ⇒ 判据在**默认的 nav2
        后端上恒为 False**，"原地打转"计数一次都涨不起来。（旧夹具用的是相对
        后端，位移是构造出来的精确 0，所以看不见这一层。）
        """
        if not self._command_history:
            return False
        last = self._command_history[-1]
        return (
            abs(float(last.get("dyaw_deg") or 0.0)) > 0.0
            and abs(float(last.get("distance_m") or 0.0))
            <= _MIN_COMMANDED_TRANSLATION_M
        )

    def _record_execution_history(
        self, goal: ExplorationGoal, pose: Any, result: NavigationResult
    ) -> None:
        """Keep the short pose/command window the stuck detector reads."""
        window = max(1, int(self.search_policy.stuck_detection.history_cycles)) + 1
        if pose is not None:
            self._pose_history.append((float(pose.x), float(pose.y), float(pose.yaw)))
        self._command_history.append({
            "goal_type": goal.goal_type,
            # 两种后端把"要平移多少 / 要转多少"写在**两处不同的字段**上，这里统一
            # 折算（见 ``_commanded_translation_m`` / ``_commanded_turn_deg``）。
            # 图省事直接用 ``goal.relative_*`` 会让地图系后端（nav2，**默认**）
            # 每一条命令都记成 0 位移 0 转角 —— "原地打转"判据在那个后端上恒不成立。
            "distance_m": _commanded_translation_m(goal, pose),
            "dyaw_deg": _commanded_turn_deg(goal, pose),
        })
        self._result_history.append(result.status.value)
        del self._pose_history[:-window]
        del self._command_history[:-window]
        del self._result_history[:-window]

    def _clearance_map(
        self, observation: LiveObservation, safety: LocalSafetySnapshot | None
    ) -> ClearanceMap | None:
        """净空地图：优先用注入的 provider，否则由局部安全快照推导。"""
        if self._clearance_map_provider is not None:
            return self._clearance_map_provider(observation, safety)
        if safety is None:
            return None
        return clearance_map_from_local_safety(
            safety, pose=_pose_xy_yaw(observation.pose)
        )

    # ---- 目标接近闭环（计划书 §18/§21/§23） ------------------------------

    def _fresh_observation(self) -> LiveObservation | None:
        """接近闭环里的一帧新观测；拿不到就返回 None，由调用方保守停车。"""
        try:
            observation = self._observer()
        except Exception as exc:
            self._emit("approach_observer_error", error=f"{type(exc).__name__}: {exc}")
            return None
        if observation is None:
            return None
        pose = self._backend.get_pose()
        observation.pose = pose.to_dict() if pose is not None else None
        return observation

    def _target_scene_object(
        self, observation: LiveObservation
    ) -> dict[str, Any] | None:
        """找出这一帧里代表目标的那个物体。

        live 链路由 ``DepthObjectLocalizer`` 给目标物体打上 ``category="target"``；
        没有该标记时退回标签互相包含的匹配。``observation.spatial_quality`` 说的是
        置信度最高的物体，不一定是目标，所以这里必须逐物体取。"""
        if observation.provenance.get('target_spatial_contract') == 'capture_v1':
            from app.perception.target_spatial import selected_current_target
            return selected_current_target(observation)
        target = (self.target or "").strip().lower()
        fallback: dict[str, Any] | None = None
        for item in observation.scene_objects or ():
            if not isinstance(item, dict):
                continue
            if str(item.get("category") or "").strip().lower() == "target":
                return item
            label = str(item.get("label") or "").strip().lower()
            if fallback is None and target and label and (label in target or target in label):
                fallback = item
        return fallback

    def _target_approach_context(
        self, observation: LiveObservation, *, object_id: str | None = None
    ) -> TargetApproachContext | None:
        """把目标物体的 RGBD 真值装进接近上下文（§19.1）。"""
        item = self._target_scene_object(observation)
        if item is None:
            return None
        target_contract = observation.provenance.get('target_spatial_contract') == 'capture_v1'
        if target_contract:
            from app.perception.target_spatial import validated_target_binding
            bound = validated_target_binding(item, self.semantic_graph.object_map)
            if bound is None or not object_id or bound != object_id:
                return None
        quality = str(item.get("spatial_quality") or "RGB_ONLY")
        depth = item.get("depth_m")
        camera_xyz = item.get("camera_xyz")
        # 只有 metric 等级才谈得上 global 位姿，其余等级坚决不伪造 map_xyz（§20.2）。
        map_xyz: tuple[float, float, float] | None = None
        if quality == "METRIC_RGBD" and object_id:
            entry = self.semantic_graph.object_map.objects.get(object_id)
            if entry is not None and entry.map_xyz is not None:
                map_xyz = tuple(float(value) for value in entry.map_xyz)
        if target_contract:
            # Fresh capture-time TF supplies the point. The durable entry
            # supplies identity/mobility, not another object's averaged point.
            map_xyz = tuple(float(value) for value in item['map_xyz'])
        bearing = robot_bearing_from_camera(item.get("bearing_deg"))
        if target_contract:
            current_pose = observation.pose or {}
            if current_pose.get('frame_id') != 'map' or current_pose.get('quality') != 'metric':
                return None
            try:
                x,y,yaw = _pose_xy_yaw(current_pose)
            except (TypeError,ValueError,OverflowError):
                return None
            if not all(math.isfinite(v) for v in (x,y,yaw)):
                return None
            delta = math.atan2(map_xyz[1]-y,map_xyz[0]-x)-yaw
            bearing = math.degrees(math.atan2(math.sin(delta),math.cos(delta)))
        policy = self.search_policy.target_approach
        return TargetApproachContext(
            object_id=object_id or str(item.get("object_id") or ""),
            label=str(item.get("label") or self.target),
            depth_m=None if depth is None else float(depth),
            # 相机 bearing 图像右为正，机器人系左为正，进决策前必须取反（§21.1）。
            bearing_deg=bearing,
            camera_xyz=None if not camera_xyz else tuple(
                float(value) for value in camera_xyz),
            map_xyz=map_xyz,
            spatial_quality=quality,
            confidence=float(item.get("confidence") or item.get("score") or 0.0),
            desired_standoff_m=policy.preferred_standoff_m,
            min_standoff_m=policy.minimum_standoff_m,
            max_standoff_m=policy.maximum_standoff_m,
            last_seen_timestamp=float(
                (item.get('spatial_provenance') or {}).get('image_timestamp',0.)
                if target_contract else observation.timestamp or 0.0),
        )

    @staticmethod
    def _metric_approach_goal(
        decision: Any,
        observation: LiveObservation,
        best: Any,
        *,
        object_id: str | None,
        steps: int,
    ) -> ExplorationGoal | None:
        """计划书 §6.7：把一步接近动作写成地图系 ``NAVIGATE_POSE``。

        转向写成「原地转到新朝向」，前进写成 standoff ring 里已经过五道门
        的那条候选——两者都是地图系目标，都要经过 Nav2 的规划、走廊分析与
        碰撞监视。表达不出来就返回 ``None``，由调用方停车，绝不退回相对位移。
        """
        if decision.action == ACTION_ROTATE_SMALL:
            # §3.2：没有真实位姿就表达不出「原地转向」——``_pose_xy_yaw`` 这时
            # 会编一个原点，而目标是地图系的，等于叫 Nav2 开回地图原点。按本函数
            # 既有契约返回 None，由调用方停车。
            if not _has_xy(observation.pose):
                return None
            x, y, yaw = _pose_xy_yaw(observation.pose)
            return ExplorationGoal(
                goal_id=f"approach_step_{steps}",
                goal_type=GOAL_NAVIGATE_POSE,
                position=(x, y),
                yaw=math.degrees(yaw) + float(decision.relative_dyaw_deg),
                frame="map",
                semantic_reason=decision.reason,
                provenance={"approach": decision.to_dict(), "object_id": object_id},
            )
        if decision.action == ACTION_APPROACH_FORWARD and best is not None:
            x, y, yaw = best.pose
            return ExplorationGoal(
                goal_id=f"approach_step_{steps}",
                goal_type=GOAL_NAVIGATE_POSE,
                position=(float(x), float(y)),
                yaw=math.degrees(float(yaw)),
                frame="map",
                semantic_reason=decision.reason,
                provenance={
                    "approach": decision.to_dict(),
                    "object_id": object_id,
                    "viewpoint": best.to_dict(),
                },
            )
        return None

    def _run_target_approach(
        self, observation: LiveObservation, match: SemanticMatch,
        object_id: str | None,
    ) -> tuple[bool, dict[str, Any]]:
        """确认之后一步步走到 standoff，停稳后再确认一次（§18/§21/§23）。

        返回 ``(reached, detail)``。任何一步失去 RGBD 真值、前向硬门不通过或运动
        失败都返回 False，让调用方继续探索——绝不假装成功。"""
        policy = self.search_policy.target_approach
        detail: dict[str, Any] = {"object_id": object_id, "steps": 0, "reason": ""}
        generation = self._observation_generation
        current = observation
        lost_count = 0
        steps = 0
        # 计划书 §6.7：metric 后端（nav2）下接近动作也必须表达成地图系
        # NAVIGATE_POSE，相对位移一律不再生成。判定只看能力，不看后端名字。
        metric_backend = bool(
            getattr(self._backend.capabilities(), "supports_metric_navigation", False)
        )

        while steps < 12:
            if self._control_interrupted(generation):
                detail.update(reason="OPERATOR_CONTROL_CHANGED", steps=steps)
                return False, detail
            if current.provenance.get('target_spatial_contract') == 'capture_v1':
                latest_pose = self._backend.get_pose()
                current.pose = latest_pose.to_dict() if latest_pose is not None else None
            best = None
            # ---- TARGET_LOCALIZE ---------------------------------------
            self._state = ExplorerState.TARGET_LOCALIZE
            context = self._target_approach_context(current, object_id=object_id)
            safety = self._local_safety(current)
            perception = self._perception_quality(current, 0.0)
            if context is None or safety is None or perception is None:
                detail["reason"] = "NO_APPROACH_EVIDENCE: 缺少目标 RGBD 或局部安全证据"
                return False, detail
            context.lost_count = lost_count
            self._emit("target_localized", **context.to_dict())

            # ---- APPROACH_PLAN：只有 metric 目标才有 global 视点可排 ----
            # §3.2：视点排序要拿机器人真实位置去算距离/朝向，位姿这一帧丢了就
            # 不能排——``_pose_xy_yaw`` 会把它当成站在原点。少了这个条件，下面
            # 选出来的 best 是「按原点算出来的视点」，照样是地图系目标。位姿缺失
            # 时 best 保持 None，接近动作由 ``_metric_approach_goal`` 返回 None，
            # 调用方停车。
            if (
                context.spatial_quality == "METRIC_RGBD"
                and context.map_xyz is not None
                and _has_xy(current.pose)
            ):
                self._state = ExplorerState.APPROACH_PLAN
                clearance_map = self._clearance_map(current, safety)
                if clearance_map is not None:
                    candidates = generate_target_approach_viewpoints(
                        context,
                        robot_pose=_pose_xy_yaw(current.pose),
                        clearance_map=clearance_map,
                        policy=policy,
                        planner=self._viewpoint_planner,
                    )
                    best = candidates[0] if candidates and candidates[0].hard_safe else None
                    self._emit(
                        "approach_candidate_selected",
                        candidate_count=len(candidates),
                        selected=best.to_dict() if best is not None else None,
                        candidates=[item.to_dict() for item in candidates[:6]],
                    )

            # ---- APPROACH_EXECUTE --------------------------------------
            self._state = ExplorerState.APPROACH_EXECUTE
            decision = decide_approach(
                context,
                safety=safety,
                perception=perception,
                policy=policy,
                clearance=self.search_policy.clearance_navigation,
                capabilities=self._backend.capabilities(),
            )
            self._emit("approach_step", step=steps + 1, **decision.to_dict())

            if decision.action in (
                ACTION_APPROACH_COMPLETE, ACTION_TARGET_REACHED_OR_TOO_CLOSE
            ):
                break
            if decision.action == ACTION_APPROACH_BLOCKED:
                detail.update(reason=decision.reason, steps=steps)
                return False, detail
            if decision.action == ACTION_STOP_REOBSERVE:
                self._backend.stop()
                lost_count += 1
                if lost_count > 2:
                    # 带上最后一次决策原因，否则现场分不清「真丢了」还是
                    # 「深度不新鲜」。
                    detail.update(reason="TARGET_LOST_DURING_APPROACH",
                                  steps=steps, lost_count=lost_count,
                                  decision_reason=decision.reason)
                    return False, detail
                refreshed = self._fresh_observation()
                if refreshed is None:
                    detail.update(reason="NO_FRESH_FRAME_DURING_APPROACH", steps=steps)
                    return False, detail
                current = refreshed
                continue
            if decision.action == ACTION_APPROACH_FORWARD and self.turn_only:
                detail.update(reason="TURN_ONLY_MODE: 只允许转向，无法接近目标",
                              steps=steps)
                return False, detail

            steps += 1
            goal = (
                self._metric_approach_goal(
                    decision, current, best, object_id=object_id, steps=steps
                )
                if metric_backend
                else ExplorationGoal(
                    goal_id=f"approach_step_{steps}",
                    goal_type=(
                        GOAL_ROTATE_VIEW if decision.action == ACTION_ROTATE_SMALL
                        else GOAL_RELATIVE_MOVE
                    ),
                    relative_dx=decision.relative_dx,
                    relative_dyaw=decision.relative_dyaw_deg,
                    semantic_reason=decision.reason,
                    provenance={"approach": decision.to_dict(), "object_id": object_id},
                )
            )
            if goal is None:
                # 计划书 §6.7：metric 后端下表达不出地图系目标就停在这里，
                # 绝不退回相对位移。原因写进 detail，现场能分辨是哪种缺证据。
                detail.update(
                    reason="NO_METRIC_APPROACH_TARGET", steps=steps,
                    decision_reason=decision.reason,
                    action=decision.action,
                )
                return False, detail
            if self._motion_budget_expired(goal.goal_id):
                detail.update(reason="TIMEOUT_BEFORE_APPROACH_MOTION", steps=steps)
                return False, detail
            # 同主循环：位姿取在下发之前，否则"命令要求平移多少"会被算成
            # "最后停在离目标多远"（见 EXECUTE 那一段的注释）。
            approach_command_pose = self._backend.get_pose()
            handle = self._dispatch_goal(goal, generation)
            if handle is None:
                detail.update(reason=("OPERATOR_CONTROL_CHANGED" if
                    self._control_interrupted(generation) else "NAVIGATION_GOAL_BUDGET"), steps=steps)
                return False, detail
            result = self._wait_result(handle, timeout_sec=20.0,
                                       control_generation=generation)
            self._record_execution_history(goal, approach_command_pose, result)
            if result.status is not NavigationStatus.SUCCEEDED:
                detail.update(reason=f"APPROACH_MOTION_{result.status.value}",
                              steps=steps, message=result.message)
                return False, detail
            refreshed = self._fresh_observation()
            if refreshed is None:
                detail.update(reason="NO_FRESH_FRAME_DURING_APPROACH", steps=steps)
                return False, detail
            current = refreshed
        else:
            detail.update(reason="APPROACH_STEP_BUDGET_EXHAUSTED", steps=steps)
            return False, detail

        # ---- FINAL_OBSERVE -> FINAL_VERIFY（§23）----------------------
        self._state = ExplorerState.FINAL_OBSERVE
        self._backend.stop()
        final = self._fresh_observation()
        if self._control_interrupted(generation):
            detail.update(reason="OPERATOR_CONTROL_CHANGED", steps=steps)
            return False, detail
        if final is None:
            detail.update(reason="NO_FINAL_FRAME", steps=steps)
            return False, detail
        self._state = ExplorerState.FINAL_VERIFY
        final_context = self._target_approach_context(final, object_id=object_id)
        final_safety = self._local_safety(final)
        final_perception = self._perception_quality(final, 0.0)
        if final_context is None or final_safety is None or final_perception is None:
            detail.update(reason="NO_FINAL_EVIDENCE", steps=steps)
            return False, detail
        try:
            final_verification = self._verifier(final, match)
            verified_again = bool(
                final_verification is not None and final_verification.confirmed)
        except Exception as exc:
            verified_again = False
            self._emit("verification_error", phase="FINAL_VERIFY",
                       error=f"{type(exc).__name__}: {exc}")
        reached, reason = evaluate_target_reached(
            final_context,
            safety=final_safety,
            perception=final_perception,
            verified_again=verified_again,
            robot_stationary=True,
            policy=policy,
        )
        detail.update(reason=reason, steps=steps, reached=reached,
                      verified_again=verified_again,
                      final_target=final_context.to_dict())
        self._emit("final_verification", reached=reached, reason=reason,
                   verified_again=verified_again, steps=steps,
                   target=final_context.to_dict())
        if not reached:
            return False, detail
        self._state = ExplorerState.TARGET_REACHED
        self._emit("target_reached", **detail)
        return True, detail

    def _forward_fallback_could_apply(self) -> bool:
        """兜底前进这一轮到底有没有可能被发出来。

        与 `candidate_goal_generator` 里那道闸的**其余**四个条件逐条对齐
        （能力表允许相对平移、不是 turn_only、词汇表里有 FORWARD）——
        那边是权威，改那边要同步看这里。这里只决定"该不该把拒绝原因
        报出来"，不参与任何安全判定，报错了也只是多一行诊断。
        """
        if self.turn_only:
            return False
        capabilities = self._backend.capabilities()
        if not getattr(capabilities, "supports_relative_translation", False):
            return False
        allowed = {
            str(item).upper()
            for item in (getattr(capabilities, "allowed_motion_primitives", None) or ())
        }
        return not allowed or "FORWARD" in allowed

    def _candidate_plan_context(
        self,
        *,
        observation: LiveObservation,
        safety: LocalSafetySnapshot | None,
        perception: PerceptionQuality | None,
        clearance_map: ClearanceMap | None,
        safety_rejections: list[dict[str, Any]],
    ) -> dict[str, Any]:
        """计划书 §7/§8：把 frontier / 净空地图 / 视点规划器交给候选生成。

        没有安全证据时返回空字典，自定义 candidate_generator 与旧会话完全不受
        影响。"""
        context: dict[str, Any] = {}
        if self._local_safety_provider is None:
            return context
        context["local_safety"] = safety
        context["perception_quality"] = perception
        if clearance_map is None:
            return context
        frontiers = [
            entry for entry in self.semantic_graph.frontiers.values()
            if str(entry.get("status") or entry.get("state") or "OPEN").upper() == "OPEN"
        ]
        if not frontiers:
            return context
        backend_config = getattr(self._backend, "config", None)
        context.update(
            clearance_map=clearance_map,
            viewpoint_planner=self._viewpoint_planner,
            frontiers=frontiers,
            # 计划书 §3.2：没有有效定位就不许产生地图系目标。``_pose_xy_yaw``
            # 是个「读」的便利函数——喂它一个 None 或半截 dict，它会**编**一个
            # 原点出来（``(0.0, 0.0, 0.0)``），而生成器拿到它之后会当成真实位姿
            # 用：§6.7 的原地观察转向会变成「开到地图原点」，引导点也会落回原点
            # 旁边（生成器里那段注释说的「横穿整张地图」正是这个）。
            # 所以这里只在本帧真的带坐标时才交出去；拿不到就交 None，让生成器
            # 按「没有这条证据」处理——宁可这个候选不出，也不编一个坐标。
            robot_pose=_pose_xy_yaw(observation.pose) if _has_xy(observation.pose) else None,
            # 计划书 §7.5：候选的对称净空分要沿候选路径量左右机身净空，需要
            # 后端订阅到的那份占据栅格。只有 nav2 后端有；没有就传 None，
            # 那一条证据为空、不影响其它打分项。
            occupancy_grid=getattr(self._backend, "occupancy_grid", None),
            traversal_failures={
                str(entry.get("frontier_id")): int(entry.get("failure_count") or 0)
                for entry in frontiers
            },
            forward_step_m=float(
                getattr(backend_config, "max_forward_step_m", 0.20) or 0.20
            ),
            safety_rejections=safety_rejections,
        )
        return context

    def llm_embodiment_context(
        self,
        *,
        safety: LocalSafetySnapshot | None,
        perception: PerceptionQuality | None,
        viewpoint_provenance: Sequence[dict[str, Any]] = (),
        navigation_failures: int = 0,
    ) -> dict[str, Any]:
        """计划书 §15：LLM 推理请求的几何/拓扑/脱困上下文。"""
        frontiers = [
            entry for entry in self.semantic_graph.frontiers.values()
            if str(entry.get("status") or entry.get("state") or "OPEN").upper() == "OPEN"
        ]
        recent: list[str] = []
        for record in reversed(self.decision_records[-5:]):
            place = record.current_place_id
            if place and place not in recent:
                recent.append(place)
        return {
            "local_safety": build_local_safety_context(safety),
            "perception_quality": build_perception_quality_context(perception),
            "topology_context": build_topology_context(
                current_place_id=self._current_place_id,
                recent_place_ids=recent,
                frontiers=frontiers,
                viewpoint_provenance=viewpoint_provenance,
            ),
            "recovery_context": build_recovery_context(
                stuck_state=self._last_stuck_state,
                navigation_failures=navigation_failures,
                recoveries=self._recovery_count,
                breadcrumb_backward_available=self._breadcrumb() is not None,
            ),
        }

    def _stuck_state(
        self,
        observation: LiveObservation,
        pose: Any,
        safety: LocalSafetySnapshot | None,
        perception: PerceptionQuality | None,
        navigation_failures: int,
    ) -> StuckState:
        history = StuckDetectorInput(
            pose_history=tuple(self._pose_history),
            command_history=tuple(self._command_history),
            result_history=tuple(self._result_history),
            local_safety=safety,
            perception_quality=perception,
            # 2026-10-14：这里原本喂 ``graph.sector_visited_count(...)``，它只返回
            # 0/1，而 ``repeated_heading_limit`` 是 3 —— 那个判据在数学上不可能
            # 成立（``stuck_detected`` 在两轮语料里 0 次发射，正是它从没触发过）。
            # 换成"连续几轮只转不平移且没换到新信息"的真实计数。
            repeated_heading_count=int(self._in_place_turn_streak),
            navigation_failures=int(navigation_failures),
        )
        state = detect_stuck(history, policy=self.search_policy.stuck_detection)
        self._last_stuck_state = state
        self._emit("stuck_detected", stuck_state=state.value,
                   navigation_failures=int(navigation_failures),
                   local_safety=safety.summary() if safety is not None else None,
                   perception_quality=perception.to_dict() if perception is not None else None)
        return state

    def _run_recovery(
        self,
        *,
        observation: LiveObservation,
        pose: Any,
        result: NavigationResult | None = None,
        stuck_state: StuckState | None = None,
        goal_id: str = "recovery",
    ) -> dict[str, Any]:
        """计划书 §13.2：停 -> 刷新证据 -> 决策 -> 先退后转，全程 fail-closed。"""
        generation = self._observation_generation
        if self._control_interrupted(generation):
            return {"reason": "OPERATOR_CONTROL_CHANGED", "executed": None}
        self._state = ExplorerState.RECOVER
        self._recovery_count += 1
        self._backend.stop()
        metric_backend = bool(
            getattr(self._backend.capabilities(), "supports_metric_navigation", False)
        )
        safety = self._local_safety(observation)
        breadcrumb = self._breadcrumb()
        current_pose = (
            (float(pose.x), float(pose.y), float(pose.yaw))
            if pose is not None
            else (0.0, 0.0, 0.0)
        )
        hard_clearance = self.search_policy.clearance_navigation.hard_clearance_m
        perception = self._perception_quality(observation, 0.0)
        decision = self._recovery_manager.decide(
            result=result,
            breadcrumb=breadcrumb,
            current_pose=current_pose,
            front_clearance_m=safety.front_clearance_m if safety is not None else None,
            forward_min_clearance_m=hard_clearance,
            camera_occluded=bool(perception is not None and perception.camera_blocked),
            stuck_state=stuck_state.value if stuck_state is not None else None,
            goal_id=goal_id,
            # 计划书 §6.7：metric 后端下倒车恢复表达成地图系 NAVIGATE_POSE。
            metric_navigation=metric_backend,
        )
        payload: dict[str, Any] = {
            "stuck_state": stuck_state.value if stuck_state is not None else None,
            "decision": decision.to_dict(),
            "escape_rotation": None,
            "executed": None,
        }
        self._emit("recovery_triggered", **payload)
        if not decision.trigger:
            # The recovery manager is the authority on whether a physical
            # escape is warranted.  A plain navigation failure can produce
            # trigger="" / should_stop=False; selecting a clearance turn after
            # that decision would silently override it and move the robot for
            # a recovery that the manager explicitly did not request.
            payload["reason"] = decision.reason or "recovery_not_required"
            self._emit("recovery_result", **payload)
            return payload
        if decision.backward_allowed and decision.backward_goal is not None:
            goal = decision.backward_goal
            if self._motion_budget_expired(goal.goal_id):
                payload["reason"] = "TIMEOUT_BEFORE_RECOVERY_MOTION"
                self._emit("recovery_result", **payload)
                return payload
            self._emit("recovery_backward_start", goal=goal.to_dict())
            # 脱困动作也是**真执行过的命令**，必须进命令/位姿窗口：不记的话
            # ``_last_command_was_in_place_turn`` 在下一个周期读到的是**脱困之前**
            # 那条陈旧命令，而脱困转向恰恰就是原地转向原语（metric 后端下是
            # 「当前坐标 + 新朝向」的地图系 NAVIGATE_POSE）——连跳计数会漏数，
            # ``detect_stuck`` 拿到的也是脱困盲的窗口。位姿同主循环，取在下发之前。
            recovery_pose = self._backend.get_pose()
            handle = self._dispatch_goal(goal, generation)
            if handle is None:
                payload["reason"] = ("OPERATOR_CONTROL_CHANGED" if
                    self._control_interrupted(generation) else "NAVIGATION_GOAL_BUDGET")
                self._emit("recovery_result", **payload)
                return payload
            backward_result = self._wait_result(handle, timeout_sec=20.0,
                                                 control_generation=generation)
            self._record_execution_history(goal, recovery_pose, backward_result)
            self._emit("recovery_backward_result", goal_id=goal.goal_id,
                       status=backward_result.status.value,
                       message=backward_result.message,
                       requested_motion=backward_result.requested_motion,
                       observed_motion=backward_result.observed_motion)
            self._backend.stop()
            payload["executed"] = "BACKWARD_RECOVERY"
            payload["status"] = backward_result.status.value
            self._emit("recovery_reobserve",
                       reason="fresh frame required after backward recovery")
            self._emit("recovery_result", **payload)
            return payload
        escape = select_escape_rotation(
            safety,
            candidates_deg=self.search_policy.recovery.escape_turn_candidates_deg,
            rotation_clearance_m=hard_clearance,
        )
        payload["escape_rotation"] = escape.to_dict()
        if not escape.allowed:
            self._backend.stop()
            payload["executed"] = None
            payload["reason"] = escape.reason or decision.reason
            self._emit("recovery_budget_exceeded", reason=payload["reason"],
                       escape_rotation=escape.to_dict())
            self._emit("recovery_result", **payload)
            return payload
        # 计划书 §6.7：metric 后端下脱困转向也是「原地转到新朝向」的地图系
        # NAVIGATE_POSE，不生成相对转动。
        turn_provenance = {
            "recovery_reason": decision.trigger or (payload["stuck_state"] or ""),
            "escape_rotation": escape.to_dict(),
        }
        turn = (
            ExplorationGoal(
                goal_id=f"{goal_id}_escape_turn",
                goal_type=GOAL_NAVIGATE_POSE,
                position=(current_pose[0], current_pose[1]),
                yaw=math.degrees(current_pose[2]) + float(escape.yaw_deg),
                frame="map",
                semantic_reason=f"escape rotation after {payload['stuck_state']}",
                provenance=turn_provenance,
            )
            if metric_backend
            else ExplorationGoal(
                goal_id=f"{goal_id}_escape_turn",
                goal_type=GOAL_ROTATE_VIEW,
                relative_dyaw=float(escape.yaw_deg),
                semantic_reason=f"escape rotation after {payload['stuck_state']}",
                provenance=turn_provenance,
            )
        )
        self._emit("recovery_backward_start", goal=turn.to_dict(),
                   primitive="ESCAPE_ROTATION")
        if self._motion_budget_expired(turn.goal_id):
            payload["reason"] = "TIMEOUT_BEFORE_RECOVERY_MOTION"
            self._emit("recovery_result", **payload)
            return payload
        recovery_pose = self._backend.get_pose()  # 同倒车：位姿取在下发之前
        handle = self._dispatch_goal(turn, generation)
        if handle is None:
            payload["reason"] = ("OPERATOR_CONTROL_CHANGED" if
                self._control_interrupted(generation) else "NAVIGATION_GOAL_BUDGET")
            self._emit("recovery_result", **payload)
            return payload
        turn_result = self._wait_result(handle, timeout_sec=20.0,
                                       control_generation=generation)
        self._record_execution_history(turn, recovery_pose, turn_result)
        self._backend.stop()
        payload["executed"] = "ESCAPE_ROTATION"
        payload["status"] = turn_result.status.value
        self._emit("recovery_reobserve",
                   reason="fresh frame required after escape rotation")
        self._emit("recovery_result", **payload)
        return payload

    def _classify_no_information(
        self, observation: LiveObservation, pose: Any, navigation_failures: int
    ) -> dict[str, Any]:
        """计划书 §12：低信息不等于搜索穷尽，先排除卡死与传感器退化。"""
        if self._local_safety_provider is None:
            return {"decision": "SEARCH_EXHAUSTED", "stuck_state": None}
        safety = self._local_safety(observation)
        perception = self._perception_quality(observation, 0.0)
        stuck = self._stuck_state(observation, pose, safety, perception, navigation_failures)
        if requires_recovery(stuck):
            return {"decision": "RECOVERY_REQUIRED", "stuck_state": stuck}
        if stuck is StuckState.SENSOR_DEGRADED:
            return {"decision": "WAITING_FOR_MAP", "stuck_state": stuck}
        # ``semantic_graph.frontiers`` 的元素是**普通 dict**（
        # ``SemanticNavigationGraph`` 里那个 dict 字面量），不是对象。原来写
        # ``getattr(frontier, "status", "")`` ⇒ 对 dict 恒得 "" ⇒
        # ``open_frontiers`` 永远为空、``REPLAN`` 这一支永远不可达 ⇒ 计数一到阈值
        # 就直接 SEARCH_EXHAUSTED 收工，哪怕图上还有 OPEN 前沿。
        # 在 Fix #3 之前的几轮里 ``_no_information_cycles`` 恒为 0，这个缺陷**从来
        # 没被执行到**；计数能被推起来之后它才有后果，所以同批修掉。
        # 同文件 ``_plan_context`` 里那份写法（``entry.get("status") or
        # entry.get("state")``）才是对的。
        open_frontiers = [
            frontier for frontier in self.semantic_graph.frontiers.values()
            if str(frontier.get("status") or frontier.get("state") or "OPEN").upper()
            == "OPEN"
        ]
        if open_frontiers and self._no_information_replans < 2:
            return {"decision": "REPLAN", "stuck_state": stuck,
                    "open_frontiers": len(open_frontiers)}
        return {"decision": "SEARCH_EXHAUSTED", "stuck_state": stuck,
                "open_frontiers": len(open_frontiers)}

    def _classify_empty_candidates(self) -> dict[str, Any]:
        """Classify an empty candidate list against real map/frontier evidence.

        计划书 §6.3 只承认三种情况：地图不新鲜要等地图；地图健康且还有可达
        frontier 说明规划链有缺陷；只有地图健康、方向扫完、可达 frontier 全部
        访问或失败才是真正的 ``SEARCH_EXHAUSTED``。
        """
        probe: dict[str, Any] = {}
        if self._exhaustion_probe is not None:
            try:
                probe = dict(self._exhaustion_probe() or {})
            except Exception as exc:
                probe = {"probe_error": f"{type(exc).__name__}: {exc}"}
        frontiers = self.semantic_graph.frontiers
        reachable = probe.get("reachable_frontier_count")
        if reachable is None:
            reachable = sum(
                1 for item in frontiers.values()
                if str(getattr(item, "status", "OPEN")).upper() == "OPEN"
            )
        visited = probe.get("visited_frontier_count")
        if visited is None:
            visited = sum(
                1 for item in frontiers.values()
                if str(getattr(item, "status", "")).upper() == "VISITED"
            )
        unreachable = probe.get("unreachable_frontier_count")
        if unreachable is None:
            unreachable = sum(
                1 for item in frontiers.values()
                if str(getattr(item, "status", "")).upper() in {"UNREACHABLE", "FAILED"}
            )
        place = self.semantic_graph.place_graph.places.get(self._current_place_id or "")
        scanned_sectors = probe.get("scanned_sector_count")
        if scanned_sectors is None:
            coverage = dict(getattr(place, "heading_coverage", {}) or {}) if place else {}
            scanned_sectors = sum(1 for value in coverage.values() if value)
        evidence = {
            "map_fresh": probe.get("map_fresh"),
            "map_source": probe.get("map_source"),
            "map_revision": probe.get("map_revision"),
            "generator_empty_reason": probe.get("empty_reason"),
            "local_scan_quota_exhausted": probe.get("local_scan_quota_exhausted"),
            # §6.3：区分「位姿没进来」与「候选真没有」——地图系后端拿不到当前位置
            # 就没法把观察转向表达成 NAVIGATE_POSE，两者都表现为同一句
            # ``local_executor_no_primitive``，只看 reason 会把规划缺陷误判成穷尽。
            "robot_pose_present": probe.get("robot_pose_present"),
            "robot_pose_source": probe.get("robot_pose_source"),
            "plan_context_keys": probe.get("plan_context_keys"),
            "capabilities_metric_navigation": probe.get("capabilities_metric_navigation"),
            "intent_type": probe.get("intent_type"),
            "intent_had_preferred_position": probe.get("intent_had_preferred_position"),
            "scanned_sector_count": int(scanned_sectors),
            "reachable_frontier_count": int(reachable),
            "visited_frontier_count": int(visited),
            "unreachable_frontier_count": int(unreachable),
            "place_id": self._current_place_id,
        }
        if probe.get("map_fresh") is False:
            return {
                "classification": "WAITING_FOR_MAP",
                "reason": (
                    "metric map not fresh "
                    f"(source={probe.get('map_source')}, "
                    f"revision={probe.get('map_revision')})"
                ),
                **evidence,
            }
        if int(reachable) > 0:
            return {
                "classification": "PLANNING_ERROR",
                "reason": (
                    f"{int(reachable)} reachable frontier(s) remain but the "
                    "candidate generator produced no goal"
                ),
                **evidence,
            }
        return {
            "classification": "SEARCH_EXHAUSTED",
            "reason": (
                "search space exhausted: "
                f"{int(scanned_sectors)} sector(s) scanned, "
                f"{int(reachable)} reachable / {int(visited)} visited / "
                f"{int(unreachable)} unreachable frontier(s)"
            ),
            **evidence,
        }

    def _current_node_id(self, observation: LiveObservation) -> str:
        # ExplorationGraph remains an internal scoring/recovery ledger keyed
        # by observation bundles.  The exposed navigation map is the separate
        # semantic graph, whose current stable node is ``current_place_id``.
        bundle_id = str(getattr(observation, "bundle_id", "") or "")
        return f"node_{bundle_id}" if bundle_id else "node_unknown"

    def _update_memory(
        self,
        observation: LiveObservation,
        match: SemanticMatch,
        pose: Any,
    ) -> tuple[float, list[str], list[str], bool]:
        spatial_update = self.semantic_graph.update_observation(
            observation_id=observation.bundle_id,
            heading_sector=observation.heading_sector,
            scene_objects=observation.scene_objects,
            scene_relations=observation.scene_relations,
            object_update_result=observation.object_update_result,
            pose=observation.pose,
            timestamp=observation.timestamp,
            target_candidate=observation.target_present,
        )
        self._current_place_id = spatial_update["place_id"]
        node_id = self._current_node_id(observation)
        labels = observation.object_labels
        new_labels = [label for label in labels if label not in self._seen_labels]
        self._seen_labels.update(labels)
        relations = [str(item.get("relation") or "") for item in observation.scene_relations]
        existing_relations = {
            item for node in self.graph.nodes.values() for item in node.relations
        }
        new_relations = [rel for rel in relations if rel and rel not in existing_relations]
        new_sector = (
            observation.heading_sector is not None
            and self.graph.sector_visited_count(observation.heading_sector) == 0
        )
        info_gain = min(
            1.0,
            len(new_labels) * 0.25 + len(new_relations) * 0.2 + (0.3 if new_sector else 0.0),
        )
        node = self.graph.node_or_create(
            node_id,
            pose=pose,
            pose_quality="relative" if pose is not None else "unavailable",
            heading=(pose.yaw * 180.0 / 3.141592653589793) if pose is not None else None,
            heading_sector=observation.heading_sector,
            objects=labels,
            relations=relations,
            scene_graph=observation.scene_graph,
            target_match_level=match.target_match_level,
            target_score=match.target_score,
            semantic_relevance=match.target_score,
            information_gain=info_gain,
            source_bundle_id=observation.bundle_id,
            provenance={"source": "live_observation", "place_id": self._current_place_id},
        )
        # node_or_create keeps kwargs only for new nodes; refresh the view
        # fields on revisits so sector coverage stays correct.
        if observation.heading_sector is not None:
            node.heading_sector = observation.heading_sector
        if pose is not None:
            node.pose = pose
        node = self.graph.add_observation(node)
        if not observation.target_present:
            self.graph.mark_negative(node_id, reason="target not observed in this view")
            if self.negative_memory is not None and observation.heading_sector is not None:
                try:
                    self.negative_memory.add_negative(
                        target_key=self.negative_target_key,
                        heading_sector=observation.heading_sector,
                        reason="当前稳定观察未发现目标",
                        source_event_id=f"explore_{self.session_id}_{node_id}",
                        observation_pose=observation.pose,
                        confidence=0.6,
                    )
                except Exception:
                    pass
        if match.has_candidate:
            self.graph.mark_target_candidate(node_id)
        return info_gain, new_labels, new_relations, new_sector

    def _confirmed_target_object_id(
        self,
        observation: LiveObservation,
        spatial_update: dict[str, Any],
        verification: VerificationOutcome | None = None,
    ) -> str | None:
        """Return the persistent object id of this frame's target candidate.

        The quick VLM puts the confirmed/possible target into scene_objects
        with category "target", so the object map already holds it and the
        association carries frame id -> persistent id.  Label matching is only
        the fallback for frames whose target had no box.
        """
        if observation.provenance.get('target_spatial_contract') == 'capture_v1':
            from app.perception.target_spatial import selected_current_target, target_identity, validated_target_binding
            from app.spatial.target_candidate_map import target_source_id
            selected = selected_current_target(observation)
            identity = target_identity(selected)
            verified = (verification.details.get('target_identity')
                        if verification is not None and verification.confirmed else None)
            if identity is None or verified != identity:
                return None
            bound = validated_target_binding(selected,self.semantic_graph.object_map)
            mapping = spatial_update.get('frame_object_ids') or {}
            if bound is None or mapping.get(target_source_id(identity['entity_id'])) != bound:
                return None
            observation.provenance['verified_target_identity'] = identity
            return bound
        mapping = spatial_update.get("frame_object_ids") or {}
        for item in observation.scene_objects:
            if str(item.get("category") or "") != "target":
                continue
            frame_id = str(item.get("frame_object_id") or item.get("id") or "")
            if frame_id in mapping:
                return mapping[frame_id]
        return self._target_object_id()

    def _target_object_id(self) -> str | None:
        """Return the persistent object whose label is the search target.

        Matching any label of the current frame stamped the first unrelated
        object in insertion order instead: measured search_20260902_180313,
        the confirmed object was a white mesh bin but the flag landed on
        obj_005 "纸箱" (STALE, 4.17 m, last seen 770 s earlier) because
        "纸箱" happened to be in that frame.  When the target itself is not
        in the object map the confirmation stays on the Place only.
        """
        objects = self.semantic_graph.object_map.objects
        for object_id, entry in objects.items():
            if entry.label == self.target:
                return object_id
        # The scene VLM writes the same bin as "绿色垃圾桶" in one frame and
        # "浅绿色垃圾桶" in the next, so exact equality alone leaves the
        # confirmation on the Place.  Containment either way still rejects an
        # unrelated object: "白色垃圾桶" neither contains nor is contained by
        # "绿色垃圾桶" or "纸箱".
        for object_id, entry in objects.items():
            if entry.label and (entry.label in self.target or self.target in entry.label):
                return object_id
        return None

    def _make_decision_record(
        self,
        *,
        observation: LiveObservation,
        match: SemanticMatch,
        goal: Any,
        scored: ScoredGoal,
        candidates: list[ScoredGoal],
        cycle: int,
        safety: LocalSafetySnapshot | None = None,
        perception: PerceptionQuality | None = None,
        safety_rejections: list[dict[str, Any]] | None = None,
    ) -> DecisionRecord:
        decision_id = f"decision_{self.session_id}_{cycle}_{len(self.decision_records) + 1}"
        provenance = goal.provenance if isinstance(goal.provenance, dict) else {}
        command = make_motion_command(
            plan_id=f"live_plan_{self.session_id}_{cycle}",
            decision_id=decision_id,
            turn_deg=float(goal.relative_dyaw or 0.0),
            forward_m=float(goal.relative_dx or 0.0),
            reason_zh="；".join(scored.reasons) or goal.semantic_reason or "选择当前可达候选",
            target_place_id=self._current_place_id,
            target_frontier_id=provenance.get("frontier_id"),
            safety_limited=True,
        )
        command_values = command.to_dict()
        if command.forward_m > 0.0:
            backend_config = getattr(self._backend, "config", None)
            segment_limit = float(
                getattr(backend_config, "forward_step_m", command.forward_m)
            )
            maximum = float(
                getattr(backend_config, "max_forward_step_m", command.forward_m)
            )
            executable = min(command.forward_m, maximum)
            segment_limit = max(0.01, min(segment_limit, executable))
            segment_count = max(1, int(math.ceil(executable / segment_limit)))
            command_values["forward_m"] = executable
            command_values["instruction_zh"] = (
                f"前进 {executable:.2f} m（分 {segment_count} 段，"
                f"每段不超过 {segment_limit:.2f} m），每段停止并校验"
            )
            command_values["safety_limited"] = bool(
                executable < command.forward_m or segment_count > 1
            )
            command_values["requested_motion"] = {
                "turn_deg": command.turn_deg,
                "planner_forward_m": command.forward_m,
                "forward_m": executable,
                "segment_limit_m": segment_limit,
                "segment_count": segment_count,
                "segmented": segment_count > 1,
            }
            command = type(command)(**command_values)
        # Rebuild with the stable decision id used by the record.
        command = type(command)(**{**command.to_dict(), "decision_id": decision_id})
        live_plan = self._build_live_navigation_plan(goal, observation)
        navigation_plan = (
            live_plan.to_dict()
            if live_plan is not None
            else {
                "plan_id": command.plan_id,
                "planning_frame": "odom",
                "start_pose": observation.pose,
                "goal": goal.to_dict(),
                "executable": True,
                "planner": "relative_goal_fallback",
            }
        )
        try:
            world = dict(self._decision_context_provider() or {}) \
                if self._decision_context_provider is not None else {}
        except Exception:
            world = {}
        selected_candidate_id = (
            provenance.get("frontier_id")
            or provenance.get("viewpoint_id")
            or provenance.get("target_object_id")
        )
        if selected_candidate_id is None and isinstance(provenance.get("intent"), dict):
            intent = provenance["intent"]
            selected_candidate_id = (
                intent.get("target_frontier_id")
                or intent.get("target_object_id")
                or intent.get("target_place_id")
            )
        return DecisionRecord(
            decision_id=decision_id,
            session_id=self.session_id,
            task_id=self.task_context.task_id if self.task_context else f"task_{self.target}",
            cycle=cycle,
            timestamp=float(self._now()),
            raw_task_text=self.task_context.raw_text if self.task_context else self.target,
            canonical_target=self.target,
            map_revision=self.semantic_graph.revision,
            current_place_id=self._current_place_id,
            current_pose=observation.pose,
            target_match_level=match.target_match_level,
            selected_long_term_goal=goal.to_dict(),
            candidate_ranking=[item.to_dict() for item in candidates],
            navigation_plan=navigation_plan,
            next_motion_command=command.to_dict(),
            reason_zh=command.reason_zh,
            evidence=[{"type": "visual_observation", "observation_id": observation.bundle_id}],
            negative_evidence=[],
            requested_motion=command.requested_motion,
            # 计划书 §29：可诊断字段随决策一起落盘。
            local_safety_summary=safety.summary() if safety is not None else None,
            perception_quality=perception.to_dict() if perception is not None else None,
            stuck_state=(
                self._last_stuck_state.value
                if self._last_stuck_state is not None else None
            ),
            selected_frontier_id=provenance.get("frontier_id"),
            selected_viewpoint_id=provenance.get("viewpoint_id"),
            path_min_clearance_m=provenance.get("path_min_clearance_m"),
            path_mean_clearance_m=provenance.get("path_mean_clearance_m"),
            view_quality=provenance.get("view_quality"),
            hard_safety_rejections=list(safety_rejections or []),
            snapshot_id=world.get("snapshot_id"),
            map_id=world.get("map_id"),
            map_epoch=world.get("map_epoch"),
            graph_revision=world.get("graph_revision"),
            grid_revision=world.get("grid_revision"),
            semantic_revision=world.get("semantic_revision"),
            sensor_profile_id=world.get("sensor_profile_id"),
            source_observation_ids=[str(observation.bundle_id)],
            evidence_age_ms=max(
                0.0, (float(self._now()) - float(observation.timestamp)) * 1000.0
            ),
            candidates=list(world.get("candidates") or []),
            selected_candidate_id=(
                str(selected_candidate_id) if selected_candidate_id is not None else None
            ),
            downstream_selected_id=str(goal.goal_id),
            downstream_rejection=None,
            arrival_verified=False,
            target_verified=False,
        )

    def _build_live_navigation_plan(
        self, goal: Any, observation: LiveObservation
    ) -> Any | None:
        """Plan against the current semantic graph when a frontier exists.

        Relative turn goals remain executable primitives for RGB-only robots;
        the shared NavigationPlan is still attached whenever the live graph
        has a concrete frontier for the selected heading.
        """
        sector = getattr(goal, "heading_sector", None)
        if sector is None or self._current_place_id is None:
            return None
        frontier_id = self._unique_frontier_id(int(sector))
        if frontier_id is None:
            return None
        try:
            return plan_live_graph_path(
                self.semantic_graph.to_dict(),
                current_place_id=self._current_place_id,
                goal={"target_frontier_id": frontier_id, "confidence": 0.8},
                robot_pose=observation.pose,
                target_status="target_candidate" if observation.target_present else "target_not_seen",
            )
        except (KeyError, TypeError, ValueError):
            return None

    def _unique_frontier_id(self, sector: int) -> str | None:
        """计划书 §12：frontier node_id 全局唯一（frontier:P<place>:<sector>）。

        UI 短标签 Fxx 只是显示名；node_id 必须跨 Place 不冲突。
        """
        if self._current_place_id is None:
            return None
        sector_deg = 360.0 / max(1, self.policy.candidates.heading_sectors)
        expected_bearing = sector * sector_deg
        for entry in self.semantic_graph.frontiers.values():
            if entry.get("source_place") != self._current_place_id:
                continue
            bearing = float(entry.get("bearing_deg") or 0.0)
            if abs((bearing - expected_bearing + 180.0) % 360.0 - 180.0) <= sector_deg / 2.0:
                return str(entry.get("frontier_id"))
        # 兼容旧数据：唯一的旧式短 ID 也可用。
        legacy = f"F{int(sector) + 1:02d}"
        if legacy in self.semantic_graph.frontiers:
            return legacy
        return None

    def _emit(self, name: str, **details: Any) -> None:
        event: dict[str, Any] = {
            "event": name,
            "state": self._state.value,
            "host_s": round(float(self._now()), 6),
            **details,
        }
        event.setdefault("task_id", self.task_context.task_id if self.task_context else None)
        event.setdefault("executor_id", self.executor_id)
        event.setdefault("worker_generation", self.worker_generation)
        self.events.append(event)
        self._on_event(event)

    def _result(self, finish_reason: str, started: float, planning_cycles: int,
                motion_steps: int, observations: int, replans: int,
                navigation_failures: int, verify_attempts: int,
                semantic_goal_selection_count: int,
                fallback_goal_selection_count: int, reason: str) -> SessionResult:
        duration = max(0.0, float(self._now()) - started)
        summary = {
            "result": finish_reason,
            "target": self.target,
            "duration_s": round(duration, 3),
            "planning_cycles": planning_cycles,
            "motion_steps": motion_steps,
            "observations": observations,
            "unique_nodes": len(self.graph.nodes),
            "unique_places": len(self.semantic_graph.place_graph.places),
            "unique_objects": len(self.semantic_graph.object_map.objects),
            "frontiers_discovered": len(self.semantic_graph.frontiers),
            "map_nodes_total": self.semantic_graph.node_count,
            "replans": replans,
            "navigation_failures": navigation_failures,
            "verify_attempts": verify_attempts,
            "semantic_goal_selection_count": semantic_goal_selection_count,
            "fallback_goal_selection_count": fallback_goal_selection_count,
            "finish_reason": reason,
        }
        if self._perception_failure_detail is not None:
            summary["perception_failure"] = self._perception_failure_detail
        return SessionResult(
            result=finish_reason,
            target=self.target,
            duration_s=duration,
            planning_cycles=planning_cycles,
            motion_steps=motion_steps,
            observations=observations,
            unique_nodes=len(self.graph.nodes),
            unique_places=len(self.semantic_graph.place_graph.places),
            unique_objects=len(self.semantic_graph.object_map.objects),
            frontiers_discovered=len(self.semantic_graph.frontiers),
            map_nodes_total=self.semantic_graph.node_count,
            replans=replans,
            navigation_failures=navigation_failures,
            verify_attempts=verify_attempts,
            semantic_goal_selection_count=semantic_goal_selection_count,
            fallback_goal_selection_count=fallback_goal_selection_count,
            finish_reason=reason,
            session_id=self.session_id,
            summary=summary,
        )

    def save_artifacts(self, session_dir: str | Path) -> Path:
        """Persist compatibility ledger plus the authoritative semantic map."""
        run_dir = Path(session_dir) / self.session_id
        run_dir.mkdir(parents=True, exist_ok=True)
        target = run_dir / "exploration_graph.json"
        graph_path = self.graph.save(target)
        (run_dir / "semantic_map.json").write_text(
            json.dumps(self.semantic_graph.to_dict(), ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        return graph_path
