"""计划书 §6.1：自主搜索的 Nav2 后端。

直接实现 ``app.navigation.robot_backend.RobotBackend`` 需要的最小接口，不新增抽象
层。它只接受三种动作：

* ``NAVIGATE_POSE``：先 ``ComputePathToPose``，路径非空才做走廊分析，分析通过才
  ``NavigateToPose``；
* ``STOP``：取消当前目标并让控制链输出停止；
* 取消：``cancel()``。

``RELATIVE_MOVE`` 一律明确拒绝，绝不翻译成底层前进命令。

也不搬 ``scripts/nav2_bridge_worker.py`` 的文件 IPC：本后端通过 ROS 2 Action
直接连 Nav2，搜索进程里本来就有 node 和 TF buffer。

测试用的注入缝
--------------

后端只通过一个很小的端口跟 Action 打交道（``send`` / ``poll`` / ``cancel`` /
``server_ready``）。真实实现是 ``create_nav2_action_clients()`` 返回的 ROS 2
ActionClient 包装，测试用 ``tests/test_nav2_search_backend.py`` 里的假服务器。
这是**一个端口 + 一个真实实现**，不是可插拔后端注册表。
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Protocol, Sequence

from .corridor_analyzer import (
    DECISION_OK,
    MIN_SPEED_MPS,
    PREFERRED_SIDE_GAP_M,
    OCCUPIED_THRESHOLD,
    ROTATION_TRANSLATION_TOL_M,
    ROTATION_MIN_SWEEP_RAD,
    CorridorDecision,
    CorridorGrid,
    ForwardPaddingEgressDecision,
    RotationSweepDecision,
    classify_forward_padding_egress,
    classify_path_clearance,
    classify_rotation_sweep,
    current_footprint_observed_grid,
    measure_path_clearance,
)
from .models import ExplorationGoal
from .nav2_path_utils import compute_path_length
from .trial_budget import NavigationTrialBudget, TrialBudgetError
from .goal_travel_budget import GoalTravelBudget, GoalTravelError
from .localization_health import (
    TF_FUTURE_TOLERANCE_SECONDS,
    TF_STAMP_FRESHNESS_SECONDS,
)
from .robot_backend import (
    TERMINAL_NAVIGATION_STATUSES,
    BackendHealth,
    NavigationHandle,
    NavigationResult,
    NavigationStatus,
    PoseQuality,
    RobotCapabilities,
    RobotPose,
    navigation_result,
)

BACKEND_NAME = "nav2"

# 计划书 §6.1：只接受这三种动作类型（CANCEL 走 cancel()，不占 goal_type）。
SUPPORTED_GOAL_TYPES = ("NAVIGATE_POSE", "STOP")

#: nav2 后端支撑能力的**唯一真值来源**。
#:
#: 它必须是模块级常量而不是 ``capabilities()`` 里的字面量：启动自检
#: （``run_semantic_exploration._probe_readiness``）在 Nav2 后端实例建起来之前
#: 就要出一份报告，那时手上只有后端**种类**、没有实例。它曾经为此自己抄了一份
#: 相对后端的能力表，于是同一次会话里 ``experiment_readiness`` 报
#: ``supports_metric_navigation=false`` 而 ``backend_health`` 报 ``true`` ——
#: 两份报告互相打脸。抄本会和真值分叉，所以这里只留一份。
NAV2_CAPABILITIES = RobotCapabilities(
    supports_global_pose=True,
    supports_metric_navigation=True,
    supports_relative_translation=False,
    supports_relative_rotation=False,
    supports_heading_control=False,
    supports_navigation_cancel=True,
    supports_navigation_feedback=True,
    supports_platform_obstacle_avoidance=True,
    emergency_collision_stop_supported=True,
    clearance_aware_navigation_supported=True,
    autonomous_obstacle_bypass_supported=False,
    allowed_motion_primitives=SUPPORTED_GOAL_TYPES,
)

# 计划书 §14 固定的候选级拒绝原因。
REJECT_NO_MAP = "NO_MAP"
REJECT_NO_LOCALIZATION = "NO_LOCALIZATION"
REJECT_NO_PATH = "NO_PATH"
REJECT_GOAL_OCCUPIED = "GOAL_OCCUPIED"
REJECT_TOO_NARROW = "TOO_NARROW"
REJECT_NAV2_NOT_READY = "NAV2_NOT_READY"

# The 15 W field trace on 2026-11-17 measured one 0.34 s localization-health
# dropout while the raw/structural scan source stamps caught up; the velocity
# bridge selected zero for the whole interval and health recovered by itself.
# Keep that immediate zero-speed behavior, but do not permanently cancel the
# Nav2 action until the invalid state lasts 0.50 s.
LOCALIZATION_CANCELLATION_GRACE_SECONDS = 0.50
# 调用方违反了「Nav2 模式下目标必须是 map 系」的契约，不属于候选判定。
REJECT_NOT_MAP_FRAME = "NOT_MAP_FRAME"

# §14 允许写进候选记录的那六个。收口用它，别处不要再列一遍。
CANDIDATE_REJECT_REASONS = (
    REJECT_NO_MAP,
    REJECT_NO_LOCALIZATION,
    REJECT_NO_PATH,
    REJECT_GOAL_OCCUPIED,
    REJECT_TOO_NARROW,
    REJECT_NAV2_NOT_READY,
)

MAP_FRAME = "map"

# action_msgs/msg/GoalStatus 的常量，这里用字符串以便离线断言。
ACTION_UNKNOWN = "UNKNOWN"
ACTION_ACCEPTED = "ACCEPTED"
ACTION_EXECUTING = "EXECUTING"
ACTION_CANCELING = "CANCELING"
ACTION_SUCCEEDED = "SUCCEEDED"
ACTION_CANCELED = "CANCELED"
ACTION_ABORTED = "ABORTED"

_GOAL_STATUS_CODES = {
    0: ACTION_UNKNOWN,
    1: ACTION_ACCEPTED,
    2: ACTION_EXECUTING,
    3: ACTION_CANCELING,
    4: ACTION_SUCCEEDED,
    5: ACTION_CANCELED,
    6: ACTION_ABORTED,
}

_PHASE_PLANNING = "PLANNING"
_PHASE_NAVIGATING = "NAVIGATING"
_PHASE_APPLYING_SPEED = "APPLYING_SPEED"

# 动作分类（计划书 §7.2）。拿不到位姿时既不算平移也不算原地转向，走廊判据照旧
# 兜底 —— 判据退化总比凭空说「这是转向」安全。
_MODE_TRANSLATION = "translation"
_MODE_ROTATION_IN_PLACE = "rotation_in_place"
_MODE_UNKNOWN = "unknown"

# The normal autonomous-search corridor remains 0.40 m.  A separately
# authorized, one-step collision diagnostic may use a conservative stale map
# to leave known rear padding, while the independent live-scan Collision
# Monitor remains responsible for stopping.  Keep this local threshold above
# the robot padding shell and never apply it to ordinary or frontier search.
COLLISION_DIAGNOSTIC_MIN_SIDE_GAP_M = 0.30


@dataclass(frozen=True)
class Nav2GoalRequest:
    """发往 Nav2 的最小目标描述；``x``/``y`` 为米，``yaw`` 为弧度。"""

    x: float
    y: float
    yaw: float
    frame_id: str = MAP_FRAME

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 6),
        }


@dataclass(frozen=True)
class SpinGoalRequest:
    """发往 Nav2 ``Spin`` behavior 的相对转角与硬时限。"""

    target_yaw: float
    time_allowance_s: float = 15.0


@dataclass(frozen=True)
class ActionPoll:
    """一次 ActionClient 轮询的结果。"""

    status: str = ACTION_UNKNOWN
    path: tuple[tuple[float, float], ...] = ()
    error: str = ""

    @classmethod
    def from_goal_status_code(
        cls, code: int, *, path: Sequence[Any] = (), error: str = ""
    ) -> "ActionPoll":
        return cls(
            status=_GOAL_STATUS_CODES.get(int(code), ACTION_UNKNOWN),
            path=tuple((float(item[0]), float(item[1])) for item in path),
            error=error,
        )


class ActionClientPort(Protocol):
    """后端需要的全部 Action 能力。"""

    def server_ready(self) -> bool: ...

    def send(self, request: Nav2GoalRequest) -> None: ...

    def poll(self) -> ActionPoll: ...

    def cancel(self) -> bool: ...


class RosActionClientPort:
    """``ActionClientPort`` 的唯一真实实现：包一层 rclpy 的 ActionClient。

    rclpy 只在构造与发送时导入，模块顶层保持无 ROS 依赖，离线测试才能直接
    import 这个文件。``poll()`` 读的是回调里记下的状态，因此**必须有人持续
    spin 这个 node**（搜索进程的循环本来就在 spin），否则状态永远停在
    ``UNKNOWN``，最终由后端的规划超时兜底。
    """

    def __init__(
        self,
        node: Any,
        action_name: str,
        action_type: Any,
        *,
        server_wait_s: float = 0.1,
        goal_timeout_s: float = 5.0,
    ) -> None:
        import rclpy.action  # noqa: PLC0415

        self._node = node
        self._action_name = action_name
        self._action_type = action_type
        self._client = rclpy.action.ActionClient(node, action_type, action_name)
        self._server_wait_s = float(server_wait_s)
        self._goal_timeout_s = float(goal_timeout_s)
        self._goal_handle: Any = None
        self._status = ACTION_UNKNOWN
        self._path: tuple[tuple[float, float], ...] = ()
        self._error = ""
        self._goal_lock = threading.RLock()
        self._goal_generation = 0
        self._goal_send_pending = False
        self._cancel_requested = False
        self._cancel_sent = False

    def server_ready(self) -> bool:
        if self._client.server_is_ready():
            return True
        # 只在没就绪时才等一小会儿，避免每次健康检查都卡住搜索循环。
        return bool(self._client.wait_for_server(timeout_sec=self._server_wait_s))

    def send(self, request: Nav2GoalRequest) -> None:
        from geometry_msgs.msg import PoseStamped  # noqa: PLC0415

        goal = self._action_type.Goal()
        pose = PoseStamped()
        pose.header.frame_id = request.frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(request.x)
        pose.pose.position.y = float(request.y)
        pose.pose.orientation.z = math.sin(request.yaw / 2.0)
        pose.pose.orientation.w = math.cos(request.yaw / 2.0)
        _attach_pose(goal, pose)

        self._send_goal(goal)

    def _send_goal(self, goal: Any) -> None:
        # Sending and callback registration share the cancellation lock. A
        # response may already be complete when add_done_callback is called.
        with self._goal_lock:
            self.cancel()
            self._goal_generation += 1
            generation = self._goal_generation
            self._goal_handle = None
            self._status = ACTION_UNKNOWN
            self._path = ()
            self._error = ""
            self._goal_send_pending = True
            self._cancel_requested = False
            self._cancel_sent = False
            try:
                future = self._client.send_goal_async(goal)
            except Exception as error:
                self._goal_send_pending = False
                self._status = ACTION_ABORTED
                self._error = f"{self._action_name}: {error}"
                raise
            future.add_done_callback(
                lambda response: self._on_goal_response(response, generation))

    def poll(self) -> ActionPoll:
        with self._goal_lock:
            return ActionPoll(status=self._status, path=self._path, error=self._error)

    def cancel(self) -> bool:
        with self._goal_lock:
            if not self._goal_send_pending and (
                self._goal_handle is None or self._status in (
                    ACTION_SUCCEEDED, ACTION_CANCELED, ACTION_ABORTED)
            ):
                return False
            self._cancel_requested = True
            self._status = ACTION_CANCELING
            if self._goal_handle is not None and not self._cancel_sent:
                self._cancel_sent = self._cancel_handle(self._goal_handle)
            return True

    def _cancel_handle(self, handle: Any) -> bool:
        try:
            handle.cancel_goal_async()
            return True
        except Exception as error:  # noqa: BLE001
            self._error = f"{self._action_name}: cancel request failed: {error}"
            return False

    # ------------------------------------------------------------ callbacks
    def _on_goal_response(self, future: Any, generation: int) -> None:
        with self._goal_lock:
            try:
                handle = future.result()
            except Exception as error:  # noqa: BLE001
                if generation == self._goal_generation:
                    self._goal_send_pending = False
                    self._status = ACTION_ABORTED
                    self._error = f"{self._action_name}: {error}"
                return
            if generation != self._goal_generation:
                # A superseded send still needs cancellation when its server
                # response arrives, but must never replace the current state.
                if handle is not None and handle.accepted:
                    try:
                        handle.cancel_goal_async()
                    except Exception as error:  # noqa: BLE001
                        self._node.get_logger().error(
                            f"{self._action_name}: retired goal cancel failed: {error}")
                return
            self._goal_send_pending = False
            self._goal_handle = handle
            if handle is None or not handle.accepted:
                self._status = ACTION_ABORTED
                self._error = f"{self._action_name}: goal rejected by the server"
                return
            self._status = ACTION_CANCELING if self._cancel_requested else ACTION_ACCEPTED
            try:
                if self._cancel_requested:
                    self._cancel_sent = self._cancel_handle(handle)
                result_future = handle.get_result_async()
                result_future.add_done_callback(
                    lambda result: self._on_result(result, generation))
            except Exception as error:  # noqa: BLE001
                # A failed cancel RPC is not evidence that the robot stopped.
                # Preserve CANCELING and the diagnostic until a real result.
                self._error = f"{self._action_name}: {error}"

    def _on_result(self, future: Any, generation: int) -> None:
        with self._goal_lock:
            if generation != self._goal_generation:
                return
            try:
                response = future.result()
            except Exception as error:  # noqa: BLE001
                self._status = ACTION_ABORTED
                self._error = f"{self._action_name}: {error}"
                return
            self._status = _GOAL_STATUS_CODES.get(int(response.status), ACTION_UNKNOWN)
            self._path = _extract_path(getattr(response, "result", None))


class RosSpinActionClientPort(RosActionClientPort):
    """Nav2 ``Spin`` 的 ROS 端口；它接收相对角度，不接收地图位姿。"""

    def send(self, request: SpinGoalRequest) -> None:
        goal = self._action_type.Goal()
        goal.target_yaw = float(request.target_yaw)
        seconds = max(0.0, float(request.time_allowance_s))
        goal.time_allowance.sec = int(seconds)
        goal.time_allowance.nanosec = int(round((seconds - int(seconds)) * 1e9))

        self._send_goal(goal)


def _attach_pose(goal: Any, pose: Any) -> None:
    """ComputePathToPose 与 NavigateToPose 的目标字段名不同，这里各自适配。"""
    if hasattr(goal, "goal"):
        goal.goal = pose
    else:
        goal.pose = pose


def _extract_path(result: Any) -> tuple[tuple[float, float], ...]:
    path = getattr(result, "path", None)
    if path is None:
        return ()
    poses = getattr(path, "poses", None) or ()
    return tuple(
        (float(item.pose.position.x), float(item.pose.position.y)) for item in poses
    )


#: 走廊限速写进 RPP 的那一个参数名（计划书 §6.4）。
DESIRED_LINEAR_VEL_PARAM = "FollowPath.desired_linear_vel"

#: 栅格轴与世界轴的对齐容差。`CorridorGrid` 反算坐标时**没有任何旋转项**
#: （`origin + 列行 × 分辨率`），所以只接受与世界轴对齐的地图。
ORIGIN_ROTATION_TOL = 1e-6


class SpeedLimitApplier:
    """`controller_server.desired_linear_vel` 的**异步写入状态机**（计划书 §P3-2）。

    整个后端只有这一处是"发出去一个请求、要等下一轮才知道成不成"，而它的状态是
    **跨目标共享**的：一个在途请求挂在这里，不属于任何一次导航。所以下面这三件事
    不是显然的，必须钉住：

    * 新目标不能把**不属于自己**的 ACK 当成自己的结论；
    * 同一个槽不能把两次**不同速度**的请求混起来；
    * 取消之后**晚到**的响应不能改变后续目标的结论。

    语义（与 `controller_server` 的参数性质绑死）：``desired_linear_vel`` 是**节点级**
    参数，写一次对所有后续 follow_path 都生效。因此"后一个目标复用前一个目标对
    **同一个速度**的 ACK"是正确的，不是继承错误；而速度**不同**时，刚落地的 ACK
    说的是旧值，必须按本段的速度重新发一次。

    返回 True/False/None = 这一段已生效 / 确定没能生效 / 还在等下一轮。``None`` 由
    调用方（`Nav2SearchBackend._advance_speed_limit`）连同自己的超时一起处理。

    与 ROS 无关：``send`` 与 ``service_ready`` 都是注入的，所以能用假 future 直接测。
    """

    def __init__(
        self,
        *,
        send: Callable[[float], Any],
        service_ready: Callable[[], bool],
    ) -> None:
        self._send = send
        self._service_ready = service_ready
        self._pending: dict[str, Any] | None = None

    @property
    def pending_speed_mps(self) -> float | None:
        """在途请求要写的速度；没有在途请求时为 ``None``。报告与测试用。"""
        return None if self._pending is None else float(self._pending["speed"])

    def apply(self, speed_limit_mps: float) -> bool | None:
        if not self._service_ready():
            # 服务不在（控制器没起来）：这一段的限速**没生效**。上层据此拒绝，
            # 而不是照发 NavigateToPose —— 那样走廊算出来的 0.05 只是纸面数字，
            # 控制器会按 YAML 的 0.10 跑（计划书 §7.4 / §3.2）。
            return False
        if self._pending is not None:
            future = self._pending["future"]
            if not future.done():
                return None
            previous_speed = float(self._pending["speed"])
            self._pending = None
            try:
                response = future.result()
                applied = bool(
                    response and response.results
                    and all(item.successful for item in response.results)
                )
                if applied and previous_speed != float(speed_limit_mps):
                    return self.apply(speed_limit_mps)
                return applied
            except Exception:  # noqa: BLE001 - 拿不到结论就是没生效，不猜
                return False
        self._pending = {
            "future": self._send(float(speed_limit_mps)),
            "speed": float(speed_limit_mps),
        }
        return None


def map_message_reject_reason(
    message: Any, *, origin_rotation_tol: float = ORIGIN_ROTATION_TOL
) -> str:
    """这张 `/map` 能不能进走廊分析？返回空串表示可以，否则返回原因（计划书 §P3-5）。

    健康监视器的地图门是 `grid["valid"] is True and grid["frame"] == "map"`
    （`localization_health.evaluate_localization` 的 `map_valid`），那里覆盖了
    frame、宽高、分辨率有限且为正、`len(data) == width*height`、单元格取值合法。
    后端此前**一项都不查**，直接塞进 `CorridorGrid`。要分清两件事：

    * **监视器已覆盖的那些**，后端是*间接*安全的 —— 监视器一判 `valid=False`，
      `/go2w/nav/localization_status` 的 `valid` 就变 False，而后端的
      `localization_valid_provider` 要求它，于是 `_readiness()` 先报
      NO_LOCALIZATION。也就是说"监视器拒图、后端还照跑"在**端到端结果**上不会
      发生，但后端自己并不知道图坏在哪，报告里也看不出。
    * **监视器没覆盖的那些**（原点是不是有限值、origin 四元数是不是单位阵）
      则**完全没有**兜底，而且它们的失败方式不是"判不通过"而是**异常**：分辨率
      NaN 时 `CorridorGrid.value_at` 的 `resolution <= 0.0` 拦不住（NaN 的比较
      恒为 False），随后 `math.floor(NaN)` 抛 ValueError；原点 ±inf 抛
      OverflowError。这些会从走廊分析里冒出来，归因指向完全无关的方向。

    所以这里按同一套约定在入口处验一遍，把"图不能用"变成一条**可读的原因**，
    而不是下游一个异常或一句 TOO_NARROW。

    与 ROS 无关：只读 `header.frame_id` / `info.*` / `data`，鸭子类型即可，
    所以能用普通对象直接测（见 `tests/test_nav2_search_backend.py`）。
    """
    info = message.info
    frame = str(message.header.frame_id or "").lstrip("/")
    if frame != MAP_FRAME:
        return "frame=%r (expected %r)" % (frame, MAP_FRAME)
    resolution = float(info.resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        return "resolution=%r" % (info.resolution,)
    origin = info.origin.position
    if not (math.isfinite(float(origin.x)) and math.isfinite(float(origin.y))):
        return "origin=(%r, %r)" % (origin.x, origin.y)
    width, height = int(info.width), int(info.height)
    if width <= 0 or height <= 0:
        return "size=%dx%d" % (width, height)
    if len(message.data) != width * height:
        return "cells=%d (expected %d)" % (len(message.data), width * height)
    orientation = info.origin.orientation
    rotated = (abs(float(orientation.x)) > origin_rotation_tol
               or abs(float(orientation.y)) > origin_rotation_tol
               or abs(float(orientation.z)) > origin_rotation_tol
               or abs(float(orientation.w) - 1.0) > origin_rotation_tol)
    if rotated:
        return "origin_rotated=(%r, %r, %r, %r)" % (
            orientation.x, orientation.y, orientation.z, orientation.w)
    return ""


def create_nav2_action_clients(
    node: Any,
) -> tuple[RosActionClientPort, RosActionClientPort, RosSpinActionClientPort]:
    """建立规划、平移导航和原地转向三个 Nav2 Action 客户端。"""
    from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin  # noqa: PLC0415
    if os.environ.get("GO2W_FOXY_SPIN_ACTION") == "1":
        if os.environ.get("ROS_DISTRO") != "foxy":
            raise RuntimeError("The duration-capable Foxy Spin profile requires ROS Foxy")
        from go2w_nav2_compat_msgs.action import Spin  # noqa: PLC0415

    return (
        RosActionClientPort(node, "compute_path_to_pose", ComputePathToPose),
        RosActionClientPort(node, "navigate_to_pose", NavigateToPose),
        RosSpinActionClientPort(node, "spin", Spin),
    )


def create_nav2_search_backend(
    node: Any,
    *,
    dry_run: bool = False,
    tf_buffer: Any = None,
    map_topic: str = "/map",
    base_frame: str = "base_link",
    stop_topic: str = "/go2w/search_cmd_vel",
    controller_node: str = "controller_server",
    localization_valid_topic: str = "/go2w/nav/localization_valid",
    tf_lookup_timeout_s: float = 0.2,
    motion_authorizer: Callable[[], bool] | None = None,
    motion_preflight: Callable[[], bool] | None = None,
    motion_releaser: Callable[[], None] | None = None,
    trial_budget: NavigationTrialBudget | None = None,
    max_path_length_m: float = 3.0,
) -> Nav2SearchBackend:
    """把 Nav2 后端接到真实的 ROS 图上。

    各外部输入都是**读到就用、读不到就判不通过**，不缓存、不沿用上一次的值：
    地图、TF、``localization_valid`` 任何一项缺失都会让 ``execute_goal`` 返回
    ``REJECTED``（计划书 §3.2 / §14）。
    """
    # 延迟导入：模块顶层必须保持无 ROS 依赖。
    from geometry_msgs.msg import Twist  # noqa: PLC0415
    from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: PLC0415
    from rcl_interfaces.msg import Parameter as ParameterMsg  # noqa: PLC0415
    from rcl_interfaces.msg import ParameterType, ParameterValue  # noqa: PLC0415
    from rcl_interfaces.srv import SetParameters  # noqa: PLC0415
    from rclpy.duration import Duration  # noqa: PLC0415
    from rclpy.qos import (  # noqa: PLC0415
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from rclpy.time import Time  # noqa: PLC0415
    from std_msgs.msg import Bool, String  # noqa: PLC0415
    from tf2_ros import Buffer, TransformListener  # noqa: PLC0415

    if tf_buffer is None:
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node,
                          qos=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    state: dict[str, Any] = {"grid": None, "localization_valid": False, "localization_at": 0.0,
                             "status_valid": False, "status_stamp": 0.0,
                             "trial_pose": None, "trial_pose_at": 0.0}

    # 最近一次拒图的原因与累计次数。原因要往上走（进 health().details），
    # 否则"没有地图"和"有地图但不合格"在报告里长得一模一样，而两者的现场处置
    # 完全不同（去起 slam_toolbox vs 去看 frame/尺寸）。
    map_reject: dict[str, Any] = {"reason": "", "count": 0}

    def _on_map(message: Any) -> None:
        reason = map_message_reject_reason(message)
        if reason:
            # fail-closed：**不装**这张图，并且把上一张也清掉。留着上一张等于
            # 拿一张刚被判定不可用的图继续放行，正好违反本函数上方那句
            # "不缓存、不沿用上一次的值"。清掉之后 grid=None -> REJECT_NO_MAP。
            state["grid"] = None
            state["map_reject_reason"] = reason
            map_reject["count"] += 1
            # 只在原因**变化**时出声。slam_toolbox 按 map_update_interval 重复
            # 发同一张图，每帧一条会把日志淹掉，等于没有信号。
            if reason != map_reject["reason"]:
                node.get_logger().warn(
                    "拒绝 /map，不进走廊分析（后端按 NO_MAP 处理）：" + reason)
            map_reject["reason"] = reason
            return
        if map_reject["reason"]:
            node.get_logger().info(
                "接受 /map，此前拒绝的原因已消失：" + map_reject["reason"])
            map_reject["reason"] = ""
        info = message.info
        state["grid"] = CorridorGrid(
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            width=int(info.width),
            height=int(info.height),
            data=tuple(int(value) for value in message.data),
        )

    def _on_localization(message: Any) -> None:
        state["localization_valid"] = bool(message.data)
        state["localization_at"] = time.monotonic()

    def _on_status(message: Any) -> None:
        import json
        try:
            payload = json.loads(message.data)
            state["status_valid"] = payload.get("valid") is True
            state["status_stamp"] = float(payload["wall_time"])
        except (ValueError, TypeError, KeyError):
            state["status_valid"] = False

    def _on_trial_odometry(message: Any) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x, position.y,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            state["trial_pose"] = None
            return
        state["trial_pose"] = RobotPose(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
            frame_id=str(message.header.frame_id or "odom_fused"),
            quality=PoseQuality.METRIC,
            timestamp=node.get_clock().now().nanoseconds / 1e9,
            source="topic:/go2w/odom/fused",
        )
        state["trial_pose_at"] = time.monotonic()

    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(OccupancyGrid, map_topic, _on_map, latched)
    node.create_subscription(Bool, localization_valid_topic, _on_localization, 10)
    node.create_subscription(String, "/go2w/nav/localization_status", _on_status, 1)
    node.create_subscription(
        Odometry, "/go2w/odom/fused", _on_trial_odometry, qos_profile_sensor_data
    )

    stop_publisher = node.create_publisher(Twist, stop_topic, 10)
    parameter_client = node.create_client(
        SetParameters, f"/{controller_node}/set_parameters"
    )

    def _send_speed_limit(speed_limit_mps: float):
        """发一次 set_parameters；只负责发，不负责记状态（那是 SpeedLimitApplier）。"""
        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name=DESIRED_LINEAR_VEL_PARAM,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(speed_limit_mps),
                ),
            )
        ]
        return parameter_client.call_async(request)

    speed_limit_applier = SpeedLimitApplier(
        send=_send_speed_limit,
        service_ready=parameter_client.service_is_ready,
    )

    def _pose_provider() -> RobotPose | None:
        try:
            transform = tf_buffer.lookup_transform(
                MAP_FRAME,
                base_frame,
                # 查「已有最新一帧」，不查 node 时钟的 now()。TF 从发布到进 buffer
                # 有几十毫秒延迟，按 now() 查必然外推失败（2026-10-08 实测差
                # 0.037 s：请求 1791439845.3167，最新数据 1791439845.2796）。
                Time(),
                # tf2 的 timeout 只接受 Duration。原写法直接传了 float
                # （``tf_lookup_timeout_s`` 默认 0.2），比较时抛 TypeError，被下面
                # 那句 ``except`` 吃掉 ⇒ ``get_pose()`` 恒返回 None；而 ``health()``
                # 的 pose_quality 取自 readiness、照报 ``metric`` —— 就绪全绿，位姿
                # 其实一直是空的，自主搜索因此连一个地图系目标都表达不出来。
                timeout=Duration(seconds=max(0.0, float(tf_lookup_timeout_s))),
            )
        except Exception:  # noqa: BLE001 - TF 没到就是没位姿，不编造
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        age = node.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec * 1e-9)
        if not (-TF_FUTURE_TOLERANCE_SECONDS <= age
                <= TF_STAMP_FRESHNESS_SECONDS) or not all(math.isfinite(value) for value in (
            translation.x, translation.y, rotation.x, rotation.y, rotation.z, rotation.w
        )):
            return None
        return RobotPose(
            x=float(translation.x),
            y=float(translation.y),
            yaw=yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
            frame_id=MAP_FRAME,
            quality=PoseQuality.METRIC,
            timestamp=node.get_clock().now().nanoseconds / 1e9,
            source="tf:map->base_link",
        )

    path_client, navigate_client, spin_client = create_nav2_action_clients(node)
    travel_budget = None
    if not dry_run and trial_budget is None:
        travel_publisher = node.create_publisher(String, "/go2w/nav/goal_travel/request", 10)
        def send_travel_request(payload):
            import json
            travel_publisher.publish(String(data=json.dumps(payload,allow_nan=False)))
        travel_budget = GoalTravelBudget(send_travel_request)
        def on_travel_status(message):
            import json
            try:
                travel_budget.update(json.loads(message.data))
            except (ValueError,TypeError):
                travel_budget.update(None)
        node.create_subscription(String,"/go2w/nav/goal_travel/status",on_travel_status,1)
    return Nav2SearchBackend(
        path_client=path_client,
        navigate_client=navigate_client,
        spin_client=spin_client,
        occupancy_grid_provider=lambda: state["grid"],
        map_reject_provider=lambda: dict(map_reject),
        localization_valid_provider=lambda: bool(state["localization_valid"])
        and time.monotonic() - state["localization_at"] <= .3
        and state["status_valid"] and -.1 <= time.time() - state["status_stamp"] <= .3,
        pose_provider=_pose_provider,
        trial_pose_provider=lambda: (
            state["trial_pose"]
            if time.monotonic() - state["trial_pose_at"] <= 0.3
            else None
        ),
        speed_limit_setter=speed_limit_applier.apply,
        motion_authorizer=motion_authorizer or (lambda: False),
        motion_preflight=motion_preflight,
        motion_releaser=motion_releaser,
        trial_budget=trial_budget,
        travel_budget=travel_budget,
        translation_speed_mps=.30 if trial_budget is None else None,
        stop_publisher=lambda: stop_publisher.publish(Twist()),
        dry_run=dry_run,
        max_path_length_m=max_path_length_m,
        goal_xy_tolerance_m=.03 if trial_budget is not None else .20,
        goal_yaw_tolerance_rad=.05 if trial_budget is not None else .20,
        # 单调钟：node 的时钟可能被仿真时间或外部对时改动，超时不能跟着跳。
        now=time.monotonic,
    )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """把四元数压成绕 Z 的偏航角。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class _ActiveGoal:
    goal: ExplorationGoal
    handle: NavigationHandle
    phase: str
    request: Nav2GoalRequest
    started_at: float
    plan_sent_at: float
    path_length_m: float = 0.0
    plan_endpoint_error_m: float | None = None
    speed_limit_mps: float = 0.0
    corridor: CorridorDecision | None = None
    rotation: RotationSweepDecision | None = None
    initial_footprint: RotationSweepDecision | None = None
    initial_footprint_egress: ForwardPaddingEgressDecision | None = None
    current_footprint_overlay: dict[str, Any] | None = None
    trial_speed_policy: str = ""
    diagnostic_corridor_policy: str = ""
    # 目标相对当前位姿的平移量；``None`` 表示拿不到位姿，无法分类动作。
    translation_m: float | None = None
    rotation_translation_radius_m: float = 0.0
    motion_mode: str = ""
    planning_pose: RobotPose | None = None
    # 精确原地转向由 Nav2 Spin 执行。Spin 接收相对角，而搜索目标保存绝对 map yaw。
    spin_target_yaw: float | None = None
    tracking_start: RobotPose | None = None
    reject_reason: str = ""
    #: 这一个目标是否**真的**把运动授权（arm）断言上了。
    #: 只有它为真才谈得上"收回来"；没有它就是没武装过，解除武装既无必要也可能
    #: 打掉别人（操作员）正在用的授权。
    armed: bool = False
    #: True only after a Nav2 motion Action has actually been sent.  A goal can
    #: be REJECTED after ComputePath/corridor/trial-envelope checks without ever
    #: moving the robot; the explorer must not mistake that for a physical
    #: navigation failure and command a recovery turn.
    motion_action_sent: bool = False
    #: The velocity bridge remains fail-fast on every false localization
    #: sample.  Keep the Nav2 action alive briefly so one scheduler-stall
    #: sample does not permanently cancel an otherwise healthy bounded goal.
    localization_invalid_since: float | None = None


class Nav2SearchBackend:
    """计划书 §6.1 的 Nav2 后端。"""

    def __init__(
        self,
        *,
        path_client: ActionClientPort,
        navigate_client: ActionClientPort,
        spin_client: ActionClientPort | None = None,
        occupancy_grid_provider: Callable[[], CorridorGrid | None] | None = None,
        localization_valid_provider: Callable[[], bool] | None = None,
        pose_provider: Callable[[], RobotPose | None] | None = None,
        speed_limit_setter: Callable[[float], bool | None] | None = None,
        stop_publisher: Callable[[], None] | None = None,
        dry_run: bool = False,
        now: Callable[[], float] = time.monotonic,
        plan_timeout_s: float = 10.0,
        max_path_length_m: float | None = None,
        motion_authorizer: Callable[[], bool] | None = None,
        motion_preflight: Callable[[], bool] | None = None,
        motion_releaser: Callable[[], None] | None = None,
        trial_budget: NavigationTrialBudget | None = None,
        trial_pose_provider: Callable[[], RobotPose | None] | None = None,
        travel_budget: GoalTravelBudget | None = None,
        map_reject_provider: Callable[[], dict[str, Any]] | None = None,
        translation_speed_mps: float | None = None,
        goal_xy_tolerance_m: float | None = None,
        goal_yaw_tolerance_rad: float | None = None,
    ) -> None:
        self._path_client = path_client
        self._navigate_client = navigate_client
        self._spin_client = spin_client
        self._grid_provider = occupancy_grid_provider
        # 上一张被拒的图**为什么**被拒。没有它，"没有地图"和"有地图但不合格"
        # 在报告里长得一样，而处置完全不同（见 create_nav2_search_backend）。
        self._map_reject_provider = map_reject_provider
        self._localization_valid = localization_valid_provider or (lambda: True)
        self._pose_provider = pose_provider
        self._speed_limit_setter = speed_limit_setter
        self._stop_publisher = stop_publisher
        self._dry_run = bool(dry_run)
        self._now = now
        self._plan_timeout_s = float(plan_timeout_s)
        self._max_path_length_m = max_path_length_m
        self._motion_authorizer = motion_authorizer
        self._motion_preflight = motion_preflight
        # authorize 的**对称半边**（计划书 §P3-3）。只有 authorize 在这里被调过、
        # 而且这一次真的断言上了，才会走到它 —— 见 _release_motion。
        self._motion_releaser = motion_releaser
        self._trial_budget = trial_budget
        self._trial_pose_provider = trial_pose_provider
        if translation_speed_mps is not None and translation_speed_mps != .30:
            raise ValueError('field translation uses the measured working command 0.30 m/s')
        self._translation_speed_mps = translation_speed_mps
        for tolerance in (goal_xy_tolerance_m,goal_yaw_tolerance_rad):
            if tolerance is not None and (not math.isfinite(tolerance) or tolerance<=0):
                raise ValueError('goal tolerance must be finite and positive')
        self._goal_xy_tolerance_m=goal_xy_tolerance_m
        self._goal_yaw_tolerance_rad=goal_yaw_tolerance_rad
        if travel_budget is not None and (max_path_length_m is None
                or not math.isfinite(max_path_length_m) or not 0 < max_path_length_m <= 3.):
            raise ValueError('wheel travel guard requires a finite per-goal limit <= 3 m')
        self._travel_budget = travel_budget
        self._active: _ActiveGoal | None = None
        self._last_decision: dict[str, Any] | None = None

    @property
    def occupancy_grid(self) -> CorridorGrid | None:
        """后端订阅到的最新占据栅格（计划书 §7.5）。

        候选排序要沿候选路径量左右机身净空，那份数据只有这里持有；没有地图时
        返回 ``None``，调用方按「这条证据为空」处理，不编造。
        """
        return self._grid_provider() if self._grid_provider is not None else None

    # ------------------------------------------------------------ RobotBackend
    def capabilities(self) -> RobotCapabilities:
        """支撑能力只暴露地图系导航：相对平移/旋转一律不声明。

        候选生成器按这里的能力表过滤，所以「不再产生相对移动」是靠这张表落地的，
        而不是靠在下游到处加 if。内容在 ``NAV2_CAPABILITIES``（见那里的注释：
        启动自检也要用同一份，不能各抄一份）。
        """
        return NAV2_CAPABILITIES

    def get_pose(self) -> RobotPose | None:
        if self._pose_provider is None:
            return None
        return self._pose_provider()

    def navigation_timeout_sec(self) -> float:
        """Finite action allowance, independent of search session age/count.

        Include planning, the slowest permitted translation and 35 seconds
        for turning/guard handshakes plus 10 seconds of arrival margin.
        Controller progress and sensor watchdogs continue to run throughout.
        """
        distance = self._max_path_length_m if self._max_path_length_m is not None else 3.
        return self._plan_timeout_s + distance/MIN_SPEED_MPS + 45.

    def execute_goal(self, goal: ExplorationGoal) -> NavigationHandle:
        goal_type = str(goal.goal_type or "").strip().upper()
        if goal_type == "STOP":
            self._finish_canceled(goal, "STOP goal: canceled the active Nav2 goal")
            self.stop()
            return NavigationHandle(
                goal_id=goal.goal_id,
                result=self._result(
                    goal.goal_id,
                    NavigationStatus.SUCCEEDED,
                    message="nav2 stop issued",
                    provenance={"backend": BACKEND_NAME, "stop": True},
                ),
            )
        if goal_type not in SUPPORTED_GOAL_TYPES:
            # 计划书 §6.1：RELATIVE_MOVE 必须明确返回不支持。
            return NavigationHandle(
                goal_id=goal.goal_id,
                result=self._result(
                    goal.goal_id,
                    NavigationStatus.REJECTED,
                    message=(
                        f"nav2 backend does not support {goal_type or 'unknown'}; "
                        f"only {', '.join(SUPPORTED_GOAL_TYPES)} are accepted"
                    ),
                    provenance={"backend": BACKEND_NAME, "reject_reason": "UNSUPPORTED_GOAL_TYPE"},
                ),
            )
        if self._active is not None:
            return NavigationHandle(
                goal_id=goal.goal_id,
                result=self._result(
                    goal.goal_id,
                    NavigationStatus.REJECTED,
                    message="nav2 backend already has an active goal",
                    provenance={"backend": BACKEND_NAME, "reject_reason": "GOAL_IN_FLIGHT"},
                ),
            )

        ready, reason = self._readiness()
        if not ready:
            return NavigationHandle(
                goal_id=goal.goal_id,
                result=self._rejected(goal, reason, f"nav2 not usable: {reason}"),
            )

        request, error = _request_from_goal(goal)
        if request is None:
            return NavigationHandle(
                goal_id=goal.goal_id,
                result=self._result(
                    goal.goal_id,
                    NavigationStatus.REJECTED,
                    message=error,
                    provenance={
                        "backend": BACKEND_NAME,
                        "reject_reason": REJECT_NOT_MAP_FRAME
                        if "frame" in error
                        else "INVALID_GOAL",
                    },
                ),
            )

        # 计划书 §3.2：目标点本身落在占据栅格里就直接拒，别指望规划器兜底。
        # NavFn 对占据终点通常返回空路径，结果是同一个「不发 NavigateToPose」，
        # 但拒因会被记成 NO_PATH —— 现场看不出是「过不去」还是「目标就在墙里」，
        # 而 GOAL_OCCUPIED 这个枚举此前没有任何产生点（§14 只允许这六个拒因）。
        # 未知栅格不在这里拦：拦住会把正常的 frontier 目标一起误杀，交给
        # costmap 的 allow_unknown=false 处理。
        grid = self._grid_provider() if self._grid_provider is not None else None
        if grid is not None:
            cell = grid.value_at(request.x, request.y)
            if cell is not None and cell >= OCCUPIED_THRESHOLD:
                return NavigationHandle(
                    goal_id=goal.goal_id,
                    result=self._rejected(
                        goal,
                        REJECT_GOAL_OCCUPIED,
                        f"goal ({request.x:.2f}, {request.y:.2f}) is in an occupied cell "
                        f"(cost={cell})",
                    ),
                )

        handle = NavigationHandle(goal_id=goal.goal_id)
        started = float(self._now())
        self._active = _ActiveGoal(
            goal=goal,
            handle=handle,
            phase=_PHASE_PLANNING,
            request=request,
            started_at=started,
            plan_sent_at=started,
        )
        self._path_client.send(request)
        return handle

    def get_navigation_status(self, handle: NavigationHandle) -> NavigationResult:
        if handle.result is not None:
            return handle.result
        active = self._active
        if active is None or active.handle is not handle:
            return self._result(
                handle.goal_id,
                NavigationStatus.FAILED,
                message="unknown nav2 handle",
                provenance={"backend": BACKEND_NAME},
            )
        localization_valid = self._localization_valid()
        if not localization_valid:
            now = float(self._now())
            if active.localization_invalid_since is None or now < active.localization_invalid_since:
                active.localization_invalid_since = now
            if now - active.localization_invalid_since < LOCALIZATION_CANCELLATION_GRACE_SECONDS:
                # This does not authorize motion: the independent velocity
                # bridge consumes the same false flag and immediately selects
                # zero.  It only defers permanent action cancellation.
                return self._result(
                    handle.goal_id,
                    NavigationStatus.RUNNING,
                    message="waiting for localization health to recover",
                    provenance={"backend": BACKEND_NAME, "localization_hold": True},
                )
            self._path_client.cancel()
            self._navigate_client.cancel()
            if self._spin_client is not None:
                self._spin_client.cancel()
            if self._stop_publisher:
                try:
                    self._stop_publisher()
                except Exception:
                    pass  # Settlement must still close the guard and disarm.
            result = self._rejected(active.goal, REJECT_NO_LOCALIZATION, "localization lost during goal")
        else:
            active.localization_invalid_since = None
        if localization_valid and active.phase == _PHASE_PLANNING:
            result = self._advance_planning(active)
        elif localization_valid and active.phase == "APPLYING_SPEED":
            result = self._advance_speed_limit(active)
        elif localization_valid and active.phase == "STARTING_NAVIGATION":
            result = self._start_guarded_navigation(active)
        elif localization_valid:
            result = self._advance_navigating(active)
        if result is not None:
            # Guarded goals always close their own reservation and arm.
            self._release_motion(
                active, succeeded=result.status is NavigationStatus.SUCCEEDED)
            handle.result = result
            self._active = None
            self._last_decision = result.provenance.get("candidate_decision")
            return result
        return self._result(
            handle.goal_id,
            NavigationStatus.RUNNING,
            message=f"nav2 {active.phase.lower()}",
            provenance={"backend": BACKEND_NAME},
        )

    def cancel(self, handle: NavigationHandle | None = None) -> bool:
        canceled = False
        try:
            if self._path_client.cancel():
                canceled = True
        except Exception:  # noqa: BLE001 - 取消失败也必须继续往下取消导航
            canceled = False
        try:
            if self._navigate_client.cancel():
                canceled = True
        except Exception:  # noqa: BLE001
            canceled = False
        try:
            if self._spin_client is not None and self._spin_client.cancel():
                canceled = True
        except Exception:  # noqa: BLE001
            canceled = False
        active = self._active
        if active is not None and (handle is None or active.handle is handle):
            self._release_motion(active)
            active.handle.result = self._result(
                active.goal.goal_id,
                NavigationStatus.CANCELLED,
                message="nav2 goal canceled",
                provenance={**self._decision_provenance(active, REJECT_NAV2_NOT_READY, False),
                            "backend": BACKEND_NAME, "canceled": True},
            )
            # 结算了这一个 handle 就必须让出槽位，否则下一个 goal 会被
            # ``execute_goal`` 以 ``GOAL_IN_FLIGHT`` 永久拒掉 —— 而唯一会清
            # ``self._active`` 的地方是 ``get_navigation_status`` 走到
            # :603，可这里已经把 ``handle.result`` 填上了，那句提前 return
            # （:587）之后再没人能走到 :603。``_finish_canceled`` 一直是这样
            # 做的（:822），这里是漏了。
            #
            # 现场表现（2026-10-08 真机验收，execution_enabled:=true）：
            # explorer 的 ``_wait_result`` 等 15 s 超时后调 ``cancel(handle)``
            # 准备重试，但从第 2 轮 cycle 起每个候选都被立刻拒成
            # ``"nav2 backend already has an active goal"``，
            # 于是 3 轮 cycle、2 次 navigation_failure、``motion_steps=0``
            # —— 日志里没有任何一行指出是这里。
            self._active = None
        return canceled

    def stop(self) -> None:
        """取消目标并走已有控制链输出停止。"""
        self.cancel()
        if not self._dry_run and self._stop_publisher is not None:
            self._stop_publisher()

    def health(self) -> BackendHealth:
        ready, reason = self._readiness()
        degraded = [] if ready else [reason]
        return BackendHealth(
            ready=ready,
            backend=BACKEND_NAME,
            degraded=degraded,
            capabilities=self.capabilities(),
            pose_quality=PoseQuality.METRIC if ready else PoseQuality.UNAVAILABLE,
            details={
                "reject_reason": reason,
                "localization_valid": self._localization_valid(),
                "last_candidate_decision": self._last_decision,
                # 空串 = 最近一张 /map 合格。非空时 reason 一定是 NO_MAP，
                # 这一项说明它是"图根本没来"还是"图来了但不合格"。
                "map_reject_reason": (
                    self._map_reject_provider() if self._map_reject_provider else {}
                ),
            },
        )

    # ------------------------------------------------------------ internals
    def _readiness(self) -> tuple[bool, str]:
        if not self._path_client.server_ready() or not self._navigate_client.server_ready():
            return False, REJECT_NAV2_NOT_READY
        if not self._localization_valid():
            return False, REJECT_NO_LOCALIZATION
        if self._grid_provider is None or self._grid_provider() is None:
            return False, REJECT_NO_MAP
        return True, ""

    def _advance_planning(self, active: _ActiveGoal) -> NavigationResult | None:
        poll = self._path_client.poll()
        if poll.status in (ACTION_UNKNOWN, ACTION_ACCEPTED, ACTION_EXECUTING, ACTION_CANCELING):
            if float(self._now()) - active.plan_sent_at > self._plan_timeout_s:
                return self._rejected(active.goal, REJECT_NO_PATH, "compute path timeout")
            return None
        if poll.status == ACTION_CANCELED:
            return self._result(
                active.goal.goal_id,
                NavigationStatus.CANCELLED,
                message="compute path canceled",
                provenance={"backend": BACKEND_NAME},
            )
        if poll.status != ACTION_SUCCEEDED:
            return self._rejected(
                active.goal, REJECT_NO_PATH, f"compute path {poll.status.lower()}"
            )
        path = list(poll.path)
        active.path_length_m = _path_length_m(path) if path else 0.0
        if not path:
            return self._rejected(active.goal, REJECT_NO_PATH, "nav2 returned an empty path")
        if getattr(self._trial_budget, "forward_only", False):
            pose=self.get_pose()
            grid=self._grid_provider() if self._grid_provider is not None else None
            forward_path=[]
            for index,(x,y) in enumerate(path):
                if pose is None:
                    break
                forward=(x-pose.x)*math.cos(pose.yaw)+(y-pose.y)*math.sin(pose.yaw)
                lateral=-(x-pose.x)*math.sin(pose.yaw)+(y-pose.y)*math.cos(pose.yaw)
                # NavFn starts at the center of the robot's occupancy cell.
                # Account only for that first point's bounded quantization;
                # the independent measured-motion reverse stop stays at 2cm.
                if (index==0 and grid is not None and
                        math.hypot(x-pose.x,y-pose.y)<=grid.resolution/math.sqrt(2)+1e-6):
                    continue
                forward_path.append((forward,lateral))
            if (pose is None or any(f < -.02 or abs(l) > .10 for f,l in forward_path)
                    or any(b[0]<a[0]-.02 for a,b in zip(forward_path,forward_path[1:]))):
                return self._rejected(active.goal, REJECT_NO_PATH,
                                      "forward test refuses a reversing or lateral detour")
        if self._max_path_length_m is not None and active.path_length_m > self._max_path_length_m + .001:
            # 上限**必须印成实际值**。这里原来写死"the 3 m step limit"，而真机验收
            # 档把上限收到 0.10~0.12 m（见 run_semantic_exploration 里
            # ``min(3., forward_step_m)``）—— 于是一次 0.2 m 的路径被拒，报告却说
            # 它超过了 3 m。读的人只会得出"这个目标很合理、后端算错了"，
            # 而真因是"它比本次允许的单段长度长"。
            return self._rejected(
                active.goal,
                REJECT_NO_PATH,
                f"planned path {active.path_length_m:.3f} m exceeds this run's "
                f"{self._max_path_length_m:.3f} m per-step limit; choose a nearer goal",
            )

        grid = self._grid_provider() if self._grid_provider is not None else None
        if grid is None:
            return self._rejected(active.goal, REJECT_NO_MAP, "map unavailable")
        # 位姿只读一次：分类和扫掠都要用它，分两次读既浪费又会在两次之间被
        # TF 更新掉，让「按位姿 A 分类、按位姿 B 判扫掠」这种不可能解释的组合
        # 写进provenance。
        pose = self.get_pose()
        self._classify_motion(active, pose)
        if pose is None:
            return self._rejected(active.goal, REJECT_NO_LOCALIZATION,
                                  "current robot pose unavailable for footprint check")
        # NavFn may satisfy an occupied goal through its own fallback
        # tolerance.  That tolerance must not silently exceed this backend's
        # arrival contract: a short request can otherwise produce a path that
        # barely leaves the start yet is recorded as an accepted candidate.
        active.plan_endpoint_error_m = math.hypot(
            float(path[-1][0]) - float(active.request.x),
            float(path[-1][1]) - float(active.request.y),
        )
        if self._goal_xy_tolerance_m is not None:
            endpoint_limit = max(
                float(self._goal_xy_tolerance_m),
                float(grid.resolution) * math.sqrt(2.0),
            )
            if active.plan_endpoint_error_m > endpoint_limit + 1e-6:
                return self._rejected(
                    active.goal,
                    REJECT_NO_PATH,
                    f"planned endpoint is {active.plan_endpoint_error_m:.3f} m from "
                    f"the requested goal (limit {endpoint_limit:.3f} m)",
                )
        # Reconcile only cells under the current physical body.  The robot's
        # valid pose proves that volume is presently occupiable; stale SLAM
        # self-cells there must not deadlock every future goal.  The outer
        # padding ring and the whole future sweep remain fail-closed, while
        # the independent live-scan/Collision Monitor chain is untouched.
        grid, active.current_footprint_overlay = current_footprint_observed_grid(
            grid, float(pose.x), float(pose.y), float(pose.yaw)
        )
        # Lateral rays along a planned path cannot certify the robot's current
        # rear corners. Check the padded starting rectangle for every goal,
        # independently of the target heading or translation/rotation split.
        active.initial_footprint = classify_rotation_sweep(
            grid, float(pose.x), float(pose.y), float(pose.yaw), float(pose.yaw),
            minimum_sweep_rad=0.0, margin_rad=0.0,
        )
        if not active.initial_footprint.passable:
            diagnostic_egress = bool(
                self._trial_budget is not None
                and getattr(self._trial_budget, "collision_diagnostic", False)
                and active.translation_m is not None
                and 0.10 < active.translation_m <= 0.30
                and abs(math.remainder(active.request.yaw - pose.yaw, 2 * math.pi))
                    <= math.radians(2.0)
            )
            intent = active.goal.provenance.get("intent") or {}
            bounded_frontier_egress = bool(
                self._trial_budget is not None
                and isinstance(intent, dict)
                and intent.get("intent_type") == "EXPLORE_FRONTIER"
                and active.motion_mode == _MODE_TRANSLATION
                and active.translation_m is not None
                and 0.10 < active.translation_m <= 0.60
            )
            field_frontier_egress = bool(
                self._travel_budget is not None
                and self._trial_budget is None
                and isinstance(intent, dict)
                and intent.get("intent_type") == "EXPLORE_FRONTIER"
                and active.motion_mode == _MODE_TRANSLATION
                and active.translation_m is not None
                and 0.10 < active.translation_m <= 0.60
            )
            if diagnostic_egress:
                active.initial_footprint_egress = classify_forward_padding_egress(
                    grid, float(pose.x), float(pose.y), float(pose.yaw), path,
                    max_distance_m=0.30,
                    # A 5 cm NavFn grid turns a centimetre-scale straight
                    # request into staircase segments.  The live GO2-W case
                    # measured 16.99 degrees with only 1.9 cm total lateral
                    # offset.  Keep this relaxation local to the separately
                    # authorized, one-step collision diagnostic; ordinary
                    # search retains its tighter policy below.
                    max_segment_angle_rad=math.radians(20.0),
                    allow_known_rear_corner_padding=True,
                    rear_corner_quantization_m=float(grid.resolution),
                )
            elif bounded_frontier_egress:
                # A supervised bounded search may start with a mapped return
                # in the 4 cm rear padding shell.  Preserve the full footprint
                # everywhere else and admit only a known occupied cell strictly
                # behind the physical body while the checked Nav2 path leaves
                # it monotonically forward.  UNKNOWN/off-map, side/front cells,
                # turns and paths over 0.60 m still reject.  Live scan collision
                # monitoring and the wheel travel guard remain independent.
                active.initial_footprint_egress = classify_forward_padding_egress(
                    grid, float(pose.x), float(pose.y), float(pose.yaw), path,
                    max_distance_m=0.60, max_lateral_m=0.05,
                    allow_unknown_rear=False,
                    allow_known_rear_corner_padding=True,
                    max_segment_angle_rad=math.radians(5.0),
                )
            elif field_frontier_egress:
                # Mapping leaves a one-cell UNKNOWN crescent at the rear body
                # boundary because the roof lidar cannot observe through the
                # chassis.  For an incremental exploration frontier only,
                # permit a monotonic forward path to leave that contact shell.
                # Known side/front obstacles and off-map cells still reject;
                # Collision Monitor and the independent wheel guard remain live.
                active.initial_footprint_egress = classify_forward_padding_egress(
                    grid, float(pose.x), float(pose.y), float(pose.yaw), path,
                    max_distance_m=0.60, max_lateral_m=0.15,
                    allow_unknown_rear=True,
                )
            if not (
                active.initial_footprint_egress is not None
                and active.initial_footprint_egress.allowed
            ):
                return self._rejected(
                    active.goal, REJECT_TOO_NARROW,
                    "current robot footprint overlaps occupied or unknown map cells",
                )
        if pose is not None:
            active.rotation_translation_radius_m = max(
                float(active.translation_m or 0),
                *(math.hypot(float(px)-float(pose.x), float(py)-float(pose.y)) for px, py in path))
        sweep = self._rotation_sweep(grid, active, pose)
        active.rotation = sweep
        if active.motion_mode == _MODE_ROTATION_IN_PLACE and sweep is not None and sweep.applicable:
            # 计划书 §7.2：几乎零平移、只改偏航的目标，平移走廊没有分辨力 ——
            # 那条 0.008 m 的退化路径量出来的左右「净空」打的是机器人自己身边
            # 的格（现场 −0.055 / −0.105 m），跟这次转向毫无关系。转向该由机身
            # 矩形扫过的区域来判。见 corridor_analyzer 顶部的现场记录。
            if sweep.decision != DECISION_OK:
                return self._rejected(
                    active.goal,
                    REJECT_TOO_NARROW,
                    "rotation sweep blocked the turn: "
                    f"{sweep.reject_reason} "
                    f"(nearest blocker {_fmt_m(sweep.nearest_blocker_m)} at "
                    f"{_fmt_deg(sweep.nearest_blocker_deg)})",
                )
            # 精确原地转向必须把 RPP 的线速度目标写成 0。现场反例
            # ``authorized_motion_20261023_1826`` 证明：写成 0.05 m/s 后，mapping
            # 模式的一次 map 校正会让固定目标点看起来偏离当前位置，控制器随即
            # 真的发出 0.05 m/s，15° 请求累计爬行 0.53 m 后才被总里程门停住。
            # 仍含厘米级规划路径的 near-in-place 目标保留低速；那类目标本来就
            # 请求了平移，不能用 0 把它卡死。试验账本另外对精确转向设置 0.10 m
            # 独立平移熔断，防止控制器或定位校正再次把转向变成走路。
            exact_turn = (
                active.translation_m is not None
                and active.translation_m <= 1e-3
                # NavFn may return a centimetre-scale start-cell segment even
                # when the requested map position is exactly the current pose.
                # Treat that as quantization, not as a request to make RPP
                # chase a zero-distance goal.  The broader motion classifier
                # and full-body rotation sweep have already bounded this path.
                and active.path_length_m <= ROTATION_TRANSLATION_TOL_M
            )
            active.speed_limit_mps = 0.0 if exact_turn else MIN_SPEED_MPS
            if exact_turn:
                # Spin 的 target_yaw 是相对当前姿态，而 request.yaw 是 map 绝对角。
                # 在完成几何扫掠的同一份位姿上冻结这个差值，避免 action 发出前的
                # SLAM 小修正把“原地转”重新解释成平移追点。
                active.spin_target_yaw = math.remainder(
                    float(active.request.yaw) - float(pose.yaw), 2 * math.pi
                )
        else:
            # 三种情况落到这里，判据都还是走廊（P1 之前的原样）：
            #   * 会平移的目标。RPP 起步转的是*路径切向*、到点才转到目标偏航，
            #     也可能反过来；按「当前位置 + 目标偏航」判扫掠是在判一条 Nav2
            #     未必会走的轨迹，拿它拒绝会是假拒绝（所以上面那份扫掠只记录、
            #     不参与判定）。
            #   * 近原地目标没有可分辨的转角（``applicable=False``），保留平移判据。
            #   * 原地转向但拿不到位姿（``sweep is None``）。判据说不出话时收紧。
            decision = classify_path_clearance(measure_path_clearance(path, grid))
            diagnostic_corridor_egress = bool(
                self._trial_budget is not None
                and getattr(self._trial_budget, "collision_diagnostic", False)
                and active.initial_footprint_egress is not None
                and active.initial_footprint_egress.allowed
                and active.motion_mode == _MODE_TRANSLATION
                and active.translation_m is not None
                and 0.10 < active.translation_m <= 0.30
                and decision.min_side_gap_m is not None
                and decision.min_side_gap_m
                    >= COLLISION_DIAGNOSTIC_MIN_SIDE_GAP_M
            )
            if decision.decision != DECISION_OK and diagnostic_corridor_egress:
                # The geometry has already proved this is a short, monotonic
                # forward exit from a known rear-padding contact.  Admit the
                # map-only 0.30--0.40 m band at the minimum geometric speed;
                # the separately bounded trial policy may then select its
                # recorded working command.  No global corridor constant,
                # footprint, costmap or Collision Monitor setting is changed.
                decision = replace(
                    decision,
                    decision=DECISION_OK,
                    reject_reason="",
                    speed_limit_mps=MIN_SPEED_MPS,
                )
                active.diagnostic_corridor_policy = (
                    "collision_diagnostic_min_side_gap_0.30m"
                )
            active.corridor = decision
            active.speed_limit_mps = decision.speed_limit_mps
            if decision.decision != DECISION_OK:
                # 计划书 §6.1 / §12.2：不安全就不发 NavigateToPose。
                # 走廊分析器内部还有更细的原因串（例如 NO_PATH_SAMPLES：路径一个
                # 采样点都取不到）。§14 把候选记录的 reject_reason 限定在固定枚举
                # 里，所以这里做一次收口；原始串不丢，它就在 message 里。
                return self._rejected(
                    active.goal,
                    _candidate_reject_reason(decision.reject_reason),
                    f"corridor rejected the path: {decision.reject_reason}",
                )

        # On the live Go2-W, .05 m/s repeatedly produced no useful movement;
        # .12 completed the recorded physical goal. Choose that working command
        # only AFTER the full footprint/corridor checks above. This is a fixed
        # low cruise speed, not an assertion that lower commands were calibrated.
        # Collision stopping, smoothing, travel guards and per-goal timeouts stay
        # independent. The separately budgeted trial retains its own policy.
        if (self._trial_budget is None and self._translation_speed_mps is not None
                and active.motion_mode == _MODE_TRANSLATION):
            active.speed_limit_mps = self._translation_speed_mps

        self._select_trial_speed(active)
        if self._dry_run:
            return self._result(
                active.goal.goal_id,
                NavigationStatus.CANCELLED,
                message="dry run: path checked; motion action not sent",
                provenance={**self._decision_provenance(active, "", True),
                            "backend": BACKEND_NAME, "dry_run": True},
            )

        active.phase = _PHASE_APPLYING_SPEED
        return self._advance_speed_limit(active)

    def _classify_motion(self, active: _ActiveGoal, pose: RobotPose | None) -> None:
        """把这次目标分类成平移还是原地转向（计划书 §7.2）。

        判据要**同时**看两件事：目标点离机器人多远，以及 Nav2 返回的路径多长。
        只看目标点距离，一个「绕开障碍走 3 m 回到身边 5 cm 处」的目标会被误判成
        原地转向 —— 平移走廊被跳过，扫掠判据又只查了转向那一圈，两次都判错。
        两个量都退化才叫原地转向。

        拿不到位姿时一律回到 ``_MODE_UNKNOWN``，走廊判据照旧兜底：判据说不出
        话的时候收紧，不放松。
        """
        if pose is None:
            active.translation_m = None
            active.motion_mode = _MODE_UNKNOWN
            return
        active.planning_pose = pose
        translation = math.hypot(
            float(active.request.x) - float(pose.x),
            float(active.request.y) - float(pose.y),
        )
        active.translation_m = translation
        if (
            translation <= ROTATION_TRANSLATION_TOL_M
            and float(active.path_length_m) <= ROTATION_TRANSLATION_TOL_M
        ):
            active.motion_mode = _MODE_ROTATION_IN_PLACE
        else:
            active.motion_mode = _MODE_TRANSLATION

    def _rotation_sweep(
        self, grid: CorridorGrid, active: _ActiveGoal, pose: RobotPose | None
    ) -> RotationSweepDecision | None:
        """按当前位姿与目标偏航算一次扫掠判定；位姿不可用时返回 ``None``。

        ``None`` 表示「这项证据没有」，与「扫掠通过」是两回事：原地转向分支
        在 ``None`` 时拒绝（那条路径只有这一个判据），平移分支在 ``None`` 时
        放行（平移段已有走廊判据）。
        """
        if pose is None:
            return None
        return classify_rotation_sweep(
            grid,
            float(pose.x),
            float(pose.y),
            float(pose.yaw),
            float(active.request.yaw),
            # A path up to 10cm is classified as near-in-place. Its center
            # still moves, and that entire excursion must enter the sweep.
            translation_radius_m=(active.rotation_translation_radius_m
                                  if active.motion_mode == _MODE_ROTATION_IN_PLACE else 0.0),
            # Small bounded trial turns need the same footprint proof as
            # larger turns. A lateral corridor cannot certify corner motion.
            minimum_sweep_rad=(1e-6 if active.motion_mode == _MODE_ROTATION_IN_PLACE
                               else ROTATION_MIN_SWEEP_RAD),
        )

    def _select_trial_speed(self, active: _ActiveGoal) -> None:
        if self._trial_budget is not None:
            geometric_speed=(active.corridor.speed_limit_mps if active.corridor is not None
                             else active.speed_limit_mps)
            active.speed_limit_mps = min(geometric_speed, self._trial_budget.MAX_SPEED_MPS)
            active.trial_speed_policy=''
            if ((getattr(self._trial_budget, "forward_only", False)
                    or getattr(self._trial_budget, "bounded_search", False))
                    and active.motion_mode == _MODE_TRANSLATION
                    and active.corridor is not None and active.corridor.passable):
                active.speed_limit_mps=(.30 if getattr(
                    self._trial_budget, 'speed_calibration_0_30', False) else .12)
                active.trial_speed_policy=(
                    'supervised_velocity_calibration_0_30'
                    if getattr(self._trial_budget, 'speed_calibration_0_30', False)
                    else 'bounded_forward_working_command')
            # Explicitly authorized v2 calibration, limited to a short forward
            # goal in the preferred-clearance band. It is not a change to the
            # normal field corridor formula or permission for longer travel.
            if (self._trial_budget.MAX_SPEED_MPS == .12
                    and not getattr(self._trial_budget, "forward_only", False)
                    and not getattr(self._trial_budget, "bounded_search", False)
                    and active.motion_mode == _MODE_TRANSLATION
                    and active.corridor is not None
                    and active.corridor.passable
                    and active.corridor.min_side_gap_m is not None
                    and active.corridor.min_side_gap_m >= PREFERRED_SIDE_GAP_M
                    and active.path_length_m <= .15):
                pose=self.get_pose()
                if pose is not None:
                    dx=active.request.x-pose.x;dy=active.request.y-pose.y
                    if (.10 < math.hypot(dx,dy) <= .12
                            and abs(math.remainder(math.atan2(dy,dx)-pose.yaw,2*math.pi))<=math.radians(5)
                            and abs(math.remainder(active.request.yaw-pose.yaw,2*math.pi))<=math.radians(5)):
                        active.speed_limit_mps=.12
                        active.trial_speed_policy='wide_straight_velocity_calibration'
    def _advance_speed_limit(self, active: _ActiveGoal) -> NavigationResult | None:
        self._select_trial_speed(active)
        if active.spin_target_yaw is not None and (
            self._spin_client is None or not self._spin_client.server_ready()
        ):
            return self._rejected(
                active.goal, REJECT_NAV2_NOT_READY,
                "Nav2 Spin action server is unavailable for an in-place turn",
            )
        if active.spin_target_yaw is not None and self._trial_pose_provider is not None:
            active.tracking_start = self._trial_pose_provider()
            if active.tracking_start is None:
                return self._rejected(
                    active.goal, REJECT_NO_LOCALIZATION,
                    "fresh fused odometry is unavailable for in-place turn tracking",
                )
        applied = self._speed_limit_setter(active.speed_limit_mps) if self._speed_limit_setter else True
        if applied is None:
            if float(self._now()) - active.plan_sent_at > self._plan_timeout_s:
                return self._rejected(active.goal, REJECT_NAV2_NOT_READY, "controller speed limit response timeout")
            return None
        if not applied:
            # The selected working speed (or separately bounded trial cap)
            # must be acknowledged by RPP before any NavigateToPose is sent.
            # A missing controller/service must never silently retain its
            # previous goal's desired speed.
            return self._rejected(
                active.goal,
                REJECT_NAV2_NOT_READY,
                f"navigation speed limit {active.speed_limit_mps:.3f} m/s could not be "
                "applied to controller_server",
            )
        if self._travel_budget is not None:
            try:
                prepared = self._travel_budget.prepare(active.goal.goal_id,self._max_path_length_m)
            except GoalTravelError as exc:
                return self._rejected(active.goal,REJECT_NAV2_NOT_READY,str(exc))
            if not prepared:
                if float(self._now())-active.plan_sent_at > self._plan_timeout_s:
                    return self._rejected(active.goal,REJECT_NAV2_NOT_READY,"wheel travel guard prepare timeout")
                return None
        # Read-only admission must happen before the durable trial reservation.
        # Missing launcher authorization or stale safety evidence cannot consume
        # a one-way field slot when no motion Action could be sent.
        if self._motion_preflight is not None and not self._motion_preflight():
            return self._rejected(
                active.goal,
                REJECT_NAV2_NOT_READY,
                "motion preflight authorization or safety check failed",
            )
        if self._trial_budget is not None:
            try:
                trial_pose = (
                    self._trial_pose_provider()
                    if self._trial_pose_provider is not None
                    else self.get_pose()
                )
                active.tracking_start = trial_pose
                self._trial_budget.reserve(
                    goal_id=active.goal.goal_id, path_length_m=active.path_length_m,
                    goal_x=active.request.x, goal_y=active.request.y, goal_yaw=active.request.yaw,
                    pose=self.get_pose(), tracking_pose=trial_pose,
                )
            except TrialBudgetError as exc:
                return self._rejected(active.goal, REJECT_NAV2_NOT_READY, str(exc))
        if self._motion_authorizer is not None and not self._motion_authorizer():
            return self._rejected(active.goal, REJECT_NAV2_NOT_READY, "motion authorization or arm failed")
        # 授权在这一刻真的断言上了 —— 记下来，结算时才知道该不该收回（§P3-3）。
        active.armed = True
        if self._travel_budget is not None:
            active.phase = 'STARTING_NAVIGATION'
            return self._start_guarded_navigation(active)
        active.phase = _PHASE_NAVIGATING
        self._send_motion_action(active)
        return None

    def _start_guarded_navigation(self, active: _ActiveGoal) -> NavigationResult | None:
        try:
            running = self._travel_budget.activate()
        except GoalTravelError as exc:
            return self._rejected(active.goal,REJECT_NAV2_NOT_READY,str(exc))
        if not running:
            return None
        active.phase = _PHASE_NAVIGATING
        self._send_motion_action(active)
        return None

    def _send_motion_action(self, active: _ActiveGoal) -> None:
        if active.spin_target_yaw is not None:
            if self._spin_client is None:  # 已在 _advance_speed_limit fail-closed。
                raise RuntimeError("Nav2 Spin action client is unavailable")
            self._spin_client.send(SpinGoalRequest(active.spin_target_yaw))
            active.motion_action_sent = True
            return
        self._navigate_client.send(active.request)
        active.motion_action_sent = True

    def _motion_action_client(self, active: _ActiveGoal) -> ActionClientPort:
        if active.spin_target_yaw is not None and self._spin_client is not None:
            return self._spin_client
        return self._navigate_client

    def _advance_navigating(self, active: _ActiveGoal) -> NavigationResult | None:
        motion_client = self._motion_action_client(active)
        if active.spin_target_yaw is not None and self._trial_pose_provider is not None:
            tracking_pose = self._trial_pose_provider()
            if tracking_pose is None:
                motion_client.cancel()
                return self._rejected(
                    active.goal, REJECT_NO_LOCALIZATION,
                    "fresh fused odometry was lost during in-place turn",
                )
            if active.tracking_start is not None and math.hypot(
                tracking_pose.x-active.tracking_start.x,
                tracking_pose.y-active.tracking_start.y,
            ) > 0.10:
                motion_client.cancel()
                if self._stop_publisher:
                    self._stop_publisher()
                return self._rejected(
                    active.goal, REJECT_NAV2_NOT_READY,
                    "in-place turn exceeded 0.10 m translation drift limit",
                )
        if self._travel_budget is not None:
            try:
                self._travel_budget.check()
            except GoalTravelError as exc:
                try:
                    motion_client.cancel()
                except Exception:
                    pass
                return self._rejected(active.goal,REJECT_NAV2_NOT_READY,str(exc))
        if self._trial_budget is not None:
            try:
                trial_pose = (
                    self._trial_pose_provider()
                    if self._trial_pose_provider is not None
                    else self.get_pose()
                )
                reason = self._trial_budget.observe(trial_pose)
            except TrialBudgetError as exc:
                reason = str(exc)
            if reason:
                motion_client.cancel()
                if self._stop_publisher:
                    self._stop_publisher()
                return self._rejected(active.goal, REJECT_NAV2_NOT_READY, reason)
        poll = motion_client.poll()
        if poll.status in (ACTION_UNKNOWN, ACTION_ACCEPTED, ACTION_EXECUTING, ACTION_CANCELING):
            return None
        mapping = {
            ACTION_SUCCEEDED: NavigationStatus.SUCCEEDED,
            ACTION_CANCELED: NavigationStatus.CANCELLED,
            ACTION_ABORTED: NavigationStatus.FAILED,
        }
        status = mapping.get(poll.status, NavigationStatus.FAILED)
        action_name = "spin" if active.spin_target_yaw is not None else "navigate to pose"
        message = poll.error or f"{action_name} {poll.status.lower()}"
        goal_errors={}
        if status is NavigationStatus.SUCCEEDED and (
                self._goal_xy_tolerance_m is not None or self._goal_yaw_tolerance_rad is not None):
            pose=self.get_pose()
            if pose is None:
                return self._rejected(active.goal,REJECT_NO_LOCALIZATION,
                                      'Nav2 reported success without a fresh final pose')
            if (self._trial_budget is not None and active.tracking_start is not None
                    and active.planning_pose is not None):
                final_tracking = (
                    self._trial_pose_provider()
                    if self._trial_pose_provider is not None else None
                )
            else:
                final_tracking = None
            if final_tracking is not None:
                map_dx = active.request.x-active.planning_pose.x
                map_dy = active.request.y-active.planning_pose.y
                forward = (math.cos(active.planning_pose.yaw)*map_dx
                           + math.sin(active.planning_pose.yaw)*map_dy)
                lateral = (-math.sin(active.planning_pose.yaw)*map_dx
                           + math.cos(active.planning_pose.yaw)*map_dy)
                expected_dx = (math.cos(active.tracking_start.yaw)*forward
                               - math.sin(active.tracking_start.yaw)*lateral)
                expected_dy = (math.sin(active.tracking_start.yaw)*forward
                               + math.cos(active.tracking_start.yaw)*lateral)
                observed_dx = final_tracking.x-active.tracking_start.x
                observed_dy = final_tracking.y-active.tracking_start.y
                xy_error = math.hypot(
                    observed_dx-expected_dx, observed_dy-expected_dy,
                )
                observed_turn = math.remainder(
                    final_tracking.yaw-active.tracking_start.yaw, 2*math.pi
                )
                requested_turn = math.remainder(
                    active.request.yaw-active.planning_pose.yaw, 2*math.pi
                )
                yaw_error = abs(math.remainder(
                    observed_turn-requested_turn, 2*math.pi
                ))
                goal_errors['odometry_translation_m'] = math.hypot(
                    observed_dx, observed_dy
                )
                goal_errors['odometry_turn_rad'] = observed_turn
                goal_errors['odometry_final_pose'] = final_tracking.to_dict()
            else:
                xy_error=math.hypot(pose.x-active.request.x,pose.y-active.request.y)
                yaw_error=abs(math.remainder(pose.yaw-active.request.yaw,2*math.pi))
            goal_errors={'final_position_error_m':xy_error,'final_yaw_error_rad':yaw_error,
                         'final_pose':pose.to_dict(), **goal_errors}
            if (self._goal_xy_tolerance_m is not None and xy_error>self._goal_xy_tolerance_m+1e-6 or
                    self._goal_yaw_tolerance_rad is not None and yaw_error>self._goal_yaw_tolerance_rad+1e-6):
                status=NavigationStatus.FAILED
                message=(f'Nav2 reported success outside current goal tolerance: '
                         f'position error {xy_error:.3f}m, yaw error {yaw_error:.3f}rad')
        return self._result(
            active.goal.goal_id,
            status,
            message=message,
            requested_motion=_requested_motion(active),
            observed_motion={**self._observed_motion(active, status), **goal_errors},
            elapsed_sec=float(self._now()) - active.started_at,
            provenance=self._decision_provenance(
                active, "" if status == NavigationStatus.SUCCEEDED else REJECT_NO_PATH,
                status == NavigationStatus.SUCCEEDED,
            ),
        )

    def _rejected(
        self, goal: ExplorationGoal, reason: str, message: str
    ) -> NavigationResult:
        active = self._active
        if active is not None and active.handle.goal_id == goal.goal_id:
            active.reject_reason = reason
            provenance = self._decision_provenance(active, reason, False)
        else:
            provenance = {"backend": BACKEND_NAME}
        # 顶层也放一份：候选记录里的原因在 candidate_decision 下，但读取这条
        # 结果的日志与闸门只关心「为什么被拒」，不该每处都往里钻一层。
        provenance["reject_reason"] = reason
        return self._result(
            goal.goal_id,
            NavigationStatus.REJECTED,
            message=message,
            provenance=provenance,
        )

    def _decision_provenance(
        self, active: _ActiveGoal, reject_reason: str, accepted: bool
    ) -> dict[str, Any]:
        """计划书 §14 要求的每个候选一行结构化记录。"""
        decision = active.corridor
        goal = active.goal
        request = active.request
        record: dict[str, Any] = {
            "candidate_id": goal.goal_id,
            "goal_x": round(request.x, 3),
            "goal_y": round(request.y, 3),
            "goal_yaw": round(math.degrees(request.yaw), 3),
            "path_length_m": round(active.path_length_m, 3),
            "plan_endpoint_error_m": _round_or_none(active.plan_endpoint_error_m),
            # 动作类别与到目标的平移量：判据用的是哪一套，读日志的人要能一眼看出来。
            # 2026-10-15 那场只读搜索的候选记录里没有这两个字段，于是
            # 「TOO_NARROW」到底是走廊判的还是扫掠判的，事后无法从记录里分辨。
            "motion_mode": active.motion_mode or _MODE_UNKNOWN,
            "translation_to_goal_m": _round_or_none(active.translation_m),
            "min_left_body_gap_m": _round_or_none(
                decision.min_left_body_gap_m if decision else None
            ),
            "min_right_body_gap_m": _round_or_none(
                decision.min_right_body_gap_m if decision else None
            ),
            "min_side_gap_m": _round_or_none(
                decision.min_side_gap_m if decision else None
            ),
            "corridor_decision": decision.decision if decision else "",
            "speed_limit_mps": round(active.speed_limit_mps, 4),
            "center_score": round(decision.center_score, 4) if decision else 0.0,
            "accepted": bool(accepted),
            "reject_reason": reject_reason,
        }
        if active.rotation is not None:
            record.update(active.rotation.to_dict())
        if active.initial_footprint is not None:
            footprint = active.initial_footprint
            record['initial_footprint'] = {
                'accepted': footprint.passable,
                'nearest_blocker_m': _round_or_none(footprint.nearest_blocker_m),
                'blocking_cells': list(footprint.blocked_samples),
            }
        if active.initial_footprint_egress is not None:
            record['initial_footprint_egress'] = (
                active.initial_footprint_egress.to_dict()
            )
        if active.current_footprint_overlay is not None:
            record['current_footprint_overlay'] = dict(active.current_footprint_overlay)
        if active.trial_speed_policy:
            record['trial_speed_policy']=active.trial_speed_policy
        if active.diagnostic_corridor_policy:
            record['diagnostic_corridor_policy'] = active.diagnostic_corridor_policy
        if (self._trial_budget is None and self._translation_speed_mps is not None
                and active.motion_mode == _MODE_TRANSLATION and accepted):
            record['linear_speed_policy']='recorded_working_command'
            record['geometric_reference_speed_mps']=round(decision.speed_limit_mps,4) if decision else None
        provenance = {
            "backend": BACKEND_NAME,
            "candidate_decision": record,
            # Status alone is ambiguous: the observed-travel guard can also
            # settle an already-moving goal as REJECTED.
            "motion_action_sent": bool(active.motion_action_sent),
        }
        if self._travel_budget is not None:
            provenance['wheel_travel'] = self._travel_budget.snapshot()
        return provenance

    def _observed_motion(
        self, active: _ActiveGoal, status: NavigationStatus
    ) -> dict[str, Any]:
        observed = {
            "path_length_m": round(active.path_length_m, 3),
            "speed_limit_mps": round(active.speed_limit_mps, 4),
            "goal_x": round(active.request.x, 3),
            "goal_y": round(active.request.y, 3),
            "goal_yaw_deg": round(math.degrees(active.request.yaw), 3),
            "status": status.value,
        }
        if self._travel_budget is not None:
            measured = self._travel_budget.snapshot()
            observed.update(wheel_traveled_m=measured.get('traveled_m'),
                translation_limit_m=measured.get('limit_m'))
        return observed

    def _release_motion(self, active: _ActiveGoal | None, *, succeeded: bool = False) -> None:
        """目标结算时，把**为它**断言上的运动授权收回来（计划书 §P3-3）。

        `authorize_nav2_motion` 只回答"能不能动"，它成功的那一刻机器人就处于
        **授权态**；而取消/失败/超时之后没有任何人把它收回去。授权是按空闲租约
        计时的（60 s），所以窗口是有界的 —— 但窗口内只要还有别的发布者往速度
        链上写，门就不再清零了。计划书要的就是这一段："武装后发送失败须取消并
        解除武装"。

        有 trial/累计行程 guard 的目标无论成功失败都完整 stop/disarm，下一
        目标必须重新握手计程。没有 guard 的旧注入端口保留仅失败时收回的语义。

        **只收回自己断言的那一次。** ``active.armed`` 为假说明这个目标从没武装过
        （授权被拒、或走的是 dry-run），此时去解除武装会打掉**别人**（操作员）
        正在用的授权。

        收回失败不抛：调用点在"目标结束、准备结算结果"的路径上，一次失败的服务
        调用不该把一次正常的导航结算变成异常。ARM 是租约，它自己会过期。
        """
        if active is None:
            return
        if self._travel_budget is not None and self._travel_budget.request is not None:
            try:
                self._travel_budget.finish()
            except Exception:
                pass  # Bridge lease expiry is independent of this transport.
            if self._stop_publisher is not None and active.armed:
                try:
                    self._stop_publisher()
                except Exception:
                    pass
            succeeded = False  # Each guarded goal owns a complete arm/disarm.
        if self._trial_budget is not None:
            # A trial closes its own arm even on success; reservations are
            # never refunded after rejected arms or failed Navigate sends.
            if self._stop_publisher is not None and (active.armed or self._trial_budget.token is not None):
                try:
                    self._stop_publisher()
                except Exception:
                    # A failed stop transport must never skip disarming the
                    # lease that this goal owns.
                    pass
            try:
                self._trial_budget.finish()
            except TrialBudgetError:
                # The durable active reservation remains locked. A damaged
                # ledger must not prevent stop/disarm on this exit path.
                pass
            succeeded = False
        if not active.armed:
            return
        active.armed = False
        if succeeded or self._motion_releaser is None:
            return
        try:
            self._motion_releaser()
        except Exception:  # noqa: BLE001 - 收不回来也要让目标正常结束
            pass

    def _finish_canceled(self, goal: ExplorationGoal, message: str) -> None:
        active = self._active
        if active is None:
            return
        self._release_motion(active)
        active.handle.result = self._result(
            active.goal.goal_id,
            NavigationStatus.CANCELLED,
            message=message,
            provenance={"backend": BACKEND_NAME},
        )
        self._active = None

    def _result(
        self,
        goal_id: str,
        status: NavigationStatus,
        *,
        message: str = "",
        requested_motion: dict[str, Any] | None = None,
        observed_motion: dict[str, Any] | None = None,
        elapsed_sec: float | None = None,
        provenance: dict[str, Any] | None = None,
    ) -> NavigationResult:
        return navigation_result(
            goal_id,
            status,
            message=message,
            requested_motion=requested_motion,
            observed_motion=observed_motion,
            elapsed_sec=elapsed_sec,
            provenance=provenance or {"backend": BACKEND_NAME},
        )


def _candidate_reject_reason(reason: str) -> str:
    """把上游更细的原因串收口到 §14 的固定枚举。

    枚举外的一律记成 ``TOO_NARROW``：走到这里的都是「路径不能安全通过」这一个
    失败类，而走廊分析对取不到采样点的路径本来就是 fail-closed 判窄的。
    """
    return reason if reason in CANDIDATE_REJECT_REASONS else REJECT_TOO_NARROW


def _path_length_m(path: Sequence[tuple[float, float]]) -> float:
    """端口里的路径是 ``(x, y)`` 元组，而长度工具吃 dict / 带属性的对象。"""
    return compute_path_length([{"x": x, "y": y} for x, y in path])


def _request_from_goal(goal: ExplorationGoal) -> tuple[Nav2GoalRequest | None, str]:
    if goal.position is None:
        return None, "NAVIGATE_POSE requires a position"
    frame = str(goal.frame or MAP_FRAME).strip().lower()
    if frame not in (MAP_FRAME, "map_frame"):
        return None, f"nav2 backend only accepts map-frame goals, got frame={goal.frame!r}"
    x = float(goal.position[0])
    y = float(goal.position[1])
    yaw = math.radians(float(goal.yaw or 0.0))
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
        return None, "NAVIGATE_POSE goal is not finite"
    return Nav2GoalRequest(x=x, y=y, yaw=yaw), ""


def _requested_motion(active: _ActiveGoal) -> dict[str, Any]:
    return {
        "goal_type": active.goal.goal_type,
        "frame": MAP_FRAME,
        "position": [round(active.request.x, 3), round(active.request.y, 3)],
        "yaw_deg": round(math.degrees(active.request.yaw), 3),
    }


def _round_or_none(value: float | None) -> float | None:
    return None if value is None else round(float(value), 3)


def _fmt_m(value: float | None) -> str:
    """只给拒绝消息用；``None`` 要说成「没测到」，不能写成 0.00 m。"""
    return "n/a" if value is None else f"{float(value):.2f} m"


def _fmt_deg(value: float | None) -> str:
    return "n/a" if value is None else f"{float(value):.1f} deg"


__all__ = [
    "ACTION_ABORTED",
    "ACTION_ACCEPTED",
    "ACTION_CANCELED",
    "ACTION_CANCELING",
    "ACTION_EXECUTING",
    "ACTION_SUCCEEDED",
    "ACTION_UNKNOWN",
    "ActionClientPort",
    "ActionPoll",
    "BACKEND_NAME",
    "CANDIDATE_REJECT_REASONS",
    "MAP_FRAME",
    "Nav2GoalRequest",
    "Nav2SearchBackend",
    "REJECT_GOAL_OCCUPIED",
    "REJECT_NAV2_NOT_READY",
    "REJECT_NOT_MAP_FRAME",
    "REJECT_NO_LOCALIZATION",
    "REJECT_NO_MAP",
    "REJECT_NO_PATH",
    "REJECT_TOO_NARROW",
    "RosActionClientPort",
    "SUPPORTED_GOAL_TYPES",
    "create_nav2_action_clients",
    "create_nav2_search_backend",
    "yaw_from_quaternion",
]
