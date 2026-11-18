"""计划书 §12.2：Nav2 搜索后端测试。

用假的 Action 服务器驱动后端，重点验证**顺序**与**拒绝路径**：
先 ComputePathToPose，路径非空且走廊达标才发 NavigateToPose；
``RELATIVE_MOVE`` 明确拒绝且不产出任何底层前进命令。
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from app.navigation.corridor_analyzer import (
    DECISION_OK,
    DECISION_TOO_NARROW,
    CorridorGrid,
    classify_path_clearance,
    measure_path_clearance,
)
from app.navigation.models import ExplorationGoal
from app.navigation.nav2_search_backend import (
    ACTION_ABORTED,
    ACTION_CANCELED,
    ACTION_EXECUTING,
    ACTION_SUCCEEDED,
    CANDIDATE_REJECT_REASONS,
    REJECT_GOAL_OCCUPIED,
    REJECT_NAV2_NOT_READY,
    REJECT_NO_LOCALIZATION,
    REJECT_NO_MAP,
    REJECT_NO_PATH,
    REJECT_NOT_MAP_FRAME,
    REJECT_TOO_NARROW,
    SUPPORTED_GOAL_TYPES,
    ActionPoll,
    Nav2GoalRequest,
    Nav2SearchBackend,
    SpeedLimitApplier,
    _ActiveGoal,
    map_message_reject_reason,
)
from app.navigation.robot_backend import NavigationHandle, NavigationStatus, RobotPose

RESOLUTION = 0.05
# 机身半宽 + footprint padding：走廊量出来的净空要再扣掉它才是机身净空。
BODY_INSET_M = 0.215 + 0.04


# ---------------------------------------------------------------------------
# 替身
# ---------------------------------------------------------------------------


class FakeActionClient:
    """记录收发顺序的假 Action server。"""

    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.sent: list[object] = []
        self.cancel_calls = 0
        self._polls: list[ActionPoll] = []

    def queue(self, *polls: ActionPoll) -> None:
        """排入若干次轮询结果；最后一个会一直重复到被替换。"""
        self._polls.extend(polls)

    def server_ready(self) -> bool:
        return self.ready

    def send(self, request: object) -> None:
        self.sent.append(request)

    def poll(self) -> ActionPoll:
        if not self._polls:
            return ActionPoll()
        if len(self._polls) > 1:
            return self._polls.pop(0)
        return self._polls[0]

    def cancel(self) -> bool:
        self.cancel_calls += 1
        return True


class Clock:
    def __init__(self, now: float = 1000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class SpeedRecorder:
    def __init__(self) -> None:
        self.calls: list[float] = []

    def __call__(self, speed_limit_mps: float) -> bool:
        self.calls.append(float(speed_limit_mps))
        return True


# ---------------------------------------------------------------------------
# 地图
# ---------------------------------------------------------------------------


def _free_grid(size_m: float = 6.0) -> CorridorGrid:
    side = int(round(size_m / RESOLUTION))
    return CorridorGrid(
        resolution=RESOLUTION,
        origin_x=-size_m / 2.0,
        origin_y=-size_m / 2.0,
        width=side,
        height=side,
        data=tuple([0] * (side * side)),
    )


def _paint(grid: CorridorGrid, x_min, x_max, y_min, y_max, value: int) -> CorridorGrid:
    data = list(grid.data)
    col_lo = max(0, int(math.floor((x_min - grid.origin_x) / grid.resolution)))
    col_hi = min(
        grid.width, int(math.floor((x_max - grid.origin_x) / grid.resolution)) + 1
    )
    row_lo = max(0, int(math.floor((y_min - grid.origin_y) / grid.resolution)))
    row_hi = min(
        grid.height, int(math.floor((y_max - grid.origin_y) / grid.resolution)) + 1
    )
    for row in range(row_lo, row_hi):
        for column in range(col_lo, col_hi):
            data[row * grid.width + column] = value
    return CorridorGrid(
        resolution=grid.resolution,
        origin_x=grid.origin_x,
        origin_y=grid.origin_y,
        width=grid.width,
        height=grid.height,
        data=tuple(data),
    )


def _corridor_grid(half_width_m: float, *, narrow_from_x: float | None = None,
                   narrow_half_width_m: float = 0.30) -> CorridorGrid:
    """沿 y=0 造一条走廊，两侧墙在 ``±half_width_m``。

    ``narrow_from_x`` 之后收窄，用来构造「路径中段过不去」的情形。
    """
    grid = _free_grid()
    grid = _paint(grid, -10.0, 10.0, half_width_m, 10.0, 100)
    grid = _paint(grid, -10.0, 10.0, -10.0, -half_width_m, 100)
    if narrow_from_x is not None:
        grid = _paint(grid, narrow_from_x, 10.0, -10.0, narrow_half_width_m, 100)
        grid = _paint(grid, narrow_from_x, 10.0, -narrow_half_width_m, 10.0, 100)
    return grid


def _straight_path(length_m: float = 2.0, step_m: float = 0.10):
    count = int(round(length_m / step_m))
    return tuple((index * step_m, 0.0) for index in range(count + 1))


# ---------------------------------------------------------------------------
# 后端装配
# ---------------------------------------------------------------------------


def _backend(
    *,
    grid: CorridorGrid | None = None,
    localization_valid: bool = True,
    path_ready: bool = True,
    navigate_ready: bool = True,
    spin_ready: bool = True,
    clock: Clock | None = None,
    speed_setter: SpeedRecorder | None = None,
    stop_publisher=None,
    dry_run: bool = False,
    pose: "RobotPose | None" = RobotPose(x=0.0, y=0.0, yaw=0.0),
    has_pose_provider: bool = True,
    motion_authorizer=None,
    motion_preflight=None,
    motion_releaser=None,
    translation_speed_mps=None,
    goal_xy_tolerance_m=None,
):
    path_client = FakeActionClient(ready=path_ready)
    navigate_client = FakeActionClient(ready=navigate_ready)
    spin_client = FakeActionClient(ready=spin_ready)
    backend = Nav2SearchBackend(
        path_client=path_client,
        navigate_client=navigate_client,
        spin_client=spin_client,
        occupancy_grid_provider=lambda: grid,
        localization_valid_provider=lambda: localization_valid,
        pose_provider=(lambda: pose) if has_pose_provider else None,
        speed_limit_setter=speed_setter,
        stop_publisher=stop_publisher,
        dry_run=dry_run,
        now=clock or Clock(),
        plan_timeout_s=10.0,
        motion_authorizer=motion_authorizer,
        motion_preflight=motion_preflight,
        motion_releaser=motion_releaser,
        translation_speed_mps=translation_speed_mps,
        goal_xy_tolerance_m=goal_xy_tolerance_m,
    )
    backend._test_spin_client = spin_client
    return backend, path_client, navigate_client


def _pose(x: float = 0.0, y: float = 0.0, yaw_deg: float = 0.0) -> "RobotPose":
    return RobotPose(x=x, y=y, yaw=math.radians(yaw_deg))


def _goal(goal_id: str = "g1", *, yaw_deg: float = 0.0, frame: str = "map",
          position=(2.0, 0.0)) -> ExplorationGoal:
    return ExplorationGoal(
        goal_id=goal_id,
        goal_type="NAVIGATE_POSE",
        position=position,
        yaw=yaw_deg,
        frame=frame,
    )


def _drive_to_navigating(backend, path_client, navigate_client, grid=None):
    """跑到「已经发出 NavigateToPose」这一步，返回 handle。"""
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal())
    status = backend.get_navigation_status(handle)
    assert status.status == NavigationStatus.RUNNING
    assert len(navigate_client.sent) == 1
    return handle


# ---------------------------------------------------------------------------
# §12.2 先规划后导航
# ---------------------------------------------------------------------------


def test_navigate_pose_plans_first_then_navigates() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))

    handle = backend.execute_goal(_goal(yaw_deg=90.0))
    # 规划已发出，导航还没发。
    assert len(path_client.sent) == 1
    assert navigate_client.sent == []
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING

    request = navigate_client.sent[0]
    assert request.x == pytest.approx(2.0)
    assert request.y == pytest.approx(0.0)
    # ExplorationGoal.yaw 是角度，Nav2 要弧度。
    assert request.yaw == pytest.approx(math.pi / 2.0)
    assert request.frame_id == "map"


def test_the_planned_request_keeps_the_goal_position() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal(position=(1.5, -0.75), yaw_deg=-45.0))
    backend.get_navigation_status(handle)
    sent = path_client.sent[0]
    assert sent.x == pytest.approx(1.5)
    assert sent.y == pytest.approx(-0.75)
    assert sent.yaw == pytest.approx(math.radians(-45.0))


def test_success_maps_to_succeeded() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_SUCCEEDED))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.SUCCEEDED
    assert result.provenance["candidate_decision"]["accepted"] is True


def test_abort_maps_to_failed() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_ABORTED, error="stuck"))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.FAILED
    assert result.message == "stuck"


def test_cancel_maps_to_cancelled() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_CANCELED))
    assert backend.get_navigation_status(handle).status == NavigationStatus.CANCELLED


def test_executing_keeps_running() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_EXECUTING))
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING


# ---------------------------------------------------------------------------
# §12.2 拒绝路径绝不发 NavigateToPose
# ---------------------------------------------------------------------------


def test_empty_path_never_calls_navigate_to_pose() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=()))
    handle = backend.execute_goal(_goal())

    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NO_PATH
    assert result.provenance["motion_action_sent"] is False
    assert navigate_client.sent == []


def test_occupied_goal_is_rejected_before_planning() -> None:
    """计划书 §3.2 / §14：目标落在占据栅格里 → ``GOAL_OCCUPIED``。

    不在这里拦的话，NavFn 对占据终点多半返回空路径，拒因会被记成 ``NO_PATH``
    —— 结果是同一个「不发 NavigateToPose」，但现场分不清是「过不去」还是
    「目标点本身就在墙里」。这个枚举此前没有任何产生点。
    """
    # (2.0, 2.0) 落在 _corridor_grid 画出来的上侧墙里。
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = backend.execute_goal(_goal(position=(2.0, 2.0)))

    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_GOAL_OCCUPIED
    assert path_client.sent == []
    assert navigate_client.sent == []


def test_unknown_goal_cell_is_left_to_the_planner() -> None:
    """未知栅格不在这里拦 —— 拦了会把正常的 frontier 目标一起误杀。

    未知区域的处理归 costmap（``allow_unknown: false``）与走廊分析器
    （§7.3），本处只判「已探明是障碍」这一种。
    """
    grid = _paint(_free_grid(), -1.0, 1.0, -1.0, 1.0, -1)
    backend, path_client, _ = _backend(grid=grid)
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    backend.execute_goal(_goal(position=(0.0, 0.0)))

    assert len(path_client.sent) == 1, "未知目标点必须照常送去规划"


def test_too_narrow_never_calls_navigate_to_pose() -> None:
    # 半宽 0.55 m ⇒ 机身净空约 0.30 m，低于 0.40 m 硬门。
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(0.55))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal())

    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_TOO_NARROW
    assert navigate_client.sent == []


def test_dry_run_checks_path_without_sending_navigation_or_speed_commands() -> None:
    speed = SpeedRecorder()
    stops: list[str] = []
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(1.25), speed_setter=speed,
        stop_publisher=lambda: stops.append("stop"), dry_run=True,
    )
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    result = backend.get_navigation_status(backend.execute_goal(_goal()))

    assert result.status == NavigationStatus.CANCELLED
    assert result.provenance["dry_run"] is True
    assert result.provenance["candidate_decision"]["accepted"] is True
    assert len(path_client.sent) == 1
    assert navigate_client.sent == []
    assert speed.calls == []
    backend.stop()
    assert stops == []


def test_path_that_narrows_in_the_middle_is_rejected() -> None:
    """整条路径取最窄处判定：前 1.5 m 宽、之后收窄成 0.30 m 半宽。"""
    grid = _corridor_grid(1.25, narrow_from_x=1.50, narrow_half_width_m=0.30)
    backend, path_client, navigate_client = _backend(grid=grid)
    path_client.queue(
        ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path(length_m=3.0))
    )
    handle = backend.execute_goal(_goal(position=(3.0, 0.0)))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_TOO_NARROW
    assert navigate_client.sent == []


def test_planning_abort_never_calls_navigate_to_pose() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    path_client.queue(ActionPoll(status=ACTION_ABORTED, error="no plan"))
    handle = backend.execute_goal(_goal())
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NO_PATH
    assert navigate_client.sent == []


def test_navigation_failure_has_no_experimental_fallback() -> None:
    """导航失败就失败，不转投实验后端、也不重发相对前进命令。"""
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_ABORTED, error="blocked"))
    assert backend.get_navigation_status(handle).status == NavigationStatus.FAILED

    assert len(path_client.sent) == 1
    assert len(navigate_client.sent) == 1
    assert not hasattr(backend, "experimental_backend")
    assert not hasattr(backend, "fallback_backend")


# ---------------------------------------------------------------------------
# §12.2 RELATIVE_MOVE 明确拒绝
# ---------------------------------------------------------------------------


def test_relative_move_is_rejected_and_sends_nothing() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    goal = ExplorationGoal(goal_id="r1", goal_type="RELATIVE_MOVE", relative_dx=0.3)
    handle = backend.execute_goal(goal)

    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert "RELATIVE_MOVE" in result.message
    # 关键：没有把它翻译成任何底层前进命令。
    assert path_client.sent == []
    assert navigate_client.sent == []


def test_relative_rotation_is_rejected_too() -> None:
    backend, path_client, _ = _backend(grid=_corridor_grid(1.25))
    goal = ExplorationGoal(goal_id="r2", goal_type="ROTATE_VIEW", relative_dyaw=15.0)
    result = backend.get_navigation_status(backend.execute_goal(goal))
    assert result.status == NavigationStatus.REJECTED
    assert path_client.sent == []


def test_capabilities_do_not_advertise_relative_motion() -> None:
    backend, _, _ = _backend(grid=_corridor_grid(1.25))
    capabilities = backend.capabilities()
    assert capabilities.supports_metric_navigation is True
    assert capabilities.supports_relative_translation is False
    assert capabilities.supports_relative_rotation is False
    assert capabilities.allowed_motion_primitives == SUPPORTED_GOAL_TYPES


# ---------------------------------------------------------------------------
# 就绪门
# ---------------------------------------------------------------------------


def test_missing_action_server_rejects_before_planning() -> None:
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(1.25), navigate_ready=False
    )
    result = backend.get_navigation_status(backend.execute_goal(_goal()))
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NAV2_NOT_READY
    assert path_client.sent == []


def test_invalid_localization_rejects() -> None:
    backend, path_client, _ = _backend(
        grid=_corridor_grid(1.25), localization_valid=False
    )
    result = backend.get_navigation_status(backend.execute_goal(_goal()))
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NO_LOCALIZATION
    assert path_client.sent == []


def test_missing_map_rejects() -> None:
    backend, path_client, _ = _backend(grid=None)
    result = backend.get_navigation_status(backend.execute_goal(_goal()))
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NO_MAP
    assert path_client.sent == []


def test_health_reports_the_blocking_reason() -> None:
    backend, _, _ = _backend(grid=None)
    health = backend.health()
    assert not health.ready
    assert REJECT_NO_MAP in health.degraded
    assert health.details["reject_reason"] == REJECT_NO_MAP


def test_health_is_ready_when_everything_is_present() -> None:
    backend, _, _ = _backend(grid=_corridor_grid(1.25))
    assert backend.health().ready


def test_odom_frame_goals_are_refused() -> None:
    backend, path_client, _ = _backend(grid=_corridor_grid(1.25))
    result = backend.get_navigation_status(backend.execute_goal(_goal(frame="odom")))
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NOT_MAP_FRAME
    assert path_client.sent == []


# ---------------------------------------------------------------------------
# 速度上限与候选记录
# ---------------------------------------------------------------------------


def test_speed_limit_follows_the_corridor_width() -> None:
    recorder = SpeedRecorder()
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(1.25), speed_setter=recorder
    )
    _drive_to_navigating(backend, path_client, navigate_client)
    assert recorder.calls == [pytest.approx(0.10)]


def test_narrow_but_passable_corridor_lowers_the_speed_limit() -> None:
    recorder = SpeedRecorder()
    # 半宽 0.90 m ⇒ 净空约 0.645 m，落在 [0.40, 0.80) 之间。
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(0.90), speed_setter=recorder
    )
    _drive_to_navigating(backend, path_client, navigate_client)
    assert len(recorder.calls) == 1
    assert 0.05 <= recorder.calls[0] < 0.10


def test_unwritable_speed_limit_stops_before_navigate_to_pose() -> None:
    """写不进 RPP 的限速就不许发 NavigateToPose（计划书 §7.4 / §3.2）。

    否则走廊算出的 0.05~0.10 只是纸面数字，控制器照 YAML 的 0.10 跑——
    这类「算出来但没生效」的静默降级正是要消灭的东西。
    """

    def refusing_setter(speed_limit_mps: float) -> bool:
        return False

    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(0.90), speed_setter=refusing_setter
    )
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal())
    result = backend.get_navigation_status(handle)

    assert result.status == NavigationStatus.REJECTED
    assert navigate_client.sent == []
    record = result.provenance["candidate_decision"]
    assert record["reject_reason"] == REJECT_NAV2_NOT_READY
    assert record["accepted"] is False
    # 候选记录里的限速必须还是走廊算出来的那个，不能被悄悄改成 0 或 None。
    assert 0.05 <= record["speed_limit_mps"] < 0.10


def test_rejected_candidate_never_sets_a_speed_limit() -> None:
    recorder = SpeedRecorder()
    backend, path_client, _ = _backend(grid=_corridor_grid(0.55), speed_setter=recorder)
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    backend.get_navigation_status(backend.execute_goal(_goal()))
    assert recorder.calls == []


def test_candidate_decision_carries_the_plan_section_14_fields() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_SUCCEEDED))
    record = backend.get_navigation_status(handle).provenance["candidate_decision"]
    for key in (
        "candidate_id",
        "goal_x",
        "goal_y",
        "goal_yaw",
        "path_length_m",
        "min_left_body_gap_m",
        "min_right_body_gap_m",
        "min_side_gap_m",
        "corridor_decision",
        "speed_limit_mps",
        "center_score",
        "accepted",
        "reject_reason",
    ):
        assert key in record, key
    assert record["candidate_id"] == "g1"
    assert record["corridor_decision"] == "OK"
    assert record["path_length_m"] == pytest.approx(2.0, abs=0.05)


# ---------------------------------------------------------------------------
# 并发、取消与停止
# ---------------------------------------------------------------------------


def test_a_second_goal_while_busy_is_rejected() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    _drive_to_navigating(backend, path_client, navigate_client)
    second = backend.execute_goal(_goal(goal_id="g2", position=(1.0, 1.0)))
    assert second.result is not None
    assert second.result.status == NavigationStatus.REJECTED
    assert len(path_client.sent) == 1


def test_cancel_settles_the_active_goal() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    assert backend.cancel() is True
    assert handle.result is not None
    assert handle.result.status == NavigationStatus.CANCELLED
    # 两个 Action 都要收到取消：规划阶段的 goal 可能还挂着。
    assert path_client.cancel_calls == 1
    assert navigate_client.cancel_calls == 1


def test_cancel_frees_the_slot_for_the_next_goal() -> None:
    """取消之后必须能发下一个 goal。

    现场（2026-10-08 真机验收）栽在这里：``_wait_result`` 等 15 s 超时后
    explorer 调 ``cancel(handle)`` 重试，但 ``cancel`` 只填了 ``handle.result``、
    没让出 ``self._active``，而 ``get_navigation_status`` 对已结算的 handle 会在
    开头就 return ⇒ ``_active`` 永远清不掉，之后每一轮 cycle 的候选都是
    ``rejected / "nav2 backend already has an active goal"``，
    ``motion_steps`` 全程 0。

    分辨力：把 ``cancel`` 里那句 ``self._active = None`` 去掉，本测试失败
    （``assert second.result is None`` 挂），同文件里其余 cancel 用例照过
    —— 它们只查 handle 是否被结算，查不出槽位没让出来。
    """
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    first = _drive_to_navigating(backend, path_client, navigate_client)

    backend.cancel(first)

    # 光调 get_navigation_status 救不回来：handle 已结算，走的是开头那句 return。
    assert backend.get_navigation_status(first).status == NavigationStatus.CANCELLED

    second = backend.execute_goal(_goal(goal_id="g2", position=(1.0, 1.0)))
    assert second.result is None, second.result and second.result.message
    assert len(path_client.sent) == 2


def test_stop_goal_cancels_and_publishes_a_zero_velocity() -> None:
    calls: list[int] = []
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(1.25), stop_publisher=lambda: calls.append(1)
    )
    active = _drive_to_navigating(backend, path_client, navigate_client)

    stop_goal = ExplorationGoal(goal_id="s1", goal_type="STOP")
    handle = backend.execute_goal(stop_goal)
    assert handle.result is not None
    assert handle.result.status == NavigationStatus.SUCCEEDED
    assert active.result is not None
    assert active.result.status == NavigationStatus.CANCELLED
    assert calls == [1]


def test_settled_result_is_returned_again_without_re_advancing() -> None:
    backend, path_client, navigate_client = _backend(grid=_corridor_grid(1.25))
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_SUCCEEDED))
    first = backend.get_navigation_status(handle)
    assert first.status == NavigationStatus.SUCCEEDED
    # 再问一次不该把已经结束的目标重新推进一步。
    assert backend.get_navigation_status(handle) is first
    assert len(navigate_client.sent) == 1


def test_planning_timeout_rejects() -> None:
    clock = Clock()
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(1.25), clock=clock
    )
    path_client.queue(ActionPoll(status=ACTION_EXECUTING))
    handle = backend.execute_goal(_goal())
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    clock.advance(11.0)
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert "timeout" in result.message
    assert navigate_client.sent == []


def test_candidate_reject_reason_is_closed_over_the_plan_enum() -> None:
    """计划书 §14：候选记录的 reject_reason 必须是固定六项之一。

    走廊分析器内部还有更细的原因串（``NO_PATH_SAMPLES``：路径一个采样点
    都取不到）。它不能被原样写进候选记录——现场的判读脚本按枚举分箱，
    多一个值就多一个没人认识的分箱。
    """
    from app.navigation.nav2_search_backend import (
        CANDIDATE_REJECT_REASONS,
        _candidate_reject_reason,
    )

    assert CANDIDATE_REJECT_REASONS == (
        "NO_MAP",
        "NO_LOCALIZATION",
        "NO_PATH",
        "GOAL_OCCUPIED",
        "TOO_NARROW",
        "NAV2_NOT_READY",
    )
    for reason in CANDIDATE_REJECT_REASONS:
        assert _candidate_reject_reason(reason) == reason
    # 枚举外的一律收口，且不能变成空串。
    assert _candidate_reject_reason("NO_PATH_SAMPLES") == REJECT_TOO_NARROW
    assert _candidate_reject_reason("") == REJECT_TOO_NARROW


def test_blocked_path_is_reported_inside_the_enum() -> None:
    """整条路径都在可通行区之外的候选，记录里的原因也必须落在 §14 枚举内。"""
    backend, path_client, _ = _backend(grid=_corridor_grid(1.25))
    path_client.queue(
        ActionPoll(status=ACTION_SUCCEEDED, path=[(900.0, 900.0), (901.0, 900.0)])
    )
    handle = backend.execute_goal(_goal(position=(901.0, 900.0)))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    record = result.provenance["candidate_decision"]
    assert record["reject_reason"] in CANDIDATE_REJECT_REASONS
    assert record["min_side_gap_m"] < 0.0


def test_speed_limit_acknowledgement_precedes_navigation():
    answers = iter([None, None, True])
    backend, path, navigate = _backend(grid=_corridor_grid(1.25), speed_setter=lambda _: next(answers))
    handle = backend.execute_goal(_goal())
    path.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    assert not navigate.sent
    backend.get_navigation_status(handle)
    assert not navigate.sent
    backend.get_navigation_status(handle)
    assert len(navigate.sent) == 1


def test_localization_loss_cancels_inflight_navigation_and_stops():
    valid = [True]
    stopped = []
    clock = Clock()
    backend, path, navigate = _backend(
        grid=_corridor_grid(1.25), clock=clock,
        stop_publisher=lambda: stopped.append(True))
    backend._localization_valid = lambda: valid[0]
    handle = _drive_to_navigating(backend, path, navigate)
    valid[0] = False
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    assert navigate.cancel_calls == 0 and not stopped
    # The bridge still stops immediately.  A measured 0.34 s scheduler/source
    # timestamp stall must not permanently cancel the action, while a true
    # >=0.50 s loss still does.
    clock.advance(.34)
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    assert navigate.cancel_calls == 0 and not stopped
    clock.advance(.17)
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance['reject_reason'] == REJECT_NO_LOCALIZATION
    assert navigate.cancel_calls == 1 and stopped


def test_single_localization_jitter_sample_holds_then_recovers_navigation():
    valid = [True]
    clock = Clock()
    backend, path, navigate = _backend(grid=_corridor_grid(1.25), clock=clock)
    backend._localization_valid = lambda: valid[0]
    handle = _drive_to_navigating(backend, path, navigate)
    valid[0] = False
    held = backend.get_navigation_status(handle)
    assert held.status == NavigationStatus.RUNNING
    assert held.provenance['localization_hold'] is True
    clock.advance(.10)
    valid[0] = True
    resumed = backend.get_navigation_status(handle)
    assert resumed.status == NavigationStatus.RUNNING
    assert navigate.cancel_calls == 0


def test_three_metre_cap_rejects_a_long_detour_even_when_goal_is_near():
    backend, path, navigate = _backend(grid=_corridor_grid(1.25))
    backend._max_path_length_m = 3.
    handle = backend.execute_goal(_goal(position=(1.,0.)))
    path.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0.,0.),(2.,0.),(0.,0.),(1.,0.))))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert 'step limit' in result.message
    assert '3.000 m per-step limit' in result.message
    assert not navigate.sent


def test_the_rejection_message_names_the_runs_own_limit_not_a_hardcoded_three():
    """§P3-8：上限必须印成**本次运行**的值。

    真机验收档把上限定成 0.10~0.12 m（`min(3., forward_step_m)`），而消息里原来
    写死"the 3 m step limit"：一次 0.2 m 的路径被拒，报告却说它超过了 3 m ——
    读的人只会得出"目标很合理、后端算错了"。0.2 m > 0.12 m 才是真因。
    """
    backend, path, navigate = _backend(grid=_corridor_grid(1.25))
    backend._max_path_length_m = 0.12
    handle = backend.execute_goal(_goal(position=(0.2, 0.)))
    path.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0., 0.), (0.1, 0.), (0.2, 0.))))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance['reject_reason'] == REJECT_NO_PATH
    assert '0.200 m' in result.message and '0.120 m per-step limit' in result.message
    assert '3 m' not in result.message
    assert not navigate.sent


def test_a_path_inside_the_runs_own_limit_is_still_accepted():
    """反向那一刀：把上限印出来不能顺手把判据改了。"""
    backend, path, navigate = _backend(grid=_corridor_grid(1.25))
    backend._max_path_length_m = 0.12
    handle = backend.execute_goal(_goal(position=(0.1, 0.)))
    path.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0., 0.), (0.05, 0.), (0.1, 0.))))
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    assert len(navigate.sent) == 1


def test_missing_motion_authorization_never_sends_navigation_after_planning():
    backend, path, navigate = _backend(grid=_corridor_grid(1.25))
    backend._motion_authorizer = lambda: False
    handle = backend.execute_goal(_goal())
    path.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert 'authorization' in result.message
    assert not navigate.sent


def test_motion_preflight_rejects_before_durable_trial_reservation():
    """A missing launcher/safety admission must not burn a field ledger slot."""

    class Trial:
        MAX_SPEED_MPS = .12
        forward_only = False
        bounded_search = True
        token = None

        def __init__(self):
            self.reserve_calls = 0
            self.finish_calls = 0

        def reserve(self, **kwargs):
            self.reserve_calls += 1

        def finish(self):
            self.finish_calls += 1

    trial = Trial()
    backend, path, navigate = _backend(
        grid=_corridor_grid(1.25), motion_preflight=lambda: False,
        motion_authorizer=lambda: pytest.fail("arm must not be attempted"),
    )
    backend._trial_budget = trial
    backend._trial_pose_provider = lambda: _pose()
    handle = backend.execute_goal(_goal())
    path.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    result = backend.get_navigation_status(handle)

    assert result.status == NavigationStatus.REJECTED
    assert "motion preflight" in result.message
    assert result.provenance["motion_action_sent"] is False
    assert trial.reserve_calls == 0
    assert trial.finish_calls == 1
    assert navigate.sent == []


# ---------------------------------------------------------------------------
# §7.2 平移走廊 vs 原地转向扫掠
# ---------------------------------------------------------------------------
#
# 现场（2026-10-15 只读搜索）：一个平移 0.008 m、目标 yaw≈30° 的 local_scan 目标
# 被判 TOO_NARROW，判它的是*平移走廊* —— 那条退化路径的左右射线打的是机器人自己
# 身边的格，跟这次转向毫无关系。下面这批用例把两种动作的判据分开钉住。
#
# 共同地形：机器人站在原点，左前方有一段矮墙。沿 +x 的走廊量出来左净空 0.095 m
# ⇒ 走廊判据必然 TOO_NARROW；而 30° 的左转扫不到那堵墙 ⇒ 扫掠判据通过。两个判据
# 在这里给出相反结论，正是要分辨的东西。


def _turn_test_grid() -> CorridorGrid:
    """左前方一堵墙：平移走廊判窄，30° 左转扫不到。"""
    return _paint(_free_grid(), -0.10, 0.05, 0.46, 0.65, 100)


def _micro_path(dx: float = 0.03) -> tuple[tuple[float, float], ...]:
    return ((0.0, 0.0), (dx, 0.0))


def _micro_goal(*, yaw_deg: float, dx: float = 0.03) -> ExplorationGoal:
    return _goal(position=(dx, 0.0), yaw_deg=yaw_deg)


def _settle_navigation(backend, handle, navigate_client, *, poll=ACTION_SUCCEEDED):
    """推进到发完 NavigateToPose，再喂一个终态结算掉，返回最终结果。

    RUNNING 那条结果不带候选记录（``{"backend": ...}`` 而已），要看判据必须结算。
    """
    running = backend.get_navigation_status(handle)
    assert running.status == NavigationStatus.RUNNING, running.message
    assert len(navigate_client.sent) == 1, "NavigateToPose 还没发出去"
    navigate_client.queue(ActionPoll(status=poll))
    return backend.get_navigation_status(handle)


def test_the_corridor_would_reject_this_path_on_its_own() -> None:
    """先把对照组钉死：这条退化路径在走廊判据下确实过不去。

    没有这一条，下面「换成扫掠后通过了」就可能只是因为地形本来就很宽，而不是因为
    判据换了。
    """
    grid = _turn_test_grid()
    decision = classify_path_clearance(measure_path_clearance(_micro_path(), grid))
    assert decision.decision == DECISION_TOO_NARROW
    assert decision.min_side_gap_m is not None
    assert decision.min_side_gap_m < 0.40


def test_micro_path_with_a_real_turn_is_judged_by_the_rotation_sweep() -> None:
    """0.03 m 平移 + 30° 左转：走廊说窄、扫掠说能转 ⇒ 放行，并记下是哪一套判的。"""
    grid = _turn_test_grid()
    speed = SpeedRecorder()
    backend, path_client, navigate_client = _backend(
        grid=grid, pose=_pose(0.0, 0.0, 0.0), speed_setter=speed
    )
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_micro_path()))

    handle = backend.execute_goal(_micro_goal(yaw_deg=30.0))
    result = _settle_navigation(backend, handle, navigate_client)

    assert result.status == NavigationStatus.SUCCEEDED, result.message
    record = result.provenance["candidate_decision"]
    assert record["motion_mode"] == "rotation_in_place"
    assert record["translation_to_goal_m"] == pytest.approx(0.03, abs=1e-6)
    assert record["rotation_decision"] == "OK"
    assert record["rotation_checked_sweep_deg"] == pytest.approx(35.0)
    assert record["accepted"] is True
    # 走廊这一栏是空的（这一趟根本没跑走廊），不能悄悄填一份「通过」出来。
    assert record["corridor_decision"] == ""
    assert record["min_side_gap_m"] is None
    # 转向目标没有平移，限速取速度带下限，不是 0。
    assert speed.calls == [pytest.approx(0.05)]


def test_exact_in_place_turn_sets_zero_linear_speed() -> None:
    """精确转向不能因 map 校正退化成 0.05 m/s 的平移追点。"""
    speed = SpeedRecorder()
    backend, path_client, navigate_client = _backend(
        grid=_free_grid(), pose=_pose(0.0, 0.0, 0.0), speed_setter=speed
    )
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0.0, 0.0),)))

    handle = backend.execute_goal(_goal(position=(0.0, 0.0), yaw_deg=15.0))
    running = backend.get_navigation_status(handle)
    assert running.status == NavigationStatus.RUNNING
    spin_client = backend._test_spin_client
    assert navigate_client.sent == []
    assert len(spin_client.sent) == 1
    assert spin_client.sent[0].target_yaw == pytest.approx(math.radians(15.0))
    assert spin_client.sent[0].time_allowance_s == pytest.approx(15.0)
    spin_client.queue(ActionPoll(status=ACTION_SUCCEEDED))
    result = backend.get_navigation_status(handle)

    assert result.status == NavigationStatus.SUCCEEDED
    assert result.provenance["candidate_decision"]["motion_mode"] == "rotation_in_place"


def test_exact_turn_with_navfn_start_cell_quantization_uses_spin() -> None:
    """A 1.9 cm planner stub must not route a zero-distance turn through RPP."""
    speed = SpeedRecorder()
    backend, path_client, navigate_client = _backend(
        grid=_free_grid(), pose=_pose(), speed_setter=speed
    )
    path_client.queue(ActionPoll(
        status=ACTION_SUCCEEDED, path=((0.0, 0.0), (0.019, 0.0))
    ))

    handle = backend.execute_goal(
        _goal(position=(0.0, 0.0), yaw_deg=10.0)
    )
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    assert navigate_client.sent == []
    assert len(backend._test_spin_client.sent) == 1
    assert backend._test_spin_client.sent[0].target_yaw == pytest.approx(
        math.radians(10.0)
    )
    assert speed.calls == [pytest.approx(0.0)]


def test_exact_turn_fails_closed_when_spin_server_is_unavailable() -> None:
    backend, path_client, navigate_client = _backend(
        grid=_free_grid(), pose=_pose(), spin_ready=False
    )
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0.0, 0.0),)))
    handle = backend.execute_goal(_goal(position=(0.0, 0.0), yaw_deg=10.0))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_NAV2_NOT_READY
    assert navigate_client.sent == []
    assert backend._test_spin_client.sent == []


def test_exact_turn_cancels_on_ten_centimeter_odometry_drift() -> None:
    tracking = [_pose()]
    stops = []
    backend, path_client, navigate_client = _backend(
        grid=_free_grid(), pose=_pose(), stop_publisher=lambda: stops.append(True)
    )
    backend._trial_pose_provider = lambda: tracking[0]
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0.0, 0.0),)))
    handle = backend.execute_goal(_goal(position=(0.0, 0.0), yaw_deg=10.0))
    assert backend.get_navigation_status(handle).status == NavigationStatus.RUNNING
    tracking[0] = _pose(x=0.101)
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert "translation drift limit" in result.message
    assert backend._test_spin_client.cancel_calls == 1
    assert stops == [True]


def test_trial_arrival_uses_relative_odometry_across_a_mapping_pose_jump() -> None:
    """SLAM 的 map 修正不能把一次物理到达误判成偏离目标。"""
    class Trial:
        token = None
        def observe(self, pose):
            return ""

    map_pose = [_pose(0.0, 0.0, 0.0)]
    odom_pose = [_pose(10.0, 4.0, 30.0)]
    backend, _, navigate = _backend(
        grid=_free_grid(), pose=map_pose[0], goal_xy_tolerance_m=.03
    )
    backend._trial_budget = Trial()
    backend._trial_pose_provider = lambda: odom_pose[0]
    goal = _goal(position=(.15, 0.0), yaw_deg=0.0)
    active = _ActiveGoal(
        goal=goal, handle=NavigationHandle(goal.goal_id), phase="NAVIGATING",
        request=Nav2GoalRequest(x=.15, y=0.0, yaw=0.0),
        started_at=0.0, plan_sent_at=0.0, path_length_m=.15,
        planning_pose=_pose(), tracking_start=odom_pose[0],
    )
    backend._active = active
    # 真实里程只差目标 2 cm；map 同时跳了 30 cm。
    heading = math.radians(30.0)
    odom_pose[0] = RobotPose(
        x=10.0 + .13*math.cos(heading),
        y=4.0 + .13*math.sin(heading), yaw=heading,
    )
    map_pose[0] = _pose(.45, .0, 0.0)
    backend._pose_provider = lambda: map_pose[0]
    navigate.queue(ActionPoll(status=ACTION_SUCCEEDED))
    result = backend._advance_navigating(active)
    assert result is not None and result.status == NavigationStatus.SUCCEEDED
    assert result.observed_motion["final_position_error_m"] == pytest.approx(.02)
    assert result.observed_motion["odometry_translation_m"] == pytest.approx(.13)


def test_micro_path_with_a_blocked_turn_is_rejected_as_too_narrow() -> None:
    """同一条退化路径，把墙挪进扫掠区 ⇒ 拒，且拒因指明是扫掠。"""
    grid = _paint(_free_grid(), -0.10, 0.10, 0.31, 0.60, 100)
    backend, path_client, navigate_client = _backend(grid=grid, pose=_pose(0.0, 0.0, 0.0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_micro_path()))

    result = backend.get_navigation_status(
        backend.execute_goal(_micro_goal(yaw_deg=30.0))
    )

    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_TOO_NARROW
    assert "rotation sweep" in result.message
    assert navigate_client.sent == []

    record = result.provenance["candidate_decision"]
    assert record["motion_mode"] == "rotation_in_place"
    assert record["rotation_decision"] == "ROTATION_SWEEP_BLOCKED"
    assert record["rotation_blocked_samples"] > 0
    assert record["rotation_nearest_blocker_m"] is not None
    assert record["rotation_started_blocked"] is False


def test_micro_path_without_a_real_turn_still_goes_through_the_corridor() -> None:
    """平移 ≈0、没有转向：扫掠判据不适用，走廊判据照旧说了算。

    防的是「为了让动作走起来就把 unknown/净空检查删掉」：判据只是**换**，不是
    **撤**。同一张图、同一个位姿，只要不转向就还是被拒。
    """
    grid = _turn_test_grid()
    backend, path_client, navigate_client = _backend(grid=grid, pose=_pose(0.0, 0.0, 0.0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_micro_path()))

    result = backend.get_navigation_status(
        backend.execute_goal(_micro_goal(yaw_deg=0.0))
    )

    assert result.status == NavigationStatus.REJECTED
    assert result.provenance["reject_reason"] == REJECT_TOO_NARROW
    assert "corridor rejected" in result.message
    record = result.provenance["candidate_decision"]
    assert record["motion_mode"] == "rotation_in_place"
    assert record["rotation_decision"] == "NOT_APPLICABLE"
    assert record["corridor_decision"] == DECISION_TOO_NARROW
    assert navigate_client.sent == []


def test_a_long_path_is_never_judged_by_the_rotation_sweep() -> None:
    """目标在 2 m 外：即使目标偏航差 90°，判据也必须还是走廊。"""
    grid = _corridor_grid(1.25)
    backend, path_client, navigate_client = _backend(grid=grid, pose=_pose(0.0, 0.0, 0.0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))

    handle = backend.execute_goal(_goal(position=(2.0, 0.0), yaw_deg=90.0))
    result = _settle_navigation(backend, handle, navigate_client)

    record = result.provenance["candidate_decision"]
    assert record["motion_mode"] == "translation"
    assert record["translation_to_goal_m"] == pytest.approx(2.0, abs=1e-6)
    assert record["corridor_decision"] == DECISION_OK
    # 扫掠只作为参考记录，不参与判定。
    assert "rotation_decision" in record


def test_a_blocked_initial_turn_does_not_reject_a_translation_goal() -> None:
    """平移目标的起步转向只是**记录**，不构成拒绝理由。

    RPP 起步转的是路径切向、到点才转到目标偏航，也可能反过来；按「当前位置 +
    目标偏航」判扫掠，判的是一条 Nav2 未必会走的轨迹。这里地形让扫掠判「挡住」，
    但走廊是通的，目标必须照发 —— 锁住这个决定，免得以后有人顺手把它变成一道门。

    墙故意摆在机身正前方的左边一点点（x∈[−0.03, 0.18]，y≥0.375）：它落在
    yaw=0 的机身之外，所以起步那一刻不压；但左转 90° 会扫到。而走廊把路径两端
    各裁掉 0.20 m，第一个采样点在 x=0.20，正好从墙的右侧过去 —— 走廊判据看不见
    它。这正是「走廊管不到起步那一段转」的现实版。
    """
    grid = _paint(_free_grid(), -0.05, 0.15, 0.36, 0.65, 100)
    backend, path_client, navigate_client = _backend(grid=grid, pose=_pose(0.0, 0.0, 0.0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))

    handle = backend.execute_goal(_goal(position=(2.0, 0.0), yaw_deg=90.0))
    # 没被扫掠拦下：前进一步就该把 NavigateToPose 发出去。
    result = _settle_navigation(backend, handle, navigate_client)

    record = result.provenance["candidate_decision"]
    assert record["motion_mode"] == "translation"
    assert record["rotation_decision"] == "ROTATION_SWEEP_BLOCKED"
    assert record["corridor_decision"] == DECISION_OK
    assert result.status == NavigationStatus.SUCCEEDED


def test_a_long_detour_to_a_nearby_goal_is_not_a_rotation() -> None:
    """目标点就在身边、路径却绕了 1 m：这是平移，不是原地转向。

    只看「目标点多远」会把这种目标误判成转向，于是平移走廊被跳过、扫掠又只查了
    转向那一圈，两次都判错。两个量都退化才算原地转向。
    """
    grid = _corridor_grid(1.25)
    detour = ((0.0, 0.0), (0.5, 0.5), (1.0, 0.5), (1.0, 0.0), (0.03, 0.0))
    backend, path_client, navigate_client = _backend(grid=grid, pose=_pose(0.0, 0.0, 0.0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=detour))

    handle = backend.execute_goal(_micro_goal(yaw_deg=30.0, dx=0.03))
    result = _settle_navigation(backend, handle, navigate_client)

    record = result.provenance["candidate_decision"]
    assert record["translation_to_goal_m"] == pytest.approx(0.03, abs=1e-6)
    assert record["path_length_m"] > 0.10
    assert record["motion_mode"] == "translation"
    assert record["corridor_decision"] == DECISION_OK


def test_missing_pose_rejects_without_guessing_the_initial_footprint() -> None:
    """定位布尔量仍绿、当前 TF 已失效时，也不能用路径起点冒充机身位姿。"""
    grid = _free_grid()
    for label, kwargs in (
        ("pose 为 None", {"pose": None}),
        ("没有 pose provider", {"has_pose_provider": False}),
    ):
        backend, path_client, _ = _backend(grid=grid, **kwargs)
        path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_micro_path()))
        result = backend.get_navigation_status(
            backend.execute_goal(_micro_goal(yaw_deg=30.0))
        )
        assert result.status == NavigationStatus.REJECTED, label
        assert result.provenance['reject_reason'] == REJECT_NO_LOCALIZATION, label
        record = result.provenance["candidate_decision"]
        assert record["motion_mode"] == "unknown", label
        assert record["translation_to_goal_m"] is None, label
        assert record["corridor_decision"] == "", label
        assert "rotation_decision" not in record, label


# ---------------------------------------------------------------------------
# §P3-5：/map 进走廊分析前的入口校验
#
# 健康监视器按 `evaluate_localization` 的地图门拒图，后端此前一项都不查。
# 监视器已覆盖的那些，后端是*间接*安全的（valid=False 会让 readiness 先报
# NO_LOCALIZATION）；监视器**没**覆盖的两项（原点有限值、origin 四元数）
# 完全没有兜底，而且它们的失败方式不是"判不通过"而是异常 —— 见下面
# test_corridor_grid_really_raises_on_those_maps 把这条前提钉住。
# ---------------------------------------------------------------------------


class _Stub:
    def __init__(self, **fields) -> None:
        self.__dict__.update(fields)


def _map_message(*, frame="map", width=4, height=4, resolution=0.05,
                 origin_x=0.0, origin_y=0.0, orientation=(0.0, 0.0, 0.0, 1.0),
                 cells=None):
    qx, qy, qz, qw = orientation
    return _Stub(
        header=_Stub(frame_id=frame),
        info=_Stub(
            width=width, height=height, resolution=resolution,
            origin=_Stub(position=_Stub(x=origin_x, y=origin_y),
                         orientation=_Stub(x=qx, y=qy, z=qz, w=qw)),
        ),
        data=[0] * (width * height) if cells is None else cells,
    )


def test_a_well_formed_map_is_accepted() -> None:
    assert map_message_reject_reason(_map_message()) == ""


@pytest.mark.parametrize(
    "label, kwargs, needle",
    [
        # 图上有 pslam_map 这另一个世界系，认错了不会报错，只会让"自由格"
        # 和机器人的 map 位姿对不上。
        ("frame 不是 map", {"frame": "pslam_map"}, "frame="),
        ("frame 带前导斜杠也认", {"frame": "/map"}, None),
        ("分辨率 NaN", {"resolution": float("nan")}, "resolution="),
        ("分辨率 0", {"resolution": 0.0}, "resolution="),
        ("分辨率负", {"resolution": -0.05}, "resolution="),
        ("原点 x 为 inf", {"origin_x": float("inf")}, "origin="),
        ("原点 y 为 NaN", {"origin_y": float("nan")}, "origin="),
        ("宽为 0", {"width": 0}, "size="),
        ("高为负", {"height": -1}, "size="),
        ("data 长度不符", {"cells": [0, 0, 0]}, "cells="),
        # CorridorGrid 反算坐标时没有旋转项，带旋转的图必须拒绝而不是
        # 静默按轴对齐去读 —— 那读出来的是别的坐标系里的格子。
        ("origin 带 90° 旋转", {"orientation": (0.0, 0.0, 0.70710678, 0.70710678)},
         "origin_rotated="),
    ],
)
def test_bad_maps_are_rejected_with_a_readable_reason(label, kwargs, needle) -> None:
    reason = map_message_reject_reason(_map_message(**kwargs))
    if needle is None:
        assert reason == "", label
    else:
        assert needle in reason, f"{label}: {reason!r}"


def test_corridor_grid_really_raises_on_those_maps() -> None:
    """钉住上面那条论证的前提：这两类坏图在 CorridorGrid 里是**异常**，不是"挡"。

    没有这条，上面那段"入口校验是为了把异常变成一条可读原因"就只是推测。
    """
    for label, resolution, origin_x, expected in (
        ("分辨率 NaN", float("nan"), 0.0, ValueError),
        ("原点 inf", 0.05, float("inf"), OverflowError),
    ):
        grid = CorridorGrid(resolution=resolution, origin_x=origin_x, origin_y=0.0,
                            width=4, height=4, data=[0] * 16)
        with pytest.raises(expected):
            grid.blocked(0.1, 0.1)
        # 也就是说它既不是"通过"也不是"挡住"，而是把异常抛给走廊分析。
        assert map_message_reject_reason(
            _map_message(resolution=resolution, origin_x=origin_x)
        ) != "", label


# ---------------------------------------------------------------------------
# §P3-2：限速服务 ACK 的异步状态、跨目标在途请求、取消后晚到的响应
#
# 后端里只有这一处是"发出去一个请求、要等下一轮才知道成不成"，而且这个在途槽
# **不属于任何一次导航**。所以要钉的不是"能不能写进去"，而是"结论是不是这一次
# 的"：一个刚落地、说的是**旧速度**的 ACK，不能被当成新目标的结论。
# ---------------------------------------------------------------------------


class FakeFuture:
    def __init__(self) -> None:
        self._done = False
        self._result = None
        self._error: Exception | None = None

    def done(self) -> bool:
        return self._done

    def resolve(self, successful: bool = True) -> None:
        self._result = _Stub(results=[_Stub(successful=successful)])
        self._done = True

    def resolve_empty(self) -> None:
        """服务回了一个空的 results —— 不是"成功"，是"没结论"。"""
        self._result = _Stub(results=[])
        self._done = True

    def fail(self, error: Exception) -> None:
        self._error = error
        self._done = True

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result


class FakeSpeedService:
    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.sent: list[float] = []
        self.futures: list[FakeFuture] = []

    def send(self, speed_limit_mps: float) -> FakeFuture:
        self.sent.append(float(speed_limit_mps))
        future = FakeFuture()
        self.futures.append(future)
        return future

    def service_ready(self) -> bool:
        return self.ready


def _applier(service: FakeSpeedService) -> SpeedLimitApplier:
    return SpeedLimitApplier(send=service.send, service_ready=service.service_ready)


def test_first_speed_limit_request_is_in_flight_not_applied() -> None:
    service = FakeSpeedService()
    applier = _applier(service)
    assert applier.apply(0.05) is None
    assert service.sent == [0.05]
    assert applier.pending_speed_mps == 0.05


def test_missing_service_fails_closed_without_sending() -> None:
    service = FakeSpeedService(ready=False)
    applier = _applier(service)
    assert applier.apply(0.05) is False
    assert service.sent == []


def test_same_speed_reuses_the_ack_without_resending() -> None:
    """速度相同：controller 的参数是节点级的，上一个 ACK 说的就是这个值。"""
    service = FakeSpeedService()
    applier = _applier(service)
    assert applier.apply(0.05) is None
    service.futures[0].resolve()
    assert applier.apply(0.05) is True
    assert service.sent == [0.05]
    assert applier.pending_speed_mps is None


def test_new_goal_with_a_different_speed_never_borrows_the_old_ack() -> None:
    """**核心用例**：新目标的速度和刚落地的 ACK 不同，必须重新发，不能算通过。"""
    service = FakeSpeedService()
    applier = _applier(service)
    # 上一个目标（比如已被取消的那个）请求 0.05，ACK 晚到并成功。
    assert applier.apply(0.05) is None
    service.futures[0].resolve(successful=True)
    # 新目标要 0.08。此刻 controller 里写的是 0.05，绝不能就此放行。
    assert applier.apply(0.08) is None
    assert service.sent == [0.05, 0.08], "必须按新速度重新发"
    assert applier.pending_speed_mps == 0.08
    service.futures[1].resolve()
    assert applier.apply(0.08) is True


def test_late_response_of_an_abandoned_goal_never_reads_as_pass() -> None:
    """取消之后晚到的响应：在那之前新目标一次都不能拿到 True。"""
    service = FakeSpeedService()
    applier = _applier(service)
    applier.apply(0.05)          # 目标 A 发出请求
    # ……A 在这里被取消。没有任何人再为 A 调用 apply。
    assert applier.apply(0.08) is None, "A 的请求还在途，B 只能等"
    service.futures[0].resolve(successful=True)   # A 的响应晚到
    # B 不能把这条响应读成自己的结论 —— 它写的是 0.05。
    assert applier.apply(0.08) is None
    assert service.sent == [0.05, 0.08]
    service.futures[1].resolve()
    assert applier.apply(0.08) is True


def test_failed_ack_is_reported_and_clears_the_slot() -> None:
    service = FakeSpeedService()
    applier = _applier(service)
    applier.apply(0.05)
    service.futures[0].fail(RuntimeError("service call failed"))
    assert applier.apply(0.05) is False
    assert applier.pending_speed_mps is None
    # 槽已清空，下一个目标会重新发，而不是继承一个失败的结论。
    assert applier.apply(0.05) is None
    assert service.sent == [0.05, 0.05]


def test_unsuccessful_or_empty_response_is_not_applied() -> None:
    for label, finish in (("successful=False", "reject"), ("results 为空", "empty")):
        service = FakeSpeedService()
        applier = _applier(service)
        applier.apply(0.05)
        if finish == "reject":
            service.futures[0].resolve(successful=False)
        else:
            service.futures[0].resolve_empty()
        assert applier.apply(0.05) is False, label


# ---------------------------------------------------------------------------
# §P3-3 运动授权：断言之后必须有人收回
# ---------------------------------------------------------------------------


class ArmRecorder:
    """授权/解除授权的成对记录器 —— 只记顺序，不碰任何服务。"""

    def __init__(self, *, grants: bool = True, release_raises: bool = False) -> None:
        self.grants = grants
        self.release_raises = release_raises
        self.log: list[str] = []

    def authorize(self) -> bool:
        self.log.append("arm")
        return self.grants

    def release(self) -> None:
        self.log.append("disarm")
        if self.release_raises:
            raise RuntimeError("disarm service unavailable")


def _armed_backend(grid=None, **kwargs):
    arm = ArmRecorder(**kwargs)
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(1.25) if grid is None else grid,
        motion_authorizer=arm.authorize, motion_releaser=arm.release)
    return backend, path_client, navigate_client, arm


def test_authorization_is_released_when_the_goal_ends_without_success() -> None:
    """计划书原话：「武装后发送失败须取消并解除武装」。"""
    backend, path_client, navigate_client, arm = _armed_backend()
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    assert arm.log == ["arm"]
    navigate_client.queue(ActionPoll(status=ACTION_ABORTED))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.FAILED
    assert arm.log == ["arm", "disarm"]


def test_a_successful_goal_does_not_pay_an_extra_disarm() -> None:
    """成功走完的维持原行为：多一次 arm/disarm 往返是三次服务调用起步。"""
    backend, path_client, navigate_client, arm = _armed_backend()
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_SUCCEEDED))
    assert backend.get_navigation_status(handle).status == NavigationStatus.SUCCEEDED
    assert arm.log == ["arm"]


def test_cancel_releases_the_authorization() -> None:
    backend, path_client, navigate_client, arm = _armed_backend()
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    assert backend.cancel(handle) is True
    assert arm.log == ["arm", "disarm"]


def test_stop_releases_the_authorization_too() -> None:
    """`stop()` 走的就是 `cancel()` —— 收尾路径必须也收回。"""
    backend, path_client, navigate_client, arm = _armed_backend()
    _drive_to_navigating(backend, path_client, navigate_client)
    backend.stop()
    assert arm.log == ["arm", "disarm"]


def test_a_refused_authorization_is_never_released() -> None:
    """授权被拒 = 从没武装过。此时去解除武装会打掉**别人**正在用的授权。"""
    backend, path_client, navigate_client, arm = _armed_backend(grants=False)
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal())
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.REJECTED
    assert len(navigate_client.sent) == 0
    assert arm.log == ["arm"], "没武装过就不该发解除"


def test_a_goal_that_never_reached_navigation_is_never_released() -> None:
    """走廊判 TOO_NARROW，压根走不到武装那一步。"""
    arm = ArmRecorder()
    backend, path_client, navigate_client = _backend(
        grid=_corridor_grid(0.55),
        motion_authorizer=arm.authorize, motion_releaser=arm.release)
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal())
    assert backend.get_navigation_status(handle).status == NavigationStatus.REJECTED
    assert arm.log == [], "没武装过就不该发解除"


def test_release_happens_at_most_once_per_goal() -> None:
    backend, path_client, navigate_client, arm = _armed_backend()
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_ABORTED))
    backend.get_navigation_status(handle)
    # 结算过的 handle 再问一次走的是提前 return，不该重复解除。
    backend.get_navigation_status(handle)
    backend.stop()
    assert arm.log == ["arm", "disarm"]


def test_a_failing_release_does_not_turn_settlement_into_an_exception() -> None:
    """解除失败可以接受（ARM 是租约，自己会过期）；把结算变成异常不可以。"""
    backend, path_client, navigate_client, arm = _armed_backend(release_raises=True)
    handle = _drive_to_navigating(backend, path_client, navigate_client)
    navigate_client.queue(ActionPoll(status=ACTION_ABORTED))
    result = backend.get_navigation_status(handle)
    assert result.status == NavigationStatus.FAILED
    assert arm.log == ["arm", "disarm"]


def test_send_failure_releases_through_the_runners_stop_path() -> None:
    """发送那一刻抛出来的情形。

    异常会一路冒到 ``run_go2w`` 的 ``except``（那条链上没有任何 try 吞掉它：
    ``_advance_speed_limit`` → ``get_navigation_status`` → 记录层 →
    ``autonomous_explorer._wait_result``，都没有 try），那里调 ``backend.stop()``。
    这里就复现后半段：抛出来之后再 stop，授权必须被收回。
    """
    backend, path_client, navigate_client, arm = _armed_backend()

    def explode(request: object) -> None:
        raise RuntimeError("send_goal_async failed")

    navigate_client.send = explode
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    handle = backend.execute_goal(_goal())
    with pytest.raises(RuntimeError):
        backend.get_navigation_status(handle)
    assert arm.log == ["arm"], "武装已经断言上了，但还没有人收回"
    backend.stop()
    assert arm.log == ["arm", "disarm"]


def test_near_in_place_goal_checks_translation_before_sending_navigation():
    grid = _paint(_free_grid(), .511, .539, .011, .039, 100)
    backend, path_client, navigate_client = _backend(grid=grid, pose=_pose(0, 0, 0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_micro_path(dx=.09)))
    result = backend.get_navigation_status(backend.execute_goal(_micro_goal(yaw_deg=30, dx=.09)))
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance['reject_reason'] == REJECT_TOO_NARROW
    assert navigate_client.sent == []


@pytest.mark.parametrize('yaw_deg',[3.,9.9,-9.9])
def test_small_in_place_goal_still_checks_its_entire_footprint(yaw_deg):
    # Side rays at x=0 miss this front corner cell; only a swept footprint
    # (including the initial footprint) catches it.
    grid=_paint(_free_grid(), .32,.37,.21,.26,100)
    backend,path_client,navigate_client=_backend(grid=grid,pose=_pose(0,0,0))
    path_client.queue(ActionPoll(status=ACTION_SUCCEEDED,path=_micro_path()))
    result=backend.get_navigation_status(backend.execute_goal(_micro_goal(yaw_deg=yaw_deg)))
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance['reject_reason'] == REJECT_TOO_NARROW
    assert navigate_client.sent==[]


@pytest.mark.parametrize('cell_value', [100, -1])
@pytest.mark.parametrize('dry_run', [False, True])
def test_translation_checks_current_padding_ring_before_speed_or_arm(cell_value, dry_run):
    # Forward lateral rays all pass: this rear-side cell lies outside the
    # physical body but inside its required 4 cm padding ring.
    grid = _paint(_free_grid(), -.30, -.25, .225, .245, cell_value)
    path = _straight_path()
    assert classify_path_clearance(measure_path_clearance(path, grid)).passable
    events = []
    backend, planner, navigator = _backend(
        grid=grid, dry_run=dry_run,
        speed_setter=lambda speed: events.append(('speed', speed)) or True,
        motion_authorizer=lambda: events.append(('arm', True)) or True,
    )
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    result = backend.get_navigation_status(backend.execute_goal(_goal()))
    assert result.status == NavigationStatus.REJECTED
    assert 'current robot footprint' in result.message
    decision = result.provenance['candidate_decision']['initial_footprint']
    assert decision['accepted'] is False
    assert cell_value in {cell['cell_value'] for cell in decision['blocking_cells']}
    assert events == []
    assert navigator.sent == []


def test_collision_diagnostic_can_only_egress_known_rear_padding_in_dry_run():
    # y=.27 lies outside body+padding (.255 m), but its 5 cm cell intersects
    # that boundary.  Only the diagnostic call may absorb this one-cell centre
    # quantization while moving monotonically away from the rear corner.
    # The 0.60 m wall centre leaves about 0.345 m of body+padding clearance:
    # below ordinary search's fixed 0.40 m, but above this one diagnostic's
    # explicit 0.30 m floor.
    grid = _paint(_corridor_grid(.60), -.40, -.39, .265, .275, 100)
    budget = SimpleNamespace(
        collision_diagnostic=True,
        forward_only=True,
        bounded_search=False,
        MAX_SPEED_MPS=.12,
        token=None,
        finish=lambda: None,
    )
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    backend._trial_budget = budget
    path = ((-.02, .005), (.02, .003), (.12, .002), (.24, 0.))
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    result = backend.get_navigation_status(
        backend.execute_goal(_goal(position=(.24, 0.)))
    )
    assert result.status == NavigationStatus.CANCELLED
    assert navigator.sent == []
    decision = result.provenance['candidate_decision']
    assert decision['accepted'] is True
    assert decision['initial_footprint']['accepted'] is False
    assert decision['initial_footprint_egress']['allowed'] is True
    assert decision['initial_footprint_egress']['policy'] == (
        'known_rear_corner_padding_forward_egress_only'
    )
    assert .30 <= decision['min_side_gap_m'] < .40
    assert decision['diagnostic_corridor_policy'] == (
        'collision_diagnostic_min_side_gap_0.30m'
    )


def test_collision_diagnostic_still_rejects_side_gap_below_local_floor():
    grid = _paint(_corridor_grid(.55), -.40, -.39, .265, .275, 100)
    budget = SimpleNamespace(
        collision_diagnostic=True,
        forward_only=True,
        bounded_search=False,
        MAX_SPEED_MPS=.12,
        token=None,
        finish=lambda: None,
    )
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    backend._trial_budget = budget
    path = ((-.02, .005), (.02, .003), (.12, .002), (.24, 0.))
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    result = backend.get_navigation_status(
        backend.execute_goal(_goal(position=(.24, 0.)))
    )
    decision = result.provenance['candidate_decision']
    assert result.status == NavigationStatus.REJECTED
    assert decision['initial_footprint_egress']['allowed'] is True
    assert decision['min_side_gap_m'] < .30
    assert 'diagnostic_corridor_policy' not in decision
    assert navigator.sent == []


def test_normal_search_keeps_point_four_metre_corridor_floor():
    grid = _corridor_grid(.60)
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    path = ((0., 0.), (.12, 0.), (.24, 0.))
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    result = backend.get_navigation_status(
        backend.execute_goal(_goal(position=(.24, 0.)))
    )
    decision = result.provenance['candidate_decision']
    assert .30 <= decision['min_side_gap_m'] < .40
    assert result.status == NavigationStatus.REJECTED
    assert 'diagnostic_corridor_policy' not in decision
    assert navigator.sent == []


@pytest.mark.parametrize(('segment_angle_deg', 'accepted'), [(17.0, True), (21.0, False)])
def test_collision_diagnostic_allows_only_bounded_navfn_staircase_angle(
    segment_angle_deg, accepted
):
    """The 20 degree allowance is diagnostic-only and remains a hard cone."""
    grid = _paint(_free_grid(), -.40, -.39, .17, .19, 100)
    budget = SimpleNamespace(
        collision_diagnostic=True,
        forward_only=True,
        bounded_search=False,
        MAX_SPEED_MPS=.12,
        token=None,
        finish=lambda: None,
    )
    angle = math.radians(segment_angle_deg)
    # NavFn staircase: each grid segment is angled, but alternating lateral
    # offsets keep the complete request within the independent 3 cm bound.
    dx = .065
    dy = dx * math.tan(angle)
    endpoint = (.24, 0.)
    path = ((-.02, 0.), (.045, dy), (.11, 0.), (.175, dy), endpoint)
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    backend._trial_budget = budget
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    result = backend.get_navigation_status(
        backend.execute_goal(_goal(position=endpoint))
    )
    decision = result.provenance['candidate_decision']
    assert decision['initial_footprint_egress']['allowed'] is accepted
    assert result.status == (
        NavigationStatus.CANCELLED if accepted else NavigationStatus.REJECTED
    )
    assert navigator.sent == []


def test_normal_search_cannot_use_rear_padding_egress():
    grid = _paint(_free_grid(), -.40, -.39, .17, .19, 100)
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    planner.queue(ActionPoll(
        status=ACTION_SUCCEEDED,
        path=((-.02, .005), (.02, .003), (.12, .002), (.24, 0.)),
    ))
    result = backend.get_navigation_status(
        backend.execute_goal(_goal(position=(.24, 0.)))
    )
    assert result.status == NavigationStatus.REJECTED
    assert 'initial_footprint_egress' not in result.provenance['candidate_decision']
    assert navigator.sent == []


def test_bounded_frontier_can_leave_known_rear_padding_in_dry_run():
    grid = _paint(_free_grid(), -.40, -.39, .17, .19, 100)
    budget = SimpleNamespace(
        collision_diagnostic=False,
        forward_only=True,
        bounded_search=True,
        MAX_SPEED_MPS=.12,
        token=None,
        finish=lambda: None,
    )
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    backend._trial_budget = budget
    path = ((-.02, .005), (.02, .003), (.12, .002), (.24, 0.))
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    goal = _goal(position=(.24, 0.))
    goal.provenance = {'intent': {'intent_type': 'EXPLORE_FRONTIER'}}
    result = backend.get_navigation_status(backend.execute_goal(goal))
    assert result.status == NavigationStatus.CANCELLED
    assert navigator.sent == []
    decision = result.provenance['candidate_decision']
    assert decision['accepted'] is True
    assert decision['initial_footprint']['accepted'] is False
    assert decision['initial_footprint_egress']['allowed'] is True
    assert decision['initial_footprint_egress']['policy'] == (
        'known_rear_corner_padding_forward_egress_only'
    )


def test_field_frontier_can_leave_unknown_rear_contact_shell_in_dry_run():
    grid = _paint(_free_grid(), -.40, -.39, .24, .25, -1)
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    # Presence of the independent per-goal guard selects production field
    # scope; dry-run keeps the unit test actuator-free.
    backend._travel_budget = SimpleNamespace(snapshot=lambda: {}, request=None)
    backend._max_path_length_m = 3.0
    path = tuple((i * .05, i * .007) for i in range(11))
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=path))
    goal = _goal(position=path[-1])
    goal.provenance = {'intent': {'intent_type': 'EXPLORE_FRONTIER'}}
    result = backend.get_navigation_status(backend.execute_goal(goal))
    assert result.status == NavigationStatus.CANCELLED
    assert navigator.sent == []
    decision = result.provenance['candidate_decision']
    assert decision['accepted'] is True
    assert decision['initial_footprint_egress']['allowed'] is True
    assert decision['initial_footprint_egress']['policy'] == (
        'rear_contact_shell_forward_egress'
    )


@pytest.mark.parametrize('cell_value', [100, -1])
def test_translation_reconciles_stale_cells_strictly_under_current_body(cell_value):
    # Keep the painted cell centre strictly inside the physical rectangle;
    # neighbouring padding-ring cells must remain untouched.
    grid = _paint(_free_grid(), -.30, -.29, .15, .16, cell_value)
    backend, planner, navigator = _backend(grid=grid, dry_run=True)
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=_straight_path()))
    result = backend.get_navigation_status(backend.execute_goal(_goal()))
    assert result.status == NavigationStatus.CANCELLED
    assert navigator.sent == []
    decision = result.provenance['candidate_decision']
    assert decision['accepted'] is True
    overlay = decision['current_footprint_overlay']
    assert overlay['policy'] == 'current_physical_body_only'
    assert overlay['footprint_padding_masked'] is False
    assert overlay['masked_cells'] > 0
    expected_key = ('masked_occupied_cells' if cell_value == 100
                    else 'masked_unknown_cells')
    assert overlay[expected_key] > 0


def test_short_trial_rejects_navfn_path_that_ends_near_start_instead_of_goal():
    backend, planner, navigator = _backend(
        grid=_free_grid(), dry_run=True, goal_xy_tolerance_m=.03
    )
    planner.queue(ActionPoll(status=ACTION_SUCCEEDED, path=((0., 0.), (.02, 0.))))
    result = backend.get_navigation_status(
        backend.execute_goal(_goal(position=(.12, 0.)))
    )
    assert result.status == NavigationStatus.REJECTED
    assert result.provenance['reject_reason'] == REJECT_NO_PATH
    assert result.provenance['candidate_decision']['plan_endpoint_error_m'] == pytest.approx(.1)
    assert navigator.sent == []
