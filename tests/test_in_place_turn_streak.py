"""Fix #3：「原地打转」必须能被判出来，而合法观察不许被误伤。

背景（2026-10-14 离线核查）：``StuckDetectorInput.repeated_heading_count``
（阈值 ``repeated_heading_limit = 3``）原本喂的是
``graph.sector_visited_count(observation.heading_sector)``。那个计数在**一个
Place 内部**只会返回 0 或 1 —— 它数的是"这个扇区见过没"，不是"连着转了几次"。
拿一个上界为 1 的量去比 3，那个分支在数学上不可达：卡死判据里的 NO_PROGRESS
永远不会触发，机器人可以在原地转到预算耗尽。

修法是把计数换成真正数"连续原地转向且没有换到任何新信息"的轮数。这里钉的是
**判据的分辨力**：四个臂之间只差一个变量，正控必须触发、三个负控必须沉默。
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from app.live_robot.autonomous_explorer import (
    _MIN_COMMANDED_TRANSLATION_M,
    AutonomousExplorer,
    _commanded_translation_m,
    _commanded_turn_deg,
)
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
    scenario_no_target,
)
from app.navigation.backend_factory import MockBackend, MockMetricBackend
from app.navigation.exploration_config import ExplorationBudget, load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.models import ExplorationGoal
from app.navigation.robot_backend import (
    NavigationResult,
    NavigationStatus,
    PoseQuality,
    RobotPose,
)
from app.navigation.safety_models import (
    SECTOR_COUNT,
    SECTOR_WIDTH_DEG,
    STATE_CLEAR,
    build_local_safety_snapshot,
    evaluate_perception_quality,
    wrap_deg,
)

# 这个预算有个**结构下界**，别再往下调：``_update_memory`` 里
# ``new_sector = sector_visited_count(heading_sector) == 0``，而
# ``SECTOR_COUNT = 24``（每扇区 15°）、原地转向步长 ``fallback_turn_deg = 30.0``
# = 每轮跨 2 个扇区 ⇒ **前 12 轮每一轮都落在一个全新扇区**，
# ``new_sector=True`` 会把打转计数每轮清零。所以预算 ≤12 时正控与负控都恒为 0，
# 这个夹具在那时**测不出任何东西**（我第一次把预算从 30 调到 12 就是这么红的）。
# 实测：20 时正控 streak=11 / 卡死 3 次，24 时 17 / 5。取 20 是「够到稳态」与
# 「跑得动」的折中，离阈值 3 有 8 轮余量。
CYCLE_BUDGET = 20


def _snapshot():
    evidence = [
        {"bearing_deg": wrap_deg(index * SECTOR_WIDTH_DEG), "clearance_m": 2.5,
         "state": STATE_CLEAR}
        for index in range(SECTOR_COUNT)
    ]
    return build_local_safety_snapshot(
        sector_evidence=evidence, lidar_fresh=True, geometry_validated=True,
    )


def _perception():
    return evaluate_perception_quality(
        depth_valid_ratio=0.9, near_field_ratio=0.1, visual_feature_count=250,
        stable_object_count=5, estimated_information_gain=0.6, front_clearance_m=2.5,
    )


def _constant_scene() -> MockObservationScene:
    """每轮都是同一批物体、同一个位姿 ⇒ 永远换不到新信息。"""
    return scenario_no_target(empty_scenes=1)


def _learning_scene() -> MockObservationScene:
    """每轮一个新 label ⇒ 每轮都有新信息（合法的原地观察）。"""
    return MockObservationScene(
        scenes=[
            MockSceneStep(objects=[f"obj_{index:02d}"], bundle_id=f"o{index:03d}")
            for index in range(CYCLE_BUDGET + 4)
        ]
    )


# 「先打转、后学到东西」那一臂的两段长度。常量段必须长过 12 轮那个「绕场一周」
# 的结构下界（见 CYCLE_BUDGET 的注释），否则计数根本涨不起来，"被打断"就成了
# 一句空话；学习段要够长，才能证明它**保持**在 0 而不是碰巧被清零一次。
_SPIN_CONSTANT_CYCLES = 16
_SPIN_THEN_LEARN_BUDGET = 24


def _spin_then_learn_scene() -> MockObservationScene:
    """先原地转 16 轮换不到新信息，然后每轮一个新 label。"""
    steps = [
        MockSceneStep(objects=["obj_same"], bundle_id="same")
        for _ in range(_SPIN_CONSTANT_CYCLES)
    ]
    steps += [
        MockSceneStep(objects=[f"obj_{index:02d}"], bundle_id=f"o{index:03d}")
        for index in range(_SPIN_THEN_LEARN_BUDGET)
    ]
    return MockObservationScene(scenes=steps)


def _explorer(backend, *, scene, safety: bool = True,
              budget: int = CYCLE_BUDGET,
              no_information_cycles: int = 3) -> AutonomousExplorer:
    # ``no_information_cycles`` 默认 3 是**夹具**的压缩，不是生产值：生产默认是
    # 20（``configs/exploration/default.yaml:18``）。把它调回 20 会把「卡死分类
    # 只挂在这一个闸门后面」这件事整个藏起来 —— 下面专门有一条臂用生产值跑。
    policy = load_exploration_policy()
    policy.budget = ExplorationBudget(
        max_motion_steps=budget,
        max_planning_cycles=budget,
        max_consecutive_no_information_cycles=no_information_cycles,
    )
    return AutonomousExplorer(
        target="t", observer=scene.observer(), matcher=scene.matcher(),
        verifier=scene.verifier(), backend=backend,
        graph=ExplorationGraph(session_id="t"), policy=policy,
        local_safety_provider=(lambda obs: _snapshot()) if safety else None,
        perception_quality_provider=lambda obs, gain: _perception(),
    )


def _events(explorer: AutonomousExplorer) -> dict[str, int]:
    counts: dict[str, int] = {}
    for event in explorer.events:
        counts[event["event"]] = counts.get(event["event"], 0) + 1
    return counts


def _command_split(explorer: AutonomousExplorer) -> tuple[int, int]:
    """(原地转向命令数, 平移命令数) —— 用来证明"两种臂真的在发不同的命令"。

    **读的是 ``_command_history``，而它被裁到 ``history_cycles + 1``（这里 = 4）
    条** —— 所以这两个数是"最后四条命令"的分布，不是整场的统计。别把它当成
    "这一臂一次都没转过"：预算 12 时 safety=False 那一臂的最后四条里就有 3 条
    转向，一直到 20 才被平移命令挤出窗口。窗口语义同时也是判据本身的语义
    （``_last_command_was_in_place_turn`` 只看最后一条）。
    """
    turns = sum(
        1 for command in explorer._command_history
        if abs(command["dyaw_deg"]) > 0.0 and command["distance_m"] == 0.0
    )
    translations = sum(
        1 for command in explorer._command_history if command["distance_m"] > 0.0
    )
    return turns, translations


# --------------------------------------------------------------------------
# 判据的分辨力：四个臂只差一个变量
# --------------------------------------------------------------------------

def test_in_place_spin_without_new_information_is_detected() -> None:
    """正控：连着原地转向、一点新信息都换不到 ⇒ 计数要涨、卡死要报。

    先断言这一轮真的在原地转（不然"检出"可能只是别的原因），再断言闸门开了。

    这一臂同时钉住了**第一个**结构缺陷（``_last_execution_moved`` 原来是
    ``bool(result.succeeded)``）：``MockBackend`` 对转向目标**不带**
    ``displacement_m`` 遥测（``_apply_mock_motion`` 只回 ``yaw_delta_deg``），
    所以"状态是 SUCCEEDED、观测里没有任何位移"这件事天然就在这一臂里，不需要
    再写一个"报成功但没动"的假后端。把 ``_did_translate`` 换回
    ``bool(result.succeeded)`` 时，下面 ``no_information_classified > 0`` 会红。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    explorer.run()

    turns, translations = _command_split(explorer)
    assert turns > 0, "前提不成立：这一轮没有发出原地转向命令"
    assert explorer._in_place_turn_streak >= 3, (
        f"连续原地打转没有被计数: streak={explorer._in_place_turn_streak}"
    )
    counts = _events(explorer)
    assert counts.get("no_information_classified", 0) > 0
    assert counts.get("stuck_detected", 0) > 0


def test_spin_counter_is_not_the_sector_visit_count() -> None:
    """把喂给卡死判据的那个数钉下来 —— 它必须等于新计数，而不是扇区访问数。

    这一段是"上界为 1 的量去比阈值 3"那个数学上不可达的 bug 的回归钉：
    只要有人把 ``repeated_heading_count`` 换回 ``sector_visited_count``，
    下面的断言立刻红。
    """
    from app.live_robot import autonomous_explorer as module

    explorer = _explorer(MockBackend(), scene=_constant_scene())
    explorer.run()

    sector_counts = {
        explorer.graph.sector_visited_count(sector)
        for sector in range(SECTOR_COUNT)
    }
    assert sector_counts <= {0, 1}, (
        "前置事实变了：sector_visited_count 不再是个 0/1 量 —— "
        "当初判定它数学上不可达的前提需要重做"
    )

    # 直接截住喂给 ``detect_stuck`` 的那个字段，不依赖 ``_stuck_state`` 的其余入参。
    captured: dict[str, int] = {}
    real_detect = module.detect_stuck

    def spy(payload, **kwargs):
        kwargs.pop('policy', None)
        captured["repeated_heading_count"] = payload.repeated_heading_count
        return real_detect(payload, **kwargs)

    module.detect_stuck = spy
    try:
        # ``observation`` 传个带 ``heading_sector`` 的替身而不是 None：这样万一
        # 有人把这一行换回 ``graph.sector_visited_count(observation.heading_sector)``，
        # 注入版会**算出个错的值**（0 或 1）被下面的断言抓住，而不是靠
        # ``None.heading_sector`` 崩掉——崩掉只证明"测试注意到了改动"，
        # 证明不了"断言有分辨力"。
        explorer._stuck_state(
            observation=SimpleNamespace(heading_sector=0), pose=None, safety=None,
            perception=None, navigation_failures=0,
        )
    finally:
        module.detect_stuck = real_detect

    assert captured["repeated_heading_count"] == explorer._in_place_turn_streak
    assert captured["repeated_heading_count"] > 1, (
        "喂进去的还是那个上界为 1 的扇区计数"
    )


def test_learning_while_rotating_is_not_a_stuck_symptom() -> None:
    """负控（配对）：配置、后端、命令序列全都一样，只有观测流不同。

    每轮都换到一个新 label ⇒ 即便一直在原地转，也不是"打转"。
    这一臂是"判据有分辨力"的核心证据：把 --spatial-v2 下的正常原地观察
    误报成卡死，代价比漏报更大。
    """
    explorer = _explorer(MockBackend(), scene=_learning_scene())
    explorer.run()

    turns, _ = _command_split(explorer)
    assert turns > 0, "前提不成立：这一臂没有发出原地转向命令，配对失效"
    assert explorer._in_place_turn_streak == 0
    counts = _events(explorer)
    assert counts.get("no_information_classified", 0) == 0
    assert counts.get("stuck_detected", 0) == 0


def test_new_information_breaks_a_running_spin_streak() -> None:
    """负控（时序）：计数必须先涨起来，**再**被新信息打断。

    这一条是补一个漏网注入的：把 ``else: self._in_place_turn_streak = 0`` 整段
    删掉（"一旦转过就永远算打转"）时，上面那条"每轮都有新信息"的负控**抓不住**
    —— 因为在那条臂上 ``if`` 的条件恒为假，计数从头到尾就是 0，删不删那个 else
    都一样。要证明"能被打断"，就得先让它涨起来。

    所以这一臂分两段：前 16 轮恒定场景（计数涨到 ≥3，用 ``detect_stuck`` 的
    入参序列确证真的涨过），之后每轮一个新 label（计数必须掉回 0）。
    """
    from app.live_robot import autonomous_explorer as module

    explorer = _explorer(MockBackend(), scene=_spin_then_learn_scene(),
                         budget=_SPIN_THEN_LEARN_BUDGET)
    seen: list[int] = []
    real_detect = module.detect_stuck

    def spy(payload, **kwargs):
        kwargs.pop("policy", None)
        seen.append(int(payload.repeated_heading_count))
        return real_detect(payload, **kwargs)

    module.detect_stuck = spy
    try:
        explorer.run()
    finally:
        module.detect_stuck = real_detect

    assert seen and max(seen) >= 3, (
        f"前提不成立：计数没涨起来，'被打断'就无从谈起 seen={seen}"
    )
    assert explorer._in_place_turn_streak == 0, (
        f"新信息没能打断打转计数，结束时还留着 "
        f"{explorer._in_place_turn_streak}（seen={seen}）"
    )


def test_translating_commands_never_count_as_in_place_turns() -> None:
    """负控：命令本身含平移时，即便一直没换到新信息也不许记成"打转"。

    这两件事要的脱困动作不一样（一个是换方向，一个是退出死角），
    混在一起会让恢复策略选错。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene(), safety=False)
    explorer.run()

    turns, translations = _command_split(explorer)
    assert translations > 0 and turns == 0, (
        "前提不成立：这一臂的最后几条命令里还有转向，稳态没到，"
        f"turns={turns} translations={translations}"
    )
    assert not explorer._last_command_was_in_place_turn()
    assert explorer._in_place_turn_streak == 0


def test_dry_run_never_self_reports_as_stuck() -> None:
    """负控：干跑一条运动指令都没发，不许把链路自检报成卡死。

    干跑会把 ``_in_place_turn_streak`` 数上去（命令按构造是转向、又换不到
    新信息），所以这里钉的不是计数而是**闸门**：``_no_information_cycles``
    必须保持 0，卡死事件一个都不许有。

    **两个闸门在干跑里都必须关着**：第一个闸门靠 ``_last_execution_moved``
    恒真（干跑把每条命令都算成"动了"）才保持 0，第二个闸门（连跳）得自己
    豁免干跑 —— 否则一个合成观测流会让链路自检冒出 ``stuck_detected`` 与
    脱困动作。下面先断言"跳数确实涨上去了"，这样"闸门关着"才是个有内容的
    结论，而不是"这条臂压根没转"。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    explorer.dry_run = True
    explorer.run()

    assert explorer._in_place_turn_streak >= 3, (
        f"前提不成立：干跑没把连跳数上去"
        f"（streak={explorer._in_place_turn_streak}）"
    )
    assert explorer._no_information_cycles == 0
    counts = _events(explorer)
    assert counts.get("no_information_classified", 0) == 0
    assert counts.get("stuck_detected", 0) == 0
    assert counts.get("search_exhausted", 0) == 0


# --------------------------------------------------------------------------
# "动没动"的定义：一张判据表，逐行给定输入与期望
# --------------------------------------------------------------------------

def _goal(*, dx: float = 0.0, dy: float = 0.0, dyaw: float = 0.0) -> ExplorationGoal:
    return ExplorationGoal(
        goal_id="g", goal_type="NAVIGATE_POSE",
        relative_dx=dx, relative_dy=dy, relative_dyaw=dyaw,
    )


def _result(*, status: NavigationStatus, displacement=None, yaw_delta=0.0):
    observed = {"yaw_delta_deg": yaw_delta}
    if displacement is not None:
        observed["displacement_m"] = displacement
    return NavigationResult(goal_id="g", status=status, observed_motion=observed)


# 相对后端（``MockBackend``）的判据不看位姿，但签名要求给一个。给真值而不是
# ``None``：``_commanded_translation_m`` 在 ``target is None`` 与 ``pose is None``
# 时都返回 0.0，用 ``None`` 会把"读不出位姿"和"命令本来就不平移"混成一件事。
_POSE = RobotPose(x=0.0, y=0.0, yaw=0.0, frame_id="odom",
                  quality=PoseQuality.RELATIVE, source="mock")


@pytest.mark.parametrize(
    "label,goal,result,dry_run,expected",
    [
        # 0. 失败 ⇒ 没挪窝。**这一行是回归钉**：mock 后端在失败结果里照样带上
        #    它"本来打算走"的位移，把那个当观测值会把"每次都失败"读成"一直在动"。
        ("失败但带位移遥测", _goal(dx=0.2),
         _result(status=NavigationStatus.FAILED, displacement=0.2), False, False),
        # 1. 有位移遥测 ⇒ 以它为准
        ("遥测说动了", _goal(dyaw=30.0),
         _result(status=NavigationStatus.SUCCEEDED, displacement=0.2), False, True),
        ("遥测说没动", _goal(dx=0.2),
         _result(status=NavigationStatus.SUCCEEDED, displacement=0.0), False, False),
        # 2. 没有位移遥测 ⇒ 看命令本身要不要平移；干跑一律算"没有证据"
        ("干跑、无遥测", _goal(dyaw=30.0),
         _result(status=NavigationStatus.SUCCEEDED), True, True),
        ("平移命令、无遥测", _goal(dx=0.3),
         _result(status=NavigationStatus.SUCCEEDED), False, True),
        # 3. 原地转向命令、无遥测 ⇒ 平移量恒为 0 是构造事实
        ("转向命令、无遥测", _goal(dyaw=30.0),
         _result(status=NavigationStatus.SUCCEEDED), False, False),
    ],
)
def test_did_translate_decision_table(label, goal, result, dry_run, expected) -> None:
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    explorer.dry_run = dry_run
    assert explorer._did_translate(goal, result, _POSE) is expected, label


def test_did_translate_discriminates_on_the_same_command() -> None:
    """同一形状的输入、只有位移遥测不同 ⇒ 结论必须不同。

    这一条是那张表的"分辨力"证明：如果 ``_did_translate`` 恒返回同一个值，
    逐行断言也照样会红，但看不出它是"读懂了遥测"还是"碰巧"。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    goal = _goal(dx=0.2)
    moved = _result(status=NavigationStatus.SUCCEEDED, displacement=0.2)
    frozen = _result(status=NavigationStatus.SUCCEEDED, displacement=0.0)
    assert explorer._did_translate(goal, moved, _POSE) != explorer._did_translate(
        goal, frozen, _POSE)


def test_last_command_was_in_place_turn_reads_the_command_window() -> None:
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    assert explorer._last_command_was_in_place_turn() is False

    explorer._command_history.append({"goal_type": "NAVIGATE_POSE",
                                      "distance_m": 0.0, "dyaw_deg": 30.0})
    assert explorer._last_command_was_in_place_turn() is True

    explorer._command_history.append({"goal_type": "NAVIGATE_POSE",
                                      "distance_m": 0.0, "dyaw_deg": 0.0})
    assert explorer._last_command_was_in_place_turn() is False, \
        "既不转也不走的一拍不算'原地打转'"

    explorer._command_history.append({"goal_type": "NAVIGATE_POSE",
                                      "distance_m": 0.3, "dyaw_deg": 15.0})
    assert explorer._last_command_was_in_place_turn() is False, \
        "边走边转不是原地打转"


# --------------------------------------------------------------------------
# 地图系后端（``--backend nav2``，**默认**）那一臂
#
# 上面四个臂用的 ``MockBackend`` 是**相对**后端
# （``supports_metric_navigation=False``），命令里带着 ``relative_dyaw`` /
# ``relative_dx``。而默认后端 nav2 是地图系：``local_goal_executor._metric_goal``
# 每条命令都写成绝对 ``NAVIGATE_POSE``（``relative_dx``/``relative_dy``/
# ``relative_dyaw`` **全是 None**），``_observed_motion`` 里也**没有位移键**。
#
# **这层形状差异上面四臂一次都没覆盖到**，代价是两处默认路径上的失灵：
# 「打转判据」在 nav2 上恒为 False（Fix #3 变成空操作），而「动没动」恒判
# 「没动」（比改之前的 ``bool(result.succeeded)`` 还差）。夹具用相对臂做注入
# 校准时两边全绿 —— 校准是有效的，校准在了**错的那一臂**上。所以下面这几条不是
# 补覆盖率，是补**判据所在的那一臂**：只认 ``relative_*`` 的实现在这里必须全红。
# --------------------------------------------------------------------------

def _metric_pose(x: float, y: float, yaw_deg: float) -> RobotPose:
    """``RobotPose.yaw`` 是**弧度**（explorer 内部一律 ``* 180/pi`` 换算）。"""
    return RobotPose(x=x, y=y, yaw=math.radians(yaw_deg), frame_id="map",
                     quality=PoseQuality.METRIC, source="nav2")


def _metric_goal(*, x: float, y: float, yaw_deg: float) -> ExplorationGoal:
    """照 ``local_goal_executor._metric_goal`` 的样子构造：**只有**绝对位姿。

    刻意不写 ``relative_*`` —— 生产里这三个字段在默认后端上是 None，
    夹具若顺手填一个，测的就不是默认路径了。
    """
    return ExplorationGoal(goal_id="g", goal_type="NAVIGATE_POSE",
                           position=(x, y), yaw=yaw_deg, frame="map")


def _nav2_result(status: NavigationStatus) -> NavigationResult:
    """nav2 的 ``observed_motion`` 原样照抄：**没有** ``displacement_m``。"""
    return NavigationResult(goal_id="g", status=status, observed_motion={
        "path_length_m": 3.0,
        "speed_limit_mps": 0.5,
        "goal_x": 3.0,
        "goal_y": 0.0,
        "goal_yaw_deg": 0.0,
        "status": status.value,
    })


def test_metric_in_place_turn_survives_a_pose_residual() -> None:
    """地图系后端的一次原地转向必须被认成"原地"。

    目标是**上一拍**的坐标（1.0, 1.0），而执行时又取了一次位姿，两次采样不会
    逐位相等 ⇒ 位移是 1e-7 量级的残差、不是精确的 0.0。原来那句
    ``distance_m == 0.0`` 会把它读成"平移过了"，于是默认后端上
    ``_last_command_was_in_place_turn`` 恒为 False、``_in_place_turn_streak``
    一次都涨不起来 —— Fix #3 在默认路径上是个空操作。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    pose = _metric_pose(1.0, 1.0 + 1e-7, 0.0)
    goal = _metric_goal(x=1.0, y=1.0, yaw_deg=30.0)

    assert _commanded_translation_m(goal, pose) < _MIN_COMMANDED_TRANSLATION_M, \
        "前提不成立：这条命令本来就该被判成不平移"
    explorer._record_execution_history(goal, pose, _nav2_result(NavigationStatus.SUCCEEDED))
    assert explorer._last_command_was_in_place_turn() is True, \
        "地图系后端的原地转向被判成了'平移过'"


def test_metric_navigation_counts_as_moving() -> None:
    """地图系后端成功走完的 3 米导航必须算"动了"。

    **这是回归钉**：改之前这里喂 ``bool(result.succeeded)`` 所以是 True；只认
    ``relative_dx/dy`` 的那一版会把**每一条** nav2 命令都算成"按构造原地转向"
    ⇒ 一次货真价实的导航也返回 False ⇒ ``_no_information_cycles`` 在机器人
    真在移动时照样涨到阈值，然后以 SEARCH_EXHAUSTED 收工。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    pose = _metric_pose(0.0, 0.0, 0.0)
    goal = _metric_goal(x=3.0, y=0.0, yaw_deg=0.0)

    assert explorer._did_translate(
        goal, _nav2_result(NavigationStatus.SUCCEEDED), pose) is True


def test_metric_in_place_turn_is_not_moving() -> None:
    """配对负控：同一后端上的**原地**转向目标必须算"没动"。

    没有这一条，把 ``_goal_commands_translation`` 改成恒 True 也能让上一条变绿
    —— 那是个"永远算动了"的伪修复，会把真正卡死的情形一起放过。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene())
    pose = _metric_pose(2.0, 5.0, 0.0)
    goal = _metric_goal(x=2.0, y=5.0, yaw_deg=30.0)

    assert explorer._did_translate(
        goal, _nav2_result(NavigationStatus.SUCCEEDED), pose) is False


def test_commanded_turn_deg_reads_both_representations_and_wraps() -> None:
    """转角两种表示都要认，且单位换算是**反的**：``goal.yaw`` 是度、
    ``RobotPose.yaw`` 是弧度。绕零点的回绕也要对：目标 350°、当前 10° 是
    **-20°**（转过去 20 度）而不是 +340°（反着绕一大圈）。"""
    assert _commanded_turn_deg(_goal(dyaw=30.0), _POSE) == 30.0
    assert _commanded_turn_deg(
        _metric_goal(x=0.0, y=0.0, yaw_deg=30.0),
        _metric_pose(0.0, 0.0, 0.0)) == pytest.approx(30.0)
    assert _commanded_turn_deg(
        _metric_goal(x=0.0, y=0.0, yaw_deg=350.0),
        _metric_pose(0.0, 0.0, 10.0)) == pytest.approx(-20.0)


# ---------------------------------------------------------------------------
# 位姿取样时机：判据量的是**命令**，不是**落点**
# ---------------------------------------------------------------------------
def test_metric_command_pose_must_be_sampled_before_execution() -> None:
    """用真后端跑一遍**生产序列**，钉住"位姿取在下发之前"这件事。

    ``_commanded_translation_m`` / ``_commanded_turn_deg`` 在地图系后端上都是
    「目标点 − 当前位姿」。生产原来在 ``execute_goal`` **之后**才 ``get_pose()``，
    那个差就成了"最后停在离目标多远"：``MockMetricBackend`` 成功时把位姿精确搬到
    目标点，于是

      * 走完 3 m 的导航 → distance = 0.000 ⇒ ``_did_translate`` 判"没动"；
      * 转完 90° 的原地转向 → dyaw = 0.000 ⇒ ``_last_command_was_in_place_turn``
        判"不是原地转向"。

    **两条判据同时失灵**，Fix #3 在默认路径上整个变成空操作。真机 Nav2 只是因为
    控制器有到位容差才偶然还能动。这条臂两半都钉：落点判不出（负控）、命令位姿
    判得出（正控）。
    """
    backend = MockMetricBackend()
    goal = _metric_goal(x=0.0, y=0.0, yaw_deg=90.0)

    command_pose = backend.get_pose()          # 生产：execute_goal **之前**
    backend.execute_goal(goal)
    arrival_pose = backend.get_pose()          # 落点（mock_metric 精确搬过去）

    # 负控：拿落点当位姿，转角恒 0 ⇒ 判不出这是一次原地转向。
    assert _commanded_turn_deg(goal, arrival_pose) == pytest.approx(0.0)
    # 正控：拿命令位姿，转角就是命令要求的那 90°。
    assert _commanded_turn_deg(goal, command_pose) == pytest.approx(90.0)
    assert _commanded_translation_m(goal, command_pose) <= _MIN_COMMANDED_TRANSLATION_M

    explorer = _explorer(MockBackend(), scene=_constant_scene())
    explorer._record_execution_history(goal, command_pose,
                                       _nav2_result(NavigationStatus.SUCCEEDED))
    assert explorer._last_command_was_in_place_turn() is True

    # 同一条命令、同一份结果，只把位姿换成落点 —— 判据必须翻面。
    other = _explorer(MockBackend(), scene=_constant_scene())
    other._record_execution_history(goal, arrival_pose,
                                    _nav2_result(NavigationStatus.SUCCEEDED))
    assert other._last_command_was_in_place_turn() is False


def test_metric_translation_is_recognized_with_the_production_sequence() -> None:
    """平移方向同一件事：走完 3 m 之后拿落点当位姿，会被判成"没动"。"""
    backend = MockMetricBackend()
    goal = _metric_goal(x=3.0, y=0.0, yaw_deg=0.0)

    command_pose = backend.get_pose()
    backend.execute_goal(goal)
    arrival_pose = backend.get_pose()

    assert _commanded_translation_m(goal, arrival_pose) <= _MIN_COMMANDED_TRANSLATION_M
    assert _commanded_translation_m(goal, command_pose) == pytest.approx(3.0)

    explorer = _explorer(MockBackend(), scene=_constant_scene())
    assert explorer._did_translate(
        goal, _nav2_result(NavigationStatus.SUCCEEDED), command_pose) is True
    assert explorer._did_translate(
        goal, _nav2_result(NavigationStatus.SUCCEEDED), arrival_pose) is False


# ---------------------------------------------------------------------------
# 第二个消费者闸门：卡死分类不能只挂在「连续 N 轮无信息」后面
#
# 连跳计数（阈值 3）算出来了、也喂进了 ``_stuck_state``，但改之前整个卡死分类
# 只挂在前一个闸门后面（``max_consecutive_no_information_cycles``，生产值 20）
# ⇒ 要先连转 3 圈、再一路转到第 20 轮，判据才第一次被问到。真机五条会话
# ``planning_cycles`` 只有 1–3、``no_information_cycles`` 全程 0、
# ``no_information_classified`` 0 次发射 —— Fix #3 在真机上是个**没有消费者**的量。
# ---------------------------------------------------------------------------

# 生产默认值，照抄 ``configs/exploration/default.yaml:18``。
_PRODUCTION_NO_INFO_CYCLES = 20


def test_spin_gate_asks_the_stuck_classifier_before_the_no_information_budget() -> None:
    """连跳到阈值就该问一次卡死分类，不必再等 17 轮。

    这一臂**必须用生产阈值 20**：``_explorer`` 默认的 3 是夹具压缩，恰好让
    「第二个闸门」与「第一个闸门」重合，看起来永远是对的（我第一次就是这么
    写的，注入全绿）。阈值 20 时前 12 轮又都在换新扇区，``_no_information_cycles``
    到不了 20 ⇒ 卡死分类**一次都不会被问到**，除非连跳自己能把闸门推开。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene(),
                         no_information_cycles=_PRODUCTION_NO_INFO_CYCLES)
    explorer.run()

    classified = [e for e in explorer.events if e["event"] == "no_information_classified"]
    assert classified, (
        "连跳涨到阈值了，卡死分类却一次都没被问到 —— 判据还是没有消费者"
    )
    assert min(e["no_information_cycles"] for e in classified) < _PRODUCTION_NO_INFO_CYCLES, (
        "卡死分类仍然只在『连续 20 轮无信息』之后才问："
        f"{[e['no_information_cycles'] for e in classified]}"
    )
    counts = _events(explorer)
    assert counts.get("stuck_detected", 0) > 0
    # 问了就得给出结论：连转不平移 ⇒ NO_PROGRESS ⇒ 走脱困，而不是记一笔了事。
    assert counts.get("recovery_triggered", 0) > 0 and explorer._recovery_count > 0, (
        "卡死分类问了却没进脱困"
    )


def test_a_spin_only_consultation_cannot_end_the_search() -> None:
    """连转几圈**不等于**探索穷尽：只有连跳推开的那次咨询不许收工、不许花重规划预算。

    构造"分类器给出非脱困结论"的最小世界：关掉 ``stuck_detection.enabled``
    （``configs/go2w/search_policy.yaml:66`` 上真实存在的开关，默认 true；
    调卡死误报的人第一个动的就是它）。``detect_stuck`` 于是恒返回 FREE。

    **这条臂走的是 REPLAN 出口，不是穷尽出口**（2026-10-14 实测更正）：这个场景里
    语义图有 OPEN 前沿、``_no_information_replans`` 还是 0 ⇒ 分类给的是
    ``REPLAN``。所以能分辨"出口白名单在位"的是下面那条
    ``_no_information_replans == 0``；穷尽那一支另有
    ``test_a_spin_only_consultation_cannot_declare_exhaustion`` 专门钉。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene(),
                         no_information_cycles=_PRODUCTION_NO_INFO_CYCLES)
    explorer.search_policy.stuck_detection.enabled = False
    explorer.run()

    counts = _events(explorer)
    assert counts.get("no_information_classified", 0) > 0, (
        "前提不成立：连跳没有推开第二个闸门，这条臂测不到出口白名单"
    )
    assert explorer._no_information_cycles < _PRODUCTION_NO_INFO_CYCLES, (
        "前提不成立：这轮是第一个闸门开的，不是连跳开的"
    )
    # 前提断言，顺带把"这条臂测的是哪个出口"写进判据里：分类确实给了 REPLAN。
    # （以前这里写的是一条恒真的 ``search_exhausted == 0``：这条臂根本到不了
    # 穷尽那一支，断言再对也没有分辨力。）
    decisions = {event.get("decision") for event in explorer.events
                 if event["event"] == "no_information_classified"}
    assert decisions == {"REPLAN"}, (
        f"前提不成立：这条臂本该走 REPLAN 出口，实际给了 {decisions}"
    )
    assert explorer._no_information_replans == 0, (
        "连跳推开的那次咨询吃掉了重规划预算（``no_information_replans``）"
    )


def test_a_spin_only_consultation_cannot_declare_exhaustion() -> None:
    """同一个出口白名单的另一半：连跳推开的那次咨询也**不能**直接宣告穷尽。

    与上一条只有一处差别：``_no_information_replans`` 预先置 2（重规划预算已用尽）
    ⇒ ``_classify_no_information`` 退无可退，只能给 ``SEARCH_EXHAUSTED``。
    没有白名单的话，连转 3 圈就会把搜索判成"没得找"提前收工 —— 把"原地打转"
    修成"提前收工"是另一种假绿。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene(),
                         no_information_cycles=_PRODUCTION_NO_INFO_CYCLES)
    explorer.search_policy.stuck_detection.enabled = False
    explorer._no_information_replans = 2
    explorer.run()

    classified = [event for event in explorer.events
                  if event["event"] == "no_information_classified"]
    assert classified, "前提不成立：连跳没有推开第二个闸门，这条臂测不到出口白名单"
    assert {event.get("decision") for event in classified} == {"SEARCH_EXHAUSTED"}, (
        "前提不成立：重规划预算没有用尽，分类没落到穷尽那一支"
    )
    assert explorer._no_information_cycles < _PRODUCTION_NO_INFO_CYCLES, (
        "前提不成立：这轮是第一个闸门开的，不是连跳开的"
    )
    assert _events(explorer).get("search_exhausted", 0) == 0, (
        "原地打转被当成了搜索穷尽：连转几圈就收工"
    )


def _cap_loop(explorer: AutonomousExplorer, loops: int) -> dict[str, int]:
    """把主循环钉在 ``loops`` 轮 —— 循环的**真**边界就是 ``budget.remaining``。

    这一段要数的是"同一段『没进步』里闸门开了几次"，也就是**第 2、3… 次咨询**。
    让它自然跑完是不行的：迟滞失效时它只撞墙钟（生产 600 秒；夹具 20 轮 ≈ 30 秒，
    而 200 轮才够看出问题），迟滞在位时又要跑满 20 个规划周期（每轮要跑一遍
    视角打分，十几秒）。钉住轮数是唯一既不依赖墙钟、又跑得动的读法。
    """
    budget = explorer.policy.budget
    real = budget.remaining
    state = {"n": 0}

    def capped(*, elapsed_sec: float, planning_cycles: int, motion_steps: int) -> bool:
        state["n"] += 1
        if state["n"] > loops:
            return False
        return real(elapsed_sec=elapsed_sec, planning_cycles=planning_cycles,
                    motion_steps=motion_steps)

    budget.remaining = capped  # type: ignore[assignment]
    return state


def test_the_spin_gate_asks_at_most_once_per_stretch() -> None:
    """迟滞：连跳闸门在**同一段"没进步"**里只许问一次 —— 这是循环边界的一部分。

    闸门里那条脱困支路是 ``continue``：既不涨 ``planning_cycles``，也不一定涨
    ``motion_steps``；而连跳条件在"一直原地转"时每轮都成立。所以闸门一旦每轮
    都开，整个循环就只剩 ``max_search_seconds`` 一个边界 —— 生产墙钟预算是
    **600 秒**，夹具实测（未设迟滞）200 轮里问了 186 次、脱困 186 次：真机上就是
    不停地倒车/转向，规划一步没走，最后以 TIMEOUT 收工。

    只有**换到新信息**才复位，不是"命令不再是原地转"就复位：脱困倒车本身会写进
    命令窗口（是平移），跳数会被它清零 —— 按跳数复位的话，"转 3 轮 → 脱困 →
    再转 3 轮"能永远循环下去，等于没修。
    """
    explorer = _explorer(MockBackend(), scene=_constant_scene(),
                         no_information_cycles=_PRODUCTION_NO_INFO_CYCLES)
    # 25 轮：连跳大约在第 15 轮涨到阈值（前 12 轮每轮都落在新扇区），
    # 之后还有 10 轮可以暴露"闸门又开了"。
    _cap_loop(explorer, 25)
    explorer.run()

    classified = [e for e in explorer.events if e["event"] == "no_information_classified"]
    assert explorer._in_place_turn_streak > 3, (
        f"前提不成立：连跳没涨过阈值（streak={explorer._in_place_turn_streak}）"
    )
    assert len(classified) == 1, (
        f"同一段『没进步』里问了 {len(classified)} 次（迟滞失效）"
    )
    assert classified[0].get("consultation") == "spin_gate", (
        "这一次咨询不是连跳推开的，这条臂测不到迟滞"
    )
