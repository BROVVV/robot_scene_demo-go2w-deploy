# Copyright 2026 robot_scene_demo maintainers

"""注入校准：LIO「激光修正停摆」判据必须有分辨力，且必须锁存。

计划书 §13.4 的健康门此前是假绿（``web_server.py:377`` 的 ``slam_map_not_drifting``
恒真）。接线它之前，必须先证明两件事，否则就是拿一个未校准的闸门去拦真机搜索：

1. **有分辨力** —— 真实负样本上零误报，真实正样本上确实触发。
2. **锁存是真的在干活** —— 判据触发后会自己闭嘴（实测触发后几千帧里 run 跌回 0
   且只二次触发 1 次），所以「回落即复位」的写法会重新打开闸门。用例
   ``test_latch_survives_the_run_clearing_that_fools_a_self_clearing_judge``
   就是这条的**注入校准**：拿掉锁存它必须失败。

数据源：``artifacts/slam_validation/lio_divergence_20260915/``（2026-09-15 两次真机
发散 + 三轮控制变量实验的逐帧转录）。该目录**未被 git 跟踪**，所以依赖它的用例一律
``skipif`` 并在理由里写明预期数字 —— 数据丢了测试不能假装通过。不依赖它的用例跑
``tests/fixtures/lio_diag/`` 下两份真实日志节选（各 <30 KB，已入库）。

诚实边界：节选只能证明局部行为（「存在 8 帧连击不误报、10 帧连击报出」）；
「负样本最长 8 帧」这个全局结论依赖全量 artifacts，见 ``test_negative_samples_...``。
另外 ``lio_precursor_watch.py:85-87`` 自称的「负样本 22878 帧零误报」在本机复现不出来，
可复跑的是本文件里的数字。
"""

from __future__ import annotations

import csv
import re
import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.spatial.lio_nofix_judge import (  # noqa: E402
    DEGRADED_LIO_DRIFT,
    HEALTHY,
    LioNofixJudge,
)

ARTIFACTS = PROJECT_ROOT / "artifacts" / "slam_validation" / "lio_divergence_20260915"
FIXTURES = Path(__file__).resolve().parent / "fixtures" / "lio_diag"

_GAP = pytest.mark.skipif(
    not ARTIFACTS.is_dir(),
    reason=(
        "需要未被 git 跟踪的 artifacts/slam_validation/lio_divergence_20260915/"
        "（2026-09-15 真机发散的逐帧转录）：负样本应为 t_neg_drive=7837 帧/最长连击 8、"
        "t_neg_static=2600/0、t_neg_turn=441/0、lio_phase_frames_s2.csv=4690/0；"
        "正样本 t_pos_1240_end 触发@538、t_pos_1320 触发起点@66、frames_1320.csv 触发@12131"
    ),
)

# 真实正/负样本（csv 是离线转录，没有 GO2W-DIAG 字样，测试自己重建成判据吃的那一行）
NEGATIVE_LOGS = (
    "calibration/t_neg_drive.txt",
    "calibration/t_neg_static.txt",
    "calibration/t_neg_turn.txt",
)
NEGATIVE_CSV = ("lio_phase_frames_s2.csv",)
POSITIVE_LOGS = ("calibration/t_pos_1240_end.txt", "calibration/t_pos_1320.txt")
POSITIVE_CSV = ("frames_1320.csv",)

_NUM = r"(-?[\d.]+)"


def _diag_lines(path: Path) -> list[str]:
    return [line for line in path.read_text(errors="replace").splitlines()
            if "GO2W-DIAG" in line]


def _csv_lines(path: Path) -> list[str]:
    """把离线转录的列还原成判据要吃的那一行（列序见 lio_phase_logger.py:15-17）。"""
    out: list[str] = []
    with path.open(errors="replace", newline="") as handle:
        reader = csv.reader(handle)
        next(reader, None)
        for row in reader:
            if len(row) < 18:
                continue
            out.append(
                f"[GO2W-DIAG] iter={row[5]} fix_t={row[10]} v={row[11]} "
                f"act={row[3]} pos=({row[15]},{row[16]},{row[17]})"
            )
    return out


def _sample_lines(name: str) -> list[str]:
    path = ARTIFACTS / name
    return _csv_lines(path) if name.endswith(".csv") else _diag_lines(path)


def _feed(judge: LioNofixJudge, lines: list[str]) -> int | None:
    """喂完全部行，返回**health 首次变成 DEGRADED 的帧下标**（从未触发则 None）。"""
    fired = None
    for index, line in enumerate(lines):
        judge.feed(line)
        if fired is None and judge.health == DEGRADED_LIO_DRIFT:
            fired = index
    return fired


def _max_bad_run(lines: list[str]) -> int:
    """直接从原文复算最长连击 —— 用来把「余量」写成可断言的数字，而不是注释里的说法。"""
    best = run = 0
    for line in lines:
        match_it = re.search(r"iter=" + _NUM, line)
        match_fix = re.search(r"fix_t=" + _NUM, line)
        if not match_it or not match_fix:
            continue
        if float(match_it.group(1)) >= 10 and float(match_fix.group(1)) == 0.0:
            run += 1
            best = max(best, run)
        else:
            run = 0
    return best


def _good_lines(count: int) -> list[str]:
    return [f"[GO2W-DIAG] iter=1 fix_t=0.0003 v=0.02 act=0.96 pos=(0.00,0.00,0.00)"] * count


class _FakeClock:
    """手动推进的时钟。判据只要求它可调用、返回 float。"""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def _make_judge(**kwargs) -> LioNofixJudge:
    """构造判据，**除饿死用例之外一律注入停住的时钟**。

    判据现在有第二条降级路径：距上次成功 ``feed()`` 超过 ``starve_timeout_s``
    即报 DEGRADED_LIO_DRIFT。本文件那些校准用例要喂完上万帧（``frames_1320.csv``
    是 12456 帧）才断言，中间只要出现一次长停顿（GC、全量 pytest 下的调度），
    真实时钟就会跨过阈值，把「零误报」用例假红。注入停住的时钟让时间只在
    第 2.5 节那组用例里前进。
    """
    kwargs.setdefault("clock", lambda: 0.0)
    return LioNofixJudge(**kwargs)


# --------------------------------------------------------------------------- #
# 1. 分辨力
# --------------------------------------------------------------------------- #

def test_committed_fixtures_prove_both_directions() -> None:
    """入库的两份真实日志节选：8 帧连击不误报、10 帧连击报出。"""
    negative = FIXTURES.joinpath("nofix_negative.txt").read_text().splitlines()
    positive = FIXTURES.joinpath("nofix_positive.txt").read_text().splitlines()

    negative_judge = _make_judge()
    _feed(negative_judge, negative)
    assert negative_judge.health == HEALTHY
    # 这份节选是围着源数据里最长的那段连击（8 帧）截的，余量只有 2 帧。
    assert _max_bad_run(negative) == 8
    assert _max_bad_run(negative) < LioNofixJudge.MIN_CONSECUTIVE

    positive_judge = _make_judge()
    assert _feed(positive_judge, positive) is not None
    assert positive_judge.health == DEGRADED_LIO_DRIFT
    assert "fix_t=0" in positive_judge.reason


@_GAP
@pytest.mark.parametrize("name", NEGATIVE_LOGS + NEGATIVE_CSV)
def test_negative_samples_never_trigger(name: str) -> None:
    """真实负样本零误报。t_neg_turn 是「急转时 iter 撞上限」的专门负样本。"""
    judge = _make_judge()
    assert _feed(judge, _sample_lines(name)) is None
    assert judge.health == HEALTHY
    assert judge.reason == ""


@_GAP
def test_negative_margin_is_two_frames() -> None:
    """把「余量只有 2 帧」写成断言 —— 判据或数据漂了，这条会先响。"""
    worst = max(_max_bad_run(_sample_lines(name))
                for name in NEGATIVE_LOGS + NEGATIVE_CSV)
    assert worst == 8
    assert worst < LioNofixJudge.MIN_CONSECUTIVE


@_GAP
@pytest.mark.parametrize(
    ("name", "trigger_index"),
    [("calibration/t_pos_1240_end.txt", 538),
     ("calibration/t_pos_1320.txt", 75),
     ("frames_1320.csv", 12131)],
)
def test_positive_samples_trigger_at_the_calibrated_frame(name: str, trigger_index: int) -> None:
    """真实正样本触发，且落点与校准记录一致（连击起点 538/66/12131 的第 10 帧）。"""
    judge = _make_judge()
    assert _feed(judge, _sample_lines(name)) == trigger_index
    assert judge.health == DEGRADED_LIO_DRIFT


@_GAP
def test_known_blind_spot_the_precursor_does_not_see_the_climb() -> None:
    """已知盲区（诚实性护栏）：12:40 死亡前的爬升期判据**完全无反应**。

    所以这个判据只能表述成「已经发散了就别再开搜索」的事后闸门，
    **不许**写成「防止发散」或早期预警。这条断言是为了防止有人日后
    拿它去当预警，然后以为预警没响就是安全。
    """
    judge = _make_judge()
    assert _feed(judge, _sample_lines("calibration/t_pos_climb.txt")) is None
    assert judge.health == HEALTHY


# --------------------------------------------------------------------------- #
# 2. 锁存（本模块存在的理由）
# --------------------------------------------------------------------------- #

@_GAP
def test_latch_survives_the_run_clearing_that_fools_a_self_clearing_judge() -> None:
    """注入校准：判据触发后会自己闭嘴，锁存必须挡住这个回弹。

    实测（frames_1320.csv，触发起点 @12131）：触发那一波连击只有 33 帧，其后
    256 帧的最长连击掉到 3，且整段再没触发过。所以「回落即复位」的写法会把闸门
    重新打开，制造出比改动前更糟的假绿（现在恒真至少看得出来）。

    这条的注入校准：把 ``LioNofixJudge.health`` 里的 ``self._latched`` 判断去掉，
    本用例必须失败 —— 后半段那 256 帧会让一个未锁存的判据回到 HEALTHY。
    """
    lines = _sample_lines("frames_1320.csv")
    judge = _make_judge()
    assert _feed(judge, lines) == 12131
    assert judge.health == DEGRADED_LIO_DRIFT

    # 触发那一波连击之后的帧：run 已跌回阈值以下。判据闭嘴了，但 LIO 没有恢复。
    tail = lines[12200:]
    assert len(tail) == 256
    assert _max_bad_run(tail) == 3
    assert _max_bad_run(tail) < LioNofixJudge.MIN_CONSECUTIVE
    for line in tail:
        judge.feed(line)
    assert judge.health == DEGRADED_LIO_DRIFT, "锁存被回落的好帧骗开了"

    # 对照：同样这批行，喂给一个全新判据就是 HEALTHY —— 证明尾段确实是「好帧」，
    # 上一条断言的通过只可能来自锁存。
    fresh = _make_judge()
    for line in tail:
        fresh.feed(line)
    assert fresh.health == HEALTHY


def test_latch_clears_only_on_a_session_restart() -> None:
    """复位只有一条路：观察到 LIO 栈重启。"""
    judge = _make_judge()
    bad_line = "[GO2W-DIAG] iter=10 fix_t=0.0000 v=1.20 act=0.90 pos=(0,0,0)"
    _feed(judge, [bad_line] * LioNofixJudge.MIN_CONSECUTIVE)
    assert judge.health == DEGRADED_LIO_DRIFT

    # 不调 note_session_restart()：再喂多少好帧都不许松开。
    _feed(judge, _good_lines(500))
    assert judge.health == DEGRADED_LIO_DRIFT

    judge.note_session_restart()
    assert judge.health is None, "重启瞬间不得直接回到 HEALTHY —— 那是个恒绿窗口"
    _feed(judge, _good_lines(9))
    assert judge.health is None, "少于 MIN_FRAMES_FOR_VERDICT 帧时判据在构造上不可能触发"
    _feed(judge, _good_lines(1))
    assert judge.health == HEALTHY


def test_no_verdict_before_minimum_frames() -> None:
    """帧数不足时不表态（调用方因此不写 mapping_health 字段）。"""
    judge = _make_judge()
    _feed(judge, _good_lines(LioNofixJudge.MIN_FRAMES_FOR_VERDICT - 1))
    assert judge.health is None
    assert judge.reason == ""
    judge.feed(f"[GO2W-DIAG] iter=1 fix_t=0.0003")
    assert judge.health == HEALTHY


def test_malformed_lines_are_not_counted_as_frames() -> None:
    """不含诊断字段的行是空操作 —— 否则运维噪声能把判据推进到表态。"""
    judge = _make_judge()
    _feed(judge, ["ddsi_udp_conn_write ... failed with retcode -1"] * 50)
    assert judge.health is None
    # 空操作之后紧跟最小帧数的好帧，必须正好在此刻表态（证明噪声没在计数）
    _feed(judge, _good_lines(LioNofixJudge.MIN_FRAMES_FOR_VERDICT))
    assert judge.health == HEALTHY


# --------------------------------------------------------------------------- #
# 2.5 饿死（DIAG 行停了 —— 2026-09-16 断电现场暴露的第二个盲区）
# --------------------------------------------------------------------------- #

def test_starvation_reports_degraded_not_stale_green() -> None:
    """断电现场：DIAG 行停了之后，判据必须**不再**保持上一次的 HEALTHY。

    实测（2026-09-16，狗断电 4 分钟后）：快照里仍是 ``mapping_health=HEALTHY``
    + ``map_status=LIVE`` + ``fresh=True``。``fresh`` 由 ``generated_at`` 驱动，
    只证明**写端活着**；判据从没被调用，所以永远停在最后一次表态。

    这条的注入校准：把 ``health`` 里 ``_is_starved()`` 那个分支删掉，本用例必须
    失败 —— 这正是改动前的行为。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)

    _feed(judge, _good_lines(50))
    assert judge.health == HEALTHY

    # DIAG 行不再产生 —— 判据一次都不再被喂。时间照走。
    clock.advance(11.0)
    assert judge.health == DEGRADED_LIO_DRIFT, "判据被饿死时必须降级，不能停在 HEALTHY"

    reason = judge.reason
    assert "已停" in reason
    # 必须说清是哪一种降级：下一个读日志的人不该去查一次根本不存在的发散。
    assert "不是检测到发散" in reason
    assert "自动解除" in reason


def test_starvation_clears_itself_when_the_diag_lines_resume() -> None:
    """饿死可自清（发散 latch 不可以）—— 这是范畴区别，不是程度区别。

    发散 latch 不能自清，因为「判据不再触发」与「LIO 已恢复」同形。饿死自清靠的是
    DIAG 行**重新出现**，那是数据源自己在说话，是 LIO 还活着的直接证据。
    代价上也不该锁死：一次十几秒的卡顿就要重启 SLAM 栈、作废建图会话，
    与故障严重性完全不成比例（那条路径留给真正的发散 latch）。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    _feed(judge, _good_lines(50))
    clock.advance(60.0)
    assert judge.health == DEGRADED_LIO_DRIFT

    judge.feed("[GO2W-DIAG] iter=1 fix_t=0.0003 v=0.02 act=0.96 pos=(0,0,0)")
    assert judge.health == HEALTHY, "DIAG 行回来了就该自解，不需要重启 SLAM 栈"
    assert judge.reason == ""


def test_never_fed_stays_silent_instead_of_reporting_starvation() -> None:
    """从没喂过 ≠ 饿死。日志路径不对 / LIO 根本没起，仍走既有的第三态。

    这是判据 docstring 里那条区分：「饿死是"它跑过、然后停了"」。若把这两件事
    塌成一个，路径写错就会变成一个**看起来很严重**的 DEGRADED，运维会去查一个
    并不存在的故障；而且它会把「拿不到的一律不编」这条原则悄悄废掉。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    clock.advance(3600.0)
    assert judge.health is None, "从没喂过必须是「不表态」，不是 DEGRADED"
    assert judge.reason == ""


def test_the_timeout_is_strictly_greater_than() -> None:
    """阈值是严格大于：恰好等于阈值的那个时刻不报，跨过去才报。

    写成 >= 会让 ``2 × period_s`` 与 poll 网格重合时的抖动直接变成误报。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    _feed(judge, _good_lines(50))

    clock.advance(10.0)
    assert judge.health == HEALTHY
    clock.advance(0.001)
    assert judge.health == DEGRADED_LIO_DRIFT


def test_frames_inside_the_timeout_keep_it_healthy() -> None:
    """喂食之间时间在走，但没跨阈值 —— 不许因为"时间过去了"就降级。"""
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    _feed(judge, _good_lines(LioNofixJudge.MIN_FRAMES_FOR_VERDICT))
    for _ in range(5):
        clock.advance(9.0)
        judge.feed("[GO2W-DIAG] iter=1 fix_t=0.0003 v=0.02 act=0.96 pos=(0,0,0)")
        assert judge.health == HEALTHY


def test_too_few_frames_is_still_silence_even_after_a_long_silence() -> None:
    """帧数不足这一条排在饿死之前 —— 边界钉死，别让两套语义互相污染。

    少于 ``MIN_FRAMES_FOR_VERDICT`` 帧时判据**在构造上不可能**触发发散，
    此时它给不出任何有依据的表态（既有第三态）。饿死也沿用同一条边界：
    要"跑过"，最低门槛就是判据曾经能够表态。这里钉的是**继承**，不是新阈值。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    _feed(judge, _good_lines(LioNofixJudge.MIN_FRAMES_FOR_VERDICT - 1))
    clock.advance(3600.0)
    assert judge.health is None
    assert judge.reason == ""


def test_starvation_does_not_release_a_divergence_latch() -> None:
    """两条降级路径不许互相解锁；同时成立时 reason 必须报**更具体**的那一个。

    分辨力如实写在这里：``health`` 那几条断言在两种降级路径下都是 DEGRADED，
    单独证明不了什么。真正有分辨力的是 **reason 的优先级** —— 注入点是
    ``reason`` 里 ``if self._latched: return self._reason`` 那一行（删掉它，
    本用例与 ``test_committed_fixtures_prove_both_directions`` 必须同时红）。
    下一节那条"锁存不会被回落的好帧骗开"的注入点则在 ``health`` 的 ``_latched``。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    bad = "[GO2W-DIAG] iter=10 fix_t=0.0000 v=1.20 act=0.90 pos=(0,0,0)"
    _feed(judge, [bad] * LioNofixJudge.MIN_CONSECUTIVE)
    assert judge.health == DEGRADED_LIO_DRIFT

    clock.advance(3600.0)
    assert judge.health == DEGRADED_LIO_DRIFT
    # reason 必须仍是**发散**那条（更具体的诊断），不是饿死那条。
    assert "fix_t=0" in judge.reason
    assert "已停" not in judge.reason

    # 饿死自清也不许顺手把 latch 松开。
    judge.feed("[GO2W-DIAG] iter=1 fix_t=0.0003 v=0.02 act=0.96 pos=(0,0,0)")
    assert judge.health == DEGRADED_LIO_DRIFT


def test_session_restart_leaves_no_starvation_window() -> None:
    """重启复位后不许立刻报饿死 —— 那时日志还没被重新喂过。

    分辨力边界如实写在这里：挡住饿死的是 ``_frames`` 被清零（``health`` 的帧数
    判断排在饿死判断之前），**不是**某个单独的 ``_last_feed`` 重置 ——
    后者在本实现里不可观测，删掉没有任何用例会红，所以判据里根本没有那一行。
    这条用例钉的是**复位语义**，注入点应当选 ``note_session_restart`` 里
    ``self._frames = 0`` 那一行。
    """
    clock = _FakeClock()
    judge = _make_judge(starve_timeout_s=10.0, clock=clock)
    _feed(judge, _good_lines(50))
    clock.advance(600.0)
    assert judge.health == DEGRADED_LIO_DRIFT

    judge.note_session_restart()
    assert judge.health is None, "重启后是「还没喂过」，不是「喂过又停了」"
    clock.advance(600.0)
    assert judge.health is None
    _feed(judge, _good_lines(LioNofixJudge.MIN_FRAMES_FOR_VERDICT))
    assert judge.health == HEALTHY


def test_default_timeout_encodes_the_poll_period_calibration() -> None:
    """把 ``2 × period_s`` 这个标定结论钉成数字。

    调用方 ``webui_map_snapshot_writer.py`` 是每 ``period_s`` 秒（默认 5.0 s）
    **批量** poll 一次，这个周期本身就把 ``now - last_feed`` 量化了。取 1 × 周期
    时单轮 timer 抖动就能误报，所以阈值是 2 倍。健康段实测帧间隔 p99 = 151 ms
    （21858 帧 / 36.6 分钟），10 秒是它的 66 倍。
    """
    assert LioNofixJudge.DEFAULT_STARVE_TIMEOUT_S == 10.0
    assert 10.0 == 2.0 * 5.0, "对应 webui_map_snapshot_writer.py 的 period_s 默认值"


def test_the_real_diag_outage_would_be_caught_at_this_threshold() -> None:
    """标定用的那一次真实中断（11.27 s）必须落在这条判据的覆盖范围内。

    ``lio_phase_frames.csv`` 全程唯一一次 >1 s 的帧间隔是 11.27 s，缺口前后的帧
    全是 ``v≈6.87e10`` / ``act=0.000`` / ``iter=1`` —— **LIO 已经跑飞，而发散判据
    （要求 ``iter >= 10``）完全不会响**。这正是饥饿判据要补的那一类。

    依赖未被 git 跟踪的 artifacts，所以同样 skipif；节选进不了库是因为它是
    **时间轴**上的现象，截取片段会把这个用例要钉的"11.27 秒"本身截掉。
    """
    if not ARTIFACTS.is_dir():
        pytest.skip("需要未被 git 跟踪的 artifacts/.../lio_phase_frames.csv")

    walls: list[float] = []
    with (ARTIFACTS / "lio_phase_frames.csv").open(errors="replace", newline="") as handle:
        reader = csv.reader(handle)
        next(reader, None)
        for row in reader:
            if len(row) >= 18:
                walls.append(float(row[0]))

    gaps = [b - a for a, b in zip(walls, walls[1:])]
    worst = max(gaps)
    assert worst > LioNofixJudge.DEFAULT_STARVE_TIMEOUT_S, (
        f"最长帧间隔 {worst:.2f}s 应当大于阈值 {LioNofixJudge.DEFAULT_STARVE_TIMEOUT_S}s"
    )
    # 除那一次之外，健康段的抖动必须离阈值很远 —— 这是"零误报"的余量。
    # 实测健康段（<1 s）最大 451 ms；注意不要拿 p99（151 ms）当最大值，
    # 尾部还有 p99.9=201 ms 和这个 451 ms 的孤立点。
    healthy = sorted(g for g in gaps if g < 1.0)[-1]
    assert healthy < 0.5, f"健康段最大间隔 {healthy:.3f}s 应远小于阈值"
    assert LioNofixJudge.DEFAULT_STARVE_TIMEOUT_S > 20 * healthy


# --------------------------------------------------------------------------- #
# 3. 词表一致性（判据刻意不 import slam_web_map_state，靠这条兜住漂移）
# --------------------------------------------------------------------------- #

def test_vocabulary_matches_the_webui_map_state() -> None:
    from app.spatial import lio_nofix_judge, slam_web_map_state

    assert lio_nofix_judge.HEALTHY == slam_web_map_state.HEALTHY
    assert lio_nofix_judge.DEGRADED_LIO_DRIFT == slam_web_map_state.DEGRADED_LIO_DRIFT
