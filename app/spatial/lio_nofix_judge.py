"""LIO「激光修正停摆」判据 —— 计划书 §13.4 健康门的可观测层接线。

本模块是**纯逻辑，不 import rclpy**。理由与 ``app/spatial/slam_web_map_state.py``
开头那段相同：生产调用方 ``scripts/go2w/webui_map_snapshot_writer.py`` 是个 ROS
节点，而离线测试环境没有 rclpy，所以判据必须住在能被 pytest 直接喂日志原文的地方。

判据本身**不是新发明的**：逐字复刻
``artifacts/slam_validation/lio_divergence_20260915/tools/lio_precursor_watch.py``
里那条**唯一在两次真机发散中都命中**的判据，包括 ``fix == 0.0`` 这个严格浮点
比较 —— 改任何一点都会让那份校准失效。

2026-09-16 在 ``artifacts/slam_validation/lio_divergence_20260915/`` 上逐帧复算：

  负样本（有分辨力的证据）
    calibration/t_neg_drive.txt      7837 帧  max_bad_run = 8   ← 余量只有 2 帧
    calibration/t_neg_static.txt     2600 帧  max_bad_run = 0
    calibration/t_neg_turn.txt        441 帧  max_bad_run = 0   ← 急转不误报
    lio_phase_frames_s2.csv          4690 帧  max_bad_run = 0
  正样本
    calibration/t_pos_1240_end.txt   2605 帧  触发@538
    calibration/t_pos_1320.txt        400 帧  触发@75
    frames_1320.csv                 12456 帧  触发@12131

  已知盲区（**不许**把它说成预警）
    calibration/t_pos_climb.txt      5336 帧  完全不触发 —— 那是 12:40 死亡前的
    爬升期。这个判据覆盖的是终末段，13:20 那次只提前 0.4 秒。

**为什么必须锁存（本模块与那个离线工具最重要的区别）**：触发之后 ``bad_run``
会迅速跌回 0，且其后几千帧再也不触发 —— 而 LIO 已经死了。实测：

    t_pos_1240_end.txt  触发后 2067 帧中 2050 帧 run<10，只二次触发 1 次
    t_pos_1320.txt      触发后  325 帧中  299 帧 run<10，只二次触发 1 次
    lio_phase_frames.csv 触发后 5037 帧中 5011 帧 run<10，只二次触发 1 次

所以照搬离线工具那种「回落即复位」的写法，会在几秒内把闸门重新打开，制造出
比现在更糟的假绿。**复位只有一条路：观察到 LIO 栈被重启**（调用方在 launch
日志被截断时调 ``note_session_restart()``）。``bad_run`` 归零不是恢复的证据。

**2026-09-16 补的第二个盲区：判据被饿死时不表态（stale-green）。**

判据只在**被喂 DIAG 行**时更新状态。DIAG 行不再产生 ⇒ 判据从未被调用 ⇒ 它保持
最后一次表态（``HEALTHY``）**永远不变**。狗断电后实测：地图本体已死 4 分钟，
快照里仍是 ``mapping_health=HEALTHY`` + ``map_status=LIVE`` + ``fresh=True``。

把饿死映射成「不表态」（``health`` 返回 ``None``）**没有用**，因为那条路是**放行**的：
调用方会跳过 ``mapping_health`` / ``lio_pose_valid`` / ``map_status`` 三个键，前端走
兜底 ``map_status || (fresh ? "LIVE" : "STALE")``，而 ``fresh`` 只看 ``generated_at``
（写端心跳）⇒ 结果还是 ``LIVE``。所以饿死映射到 ``DEGRADED_LIO_DRIFT``。

``starve_timeout_s`` 的标定（**不许凭感觉改**）：

  喂食节拍（真实转录 ``lio_phase_frames{,_s2}.csv``，21858 帧 / 36.6 分钟，
  两个 session 都是 10 Hz）：帧间隔 p50 100.0 ms、p90 101.0 ms、p99 151.0 ms、
  p99.9 201.0 ms，**健康段（<1 s）最大 451 ms**。全程唯一一次 >1 s 的间隔是
  11.27 s，发生在 79.1% 处，缺口前后的帧全是 ``v=6.87e10`` / ``act=0.000`` /
  ``iter=1`` —— **LIO 已经跑飞，而发散判据（需要 ``iter>=10``）完全不会响**。
  那正是饥饿判据要补的一类，且 11.27 s 落在本阈值的覆盖范围内。

  调用方 ``webui_map_snapshot_writer.py`` 是**每 ``period_s`` 秒批量 poll 一次**
  （默认 5.0 s），不是逐行流式。所以连续 k 轮 poll 读不到新行时，
  ``now - last_feed`` ∈ (k·period, (k+1)·period]，**这个量被 poll 周期量化**。
  ``starve_timeout_s = 2 · period_s`` 的语义因此是干净的：
  **「至少连续 3 轮 poll 没有新帧」⇒ 至少 10 秒没有 DIAG 行**（实际 ∈ [10, 15) s）。
  对上面的实测它是 451 ms 的 **22 倍**余量；取 1·period 则单轮 poll 抖动就能误报。

**饿死可以自清，发散不可以 —— 这是范畴区别，不是程度区别。** 发散 latch 不能自清，
因为「判据不再触发」与「LIO 已恢复」同形（事实见上）。而饿死自清靠的是
**DIAG 行重新出现**，那是数据源自己在说话，是 LIO 还活着的**直接证据**，不是判据
自己闭嘴。代价对比也支持这样选：锁死的话，一次十几秒的雷达/读端卡顿就要
``reset_plain_slam_session.sh`` 重启 SLAM 栈、**作废当前建图会话**，与故障严重性
完全不成比例；而 LIO 真死时 DIAG 不会回来，饿死态就一直保持，自清掩盖不了它。
自清**不清** ``_frames``：恢复后立刻回 ``HEALTHY``（DIAG 真的回来了），若回来的是坏帧，
``_bad_run`` 会在 10 帧内接住。

诚实边界：本判据检出的是**「调用方读不到新的 DIAG 行」**，不是「LIO 内部死了」。
读端被调度器挂起 10 秒以上也会触发（极罕见）。它**不**断言地图点云在更新 ——
那是另一个数据源，且 ``ros_stamp`` 在机器人静止时也冻（见调用方注释），
拿它当心跳会在每一次静止时误报。
"""

from __future__ import annotations

import re
import time
from collections.abc import Callable

# 词表必须与 app/spatial/slam_web_map_state.py:19-21 一致：读取端
# app/manual_web_demo/web_server.py:377 判 != DEGRADED_LIO_DRIFT，
# 前端 static/slam_map_3d.js:274 判 === HEALTHY 才亮绿灯。
# 这里刻意不 import 那个模块 —— 为了两个字面量把 app 包依赖拉进一个 ROS 节点是
# 负收益。漂移由 tests/test_lio_nofix_gate_calibration.py 的一条断言兜住。
HEALTHY = "HEALTHY"
DEGRADED_LIO_DRIFT = "DEGRADED_LIO_DRIFT"

_NUM = r"(-?[\d.]+)"
_POS = re.compile(r"pos=\((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)")


def _value(line: str, key: str) -> float | None:
    match = re.search(key + r"=" + _NUM, line)
    return float(match.group(1)) if match else None


class LioNofixJudge:
    """吃 ``[GO2W-DIAG]`` 行，回答「LIO 的激光修正是不是停摆了」。"""

    # 单帧出现 iter 撞上限是转弯瞬态（健康数据里最长连续 8 帧），连续 ≥10 帧才报警。
    # 这两个常数是上面那份校准的一部分，不要改。
    MIN_CONSECUTIVE = 10
    # 少于 10 帧时判据**在构造上不可能触发**，此时说 HEALTHY 是逻辑上的假绿，
    # 所以宁可不表态（health 返回 None，调用方不写这个字段）。
    MIN_FRAMES_FOR_VERDICT = 10

    # 饿死阈值。调用方按 ``2 * period_s`` 传入（见模块 docstring 的标定）；
    # 这里的默认值对应 period_s=5.0 s 的默认配置，单测可直接用。
    DEFAULT_STARVE_TIMEOUT_S = 10.0

    def __init__(
        self,
        starve_timeout_s: float = DEFAULT_STARVE_TIMEOUT_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._frames = 0
        self._bad_run = 0
        self._latched = False
        self._reason = ""
        self._starve_timeout_s = float(starve_timeout_s)
        # 注入时钟是为了让校准测试能确定性地喂假时间：喂完上万帧再断言的间隙里，
        # 真实时钟一旦跨过阈值就会让"零误报"用例假红。
        self._clock = clock
        # None = 还没被喂过任何一帧。**必须与"喂过、然后停了"分开**：
        # 日志路径不对 / LIO 根本没起 ⇒ _last_feed 恒 None ⇒ 不表态（第三态）。
        self._last_feed: float | None = None

    def feed(self, line: str) -> None:
        """喂一行 launch log 原文。不含诊断字段的行是空操作，不计帧。"""
        it = _value(line, "iter")
        fix = _value(line, "fix_t")
        if it is None or fix is None:
            return
        self._last_feed = self._clock()
        self._frames += 1
        self._bad_run = self._bad_run + 1 if (it >= 10 and fix == 0.0) else 0
        if self._bad_run >= self.MIN_CONSECUTIVE and not self._latched:
            self._latched = True
            self._reason = self._describe(line, it)

    def note_session_restart(self) -> None:
        """调用方观察到 LIO 栈重启（launch 日志被截断）时调用 —— 唯一的复位路径。

        同时把帧数清零：复位后必须再攒够 MIN_FRAMES_FOR_VERDICT 帧好帧才会回到
        HEALTHY，否则重启瞬间会有一个恒绿窗口。``_frames`` 归零同时也把饿死计时
        挡在外面 —— ``health`` 的帧数判断排在前，所以**不需要**额外清 ``_last_feed``
        （清了也没有可观测差异，那是死代码）。
        """
        self._frames = 0
        self._bad_run = 0
        self._latched = False
        self._reason = ""

    def _starve_seconds(self) -> float | None:
        """距上次成功 ``feed()`` 的秒数；从没被喂过则 ``None``。"""
        if self._last_feed is None:
            return None
        return self._clock() - self._last_feed

    def _is_starved(self) -> bool:
        gap = self._starve_seconds()
        return gap is not None and gap > self._starve_timeout_s

    @property
    def health(self) -> str | None:
        """``DEGRADED_LIO_DRIFT`` / ``HEALTHY`` / ``None``（不表态）。

        ``None`` 是设计的一部分，不是错误状态：日志路径不对、LIO 根本没起、
        或帧数不足时，调用方应当**不写** mapping_health 字段，让前端显示 "--"、
        让就绪闸门放行。这延续了写端脚本自己立的原则 ——
        「拿不到的一律不编，好过一个假的 0 或者假的 HEALTHY」。

        ⚠ 帧数判断**必须排在饿死判断之前**：``_frames == 0``（从没被喂过）属于
        上面那条第三态，不是饿死。饿死的定义是「它跑过、然后停了」。
        """
        if self._latched:
            return DEGRADED_LIO_DRIFT
        if self._frames < self.MIN_FRAMES_FOR_VERDICT:
            return None
        if self._is_starved():
            return DEGRADED_LIO_DRIFT
        return HEALTHY

    @property
    def reason(self) -> str:
        # 刻意从 health 派生，保证「reason 非空 ⟺ health == DEGRADED」在结构上成立：
        # 否则 _frames 不足时会出现 health 为 None 而 reason 有内容的组合。
        if self.health != DEGRADED_LIO_DRIFT:
            return ""
        if self._latched:
            return self._reason
        gap = self._starve_seconds() or 0.0
        return (
            f"LIO 诊断帧已停 {gap:.1f} 秒"
            f"（阈值 {self._starve_timeout_s:.0f} 秒，判据已被饿死）"
            " —— 建图数据源已不可信，这不是检测到发散；"
            "DIAG 行恢复后会自动解除，无需重启 SLAM 栈"
        )

    def _describe(self, line: str, it: float) -> str:
        pos = _POS.search(line)
        where = (
            f"pos=({float(pos.group(1)):.2f},{float(pos.group(2)):.2f},"
            f"{float(pos.group(3)):.2f})" if pos else "pos=?"
        )
        return (
            f"激光修正停摆 {self._bad_run} 帧"
            f"（iter={it:.0f} fix_t=0，阈值 {self.MIN_CONSECUTIVE} 帧）"
            f" v={_value(line, 'v') or 0.0:.2f}"
            f" act={_value(line, 'act') or 0.0:.3f}"
            f" {where}"
            " —— 大概率已发散，需重启 LIO 栈后才能继续"
        )
