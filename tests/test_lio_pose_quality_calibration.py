"""Pandar 计划 §17.1 LIO 位姿检查 —— 标定、分辨力、以及那条**已知盲区**。

仓库硬规矩：没证明判据有分辨力的"没检出"不算数。所以这里不是"跑一遍看它通过"，
而是三件事：

1. **正样本必须报警** —— 09-15 三次真机发散（11:31 / 12:40 / 13:20）全在数据里，
   6 组正样本一个都不许漏。
2. **负样本必须不报警** —— 4 组 15568 帧，含一次真实驱动（1 s 窗口峰值 1.48 m/s），
   一次急转。
3. **阈值缝必须还在** —— 断言阈值严格落在「最坏负样本」与「最轻发散」之间。
   谁把阈值改成缝外面，这条先红，并告诉他去重跑标定。

**已知盲区写成了断言，不是注释**：``t_pos_climb`` 是 12:40 死亡前的爬升期，
本判据**不报警**，测试里断言它不报警。这样"它只管终末段"这句实话不会在某个
未来提交里被悄悄改成"本判据可提前预警"。

**数据依赖**：``artifacts/slam_validation/`` 不被 git 跟踪，全量用例在数据缺失时
skip（跳过理由里写明预期数字，数据丢了不能假装通过）。恒跑的是
``tests/fixtures/lio_pose_quality/`` 下两份真日志节选。

**没有实测支撑的一项**：csv 与 DIAG 原文都只有 x y z，**没有四元数**，所以
四元数判据没有标定数据，容差 1e-3 是标准值。它由合成用例覆盖（含"有牙齿"的
注入校准），不由真实数据覆盖 —— 不要在别处声称它标定过。
"""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app.spatial.lio_pose_quality import (  # noqa: E402
    BURST_WINDOW_SECONDS,
    FAILED,
    GOOD,
    MAX_ABS_Z_M,
    MAX_TRANSLATION_SPEED_MPS,
    MIN_SAMPLES_FOR_VERDICT,
    WINDOW_SECONDS,
    QUATERNION_NORM_TOLERANCE,
    LioPoseQualityJudge,
    PoseSample,
    _max_translation_speed,
)

ARTIFACTS = ROOT / "artifacts" / "slam_validation" / "lio_divergence_20260915"
FIXTURES = ROOT / "tests" / "fixtures" / "lio_pose_quality"

_POS = re.compile(r"pos=\(([-0-9.eE+]+),([-0-9.eE+]+),([-0-9.eE+]+)\)")
_PT = re.compile(r"pt=([-0-9.eE+]+)")

# 09-15 三次真机发散。11:31 那次在 launch 日志里，其余在 calibration/ 与 csv。
# ``t_pos_climb`` **不在**正样本名单里 —— 它是 12:40 死亡前的爬升期，判据不报警，
# 单列成下面那条盲区断言。放进正样本名单会自相矛盾。
POSITIVE_TXT = ("t_pos_1240_end", "t_pos_1320")
NEGATIVE_TXT = ("t_neg_static", "t_neg_drive", "t_neg_turn")
POSITIVE_CSV = ("session_before_divergence.csv", "frames_1320.csv", "lio_phase_frames.csv")
NEGATIVE_CSV = ("lio_phase_frames_s2.csv",)
BLIND_SPOT_TXT = "t_pos_climb"
# 重放时每隔这么多帧出一份报告（10 帧 = 1 s @10 Hz）。见 _replay 的说明。
REPORT_EVERY = 10


def _parse_diag(text: str) -> list[PoseSample]:
    """GO2W-DIAG 原文 → PoseSample。

    四元数在原文里不存在，填单位值 —— 这正是本组用例不覆盖四元数判据的原因。
    时间轴用逐行 ``pt`` 累加（日志没有 wall）。
    """
    out: list[PoseSample] = []
    t = 0.0
    for line in text.splitlines():
        if "GO2W-DIAG" not in line:
            continue
        pos = _POS.search(line)
        if not pos:
            continue
        pt = _PT.search(line)
        t += float(pt.group(1)) if pt else 0.1
        x, y, z = (float(v) for v in pos.groups())
        out.append(PoseSample(t=t, x=x, y=y, z=z, qx=0.0, qy=0.0, qz=0.0, qw=1.0))
    return out


def _parse_csv(path: Path) -> list[PoseSample]:
    import csv

    out: list[PoseSample] = []
    with path.open() as handle:
        for row in csv.DictReader(handle):
            try:
                out.append(
                    PoseSample(
                        t=float(row["wall"]), x=float(row["x"]), y=float(row["y"]),
                        z=float(row["z"]), qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                    )
                )
            except (ValueError, KeyError):
                continue
    return out


def _load_txt(name: str) -> list[PoseSample]:
    return _parse_diag((ARTIFACTS / "calibration" / f"{name}.txt").read_text(errors="ignore"))


def _load_csv(name: str) -> list[PoseSample]:
    return _parse_csv(ARTIFACTS / name)


def _replay(samples: list[PoseSample]) -> tuple[str | None, bool]:
    """按在线语义喂（30 s 滑动窗口），**每 REPORT_EVERY 帧出一次报告**。

    返回（最后一份报告的评级, 过程中是否出现过 FAILED）。发散是**终末且永久**的，
    所以"中间报过"比"最后报过"更严格 —— 用它断言正样本。

    为什么不逐帧出报告：生产里判据是按定时器调的，不是每来一帧调一次；而
    ``report()`` 是 O(窗口)，逐帧调在全量数据上要 55 秒。10 帧 = 1 秒的节奏，
    比 3 秒的判据窗口密，漏不掉真发散（真发散的位移持续增长很多秒）。
    """
    judge = LioPoseQualityJudge()
    ever_failed = False
    last = None
    final = len(samples) - 1
    for index, sample in enumerate(samples):
        judge.feed(sample)
        if index % REPORT_EVERY and index != final:
            continue
        report = judge.report()
        last = report["verdict"]
        ever_failed = ever_failed or report["verdict"] == FAILED
    return last, ever_failed


needs_artifacts = pytest.mark.skipif(
    not ARTIFACTS.exists(),
    reason=(
        "artifacts/slam_validation/ 不被 git 跟踪。本组断言比的是全量标定数字"
        "（负样本 3s 速度上界 0.897 m/s / max|z| 0.291 m；正样本下界 132 m/s / "
        "122.7 m）。恒跑的降级版本在 tests/fixtures/lio_pose_quality/ 里。"
    ),
)


# --- 恒跑：fixture（真日志节选） ------------------------------------------

def _fixture(name: str) -> list[PoseSample]:
    return _parse_diag((FIXTURES / name).read_text(encoding="utf-8"))


def test_healthy_excerpt_reports_good() -> None:
    """负样本里**最坏**的那一段（t_neg_drive 的 3s 速度峰值处）。"""
    samples = _fixture("healthy.txt")
    assert len(samples) >= MIN_SAMPLES_FOR_VERDICT
    verdict, ever_failed = _replay(samples)
    assert verdict == GOOD, f"健康节选被判 FAILED：{LioPoseQualityJudge and ''}"
    assert not ever_failed, "健康节选在过程中报过 FAILED"


def test_diverged_excerpt_reports_failed() -> None:
    """t_pos_1320 里 3s 速度首次超过阈值的那一段。"""
    samples = _fixture("diverged.txt")
    judge = LioPoseQualityJudge()
    for sample in samples:
        judge.feed(sample)
    report = judge.report()
    assert report["verdict"] == FAILED, f"发散节选没报警：{report}"
    assert report["max_translation_speed_mps"] > MAX_TRANSLATION_SPEED_MPS


def test_healthy_excerpt_would_be_flagged_if_it_were_bad() -> None:
    """**注入校准。** 把健康节选的位移放大，判据必须翻脸。

    没有这一条，上面那条 GOOD 断言是空的 —— 它可能只是因为判据恒不触发。
    """
    samples = _fixture("healthy.txt")
    scaled = [
        PoseSample(t=s.t, x=s.x * 500, y=s.y * 500, z=s.z, qx=s.qx, qy=s.qy, qz=s.qz, qw=s.qw)
        for s in samples
    ]
    _, ever_failed = _replay(scaled)
    assert ever_failed, "把健康节选的位移放大 500 倍后判据仍不报警 —— 判据是死的"


# --- 恒跑：合成用例（覆盖真实数据里没有的字段） --------------------------

def _ramp(n: int = 40, dz: float = 0.0, speed: float = 0.1) -> list[PoseSample]:
    """一条以 ``speed`` m/s 沿 x 走的直线，每次采样 0.1 s。"""
    return [
        PoseSample(t=i * 0.1, x=i * 0.1 * speed, y=0.0, z=dz,
                   qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        for i in range(n)
    ]


def _verdict(samples: list[PoseSample]) -> dict:
    judge = LioPoseQualityJudge()
    for sample in samples:
        judge.feed(sample)
    return judge.report()


def test_nan_pose_is_failed() -> None:
    samples = _ramp()
    samples[20] = PoseSample(t=2.0, x=float("nan"), y=0.0, z=0.0,
                             qx=0.0, qy=0.0, qz=0.0, qw=1.0)
    report = _verdict(samples)
    assert report["verdict"] == FAILED
    assert report["pose_nan_count"] == 1
    assert "NaN" in report["reason"]


def test_denormalized_quaternion_is_failed() -> None:
    """**注入校准**：四元数判据在真实数据里够不着（没有四元数列），
    所以必须在这里证明它不是在空转。"""
    assert _verdict(_ramp())["verdict"] == GOOD
    samples = _ramp()
    samples[10] = PoseSample(t=1.0, x=1.0, y=0.0, z=0.0,
                             qx=0.0, qy=0.0, qz=0.0, qw=2.0)  # 范数 2
    report = _verdict(samples)
    assert report["verdict"] == FAILED
    assert report["quaternion_norm_max_deviation"] > QUATERNION_NORM_TOLERANCE
    assert "四元数" in report["reason"]


def test_z_runaway_is_failed_even_when_flat_in_xy() -> None:
    """z 判据必须独立于速度判据：这条轨迹 x/y 不动，只有 z 在涨。"""
    samples = [
        PoseSample(t=i * 0.1, x=0.0, y=0.0, z=i * 0.5,
                   qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        for i in range(40)
    ]
    report = _verdict(samples)
    assert report["max_translation_speed_mps"] <= MAX_TRANSLATION_SPEED_MPS
    assert report["verdict"] == FAILED
    assert "z 漂移" in report["reason"]


def test_insufficient_samples_do_not_get_a_verdict() -> None:
    """样本不足时**不表态**（``None``），不是 GOOD —— 那会是逻辑上的假绿。"""
    short = _ramp(n=3)
    report = _verdict(short)
    assert report["verdict"] is None
    assert "样本不足" in report["reason"]
    assert report["samples"] == 3


def test_window_evicts_old_samples() -> None:
    """超出窗口的历史必须被丢掉，否则一次瞬时尖峰会永久污染评级。

    判据是**跨度**而不是条数：若旧样本没被驱逐，span 会一路涨到 44 s。
    """
    judge = LioPoseQualityJudge()
    for sample in _ramp(n=40):
        judge.feed(sample)
    assert judge.report()["samples"] == 40
    # 再喂 40 s 的静止样本（t 接上），窗口 30 s ⇒ 40 s 之前的老样本应被挤掉
    base_t = 4.0
    for i in range(401):
        judge.feed(PoseSample(t=base_t + i * 0.1, x=0.0, y=0.0, z=0.0,
                              qx=0.0, qy=0.0, qz=0.0, qw=1.0))
    report = judge.report()
    assert report["span_s"] <= WINDOW_SECONDS + 0.2, (
        f"窗口没生效：跨度 {report['span_s']}s 超过 {WINDOW_SECONDS}s，"
        "旧的尖峰会永久污染评级"
    )
    # 10 Hz、30 s ⇒ 约 301 条
    assert 290 <= report["samples"] <= 310, report["samples"]
    assert report["max_translation_speed_mps"] == 0.0, "老的快样本仍在窗口里"


# --- 全量标定（数据在时才跑） --------------------------------------------

@needs_artifacts
@pytest.mark.parametrize("name", NEGATIVE_TXT + NEGATIVE_CSV)
def test_calibrated_negatives_are_not_flagged(name: str) -> None:
    samples = _load_csv(name) if name.endswith(".csv") else _load_txt(name)
    assert samples, f"{name} 没解析出样本"
    _, ever_failed = _replay(samples)
    assert not ever_failed, f"负样本 {name} 被误报"


@needs_artifacts
@pytest.mark.parametrize("name", POSITIVE_TXT + POSITIVE_CSV)
def test_calibrated_positives_are_flagged(name: str) -> None:
    """**三次真机发散全在名单里**，一个都不许漏。"""
    samples = _load_csv(name) if name.endswith(".csv") else _load_txt(name)
    assert samples, f"{name} 没解析出样本"
    _, ever_failed = _replay(samples)
    assert ever_failed, f"正样本 {name} 没被检出"


@needs_artifacts
def test_known_blind_spot_the_climb_is_not_flagged() -> None:
    """**这是说实话的那条断言，不要"修"它。**

    ``t_pos_climb`` 是 12:40 死亡前的爬升期（5336 帧）。本判据和
    ``lio_nofix_judge`` 一样只看终末段，对爬升期**完全不报警**。
    把它改成断言"应该报警"就等于宣称本判据能提前预警 —— 那是假的。
    """
    samples = _load_txt(BLIND_SPOT_TXT)
    _, ever_failed = _replay(samples)
    assert not ever_failed, (
        "爬升期被检出了 —— 若这是判据改进带来的，请连同 memory 与 docstring "
        "一起更新，并把这条断言升级成正样本；不要只改断言。"
    )


@needs_artifacts
def test_thresholds_sit_inside_the_calibrated_gap() -> None:
    """**阈值的防漂移护栏。** 用生产函数算，不用测试里另写一份。

    改阈值让缝不成立时这条先红，并给出重标定所需的实测数字。
    """
    negatives = [_load_csv(n) if n.endswith(".csv") else _load_txt(n)
                 for n in NEGATIVE_TXT + NEGATIVE_CSV]
    positives = [_load_csv(n) if n.endswith(".csv") else _load_txt(n)
                 for n in POSITIVE_TXT + POSITIVE_CSV if n != BLIND_SPOT_TXT]

    def peak(samples: list[PoseSample]) -> tuple[float, float]:
        return (
            _max_translation_speed(samples, BURST_WINDOW_SECONDS),
            max(abs(s.z) for s in samples),
        )

    neg = [peak(s) for s in negatives]
    pos = [peak(s) for s in positives]
    worst_neg_speed, worst_neg_z = max(v[0] for v in neg), max(v[1] for v in neg)
    best_pos_speed, best_pos_z = min(v[0] for v in pos), min(v[1] for v in pos)

    assert worst_neg_speed < MAX_TRANSLATION_SPEED_MPS < best_pos_speed, (
        f"速度阈值 {MAX_TRANSLATION_SPEED_MPS} 不在缝里："
        f"最坏负样本 {worst_neg_speed:.3f} / 最轻发散 {best_pos_speed:.1f}"
    )
    assert worst_neg_z < MAX_ABS_Z_M < best_pos_z, (
        f"z 阈值 {MAX_ABS_Z_M} 不在缝里："
        f"最坏负样本 {worst_neg_z:.3f} / 最轻发散 {best_pos_z:.1f}"
    )
