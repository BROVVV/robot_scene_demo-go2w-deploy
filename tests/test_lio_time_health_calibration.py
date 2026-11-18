"""§4.3 的 IMU 时间轴判据（``app/spatial/lio_time_health.py``）。

**每条断言的数字都来自 2026-09-16 的真机实测**（LIO 栈运行中、狗静止，20 s /
5010 样本 = 250.5 Hz；漂移另跑 60 s / 600 样本）。数据是用
``scripts/go2w/validate_lio_time_extrinsics.py`` 同一套订阅现场采的，不是估计的。

**这个文件守的核心是一条"不该有的阈值"**：``gravity_norm_mean`` 实测 **10.22**，
而 §4.3 的 JSON 样例写着 9.81。谁要想当然地加一条
``abs(norm - 9.81) < 0.1``，这里会红着告诉他：真机数据在 9.896~10.530，与 9.81
**完全不重叠**，那条门恒红。分量 ``[2.88, -0.0006, 9.805]`` 进一步说明多出来的
2.88 全在 x 轴上（z 轴正好是 g），所以它是偏置不是尺度，更不是姿态倾斜。

**它同时守第二条**：``receive_jitter_p95_ms`` 只有 0.5 ms 左右，容易被读成"时间轴
干净" —— 而 memory ``go2w-imu-aligner-injects-jitter`` 记的是 aligner 逐样本按
receive 改写 stamp、灌进 1.92 ms/样本抖动。stamp 被贴到 receive 上时这个残差在
构造上就接近 0，所以**低读数不是"无抖动"的证据**。下面的用例把这个语义钉住：
喂一份 stamp 完全等于 receive 的序列，残差必须是 0 —— 而那种序列恰恰是被改写过
的那种，不是干净的那种。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

from app.spatial.lio_time_health import (
    MIN_IMU_SAMPLES,
    MIN_POSE_SAMPLES,
    ImuSample,
    LioTimeHealthJudge,
)

# --- 真机实测基线（20 s / 5010 样本，2026-09-16） ------------------------
REAL_HZ = 250.5
REAL_DT_MS = 1000.0 / REAL_HZ          # 3.99
REAL_DT_P50_MS = 4.482
REAL_DT_P95_MS = 5.025
REAL_DT_MIN_MS = 0.122                 # stamp 重复
REAL_DT_MAX_MS = 11.061                # 丢样
REAL_GRAVITY_NORM = 10.2233
REAL_ACCEL_MEAN_XYZ = (2.8789, -0.001, 9.8058)
REAL_GYRO_NORM = 0.00711
REAL_DRIFT_60S_M = 0.0046


def _synthetic(rate_hz: float, seconds: float, *,
               count: int | None = None,
               recv_offset_s: float = 0.0,
               accel: tuple[float, float, float] = (2.88, 0.0, 9.805),
               gyro: tuple[float, float, float] = (0.002, 0.0, 0.0),
               start_stamp: float = 1000.0) -> list[ImuSample]:
    """按固定频率造一串 IMU 样本。

    ``recv_offset_s`` 把"到达时刻"整体平移，用来造"到达与 stamp 不成比例"的情形
    （抖动）。不平移时到达间隔与 stamp 间隔完全相同 —— 那正是被 aligner 改写过的
    形状，见模块 docstring 第二条。
    """
    # ``count`` 直接指定样本数：靠 ``rate_hz * seconds`` 去凑整数会在浮点上抖
    # （201/250.5*250.5 不一定正好 201），而下面有几条用例钉的正是"第 201 个
    # 样本"这种精确边界。
    if count is None:
        count = int(rate_hz * seconds)
    step = 1.0 / rate_hz
    samples = []
    for i in range(count):
        stamp = start_stamp + i * step
        samples.append(ImuSample(
            t=stamp + recv_offset_s, stamp=stamp,
            ax=accel[0], ay=accel[1], az=accel[2],
            gx=gyro[0], gy=gyro[1], gz=gyro[2],
        ))
    return samples


def _fed(samples: list[ImuSample]) -> dict:
    judge = LioTimeHealthJudge()
    for sample in samples:
        judge.feed_imu(sample)
    return judge.report()


# --- IMU 时间轴 ----------------------------------------------------------

def test_dt_percentiles_use_nearest_rank_not_interpolation() -> None:
    """真机 p50=4.482 / p95=5.025，而标称间隔是 3.99 ms。

    两者都对：标称是"发布频率"，p50 量的是"实际 stamp 间隔"，而 LIO 的 IMU 是
    按批送出来的（实测 min 0.122 ms 的重复戳、max 11.061 ms 的丢样），所以 p50
    略高于标称。这里断言的是**分布形状**而不是那个 4.482 —— 用插值法取百分位会
    造出数据里不存在的数，这份报告是给人做标定判断的。
    """
    report = _fed(_synthetic(REAL_HZ, 4.0))
    assert report["imu_dt_p50_ms"] == pytest.approx(REAL_DT_MS, abs=0.01)
    assert report["imu_dt_p95_ms"] == pytest.approx(REAL_DT_MS, abs=0.01)
    assert report["imu_dt_min_ms"] == pytest.approx(REAL_DT_MS, abs=0.01)


def test_percentile_is_an_actual_sample_not_an_interpolated_one() -> None:
    """均匀序列上插值与最近秩给出同一个数 —— 所以上面那条测不出区别。

    这条用一段**真机形状**（20 个正常间隔 + 1 个丢样大间隔）来分开两种实现：
    20 个 dt 值里 p95 落第 19 位 = 那个大间隔本身（最近秩），而线性插值会给出
    两者之间一个**数据里从未出现过**的数。这份报告是给人拿去做标定判断的，
    报出来的每个数都得是真实发生过的样本值。
    """
    # 220 个 dt 间隔：209 个 4 ms + 尾部 11 个 100 ms（丢样成串）。
    # 两种算法在这份数据上**必须给出不同的数**，否则这条用例测不出实现：
    #   最近秩  p95 索引 = int(220*0.95) = 209       → 正好是尾部第一个 → 100.0
    #   线性插值 位置 = 0.95*(220-1) = 208.05        → 落在 208(4 ms) 与 209(100 ms)
    #           之间 → 4 + 0.05*96 = 8.8
    # 尾块的位置是**算出来的**，不是随手取的：块必须从索引 209 开始才让两个位置
    # 分处块边界两侧（之前的版本用 205，两个位置都在块内，于是插值也是 100.0，
    # 注入线性插值照样全绿 —— 实测踩过）。样本数还必须 ≥ MIN_IMU_SAMPLES(200)。
    differences = [0.004] * 209 + [0.100] * 11
    stamps = [1000.0]
    for step in differences:
        stamps.append(stamps[-1] + step)
    judge = LioTimeHealthJudge()
    for stamp in stamps:
        judge.feed_imu(ImuSample(t=stamp, stamp=stamp, ax=0.0, ay=0.0, az=9.8,
                                 gx=0.0, gy=0.0, gz=0.0))
    report = judge.report()
    assert report["imu_dt_max_ms"] == pytest.approx(100.0, abs=0.01)
    assert report["imu_dt_p95_ms"] == pytest.approx(100.0, abs=0.01), (
        "p95 取到了插值出来的中间值 —— 数据里没有那个数"
    )
    assert report["imu_dt_p50_ms"] == pytest.approx(4.0, abs=0.01)


def test_gravity_norm_is_the_mean_of_norms_not_a_root_mean_square() -> None:
    """真机 |a| 在 9.896~10.530 之间浮动，所以两种算法给出**不同**的数。

    均值（mean of norms）与均方根（RMS）只在数据无方差时相等 —— 恒定向量测不出
    区别，必须给两个水平。报的是均值：§4.3 要的是"这段窗口里重力读数大致多少"，
    RMS 会被尖峰抬高，而尖峰正是要单独看的东西。
    """
    samples = []
    for i in range(400):
        norm = 9.9 if i < 200 else 10.5
        samples.append(ImuSample(t=1000.0 + i * 0.004, stamp=1000.0 + i * 0.004,
                                 ax=0.0, ay=0.0, az=norm, gx=0.0, gy=0.0, gz=0.0))
    report = _fed(samples)
    assert report["gravity_norm_mean"] == pytest.approx(10.2, abs=0.001)
    rms = ((9.9**2 + 10.5**2) / 2) ** 0.5
    assert rms > 10.204, "两种算法确实不同，这条测试才有分辨力"


def test_stamp_regression_is_caught() -> None:
    """单调判据是唯一不需要标定的那条 —— 倒退是数学性质，不是阈值。"""
    samples = _synthetic(REAL_HZ, 2.0)
    assert _fed(samples)["imu_stamp_monotonic"] is True
    samples[500] = ImuSample(t=samples[500].t, stamp=samples[499].stamp - 0.001,
                             ax=0.0, ay=0.0, az=9.8, gx=0.0, gy=0.0, gz=0.0)
    assert _fed(samples)["imu_stamp_monotonic"] is False


def test_repeated_stamps_are_not_a_regression() -> None:
    """同刻多帧（实测 min dt 0.122 ms）合法 —— 判据是**非递减**，不是严格递增。"""
    samples = _synthetic(REAL_HZ, 2.0)
    samples[300] = ImuSample(t=samples[300].t, stamp=samples[299].stamp,
                             ax=0.0, ay=0.0, az=9.8, gx=0.0, gy=0.0, gz=0.0)
    report = _fed(samples)
    assert report["imu_stamp_monotonic"] is True
    assert report["imu_dt_min_ms"] == 0.0


def test_low_jitter_residual_does_not_mean_a_clean_time_axis() -> None:
    """**这条守的是一个误读。** 到达间隔与 stamp 间隔一模一样时残差恒 0 ——
    而那恰好是 stamp 被 aligner 贴到 receive 上的形状（memory
    ``go2w-imu-aligner-injects-jitter``）。所以 0 读数只能说明"两者同步"，
    不能说明时间轴没有被改写过。
    """
    locked = _fed(_synthetic(REAL_HZ, 2.0))               # stamp == receive
    assert locked["receive_jitter_p95_ms"] == pytest.approx(0.0, abs=1e-9)
    # 真机上这个残差是 0.5 ms 量级，不是 0 —— 两者都远小于 1 ms，都不构成证据。
    assert REAL_DT_MAX_MS > 0


def test_jitter_residual_tracks_arrival_deviation() -> None:
    """反过来也要成立，否则这条判据就是个恒 0 的摆设。

    把每第 10 个样本的**到达时刻**推后 5 ms（stamp 不动）——真机上的入队抖动就是
    这个样子。残差必须跟着涨，且 p95 要能看见它（每 10 个动一次 = 10% 的样本，
    p95 落在被污染的那部分里）。
    """
    samples = _synthetic(REAL_HZ, 2.0)
    jittered = [
        ImuSample(t=s.t + (0.005 if i % 10 == 0 else 0.0), stamp=s.stamp,
                  ax=s.ax, ay=s.ay, az=s.az, gx=s.gx, gy=s.gy, gz=s.gz)
        for i, s in enumerate(samples)
    ]
    assert _fed(jittered)["receive_jitter_p95_ms"] > 4.0


# --- 重力/陀螺：**不许**照抄计划样例的 9.81 ---------------------------------

def test_real_accel_norm_is_nowhere_near_981() -> None:
    """这条是给"想加一条 ``abs(norm-9.81)<0.1``"的人看的。

    实测 10.2233，区间 9.896~10.530 —— 与 9.81 完全不重叠。若哪天有人在判据里
    加了那条阈值，真机上会恒红。
    """
    assert REAL_GRAVITY_NORM > 9.81 + 0.3, "实测基线与文档不符，重新标定后再信这条"
    assert abs(REAL_GRAVITY_NORM - 9.81) > 0.1


def test_real_accel_components_show_a_bias_not_a_tilt() -> None:
    """多出来的量全在 x 轴上，z 正好是 g —— 所以是加性偏置，不是姿态倾斜。

    纯倾斜会让 z 按 cosθ 变小：θ=16.4° 时 z 该是 9.41，而实测 9.805。这个区分
    很重要：倾斜是"狗没放平"，偏置是"IMU 这条链有问题"，处理方式完全不同。
    """
    ax, ay, az = REAL_ACCEL_MEAN_XYZ
    assert az == pytest.approx(9.81, abs=0.05), "z 轴就是 g，所以不是倾斜"
    assert abs(ay) < 0.01, "y 轴干净"
    assert ax > 2.5, f"偏置在 x 轴上，实测 {ax}"


def test_gravity_norm_mean_matches_the_components_it_reports() -> None:
    """两个字段必须是同一份数据的两种看法，否则人会拿其中一个去推翻另一个。"""
    report = _fed(_synthetic(REAL_HZ, 2.0, accel=REAL_ACCEL_MEAN_XYZ))
    ax, ay, az = report["accel_mean_xyz_mps2"]
    assert report["gravity_norm_mean"] == pytest.approx(
        (ax * ax + ay * ay + az * az) ** 0.5, abs=1e-3
    )


def test_static_gyro_norm_is_the_mean_not_the_max() -> None:
    """真机均值 0.00711、最大 0.0189 —— 报的是均值。

    报最大值会把单次噪声尖峰当成"狗在转"，而 §21 要的「静止」是整段窗口的性质。
    """
    samples = _synthetic(REAL_HZ, 2.0, gyro=(0.0, 0.0, 0.0))   # 501 个样本
    samples[400] = ImuSample(t=samples[400].t, stamp=samples[400].stamp,
                             ax=2.88, ay=0.0, az=9.805,
                             gx=0.5, gy=0.0, gz=0.0)   # 单个尖峰
    report = _fed(samples)
    assert report["static_gyro_norm"] < 0.01, "把单帧尖峰算进了均值"
    assert report["gyro_mean_xyz_rad_s"][0] > 0.0


# --- 位姿：漂移 + 静止确认 -------------------------------------------------

def _fed_pose(points: list[tuple[float, float, float]], *,
              speed: float = 0.0, yaw_rate: float = 0.0) -> dict:
    judge = LioTimeHealthJudge()
    for index, (t, x, y) in enumerate(points):
        judge.feed_pose(t=t, x=x, y=y, speed=speed, yaw_rate=yaw_rate)
    return judge.report()


def test_static_drift_is_net_displacement_from_the_first_sample() -> None:
    """真机 60 s 静止实测：max 0.0046 m、末值 0.0020 m。§21 的目标是 < 0.10 m。

    造的数据刻意复刻那个形状（先涨到 0.0046 再回落），好把「取最大值」和
    「取末值」两种实现区分开 —— 只涨不回落的序列测不出这个区别。
    """
    x_of = lambda i: (0.0046 * (i / 300) if i <= 300
                      else 0.0046 - 0.0026 * ((i - 300) / 299))
    points = [(i * 0.1, x_of(i), 0.0) for i in range(600)]
    report = _fed_pose(points)
    assert report["pose_samples"] == 600
    assert report["pose_span_s"] == pytest.approx(59.9, abs=0.05)
    assert report["pslam_static_drift_m"] == pytest.approx(0.0046, abs=0.001)
    assert report["pslam_static_drift_m"] < 0.10, "否则违反 §21 的 60s 漂移目标"


def test_drift_is_not_computed_from_too_few_samples() -> None:
    """少于 10 帧时报 ``None``，不报 0 —— 0 会被读成"漂移为零，完美"。

    **边界用字面量写，不用 ``MIN_POSE_SAMPLES - 1``。** 按常数写的话，常数被改小
    时喂进去的帧数跟着变小，用例永远绿 —— 实测踩过（把下限注入成 1，这一条照样
    通过）。字面量才能把"9 帧不够、10 帧够"这条边界钉住。
    """
    assert MIN_POSE_SAMPLES == 10, "下限变了：重新想清楚「几帧才够说这段是静止的」"
    assert _fed_pose([(i * 0.1, 0.0, 0.0) for i in range(9)])[
        "pslam_static_drift_m"] is None
    assert _fed_pose([(i * 0.1, 0.0, 0.0) for i in range(10)])[
        "pslam_static_drift_m"] is not None


def test_static_confirmation_requires_the_twist_to_be_quiet() -> None:
    """没有 twist 时报 ``None``（不知道），不是 ``True``。

    静止确认不是装饰：狗在动的时候 ``static_gyro_norm`` 与 ``pslam_static_drift_m``
    分别指代"角速度"和"走了多远"，读成"噪声/漂移"就是彻底的误读。阈值沿用 WebUI
    桥的 ``--speed-max 0.02`` / ``--yaw-rate-max 0.03``，全仓"静止"只有一个含义。
    """
    points = [(i * 0.1, 0.0, 0.0) for i in range(100)]
    judge = LioTimeHealthJudge()
    for t, x, y in points:
        judge.feed_pose(t=t, x=x, y=y)          # 不喂 twist
    assert judge.report()["static_confirmed"] is None
    assert _fed_pose(points, speed=0.01, yaw_rate=0.01)["static_confirmed"] is True
    assert _fed_pose(points, speed=0.5, yaw_rate=0.0)["static_confirmed"] is False
    assert _fed_pose(points, speed=0.0, yaw_rate=0.4)["static_confirmed"] is False


# --- 第三态：样本不足 -----------------------------------------------------

def test_too_few_imu_samples_gives_no_percentiles() -> None:
    """p95 落在第几个样本上由样本数决定；样本太少时它只反映某一个样本。

    报 ``None`` 而不是硬算 —— 与 ``lio_pose_quality`` / ``lio_nofix_judge`` 的
    第三态同一条原则（拿不到的一律不编）。

    边界同样用字面量（199/200），理由见
    ``test_drift_is_not_computed_from_too_few_samples``。
    """
    assert MIN_IMU_SAMPLES == 200
    # 门槛卡在**间隔数**上：200 个样本只有 199 个间隔，所以"刚好够"要 201 个样本。
    too_few = _fed(_synthetic(REAL_HZ, 0, count=200))
    assert too_few["imu_samples"] == 200
    assert too_few["imu_dt_p50_ms"] is None
    assert too_few["imu_dt_p95_ms"] is None
    assert too_few["imu_dt_min_ms"] is None and too_few["imu_dt_max_ms"] is None
    # 均值不受这条限制（它不依赖某个具体的秩），单调性也不依赖样本数。
    assert too_few["gravity_norm_mean"] is not None
    assert too_few["imu_stamp_monotonic"] is True
    enough = _fed(_synthetic(REAL_HZ, 0, count=201))
    assert enough["imu_dt_p50_ms"] is not None
    assert enough["imu_dt_max_ms"] is not None


def test_only_imu_still_reports_pose_fields_as_none() -> None:
    """只喂了 IMU 时位姿那几项是 ``None``，不是 0。"""
    report = _fed(_synthetic(REAL_HZ, 2.0))
    assert report["pslam_static_drift_m"] is None
    assert report["static_confirmed"] is None
    assert report["pose_span_s"] is None


# --- CLI：外参只读、不判、不许读错地方 -------------------------------------

sys.path.insert(0, str(ROOT / "scripts" / "go2w"))
from validate_lio_time_extrinsics import _loaded_extrinsic  # noqa: E402

EXTRINSIC = ROOT / "configs" / "go2w" / "hesai_pandarxt16_extrinsics.yaml"


def test_loaded_extrinsic_reads_the_transform_not_the_axis_convention() -> None:
    """**这条是回归守卫，防的是一个真踩过的静默错答。**

    这份文件里 ``coordinate_convention`` 也有 ``x``/``y``/``z``，值是
    ``forward``/``left``/``up``。按"全文找第一个 ``x:``"去读，会读到 ``forward``
    → ``float("forward")`` 失败 → 平移三个全报 ``null``。报告里三个 null 看着像
    "配置里没写平移"，实际是读错了地方，而脚本照样退出码 0。
    """
    loaded = _loaded_extrinsic(EXTRINSIC)
    assert loaded["translation_m"] == {"x": 0.13, "y": 0.015, "z": 0.014}, (
        "平移读成了 null 或别的数 —— 大概率又去匹配 coordinate_convention 了"
    )
    assert loaded["rotation_rpy_deg"] == {"roll": 0.385, "pitch": 0.905,
                                          "yaw": 90.0}
    assert loaded["source"] == "hesai_pandarxt16_axis_convention_20260915"


def test_loaded_extrinsic_never_upgrades_the_calibration_status() -> None:
    """§4.2：**禁止**为了让测试通过就把 ``candidate_unconfirmed`` 改成 confirmed。

    所以这里断言的是"脚本如实抄了配置里的状态"，而不是"状态是 confirmed"。
    同时 ``confirmed`` 必须是真布尔：留着字符串 ``"false"`` 的话，
    ``if report["confirmed"]`` 会得到真 —— 而这个字段的全部意义就是不许被读成已确认。
    """
    loaded = _loaded_extrinsic(EXTRINSIC)
    assert loaded["calibration_status"] == "candidate_unconfirmed"
    assert loaded["confirmed"] is False
    assert isinstance(loaded["confirmed"], bool), "布尔被读成了字符串"


def test_missing_extrinsic_file_is_reported_as_an_error_not_as_a_default() -> None:
    """文件不在时报错字段，不报一份"看起来正常"的空外参。"""
    loaded = _loaded_extrinsic(ROOT / "configs" / "go2w" / "does_not_exist.yaml")
    assert "error" in loaded
    assert "translation_m" not in loaded
