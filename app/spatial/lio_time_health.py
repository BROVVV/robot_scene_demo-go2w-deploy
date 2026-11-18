"""拓扑计划 §4.3 的 **IMU 时间轴 / 位姿静止质量** 判据（纯逻辑，不 import rclpy）。

供 ``scripts/go2w/validate_lio_time_extrinsics.py`` 取数落盘用。§4.3 要的是
「可复跑的 CLI + 一份 JSON」，不是又一道闸门 —— 所以这里**只报数**，
不发明阈值（见下）。

**2026-09-16 在本机实测标定（LIO 栈运行中、狗静止 20 s、5010 样本 = 250.5 Hz）：**

===========================================  ==================================
``imu_stamp_monotonic``                      ``True``，负 dt 0 条
``imu_dt_p50_ms`` / ``p95``                  **4.482** / **5.025**
``imu_dt_min_ms`` / ``max_ms``               **0.122** / **11.061**
``receive_jitter_p95_ms``                    **0.494**
``gravity_norm_mean``                        **10.2233**（区间 9.896 ~ 10.530）
``static_gyro_norm``                         **0.00711**（最大 0.0189）
``pslam_static_drift_m``（另跑 60 s）         **0.0046**（末值 0.0020）
===========================================  ==================================

**三个非显然结论，都是实测推翻了直觉之后记下来的：**

1. **``gravity_norm_mean`` 不是 9.81，是 10.22**（偏离 +4.2%）。§4.3 的 JSON 样例
   里写的是 9.81，照抄那个数去写 ``abs(norm - 9.81) < 0.1`` 会**恒红**。实测区间
   9.896~10.530 与 9.81 完全不重叠，所以这不是噪声而是系统性的。
   **分量把它定位到了轴上**：``accel_mean_xyz_mps2 = [2.88, -0.0006, 9.805]`` ——
   y 轴干净、z 轴正好是 g（9.805），多出来的 2.88 全在 **x 轴**上。这不是姿态倾斜：
   纯倾斜会让 z 按 cosθ 变小（z 该是 9.41 而不是 9.805），而这里是给 x **加了**
   0.29 g 的偏置。两次不同时长的采样都复现同一组数（2.8789 / 2.8799），所以它是
   稳定偏置，不是噪声。**但我仍然不判它** —— 我不知道这个偏置来自狗的 IMU 原始
   标定，还是 aligner/LIO 的"corrected"环节引入的，而我手上只有一台设备、一段
   静止数据，判它就是在编一个没校准过的结论。``accel_mean_xyz_mps2`` 存在的唯一
   理由就是让下一个拿到这份报告的人**不用重新推一遍**这个定位过程。

2. **``receive_jitter_p95_ms`` 只有 0.494，这不能读成"时间轴干净"。** 它量的是
   「到达间隔与 stamp 间隔之差」，而 memory ``go2w-imu-aligner-injects-jitter``
   记的是 aligner **逐样本按 receive 时刻改写 stamp**、把 1.92 ms/样本的抖动灌进
   时间轴 —— 若 stamp 已被贴到 receive 上，这个残差**在构造上就接近 0**。两件事
   不是同一个量，本模块不断言因果，也**不把 0.494 当"无抖动"的证据**。

3. **``imu_dt_min_ms = 0.122`` 与 ``max = 11.061`` 都是真的**：前者是 stamp 重复
   （同刻多帧），后者是丢样（250 Hz 下丢两帧）。p50/p95 看不见这两个尾巴，所以
   min/max 一并报出来 —— §21 那条「无 callback-jitter 型大跳」要的正是尾巴。

**为什么只给 ``imu_stamp_monotonic`` 一个布尔结论。** 它是唯一**不需要标定**的
判据：单调不减是数学性质，不是阈值。§21 另外两条（「IMU dt p95 合理稳定」、
「无 callback-jitter 型大跳」）在原计划里**没有给阈值**，而我手上只有一段 20 s
静止窗口 —— 按本仓库的规矩（没校准过阈值的判据不许进门），这两条留给人看数字，
不由代码下结论。**宁可不判，也不放一个拍出来的门。**
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# --- 判据常数：改了必须重跑 tests/test_lio_time_health_calibration.py ---------
# p95 落在第几个样本上由样本数决定：少于 200 个时"第 95 百分位"完全取决于某一个
# 样本，报出来的数会随采样窗口跳。250 Hz 下 200 个样本 ≈ 0.8 s。
MIN_IMU_SAMPLES = 200
# 与 app/spatial/lio_pose_quality.py 的 MIN_SAMPLES_FOR_VERDICT 同量级：
# 0.1 s 级别的窗口不足以说"这段位姿是静止的"。
MIN_POSE_SAMPLES = 10
# 「静止」沿用本仓库既有的定义，不是新拍的：WebUI 桥的 MotionEpisodeGate 用
# `--speed-max 0.02` / `--yaw-rate-max 0.03` 判静止（scripts/go2w/
# plain_slam_web_bridge.py 的 argparse 默认值）。这里取同一组数，好让"静止"在
# 全仓只有一个含义。
STATIC_MAX_PLANAR_SPEED_MPS = 0.02
STATIC_MAX_ABS_YAW_RATE_RAD_S = 0.03


@dataclass(frozen=True)
class ImuSample:
    """一帧 IMU。``t`` 是收到消息的单调钟（秒），``stamp`` 是 header 时间戳（秒）。

    两个时间都留，因为 §4.3 要量的正是**它们之间的关系**（抖动、单调性）。
    """

    t: float
    stamp: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float

    @property
    def accel_norm(self) -> float:
        return math.sqrt(self.ax**2 + self.ay**2 + self.az**2)

    @property
    def gyro_norm(self) -> float:
        return math.sqrt(self.gx**2 + self.gy**2 + self.gz**2)


def _percentile(values: list[float], fraction: float) -> float:
    """最近秩法取百分位 —— 不插值。

    样本少时插值会造出一个**数据里不存在**的数（两个样本的平均值），而这份报告
    是给人拿去做标定判断的，报出来的每个数都得是真实发生过的样本值。
    """
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * fraction))
    return ordered[index]


class LioTimeHealthJudge:
    """吃 IMU 与 LIO 位姿样本，报 §4.3 那组量。

    两个 feed 各自独立：只有 IMU 就只报 IMU 那几项，位姿那几项是 ``None``
    （而不是 0 —— 0 会被读成"漂移为零，完美"）。
    """

    def __init__(self) -> None:
        self._imu: list[ImuSample] = []
        self._pose_xy: list[tuple[float, float]] = []
        self._pose_t0: float | None = None
        self._pose_t_last = 0.0
        self._twist_seen = False
        self._max_speed = 0.0
        self._max_yaw_rate = 0.0

    def feed_imu(self, sample: ImuSample) -> None:
        self._imu.append(sample)

    def feed_pose(self, t: float, x: float, y: float,
                  speed: float | None = None,
                  yaw_rate: float | None = None) -> None:
        """喂一帧 LIO 位姿 + 该帧的 twist（有就喂，用来确认"这段确实是静止的"）。

        静止确认不是可选的装饰：``static_gyro_norm`` 与 ``pslam_static_drift_m``
        只有在静止时才指代"噪声/漂移"，狗在动的时候它们分别是"角速度"和"走了多远"。
        缺 twist 时 ``static_confirmed`` 报 ``None``（不知道），不是 ``True``。
        """
        if self._pose_t0 is None:
            self._pose_t0 = float(t)
        self._pose_t_last = float(t)
        self._pose_xy.append((float(x), float(y)))
        if speed is not None:
            self._twist_seen = True
            self._max_speed = max(self._max_speed, abs(float(speed)))
        if yaw_rate is not None:
            self._twist_seen = True
            self._max_yaw_rate = max(self._max_yaw_rate, abs(float(yaw_rate)))

    def report(self) -> dict:
        imu = self._imu
        report: dict = {
            "imu_samples": len(imu),
            # 漂移数没有窗口长度就没法解释：0.005 m / 60 s 与 0.005 m / 3 s 是
            # 两个不同的结论。所以跨度与漂移一起报。
            "pose_samples": len(self._pose_xy),
            "pose_span_s": (round(self._pose_t_last - self._pose_t0, 3)
                            if self._pose_t0 is not None else None),
            "imu_stamp_monotonic": None,
            "imu_dt_p50_ms": None,
            "imu_dt_p95_ms": None,
            "imu_dt_min_ms": None,
            "imu_dt_max_ms": None,
            "receive_jitter_p95_ms": None,
            "gravity_norm_mean": None,
            "accel_mean_xyz_mps2": None,
            "static_gyro_norm": None,
            "gyro_mean_xyz_rad_s": None,
            "pslam_static_drift_m": None,
            "static_confirmed": None,
            "thresholds": {
                "static_max_planar_speed_mps": STATIC_MAX_PLANAR_SPEED_MPS,
                "static_max_abs_yaw_rate_rad_s": STATIC_MAX_ABS_YAW_RATE_RAD_S,
                "min_imu_samples": MIN_IMU_SAMPLES,
                "min_pose_samples": MIN_POSE_SAMPLES,
            },
        }
        if self._pose_xy and self._twist_seen:
            report["static_confirmed"] = bool(
                self._max_speed <= STATIC_MAX_PLANAR_SPEED_MPS
                and self._max_yaw_rate <= STATIC_MAX_ABS_YAW_RATE_RAD_S
            )

        if imu:
            # 单调：**非递减**。等号合法（同刻多帧，实测 min dt 0.122 ms 那类），
            # 倒退才是故障 —— LIO 的时间轴一旦倒退，deskew 会按错误的顺序插值。
            differences = [
                (imu[i].stamp - imu[i - 1].stamp) * 1000.0 for i in range(1, len(imu))
            ]
            report["imu_stamp_monotonic"] = bool(
                all(d >= 0.0 for d in differences)
            )
            # 抖动 = 到达间隔与 stamp 间隔之差。**读数前先看模块 docstring 第 2 条**：
            # aligner 把 stamp 贴到 receive 上会让这个残差天然接近 0。
            jitter = [
                abs((imu[i].t - imu[i - 1].t) - (imu[i].stamp - imu[i - 1].stamp))
                * 1000.0
                for i in range(1, len(imu))
            ]
            if jitter:
                report["receive_jitter_p95_ms"] = round(_percentile(jitter, 0.95), 3)
            report["gravity_norm_mean"] = round(
                sum(s.accel_norm for s in imu) / len(imu), 4
            )
            report["accel_mean_xyz_mps2"] = [
                round(sum(getattr(s, axis) for s in imu) / len(imu), 4)
                for axis in ("ax", "ay", "az")
            ]
            report["static_gyro_norm"] = round(
                sum(s.gyro_norm for s in imu) / len(imu), 5
            )
            report["gyro_mean_xyz_rad_s"] = [
                round(sum(getattr(s, axis) for s in imu) / len(imu), 5)
                for axis in ("gx", "gy", "gz")
            ]
            if len(differences) >= MIN_IMU_SAMPLES:
                report["imu_dt_p50_ms"] = round(_percentile(differences, 0.50), 3)
                report["imu_dt_p95_ms"] = round(_percentile(differences, 0.95), 3)
                report["imu_dt_min_ms"] = round(min(differences), 3)
                report["imu_dt_max_ms"] = round(max(differences), 3)

        if len(self._pose_xy) >= MIN_POSE_SAMPLES:
            x0, y0 = self._pose_xy[0]
            report["pslam_static_drift_m"] = round(
                max(math.hypot(x - x0, y - y0) for x, y in self._pose_xy), 4
            )
        return report
