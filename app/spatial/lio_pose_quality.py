"""Pandar 计划 §17.1 的 **LIO 位姿检查** —— 「不要求人工专门操控」的自动质量判断。

**为什么只有 LIO 这一段，不做 PointCloud / IMU 那两段。** §17.1 列了三类检查。
点云与 IMU 那两类的自检**已经在 C++ 侧跑着**：``pandar_slam_adapter.cpp`` 发布
``/go2w/slam/point_status``（含 ``points_total`` / ``non_monotonic`` /
``timestamp_mode``），``plain_slam_health_monitor.cpp`` 据此报
``TIMESTAMP_SYNTHETIC`` 等 flag。在 Python 里再判一遍只会造出**第二个真相源**，
两边不一致时没人知道该信谁 —— 这正是本仓库栽过跟头的那类问题（把两个健康概念
混成一个）。§17.1 里真正**无人做**的只有 LIO 这四项，所以本模块只做这四项。

**阈值来自实测标定，不是拍的。** 数据在
``artifacts/slam_validation/lio_divergence_20260915/``，2026-09-16 逐帧复算。
时间轴：csv 用 ``wall`` 列；``calibration/*.txt`` 是 launch 日志节选、没有 wall，
所以用逐行 ``pt``（LIO 自己的步长，实测 ≈0.1 s）累加。速度取滑动窗口内
「位移 / 时长」，窗口 3 s（允许缩到 80%，即 2.4~3.0 s）。

  负样本 4 组 15568 帧（t_neg_static / t_neg_drive / t_neg_turn /
                       lio_phase_frames_s2.csv）
      3 s 最大速度上界  0.897 m/s   ← 含一次真实驱动（1 s 窗口峰值 1.48 m/s）
      max|z| 上界       0.291 m
  正样本 6 组 50298 帧（**09-15 三次发散全在**：11:31 / 12:40 / 13:20）
      3 s 最大速度下界  132.0 m/s
      max|z| 下界       122.7 m
  已知盲区（**不许**把它说成预警）
      calibration/t_pos_climb.txt  5336 帧  3 s 速度 0.056 m/s，max|z| 0.100 m
      —— 那是 12:40 死亡前的爬升期，本模块**完全不报警**。与
      ``app/spatial/lio_nofix_judge.py`` 一样，覆盖的是**终末段**。

阈值就取在这条缝里：5.0 m/s 比最坏的负样本高 5.6 倍、比最轻的发散低 26 倍；
10.0 m 比最坏的负样本高 34 倍、比最轻的发散低 12 倍。缝宽 4 个数量级，
所以选值不敏感 —— 但**别把它当早期预警**，它和 nofix 闸门一样是事后判据。

**为什么它值得存在。** 现有代码里没有任何一处看位姿的速度或 z：唯一的量级护栏
``scripts/go2w/webui_map_snapshot_writer.py`` 的 ``MAX_TRAJECTORY_ABS_M = 1000``
只管清洗轨迹，对 09-15 那两次（max |xy| 313.7 / 996.8 m）**一次都没碰到**；
而同一批数据的速度是 132~225 m/s，本判据会当场喊出来。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

# 评级词表。只有两级是**可达**的，这不是疏漏：
#   FAILED —— 位姿流本身不可用（下面四项硬检查任一失败）
#   GOOD   —— 四项全过
# 计划书 §17 举的四级里，`USABLE_MAPPING_ONLY` 的判据是 §17.2（pslam 位移 vs
# `/go2w/odom/fused` 位移），本次**不做** —— 两个 odom 世界系没有 TF、夹角与原点
# 每次都变（见 memory `go2w-two-odom-world-frames`），naive 相减得到的数没有意义，
# 且计划没给阈值。`DEGRADED` 在 §17.1 里对应的是点云/IMU 那两段，而它们不在这里
# （见模块开头）。**宁可不写，也不放一个永远不可达的档位**。
GOOD = "GOOD"
FAILED = "FAILED"

# --- 判据常数：改了必须重跑 tests/test_lio_pose_quality_calibration.py ---------
BURST_WINDOW_SECONDS = 3.0
MAX_TRANSLATION_SPEED_MPS = 5.0
MAX_ABS_Z_M = 10.0
# 四元数归一化是数学恒等式，不是可调判断，所以这个容差是标准值而非标定值。
# 标定数据里没有四元数（csv/txt 都只有 x y z），**没有实测支撑**，如实记在这。
QUATERNION_NORM_TOLERANCE = 1e-3
# 环形缓冲保留时长。§17.1 原文写「静止 30 秒内」，这里取同一量级。
WINDOW_SECONDS = 30.0
# 少于这么多帧、或跨度不足一个速度窗口时不下结论（与 LioNofixJudge 的第三态同理）。
MIN_SAMPLES_FOR_VERDICT = 10
# 速度窗口允许缩到 80%，否则端点采样抖动会让 3 s 窗口经常凑不满而漏判。
_MIN_WINDOW_FRACTION = 0.8


@dataclass(frozen=True)
class PoseSample:
    """一帧 LIO 位姿。``t`` 是单调不减的秒。"""

    t: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @property
    def finite(self) -> bool:
        return all(
            math.isfinite(v)
            for v in (self.t, self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
        )

    @property
    def quaternion_norm(self) -> float:
        return math.sqrt(self.qx**2 + self.qy**2 + self.qz**2 + self.qw**2)


def _max_translation_speed(samples: list[PoseSample], window_s: float) -> float:
    """滑动窗口内的最大平均速度（m/s）。

    只对**有限**的样本算：NaN 会让 ``math.dist`` 返回 NaN，而 NaN 比较恒 False，
    于是速度判据会静默失效 —— 那正好把 NaN 检查的活儿又推回给 NaN 检查，
    两边都不报。所以先过滤，NaN 由 ``pose_nan_count`` 单独负责。
    """
    finite = [s for s in samples if s.finite]
    best = 0.0
    j = 0
    for i in range(len(finite)):
        while finite[i].t - finite[j].t > window_s:
            j += 1
        span = finite[i].t - finite[j].t
        if span >= window_s * _MIN_WINDOW_FRACTION:
            moved = math.dist(
                (finite[j].x, finite[j].y, finite[j].z),
                (finite[i].x, finite[i].y, finite[i].z),
            )
            best = max(best, moved / span)
    return best


class LioPoseQualityJudge:
    """吃 LIO 位姿样本，回答「这条位姿流还能不能用」。"""

    def __init__(self, window_s: float = WINDOW_SECONDS) -> None:
        self._window_s = window_s
        self._samples: deque[PoseSample] = deque()

    def feed(self, sample: PoseSample) -> None:
        """喂一帧。要求 ``sample.t`` 相对上一帧单调不减（调用方保证）。"""
        self._samples.append(sample)
        cutoff = sample.t - self._window_s
        while self._samples and self._samples[0].t < cutoff:
            self._samples.popleft()

    def report(self) -> dict:
        """算出 §17.1 的四项 + 评级。字段全是数值或明确的 None，不写自然语言了事。"""
        samples = list(self._samples)
        span = samples[-1].t - samples[0].t if len(samples) >= 2 else 0.0
        nan_count = sum(1 for s in samples if not s.finite)
        finite = [s for s in samples if s.finite]
        quat_dev = (
            max(abs(s.quaternion_norm - 1.0) for s in finite) if finite else 0.0
        )
        speed = _max_translation_speed(samples, BURST_WINDOW_SECONDS)
        max_abs_z = max((abs(s.z) for s in finite), default=0.0)

        report = {
            "verdict": None,
            "reason": "",
            "samples": len(samples),
            "span_s": round(span, 3),
            "max_translation_speed_mps": round(speed, 3),
            "max_abs_z_m": round(max_abs_z, 3),
            "pose_nan_count": nan_count,
            "quaternion_norm_max_deviation": quat_dev,
            "thresholds": {
                "burst_window_s": BURST_WINDOW_SECONDS,
                "max_translation_speed_mps": MAX_TRANSLATION_SPEED_MPS,
                "max_abs_z_m": MAX_ABS_Z_M,
                "quaternion_norm_tolerance": QUATERNION_NORM_TOLERANCE,
            },
        }

        if len(samples) < MIN_SAMPLES_FOR_VERDICT or span < BURST_WINDOW_SECONDS:
            report["reason"] = (
                f"样本不足（{len(samples)} 帧 / {span:.1f}s，需 "
                f"≥{MIN_SAMPLES_FOR_VERDICT} 帧且 ≥{BURST_WINDOW_SECONDS:.0f}s），"
                "不下结论"
            )
            return report

        if nan_count:
            report["verdict"] = FAILED
            report["reason"] = f"位姿出现 {nan_count} 帧 NaN/inf（§17.1 第 1 项）"
        elif quat_dev > QUATERNION_NORM_TOLERANCE:
            report["verdict"] = FAILED
            report["reason"] = (
                f"四元数未归一化：最大偏差 {quat_dev:.4f} > "
                f"{QUATERNION_NORM_TOLERANCE}（§17.1 第 4 项）"
            )
        elif speed > MAX_TRANSLATION_SPEED_MPS:
            report["verdict"] = FAILED
            report["reason"] = (
                f"{BURST_WINDOW_SECONDS:.0f}s 内最大位移速度 {speed:.1f} m/s > "
                f"{MAX_TRANSLATION_SPEED_MPS} m/s —— 位姿已爆走（§17.1 第 2 项）；"
                f"max|z|={max_abs_z:.1f} m"
            )
        elif max_abs_z > MAX_ABS_Z_M:
            report["verdict"] = FAILED
            report["reason"] = (
                f"z 漂移发散：max|z|={max_abs_z:.1f} m > {MAX_ABS_Z_M} m"
                "（§17.1 第 3 项）"
            )
        else:
            report["verdict"] = GOOD
            report["reason"] = (
                f"位姿四项全过（{len(samples)} 帧 / {span:.1f}s，"
                f"峰值 {speed:.2f} m/s，max|z|={max_abs_z:.2f} m）"
            )
        return report
