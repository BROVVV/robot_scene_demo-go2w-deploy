#!/usr/bin/env python3
# Copyright 2026 robot_scene_demo maintainers
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Session-locked IMU sensor-clock alignment and monotonicity policy.

计划书 §8.4：一个 mapping session 只允许有一个时钟偏移。启动窗口内用
``receive - sensor`` 的中位数估计偏移并在锁定后才开始输出（锁定前的中位数
每来一个样本就抖一次，用它生成时间戳等于在拖时间轴）；运动期间 callback 到达抖动
只进诊断，绝不再拖动时间轴（旧版滚动中位数窗口只有 62 ms，
CPU 负载一变就把 IMU 时间轴拉着走，这正是原地转向出现数米假平移的来源）。

时间回退与重复戳不会自动恢复：停止给 LIO 喂数据并要求重启 session。
传感器数据空洞（>``max_sensor_gap_s``）是例外——它不改变已锁定的 offset，只丢弃
空洞后的第一帧，随后照常喂数据，这样一次链路闪断不会废掉整个建图会话。丢那一帧
**不是** dt 防护：LIO 的 dt 取自相邻两条**已收到**的报文（lio_3d_node.cpp:215），
拦掉一帧只是把同一段空洞挪给下一帧；真正的兜底是那里的 ``std::clamp(dt, 0, 0.1)``。

锁定后的 offset 阶跃（``max_offset_step_s``）走**重新锁定**，不是判死。判死会让
LIO 从此收不到任何 IMU 样本、退化成纯几何配准，错误位姿被当成关键帧写进位姿图，
地图上就会出现同一面墙被几个不同角度的副本叠在一起（2026-09-11 实测）。重新锁定
的代价是 ``lock_after_samples`` 个样本不喂 IMU：/go2w/slam/imu 实测 248 Hz，
500 个样本 = 2.0 s。

门限必须小于**消费端的容忍度**，而不是随便取一个"比 4.673 s 小"的值。LIO 只取
落在扫描自身点时间跨度内的 IMU（lio_3d_interface.cpp:158-173 的严格窗口匹配），
该窗口实测 p50 0.099 s，再叠加 launch 刻意加的 -25 ms 戳偏置，留给 offset 正漂移
的预算只剩约 0.075 s。门限一旦大过它，落在 (0.075, 门限] 的阶跃既不重锁、又让
IMU 戳整体滑出窗口 —— imu_n 恒为 0，与 09-11 事故同型同签名。实测相邻到达间隔
p50 4.482 ms / max 8.994 ms，0.05 s 有 5.5 倍余量。
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass

MODE_PASSTHROUGH = "PASSTHROUGH_SENSOR_STAMP"
MODE_ESTIMATING = "OFFSET_ESTIMATING_SENSOR_STAMP"
MODE_LOCKED = "OFFSET_LOCKED_SENSOR_STAMP"
MODE_INVALID_STAMP = "IMU_SOURCE_INVALID_STAMP"
MODE_RESTART_REQUIRED = "IMU_SOURCE_RESTART_REQUIRED"


@dataclass(frozen=True)
class TimestampAlignmentResult:
    corrected_sec: float | None
    valid: bool
    mode: str
    raw_sensor_sec: float
    receive_ros_sec: float
    estimated_offset_sec: float = 0.0
    reset_detected: bool = False
    reason: str = ""
    offset_locked: bool = False
    offset_samples: int = 0
    offset_drift_sec: float = 0.0
    restart_required: bool = False


class ImuTimestampAligner:
    """Align sensor time to ROS time once per session, never per callback."""

    def __init__(
        self,
        *,
        lock_after_samples: int = 500,
        passthrough_tolerance_s: float = 5.0,
        max_sensor_gap_s: float = 2.0,
        max_offset_step_s: float = 0.05,
    ) -> None:
        self.lock_after_samples = max(1, int(lock_after_samples))
        self.passthrough_tolerance_s = max(0.0, float(passthrough_tolerance_s))
        self.max_sensor_gap_s = max(0.1, float(max_sensor_gap_s))
        # 下限只是防止传 0 让每个样本都重锁；1 ms 远小于实测到达间隔 p50 4.482 ms。
        self.max_offset_step_s = max(0.001, float(max_offset_step_s))
        self._last_sensor_sec: float | None = None
        self._last_corrected_sec: float | None = None
        self._startup_offsets: list[float] = []
        self._locked_offset: float | None = None
        self._restart_required = False
        self._restart_reason = ""

    @property
    def estimated_offset_sec(self) -> float:
        """Return the locked offset or the diagnostic startup estimate."""
        if self._locked_offset is not None:
            return self._locked_offset
        if self._startup_offsets:
            return float(statistics.median(self._startup_offsets))
        return 0.0

    @property
    def offset_locked(self) -> bool:
        return self._locked_offset is not None

    @property
    def restart_required(self) -> bool:
        return self._restart_required

    def reset(self) -> None:
        """Start a new mapping session: re-estimate and re-lock the offset."""
        self._last_sensor_sec = None
        self._last_corrected_sec = None
        self._startup_offsets = []
        self._locked_offset = None
        self._restart_required = False
        self._restart_reason = ""

    def update(self, sensor_stamp_sec: float, receive_ros_sec: float) -> TimestampAlignmentResult:
        raw = float(sensor_stamp_sec) if _finite(sensor_stamp_sec) else float("nan")
        receive = float(receive_ros_sec) if _finite(receive_ros_sec) else float("nan")
        if self._restart_required:
            return self._blocked(raw, receive, self._restart_reason)
        if not _finite(raw) or raw <= 0.0:
            return self._invalid(raw, receive, "sensor stamp is non-finite or non-positive")
        if not _finite(receive) or receive <= 0.0:
            return self._invalid(raw, receive, "ROS receive time is invalid")

        # 计划书 §8.4：时间回退、重复戳、锁定后 offset 跳变一律停止喂数据并要求重启
        # session；数据空洞只丢一帧，见下方注释。
        if self._last_sensor_sec is not None:
            sensor_dt = raw - self._last_sensor_sec
            if sensor_dt == 0.0:
                return self._require_restart(raw, receive, "duplicate sensor stamp")
            if sensor_dt < 0.0:
                return self._require_restart(raw, receive, "sensor stamp moved backward")
            if sensor_dt > self.max_sensor_gap_s:
                # 数据空洞（网线闪断、丢包）不是时钟异常：已锁定的 offset 依旧成立，
                # 时间轴不会被拖走，所以不要求重启 session。§8.4 禁的是 offset 漂移，
                # 不是数据缺失；永久阻断会让一次 2 s 闪断废掉整轮建图——IMU 恢复后
                # LIO 仍然收不到样本，只能退化成纯几何配准，这正是地图重影的来源。
                # 只丢掉空洞后的这一帧，避免空洞两端的数据混进同一个 offset 窗口。
                # 它消不掉那段 dt —— LIO 的 dt 取自相邻两条已收到的报文，拦一帧只是
                # 把同一段空洞挪给下一帧，真正的兜底是 lio_3d_node.cpp 的 std::clamp。
                self._last_sensor_sec = raw
                gap_reason = (f"sensor stamp gap {sensor_dt:.3f}s exceeds "
                              f"{self.max_sensor_gap_s:.3f}s")
                if self._locked_offset is None:
                    # 窗口里出现空洞，空洞前后跨着两个纪元，攒下的 offset 样本不可信：
                    # 清空重攒。这里**不能**判死——offset 阶跃的重锁路径也会把
                    # _locked_offset 置 None 并维持 lock_after_samples 个样本
                    # （248 Hz 下 2.0 s），判死等于建图中途任意一次 2 s 链路闪断
                    # 都能永久废掉整轮建图，且全仓没有生产代码调用 reset()。
                    self._startup_offsets = []
                return self._invalid(raw, receive, gap_reason)

        offset_sample = receive - raw
        if self._locked_offset is None:
            self._startup_offsets.append(offset_sample)
            if len(self._startup_offsets) >= self.lock_after_samples:
                self._locked_offset = float(statistics.median(self._startup_offsets))
        else:
            drift = offset_sample - self._locked_offset
            if abs(drift) > self.max_offset_step_s:
                # 时钟纪元阶跃（DCU 校时、雷达带着引导纪元重启）。两条老路都是错的：
                # 判死 → LIO 从此一帧 IMU 都收不到，退化成纯几何配准，错误位姿写进
                # 位姿图 → 地图重影；沿用旧 offset → 时间轴静默偏掉，症状一模一样。
                # 2026-09-11 实测就是这个缺口：真实偏移 795.294，锁定值 790.621，
                # 差 4.673 s 卡在旧门限 5.0 s 之下，imu_n 自 09:53 起恒为 0。
                # 按启动窗口重新锁定一次即可，代价是下面这些样本不喂 IMU。
                self._locked_offset = None
                # 新 offset 可能比旧的小，重置单调性基线，否则重锁后的第一帧会被
                # "corrected stamp would move backward" 误判成重启。
                self._last_corrected_sec = None
                self._startup_offsets = [offset_sample]

        self._last_sensor_sec = raw
        if abs(self.estimated_offset_sec) <= self.passthrough_tolerance_s:
            corrected = raw
        elif self._locked_offset is None:
            # 启动窗口的中位数每来一个样本就动一次，用它生成 corrected 就是在
            # 拖时间轴（§8.4 禁止），也会让下面的单调性检查误报回退。这一段
            # 只收集样本，锁定之后才开始给 LIO 喂数据。
            return self._estimating(raw, receive, offset_sample)
        else:
            corrected = raw + self._locked_offset
        if self._last_corrected_sec is not None and corrected <= self._last_corrected_sec:
            return self._require_restart(raw, receive, "corrected stamp would move backward")

        self._last_corrected_sec = corrected
        return self._result(corrected, raw, receive, offset_sample)

    # ---- result builders -------------------------------------------------

    def _result(self, corrected: float, raw: float, receive: float,
                offset_sample: float) -> TimestampAlignmentResult:
        offset = self.estimated_offset_sec
        if abs(offset) <= self.passthrough_tolerance_s:
            mode = MODE_PASSTHROUGH
            reason = "sensor stamp preserved"
        else:
            mode = MODE_LOCKED
            reason = "session-locked sensor->ROS offset applied"
        return TimestampAlignmentResult(
            corrected_sec=corrected, valid=True, mode=mode,
            raw_sensor_sec=raw, receive_ros_sec=receive,
            estimated_offset_sec=offset, reason=reason,
            offset_locked=self.offset_locked,
            offset_samples=len(self._startup_offsets),
            offset_drift_sec=offset_sample - offset,
        )

    def _estimating(self, raw: float, receive: float,
                    offset_sample: float) -> TimestampAlignmentResult:
        offset = self.estimated_offset_sec
        return TimestampAlignmentResult(
            corrected_sec=None, valid=False, mode=MODE_ESTIMATING,
            raw_sensor_sec=raw, receive_ros_sec=receive,
            estimated_offset_sec=offset,
            reason=f"startup offset window filling "
                   f"{len(self._startup_offsets)}/{self.lock_after_samples}",
            offset_samples=len(self._startup_offsets),
            offset_drift_sec=offset_sample - offset,
        )

    def _invalid(self, raw: float, receive: float, reason: str) -> TimestampAlignmentResult:
        return TimestampAlignmentResult(
            corrected_sec=None, valid=False, mode=MODE_INVALID_STAMP,
            raw_sensor_sec=raw, receive_ros_sec=receive,
            estimated_offset_sec=self.estimated_offset_sec, reason=reason,
            offset_locked=self.offset_locked,
            offset_samples=len(self._startup_offsets),
        )

    def _require_restart(
        self, raw: float, receive: float, reason: str,
    ) -> TimestampAlignmentResult:
        if not self._restart_required:
            self._restart_required = True
            self._restart_reason = reason
        return self._blocked(raw, receive, reason)

    def _blocked(self, raw: float, receive: float, reason: str) -> TimestampAlignmentResult:
        return TimestampAlignmentResult(
            corrected_sec=None, valid=False, mode=MODE_RESTART_REQUIRED,
            raw_sensor_sec=raw, receive_ros_sec=receive,
            estimated_offset_sec=self.estimated_offset_sec,
            reset_detected=True, reason=reason,
            offset_locked=self.offset_locked,
            offset_samples=len(self._startup_offsets),
            restart_required=True,
        )


def _finite(value: float) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False
