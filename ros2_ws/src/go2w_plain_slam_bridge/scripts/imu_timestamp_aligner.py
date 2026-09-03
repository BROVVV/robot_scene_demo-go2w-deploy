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

"""Session-locked IMU sensor-clock alignment and monotonicity policy.

计划书 §8.4：一个 mapping session 只允许有一个时钟偏移。启动窗口内用
``receive - sensor`` 的中位数估计偏移并在锁定后才开始输出（锁定前的中位数
每来一个样本就抖一次，用它生成时间戳等于在拖时间轴）；运动期间 callback 到达抖动
只进诊断，绝不再拖动时间轴（旧版滚动中位数在 500 Hz 上只有 62 ms 窗口，
CPU 负载一变就把 IMU 时间轴拉着走，这正是原地转向出现数米假平移的来源）。

时间回退、重复或大跳变不再自动恢复：停止给 LIO 喂数据并要求重启 session。
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
        max_offset_step_s: float = 5.0,
    ) -> None:
        self.lock_after_samples = max(1, int(lock_after_samples))
        self.passthrough_tolerance_s = max(0.0, float(passthrough_tolerance_s))
        self.max_sensor_gap_s = max(0.1, float(max_sensor_gap_s))
        self.max_offset_step_s = max(0.1, float(max_offset_step_s))
        self._last_sensor_sec: float | None = None
        self._last_corrected_sec: float | None = None
        self._startup_offsets: list[float] = []
        self._locked_offset: float | None = None
        self._restart_required = False
        self._restart_reason = ""

    @property
    def estimated_offset_sec(self) -> float:
        """锁定后是恒定偏移；锁定前只是诊断用的估计值，不参与 corrected 生成。"""
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

        # 计划书 §8.4：时间回退、重复或大跳变一律停止喂数据并要求重启 session。
        if self._last_sensor_sec is not None:
            sensor_dt = raw - self._last_sensor_sec
            if sensor_dt == 0.0:
                return self._require_restart(raw, receive, "duplicate sensor stamp")
            if sensor_dt < 0.0:
                return self._require_restart(raw, receive, "sensor stamp moved backward")
            if sensor_dt > self.max_sensor_gap_s:
                return self._require_restart(
                    raw, receive, f"sensor stamp gap {sensor_dt:.3f}s exceeds "
                                  f"{self.max_sensor_gap_s:.3f}s")

        offset_sample = receive - raw
        if self._locked_offset is None:
            self._startup_offsets.append(offset_sample)
            if len(self._startup_offsets) >= self.lock_after_samples:
                self._locked_offset = float(statistics.median(self._startup_offsets))
        else:
            drift = offset_sample - self._locked_offset
            if abs(drift) > self.max_offset_step_s:
                return self._require_restart(
                    raw, receive,
                    f"sensor/ROS clock offset jumped {drift:+.3f}s after lock")

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

    def _require_restart(self, raw: float, receive: float, reason: str) -> TimestampAlignmentResult:
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
