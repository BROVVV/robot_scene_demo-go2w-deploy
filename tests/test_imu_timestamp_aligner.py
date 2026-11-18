"""计划书 §8.4：IMU 时间对齐必须一次锁定，运动期间不许再拖动时间轴。"""

from __future__ import annotations

from ros2_ws.src.go2w_plain_slam_bridge.scripts.imu_timestamp_aligner import (
    ImuTimestampAligner,
)


def test_sensor_stamp_is_preserved_when_clock_is_close() -> None:
    aligner = ImuTimestampAligner()
    first = aligner.update(100.0, 100.02)
    second = aligner.update(100.01, 100.03)
    assert first.valid and second.valid
    assert first.mode == "PASSTHROUGH_SENSOR_STAMP"
    assert second.corrected_sec == 100.01


def test_large_constant_offset_is_locked_after_the_startup_window() -> None:
    aligner = ImuTimestampAligner(lock_after_samples=5)
    values = [aligner.update(10.0 + i * 0.01, 1010.0 + i * 0.01) for i in range(8)]
    # 锁定之前一个样本都不喂 LIO：偏移还在估计，时间轴不许由抖动的中位数决定。
    assert [item.valid for item in values] == [False] * 4 + [True] * 4
    assert values[0].mode == "OFFSET_ESTIMATING_SENSOR_STAMP"
    assert values[0].corrected_sec is None
    assert not values[0].offset_locked
    assert not values[0].restart_required
    assert values[-1].mode == "OFFSET_LOCKED_SENSOR_STAMP"
    assert values[-1].offset_locked
    assert abs(float(values[-1].corrected_sec) - 1010.07) < 1e-6


def test_startup_jitter_never_locks_out_the_imu_stream() -> None:
    """真机回归：/utlidar/imu 落后 ROS 约 730 s，222 Hz 下 raw 每次只增 4.5 ms。

    旧实现在启动窗口里用滚动中位数生成 corrected，中位数下沉一次就判定
    "corrected stamp would move backward" 并永久停止喂 LIO（实测第 8 个样本
    就闭锁，重启 session 必然复现）。
    """
    aligner = ImuTimestampAligner(lock_after_samples=10)
    jitter = [0.10, 0.10, 0.10] + [0.0] * 9
    results = [
        aligner.update(
            1788314743.0 + index * 0.0045,
            1788314743.0 + index * 0.0045 + 730.942 + extra,
        )
        for index, extra in enumerate(jitter)
    ]
    assert not any(item.restart_required for item in results)
    assert all(item.mode == "OFFSET_ESTIMATING_SENSOR_STAMP" for item in results[:9])
    assert aligner.offset_locked
    fed = [float(item.corrected_sec) for item in results if item.valid]
    assert len(fed) == 3
    assert fed == sorted(fed) and len(set(fed)) == 3


def test_callback_jitter_after_lock_never_moves_the_time_axis() -> None:
    """原地转向时 CPU 抖动会让 receive-sensor 跳几十毫秒；时间轴必须不动。"""
    aligner = ImuTimestampAligner(lock_after_samples=4)
    for index in range(4):
        aligner.update(10.0 + index * 0.002, 1010.0 + index * 0.002)
    locked = aligner.estimated_offset_sec
    assert aligner.offset_locked

    jittered = aligner.update(10.010, 1010.010 + 0.040)
    assert jittered.valid
    assert aligner.estimated_offset_sec == locked
    assert abs(float(jittered.corrected_sec) - (10.010 + locked)) < 1e-9
    assert abs(jittered.offset_drift_sec - 0.040) < 1e-6

    settled = aligner.update(10.012, 1010.012 - 0.030)
    assert settled.valid
    assert aligner.estimated_offset_sec == locked
    assert abs(float(settled.corrected_sec) - (10.012 + locked)) < 1e-9


def test_backward_stamp_stops_the_stream_until_a_session_restart() -> None:
    aligner = ImuTimestampAligner()
    assert aligner.update(50.0, 50.0).valid
    bad = aligner.update(49.0, 50.01)
    assert not bad.valid
    assert bad.mode == "IMU_SOURCE_RESTART_REQUIRED"
    assert bad.restart_required
    # 不再自动恢复：后续正常样本也不喂给 LIO。
    blocked = aligner.update(50.01, 50.02)
    assert not blocked.valid
    assert blocked.restart_required
    aligner.reset()
    assert aligner.update(50.02, 50.03).valid


def test_sensor_gap_drops_one_sample_and_then_keeps_feeding_lio() -> None:
    """真机回归：网线闪断让 /utlidar/imu 空洞 18 s，恢复后必须继续喂 LIO。

    空洞不改变已锁定的 offset，永久阻断只会让 LIO 退化成纯几何配准，
    WebUI 三维地图随即出现重影。
    """
    aligner = ImuTimestampAligner(lock_after_samples=3)
    for i in range(3):
        aligner.update(10.0 + i * 0.002, 756.0 + i * 0.002)
    assert aligner.offset_locked
    gap = aligner.update(28.0, 774.0)
    assert not gap.valid
    assert not gap.restart_required
    assert not aligner.restart_required
    assert "gap" in gap.reason
    resumed = aligner.update(28.002, 774.002)
    assert resumed.valid
    assert resumed.mode == "OFFSET_LOCKED_SENSOR_STAMP"
    assert abs(float(resumed.corrected_sec) - 774.002) < 1e-6


def test_sensor_gap_before_offset_lock_clears_the_window_instead_of_killing_the_session() -> None:
    """窗口里出现空洞 → 清空重攒，不判死。

    空洞前后跨着两个纪元，攒下的 offset 样本不可信，所以清空是对的；但判死是错的
    —— offset 阶跃的重锁路径也会把 ``_locked_offset`` 置 None 并维持
    ``lock_after_samples`` 个样本（248 Hz 下 2.0 s），判死等于建图中途任意一次
    2 s 链路闪断都能永久废掉整轮建图（全仓没有生产代码调用 ``reset()``）。
    """
    aligner = ImuTimestampAligner(lock_after_samples=3)
    aligner.update(10.0, 756.0)
    aligner.update(10.002, 756.002)
    gap = aligner.update(28.0, 774.0)
    assert not gap.restart_required
    assert not aligner.restart_required
    assert not gap.valid
    # 恢复后重新攒窗口、重新锁定，IMU 继续喂 LIO。
    values = [aligner.update(28.002 + i * 0.002, 774.002 + i * 0.002)
              for i in range(3)]
    assert aligner.offset_locked
    assert values[-1].valid
    assert abs(float(values[-1].corrected_sec) - 774.006) < 1e-6


def test_sensor_gap_inside_a_relock_window_does_not_kill_the_session() -> None:
    """重锁窗口（248 Hz 下 2.0 s）里撞上链路闪断，不能永久闭锁整轮建图。"""
    aligner = ImuTimestampAligner(lock_after_samples=4, max_offset_step_s=0.05)
    for index in range(4):
        aligner.update(10.0 + index * 0.004, 800.0 + index * 0.004)
    assert aligner.offset_locked
    stepped = aligner.update(10.016, 804.283)          # drift 4.267 s → 重锁
    assert not stepped.valid
    assert not aligner.offset_locked
    gap = aligner.update(13.5, 807.5)                  # 重锁窗口内闪断 3.484 s
    assert not gap.restart_required
    assert not aligner.restart_required
    values = [aligner.update(13.504 + i * 0.004, 807.504 + i * 0.004)
              for i in range(4)]
    assert aligner.offset_locked
    assert values[-1].valid


def test_duplicate_stamp_requires_restart() -> None:
    aligner = ImuTimestampAligner()
    assert aligner.update(50.0, 50.0).valid
    duplicate = aligner.update(50.0, 50.002)
    assert not duplicate.valid
    assert duplicate.restart_required
    assert "duplicate" in duplicate.reason


def test_clock_jump_after_lock_relocks_instead_of_killing_the_session() -> None:
    """真机回归（2026-09-11）：真实偏移从 790.621 阶跃到 795.294（+4.673 s）。

    旧门限是 5.0 s，这一跳刚好被盖住：既不重锁也不报警，时间轴继续用旧 offset，
    IMU 戳整体偏离扫描窗口，LIO 一帧样本都取不到（imu_n=0），退化成纯几何配准，
    错误位姿被当成关键帧写进位姿图 —— WebUI 上就是同一面墙被几个不同角度的副本
    叠在一起。跳变必须走重锁，不能判死、更不能静默沿用旧 offset。
    """
    lock_offset = 790.621305
    step = 4.681533
    aligner = ImuTimestampAligner(lock_after_samples=20)
    for index in range(20):
        aligner.update(1000.0 + index * 0.002, 1000.0 + index * 0.002 + lock_offset)
    assert aligner.offset_locked
    assert abs(aligner.estimated_offset_sec - lock_offset) < 1e-6

    results = [
        aligner.update(1000.0 + index * 0.002,
                       1000.0 + index * 0.002 + lock_offset + step)
        for index in range(20, 41)
    ]

    # 跳变当场被识别：既不判死，也不再用旧 offset 生成时间戳。
    assert not any(item.restart_required for item in results)
    assert not aligner.restart_required
    assert not results[0].valid
    assert results[0].mode == "OFFSET_ESTIMATING_SENSOR_STAMP"
    # 重锁窗口攒满后重新锁定到新偏移。
    assert aligner.offset_locked
    assert abs(aligner.estimated_offset_sec - (lock_offset + step)) < 1e-5
    fed = [float(item.corrected_sec) for item in results if item.valid]
    assert fed and fed == sorted(fed) and len(set(fed)) == len(fed)


def test_threshold_stays_below_the_consumers_scan_window() -> None:
    """门限必须小于**消费端的容忍度**，否则 (容忍度, 门限] 之间的阶跃会静默冻死 IMU。

    LIO 只取落在扫描自身点时间跨度内的 IMU（``lio_3d_interface.cpp:158-173``
    的严格窗口匹配），该窗口实测 p50 0.099 s，再减去 launch 刻意施加的
    ``--stamp-offset-s -0.025``，留给 offset 正漂移的预算只剩约 0.075 s。
    门限一旦大过它，那一档阶跃既不重锁、又让 IMU 戳整体滑出窗口 —— imu_n
    恒为 0，与 09-11 事故同型同签名。
    """
    assert ImuTimestampAligner().max_offset_step_s < 0.075


def test_clock_step_under_the_old_threshold_is_no_longer_silently_swallowed() -> None:
    """4.673 s 的跳变在旧默认 5.0 s 之下，会被静默放过并冻死整轮建图。"""
    aligner = ImuTimestampAligner(lock_after_samples=3)
    for index in range(3):
        aligner.update(10.0 + index * 0.002, 800.0 + index * 0.002)
    jumped = aligner.update(10.006, 800.006 + 4.673)
    assert not jumped.restart_required
    assert not jumped.valid
    assert jumped.mode == "OFFSET_ESTIMATING_SENSOR_STAMP"


def test_step_the_size_of_the_ntp_step_threshold_is_caught() -> None:
    """0.15 s 的阶跃（ntpd 默认 step 阈值就是 128 ms）在旧门限 0.2 下会被静默放过。"""
    aligner = ImuTimestampAligner(lock_after_samples=3)
    for index in range(3):
        aligner.update(10.0 + index * 0.002, 800.0 + index * 0.002)
    jumped = aligner.update(10.006, 800.006 + 0.15)
    assert not jumped.valid
    assert not jumped.restart_required
    assert jumped.mode == "OFFSET_ESTIMATING_SENSOR_STAMP"


def test_invalid_stamp_is_never_published() -> None:
    result = ImuTimestampAligner().update(0.0, 10.0)
    assert not result.valid
    assert result.corrected_sec is None
    assert result.mode == "IMU_SOURCE_INVALID_STAMP"


def test_diagnostics_expose_raw_corrected_offset_and_mode() -> None:
    aligner = ImuTimestampAligner(lock_after_samples=2)
    aligner.update(10.0, 1010.0)
    result = aligner.update(10.002, 1010.002)
    assert result.raw_sensor_sec == 10.002
    assert result.receive_ros_sec == 1010.002
    assert result.offset_samples == 2
    assert result.offset_locked
    assert result.mode == "OFFSET_LOCKED_SENSOR_STAMP"
    assert abs(result.estimated_offset_sec - 1000.0) < 1e-6
