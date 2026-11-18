"""计划书 §8.3：分析脚本的纯函数必须可验证，否则报告不能作为标定依据。"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np

SCRIPT = (Path(__file__).resolve().parents[1]
          / "scripts/go2w/analyze_lio_motion_alignment.py")


def _module():
    spec = importlib.util.spec_from_file_location("lio_alignment_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stream_time_difference_separates_the_sensor_clock_epoch() -> None:
    """实测 /utlidar/imu 的 header 比主机时钟晚 726s；纪元差不能污染残差统计。"""
    mod = _module()
    epoch = 726.257
    imu_receive = np.arange(0.0, 74.0, 0.004) + 1.0e9
    imu_header = imu_receive - epoch
    cloud_receive = np.arange(0.5, 73.5, 0.1) + 1.0e9
    cloud_header = cloud_receive - 0.102
    stats = mod.stream_time_difference((cloud_header, cloud_receive),
                                       (imu_header, imu_receive))
    assert abs(stats["imu_header_minus_receive_s"] + epoch) < 1e-3
    assert abs(stats["cloud_header_minus_receive_s"] + 0.102) < 1e-3
    assert abs(stats["alignment_shift_s"] - (epoch - 0.102)) < 1e-3
    assert abs(stats["imu_clock_drift_s"]) < 1e-6
    # 去掉纪元差后，250 Hz IMU 与点云的最近邻残差只剩采样间隔量级。
    assert stats["max_s"] <= 0.003
    assert stats["count"] == cloud_header.size


def test_static_bag_cannot_conclude_anything_about_axis_or_offset() -> None:
    """静止 60s 的 bag 里没有转向，禁止输出"轴向错误"这种编出来的结论。"""
    mod = _module()
    times = np.arange(0.0, 60.0, 0.05)
    noise = np.random.default_rng(11).normal(0.0, 0.004, times.size)
    excite = mod.excitation(times, noise, np.zeros_like(times))
    assert excite["sufficient"] is False
    report = {
        "excitation": excite,
        "angular_velocity_vs_fused_yaw_rate": {
            "dominant_axis": "z", "axis_ok": False,
            "z": {"best_offset_s": 0.14, "best_correlation": -0.12}},
        "segments": [],
    }
    result = mod.verdict(report)
    assert result["excitation_sufficient"] is False
    assert result["extrinsic_upgrade_allowed"] is False
    assert "没有足够激励" in result["next_action"]
    assert "轴向" in result["next_action"]


def test_excitation_accepts_a_real_thirty_degree_turn_sequence() -> None:
    mod = _module()
    times = np.arange(0.0, 30.0, 0.02)
    yaw_rate = np.zeros_like(times)
    yaw_rate[(times > 5.0) & (times < 8.0)] = math.radians(30.0) / 3.0
    yaw_rate[(times > 15.0) & (times < 18.0)] = -math.radians(30.0) / 3.0
    strong = mod.excitation(times, yaw_rate, np.zeros_like(times))
    assert strong["sufficient"] is True
    assert abs(strong["total_abs_rotation_deg"] - 60.0) < 1.0

    # 单个又慢又小的动作不算激励：峰值角速度太低，拟合不出时间偏移。
    slow = np.zeros_like(times)
    slow[(times > 5.0) & (times < 11.0)] = math.radians(20.0) / 6.0
    assert mod.excitation(times, slow, np.zeros_like(times))["sufficient"] is False


def test_best_offset_recovers_a_known_forty_millisecond_lag() -> None:
    """IMU 晚到 40 ms 时，扫描必须把峰值找回来，而不是停在 0。"""
    mod = _module()
    lag = 0.040
    generator = np.random.default_rng(3)
    odom_t = np.arange(0.0, 20.0, 0.02)
    imu_t = np.arange(-0.5, 20.5, 0.002)
    imu = {"t": imu_t, "z": 0.6 * np.sin(2.0 * math.pi * 0.25 * (imu_t - lag)),
           "x": np.zeros_like(imu_t), "y": np.zeros_like(imu_t)}
    yaw_rate = (0.6 * np.sin(2.0 * math.pi * 0.25 * odom_t)
                + generator.normal(0.0, 0.03, odom_t.size))
    lags, values = mod.lag_scan(imu, "z", odom_t, yaw_rate, 0.3, 0.005)
    result = mod.offset_confidence(lags, values, odom_t.size)
    assert abs(result["best_offset_s"] - lag) < 0.006
    assert result["best_correlation"] > 0.95
    low, high = result["offset_ci_95_s"]
    assert low <= lag <= high
    assert high - low < 0.06


def test_dominant_axis_is_reported_when_yaw_sits_on_x() -> None:
    mod = _module()
    odom_t = np.arange(0.0, 12.0, 0.02)
    yaw_rate = 0.5 * np.sin(2.0 * math.pi * 0.3 * odom_t)
    imu_t = np.arange(0.0, 12.0, 0.004)
    truth = 0.5 * np.sin(2.0 * math.pi * 0.3 * imu_t)
    imu = {"t": imu_t, "x": truth, "y": 0.01 * truth, "z": np.zeros_like(imu_t)}
    report = mod.axis_report(imu, odom_t, yaw_rate, 0.2, 0.005)
    assert report["dominant_axis"] == "x"
    assert report["axis_ok"] is False


def test_icp_recovers_a_known_ten_degree_rotation() -> None:
    """手写 ICP（本机无 open3d）必须能量出 10° 旋转和残差。"""
    mod = _module()
    generator = np.random.default_rng(7)
    target = generator.uniform(-8.0, 8.0, size=(2500, 3))
    target[:, 2] *= 0.15
    angle = math.radians(10.0)
    rotation = np.array([[math.cos(angle), -math.sin(angle), 0.0],
                         [math.sin(angle), math.cos(angle), 0.0],
                         [0.0, 0.0, 1.0]])
    source = (target - np.array([0.2, 0.0, 0.0])) @ rotation
    result = mod.icp_align(source, target, correspondence_m=2.0)
    assert result["converged"]
    assert abs(result["yaw_deg"] - 10.0) < 1.0
    assert result["residual_rms_m"] < 0.05


def test_thirty_degree_gate_fails_on_the_measured_fake_translation() -> None:
    """§8.6：原地转 30° 却出现米级 pslam 平移，必须判 FAIL。"""
    mod = _module()
    fused = {"available": True, "dxy_m": 0.03, "dx_m": 0.03, "dy_m": 0.0,
             "dyaw_deg": -28.5}
    pslam = {"available": True, "dxy_m": 3.24, "dx_m": 3.2, "dy_m": 0.5,
             "dyaw_deg": -27.9}
    rows = {row["gate"]: row for row in mod.gate_rows("turn_30", fused, pslam)}
    assert rows["fused_dxy"]["passed"] is True
    assert rows["pslam_dxy"]["passed"] is False
    assert rows["yaw_sign_agrees"]["passed"] is True
    assert rows["yaw_difference"]["passed"] is True


def test_verdict_blocks_the_extrinsic_upgrade_until_every_gate_passes() -> None:
    mod = _module()
    base = {
        "excitation": {"sufficient": True, "total_abs_rotation_deg": 420.0,
                       "peak_yaw_rate_rad_s": 0.4, "total_travel_m": 1.2},
        "angular_velocity_vs_fused_yaw_rate": {
            "dominant_axis": "z", "axis_ok": True,
            "z": {"best_offset_s": 0.004, "best_correlation": 0.99}},
        "segments": [{"gates": [{"gate": "pslam_dxy", "passed": False}]}],
    }
    blocked = mod.verdict(base)
    assert blocked["extrinsic_upgrade_allowed"] is False
    assert blocked["failed_gate_count"] == 1

    base["segments"] = [{"gates": [{"gate": "pslam_dxy", "passed": True}]}]
    assert mod.verdict(base)["extrinsic_upgrade_allowed"] is True

    base["angular_velocity_vs_fused_yaw_rate"]["axis_ok"] = False
    assert mod.verdict(base)["extrinsic_upgrade_allowed"] is False


def test_segments_are_detected_without_a_timetable() -> None:
    mod = _module()
    times = np.arange(0.0, 30.0, 0.02)
    yaw_rate = np.zeros_like(times)
    yaw_rate[(times > 5.0) & (times < 8.0)] = math.radians(30.0) / 3.0
    yaw_rate[(times > 15.0) & (times < 18.0)] = -math.radians(30.0) / 3.0
    speed = np.zeros_like(times)
    speed[(times > 22.0) & (times < 26.0)] = 0.25
    segments = mod.detect_segments(times, yaw_rate, speed)
    assert [item["label"] for item in segments] == [
        "left_turn", "right_turn", "straight"]
    assert abs(segments[0]["start_s"] - 5.0) < 0.1

