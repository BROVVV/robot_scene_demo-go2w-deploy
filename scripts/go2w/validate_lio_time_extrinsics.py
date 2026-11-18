#!/usr/bin/env python3
"""拓扑计划 §4.3 的 LIO 时间轴/外参检查 —— **只读，不发布任何东西**。

    python3 scripts/go2w/validate_lio_time_extrinsics.py --duration 10 \
        --report outputs/lio_validation.json

订 ``/go2w/slam/imu`` + ``/go2w/slam/odom_base``，跑 ``--duration`` 秒，把 §4.3
点名的那些量写成 JSON。判据在 ``app/spatial/lio_time_health.py``（纯逻辑、不
import rclpy），这里只取数与落盘。

**必须用 ROS 那个解释器跑。** anaconda 的 python3.13 import 不了 rclpy（ROS 装的是
3.10 的 C 扩展，ABI 不匹配），要 ``/usr/bin/python3`` 并 source
``/opt/ros/humble/setup.bash``（**只 source 这一个** —— ``ros2_ws/setup.bash`` 记了
noetic underlay，会让人 import 到 ROS1 的 ``nav_msgs``，见 memory
``go2w-noetic-pythonpath-pollution``），且带上
``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`` + ``CYCLONEDDS_URI``（默认 fastrtps
一帧都收不到）。

**外参只读、不判、更不写。** §4.2 明令禁止把 ``candidate_unconfirmed`` 改成
``confirmed`` 来让测试通过；那个字段的唯一合法来源是
``scripts/go2w/calibrate_pandarxt16_extrinsics.py`` 的多场景标定产物。所以本脚本
把 ``configs/go2w/hesai_pandarxt16_extrinsics.yaml`` 里的候选变换与状态**原样抄进
报告**，``loaded_extrinsic`` 是"运行中的配置里写着什么"，不是"我认为它应该是什么"。

**退出码恒为 0。** 输出是给人做标定判断的报告，不是闸门。§4.3 只有 JSON 形状，
**没有给任何阈值**；接成 ``&&`` 链上的一环，就会有人拿它当"标定是否合格"的自动
判据，而它覆盖不到多场景、也覆盖不到静态外参以外的任何东西。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from app.spatial.lio_time_health import ImuSample, LioTimeHealthJudge  # noqa: E402

DEFAULT_IMU_TOPIC = "/go2w/slam/imu"
DEFAULT_ODOM_TOPIC = "/go2w/slam/odom_base"
DEFAULT_REPORT = "runtime/go2w/plain_slam/lio_validation.json"
DEFAULT_EXTRINSIC = "configs/go2w/hesai_pandarxt16_extrinsics.yaml"
# §4.3 的示例命令用的就是 10 s。
DEFAULT_DURATION_S = 10.0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S,
                        help="采样秒数（默认 10）")
    parser.add_argument("--report", default=DEFAULT_REPORT,
                        help=f"输出 JSON 路径（默认 {DEFAULT_REPORT}）")
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--extrinsic", default=DEFAULT_EXTRINSIC,
                        help="运行中所用外参的配置文件（只读，原样抄进报告）")
    return parser.parse_args(argv)


def _yaml_scalars(text: str) -> dict[tuple[str, ...], str]:
    """按缩进把 ``key: value`` 收成 ``{路径元组: 原文字符串}``。

    不用 pyyaml：ROS 那个解释器没装它，而这份报告只需要几个标量。

    **不能"全文找第一个 ``x:``"** —— 实测踩过：这份文件里
    ``coordinate_convention`` 也有 x/y/z，值是 ``forward``/``left``/``up``，
    先匹配到那里就把外参平移读成了 null。报告里三个 null 看着像"文件里没写",
    其实是读错了地方 —— 静默错答，比报错难查。所以必须带着层级路径查。
    """
    scalars: dict[tuple[str, ...], str] = {}
    stack: list[tuple[int, str]] = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        indent = len(line) - len(line.lstrip())
        key, _, value = stripped.partition(":")
        key, value = key.strip(), value.strip()
        # 键名不含空格 —— 这条把 ``note: >`` 那些正文行（里面有 ``x = ...``、
        # ``udp6_1_parser.h:162``）全部挡在外面，否则它们会被当成键值对收进来。
        if " " in key:
            continue
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if value in ("", ">", "|", ">-", "|-"):
            stack.append((indent, key))
            continue
        scalars[tuple(item for _, item in stack) + (key,)] = value
    return scalars


def _loaded_extrinsic(path: Path) -> dict:
    """把配置里**实际写着**的候选变换与状态读出来。

    刻意不在这里判断"这个外参对不对"：判它需要多场景数据与阈值，那是标定脚本的
    活儿（§4.2）。读到的就是运行中那份配置文件的内容，不是我认为它该是什么。
    """
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        return {"path": str(path), "error": f"{type(exc).__name__}: {exc}"}

    scalars = _yaml_scalars(text)

    def number(*path_keys: str) -> float | None:
        try:
            return float(scalars[path_keys])
        except (KeyError, ValueError):
            return None

    def word(*path_keys: str) -> str | None:
        return scalars.get(path_keys)

    return {
        "path": str(path),
        "sensor_model": word("sensor_model"),
        "calibration_status": word("calibration_status"),
        # 转成真布尔：留着字符串 "false" 的话，`if report["confirmed"]` 会得到真 ——
        # 而这个字段的全部意义就是"不许被读成已确认"（§4.2）。
        "confirmed": word("confirmed") == "true",
        "translation_m": {axis: number("transform_candidate", "translation_m", axis)
                          for axis in ("x", "y", "z")},
        "rotation_rpy_deg": {
            axis: number("transform_candidate", "rotation_rpy_deg", axis)
            for axis in ("roll", "pitch", "yaw")
        },
        "source": word("transform_candidate", "source"),
    }


def _yaw_of(orientation) -> float:
    return math.atan2(2.0 * (orientation.w * orientation.z
                             + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y * orientation.y
                                   + orientation.z * orientation.z))


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = rclpy.create_node("validate_lio_time_extrinsics")
    judge = LioTimeHealthJudge()

    def on_imu(msg) -> None:
        acceleration = msg.linear_acceleration
        gyro = msg.angular_velocity
        judge.feed_imu(ImuSample(
            t=time.monotonic(),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            ax=acceleration.x, ay=acceleration.y, az=acceleration.z,
            gx=gyro.x, gy=gyro.y, gz=gyro.z,
        ))

    def on_odom(msg) -> None:
        pose = msg.pose.pose
        twist = msg.twist.twist
        judge.feed_pose(
            t=time.monotonic(), x=pose.position.x, y=pose.position.y,
            speed=math.hypot(twist.linear.x, twist.linear.y),
            yaw_rate=twist.angular.z,
        )

    # BEST_EFFORT：这两个发布者都不保证送达，RELIABLE 订阅会一帧都收不到。
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Imu, args.imu_topic, on_imu, qos)
    node.create_subscription(Odometry, args.odom_topic, on_odom, qos)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    report = judge.report()
    report["imu_topic"] = args.imu_topic
    report["odom_topic"] = args.odom_topic
    report["duration_s"] = args.duration
    report["imu_rate_hz"] = round(report["imu_samples"] / args.duration, 2)
    report["loaded_extrinsic"] = _loaded_extrinsic(
        PROJECT_ROOT / args.extrinsic if not Path(args.extrinsic).is_absolute()
        else Path(args.extrinsic)
    )
    # §4.3 的 JSON 里有 extrinsic_status 这个顶层键。它**不是**我从外参文件里算出来
    # 的结论，只是把配置里那个字段抬到顶层方便比对 —— 值来自配置文件本身。
    report["extrinsic_status"] = report["loaded_extrinsic"].get("calibration_status")
    report["generated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    out_path = Path(args.report)
    if not out_path.is_absolute():
        out_path = PROJECT_ROOT / out_path
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    node.destroy_node()
    rclpy.shutdown()

    print(f"IMU {report['imu_samples']} 条 / {args.duration:.0f}s "
          f"({report['imu_rate_hz']} Hz)，位姿 {report['pose_samples']} 条 / "
          f"{report['pose_span_s']}s")
    print(f"  stamp 单调={report['imu_stamp_monotonic']}  "
          f"dt p50/p95={report['imu_dt_p50_ms']}/{report['imu_dt_p95_ms']} ms  "
          f"抖动 p95={report['receive_jitter_p95_ms']} ms")
    print(f"  |a|={report['gravity_norm_mean']}  |g|={report['static_gyro_norm']}  "
          f"静止确认={report['static_confirmed']}  "
          f"pslam 漂移={report['pslam_static_drift_m']} m")
    print(f"  外参状态={report['extrinsic_status']}（原样抄自配置，未判）"
          f" -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
