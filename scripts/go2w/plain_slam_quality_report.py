#!/usr/bin/env python3
"""Pandar 计划 §17.1 的 LIO 位姿质量报告 —— **只读，不发布任何东西**。

    python3 scripts/go2w/plain_slam_quality_report.py --duration 30 \
        --report runtime/go2w/plain_slam/quality.json

**必须用 ROS 那个解释器跑。** 实测 anaconda 的 python3.13 **import 不了 rclpy**
（ROS 装的是 3.10 的 C 扩展，报 ``No module named 'rclpy._rclpy_pybind11'``），
source 了 ``/opt/ros/humble/setup.bash`` 也一样 —— 那是 ABI 不匹配，不是环境没配好。
所以要 ``/usr/bin/python3``，并带上
``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`` + ``CYCLONEDDS_URI``（见 memory
``go2w-dds-rmw-cyclonedds-required``：默认 fastrtps 一帧都收不到）。

订一个话题（``/go2w/slam/odom_base``），跑 ``--duration`` 秒，把 §17.1 的四项
LIO 检查结果写成 JSON。判据本身在 ``app/spatial/lio_pose_quality.py``（纯逻辑、
不 import rclpy），这里只是取数与落盘。

**为什么退出码恒为 0、评级只写进 JSON。** 这个脚本的输出是**给人看的质量报告**，
不是闸门。把它接成 ``&&`` 链上的一环，就会有人拿它当"建图是否可用"的自动判据 ——
而 §17.1 的四项只覆盖位姿流本身，覆盖不到地图质量、覆盖不到 `act`/`ab`，
还有一条明确的盲区（见判据模块 docstring）。宁可让人去读 JSON 里的 reason。

**时间基准用收到消息的单调钟，不用 header stamp。** 判据量的是「真实世界里几秒
走了多少米」，而 header stamp 来自 LIO 自己的时钟 —— 时钟坏了正是要抓的故障之一，
拿它当基准会让故障同时毁掉判据。单调钟在构造上单调不减，不需要额外防护。
代价如实记下：BEST_EFFORT 下若消息成串到达，接收时间会被压缩、速度被高估。
本机 DDS 上没观察到这种成串，且真出现成串本身就说明消费端跟不上，不算误报。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from app.spatial.lio_pose_quality import LioPoseQualityJudge, PoseSample  # noqa: E402

DEFAULT_ODOM_TOPIC = "/go2w/slam/odom_base"
DEFAULT_REPORT = "runtime/go2w/plain_slam/quality.json"
# §17.1 原文是「静止 30 秒内」，默认取同一个量级。
DEFAULT_DURATION_S = 30.0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S,
                        help="采样秒数（默认 30）")
    parser.add_argument("--report", default=DEFAULT_REPORT,
                        help=f"输出 JSON 路径（默认 {DEFAULT_REPORT}）")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC,
                        help=f"LIO 位姿话题（默认 {DEFAULT_ODOM_TOPIC}）")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("plain_slam_quality_report")
    judge = LioPoseQualityJudge()
    received = 0

    def on_odom(msg) -> None:
        nonlocal received
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        judge.feed(PoseSample(
            t=time.monotonic(),
            x=position.x, y=position.y, z=position.z,
            qx=orientation.x, qy=orientation.y, qz=orientation.z, qw=orientation.w,
        ))
        received += 1

    # BEST_EFFORT：这个发布者不保证送达，RELIABLE 订阅会一帧都收不到。
    subscription = node.create_subscription(
        Odometry, args.odom_topic, on_odom,
        QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(subscription)

    report = judge.report()
    report["odom_topic"] = args.odom_topic
    report["duration_s"] = args.duration
    report["messages_received"] = received
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

    verdict = report["verdict"] or "NO_VERDICT"
    print(f"[{verdict}] {report['reason']}")
    print(f"  {received} 条消息 / {args.duration:.0f}s -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
