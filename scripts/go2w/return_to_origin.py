#!/usr/bin/env python3
"""用位姿反馈把机器狗归位到给定目标位姿（胶带原点）并回到目标朝向。

设计取舍（2026-09-10 实测）：

* 反馈必须用**轮式** ``/go2w/odom/fused``：SLAM 的 ``/go2w/slam/odom_base`` 在回环优化时跳变，
  用它做闭环控制会与优化互相打架。
* 单步 ``f0.15`` 实测约前进 0.17 m（含起步超调），所以距离按 ``--step-m`` 估算步数，
  欠执行/过执行由下一轮迭代修正。
* 每一步仍走 ``run_autonomous_loop.py``（保留 sport_mode / lidar_fresh / front_clearance /
  estop 等全部安全门），旋转步仍需 ``--operator-authorized-rotation`` 且每步 ≤30°。

用法：

    # 1) 记住胶带原点（机器人此刻就在原点）：只打印目标位姿
    /usr/bin/python3 scripts/go2w/return_to_origin.py --remember

    # 2) 回到已记住的位姿（只规划，不发运动）
    /usr/bin/python3 scripts/go2w/return_to_origin.py --target-json '{"x":1.0863,"y":2.7612,"yaw":-1.9706}'

    # 3) 真正执行
    /usr/bin/python3 scripts/go2w/return_to_origin.py --target-json '...' --execute
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_pose_probe import probe  # noqa: E402

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))


def wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def turn_steps(delta_deg: float, out: list, min_turn_deg: float) -> None:
    side = "l" if delta_deg > 0 else "r"
    remaining = abs(delta_deg)
    while remaining > 30.0:
        out.append(f"{side}30")
        remaining -= 30.0
    if remaining >= min_turn_deg:
        out.append(f"{side}{remaining:.0f}")


def build_pattern(start: dict, current: dict, step_m: float,
                  tolerance_m: float, tolerance_deg: float,
                  min_turn_deg: float) -> str:
    dx = start["x"] - current["x"]
    dy = start["y"] - current["y"]
    distance = math.hypot(dx, dy)
    yaw_error = math.degrees(wrap(start["yaw"] - current["yaw"]))
    if distance < tolerance_m and abs(yaw_error) < tolerance_deg:
        return ""
    steps: list = []
    if distance >= tolerance_m:
        heading = math.atan2(dy, dx)
        turn_steps(math.degrees(wrap(heading - current["yaw"])), steps, min_turn_deg)
        remaining = distance
        while remaining > 0.15:
            steps.append("f0.15")
            remaining -= step_m
        if remaining >= tolerance_m:
            # 小距离直接按剩余量下指令：f0.05 实测约 0.039 m、f0.15 约 0.17 m，
            # 用固定 f0.15 会在小残差上过冲。
            steps.append(f"f{remaining:.2f}")
        turn_steps(math.degrees(wrap(start["yaw"] - heading)), steps, min_turn_deg)
    else:
        turn_steps(yaw_error, steps, min_turn_deg)
    return ",".join(steps)


def run_pattern(pattern: str, output: str, odom_topic: str, max_seconds: float) -> int:
    command = [
        "/usr/bin/python3",
        os.path.join(PROJECT_ROOT, "scripts", "go2w", "run_autonomous_loop.py"),
        "--mode", "pattern", "--pattern", pattern, "--output", output,
        "--operator-authorized-rotation", "--odom-topic", odom_topic,
        "--max-seconds", str(max_seconds),
    ]
    return subprocess.call(command, cwd=PROJECT_ROOT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odom-topic", default="/go2w/odom/fused")
    parser.add_argument("--target-json", default="")
    parser.add_argument("--remember", action="store_true",
                        help="把当前位姿当作目标并打印（用于记录胶带原点）")
    parser.add_argument("--execute", action="store_true",
                        help="真正发送运动；缺省只规划并打印 pattern")
    parser.add_argument("--max-rounds", type=int, default=5)
    parser.add_argument("--tolerance-m", type=float, default=0.06)
    parser.add_argument("--tolerance-deg", type=float, default=3.0)
    parser.add_argument("--step-m", type=float, default=0.17,
                        help="单步 f0.15 的实测前进距离")
    parser.add_argument("--min-turn-deg", type=float, default=1.5)
    parser.add_argument("--output-dir", default="/tmp/return_to_origin")
    args = parser.parse_args()

    rclpy.init()
    try:
        current = probe(args.odom_topic)
        if "error" in current:
            print(json.dumps({"error": "cannot read odom", "detail": current}))
            return 1
        if args.remember:
            print(json.dumps({"target": current}, ensure_ascii=False))
            return 0
        if not args.target_json:
            print(json.dumps({"error": "--target-json or --remember is required"}))
            return 2
        target = json.loads(args.target_json)

        os.makedirs(args.output_dir, exist_ok=True)
        log_path = os.path.join(args.output_dir, "return_log.jsonl")
        with open(log_path, "a", encoding="utf-8") as handle:
            handle.write(json.dumps({"event": "start", "target": target,
                                     "current": current}, ensure_ascii=False) + "\n")
            for round_index in range(1, args.max_rounds + 1):
                current = probe(args.odom_topic)
                if "error" in current:
                    handle.write(json.dumps({"event": "probe_failed",
                                             "round": round_index}) + "\n")
                    break
                pattern = build_pattern(target, current, args.step_m,
                                        args.tolerance_m, args.tolerance_deg,
                                        args.min_turn_deg)
                handle.write(json.dumps({"event": "round", "index": round_index,
                                         "current": current, "pattern": pattern},
                                        ensure_ascii=False) + "\n")
                print(json.dumps({"round": round_index, "current": current,
                                  "pattern": pattern or "<within-tolerance>"},
                                 ensure_ascii=False))
                if not pattern:
                    break
                if not args.execute:
                    break
                run_pattern(pattern,
                            os.path.join(args.output_dir, f"motion_{round_index}.jsonl"),
                            args.odom_topic, max_seconds=200.0)
                time.sleep(3.0)
            final = probe(args.odom_topic)
            handle.write(json.dumps({"event": "final", "current": final},
                                    ensure_ascii=False) + "\n")
            print(json.dumps({"final": final, "log": log_path}, ensure_ascii=False))
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
