#!/usr/bin/env python3
"""ROS2 Humble worker for Nav2 file IPC.

Run only after sourcing ROS and the robot workspace. It deliberately uses no
packages from the application's conda environment.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import sys
import tempfile
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

STOP_REQUESTED = False


def now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="."+path.name+".", dir=path.parent)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2); stream.write("\n")
        stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


def append_jsonl(path, value):
    with Path(path).open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, ensure_ascii=False)+"\n"); stream.flush()


def status(job, request, state, message, **extra):
    value = {"schema_version": "1.0", "request_id": request.get("request_id"),
             "state": state, "backend": "nav2_humble", "is_real_nav2_path": state not in ("created", "validating", "unavailable"),
             "worker_pid": os.getpid(), "updated_at": now(), "message_zh": message,
             "nav2_available": state not in ("unavailable",)}
    old = job/"status.json"
    if old.exists():
        try: value = {**json.loads(old.read_text(encoding="utf-8")), **value}
        except Exception: pass
    value.update(extra)
    if state in ("unavailable", "canceled", "succeeded", "failed", "timed_out"):
        value["finished_at"] = now()
    atomic_json(old, value)


def make_pose(navigator, value):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped(); pose.header.frame_id = value["frame_id"]
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(value["x"]); pose.pose.position.y = float(value["y"])
    pose.pose.position.z = float(value.get("z", 0))
    yaw = float(value.get("yaw_rad", 0))
    pose.pose.orientation.z = math.sin(yaw/2); pose.pose.orientation.w = math.cos(yaw/2)
    return pose


def serialize_path(job, request, path):
    rows = []
    cumulative = 0.0
    previous = None
    for index, stamped in enumerate(path.poses):
        p, q = stamped.pose.position, stamped.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if previous is not None: cumulative += math.hypot(p.x-previous[0], p.y-previous[1])
        rows.append({"index": index, "x": p.x, "y": p.y, "yaw_rad": yaw, "cumulative_distance_m": cumulative})
        previous = (p.x, p.y)
    payload = {"schema_version": "1.0", "request_id": request["request_id"],
               "backend": "nav2_humble", "is_real_nav2_path": True,
               "frame_id": path.header.frame_id or request["map_frame"],
               "planning_time_sec": 0.0, "planner_id": request["planner_id"],
               "path_length_m": cumulative, "pose_count": len(rows),
               "start_pose": rows[0] if rows else None, "goal_pose": rows[-1] if rows else None,
               "poses": rows, "created_at": now()}
    atomic_json(job/"global_path.json", payload)
    with (job/"global_path.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["index","x","y","yaw_rad","cumulative_distance_m"])
        writer.writeheader(); writer.writerows(rows)
    return payload


def safety_check(request):
    if request["mode"] != "execute": return
    safety = request.get("safety_confirmation", {})
    if not request.get("allow_execute"): raise RuntimeError("NAV2_EXECUTION_NOT_ALLOWED: allow_execute=false")
    required = ("webui_confirmed", "environment_allowed", "footprint_confirmed", "emergency_stop_confirmed")
    if not all(safety.get(k) for k in required):
        raise RuntimeError("NAV2_SAFETY_CONFIRMATION_MISSING: 安全确认不完整")
    gate = request.get("capability_gate_result")
    if not isinstance(gate, dict):
        raise RuntimeError("NAV2_CAPABILITY_GATE_MISSING: execution gate result is required")
    if gate.get("schema_version") != "1.0" or gate.get("mode") != "nav2_execute":
        raise RuntimeError("NAV2_CAPABILITY_GATE_INVALID: invalid execution gate result")
    required_gate = (
        "level_d_passed", "physical_geometry_confirmed", "footprint_confirmed",
        "scan_frame_valid", "lidar_fresh", "lio_fresh", "map_valid", "tf_valid",
        "compute_path_to_pose_ready", "nav2_allow_execute", "collision_monitor_active",
        "velocity_smoother_active", "lease_valid", "arbiter_active",
        "cmd_vel_bridge_active", "cmd_vel_watchdog_active", "operator_armed",
        "second_confirmation", "emergency_stop_confirmed", "remote_override_clear",
        "robot_error_zero",
    )
    if gate.get("required_conditions") != list(required_gate):
        raise RuntimeError("NAV2_CAPABILITY_GATE_INVALID: execution requirements mismatch")
    evidence = gate.get("evidence")
    if not isinstance(evidence, dict) or not all(evidence.get(name) is True for name in required_gate):
        raise RuntimeError("NAV2_CAPABILITY_GATE_INVALID: required live evidence is incomplete")
    blockers = gate.get("blocking_conditions")
    if gate.get("allowed") is not True or blockers != []:
        detail = ",".join(str(item) for item in (blockers or ["gate_not_allowed"]))
        raise RuntimeError(f"NAV2_CAPABILITY_GATE_BLOCKED: {detail}")


def wait_for_action_server(client, timeout, name):
    if not client.wait_for_server(timeout_sec=float(timeout)):
        raise RuntimeError(
            f"NAV2_ACTION_SERVER_UNAVAILABLE: {name} 在 {timeout} 秒内不可用；"
            "请启动 Nav2 bringup，并检查 namespace 与 ROS_DOMAIN_ID"
        )


def wait_for_future(rclpy, navigator, future, deadline, timeout_error):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError(timeout_error)
    rclpy.spin_until_future_complete(navigator, future, timeout_sec=remaining)
    if not future.done():
        raise RuntimeError(timeout_error)
    return future.result()


def get_path_with_timeout(rclpy, navigator, start, goal, request):
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import ComputePathToPose

    timeout = float(request.get("planning_timeout_sec", 30))
    wait_for_action_server(
        navigator.compute_path_to_pose_client, timeout, "compute_path_to_pose"
    )
    deadline = time.monotonic() + timeout
    goal_msg = ComputePathToPose.Goal()
    if start is not None:
        goal_msg.start = start
    goal_msg.goal = goal
    goal_msg.planner_id = request.get("planner_id", "")
    goal_msg.use_start = not request.get("use_current_robot_pose_as_start", True)
    goal_handle = wait_for_future(
        rclpy,
        navigator,
        navigator.compute_path_to_pose_client.send_goal_async(goal_msg),
        deadline,
        "NAV2_PLANNING_TIMEOUT: 提交规划请求超时",
    )
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError("NAV2_PLAN_REJECTED: Nav2 拒绝了规划请求")
    navigator.goal_handle = goal_handle
    result = wait_for_future(
        rclpy,
        navigator,
        goal_handle.get_result_async(),
        deadline,
        "NAV2_PLANNING_TIMEOUT: 全局路径规划超时",
    )
    if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
        code = getattr(result, "status", "unknown")
        raise RuntimeError(f"NAV2_NO_PATH: Nav2 规划失败，状态码={code}")
    return result.result.path


def start_navigation_with_timeout(rclpy, navigator, goal, behavior_tree, timeout):
    from nav2_msgs.action import NavigateToPose

    wait_for_action_server(navigator.nav_to_pose_client, timeout, "navigate_to_pose")
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = goal
    goal_msg.behavior_tree = behavior_tree
    deadline = time.monotonic() + float(timeout)
    goal_handle = wait_for_future(
        rclpy,
        navigator,
        navigator.nav_to_pose_client.send_goal_async(
            goal_msg, navigator._feedbackCallback
        ),
        deadline,
        "NAV2_ACTION_TIMEOUT: 提交导航请求超时",
    )
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError("NAV2_GOAL_REJECTED: Nav2 拒绝了导航目标")
    navigator.goal_handle = goal_handle
    navigator.result_future = goal_handle.get_result_async()


def run(request_path):
    request_path = Path(request_path).resolve(); job = request_path.parent
    request = json.loads(request_path.read_text(encoding="utf-8"))
    status(job, request, "validating", "正在校验 ROS2/Nav2 运行环境")
    if os.getenv("ROS_DISTRO") not in (None, "", "humble"):
        raise RuntimeError("NAV2_ROS_DISTRO_UNSUPPORTED: 仅支持 ROS2 Humble")
    safety_check(request)
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    except ImportError as exc:
        raise RuntimeError("NAV2_RCLPY_IMPORT_FAILED: "+str(exc))
    rclpy.init()
    navigator = BasicNavigator(namespace=request.get("namespace", ""))
    last_cmd = 0.0
    cmd_count = 0
    def cmd_callback(message):
        nonlocal last_cmd, cmd_count
        wall = time.monotonic()
        hz = float(os.getenv("NAV2_CMD_VEL_SAMPLE_HZ", "10"))
        if wall-last_cmd < 1/max(1, hz): return
        last_cmd = wall; cmd_count += 1
        append_jsonl(job/"cmd_vel_trace.jsonl", {"timestamp": now(), "linear_x": message.linear.x,
            "linear_y": message.linear.y, "angular_z": message.angular.z,
            "topic": request.get("cmd_vel_topic", "/cmd_vel"), "simulated": False})
    if request["mode"] == "execute" and request.get("capture_cmd_vel", True):
        navigator.create_subscription(Twist, request.get("cmd_vel_topic", "/cmd_vel"), cmd_callback, 20)
    try:
        status(job, request, "ready", "Nav2 Worker 已就绪", nav2_available=True)
        goal = make_pose(navigator, request["goal_pose"])
        use_current = request.get("use_current_robot_pose_as_start", True)
        start = None if use_current else make_pose(navigator, request["start_pose"])
        status(job, request, "planning", "正在调用 Nav2 全局规划")
        started = time.monotonic()
        path = get_path_with_timeout(rclpy, navigator, start, goal, request)
        if path is None or not path.poses:
            raise RuntimeError("NAV2_NO_PATH: Nav2 未返回可行路径")
        path_payload = serialize_path(job, request, path)
        path_payload["planning_time_sec"] = time.monotonic() - started
        atomic_json(job/"global_path.json", path_payload)
        status(job, request, "planned", "Nav2 已生成真实全局路径",
               path_length_m=path_payload["path_length_m"])
        if request["mode"] == "plan_only": return 0
        behavior_tree = request.get("behavior_tree", "")
        start_navigation_with_timeout(
            rclpy,
            navigator,
            goal,
            behavior_tree,
            request.get("planning_timeout_sec", 30),
        )
        status(job, request, "executing", "正在执行 Nav2 导航", path_length_m=path_payload["path_length_m"])
        execute_started = time.monotonic()
        while not navigator.isTaskComplete():
            if STOP_REQUESTED or (job/"cancel.request").exists():
                status(job, request, "canceling", "正在取消 Nav2 导航")
                navigator.cancelTask()
            elapsed = time.monotonic()-execute_started
            if elapsed > float(request.get("execution_timeout_sec", 300)):
                navigator.cancelTask()
                status(job, request, "timed_out", "Nav2 执行超时", error_code="NAV2_EXECUTION_TIMEOUT")
                return 4
            feedback = navigator.getFeedback()
            if feedback:
                current = feedback.current_pose.pose
                q = current.orientation
                sample = {"timestamp": now(), "state": "executing",
                    "current_pose": {"frame_id": feedback.current_pose.header.frame_id,
                        "x": current.position.x, "y": current.position.y,
                        "yaw_rad": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))},
                    "distance_remaining_m": float(feedback.distance_remaining),
                    "estimated_time_remaining_sec": feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec/1e9,
                    "navigation_time_sec": feedback.navigation_time.sec + feedback.navigation_time.nanosec/1e9,
                    "number_of_recoveries": int(feedback.number_of_recoveries)}
                append_jsonl(job/"feedback.jsonl", sample)
                status(job, request, "executing", "正在执行 Nav2 导航", **{k:v for k,v in sample.items() if k not in ("timestamp","state")}, cmd_vel_samples=cmd_count)
            time.sleep(float(request.get("feedback_interval_sec", .5)))
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            status(job, request, "succeeded", "Nav2 导航成功", progress_ratio=1.0, cmd_vel_samples=cmd_count)
            return 0
        if result == TaskResult.CANCELED:
            status(job, request, "canceled", "Nav2 导航已取消", error_code="NAV2_TASK_CANCELED", cmd_vel_samples=cmd_count)
            return 0
        raise RuntimeError("NAV2_TASK_FAILED: Nav2 导航任务失败")
    finally:
        navigator.destroyNode()
        rclpy.shutdown()


def health_check():
    result = {"healthy_for_plan": False, "healthy_for_execute": False, "checks": [], "blocking_errors": [], "warnings": []}
    for name, module in (("rclpy", "rclpy"), ("nav2_simple_commander", "nav2_simple_commander.robot_navigator")):
        try: __import__(module); result["checks"].append({"name": name, "ok": True, "message": "可导入"})
        except ImportError as exc:
            result["checks"].append({"name": name, "ok": False, "message": str(exc)}); result["blocking_errors"].append(name)
    result["healthy_for_plan"] = not result["blocking_errors"]
    result["healthy_for_execute"] = result["healthy_for_plan"] and os.getenv("NAV2_ALLOW_EXECUTE","false").lower() == "true"
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["healthy_for_plan"] else 3


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--request"); parser.add_argument("--health-check", action="store_true")
    args = parser.parse_args()
    if args.health_check: return health_check()
    if not args.request: parser.error("--request is required")
    request_path = Path(args.request)
    try: return run(request_path)
    except Exception as exc:
        request = json.loads(request_path.read_text(encoding="utf-8")) if request_path.exists() else {"request_id":"unknown"}
        text = str(exc); code = text.split(":",1)[0] if text.startswith("NAV2_") else "NAV2_WORKER_CRASHED"
        state = "unavailable" if code in {"NAV2_RCLPY_IMPORT_FAILED","NAV2_COMMANDER_IMPORT_FAILED","NAV2_ACTION_SERVER_UNAVAILABLE"} else "failed"
        status(request_path.parent, request, state, "Nav2 Worker 失败", error_code=code,
               error_type=type(exc).__name__, error_message=text, traceback=traceback.format_exc())
        return 2


def _signal(_number, _frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


signal.signal(signal.SIGTERM, _signal); signal.signal(signal.SIGINT, _signal)
if __name__ == "__main__": raise SystemExit(main())
