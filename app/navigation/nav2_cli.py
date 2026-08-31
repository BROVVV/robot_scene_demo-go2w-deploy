"""Shared CLI integration for image and video entry points."""
from __future__ import annotations
import argparse, json, time
from pathlib import Path
from .nav2_config import Nav2Settings
from .nav2_gateway import Nav2Gateway
from .nav2_models import Nav2Mode, Nav2Pose
from .nav2_request_builder import make_request

def add_nav2_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--enable-nav2", action="store_true")
    parser.add_argument("--nav2-mode", choices=[m.value for m in Nav2Mode], default="disabled")
    parser.add_argument("--nav2-goal-x", type=float); parser.add_argument("--nav2-goal-y", type=float)
    parser.add_argument("--nav2-goal-yaw", type=float, default=0.0); parser.add_argument("--nav2-goal-frame", default="map")
    parser.add_argument("--nav2-start-x", type=float); parser.add_argument("--nav2-start-y", type=float)
    parser.add_argument("--nav2-start-yaw", type=float, default=0.0)
    parser.add_argument("--nav2-use-current-start", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--nav2-planner-id"); parser.add_argument("--nav2-controller-id")
    parser.add_argument("--nav2-behavior-tree"); parser.add_argument("--nav2-planning-timeout", type=float)
    parser.add_argument("--nav2-execution-timeout", type=float); parser.add_argument("--nav2-allow-execute", action="store_true")
    parser.add_argument("--nav2-safety-confirmed", action="store_true"); parser.add_argument("--nav2-footprint-confirmed", action="store_true")
    parser.add_argument("--nav2-estop-confirmed", action="store_true"); parser.add_argument("--nav2-candidate-pose-json")
    parser.add_argument("--nav2-capability-gate-json")
    parser.add_argument("--nav2-wait", action="store_true")

def run_nav2_from_args(args, *, task_context=None):
    mode = Nav2Mode(args.nav2_mode)
    if not args.enable_nav2 and mode == Nav2Mode.DISABLED: return None
    if mode == Nav2Mode.VISUAL_PREVIEW:
        print("[Navigation2] visual_preview 只显示视觉规划，不请求 ROS2/Nav2。")
        return None
    settings = Nav2Settings.from_env()
    if args.nav2_planner_id: object.__setattr__(settings, "planner_id", args.nav2_planner_id)
    if args.nav2_controller_id: object.__setattr__(settings, "controller_id", args.nav2_controller_id)
    if args.nav2_behavior_tree: object.__setattr__(settings, "behavior_tree", args.nav2_behavior_tree)
    if args.nav2_planning_timeout: object.__setattr__(settings, "planning_timeout_seconds", args.nav2_planning_timeout)
    if args.nav2_execution_timeout: object.__setattr__(settings, "execution_timeout_seconds", args.nav2_execution_timeout)
    if args.nav2_candidate_pose_json:
        goal = Nav2Pose.from_dict(json.loads(Path(args.nav2_candidate_pose_json).read_text(encoding="utf-8")))
    else:
        if args.nav2_goal_x is None or args.nav2_goal_y is None:
            raise ValueError("Nav2 模式需要 --nav2-goal-x 与 --nav2-goal-y；不能从像素坐标伪造目标")
        goal = Nav2Pose(frame_id=args.nav2_goal_frame, x=args.nav2_goal_x, y=args.nav2_goal_y,
                        yaw_rad=args.nav2_goal_yaw, source="manual_cli",
                        provenance={"type":"user_input","details":"CLI map goal"})
    start = None
    if not args.nav2_use_current_start:
        if args.nav2_start_x is None or args.nav2_start_y is None: raise ValueError("显式起点需要 X/Y")
        start = Nav2Pose(frame_id=args.nav2_goal_frame, x=args.nav2_start_x, y=args.nav2_start_y,
                         yaw_rad=args.nav2_start_yaw, source="manual_cli", provenance={"type":"user_input"})
    gate_result = None
    if args.nav2_capability_gate_json:
        gate_result = json.loads(Path(args.nav2_capability_gate_json).read_text(encoding="utf-8"))
    request = make_request(mode=mode, goal=goal, start=start, use_current_start=args.nav2_use_current_start,
        settings=settings, allow_execute=args.nav2_allow_execute, operator_confirmed=args.nav2_safety_confirmed,
        footprint_confirmed=args.nav2_footprint_confirmed, estop_confirmed=args.nav2_estop_confirmed,
        capability_gate_result=gate_result)
    request.task_context = task_context or {}
    gateway=Nav2Gateway(settings)
    handle=gateway.execute(request) if mode==Nav2Mode.EXECUTE else gateway.plan(request)
    print(f"[Navigation2] request_id={handle.request_id} output={handle.job_dir}")
    if args.nav2_wait:
        deadline=time.monotonic()+max(request.planning_timeout_sec, request.execution_timeout_sec)+10
        while time.monotonic()<deadline:
            state=gateway.get_status(handle.request_id)
            if state.state.terminal or state.state.value=="planned":
                print(f"[Navigation2] state={state.state.value} message={state.message_zh}")
                break
            time.sleep(.5)
    return handle
