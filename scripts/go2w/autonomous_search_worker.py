#!/usr/bin/env python3
"""Autonomous search worker for the WebUI (plan book §12, §86).

Spawned by the FastAPI process as a separate subprocess so the search chain
(real Go2-W backend: rclpy + /go2w/motion) runs under the ROS2 system Python
while the Web process stays in the Conda environment.

Protocol (stdin, JSONL):
    {"cmd":"start","params":{...}}
    {"cmd":"pause"} / {"cmd":"resume"} / {"cmd":"stop"} / {"cmd":"estop"}
    {"cmd":"status"} / {"cmd":"shutdown"}

Protocol (stdout, JSONL):
    {"type":"ready", ...}
    {"type":"graph_snapshot","revision":N,"graph":{...}}  (once per revision)
    {"type":"event","event":{explorer event dict}}   (realtime, no graph)
    {"type":"worker_status","status":{...}}
    {"type":"error","message":...}
    {"result": {...}}  (SessionResult printed by run_semantic_exploration)

The worker reuses the CLI wiring in ``run_semantic_exploration.py`` via an
``event_hook`` that forwards every explorer event to stdout in realtime.

计划书 §2.1：这张图是 worker 与 Web 之间**唯一**的整图传输。普通
``observation``/``memory_update``/``navigation_result`` 事件不再携带图，
只报 ``graph_revision``。worker 在 ``event_hook`` 里比对
``explorer.semantic_graph.revision``，变化时**先**发一条 ``graph_snapshot``，
再发导致这次变化的事件。图字段的映射只在
``SemanticNavigationGraph.web_snapshot()`` 里做一次。
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
# Some robot deployments retain legacy top-level copies named
# ``run_semantic_exploration.py`` and ``run_autonomous_loop.py``.  Remove and
# reinsert both paths deliberately so the audited scripts/go2w implementation
# is the unqualified-import winner inside this worker process.
for _path in (str(PROJECT_ROOT), str(SCRIPT_DIR)):
    while _path in sys.path:
        sys.path.remove(_path)
sys.path.insert(0, str(SCRIPT_DIR))
sys.path.insert(1, str(PROJECT_ROOT))

_STATE: dict[str, Any] = {
    "holder": None,
    "controlled_explorer": None,
    "worker_state": "idle",
    "worker_stage": "IDLE",
    "stop_requested": False,
    "pause_requested": False,
    "thread": None,
    "session_id": None,
    "task_id": None,
    "executor_id": None,
    "worker_generation": None,
    "finish_reason": "",
    "shutdown": False,
    "current_phase": "IDLE",
    "current_detail": "",
    "phase_started_at": None,
    "last_progress_at": None,
    # 上次已发给 Web 的图 revision；None 表示本会话还没发过（首次必发）。
    "last_graph_revision": None,
}

_EMIT_LOCK = threading.Lock()
_CONTROL_LOCK = threading.RLock()


def emit(message: dict[str, Any]) -> None:
    try:
        with _EMIT_LOCK:
            sys.stdout.write(json.dumps(message, ensure_ascii=False) + "\n")
            sys.stdout.flush()
    except (BrokenPipeError, OSError):
        pass


def _project_path(value: Any) -> str:
    """Resolve a worker-owned file/directory argument under the repository.

    The WebUI normally supplies relative ``outputs/...`` paths for historical
    compatibility.  The worker is also used from service managers and IDEs,
    however, where the inherited cwd is not guaranteed.  Resolving these
    paths at the IPC boundary keeps all artifacts inside this checkout and
    prevents ``/home/runtime``-style accidental paths.
    """
    path = Path(str(value))
    return str(path if path.is_absolute() else PROJECT_ROOT / path)


def emit_graph_snapshot(explorer: Any) -> None:
    """Emit the graph once per revision change (计划书 §2.1).

    Called before forwarding the ordinary event that caused the bump, so the
    Web side always has the graph at least as new as the event it just saw.
    The first call of a session emits the initial (usually revision 0) graph.
    """
    graph = getattr(explorer, "semantic_graph", None)
    if graph is None:
        return
    revision = int(getattr(graph, "revision", 0) or 0)
    if revision == _STATE.get("last_graph_revision"):
        return
    _STATE["last_graph_revision"] = revision
    emit({
        "type": "graph_snapshot",
        "session_id": _STATE.get("session_id") or getattr(explorer, "session_id", None),
        "task_id": _STATE.get("task_id"),
        "executor_id": _STATE.get("executor_id"),
        "worker_generation": _STATE.get("worker_generation"),
        "revision": revision,
        "graph": graph.web_snapshot(),
    })


def make_event_hook() -> Any:
    def hook(event: dict[str, Any], holder: dict[str, Any]) -> dict[str, Any]:
        with _CONTROL_LOCK:
            _STATE["holder"] = holder
            explorer = holder.get("explorer")
            if explorer is not None and explorer is not _STATE.get("controlled_explorer"):
                _STATE["controlled_explorer"] = explorer
                # Commands received during model/ROS startup must take effect
                # before the first observation or navigation goal.
                if _STATE.get("stop_requested"):
                    explorer.request_stop()
                elif _STATE.get("pause_requested"):
                    explorer.request_pause()
        if explorer is not None:
            _STATE["session_id"] = explorer.session_id
            if _STATE.get("worker_state") == "starting":
                _emit_stage("RUNNING", state="running")
            emit_graph_snapshot(explorer)
        if event.get("event") == "session_finish":
            _STATE["finish_reason"] = str(event.get("result") or "")
        now = time.time()
        phase = str(event.get("phase") or event.get("state") or "RUNNING")
        if phase != _STATE.get("current_phase"):
            _STATE["current_phase"] = phase
            _STATE["phase_started_at"] = now
            _STATE["current_detail"] = ""
        if event.get("detail_zh") is not None:
            _STATE["current_detail"] = str(event.get("detail_zh") or "")
        _STATE["last_progress_at"] = now
        emit({
            "type": "event",
            "event": event,
            "session_id": event.get("session_id"),
            "task_id": event.get("task_id"),
            "executor_id": event.get("executor_id"),
            "worker_generation": event.get("worker_generation"),
        })
        return event

    return hook


#: 这些键的 0 在下游是"不限次"，不是"一次都不许"（见
#: ``app/navigation/exploration_config.ExplorationBudget.remaining``）。在有限档里
#: 收到 0 必须报错，否则请求体就能把"3 次"悄悄变成"无限次"。
_LEGACY_AT_LEAST_ONE_KEYS = (
    "max_seconds", "max_planning_cycles", "max_motion_steps", "max_navigation_goals",
    "max_motion_commands",
)
#: 半径的 0 同样是"不限"（CLI：``--max-radius 0 = unlimited``），有限档必须挡住。
_LEGACY_POSITIVE_KEYS = ("max_radius",)
_LEGACY_INT_KEYS = ("max_planning_cycles", "max_motion_steps", "max_navigation_goals",
                    "max_motion_commands")

#: 旧相对运动搜索的运动包线。**只能由启动器按名字选**（见
#: ``GO2W_LEGACY_SEARCH_MOTION_PROFILE``），请求体不能选、不能放宽：
#: ``bounds`` 里每一项都是该档的上限，请求值只会被 ``min`` 到它，
#: 取 0 表示这一档该项不限次。
_LEGACY_MOTION_PROFILES: dict[str, dict[str, Any]] = {
    # 2026-10-14 现场口径：3 次运动、每次前进不超过 0.20 m。
    # 三轮短试验里 0.15 m 请求在 1.5 倍时长补偿下实测 0.1995 m，所以本档的
    # 补偿被压到 ≤1.0（见 ``_forward_duration_scale_for_run``）；请求上限取
    # 用户给的 0.20 m，实测上限同为 0.20 m —— 越界即判该目标失败并停车。
    "supervised_short": {
        "bounds": {
            "max_seconds": 240.0,
            "max_planning_cycles": 3,
            "max_motion_steps": 3,
            "max_turn_deg": 15.0,
            "forward_step_m": 0.20,
            "max_radius": 1.0,
            "max_observed_forward_m": 0.20,
            "max_navigation_goals": 3,
            "max_motion_commands": 3,
        },
        "profile_config": "configs/go2w/high_level_experiment.yaml",
    },
    # 大场地档：不限运动次数，搜索范围限制在距会话起点 5 m，单个前进目标
    # 仍 ≤3 m。3 m 在
    # ``Go2WExperimentalBackend._execute_forward`` 里仍被拆成 ≤0.30 m 的短段，
    # 每段都回一次 LiDAR / 模式 / arm 闸门并单独校验里程计。
    "field_large": {
        "bounds": {
            # No session deadline. Per-action and sensor timeouts stay active.
            "max_seconds": 0.0,
            # 0 = 不限：运动次数由时间预算兜底，不由计数兜底。
            "max_planning_cycles": 0,
            "max_motion_steps": 0,
            "max_turn_deg": 30.0,
            "forward_step_m": 3.0,
            # 会话可连续产生任意多个目标，但不得离起点超过 5 m。
            # 这与下面的单目标 3 m 里程熔断是两层独立约束。
            "max_radius": 5.0,
            "max_observed_forward_m": 3.0,
            "max_navigation_goals": 0,
            "max_motion_commands": 0,
        },
        "profile_config": "configs/go2w/high_level_experiment_large_field.yaml",
    },
}
_LEGACY_PROFILE_ENV = "GO2W_LEGACY_SEARCH_MOTION_PROFILE"
_DEFAULT_LEGACY_PROFILE = "supervised_short"


def legacy_motion_profile(name: str | None = None) -> tuple[str, dict[str, Any]]:
    """Resolve the launcher-selected motion envelope, fail closed on unknown names.

    只有"根本没给"才回落到默认档：空字符串（``--legacy-profile `` 后面没跟值、
    或环境变量被设成空）是配置错误，不是"没选"，必须报错。否则启动器横幅会
    按短程档宣讲、而操作员以为自己选了大场地档 —— 这正是本项目"默认值 ≠
    生效值"那类缺陷。
    """
    source = name if name is not None else os.environ.get(_LEGACY_PROFILE_ENV)
    selected = _DEFAULT_LEGACY_PROFILE if source is None else str(source).strip()
    if not selected:
        raise ValueError("empty legacy search motion profile name")
    profile = _LEGACY_MOTION_PROFILES.get(selected)
    if profile is None:
        raise ValueError(f"unknown legacy search motion profile: {selected!r}")
    return selected, profile


def _bounded_legacy_budget(
    params: dict[str, Any], profile: dict[str, Any],
) -> None:
    """Clamp every legacy budget key to the selected profile; never widen it."""
    for key, cap in profile["bounds"].items():
        requested = params.get(key)
        if requested is None:
            bounded = cap
        else:
            numeric = float(requested)
            if not math.isfinite(numeric) or numeric < 0:
                raise ValueError(f"invalid legacy search budget: {key}={requested!r}")
            if float(cap) <= 0:
                # 本档该项不限次；请求值原样透传（0 也是不限）。
                bounded = numeric
            else:
                if key in _LEGACY_AT_LEAST_ONE_KEYS and numeric < 1:
                    raise ValueError(f"invalid legacy search budget: {key}={requested!r}")
                if key in _LEGACY_POSITIVE_KEYS and numeric <= 0:
                    raise ValueError(f"invalid legacy search budget: {key}={requested!r}")
                bounded = min(numeric, float(cap))
        params[key] = int(bounded) if key in _LEGACY_INT_KEYS else bounded


def build_argv(params: dict[str, Any]) -> list[str]:
    # The restored relative-motion path runs under a launcher-selected motion
    # envelope. Bound it at the worker boundary as well as in the Web request,
    # because this process also accepts JSONL commands directly.
    if params.get("backend") == "go2w_experimental":
        params = dict(params)
        _, profile = legacy_motion_profile()
        _bounded_legacy_budget(params, profile)
        params["profile_config"] = profile["profile_config"]
    if params.get("backend", "nav2") == "nav2" and os.getenv("GO2W_NAV2_SEARCH_PROFILE") == "field_large":
        params = dict(params)
        _bounded_legacy_budget(params, _LEGACY_MOTION_PROFILES["field_large"])
        # This selects only high-level limits. Nav2 keeps all live safety checks.
        params["profile_config"] = _LEGACY_MOTION_PROFILES["field_large"]["profile_config"]
        params["spatial_provider"] = "nav2"
    argv = [
        "--target", str(params.get("target") or ""),
        "--backend", str(params.get("backend") or "nav2"),
        # 计划书 §3-C：reasoner 不再由请求选择，Web 侧的命令固定传现有 CLI 名。
        # 必须与 ``app/manual_web_demo/search_models.SEARCH_REASONER`` 一致
        # （test_webui_search_request_contract 钉住这一点）。
        "--reasoner", "semantic_navigation",
    ]
    if params.get("operator_supervised_experiment") or params.get(
        "enable_autonomous_motion"
    ):
        argv.append("--operator-supervised-experiment")
        # Plan §90: enable_autonomous_motion=true + turn_only=false means the
        # high-level explorer may use short forward relocations.  No hidden
        # semantic_allow_forward gate should keep the robot permanently turning.
        if not params.get("turn_only") and not params.get("dry_run_motion"):
            argv.append("--semantic-allow-forward")
    if params.get("dry_run_motion"):
        argv.append("--dry-run-motion")
    if params.get("turn_only"):
        argv.append("--turn-only")
    if params.get("finish_on_visual_confirmation") is False:
        argv.append("--no-finish-on-visual-confirmation")
    for key, flag in (
        ("max_seconds", "--max-seconds"),
        ("max_planning_cycles", "--max-planning-cycles"),
        ("max_motion_steps", "--max-motion-steps"),
        ("max_turn_deg", "--max-turn-deg"),
        ("forward_step_m", "--forward-step-m"),
        ("max_radius", "--max-radius"),
        ("max_observed_forward_m", "--max-observed-forward-m"),
        ("max_navigation_goals", "--max-navigation-goals"),
        ("max_motion_commands", "--max-motion-commands"),
    ):
        value = params.get(key)
        if value is not None:
            argv += [flag, str(value)]
    if params.get("rgbd_source"):
        argv.append("--rgbd-source")
        if params.get("rgbd_base_url"):
            argv += ["--rgbd-base-url", str(params["rgbd_base_url"])]
    if params.get("spatial_v2"):
        argv.append("--spatial-v2")
    if params.get("spatial_provider"):
        argv += ["--spatial-provider", str(params["spatial_provider"])]
    for key, flag in (
        ("llm_model", "--llm-model"),
        ("detector", "--detector"),
        # 只由 ``_bounded_legacy_budget`` 按启动器选中的档位写进来，请求体不提供。
        ("profile_config", "--profile-config"),
        ("spool_root", "--spool-root"),
        ("odom_topic", "--odom-topic"),
        ("mock_scenario", "--mock-scenario"),
        ("output", "--output"),
        ("session_dir", "--session-dir"),
        ("replay", "--replay"),
        ("record_video", "--record-video"),
        # 计划书 §2.3：三维对象上屏的两个路径由 Web 侧配置显式给出，不在下游
        # 用 glob 或 mtime 去猜"哪份数据属于当前地图"。
        ("map_meta_path", "--map-meta-path"),
        ("object_snapshot_path", "--object-snapshot-path"),
    ):
        value = params.get(key)
        if value:
            if key in {"spool_root", "output", "session_dir", "replay",
                       "map_meta_path", "object_snapshot_path", "profile_config"}:
                value = _project_path(value)
            argv += [flag, str(value)]
    # The WebUI can be launched by systemd, an IDE, or an SSH shell whose
    # current directory is not the repository.  The semantic runner resolves
    # its frame spool through ``Path(...).resolve()``; passing the relative
    # parser default here could therefore become ``/home/runtime`` (or another
    # unrelated directory), producing a misleading permission error only when
    # the real backend starts.  Make the worker boundary deterministic while
    # still honoring an explicit operator/test override above.
    if not params.get("spool_root"):
        argv += ["--spool-root", str(PROJECT_ROOT / "runtime/go2w/spool")]
    if params.get("verify_min_confidence") is not None:
        argv += ["--verify-min-confidence", str(params["verify_min_confidence"])]
    if params.get("mock_target_after") is not None:
        argv += ["--mock-target-after", str(params["mock_target_after"])]
    if params.get("mock_confirm_after_seen") is not None:
        argv += ["--mock-confirm-after-seen", str(params["mock_confirm_after_seen"])]
    if params.get("allow_degraded"):
        argv.append("--allow-degraded")
    if params.get("task_context"):
        argv += ["--task-context-json", json.dumps(params["task_context"], ensure_ascii=False)]
    for key, flag in (("session_id", "--session-id"), ("executor_id", "--executor-id"), ("worker_generation", "--worker-generation")):
        if params.get(key) is not None:
            argv += [flag, str(params[key])]
    return argv


def validate_legacy_worker_params(params: dict[str, Any]) -> None:
    """Fail closed before loading the real runner or sending a motion goal."""
    if params.get("backend") != "go2w_experimental":
        return
    enabled = os.environ.get("GO2W_ALLOW_SUPERVISED_LEGACY_SEARCH", "").lower()
    if enabled not in {"1", "true", "yes", "on"}:
        raise ValueError("legacy supervised search is not enabled by the launcher")
    # 档位名打错必须在这里就炸，不能等到 build_argv 才发现——否则一次拼写错误
    # 会先跑完感知预检再失败。未知名字一律拒绝，不回落默认档。
    legacy_motion_profile()
    if params.get("dry_run_motion"):
        return
    armed = os.environ.get("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", "").lower()
    if armed not in {"1", "true", "yes", "on"} or not (
        params.get("enable_autonomous_motion")
        and params.get("operator_supervised_experiment")
    ):
        raise ValueError("legacy supervised motion requires launcher arm and operator supervision")


_STARTUP_STAGES = [
    "SPAWN_WORKER",
    "WORKER_READY",
    "LOAD_PIPELINE",
    "WAIT_RGBD",
    "WAIT_SPATIAL_PROVIDER",
    "START_EXPLORER",
    "RUNNING",
]


def _emit_stage(stage: str, *, state: str = "starting", error: str | None = None) -> None:
    _STATE["worker_state"] = state
    _STATE["worker_stage"] = stage
    emit({
        "type": "worker_status",
        "session_id": _STATE.get("session_id"),
        "task_id": _STATE.get("task_id"),
        "executor_id": _STATE.get("executor_id"),
        "worker_generation": _STATE.get("worker_generation"),
        "status": {
            "state": state,
            "stage": stage,
            "stage_started_at": time.time(),
            "last_progress_at": time.time(),
            "worker_alive": True,
            "last_error": error,
        },
    })


def run_session(params: dict[str, Any]) -> None:
    _STATE["session_id"] = params.get("session_id")
    task_context = params.get("task_context")
    _STATE["task_id"] = params.get("task_id") or (
        task_context.get("task_id") if isinstance(task_context, dict) else None
    )
    _STATE["executor_id"] = params.get("executor_id")
    _STATE["worker_generation"] = params.get("worker_generation")
    _emit_stage("SPAWN_WORKER")

    try:
        validate_legacy_worker_params(params)
        # Keep imports inside the guarded worker boundary.  ROS/system Python
        # environments can miss an optional package (for example OpenAI); a
        # failed import must still emit an error + terminal session_result so
        # the WebUI cannot restore a phantom STARTING search after refresh.
        _emit_stage("WORKER_READY")
        import run_semantic_exploration as rse

        argv = build_argv(params)
        parser = rse.build_parser()
        args = parser.parse_args(argv)
        # This worker invokes ``run_go2w`` directly instead of rse.main(), so
        # apply the one normalization normally performed by main(). Without
        # it ``--no-finish-on-visual-confirmation`` merely sets its auxiliary
        # flag while the parser's legacy default remains True.
        if args.no_finish_on_visual_confirmation:
            args.finish_on_visual_confirmation = False
    except SystemExit as exc:
        _emit_stage("FAILED", state="failed", error=f"bad start params: {exc}")
        emit({
            "type": "error",
            "session_id": _STATE.get("session_id"),
            "task_id": _STATE.get("task_id"),
            "executor_id": _STATE.get("executor_id"),
            "worker_generation": _STATE.get("worker_generation"),
            "message": f"bad start params: {exc}",
        })
        emit({
            "type": "session_result",
            "session_id": _STATE.get("session_id"),
            "task_id": _STATE.get("task_id"),
            "executor_id": _STATE.get("executor_id"),
            "worker_generation": _STATE.get("worker_generation"),
            "result": {
                "exit_code": 2,
                "session_id": _STATE.get("session_id"),
                "finish_reason": "FAILED",
                "error": f"bad start params: {exc}",
                "reason": f"bad start params: {exc}",
            },
        })
        return
    except Exception as exc:  # noqa: BLE001 - dependency/import failures are terminal
        envelope = {
            "type": "error",
            "session_id": _STATE.get("session_id"),
            "task_id": _STATE.get("task_id"),
            "executor_id": _STATE.get("executor_id"),
            "worker_generation": _STATE.get("worker_generation"),
        }
        _emit_stage("FAILED", state="failed", error=f"{type(exc).__name__}: {exc}")
        emit({**envelope, "message": f"{type(exc).__name__}: {exc}"})
        emit({
            **envelope,
            "type": "session_result",
            "result": {
                "exit_code": 4,
                "session_id": _STATE.get("session_id"),
                "finish_reason": "FAILED",
                "error": f"{type(exc).__name__}: {exc}",
                "reason": f"{type(exc).__name__}: {exc}",
            },
        })
        return
    # 这里曾经把 ``--reasoner semantic`` 归一成 ``semantic_navigation``：worker
    # 绕过 ``rse.main()`` 直接调 ``run_go2w``，拿不到那边的同名归一。§3-C 之后
    # ``build_argv`` 固定传 CLI 名，旧别名在这条路上已经没有产出点。
    _emit_stage("LOAD_PIPELINE")
    hook = make_event_hook()
    try:
        if args.replay:
            rc = rse.run_replay(args, hook)
        elif args.backend in {"nav2", "go2w_experimental"}:
            _emit_stage("START_EXPLORER")
            rc = rse.run_go2w(args, hook)
        elif args.backend in {"mock", "mock_metric"}:
            rc = rse._run_offline(args, hook)
        else:
            # An unknown backend must fail explicitly; never substitute an
            # offline search or another real motion path.
            raise ValueError(
                f"unsupported autonomous search backend: {args.backend!r}"
            )
    except Exception as exc:  # noqa: BLE001
        import traceback

        _STATE["last_error"] = f"{type(exc).__name__}: {exc}"
        envelope = {
            "type": "error",
            "session_id": _STATE.get("session_id"),
            "task_id": _STATE.get("task_id"),
            "executor_id": _STATE.get("executor_id"),
            "worker_generation": _STATE.get("worker_generation"),
        }
        emit({**envelope, "message": f"{type(exc).__name__}: {exc}"})
        emit({**envelope, "message": traceback.format_exc()[-2000:]})
        rc = 4
    # 收尾兜底：若最后一次 revision 变化（例如目标确认）没有跟任何普通事件，
    # 这里补发一次，保证 Web 拿到的最终图与 explorer 收工时一致。
    if _STATE.get("holder"):
        explorer = _STATE["holder"].get("explorer")
        if explorer is not None:
            emit_graph_snapshot(explorer)
    if rc != 0 and not _STATE.get("finish_reason"):
        # The runner can reject a real session during hardware preflight
        # before an explorer emits session_finish.  Preserve that failure in
        # the protocol so the WebUI does not turn it into a blank FINISHED
        # session.
        _STATE["finish_reason"] = "FAILED"
    final_result = {
        "exit_code": rc,
        "session_id": _STATE.get("session_id"),
        "finish_reason": _STATE.get("finish_reason") or "",
    }
    if _STATE.get("last_error"):
        final_result["error"] = _STATE["last_error"]
        final_result["reason"] = _STATE["last_error"]
    emit(
        {
            "type": "session_result",
            "session_id": _STATE.get("session_id"),
            "task_id": _STATE.get("task_id"),
            "executor_id": _STATE.get("executor_id"),
            "worker_generation": _STATE.get("worker_generation"),
            "result": final_result,
        }
    )


def handle_command(command: dict[str, Any]) -> None:
    cmd = str(command.get("cmd") or "")
    if cmd == "start":
        if _STATE["thread"] is not None and _STATE["thread"].is_alive():
            emit({"type": "error", "message": "search already running"})
            return
        thread = threading.Thread(
            target=run_session, args=(dict(command.get("params") or {}),),
            daemon=True, name="search-session",
        )
        with _CONTROL_LOCK:
            _STATE.update(holder=None, controlled_explorer=None, thread=thread, finish_reason="",
                          last_error=None, last_graph_revision=None,
                          stop_requested=False, pause_requested=False,
                          worker_state="starting", worker_stage="SPAWN_WORKER",
                          current_phase="STARTING", current_detail="",
                          phase_started_at=time.time(), last_progress_at=time.time())
        thread.start()
    elif cmd in {"pause", "resume", "stop", "estop"}:
        with _CONTROL_LOCK:
            explorer = _explorer()
            if cmd in {"stop", "estop"}:
                _STATE["stop_requested"] = True
                _STATE["pause_requested"] = False
                if explorer is not None:
                    explorer.request_stop()
            elif not _STATE.get("stop_requested"):
                _STATE["pause_requested"] = cmd == "pause"
                if explorer is not None:
                    if cmd == "pause":
                        explorer.request_pause()
                    else:
                        explorer.request_resume()
    elif cmd == "status":
        emit(
            {
                "type": "worker_status",
                "status": {
                    "state": _STATE.get("worker_state", "starting") if _running() else "idle",
                    "session_id": _STATE.get("session_id"),
                    "finish_reason": _STATE.get("finish_reason"),
                },
            }
        )
    elif cmd == "shutdown":
        handle_command({"cmd": "stop"})
        _STATE["shutdown"] = True


def _explorer():
    holder = _STATE.get("holder")
    return holder.get("explorer") if holder else None


def _running() -> bool:
    thread = _STATE.get("thread")
    return thread is not None and thread.is_alive()


def _emit_heartbeat() -> None:
    while not _STATE.get("shutdown"):
        if _running():
            now = time.time()
            emit({
                "type": "worker_status",
                "session_id": _STATE.get("session_id"),
                "task_id": _STATE.get("task_id"),
                "executor_id": _STATE.get("executor_id"),
                "worker_generation": _STATE.get("worker_generation"),
                "status": {
                    "state": _STATE.get("worker_state", "starting"),
                    "stage": _STATE.get("worker_stage", "SPAWN_WORKER"),
                    "phase": _STATE.get("current_phase") or "RUNNING",
                    "phase_detail": _STATE.get("current_detail") or "",
                    "phase_started_at": _STATE.get("phase_started_at"),
                    "phase_elapsed_seconds": max(
                        0.0, now - float(_STATE.get("phase_started_at") or now)
                    ),
                    "last_progress_at": _STATE.get("last_progress_at"),
                    "worker_alive": True,
                },
            })
        time.sleep(2.0)


def main() -> int:
    emit({"type": "ready", "pid": str(Path("/proc/self/stat").read_text().split()[0])
          if Path("/proc/self/stat").is_file() else "unknown"})
    threading.Thread(
        target=_emit_heartbeat,
        daemon=True,
        name="search-worker-heartbeat",
    ).start()
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            command = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(command, dict):
            continue
        handle_command(command)
        if _STATE.get("shutdown"):
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
