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
    {"type":"event","event":{explorer event dict}}   (realtime, graph-augmented)
    {"type":"worker_status","status":{...}}
    {"type":"error","message":...}
    {"result": {...}}  (SessionResult printed by run_semantic_exploration)

The worker reuses the CLI wiring in ``run_semantic_exploration.py`` via an
``event_hook`` that forwards every explorer event to stdout in realtime.
"""

from __future__ import annotations

import json
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
}

_EMIT_LOCK = threading.Lock()


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


def make_event_hook() -> Any:
    def hook(event: dict[str, Any], holder: dict[str, Any]) -> dict[str, Any]:
        _STATE["holder"] = holder
        explorer = holder.get("explorer")
        if explorer is not None:
            _STATE["session_id"] = explorer.session_id
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


def build_argv(params: dict[str, Any]) -> list[str]:
    argv = [
        "--target", str(params.get("target") or ""),
        "--backend", str(params.get("backend") or "go2w_experimental"),
        "--reasoner", str(params.get("reasoner") or "semantic_navigation"),
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
    if params.get("rtabmap"):
        argv.append("--rtabmap")
    for key, flag in (
        ("llm_model", "--llm-model"),
        ("detector", "--detector"),
        ("spool_root", "--spool-root"),
        ("odom_topic", "--odom-topic"),
        ("mock_scenario", "--mock-scenario"),
        ("output", "--output"),
        ("session_dir", "--session-dir"),
        ("replay", "--replay"),
        ("record_video", "--record-video"),
    ):
        value = params.get(key)
        if value:
            if key in {"spool_root", "output", "session_dir", "replay"}:
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
        # Keep imports inside the guarded worker boundary.  ROS/system Python
        # environments can miss an optional package (for example OpenAI); a
        # failed import must still emit an error + terminal session_result so
        # the WebUI cannot restore a phantom STARTING search after refresh.
        _emit_stage("WORKER_READY")
        import run_semantic_exploration as rse

        argv = build_argv(params)
        parser = rse.build_parser()
        args = parser.parse_args(argv)
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
    if args.reasoner == "semantic":
        args.reasoner = "semantic_navigation"
    _emit_stage("LOAD_PIPELINE")
    hook = make_event_hook()
    try:
        if args.replay:
            rc = rse.run_replay(args, hook)
        elif args.backend == "go2w_experimental":
            _emit_stage("START_EXPLORER")
            rc = rse.run_go2w(args, hook)
            _emit_stage("RUNNING", state="running")
        else:
            rc = rse._run_offline(args, hook)
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
        _STATE["thread"] = thread
        _STATE["finish_reason"] = ""
        thread.start()
    elif cmd == "pause":
        explorer = _explorer()
        if explorer is not None:
            explorer.request_pause()
    elif cmd == "resume":
        explorer = _explorer()
        if explorer is not None:
            explorer.request_resume()
    elif cmd == "stop":
        explorer = _explorer()
        if explorer is not None:
            explorer.request_stop()
    elif cmd == "estop":
        explorer = _explorer()
        if explorer is not None:
            explorer.request_stop()
    elif cmd == "status":
        emit(
            {
                "type": "worker_status",
                "status": {
                    "state": "running" if _running() else "idle",
                    "session_id": _STATE.get("session_id"),
                    "finish_reason": _STATE.get("finish_reason"),
                },
            }
        )
    elif cmd == "shutdown":
        explorer = _explorer()
        if explorer is not None:
            explorer.request_stop()
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
                    "state": "running",
                    "stage": "RUNNING",
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
