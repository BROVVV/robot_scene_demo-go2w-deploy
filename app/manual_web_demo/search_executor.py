"""Search executors: where the autonomous search actually runs.

Two implementations of one protocol:

* ``SubprocessSearchExecutor`` – spawns ``scripts/go2w/autonomous_search_worker.py``
  (JSONL stdin/stdout IPC, plan book §12, §86).  Used for the real Go2-W
  backend: the worker runs under ``/usr/bin/python3`` with the ROS2
  environment while the FastAPI process stays in the Conda environment.
* ``InProcessMockExecutor`` – runs the full AutonomousExplorer in a daemon
  thread with the scripted mock scene + mock backend.  Used by CI, tests and
  offline frontend development (plan book §61, §103); no ROS needed.

Both produce the same message stream so the ``SearchSessionService`` is
executor-agnostic:

``{"type": "event", "event": {explorer dict event}}``
``{"type": "session_result", "result": {...}}``
``{"type": "worker_status", "state": ..., ...}``
``{"type": "error", "message": ...}``
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Protocol

_PROJECT_ROOT = Path(__file__).resolve().parents[2]


class SearchExecutor(Protocol):
    """Executor contract consumed by SearchSessionService."""

    def set_on_message(self, callback: Callable[[dict[str, Any]], None]) -> None: ...
    def start(self, params: dict[str, Any]) -> None: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def stop(self) -> None: ...
    def estop(self) -> None: ...
    def status(self) -> dict[str, Any]: ...
    def alive(self) -> bool: ...
    def shutdown(self) -> None: ...


# --------------------------------------------------------------------------- #
# Subprocess worker (real robot / isolated process)                            #
# --------------------------------------------------------------------------- #
class SubprocessSearchExecutor:
    """Owns the autonomous search worker subprocess (JSONL IPC)."""

    _ESTOP_GRACE_SEC = 1.0
    _TERM_GRACE_SEC = 1.0

    # Worker interpreter resolution.  The search worker must import
    # ``run_semantic_exploration`` AND talk to ROS (rclpy) on real hardware.
    # Neither dependency lives in a single interpreter on every machine:
    #   * /usr/bin/python3 (ROS system py) has rclpy but may lack openai;
    #   * the Web conda python has openai but may lack rclpy.
    # We therefore probe candidates and pick the first that satisfies both,
    # falling back to the ROS interpreter so a real robot still gets rclpy.
    # Operators can force a choice with GO2W_WORKER_PYTHON.
    def __init__(
        self,
        *,
        cmd: tuple[str, ...] | None = None,
        cwd: str | Path | None = None,
        log_path: str | Path | None = None,
    ) -> None:
        if cmd is not None:
            self._cmd = list(cmd)
        else:
            self._cmd = [
                _resolve_worker_python(),
                "scripts/go2w/autonomous_search_worker.py",
            ]
        self._cwd = str(cwd or _PROJECT_ROOT)
        self._log_path = Path(log_path) if log_path else None
        self._on_message: Callable[[dict[str, Any]], None] | None = None
        self._proc: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {"state": "stopped", "session_id": None}

    # -- lifecycle ----------------------------------------------------- #
    def set_on_message(self, callback: Callable[[dict[str, Any]], None]) -> None:
        self._on_message = callback

    def start(self, params: dict[str, Any]) -> None:
        if self.alive():
            return
        self._stop.clear()
        stderr_target = None
        if self._log_path is not None:
            self._log_path.parent.mkdir(parents=True, exist_ok=True)
            stderr_target = self._log_path.open("ab")
        env = dict(os.environ)
        env["PYTHONUNBUFFERED"] = "1"
        self._proc = subprocess.Popen(
            self._cmd,
            cwd=self._cwd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=stderr_target,
            env=env,
            text=True,
            bufsize=1,
        )
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="search-worker-reader"
        )
        self._reader.start()
        self._send({"cmd": "start", "params": params})

    def pause(self) -> None:
        self._send({"cmd": "pause"})

    def resume(self) -> None:
        self._send({"cmd": "resume"})

    def stop(self) -> None:
        self._send({"cmd": "stop"})

    def estop(self) -> None:
        self._send({"cmd": "estop"})
        proc = self._proc
        if proc is not None and proc.poll() is None:
            # The worker can be blocked inside a third-party vision/LLM call
            # and therefore unable to consume stdin.  ESTOP must remain
            # bounded even in that state: allow a brief cooperative window,
            # then terminate this exact process without blocking the API
            # event loop or touching a subsequently-created worker.
            threading.Thread(
                target=self._terminate_estopped_process,
                args=(proc,),
                daemon=True,
                name="search-worker-estop",
            ).start()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    def alive(self) -> bool:
        proc = self._proc
        return proc is not None and proc.poll() is None

    def shutdown(self) -> None:
        self._stop.set()
        self._send({"cmd": "shutdown"})
        proc = self._proc
        if proc is not None:
            try:
                proc.stdin.close()
            except OSError:
                pass
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._proc = None
        with self._lock:
            self._status["state"] = "stopped"

    def _terminate_estopped_process(
        self, proc: subprocess.Popen[str]
    ) -> None:
        try:
            proc.wait(timeout=self._ESTOP_GRACE_SEC)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=self._TERM_GRACE_SEC)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=self._TERM_GRACE_SEC)
                except subprocess.TimeoutExpired:
                    pass
        with self._lock:
            if self._proc is proc:
                self._status["state"] = "stopped"

    # -- internals ------------------------------------------------------ #
    def _send(self, command: dict[str, Any]) -> None:
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.stdin.write(
                json.dumps(command, ensure_ascii=False, separators=(",", ":")) + "\n"
            )
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            with self._lock:
                self._status["state"] = "broken"

    def _read_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            if self._stop.is_set():
                break
            line = line.rstrip("\n")
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(payload, dict):
                continue
            self._handle(payload)

    def _handle(self, payload: dict[str, Any]) -> None:
        msg_type = str(payload.get("type") or "")
        if msg_type == "event" and isinstance(payload.get("event"), dict):
            self._notify({"type": "event", "event": payload["event"]})
        elif msg_type == "session_result":
            with self._lock:
                self._status["state"] = "finished"
            self._notify(payload)
        elif msg_type == "worker_status":
            with self._lock:
                self._status.update(payload.get("status") or {})
            self._notify(payload)
        elif msg_type == "error":
            with self._lock:
                self._status["state"] = "error"
                self._status["last_error"] = payload.get("message")
            self._notify(payload)
        elif msg_type == "ready":
            with self._lock:
                self._status["state"] = "ready"
            self._notify(payload)
        elif "result" in payload:
            # SessionResult printed to stdout by run_semantic_exploration.
            self._notify({"type": "session_result", "result": payload["result"]})
        elif "ready" in payload and "checks" in payload:
            # Experiment-readiness probe printed by run_semantic_exploration;
            # informational only.
            with self._lock:
                self._status["readiness"] = payload
        elif "status" in payload:
            # run_semantic_exploration prints {"status": ...} lines to stdout;
            # blockers surface as errors so the WebUI can explain, everything
            # else (e.g. dry_run_motion notices) is informational.
            status_value = str(payload.get("status") or "")
            if status_value in {"blocked", "failed"}:
                self._notify(
                    {
                        "type": "error",
                        "message": str(
                            payload.get("reason") or payload.get("error") or payload
                        ),
                    }
                )
            else:
                with self._lock:
                    self._status["last_status_line"] = payload
        elif "reason" in payload:
            self._notify(
                {"type": "error", "message": str(payload.get("reason") or payload)}
            )

    def _notify(self, message: dict[str, Any]) -> None:
        callback = self._on_message
        if callback is None:
            return
        try:
            callback(message)
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# In-process mock executor (tests / offline frontend)                          #
# --------------------------------------------------------------------------- #
class InProcessMockExecutor:
    """Runs the full AutonomousExplorer in-process with scripted vision and
    the mock backend.  Deterministic, no ROS, no subprocess."""

    def __init__(self, *, scenario: str = "anchor_then_target",
                 mock_target_after: int = 3,
                 confirm_after_seen: int = 1,
                 outcome_sequence: list[str] | None = None,
                 backend_latency_sec: float = 0.0,
                 scene_steps: list[dict[str, Any]] | None = None) -> None:
        self.scenario = scenario
        self.mock_target_after = max(1, int(mock_target_after))
        self.confirm_after_seen = max(1, int(confirm_after_seen))
        self.outcome_sequence = list(outcome_sequence or [])
        self.backend_latency_sec = max(0.0, float(backend_latency_sec))
        self.scene_steps = list(scene_steps or [])
        self._on_message: Callable[[dict[str, Any]], None] | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {"state": "stopped", "session_id": None}
        self._holder: dict[str, Any] = {"explorer": None}
        self._envelope: dict[str, Any] = {}

    # -- lifecycle ----------------------------------------------------- #
    def set_on_message(self, callback: Callable[[dict[str, Any]], None]) -> None:
        self._on_message = callback

    def start(self, params: dict[str, Any]) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        with self._lock:
            self._status["state"] = "starting"
        self._thread = threading.Thread(
            target=self._run,
            args=(dict(params),),
            daemon=True,
            name="in-process-search",
        )
        self._thread.start()

    def pause(self) -> None:
        explorer = self._holder.get("explorer")
        if explorer is not None:
            explorer.request_pause()

    def resume(self) -> None:
        explorer = self._holder.get("explorer")
        if explorer is not None:
            explorer.request_resume()

    def stop(self) -> None:
        explorer = self._holder.get("explorer")
        if explorer is not None:
            explorer.request_stop()

    def estop(self) -> None:
        explorer = self._holder.get("explorer")
        if explorer is not None:
            explorer.request_stop()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def shutdown(self) -> None:
        self.stop()
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=5.0)
        self._thread = None
        with self._lock:
            self._status["state"] = "stopped"

    # -- internals ------------------------------------------------------ #
    def _run(self, params: dict[str, Any]) -> None:
        try:
            # Preserve the asynchronous executor contract even when a tiny
            # deterministic mock scene can finish in a single scheduler tick.
            time.sleep(0.01)
            from app.live_robot.autonomous_explorer import AutonomousExplorer
            from app.live_robot.mock_observation_scene import (
                scenario_anchor_then_target,
                scenario_no_target,
                scenario_target_appears_after,
            )
            from app.navigation.backend_factory import MockBackendConfig, create_backend
            from app.navigation.exploration_config import load_exploration_policy
            from app.navigation.exploration_graph import ExplorationGraph
            from app.task_understanding.search_task_context import SearchTaskContext

            scenario = self.scenario
            if self.scene_steps:
                from app.live_robot.mock_observation_scene import (
                    MockObservationScene,
                    MockSceneStep,
                )

                scene = MockObservationScene(
                    scenes=[MockSceneStep(**step) for step in self.scene_steps]
                )
            elif scenario == "target_appears_after_n":
                scene = scenario_target_appears_after(self.mock_target_after)
            elif scenario == "semantic_topology":
                from app.live_robot.mock_observation_scene import scenario_semantic_topology

                scene = scenario_semantic_topology()
            elif scenario == "green_bin":
                from app.live_robot.mock_observation_scene import scenario_green_bin

                scene = scenario_green_bin()
            elif scenario == "no_target":
                scene = scenario_no_target()
            else:
                scene = scenario_anchor_then_target()
            scene.confirm_after_seen = self.confirm_after_seen

            policy = load_exploration_policy(
                str(_PROJECT_ROOT / "configs" / "exploration" / "default.yaml"),
                overrides={
                    "exploration": {
                        "budget": {
                            "max_search_seconds": float(params.get("max_seconds") or 300.0),
                            "max_planning_cycles": int(
                                params.get("max_planning_cycles") or 30
                            ),
                            "max_motion_steps": int(
                                params.get("max_motion_steps") or 30
                            ),
                        },
                    },
                },
            )
            backend = create_backend(
                "mock_metric" if params.get("backend") == "mock_metric" else "mock",
                outcome_sequence=self.outcome_sequence,
                config=MockBackendConfig(latency_sec=self.backend_latency_sec),
            )
            session_id = str(params.get("session_id") or time.strftime("explore_mock_%Y%m%d_%H%M%S"))
            graph = ExplorationGraph(session_id=session_id)
            task_context = SearchTaskContext.from_dict(params["task_context"]) if isinstance(params.get("task_context"), dict) else SearchTaskContext.mock_fallback(str(params.get("task_text") or params.get("target") or "测试目标"))
            self._envelope = {
                "session_id": session_id,
                "task_id": task_context.task_id,
                "executor_id": params.get("executor_id"),
                "worker_generation": params.get("worker_generation"),
            }

            events: list[dict[str, Any]] = []

            def on_event(event: dict[str, Any]) -> None:
                explorer = self._holder.get("explorer")
                if explorer is not None and event.get("event") in (
                    "observation", "memory_update", "navigation_result",
                ):
                    event = {**event, "graph": explorer.graph.to_dict()}
                events.append(event)
                self._notify({"type": "event", "event": event})

            explorer = AutonomousExplorer(
                target=task_context.canonical_target,
                task_context=task_context,
                observer=scene.observer(),
                matcher=scene.matcher(),
                verifier=scene.verifier(),
                backend=backend,
                policy=policy,
                graph=graph,
                negative_target_key=task_context.canonical_target,
                finish_on_visual_confirmation=bool(
                    params.get("finish_on_visual_confirmation", True)
                ),
                turn_only=bool(params.get("turn_only", False)),
                session_id=graph.session_id,
                executor_id=str(params.get("executor_id") or ""),
                worker_generation=params.get("worker_generation"),
                on_event=on_event,
            )
            self._holder["explorer"] = explorer
            with self._lock:
                self._status["state"] = "running"
                self._status["session_id"] = explorer.session_id
            self._notify(
                {"type": "worker_status", "status": {
                    "state": "running", "session_id": explorer.session_id,
                }}
            )
            result = explorer.run()
            _write_mock_artifacts(params, explorer, result, events)
            self._notify({"type": "session_result", "result": result.to_dict()})
            with self._lock:
                self._status["state"] = "finished"
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._status["state"] = "error"
                self._status["last_error"] = str(exc)
            self._notify({"type": "error", "message": f"{type(exc).__name__}: {exc}"})

    def _notify(self, message: dict[str, Any]) -> None:
        callback = self._on_message
        if callback is None:
            return
        try:
            callback({**self._envelope, **message})
        except Exception:  # noqa: BLE001
            pass


def _write_mock_artifacts(params: dict[str, Any], explorer: Any, result: Any,
                          events: list[dict[str, Any]]) -> None:
    """Persist session artifacts (plan book §56) for the in-process mock path,
    mirroring what the subprocess worker writes via run_semantic_exploration.
    The run directory is the Web session dir (parent of the events.jsonl path
    passed by the service)."""
    try:
        import json

        run_dir = Path(str(params.get("output") or "")).parent
        if not run_dir or str(run_dir) == ".":
            run_dir = Path(str(params.get("session_dir") or "outputs/live_runs")) / explorer.session_id
        run_dir.mkdir(parents=True, exist_ok=True)
        explorer.graph.save(run_dir / "exploration_graph.json")
        (run_dir / "summary.json").write_text(
            json.dumps(result.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        (run_dir / "events.jsonl").write_text(
            "".join(
                json.dumps(event, ensure_ascii=False) + "\n" for event in events
            ),
            encoding="utf-8",
        )
        task_context = getattr(explorer, "task_context", None)
        if task_context is not None:
            (run_dir / "task.json").write_text(
                json.dumps(task_context.to_dict(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        (run_dir / "decisions.jsonl").write_text(
            "".join(json.dumps(item.to_dict(), ensure_ascii=False) + "\n" for item in getattr(explorer, "decision_records", [])),
            encoding="utf-8",
        )
        (run_dir / "semantic_map.json").write_text(
            json.dumps(explorer.semantic_graph.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        (run_dir / "final_state.json").write_text(
            json.dumps(
                {
                    "schema_version": "search_final_state_v1",
                    "session_id": explorer.session_id,
                    "state": explorer.state,
                    "result": result.to_dict(),
                    "task": task_context.to_dict() if task_context is not None else {},
                    "current_place_id": explorer.semantic_graph.current_place_id,
                    "map_revision": explorer.semantic_graph.revision,
                    "last_decision": (
                        explorer.decision_records[-1].to_dict()
                        if getattr(explorer, "decision_records", None) else None
                    ),
                },
                ensure_ascii=False,
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )
    except Exception:  # noqa: BLE001 - artifacts must never crash the search
        pass


def _resolve_worker_python() -> str:
    """Pick the interpreter that can run the real search worker.

    Preference order:
      1. GO2W_WORKER_PYTHON (explicit operator override),
      2. /usr/bin/python3 (ROS system python -> rclpy on the robot),
      3. sys.executable (the Web/Conda python).
    The first candidate that successfully imports ``rclpy`` and
    ``run_semantic_exploration`` is used; if none passes the probe the first
    candidate is used anyway so the failure is reported *with a reason* by the
    worker instead of a silent "cannot find interpreter".
    """
    project = str(_PROJECT_ROOT)
    scripts = str(_PROJECT_ROOT / "scripts" / "go2w")
    probe_code = (
        "import sys; "
        f"sys.path.insert(0, {project!r}); "
        f"sys.path.insert(0, {scripts!r}); "
        "import rclpy, run_semantic_exploration"
    )
    candidates: list[str] = []
    env_py = os.environ.get("GO2W_WORKER_PYTHON")
    if env_py:
        candidates.append(env_py)
    for cand in ("/usr/bin/python3", sys.executable):
        if cand not in candidates:
            candidates.append(cand)

    for cand in candidates:
        if not os.path.isfile(cand):
            continue
        try:
            result = subprocess.run(
                [cand, "-c", probe_code],
                cwd=project,
                capture_output=True,
                text=True,
                timeout=25,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        if result.returncode == 0:
            return cand
    # Nothing passed the probe (e.g. offline dev box without ROS): fall back to
    # the ROS interpreter so real hardware still works and dev boxes get a
    # clear "rclpy unavailable" reason instead of a generic FAILED.
    for cand in candidates:
        if os.path.isfile(cand):
            return cand
    return sys.executable
