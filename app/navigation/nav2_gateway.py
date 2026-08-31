"""Main-process gateway. Real Nav2 always runs in an isolated system-Python worker."""

from __future__ import annotations

import os
import shlex
import subprocess
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

from .nav2_config import Nav2Settings
from .nav2_models import Nav2JobState, Nav2Mode, Nav2Request, Nav2Status
from .nav2_offline_backend import OfflinePreviewBackend
from .nav2_result_adapter import build_webui_payload, write_path_artifacts, write_report
from .nav2_storage import atomic_write_json, atomic_write_text, read_json, read_jsonl


@dataclass
class Nav2JobHandle:
    request_id: str
    job_dir: Path
    pid: int | None = None


class Nav2Gateway:
    def __init__(self, settings: Nav2Settings | None = None, root: str | Path | None = None):
        self.settings = settings or Nav2Settings.from_env()
        self.root = Path(root or Path.cwd()).resolve()
        self.output_dir = (self.root / self.settings.output_dir).resolve()

    def create_job(self, request: Nav2Request) -> Nav2JobHandle:
        request.validate()
        job_dir = self.output_dir / "jobs" / request.request_id
        job_dir.mkdir(parents=True, exist_ok=False)
        atomic_write_json(job_dir / "request.json", request.to_dict())
        initial = Nav2Status(request.request_id, Nav2JobState.CREATED, "pending", False, "任务已创建", updated_at=datetime.now(UTC).isoformat())
        atomic_write_json(job_dir / "status.json", initial.to_dict())
        return Nav2JobHandle(request.request_id, job_dir)

    def plan(self, request: Nav2Request) -> Nav2JobHandle:
        return self._start(request)

    def execute(self, request: Nav2Request) -> Nav2JobHandle:
        if request.mode != Nav2Mode.EXECUTE:
            raise ValueError("execute() 仅接受 execute 模式请求")
        return self._start(request)

    def _start(self, request: Nav2Request) -> Nav2JobHandle:
        handle = self.create_job(request)
        if request.mode == Nav2Mode.DISABLED:
            status = Nav2Status(request.request_id, Nav2JobState.UNAVAILABLE, "disabled", False,
                                "Navigation2 已关闭", error_code="NAV2_DISABLED",
                                updated_at=datetime.now(UTC).isoformat(), finished_at=datetime.now(UTC).isoformat())
            atomic_write_json(handle.job_dir / "status.json", status.to_dict())
        elif request.mode == Nav2Mode.VISUAL_PREVIEW:
            status = Nav2Status(request.request_id, Nav2JobState.UNAVAILABLE, "visual_preview", False,
                                "Visual Preview 只生成视频视觉规划，不请求 ROS2/Nav2",
                                error_code="NAV2_VISUAL_PREVIEW_ONLY",
                                updated_at=datetime.now(UTC).isoformat(), finished_at=datetime.now(UTC).isoformat())
            atomic_write_json(handle.job_dir / "status.json", status.to_dict())
        elif request.mode == Nav2Mode.OFFLINE_PREVIEW:
            fixture = self.root / self.settings.offline_fixture
            OfflinePreviewBackend(fixture).run(request, handle.job_dir)
        else:
            self._spawn_worker(handle)
        self._publish_latest(handle)
        return handle

    def _spawn_worker(self, handle: Nav2JobHandle) -> None:
        setup = Path(self.settings.setup_bash)
        if not setup.is_file():
            standard = Path("/opt/ros") / self.settings.ros_distro / "setup.bash"
            detail = (
                f"ROS2 设置脚本不存在：{setup}。"
                f"请先运行 `bash scripts/install_nav2_humble.sh`；"
                f"安装后应存在 `{standard}`。"
                "若 ROS 安装在其他位置，请把 NAV2_SETUP_BASH 设置为一个完整的 "
                "setup.bash 文件路径，不要填写目录，也不要拼接多个路径。"
            )
            self._unavailable(handle, "NAV2_ROS_SETUP_NOT_FOUND", detail)
            return
        worker = self.root / "scripts/nav2_bridge_worker.py"
        workspace = f"source {shlex.quote(self.settings.workspace_setup)}; " if self.settings.workspace_setup else ""
        command = (f"set -e; source {shlex.quote(str(setup))}; {workspace}"
                   f"exec {shlex.quote(self.settings.system_python)} {shlex.quote(str(worker))} "
                   f"--request {shlex.quote(str(handle.job_dir / 'request.json'))}")
        stdout = (handle.job_dir / "worker.stdout.log").open("ab")
        stderr = (handle.job_dir / "worker.stderr.log").open("ab")
        process = subprocess.Popen(["bash", "-lc", command], cwd=self.root, stdout=stdout, stderr=stderr, start_new_session=True)
        handle.pid = process.pid
        atomic_write_text(handle.job_dir / "worker.pid", f"{process.pid}\n")

    def _unavailable(self, handle: Nav2JobHandle, code: str, message: str) -> None:
        status = Nav2Status(handle.request_id, Nav2JobState.UNAVAILABLE, "nav2_humble", False,
                            message, error_code=code, error_type="runtime_unavailable",
                            error_message=message, updated_at=datetime.now(UTC).isoformat(),
                            finished_at=datetime.now(UTC).isoformat())
        atomic_write_json(handle.job_dir / "status.json", status.to_dict())

    def cancel(self, request_id: str) -> None:
        job = self._job(request_id)
        atomic_write_text(job / "cancel.request", datetime.now(UTC).isoformat() + "\n")

    def get_status(self, request_id: str) -> Nav2Status:
        job = self._job(request_id)
        status = Nav2Status.from_dict(read_json(job / "status.json"))
        request = read_json(job / "request.json")
        plan_only_complete = request.get("mode") == "plan_only" and status.state == Nav2JobState.PLANNED
        pid_file = job / "worker.pid"
        if not status.state.terminal and not plan_only_complete and pid_file.exists():
            pid = int(pid_file.read_text().strip())
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                self._unavailable(Nav2JobHandle(request_id, self._job(request_id)), "NAV2_WORKER_CRASHED", "Nav2 Worker 已退出且未写入终态")
                status = Nav2Status.from_dict(read_json(self._job(request_id) / "status.json"))
        path_file = job / "global_path.json"
        if path_file.exists() and not (job / "instruction_preview.json").exists():
            path = read_json(path_file)
            write_path_artifacts(job, path, semantic_goal=request.get("goal_source", "").startswith("semantic"))
        payload = build_webui_payload(job)
        write_report(job, payload)
        self._publish_latest(Nav2JobHandle(request_id, job))
        return status

    def get_path(self, request_id: str):
        path = self._job(request_id) / "global_path.json"
        return read_json(path) if path.exists() else None

    def get_feedback(self, request_id: str, limit: int = 500):
        return read_jsonl(self._job(request_id) / "feedback.jsonl", limit)

    def get_cmd_vel_trace(self, request_id: str, limit: int = 2000):
        return read_jsonl(self._job(request_id) / "cmd_vel_trace.jsonl", limit)

    def list_jobs(self, limit: int = 20):
        jobs = sorted((self.output_dir / "jobs").glob("*/status.json"), key=lambda p: p.stat().st_mtime, reverse=True)
        return [read_json(path) for path in jobs[:limit]]

    def _job(self, request_id: str) -> Path:
        if "/" in request_id or ".." in request_id:
            raise ValueError("非法 request_id")
        return self.output_dir / "jobs" / request_id

    def _publish_latest(self, handle: Nav2JobHandle) -> None:
        mapping = {"request.json": "nav2_request.json", "status.json": "nav2_status.json",
                   "global_path.json": "nav2_global_path.json", "global_path.csv": "nav2_global_path.csv",
                   "global_path.png": "nav2_global_path.png", "instruction_preview.json": "nav2_instruction_preview.json",
                   "navigation_report.md": "nav2_navigation_report.md", "webui_payload.json": "nav2_webui_payload.json"}
        output = self.root / "outputs"
        for source_name, target_name in mapping.items():
            source = handle.job_dir / source_name
            if source.exists():
                target = output / target_name
                if source.suffix in {".json", ".csv", ".md"}:
                    atomic_write_text(target, source.read_text(encoding="utf-8"))
                else:
                    target.write_bytes(source.read_bytes())
