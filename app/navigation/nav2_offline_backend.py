"""Explicit fixture-backed offline preview."""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

from .nav2_models import Nav2JobState, Nav2Request, Nav2Status
from .nav2_result_adapter import build_webui_payload, normalize_path, write_path_artifacts, write_report
from .nav2_storage import append_jsonl, atomic_write_json, read_json

WARNING = "该路径仅用于 Web UI 和接口测试，不是 Nav2 真实规划结果"


class OfflinePreviewBackend:
    def __init__(self, fixture: str | Path):
        self.fixture = Path(fixture)

    def run(self, request: Nav2Request, job_dir: Path) -> Nav2Status:
        fixture = read_json(self.fixture)
        source = fixture["poses"]
        goal = request.goal_pose
        start = request.start_pose or request.goal_pose.__class__(
            frame_id=request.map_frame, x=0, y=0, yaw_rad=0,
            source="offline_fixture", provenance={"type": "offline_fixture"})
        sx, sy = source[0]["x"], source[0]["y"]
        gx, gy = source[-1]["x"], source[-1]["y"]
        scale_x = (goal.x-start.x)/(gx-sx) if abs(gx-sx) > 1e-9 else 1
        scale_y = (goal.y-start.y)/(gy-sy) if abs(gy-sy) > 1e-9 else scale_x
        poses = [{"x": start.x+(p["x"]-sx)*scale_x,
                  "y": start.y+(p["y"]-sy)*scale_y,
                  "yaw_rad": p.get("yaw_rad", 0)} for p in source]
        path = normalize_path(request.request_id, poses, backend="offline_preview",
                              real=False, frame_id=request.map_frame,
                              planner_id=request.planner_id)
        path["warning"] = WARNING
        write_path_artifacts(job_dir, path, semantic_goal=request.goal_source.startswith("semantic"))
        now = datetime.now(UTC).isoformat()
        for feedback in fixture.get("feedback", []):
            append_jsonl(job_dir / "feedback.jsonl", {**feedback, "timestamp": now, "state": "planned", "simulated": True})
        status = Nav2Status(request_id=request.request_id, state=Nav2JobState.PLANNED,
                            backend="offline_preview", is_real_nav2_path=False,
                            message_zh=f"OFFLINE PREVIEW / 非 Nav2 真实路径 / 不可执行。{WARNING}",
                            path_length_m=path["path_length_m"], progress_ratio=0,
                            updated_at=now, finished_at=now)
        atomic_write_json(job_dir / "status.json", status.to_dict())
        payload = build_webui_payload(job_dir)
        write_report(job_dir, payload)
        return status
