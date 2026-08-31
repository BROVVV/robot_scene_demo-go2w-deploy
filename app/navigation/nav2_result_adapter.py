"""Artifact and Web UI payload generation shared by backends."""

from __future__ import annotations

import csv
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .nav2_instruction_preview import build_instruction_preview
from .nav2_path_utils import compute_cumulative_distances, compute_path_length
from .nav2_storage import atomic_write_json, atomic_write_text, read_json, read_jsonl


def normalize_path(request_id: str, poses: list[dict[str, Any]], *, backend: str,
                   real: bool, frame_id: str = "map", planning_time_sec: float = 0.0,
                   planner_id: str = "GridBased") -> dict[str, Any]:
    cumulative = compute_cumulative_distances(poses)
    normalized = []
    for index, (pose, distance) in enumerate(zip(poses, cumulative)):
        normalized.append({"index": index, "x": float(pose["x"]), "y": float(pose["y"]),
                           "yaw_rad": float(pose.get("yaw_rad", 0.0)),
                           "cumulative_distance_m": distance})
    return {
        "schema_version": "1.0", "request_id": request_id, "backend": backend,
        "is_real_nav2_path": real, "frame_id": frame_id,
        "planning_time_sec": planning_time_sec, "planner_id": planner_id,
        "path_length_m": compute_path_length(normalized), "pose_count": len(normalized),
        "start_pose": normalized[0] if normalized else None,
        "goal_pose": normalized[-1] if normalized else None,
        "poses": normalized, "created_at": datetime.now(UTC).isoformat(),
    }


def write_path_artifacts(job_dir: Path, path: dict[str, Any], *, semantic_goal: bool = False) -> None:
    atomic_write_json(job_dir / "global_path.json", path)
    rows = path.get("poses", [])
    csv_path = job_dir / "global_path.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = csv_path.with_name(f".{csv_path.name}.tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["index", "x", "y", "yaw_rad", "cumulative_distance_m"])
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in writer.fieldnames} for row in rows)
    temporary.replace(csv_path)
    preview = build_instruction_preview(rows, semantic_goal=semantic_goal)
    atomic_write_json(job_dir / "instruction_preview.json", preview)
    _write_path_png(job_dir / "global_path.png", rows)


def _write_path_png(target: Path, rows: list[dict[str, Any]]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figure, axis = plt.subplots(figsize=(7, 5))
        if rows:
            xs, ys = [p["x"] for p in rows], [p["y"] for p in rows]
            axis.plot(xs, ys, "-o", markersize=2, label="Planned Path")
            axis.scatter(xs[0], ys[0], c="green", s=80, label="Start")
            axis.scatter(xs[-1], ys[-1], c="red", marker="*", s=120, label="Goal")
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_xlabel("X (m)"); axis.set_ylabel("Y (m)"); axis.grid(True); axis.legend()
        figure.tight_layout(); figure.savefig(target, dpi=130); plt.close(figure)
    except ImportError:
        pass


def build_webui_payload(job_dir: Path) -> dict[str, Any]:
    def optional(name: str, default: Any) -> Any:
        path = job_dir / name
        return read_json(path) if path.exists() else default
    status = optional("status.json", {})
    trace = read_jsonl(job_dir / "cmd_vel_trace.jsonl", 200)
    payload = {
        "schema_version": "1.0", "request_id": status.get("request_id"),
        "status": status, "request": optional("request.json", {}),
        "path": optional("global_path.json", {}),
        "instruction_preview": optional("instruction_preview.json", {}),
        "latest_feedback": (read_jsonl(job_dir / "feedback.jsonl", 1) or [{}])[-1],
        "feedback_tail": read_jsonl(job_dir / "feedback.jsonl", 100),
        "cmd_vel_summary": {
            "sample_count": len(trace),
            "max_linear_x": max((abs(v.get("linear_x", 0)) for v in trace), default=0),
            "max_angular_z": max((abs(v.get("angular_z", 0)) for v in trace), default=0),
            "zero_speed_count": sum(not any(abs(v.get(k, 0)) > 1e-9 for k in ("linear_x", "linear_y", "angular_z")) for v in trace),
        },
        "cmd_vel_tail": trace, "diagnostics": optional("diagnostics.json", {}),
        "artifacts": {key: str(job_dir / name) for key, name in {
            "global_path_json": "global_path.json", "global_path_csv": "global_path.csv",
            "global_path_png": "global_path.png", "report_md": "navigation_report.md"}.items()
        },
    }
    atomic_write_json(job_dir / "webui_payload.json", payload)
    return payload


def write_report(job_dir: Path, payload: dict[str, Any]) -> Path:
    request, status, path = payload["request"], payload["status"], payload["path"]
    real = status.get("is_real_nav2_path", False)
    lines = [
        "# Navigation2 导航报告", "",
        f"- 请求：`{status.get('request_id', '')}`",
        f"- 模式：`{request.get('mode', '')}`",
        f"- 后端：`{status.get('backend', '')}`",
        f"- 真实 Nav2 路径：**{'是' if real else '否'}**",
        f"- 状态：`{status.get('state', '')}`",
        f"- 路径长度：{path.get('path_length_m', 0):.3f} m",
        f"- 路径点：{path.get('pose_count', 0)}",
        f"- 恢复次数：{status.get('number_of_recoveries', 0)}",
        f"- 重规划次数：{status.get('replan_count', 0)}", "",
        "## 安全门控", "",
        f"`{request.get('safety_confirmation', {})}`", "",
        "## 结果", "",
        status.get("message_zh", ""),
    ]
    if not real:
        lines += ["", "> OFFLINE PREVIEW / 非 Nav2 真实路径 / 不可执行"]
    return atomic_write_text(job_dir / "navigation_report.md", "\n".join(lines) + "\n")
