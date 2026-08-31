"""Markdown report writer for video full-scene maps."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from app.video.schemas import FrameObservation, ObjectTrack, PSGLayer, SceneGraph


def write_video_full_scene_report(
    *,
    video_path: str,
    detector: str,
    sample_fps: float,
    frame_observations: list[FrameObservation],
    object_tracks: list[ObjectTrack],
    observed_graph: SceneGraph,
    psg_layer: PSGLayer,
    topology: dict[str, Any] | None,
    output_path: str | Path,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 视频全场景建图报告",
        "",
        "## 1. 视频概况",
        f"- 视频路径：{video_path}",
        f"- 分析帧数：{len(frame_observations)}",
        f"- 采样 FPS：{sample_fps}",
        f"- 检测器：{detector}",
        "",
        "## 2. 场景摘要",
        _scene_summary(frame_observations),
        "",
        "## 3. 检测到的主要区域",
        f"- Observed region nodes: {sum(node.node_type == 'region' for node in observed_graph.nodes)}",
        "",
        "## 4. 检测到的所有物体",
        "",
        "| ID | 名称 | 类别 | 首次出现 | 最后出现 | 稳定位置 | 置信度 | 导航作用 |",
        "|---|---|---|---:|---:|---|---:|---|",
    ]
    for track in object_tracks:
        lines.append(
            f"| {track.track_id} | {track.label_zh or track.label} | {track.category} | "
            f"{track.first_seen_sec:.2f}s | {track.last_seen_sec:.2f}s | "
            f"{track.stable_position_2d} | {track.confidence:.2f} | {track.navigation_role} |"
        )
    lines.extend(
        [
            "",
            "## 5. 可通行区域",
            _free_space_summary(frame_observations),
            "",
            "## 6. 障碍物和风险",
            _hazard_summary(frame_observations),
            "",
            "## 7. Observed Scene Graph 摘要",
            f"- 节点数：{len(observed_graph.nodes)}",
            f"- 边数：{len(observed_graph.edges)}",
            "",
            "## 8. PSG 预测图摘要",
            f"- 预测节点数：{len(psg_layer.predicted_nodes)}",
            f"- 预测边数：{len(psg_layer.predicted_edges)}",
            f"- 安全过滤警告：{len(psg_layer.warnings)}",
            "",
            "## 9. 推荐下一视角",
        ]
    )
    if psg_layer.next_best_views:
        for view in psg_layer.next_best_views:
            lines.append(
                f"- {view.action}：{view.reason_zh} "
                f"(requires_visual_confirmation={view.requires_visual_confirmation})"
            )
    else:
        lines.append("- stop_and_reobserve：当前没有足够强的预测探索点，建议停下重观测。")
    lines.extend(
        [
            "",
            "## 10. 导航拓扑说明",
            f"- map_type：{(topology or {}).get('map_type', 'not_generated')}",
            f"- 拓扑节点数：{len((topology or {}).get('nodes', []))}",
            f"- 探索候选数：{len((topology or {}).get('exploration_candidates', []))}",
            "",
            "## 11. 限制说明",
            "- 无 odom / SLAM / 深度图时，不输出米级坐标。",
            "- predicted 节点不能用于目标确认。",
            "- PSG 只用于探索建议，所有预测动作都需要视觉确认。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _scene_summary(frames: list[FrameObservation]) -> str:
    summaries = [frame.summary_zh for frame in frames if frame.summary_zh]
    return "；".join(summaries[:6]) or "没有可用场景摘要。"


def _free_space_summary(frames: list[FrameObservation]) -> str:
    items = [space.description_zh for frame in frames for space in frame.free_space]
    if not items:
        return "- 未生成明确可通行区域候选。"
    return "\n".join(f"- {item}" for item in items[:20])


def _hazard_summary(frames: list[FrameObservation]) -> str:
    items = [hazard.description_zh for frame in frames for hazard in frame.hazards]
    if not items:
        return "- 未生成明确障碍/风险候选。"
    return "\n".join(f"- {item}" for item in items[:20])
