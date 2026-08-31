"""Video-level summary, navigation trace, and explanatory report generation."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from app.video.video_scene_reasoner import FrameSceneReasoningResult


def enrich_search_result(
    search_result: dict[str, Any],
    frame_results: list[FrameSceneReasoningResult],
    memory_updates: list[dict[str, Any]],
    psg_result: dict[str, Any],
    memory_written_count: int,
    memory_store_path: str,
    retrieved_memory_context: dict[str, Any],
) -> dict[str, Any]:
    negative_evidence = [
        {
            "timestamp_sec": result.timestamp_sec,
            "frame_id": result.frame_id,
            "room_type": result.scene_understanding.get("room_type", "unknown"),
            "evidence": item,
        }
        for result in frame_results
        for item in result.negative_evidence
    ]
    observed_places = [
        {
            "place_id": f"place_{index:03d}",
            "room_type": memory.get("room_type", "unknown"),
            "time_range": memory.get("time_range", []),
            "summary": memory.get("scene_summary", ""),
            "stable_landmarks": memory.get("place_signature", {}).get(
                "stable_landmarks", []
            ),
            "target_found": memory.get("target_context", {}).get("found", False),
            "importance": memory.get("importance", "low"),
            "target_search_priority": _priority(memory),
        }
        for index, memory in enumerate(memory_updates, start=1)
    ]
    hypotheses = psg_result.get("hypotheses", [])
    search_result.update(
        {
            "target": search_result["task"]["target"],
            "direct_candidates": [
                item
                for item in search_result.get("timeline", [])
                if item.get("type") == "direct_detection"
            ],
            "indirect_candidates": search_result.get("candidate_regions", []),
            "environment_memories_written": memory_written_count,
            "memory_updates_generated": len(memory_updates),
            "negative_evidence_count": len(negative_evidence),
            "negative_evidence": negative_evidence,
            "observed_places": observed_places,
            "psg_hypotheses": hypotheses,
            "memory_store_path": memory_store_path,
            "retrieved_memory_context": retrieved_memory_context,
            "memory_statistics": {
                "written_count": memory_written_count,
                "generated_count": len(memory_updates),
                "environment_observations": sum(
                    item.get("memory_kind") == "environment_observation"
                    for item in memory_updates
                ),
                "positive_target_evidence": sum(
                    item.get("target_context", {}).get("found") is True
                    for item in memory_updates
                ),
                "negative_target_memories": sum(
                    item.get("target_context", {}).get("found") is False
                    for item in memory_updates
                ),
            },
        }
    )
    search_result["final_summary"] = _final_summary(search_result)
    if not search_result.get("target_found"):
        search_result["navigation_interpretation"]["suggestion"] = (
            _next_search_suggestion(search_result)
        )
    return search_result


def build_navigation_trace(
    video_id: str,
    target: str,
    results: list[FrameSceneReasoningResult],
) -> dict[str, Any]:
    return {
        "video_id": video_id,
        "target": target,
        "trace": [
            {
                "timestamp_sec": result.timestamp_sec,
                "frame_id": result.frame_id,
                "scene_state": (
                    f"{result.scene_understanding.get('room_type', 'unknown')}_observed"
                ),
                "target_state": (
                    "found"
                    if result.target_evidence.get("directly_found")
                    else "not_found"
                ),
                "memory_action": (
                    "write_" + result.memory_update.get(
                        "memory_kind", "environment_observation"
                    )
                    if result.memory_update.get("should_write")
                    else "skip_invalid_frame"
                ),
                "psg_action": (
                    "add_"
                    + (
                        result.psg_hypotheses[0].get(
                            "relation_type", "supports"
                        )
                        if result.psg_hypotheses
                        else "observation"
                    )
                    + "_edge"
                ),
                "reasoning_summary": result.reasoning_summary.get("conclusion", ""),
            }
            for result in results
        ],
    }


def write_video_reasoning_report(
    search_result: dict[str, Any],
    output_path: str | Path,
    output_files: list[str] | None = None,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    task = search_result["task"]
    best = search_result.get("best_evidence")
    lines = [
        "# 视频目标搜索与场景记忆报告",
        "",
        "## 1. 任务信息",
        "",
        f"- 视频：{task['video_path']}",
        f"- 目标：{task['target']}",
        f"- 检测器：{task['detector']}",
        f"- 关键帧数量：{search_result['video_meta']['sampled_keyframes']}",
        "- 分析模式：scene-centric video memory + target-conditioned reasoning",
        "",
        "## 2. 目标搜索结果",
        "",
    ]
    if best:
        lines.append(
            f"已直接发现目标“{task['target']}”，最佳证据位于 "
            f"{best['timestamp_sec']:.2f}s，置信度 {best['confidence']:.3f}。"
        )
    else:
        lines.append(
            f"本次视频未直接发现目标“{task['target']}”，但系统已继续生成环境记忆、"
            "负目标证据和 PSG 假设。"
        )
    lines.extend(["", "## 3. 已观察环境记忆", ""])
    places = search_result.get("observed_places", [])
    if places:
        for place in places:
            time_range = place.get("time_range", [])
            span = (
                f"{time_range[0]:.2f}s-{time_range[1]:.2f}s"
                if len(time_range) == 2
                else "时间未知"
            )
            lines.append(
                f"- {span} [{place['room_type']}]：{place['summary']}"
            )
    else:
        lines.append("- 没有可写入的有效环境帧。")
    lines.extend(["", "## 4. 正目标证据", ""])
    if best:
        lines.append(f"- {best['timestamp_sec']:.2f}s：{best['description']}")
    else:
        lines.append("- 没有直接正目标证据。")
    lines.extend(["", "## 5. 负目标证据", ""])
    negative = search_result.get("negative_evidence", [])
    if negative:
        for item in negative[:30]:
            lines.append(f"- {item['timestamp_sec']:.2f}s：{item['evidence']}")
    else:
        lines.append("- 没有生成负目标证据。")
    lines.extend(["", "## 6. PSG 预测假设", ""])
    hypotheses = search_result.get("psg_hypotheses", [])
    if hypotheses:
        for item in hypotheses[:30]:
            lines.append(
                f"- [{item['type']}, {item['confidence']:.2f}] {item['summary']}"
            )
    else:
        lines.append("- 没有生成 PSG 假设。")
    stats = search_result.get("memory_statistics", {})
    lines.extend(
        [
            "",
            "## 7. 长期记忆写入结果",
            "",
            f"- 生成记忆：{stats.get('generated_count', 0)} 条",
            f"- 实际写入：{stats.get('written_count', 0)} 条",
            f"- 环境观察记忆：{stats.get('environment_observations', 0)} 条",
            f"- 负目标记忆：{stats.get('negative_target_memories', 0)} 条",
            f"- 记忆库：{search_result.get('memory_store_path', '')}",
            "",
            "## 8. 后续搜索建议",
            "",
            search_result["navigation_interpretation"]["suggestion"],
            "",
            "## 9. 输出文件清单",
            "",
        ]
    )
    for item in output_files or []:
        lines.append(f"- {item}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _priority(memory: dict[str, Any]) -> str:
    if memory.get("target_context", {}).get("found"):
        return "high"
    if memory.get("target_context", {}).get("candidate_found"):
        return "high"
    return "low"


def _final_summary(search_result: dict[str, Any]) -> str:
    target = search_result["task"]["target"]
    if search_result.get("target_found"):
        return (
            f"本视频直接发现了“{target}”，并同时写入经过区域的场景记忆与 PSG 关系。"
        )
    return (
        f"本视频未直接发现“{target}”，但已生成 "
        f"{search_result.get('memory_updates_generated', 0)} 条环境记忆、"
        f"{search_result.get('negative_evidence_count', 0)} 条负证据和 "
        f"{len(search_result.get('psg_hypotheses', []))} 条 PSG 假设。"
    )


def _next_search_suggestion(search_result: dict[str, Any]) -> str:
    high_priority = [
        item
        for item in search_result.get("observed_places", [])
        if item.get("target_search_priority") == "high"
    ]
    if high_priority:
        place = high_priority[0]
        span = place.get("time_range", [])
        time_text = (
            f"{span[0]:.2f}s-{span[1]:.2f}s"
            if len(span) == 2
            else "对应时间片"
        )
        return (
            f"建议优先复查视频 {time_text} 的 {place.get('room_type')} 区域，"
            "重点检查目标常见承载面、边缘和遮挡处。"
        )
    hypotheses = search_result.get("psg_hypotheses", [])
    revisit = next(
        (
            item
            for item in hypotheses
            if item.get("type") in {"should_revisit", "high_priority_for_target"}
        ),
        None,
    )
    if revisit:
        return revisit["summary"]
    return "建议继续采集未覆盖区域的视频，并优先检查目标常见位置。"
