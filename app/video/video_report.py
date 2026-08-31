"""Human-readable Markdown reporting for video search."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def write_video_report(
    search_result: dict[str, Any],
    output_path: str | Path,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    task = search_result["task"]
    best = search_result.get("best_evidence")
    regions = search_result.get("candidate_regions", [])

    lines = [
        "# 视频目标搜索报告",
        "",
        "## 1. 任务",
        "",
        f"- 目标：{task['target']}",
        f"- 视频：{task['video_path']}",
        f"- 检测器：{task['detector']}",
        f"- 关键帧数：{search_result['video_meta']['sampled_keyframes']}",
        "",
        "## 2. 结论",
        "",
    ]
    if best:
        lines.append(
            f"已找到目标。最佳证据出现在视频第 {best['timestamp_sec']:.2f} 秒。"
        )
        lines.extend(
            [
                "",
                "## 3. 最佳证据",
                "",
                f"- 时间：{best['timestamp_sec']:.2f}s",
                f"- 帧 ID：{best['frame_id']}",
                f"- 置信度：{best['confidence']:.3f}",
                f"- 综合证据分：{best['evidence_score']:.3f}",
                f"- 画面位置：{best['image_position']}",
                f"- 相对方向提示：{best['relative_direction_hint']}",
                f"- 描述：{best['description']}",
                f"- 标注帧：{best.get('annotated_frame_path') or '未生成'}",
            ]
        )
    else:
        lines.append("未在采样帧中直接发现目标。")
        lines.extend(["", "## 3. 最佳证据", "", "无直接目标证据，系统没有编造目标位置。"])

    lines.extend(["", "## 4. 候选区域", ""])
    if regions:
        for region in regions:
            lines.append(
                f"- {region['timestamp_sec']:.2f}s [{region['priority']}]：{region['reason']}"
            )
    else:
        lines.append("- 没有足够强的候选区域线索。")

    lines.extend(
        [
            "",
            "## 5. 搜索建议",
            "",
            search_result["navigation_interpretation"]["suggestion"],
            "",
            "## 6. 限制说明",
            "",
            search_result["navigation_interpretation"]["reason"],
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
