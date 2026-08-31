"""Integrated report writer for target search with optional scene mapping."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def write_video_target_search_report(
    search_result: dict[str, Any],
    output_path: str | Path,
    output_files: dict[str, Path] | None = None,
) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    task = search_result.get("task", {})
    best = search_result.get("best_evidence")
    decision = search_result.get("navigation_decision", {})
    ranked_places = (
        search_result.get("topology_assisted_search", {}).get("ranked_places", [])
    )

    lines = [
        "# Video Target Search Report",
        "",
        "## 1. Target",
        "",
        f"- Target: {task.get('target') or search_result.get('target')}",
        "- Mode: target_search",
        f"- Scene Mapping Enabled: {search_result.get('scene_mapping_enabled', False)}",
        f"- Navigation Topology Enabled: {search_result.get('navigation_topology_enabled', False)}",
        "",
        "## 2. Target Search Result",
        "",
        f"- Target Status: {search_result.get('target_status')}",
        f"- Target Confirmed: {search_result.get('target_confirmed', False)}",
    ]
    if best:
        lines.extend(
            [
                f"- Best Candidate Frame: {best.get('frame_id')}",
                f"- Best Candidate Timestamp: {best.get('timestamp_sec')}",
                f"- Best Candidate BBox: {best.get('bbox')}",
                f"- Evidence: {best.get('evidence_source', 'visual_frame_evidence')}",
            ]
        )
    else:
        lines.append("- Best Candidate: none")

    lines.extend(["", "## 3. Visual Evidence", ""])
    candidates = [
        item for item in search_result.get("timeline", []) if item.get("type") == "direct_detection"
    ]
    if candidates:
        for item in candidates[:20]:
            lines.append(
                f"- frame={item.get('frame_id')} t={item.get('timestamp_sec')} "
                f"label={item.get('label')} score={item.get('score')}"
            )
    else:
        lines.append("- No direct visual target candidates were collected.")

    lines.extend(
        [
            "",
            "## 4. Navigation Decision",
            "",
            f"- Next Action: {decision.get('next_action')}",
            f"- Next Place: {decision.get('next_place_id', 'n/a')}",
            f"- Requires Visual Confirmation: {decision.get('requires_visual_confirmation', True)}",
            f"- Reason: {decision.get('reason', '')}",
            "",
            "## 5. Scene Map Summary",
            "",
        ]
    )
    scene_map = search_result.get("scene_map_result") or {}
    if scene_map:
        lines.extend(
            [
                f"- Places: {len(scene_map.get('place_segments', []))}",
                f"- Object Tracks: {len(scene_map.get('object_tracks', []))}",
                f"- PSG Predicted Nodes: {len(scene_map.get('psg_layer', {}).get('predicted_nodes', []))}",
            ]
        )
    else:
        lines.append("- Scene mapping was not enabled.")

    lines.extend(["", "## 6. Topology-Assisted Search", ""])
    if ranked_places:
        for place in ranked_places[:10]:
            lines.append(
                f"- {place.get('place_id')} score={place.get('target_search_score')} "
                f"confirmed=false: {place.get('reason')}"
            )
    else:
        lines.append("- No topology-assisted ranking was generated.")

    lines.extend(["", "## 7. PSG Predictions", ""])
    if scene_map.get("psg_layer", {}).get("predicted_nodes"):
        lines.append("- PSG predictions are exploration hints and cannot confirm the target.")
    else:
        lines.append("- No PSG prediction layer was generated.")

    lines.extend(["", "## 8. Output Files", ""])
    for path_item in (output_files or {}).values():
        lines.append(f"- {path_item}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
