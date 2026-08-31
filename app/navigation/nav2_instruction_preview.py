"""Turn a path into a human-readable explanation, never velocity commands."""

from __future__ import annotations

import math
from typing import Any

from .nav2_path_utils import compute_path_length, compute_segment_heading, normalize_angle, simplify_path_rdp

WARNING = "这些步骤是对 Nav2 Path 的人类可读解释，不是底层速度控制指令。"


def build_instruction_preview(poses: list[dict[str, Any]], *, epsilon: float = 0.08,
                              turn_threshold_deg: float = 15.0,
                              semantic_goal: bool = False) -> dict[str, Any]:
    simplified = simplify_path_rdp(poses, epsilon)
    steps: list[dict[str, Any]] = []
    headings = [compute_segment_heading(a, b) for a, b in zip(simplified, simplified[1:])]
    for index, (start, end) in enumerate(zip(simplified, simplified[1:])):
        if index:
            delta = math.degrees(normalize_angle(headings[index] - headings[index-1]))
            if abs(delta) >= turn_threshold_deg:
                action = "rotate_left" if delta > 0 else "rotate_right"
                steps.append({"step": len(steps)+1, "action": action,
                              "action_zh": "左转" if delta > 0 else "右转",
                              "angle_deg": round(abs(delta), 2), "distance_m": 0.0,
                              "from_index": index, "to_index": index})
        distance = compute_path_length([start, end])
        if distance > 1e-9:
            steps.append({"step": len(steps)+1, "action": "follow_curve" if index and abs(math.degrees(normalize_angle(headings[index]-headings[index-1]))) >= turn_threshold_deg else "move_forward",
                          "action_zh": "沿曲线路径前进" if index and steps and steps[-1]["action"].startswith("rotate_") else "沿路径前进",
                          "angle_deg": 0.0, "distance_m": round(distance, 3),
                          "from_index": index, "to_index": index+1})
    action = "stop_and_reobserve" if semantic_goal else "arrive"
    steps.append({"step": len(steps)+1, "action": action,
                  "action_zh": "在安全距离停止并重新观察" if semantic_goal else "到达并停止",
                  "angle_deg": 0.0, "distance_m": 0.0,
                  "from_index": max(0, len(poses)-1), "to_index": max(0, len(poses)-1)})
    return {"schema_version": "1.0", "warning": WARNING,
            "path_length_m": compute_path_length(poses), "steps": steps}
