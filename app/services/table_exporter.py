"""CSV exporters for scene analysis tables."""

from __future__ import annotations

from pathlib import Path

import pandas as pd

from app.schemas import SceneAnalysisResult


def export_object_table(result: SceneAnalysisResult, output_path: str | Path) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    rows = [
        {
            "id": obj.id,
            "英文名": obj.name,
            "中文名": obj.name_zh,
            "类别": _category_zh(obj.category),
            "类别代码": obj.category,
            "颜色": obj.color,
            "属性": "；".join(obj.attributes),
            "是否可见": obj.visible,
            "水平位置": obj.position.horizontal,
            "前后位置": obj.position.vertical,
            "相对方向": obj.position.relative_to_robot,
            "估计距离": obj.position.estimated_distance_m,
            "bbox_x1": obj.bbox_2d.x1,
            "bbox_y1": obj.bbox_2d.y1,
            "bbox_x2": obj.bbox_2d.x2,
            "bbox_y2": obj.bbox_2d.y2,
            "置信度": obj.confidence,
            "检测器分数": obj.detector_score,
            "复核分数": (
                (obj.crop_verify or {}).get("target_match_score")
                if obj.crop_verify
                else None
            ),
            "融合分数": obj.final_score,
            "决策": obj.decision,
            "拒绝原因": obj.rejection_reason,
            "解释": obj.explanation,
            "候选裁剪": obj.crop_path,
        }
        for obj in result.objects
    ]

    columns = [
        "id",
        "英文名",
        "中文名",
        "类别",
        "类别代码",
        "颜色",
        "属性",
        "是否可见",
        "水平位置",
        "前后位置",
        "相对方向",
        "估计距离",
        "bbox_x1",
        "bbox_y1",
        "bbox_x2",
        "bbox_y2",
        "置信度",
        "检测器分数",
        "复核分数",
        "融合分数",
        "决策",
        "拒绝原因",
        "解释",
        "候选裁剪",
    ]
    pd.DataFrame(
        rows,
        columns=columns,
    ).to_csv(path, index=False, encoding="utf-8-sig")
    return path


def _category_zh(category: str) -> str:
    labels = {
        "person": "人物",
        "furniture": "家具",
        "electronics": "电子设备",
        "container": "容器",
        "personal_item": "个人物品",
        "clothing": "衣物",
        "bag": "包",
        "cable": "线缆",
        "structure": "结构",
        "robot": "机器人",
        "equipment": "设备",
        "unknown": "未知",
    }
    return labels.get(category, category)


def export_relation_table(result: SceneAnalysisResult, output_path: str | Path) -> Path:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    object_names = {obj.id: obj.name_zh for obj in result.objects}

    rows = [
        {
            "source_id": relation.source_id,
            "source中文名": object_names.get(relation.source_id, ""),
            "target_id": relation.target_id,
            "target中文名": object_names.get(relation.target_id, ""),
            "relation_type": relation.relation_type,
            "中文描述": relation.description_zh,
            "估计距离": relation.estimated_distance_m,
            "置信度": relation.confidence,
        }
        for relation in result.relations
    ]

    pd.DataFrame(
        rows,
        columns=[
            "source_id",
            "source中文名",
            "target_id",
            "target中文名",
            "relation_type",
            "中文描述",
            "估计距离",
            "置信度",
        ],
    ).to_csv(path, index=False, encoding="utf-8-sig")
    return path
