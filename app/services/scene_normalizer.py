"""Normalize display labels in scene analysis results."""

from __future__ import annotations

from app.detectors.vocabulary import category_for_label, color_for_label, label_zh
from app.schemas import SceneAnalysisResult


def normalize_scene_labels(result: SceneAnalysisResult) -> SceneAnalysisResult:
    objects = []
    for obj in result.objects:
        normalized_zh = obj.name_zh
        normalized_category = obj.category
        normalized_color = obj.color

        if _looks_untranslated(normalized_zh):
            normalized_zh = label_zh(obj.name)
        if normalized_category == "unknown":
            normalized_category = category_for_label(obj.name)
        if normalized_color is None:
            normalized_color = color_for_label(obj.name)

        objects.append(
            obj.model_copy(
                update={
                    "name_zh": normalized_zh,
                    "category": normalized_category,
                    "color": normalized_color,
                }
            )
        )

    return result.model_copy(update={"objects": objects})


def _looks_untranslated(value: str) -> bool:
    return value.isascii() or " " in value
