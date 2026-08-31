"""Build detector prompts from target text and runtime LLM priors."""

from __future__ import annotations

import re
from typing import Any


def build_dynamic_detector_prompts(
    target: str,
    llm_prior: dict[str, Any] | None = None,
    scene_summary: str | None = None,
    crop_captions: list[str] | None = None,
    max_prompts: int = 12,
) -> dict[str, Any]:
    prompts: list[str] = []
    prompts.extend(_target_terms(target))
    if llm_prior and llm_prior.get("available"):
        prompts.extend(
            str(item)
            for item in llm_prior.get("suggested_detector_prompts") or []
            if str(item).strip()
        )
    prompts.extend(_caption_entities(scene_summary or ""))
    for caption in crop_captions or []:
        prompts.extend(_caption_entities(caption))
    deduped = _dedupe(prompts)[:max_prompts]
    source_parts = ["target"]
    if llm_prior and llm_prior.get("available") and llm_prior.get("suggested_detector_prompts"):
        source_parts.append("llm_runtime_commonsense")
    if scene_summary:
        source_parts.append("current_scene_caption")
    if crop_captions:
        source_parts.append("crop_caption")
    return {
        "source": " + ".join(source_parts),
        "handwritten_prompt_used": False,
        "prompts": deduped,
    }


def _target_terms(target: str) -> list[str]:
    cleaned = target.strip()
    for prefix in ["请帮我找到", "请帮我找", "帮我找到", "帮我找", "寻找", "找到", "找一下", "找"]:
        if cleaned.startswith(prefix) and len(cleaned) > len(prefix):
            cleaned = cleaned[len(prefix):].strip(" ：:，,。")
            break
    return [cleaned or target.strip()]


def _caption_entities(text: str) -> list[str]:
    # Only extracts explicit alphanumeric detector-like phrases; no static scene vocab.
    return [
        item
        for item in re.findall(r"[A-Za-z][A-Za-z0-9 -]{1,30}", text)
        if len(item.strip()) >= 2
    ][:6]


def _dedupe(values: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        normalized = " ".join(str(value).strip().lower().split())
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        result.append(normalized)
    return result
