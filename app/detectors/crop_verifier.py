"""Vision-LLM verification for localized detector crops."""

from __future__ import annotations

import base64
from io import BytesIO
from pathlib import Path
from typing import Any

from openai import OpenAI
from PIL import Image

from app.config import Settings, get_settings
from app.utils.json_utils import extract_json_from_text
from app.reasoning.target_profile import TargetProfile


class CropVerifier:
    def __init__(
        self,
        settings: Settings | None = None,
        client: Any | None = None,
    ) -> None:
        self.settings = settings or get_settings()
        self.client = client

    @property
    def available(self) -> bool:
        return bool(self.settings.siliconflow_api_key)

    def verify_crop(
        self,
        crop_path: str | Path,
        target_profile: TargetProfile,
        candidate_label: str,
        context: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not self.available:
            return {
                "verification_failed": True,
                "failure_reason": "missing_api_key",
                "is_target": False,
                "target_match_score": 0.0,
                "category_score": 0.0,
                "attribute_score": 0.0,
                "context_score": 0.0,
                "evidence": [],
                "rejection_reason": "",
            }
        try:
            client = self.client or OpenAI(
                api_key=self.settings.siliconflow_api_key,
                base_url=self.settings.siliconflow_base_url,
                timeout=self.settings.crop_verify_timeout_seconds,
            )
            response = client.chat.completions.create(
                model=self.settings.vision_model,
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "你是具身导航机器人的候选区域复核模块。"
                            "只依据局部图像证据输出严格 JSON，不要 Markdown。"
                        ),
                    },
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": _verification_prompt(
                                    target_profile, candidate_label, context
                                ),
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": _image_data_url(crop_path),
                                    "detail": "high",
                                },
                            },
                        ],
                    },
                ],
                temperature=0.0,
                max_tokens=min(1200, self.settings.siliconflow_max_tokens),
            )
            content = response.choices[0].message.content
            if not isinstance(content, str) or not content.strip():
                raise ValueError("empty verifier response")
            return normalize_verify_result(extract_json_from_text(content))
        except Exception as exc:
            return {
                "verification_failed": True,
                "failure_reason": f"{type(exc).__name__}: {exc}",
                "is_target": False,
                "target_match_score": 0.0,
                "category_score": 0.0,
                "attribute_score": 0.0,
                "context_score": 0.0,
                "evidence": [],
                "rejection_reason": "",
            }


def normalize_verify_result(payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "verification_failed": False,
        "main_object_zh": str(payload.get("main_object_zh") or ""),
        "main_object_en": str(payload.get("main_object_en") or ""),
        "is_target": bool(payload.get("is_target")),
        "target_match_score": _score(payload.get("target_match_score")),
        "category_score": _score(payload.get("category_score")),
        "attribute_score": _score(payload.get("attribute_score")),
        "context_score": _score(payload.get("context_score")),
        "observed_attributes": _list(payload.get("observed_attributes")),
        "missing_required_attributes": _list(payload.get("missing_required_attributes")),
        "possible_confusions": _list(payload.get("possible_confusions")),
        "evidence": _list(payload.get("evidence")),
        "rejection_reason": str(payload.get("rejection_reason") or ""),
    }


def _verification_prompt(
    profile: TargetProfile,
    candidate_label: str,
    context: dict[str, Any] | None,
) -> str:
    return f"""目标描述：{profile.raw_query}
核心实体：{profile.core_entities}
英文词：{profile.en_terms}
属性：{profile.attributes}
颜色：{profile.colors}
空间关系：{profile.relation_constraints}
可能混淆物：{profile.possible_confusions}
检测器初步标签：{candidate_label}
上下文：{context or {}}

要求：
1. 只根据裁剪图判断，主体不清楚时降低分数。
2. 相似但关键属性不符时 is_target=false。
3. 分数均为 0 到 1。
4. 只输出以下 JSON：
{{
  "main_object_zh": "",
  "main_object_en": "",
  "is_target": false,
  "target_match_score": 0.0,
  "category_score": 0.0,
  "attribute_score": 0.0,
  "context_score": 0.0,
  "observed_attributes": [],
  "missing_required_attributes": [],
  "possible_confusions": [],
  "evidence": [],
  "rejection_reason": ""
}}"""


def _image_data_url(path: str | Path) -> str:
    with Image.open(path) as source:
        image = source.convert("RGB")
        image.thumbnail((1280, 1280))
        buffer = BytesIO()
        image.save(buffer, format="JPEG", quality=90)
    encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
    return f"data:image/jpeg;base64,{encoded}"


def _score(value: Any) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0


def _list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item) for item in value if str(item).strip()]
