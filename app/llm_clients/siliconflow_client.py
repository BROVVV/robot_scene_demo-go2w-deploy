"""SiliconFlow vision LLM client."""

from __future__ import annotations

import base64
from io import BytesIO
import mimetypes
from pathlib import Path
from typing import Any

from openai import OpenAI
from PIL import Image

from app.config import Settings, SettingsError, get_settings
from app.llm_clients.base import BaseVisionLLMClient
from app.utils.json_utils import extract_json_from_text


JPEG_QUALITY = 85


class SiliconFlowVisionClient(BaseVisionLLMClient):
    """OpenAI-compatible SiliconFlow implementation."""

    def __init__(self, settings: Settings | None = None, client: Any | None = None) -> None:
        self.settings = settings or get_settings()
        if not self.settings.siliconflow_api_key:
            raise SettingsError(
                "Missing SILICONFLOW_API_KEY. Copy .env.example to .env and set "
                "your SiliconFlow API key before calling the vision API."
            )
        self.client = client or OpenAI(
            api_key=self.settings.siliconflow_api_key,
            base_url=self.settings.siliconflow_base_url,
            timeout=self.settings.siliconflow_timeout_seconds,
        )

    def analyze_scene(
        self,
        image_path: str,
        target_text: str,
        extra_instructions: str | None = None,
    ) -> dict:
        image_data_url = self._image_to_data_url(image_path)

        response = self.client.chat.completions.create(
            model=self.settings.vision_model,
            messages=[
                {"role": "system", "content": _FAST_SYSTEM_PROMPT},
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": _build_fast_user_prompt(
                                target_text, extra_instructions
                            ),
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image_data_url,
                                "detail": self.settings.image_detail,
                            },
                        },
                    ],
                },
            ],
            temperature=0.1,
            max_tokens=getattr(self.settings, "vlm_runtime_semantic_max_tokens", self.settings.siliconflow_max_tokens),
        )

        raw_content = self._extract_response_text(response)
        print(raw_content)

        try:
            return _normalize_fast_result(
                extract_json_from_text(raw_content), target_text
            )
        except ValueError as exc:
            raise ValueError(
                "SiliconFlow response was not valid scene JSON. Raw response:\n"
                f"{raw_content}"
            ) from exc

    def _image_to_data_url(self, image_path: str) -> str:
        path = Path(image_path)
        if not path.is_file():
            raise FileNotFoundError(f"Image file not found: {image_path}")

        image_bytes, mime_type = _load_resized_image_bytes(
            path, self.settings.image_max_side
        )

        encoded = base64.b64encode(image_bytes).decode("ascii")
        return f"data:{mime_type};base64,{encoded}"

    @staticmethod
    def _extract_response_text(response: Any) -> str:
        try:
            content = response.choices[0].message.content
        except (AttributeError, IndexError, TypeError) as exc:
            raise ValueError("SiliconFlow response did not contain message content.") from exc

        if not isinstance(content, str) or content.strip() == "":
            raise ValueError("SiliconFlow response message content was empty.")

        return content.strip()


def _load_resized_image_bytes(path: Path, max_side: int) -> tuple[bytes, str]:
    try:
        with Image.open(path) as image:
            image = image.convert("RGB")
            image.thumbnail((max_side, max_side))
            buffer = BytesIO()
            image.save(buffer, format="JPEG", quality=JPEG_QUALITY, optimize=True)
            return buffer.getvalue(), "image/jpeg"
    except OSError:
        # Image.open accepts both str and Path; normalize before attribute use.
        path = Path(path)
        mime_type, _ = mimetypes.guess_type(path.name)
        if mime_type is None:
            mime_type = "application/octet-stream"
        return path.read_bytes(), mime_type


_FAST_SYSTEM_PROMPT = """你是机器狗快速视觉识别模块。只输出紧凑 JSON，不要 Markdown。
优先识别目标相关物体、障碍物、人物、桌椅、显示器、主机、箱子、鞋、篮子、线缆等导航相关物体。
必须列出当前画面中的所有可见物体：objects 数组至少包含 5 个不重复的物体；
即使目标不在画面中，也必须把办公椅、办公桌、背包、显示器、垃圾桶、纸箱、人、墙壁、隔断等可见的非目标物体全部列出，禁止返回空数组。
只输出场景物体与空间关系，禁止输出 route_plan、task_understanding、scene_reasoning_hints；本地知识库、假设评分和任务规划模块会负责最终推理。"""


def _build_fast_user_prompt(
    target_text: str,
    extra_instructions: str | None = None,
) -> str:
    prompt = f"""目标：{target_text}

返回 JSON：
{{
  "scene_summary_zh": "一句话场景摘要",
  // 注意：objects 必须至少 5 个、禁止空数组，
  // 目标不在时也要列出所有可见的非目标物体。
  "objects": [
    {{
      "name": "chair",
      "name_zh": "椅子",
      "category": "furniture",
      "color": "black",
      "attributes": ["front-left", "key object"],
      "relative_to_robot": "front-left",
      "bbox_2d": {{"x1": 0.10, "y1": 0.20, "x2": 0.35, "y2": 0.75}},
      "estimated_distance_m": 1.2,
      "confidence": 0.8
    }}
  ],
  "relations": [
    {{
      "source_index": 2,
      "target_index": 1,
      "relation_type": "on",
      "description_zh": "黄色衣服挂在椅子上",
      "confidence": 0.8
    }}
  ],
  "target_decision": {{
    "is_present": true,
    "matched_indices": [1, 2],
    "match_reason_zh": "中文原因",
    "confidence": 0.8
  }}
}}

约束：
- 本阶段只做场景物体与空间关系提取，禁止输出 route_plan、task_understanding、scene_reasoning_hints 字段。
- 不要写完整思维链，不要生成路线规划；本地规划器会负责导航。
- relation_type 只能用 left_of/right_of/in_front_of/behind/on/under/above/below/in/near/far/contains/occluding。
- 每个可见物体必须输出 bbox_2d，坐标是相对图像宽高归一化到 0~1 的 x1/y1/x2/y2。
- bbox_2d 必须尽量紧贴物体，不要用整张图 [0,0,1,1] 代替局部物体。
- 目标是“挂着黄衣服的椅子”时，椅子和黄色衣服都要列入 objects。
- 只输出 JSON。"""
    if extra_instructions:
        prompt += f"\n额外要求：{extra_instructions}"
    return prompt


def _normalize_fast_result(raw: dict, target_text: str) -> dict:
    raw_objects = raw.get("objects") or []
    raw_relations = raw.get("relations") or []
    raw_decision = raw.get("target_decision") or {}
    all_indices = [
        relation.get(key)
        for relation in raw_relations
        for key in ("source_index", "target_index")
    ]
    all_indices.extend(raw_decision.get("matched_indices") or [])
    zero_based_indices = _contains_zero_index(all_indices)
    objects = []
    for index, obj in enumerate(raw_objects, start=1):
        obj_id = f"obj_{index:03d}"
        relative = str(obj.get("relative_to_robot") or "front")
        bbox = _normalize_bbox(obj.get("bbox_2d") or obj.get("bbox"))
        objects.append(
            {
                "id": obj_id,
                "name": str(obj.get("name") or "object"),
                "name_zh": str(obj.get("name_zh") or obj.get("name") or "物体"),
                "category": str(obj.get("category") or "unknown"),
                "color": obj.get("color"),
                "attributes": list(obj.get("attributes") or []),
                "visible": bool(obj.get("visible", True)),
                "position": {
                    "horizontal": _horizontal_from_relative(relative),
                    "vertical": "middle",
                    "relative_to_robot": relative,
                    "estimated_distance_m": obj.get("estimated_distance_m"),
                },
                "bbox_2d": bbox,
                "confidence": _clamp_confidence(obj.get("confidence", 0.6)),
            }
        )

    relations = []
    for relation in raw_relations:
        source_id = _relation_object_id(
            relation, "source", len(objects), zero_based=zero_based_indices
        )
        target_id = _relation_object_id(
            relation, "target", len(objects), zero_based=zero_based_indices
        )
        if source_id is None or target_id is None:
            continue
        relations.append(
            {
                "source_id": source_id,
                "target_id": target_id,
                "relation_type": _normalize_relation_type(
                    relation.get("relation_type")
                ),
                "description_zh": relation.get("description_zh") or "",
                "estimated_distance_m": relation.get("estimated_distance_m"),
                "confidence": _clamp_confidence(relation.get("confidence", 0.6)),
            }
        )

    decision = raw_decision
    target_is_present = bool(decision.get("is_present", False))
    matched_ids = []
    matched_indices = decision.get("matched_indices") or []
    for item in matched_indices:
        try:
            item_index = int(item)
        except (TypeError, ValueError):
            continue
        object_id = _index_to_object_id(item_index, len(objects), zero_based_indices)
        if object_id is not None:
            matched_ids.append(object_id)

    route_plan = raw.get("route_plan") or {}
    steps = []
    for index, step in enumerate(route_plan.get("steps") or [], start=1):
        steps.append(
            {
                "step_id": index,
                "action": _normalize_action(step.get("action")),
                "distance_m": step.get("distance_m"),
                "turn_angle_deg": step.get("turn_angle_deg"),
                "description_zh": step.get("description_zh") or "停止",
            }
        )
    if not steps:
        steps = [
            {
                "step_id": 1,
                "action": "stop",
                "distance_m": None,
                "turn_angle_deg": None,
                "description_zh": "停止并重新观察",
            }
        ]

    return {
        "scene_summary_zh": raw.get("scene_summary_zh") or "场景分析完成",
        "objects": objects,
        "relations": relations,
        "topology": {"nodes": [], "edges": []},
        "target_decision": {
            "target_text": target_text,
            "is_present": target_is_present,
            "matched_object_ids": matched_ids,
            "match_reason_zh": decision.get("match_reason_zh") or "",
            "confidence": _clamp_confidence(decision.get("confidence", 0.5)),
        },
        "route_plan": {
            "route_type": (
                "approach_visible_target"
                if target_is_present
                else "explore_likely_location"
            ),
            "summary_zh": route_plan.get("summary_zh") or "",
            "steps": steps,
            "safety_notes_zh": ["快速视觉估计，仅用于 Demo"],
        },
    }


def _horizontal_from_relative(relative: str) -> str:
    if "left" in relative:
        return "left"
    if "right" in relative:
        return "right"
    return "center"


def _relation_object_id(
    relation: dict,
    side: str,
    object_count: int,
    *,
    zero_based: bool | None = None,
) -> str | None:
    explicit = relation.get(f"{side}_id")
    if isinstance(explicit, str) and explicit.startswith("obj_"):
        return explicit
    try:
        index = int(relation.get(f"{side}_index"))
    except (TypeError, ValueError):
        return None
    if zero_based is None:
        zero_based = _contains_zero_index(
            [relation.get("source_index"), relation.get("target_index")]
        )
    return _index_to_object_id(index, object_count, zero_based)


def _index_to_object_id(
    index: int,
    object_count: int,
    zero_based: bool = False,
) -> str | None:
    if zero_based:
        if 0 <= index < object_count:
            return f"obj_{index + 1:03d}"
        return None
    if 1 <= index <= object_count:
        return f"obj_{index:03d}"
    return None


def _contains_zero_index(values: list[object]) -> bool:
    for value in values:
        try:
            if int(value) == 0:
                return True
        except (TypeError, ValueError):
            continue
    return False


def _clamp_confidence(value: object) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0.5
    return max(0.0, min(1.0, numeric))


def _normalize_bbox(value: object) -> dict[str, float]:
    fallback = {"x1": 0.0, "y1": 0.0, "x2": 1.0, "y2": 1.0}
    if isinstance(value, dict):
        raw_values = [value.get(key) for key in ("x1", "y1", "x2", "y2")]
    elif isinstance(value, (list, tuple)) and len(value) == 4:
        raw_values = list(value)
    else:
        return fallback
    try:
        x1, y1, x2, y2 = (
            max(0.0, min(1.0, float(item))) for item in raw_values
        )
    except (TypeError, ValueError):
        return fallback
    if x2 <= x1 or y2 <= y1:
        return fallback
    return {"x1": x1, "y1": y1, "x2": x2, "y2": y2}


def _normalize_relation_type(value: object) -> str:
    normalized = str(value or "near").lower().strip().replace(" ", "_")
    allowed = {
        "left_of",
        "right_of",
        "in_front_of",
        "behind",
        "on",
        "under",
        "above",
        "below",
        "in",
        "near",
        "far",
        "contains",
        "occluding",
    }
    if normalized in allowed:
        return normalized
    aliases = {
        "holding": "near",
        "held_by": "near",
        "beside": "near",
        "next_to": "near",
        "inside": "in",
        "within": "in",
        "over": "above",
        "in_front": "in_front_of",
    }
    return aliases.get(normalized, "near")


def _normalize_action(value: object) -> str:
    normalized = str(value or "stop").lower().strip().replace(" ", "_")
    allowed = {
        "move_forward",
        "move_backward",
        "turn_left",
        "turn_right",
        "stop",
    }
    return normalized if normalized in allowed else "stop"
