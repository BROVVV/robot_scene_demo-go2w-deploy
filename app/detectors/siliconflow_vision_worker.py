"""Worker that calls the SiliconFlow vision LLM API from the Conda environment.

The main ROS 2 autonomous-loop script runs with the system Python, which does
not have the OpenAI/PIL dependencies. This worker is executed as a subprocess
inside the project Conda environment (``go2_robot_scene_demo``) and writes a
compact target-detection payload that is compatible with the loop's existing
``objects`` contract:

.. code-block:: json

   {
     "objects": [
       {"label": "gray backpack 灰色书包", "score": 0.85,
        "bbox_2d": [0.2, 0.3, 0.5, 0.8]}
     ],
     "scene_summary_zh": "...",
     "target_decision": {...}
   }
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import sys
from io import BytesIO
from pathlib import Path

from PIL import Image

from app.perception.target_state import (
    TARGET_ABSENT,
    TARGET_POSSIBLE,
    TARGET_PRESENT,
    normalize_target_state,
    target_state_from_payload,
    target_state_is_present,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--extra-instructions", default="")
    parser.add_argument(
        "--model",
        default="",
        help="override the SiliconFlow vision model; defaults to settings",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="use the compact single-target prompt (fast, bbox only) "
             "instead of the full scene-understanding prompt",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="verify whether the object inside --bbox is the target",
    )
    parser.add_argument(
        "--bbox",
        default="",
        help="normalized bbox 'x1,y1,x2,y2' used with --verify",
    )
    return parser.parse_args()


def _write_payload(output: str, payload: dict) -> None:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


def _matched_objects(result: dict, target_text: str) -> list[dict]:
    matched_ids = set(
        result.get("target_decision", {}).get("matched_object_ids") or []
    )
    objects = []
    for obj in result.get("objects") or []:
        if obj.get("id") not in matched_ids:
            continue
        bbox = obj.get("bbox_2d") or {}
        objects.append(
            {
                "label": f"{obj.get('name', '')} {obj.get('name_zh', '')}".strip(),
                "score": float(obj.get("confidence", 0.5)),
                "bbox_2d": [
                    float(bbox.get("x1", 0.0)),
                    float(bbox.get("y1", 0.0)),
                    float(bbox.get("x2", 1.0)),
                    float(bbox.get("y2", 1.0)),
                ],
            }
        )
    return objects


def _quick_detect(settings, image_path: str, target_text: str,
                  extra_instructions: str, model_override: str = "",
                  client: Any | None = None) -> dict:
    """Call the vision API with a short prompt and return the worker payload."""
    from openai import OpenAI

    from app.llm_clients.siliconflow_client import (
        _load_resized_image_bytes,
    )
    from app.utils.json_utils import extract_json_from_text

    if client is None:
        client = OpenAI(
            api_key=settings.siliconflow_api_key,
            base_url=settings.siliconflow_base_url,
            timeout=settings.siliconflow_timeout_seconds,
        )
    model = model_override or settings.vision_model
    image_bytes, mime_type = _load_resized_image_bytes(
        image_path, settings.image_max_side
    )
    with Image.open(BytesIO(image_bytes)) as image:
        image_width, image_height = image.size
    image_data_url = (
        f"data:{mime_type};base64,"
        + base64.b64encode(image_bytes).decode("ascii")
    )
    prompt = _QUICK_SYSTEM_PROMPT.format(target=target_text)
    if extra_instructions:
        prompt += f"\n额外要求：{extra_instructions}"
    response = client.chat.completions.create(
        model=model,
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": image_data_url,
                            "detail": settings.image_detail,
                        },
                    },
                ],
            }
        ],
        temperature=0.1,
        max_tokens=getattr(settings, "vlm_runtime_quick_max_tokens", 768),
    )
    raw_content = response.choices[0].message.content
    if not isinstance(raw_content, str) or not raw_content.strip():
        raise ValueError("SiliconFlow quick response was empty")
    data = extract_json_from_text(raw_content)
    if not isinstance(data, dict):
        raise ValueError("SiliconFlow quick response was not a JSON object")

    target_objects = []
    scene_objects = []
    requested_state = target_state_from_payload(data)
    found = bool(data.get("found", False))
    raw_bbox = data.get("bbox_2d")
    bbox = _normalize_quick_bbox(raw_bbox, image_width, image_height)
    confidence = _clamp01(data.get("confidence", 0.8))
    if requested_state in {TARGET_PRESENT, TARGET_POSSIBLE} or found:
        name = str(data.get("name_zh") or data.get("name") or target_text)
        label = name if name in target_text else f"{target_text} {name}"
        # Keep an uncertain category candidate even when the model did not
        # return a valid box.  The navigation layer will re-observe it, but it
        # will never use this candidate as positive stop evidence.
        target_objects.append(
            {
                "id": "target_obj_001",
                "frame_object_id": "target_obj_001",
                "label": label.strip(),
                "score": confidence,
                "bbox_2d": bbox,
                "target_state": requested_state,
                "candidate_reason_zh": str(data.get("reason_zh") or ""),
            }
        )
    if requested_state == TARGET_PRESENT and bbox is None:
        requested_state = TARGET_POSSIBLE
    if requested_state == TARGET_PRESENT and not target_objects:
        requested_state = TARGET_POSSIBLE
    # 轻量场景物体（计划书 §4.1）：Quick 一次调用同时给出粗粒度场景物体，
    # 供 Full Semantic 超时/未完成时仍能建拓扑。绝不混入 target_objects。
    raw_light = data.get("scene_objects_light")
    if not isinstance(raw_light, list):
        raw_light = data.get("objects") or []
    for raw_obj in raw_light:
        bbox = _normalize_quick_bbox(
            raw_obj.get("bbox_2d") or raw_obj.get("bbox"), image_width, image_height
        )
        if bbox is None:
            continue
        name = str(raw_obj.get("name_zh") or raw_obj.get("name") or "物体")
        if any(name in item["label"] for item in scene_objects):
            continue
        scene_index = len(scene_objects) + 1
        scene_objects.append({
            "id": str(raw_obj.get("id") or f"scene_obj_{scene_index:03d}"),
            "frame_object_id": str(raw_obj.get("frame_object_id") or raw_obj.get("id") or f"scene_obj_{scene_index:03d}"),
            "label": name.strip(),
            "score": _clamp01(raw_obj.get("confidence", 0.6)),
            "bbox_2d": bbox,
            "category": str(raw_obj.get("category") or "object"),
        })
        if len(scene_objects) >= 10:
            break
    # 确证的目标也要进 scene_objects：object_map 与 WebUI 对象拓扑只吃
    # scene_objects，target_objects 只走目标判定通道。实测
    # search_20260902_211913 VLM 认出了绿色垃圾桶，拓扑 12 个对象节点里却没有
    # 它，target_found.object_id 只能是 null，确认标记只能落在 Place 上。
    # POSSIBLE 也要进：search_20260902_213913 quick 判 POSSIBLE、verify 确认成
    # TARGET_FOUND，只覆盖 PRESENT 时 object_id 依然是 null。无框候选没有几何
    # 位置，不进拓扑。
    if target_objects and target_objects[0]["bbox_2d"] is not None:
        scene_objects.insert(0, {
            "id": target_objects[0]["id"],
            "frame_object_id": target_objects[0]["frame_object_id"],
            "label": target_objects[0]["label"],
            "score": target_objects[0]["score"],
            "bbox_2d": target_objects[0]["bbox_2d"],
            "category": "target",
        })
    # 兼容旧调用：objects 只等于 target_objects，禁止包含普通场景物体。
    objects = list(target_objects)
    matched_ids = [item["id"] for item in target_objects]
    payload = {
        "objects": objects,
        "target_objects": target_objects,
        "scene_summary_zh": str(data.get("reason_zh") or ""),
        "target_decision": {
            "target_text": target_text,
            "is_present": requested_state == TARGET_PRESENT,
            "target_state": requested_state,
            "matched_object_ids": matched_ids if requested_state != TARGET_ABSENT else [],
            "target_candidates": target_objects,
            "match_reason_zh": str(data.get("reason_zh") or ""),
            "confidence": confidence,
        },
        "target_state": requested_state,
        "target_candidates": target_objects,
        "all_objects_count": len(target_objects),
    }
    # 只有在确有小物体列表时才暴露 scene_objects；空列表不要写入，以免
    # semantic_payload_from_quick_target_absence 把“无普通物体”误当成显式空场景。
    if scene_objects:
        payload["scene_objects"] = scene_objects
        payload["scene_objects_light"] = scene_objects
        payload["scene_objects_count"] = len(scene_objects)
    return payload


def quick_target_present(payload: dict, min_score: float = 0.15) -> bool:
    """Return whether a Quick VLM payload passes the target gate.

    The gate is: target_decision.is_present AND at least one target-only object
    AND its score >= min_score.  Ordinary scene objects in ``scene_objects``
    must never make this return True.
    """
    return target_state_is_present(payload, min_score=min_score)


def quick_target_state(payload: dict, min_score: float = 0.15) -> str:
    """Return ABSENT/POSSIBLE/PRESENT for the live target gate."""
    state = target_state_from_payload(payload)
    if state == TARGET_PRESENT and not quick_target_present(payload, min_score):
        return TARGET_POSSIBLE
    return state


def _normalize_quick_bbox(value, width: int, height: int) -> list[float] | None:
    if isinstance(value, dict):
        values = [value.get(key) for key in ("x1", "y1", "x2", "y2")]
    elif isinstance(value, (list, tuple)) and len(value) == 4:
        values = list(value)
    else:
        return None
    try:
        x1, y1, x2, y2 = (float(item) for item in values)
    except (TypeError, ValueError):
        return None
    if max(x1, y1, x2, y2) > 1.0:
        # The model sometimes returns pixel coordinates instead of 0..1.
        x1, x2 = x1 / width, x2 / width
        y1, y2 = y1 / height, y2 / height
    x1, y1, x2, y2 = (
        max(0.0, min(1.0, item)) for item in (x1, y1, x2, y2)
    )
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def _clamp01(value) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0.5
    return max(0.0, min(1.0, numeric))


_QUICK_SYSTEM_PROMPT = """你是机器狗第一人称视觉目标搜索模块。
任务：在图片中寻找目标「{target}」，同时给出画面中少量显著物体（供建图/拓扑使用）。
只输出一个 JSON 对象，不要 Markdown，不要任何其他文字：
- 如果图片中存在该目标：
  {{"target_state": "PRESENT", "found": true, "bbox_2d": [x1, y1, x2, y2], "confidence": 0.8, "name_zh": "目标中文名", "reason_zh": "一句话依据", "scene_objects_light": [...]}}
- 如果类别看起来可能是目标，但颜色/属性/遮挡/框不确定：
  {{"target_state": "POSSIBLE", "found": false, "bbox_2d": [x1, y1, x2, y2], "confidence": 0.45, "name_zh": "候选物体名", "reason_zh": "说明不确定点", "scene_objects_light": [...]}}
- 如果不存在：
  {{"target_state": "ABSENT", "found": false, "reason_zh": "一句话说明画面中有什么，为什么没找到", "scene_objects_light": [...]}}
scene_objects_light 数组：最多 6~10 个画面中最显著的普通物体（不要包含目标本身，不要重复），每个元素为
  {{"name_zh": "物体中文名", "confidence": 0.91, "bbox_2d": [x1, y1, x2, y2], "category": "furniture"}}
约束：
- bbox_2d 必须是 0 到 1 之间的归一化小数坐标 [左上x, 左上y, 右下x, 右下y]，禁止用像素值；取值保留两位小数以缩短输出。
- confidence 取值 0 到 1。
- target_state 必须是 PRESENT、POSSIBLE 或 ABSENT；类别匹配但属性不确定时必须使用 POSSIBLE。
- 目标候选只放进 target_state/found/bbox_2d/name_zh，绝不混入 scene_objects_light。
- scene_objects_light 用于快速建图：宁可少列，不要编造；画面确实没有显著物体时返回空数组。
- 严格只输出合法、完整的 JSON，数组和花括号必须闭合。"""


_VERIFY_SYSTEM_PROMPT = """你是机器狗目标复核模块。
图1是完整环境上下文；图2是候选区域的高清裁剪。候选框归一化坐标为 bbox=[{bbox}]。
请判断图2中的主要物体是什么，以及它是否满足目标「{target}」；同时结合图1检查目标中的关系约束（例如“饮水机旁边”）。
只输出一个 JSON 对象，不要 Markdown，不要任何其他文字：
{{"object_name_zh": "物体中文名", "is_target": true, "confidence": 0.9, "reason_zh": "一句话依据"}}
约束：
- object_name_zh 只写物体名（例如：黑色椅子、灰色书包、黑色背包）。
- is_target 必须是布尔值 true/false。
- confidence 取值 0 到 1。
- reason_zh 用一句中文说明判断依据（例如：框内是带靠背和扶手的椅子，不是书包；图1未看到饮水机，不满足“旁边”关系）。
- 严格只输出 JSON。"""


def _verify_detect(settings, image_path: str, target_text: str,
                   bbox_text: str, model_override: str = "",
                   client: Any | None = None) -> dict:
    """Ask the vision API whether the object inside bbox is the target.

    Uses full-frame context + high-resolution candidate crop.  If the endpoint
    rejects multi-image input, automatically falls back to the full frame only.
    """
    from openai import OpenAI

    from app.llm_clients.siliconflow_client import _load_resized_image_bytes
    from app.utils.json_utils import extract_json_from_text

    if client is None:
        client = OpenAI(
            api_key=settings.siliconflow_api_key,
            base_url=settings.siliconflow_base_url,
            timeout=settings.siliconflow_timeout_seconds,
        )
    model = model_override or settings.vision_model
    image_bytes, mime_type = _load_resized_image_bytes(
        image_path, settings.image_max_side
    )
    image_data_url = (
        f"data:{mime_type};base64,"
        + base64.b64encode(image_bytes).decode("ascii")
    )
    crop_data_url = _make_crop_data_url(image_bytes, bbox_text, settings.image_max_side)
    prompt = _VERIFY_SYSTEM_PROMPT.format(target=target_text, bbox=bbox_text)
    multi_image_content = [
        {"type": "text", "text": prompt},
        {"type": "image_url", "image_url": {"url": image_data_url, "detail": settings.image_detail}},
    ]
    if crop_data_url is not None:
        multi_image_content.append(
            {"type": "image_url", "image_url": {"url": crop_data_url, "detail": "high"}}
        )
    try:
        response = client.chat.completions.create(
            model=model,
            messages=[{"role": "user", "content": multi_image_content}],
            temperature=0.0,
            max_tokens=getattr(settings, "vlm_runtime_verify_max_tokens", 768),
        )
    except Exception:
        # Provider fallback: single full-frame verification.
        response = client.chat.completions.create(
            model=model,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": image_data_url, "detail": settings.image_detail}},
                    ],
                }
            ],
            temperature=0.0,
            max_tokens=getattr(settings, "vlm_runtime_verify_max_tokens", 768),
        )
    raw_content = response.choices[0].message.content
    if not isinstance(raw_content, str) or not raw_content.strip():
        raise ValueError("SiliconFlow verify response was empty")
    data = extract_json_from_text(raw_content)
    if not isinstance(data, dict):
        raise ValueError("SiliconFlow verify response was not a JSON object")
    return {
        "object_name_zh": str(data.get("object_name_zh") or ""),
        "is_target": bool(data.get("is_target", False)),
        "confidence": _clamp01(data.get("confidence", 0.5)),
        "reason_zh": str(data.get("reason_zh") or ""),
    }


def _make_crop_data_url(image_bytes: bytes, bbox_text: str, max_side: int) -> str | None:
    """Create a base64 JPEG crop from a normalized bbox string (x1,y1,x2,y2)."""
    try:
        from io import BytesIO

        with Image.open(BytesIO(image_bytes)) as image:
            width, height = image.size
            values = [float(v) for v in bbox_text.split(",")]
            if len(values) != 4:
                return None
            x1, y1, x2, y2 = values
            # 1.25x expansion around the box, clamped to image.
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = max(x2 - x1, 0.05)
            bh = max(y2 - y1, 0.05)
            exp_x = min(0.5, bw * 0.625)
            exp_y = min(0.5, bh * 0.625)
            x1 = max(0.0, cx - bw / 2 - exp_x)
            x2 = min(1.0, cx + bw / 2 + exp_x)
            y1 = max(0.0, cy - bh / 2 - exp_y)
            y2 = min(1.0, cy + bh / 2 + exp_y)
            if x2 <= x1 or y2 <= y1:
                return None
            box = (int(x1 * width), int(y1 * height), int(x2 * width), int(y2 * height))
            crop = image.convert("RGB").crop(box)
            crop.thumbnail((max_side, max_side))
            buffer = BytesIO()
            crop.save(buffer, format="JPEG", quality=90)
            encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
            return f"data:image/jpeg;base64,{encoded}"
    except Exception:
        return None


def main() -> int:
    args = parse_args()
    if not args.target.strip():
        print("target must not be empty", file=sys.stderr)
        return 2

    os.chdir(PROJECT_ROOT)
    try:
        from app.llm_clients.siliconflow_client import SiliconFlowVisionClient
    except Exception as exc:
        print(
            "Missing SiliconFlow runtime dependencies for this Python "
            f"interpreter. Original error: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2

    try:
        if args.verify:
            if not args.bbox:
                print("--bbox x1,y1,x2,y2 is required with --verify",
                      file=sys.stderr)
                return 2
            from app.config import get_settings

            result = _verify_detect(
                get_settings(),
                args.image,
                args.target,
                args.bbox,
                args.model,
            )
            payload = result
        elif args.quick:
            from app.config import get_settings

            result = _quick_detect(
                get_settings(),
                args.image,
                args.target,
                args.extra_instructions,
                args.model,
            )
        else:
            client = SiliconFlowVisionClient()
            result = client.analyze_scene(
                args.image,
                args.target,
                extra_instructions=args.extra_instructions or None,
            )
    except Exception as exc:
        print(
            f"SiliconFlow vision API call failed: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 1

    if args.verify or args.quick:
        payload = result
    else:
        payload = {
            "objects": _matched_objects(result, args.target),
            # Preserve the full factual observation for the event-driven
            # semantic observer. The legacy target-only ``objects`` contract
            # above remains unchanged.
            "scene_objects": [
                item.model_dump(mode="json") if hasattr(item, "model_dump") else item
                for item in (result.get("objects") or [])
            ],
            "scene_relations": [
                item.model_dump(mode="json") if hasattr(item, "model_dump") else item
                for item in (result.get("relations") or [])
            ],
            "scene_summary_zh": result.get("scene_summary_zh", ""),
            "target_decision": result.get("target_decision", {}),
            "all_objects_count": len(result.get("objects") or []),
        }
    _write_payload(args.output, payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
