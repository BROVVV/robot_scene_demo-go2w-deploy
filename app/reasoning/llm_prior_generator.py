"""Runtime LLM commonsense hypotheses for embodied target search."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any

from app.config import Settings, get_settings
from app.utils.json_utils import extract_json_from_text


PRIOR_SOURCE = "llm_runtime_commonsense"


@dataclass(frozen=True)
class LLMPriorInput:
    target: str
    scene_summary: str = ""
    observed_objects: list[dict[str, Any]] = field(default_factory=list)
    observed_relations: list[dict[str, Any]] = field(default_factory=list)
    observation_memory_summaries: list[dict[str, Any]] = field(default_factory=list)
    robot_capabilities: dict[str, Any] = field(default_factory=dict)
    language: str = "zh"


class LLMPriorGenerator:
    """Ask an LLM for search hypotheses without allowing it to confirm targets."""

    def __init__(
        self,
        settings: Settings | None = None,
        client: Any | None = None,
        auto_create_client: bool = True,
    ) -> None:
        self.settings = settings or get_settings()
        self.client = client
        if (
            self.client is None
            and auto_create_client
            and self.settings.siliconflow_api_key
        ):
            try:
                from openai import OpenAI
            except ImportError:
                # System Python may not have openai; the VLM daemon / Conda
                # worker is responsible for real API calls.  Keep the prior
                # generator fail-soft in the ROS process.
                self.client = None
            else:
                self.client = OpenAI(
                    api_key=self.settings.siliconflow_api_key,
                    base_url=self.settings.siliconflow_base_url,
                    timeout=self.settings.siliconflow_reasoning_timeout_seconds,
                )

    def generate(self, prior_input: LLMPriorInput) -> dict[str, Any]:
        if not self.settings.llm_commonsense_prior_enabled:
            return _disabled_result(prior_input.target)
        if self.client is None:
            return _unavailable_result(prior_input.target, "LLM API unavailable")
        try:
            response = self.client.chat.completions.create(
                model=self.settings.reasoning_model,
                messages=_messages(prior_input, self.settings),
                temperature=self.settings.llm_reasoning_temperature,
                max_tokens=min(1600, self.settings.siliconflow_reasoning_max_tokens),
            )
            content = response.choices[0].message.content
            if not isinstance(content, str) or not content.strip():
                raise ValueError("LLM prior response was empty.")
            payload = extract_json_from_text(content)
            return sanitize_llm_prior_result(payload, prior_input.target, self.settings)
        except Exception as exc:
            return _unavailable_result(prior_input.target, f"{type(exc).__name__}: {exc}")


def sanitize_llm_prior_result(
    payload: dict[str, Any],
    target: str,
    settings: Settings | None = None,
) -> dict[str, Any]:
    config = settings or get_settings()
    hypotheses = []
    for index, item in enumerate(payload.get("commonsense_hypotheses") or [], start=1):
        if not isinstance(item, dict):
            continue
        hypotheses.append(
            {
                "hypothesis_id": str(
                    item.get("hypothesis_id") or f"hyp_{index:03d}"
                ),
                "anchor_type": str(
                    item.get("anchor_type")
                    or "observed_object_or_place_or_visual_region"
                ),
                "anchor_label": str(item.get("anchor_label") or ""),
                "anchor_object_id": item.get("anchor_object_id"),
                "priority": _score(item.get("priority"), 0.5),
                "reason_zh": str(item.get("reason_zh") or item.get("reason") or ""),
                "required_visual_check": str(
                    item.get("required_visual_check")
                    or "需要后续视觉检测 bbox/crop/mask/frame 证据。"
                ),
                "status": "hypothesis",
                "evidence_type": "llm_commonsense",
            }
        )
        if len(hypotheses) >= config.llm_prior_max_hypotheses:
            break
    prompts = _dedupe_text(
        [
            str(item)
            for item in payload.get("suggested_detector_prompts") or []
            if str(item).strip()
        ]
    )[: config.llm_prior_max_detector_prompts]
    plans = [
        item
        for item in payload.get("next_view_plan") or []
        if isinstance(item, dict)
    ][: config.llm_prior_max_hypotheses]
    return {
        "enabled": True,
        "available": True,
        "target": str(payload.get("target") or target),
        "prior_source": PRIOR_SOURCE,
        "can_confirm_target": False,
        "commonsense_hypotheses": hypotheses,
        "suggested_detector_prompts": prompts,
        "next_view_plan": plans,
        "uncertainty_zh": str(
            payload.get("uncertainty_zh")
            or "这些只是运行时常识搜索假设，不能证明目标已经出现。"
        ),
    }


def _messages(prior_input: LLMPriorInput, settings: Settings) -> list[dict[str, str]]:
    schema = {
        "target": prior_input.target,
        "prior_source": PRIOR_SOURCE,
        "can_confirm_target": False,
        "commonsense_hypotheses": [
            {
                "hypothesis_id": "hyp_001",
                "anchor_type": "observed_object_or_place_or_visual_region",
                "anchor_label": "可见锚点或待观察区域",
                "anchor_object_id": None,
                "priority": 0.5,
                "reason_zh": "简洁原因",
                "required_visual_check": "需要检查的视觉线索",
                "status": "hypothesis",
                "evidence_type": "llm_commonsense",
            }
        ],
        "suggested_detector_prompts": ["open vocabulary visual term"],
        "next_view_plan": [
            {
                "action": "turn_toward",
                "target_anchor_object_id": None,
                "reason_zh": "下一观察理由",
                "expected_information_gain": 0.5,
            }
        ],
        "uncertainty_zh": "这些只是常识搜索假设，不能证明目标已经出现。",
    }
    system = """你是具身视觉导航系统中的运行时常识推理模块。

你的任务：根据用户目标、当前视觉观察、历史观察记忆和机器人能力，生成搜索假设。

严格规则：
1. 你可以使用自己的通用常识生成搜索假设。
2. 你不能把假设当成事实。
3. 你不能确认目标已经出现。
4. 只有视觉检测模块提供 bbox/crop/mask/frame 等证据后，目标才能被确认。
5. 如果当前画面中没有相关锚点，请说明需要主动观察哪些区域，而不是编造已观察物体。
6. 输出必须是 JSON，不要输出 Markdown。
7. 每个假设必须标记 status="hypothesis" 和 evidence_type="llm_commonsense"。
8. can_confirm_target 必须为 false。"""
    user = (
        "输入："
        f"{json.dumps(asdict(prior_input), ensure_ascii=False)}\n"
        "请输出符合 schema 的 JSON："
        f"{json.dumps(schema, ensure_ascii=False)}\n"
        f"最多 {settings.llm_prior_max_hypotheses} 个假设，"
        f"最多 {settings.llm_prior_max_detector_prompts} 个检测 prompt。"
    )
    return [{"role": "system", "content": system}, {"role": "user", "content": user}]


def _disabled_result(target: str) -> dict[str, Any]:
    return {
        "enabled": False,
        "available": False,
        "target": target,
        "prior_source": PRIOR_SOURCE,
        "can_confirm_target": False,
        "commonsense_hypotheses": [],
        "suggested_detector_prompts": [],
        "next_view_plan": [],
        "fallback_mode": "visual_only_no_handcrafted_prior",
    }


def _unavailable_result(target: str, error: str) -> dict[str, Any]:
    return {
        "enabled": True,
        "available": False,
        "target": target,
        "prior_source": PRIOR_SOURCE,
        "can_confirm_target": False,
        "error": error,
        "commonsense_hypotheses": [],
        "suggested_detector_prompts": [],
        "next_view_plan": [],
        "fallback_mode": "visual_only_no_handcrafted_prior",
    }


def _score(value: Any, default: float) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return default


def _dedupe_text(values: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        normalized = " ".join(value.strip().lower().split())
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        result.append(normalized)
    return result
