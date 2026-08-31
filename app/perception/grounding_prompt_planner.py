"""LLM-first GroundingDINO prompt expansion."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import json
import re
from typing import Any

from openai import OpenAI

from app.config import Settings, get_settings
from app.task_understanding.prompt_templates import (
    GROUNDING_PROMPT_EXPANSION_SYSTEM_PROMPT,
    GROUNDING_PROMPT_EXPANSION_USER_PROMPT_TEMPLATE,
    GROUNDING_PROMPT_RETRY_SYSTEM_PROMPT,
    GROUNDING_PROMPT_RETRY_USER_PROMPT_TEMPLATE,
)
from app.task_understanding.schemas import GroundingPromptPlan
from app.utils.json_utils import extract_json_from_text


class GroundingPromptError(RuntimeError):
    """Raised when GroundingDINO cannot receive a valid open-vocabulary prompt."""


class GroundingPromptPlanner:
    def __init__(
        self,
        llm_client: Any | None = None,
        settings: Settings | None = None,
        max_terms: int = 24,
        min_terms: int = 3,
    ) -> None:
        self.settings = settings or get_settings()
        self.llm_client = llm_client
        self.max_terms = max_terms
        self.min_terms = min_terms

    def build(
        self,
        parsed_task: Any,
        navigation_task: Any | None = None,
        target_profile: Any | None = None,
        visual_summary: Any | None = None,
    ) -> GroundingPromptPlan:
        task_payload = {
            "parsed_task": self._to_payload(parsed_task),
            "navigation_task": self._to_payload(navigation_task),
            "visual_summary": self._to_payload(visual_summary),
        }
        task_json = json.dumps(task_payload, ensure_ascii=False, indent=2)
        target_profile_json = json.dumps(
            self._to_payload(target_profile),
            ensure_ascii=False,
            indent=2,
        )
        response = self._call_llm(
            system_prompt=GROUNDING_PROMPT_EXPANSION_SYSTEM_PROMPT,
            user_prompt=GROUNDING_PROMPT_EXPANSION_USER_PROMPT_TEMPLATE.format(
                task_json=task_json,
                target_profile_json=target_profile_json,
            ),
        )
        data = self._extract_json(response)
        plan = self._parse_plan(data=data, parsed_task=parsed_task)
        plan = self._normalize_plan(plan)
        self._validate_plan(plan)
        return plan

    def build_retry(
        self,
        parsed_task: Any,
        previous_prompt: str,
        detection_summary: dict[str, Any],
        target_profile: Any | None = None,
        previous_plan: GroundingPromptPlan | None = None,
    ) -> GroundingPromptPlan:
        task_payload = {
            "parsed_task": self._to_payload(parsed_task),
            "target_profile": self._to_payload(target_profile),
        }
        response = self._call_llm(
            system_prompt=GROUNDING_PROMPT_RETRY_SYSTEM_PROMPT,
            user_prompt=GROUNDING_PROMPT_RETRY_USER_PROMPT_TEMPLATE.format(
                raw_task=self._raw_task(parsed_task),
                task_json=json.dumps(task_payload, ensure_ascii=False, indent=2),
                previous_prompt=previous_prompt,
                detection_summary_json=json.dumps(
                    detection_summary,
                    ensure_ascii=False,
                    indent=2,
                ),
            ),
        )
        data = self._extract_json(response)
        retry_prompt = str(data.get("retry_prompt") or "").strip()
        added_terms = self._string_list(data.get("added_terms_en"))
        terms = self._terms_from_prompt(retry_prompt)
        if not terms:
            terms = added_terms
        base = previous_plan or GroundingPromptPlan()
        plan = GroundingPromptPlan(
            raw_task=base.raw_task or self._raw_task(parsed_task),
            primary_intent=base.primary_intent or self._primary_intent(parsed_task),
            target_name_zh=base.target_name_zh,
            target_name_en=base.target_name_en,
            target_category=base.target_category,
            grounding_strategy=base.grounding_strategy,
            direct_terms_en=base.direct_terms_en,
            proxy_object_terms_en=[*base.proxy_object_terms_en, *added_terms],
            context_anchor_terms_en=[
                *base.context_anchor_terms_en,
                *[term for term in terms if term not in added_terms],
            ],
            state_terms_en=base.state_terms_en,
            negative_terms_en=base.negative_terms_en,
            grounding_prompt=retry_prompt,
            prompt_source="llm_grounding_prompt_retry",
            prompt_reason_zh=str(data.get("reason_zh") or "").strip(),
            requires_proxy_objects=base.requires_proxy_objects,
            requires_scene_confirmation=base.requires_scene_confirmation,
            requires_state_verification=base.requires_state_verification,
            retry_count=base.retry_count + 1,
            raw_llm_response=data,
        )
        plan = self._normalize_plan(plan)
        self._validate_plan(plan)
        return plan

    def _parse_plan(self, data: dict[str, Any], parsed_task: Any) -> GroundingPromptPlan:
        return GroundingPromptPlan(
            raw_task=self._raw_task(parsed_task),
            primary_intent=self._primary_intent(parsed_task),
            target_name_zh=str(data.get("target_name_zh") or "").strip(),
            target_name_en=str(data.get("target_name_en") or "").strip(),
            target_category=str(data.get("target_category") or "unknown").strip(),
            grounding_strategy=str(data.get("grounding_strategy") or "unknown").strip(),
            direct_terms_en=self._string_list(data.get("direct_terms_en")),
            proxy_object_terms_en=self._string_list(data.get("proxy_object_terms_en")),
            context_anchor_terms_en=self._string_list(data.get("context_anchor_terms_en")),
            state_terms_en=self._string_list(data.get("state_terms_en")),
            negative_terms_en=self._string_list(data.get("negative_terms_en")),
            grounding_prompt=str(data.get("grounding_prompt") or "").strip(),
            prompt_source="llm_grounding_prompt_expansion",
            prompt_reason_zh=str(data.get("reason_zh") or "").strip(),
            requires_proxy_objects=bool(data.get("requires_proxy_objects", False)),
            requires_scene_confirmation=bool(
                data.get("requires_scene_confirmation", False)
            ),
            requires_state_verification=bool(
                data.get("requires_state_verification", False)
            ),
            raw_llm_response=data,
        )

    def _normalize_plan(self, plan: GroundingPromptPlan) -> GroundingPromptPlan:
        all_terms = []
        all_terms.extend(plan.direct_terms_en)
        all_terms.extend(plan.proxy_object_terms_en)
        all_terms.extend(plan.context_anchor_terms_en)
        all_terms.extend(plan.state_terms_en)
        all_terms.extend(self._terms_from_prompt(plan.grounding_prompt))

        terms: list[str] = []
        seen: set[str] = set()
        for term in all_terms:
            normalized = self._normalize_term(term)
            if not normalized or normalized in _ACTION_TERMS:
                continue
            if normalized not in seen:
                seen.add(normalized)
                terms.append(normalized)

        terms = terms[: self.max_terms]
        plan.grounding_prompt = " . ".join(terms)
        if plan.grounding_prompt:
            plan.grounding_prompt += " ."
        plan.is_valid_for_grounding_dino = bool(plan.grounding_prompt.strip())
        if len(terms) < self.min_terms:
            plan.warnings.append(
                f"GroundingDINO prompt has only {len(terms)} terms; detection may be low recall."
            )
        return plan

    def _validate_plan(self, plan: GroundingPromptPlan) -> None:
        if plan.grounding_prompt.strip():
            return
        raise GroundingPromptError(
            "GroundingDINO prompt is empty. GroundingDINO requires concrete object "
            "or anchor terms."
        )

    def _call_llm(self, system_prompt: str, user_prompt: str) -> str:
        client = self.llm_client
        if client is None:
            if not self.settings.siliconflow_api_key:
                raise GroundingPromptError(
                    "Grounding prompt expansion failed because LLM is unavailable. "
                    "GroundingDINO detection was not executed."
                )
            client = OpenAI(
                api_key=self.settings.siliconflow_api_key,
                base_url=self.settings.siliconflow_base_url,
                timeout=self.settings.siliconflow_timeout_seconds,
            )
        if hasattr(client, "chat") and callable(client.chat):
            response = client.chat(system_prompt=system_prompt, user_prompt=user_prompt)
            return self._response_text(response)
        response = client.chat.completions.create(
            model=self.settings.reasoning_model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            temperature=0.0,
            max_tokens=min(1200, self.settings.siliconflow_max_tokens),
        )
        return self._response_text(response)

    def _extract_json(self, response: str) -> dict[str, Any]:
        try:
            payload = extract_json_from_text(response)
        except ValueError as exc:
            raise GroundingPromptError(
                f"Grounding prompt expansion returned invalid JSON: {response}"
            ) from exc
        if not isinstance(payload, dict):
            raise GroundingPromptError("Grounding prompt expansion JSON must be an object.")
        return payload

    def _to_payload(self, value: Any) -> Any:
        if value is None:
            return None
        if hasattr(value, "model_dump"):
            return value.model_dump(mode="json")
        if is_dataclass(value):
            return asdict(value)
        if isinstance(value, (dict, list, str, int, float, bool)):
            return value
        if hasattr(value, "to_dict"):
            return value.to_dict()
        return str(value)

    def _response_text(self, response: Any) -> str:
        if isinstance(response, dict):
            return json.dumps(response, ensure_ascii=False)
        if isinstance(response, str):
            text = response.strip()
        else:
            try:
                text = response.choices[0].message.content.strip()
            except (AttributeError, IndexError, TypeError) as exc:
                raise GroundingPromptError(
                    "Grounding prompt expansion response did not contain text."
                ) from exc
        if not text:
            raise GroundingPromptError("Grounding prompt expansion response was empty.")
        return text

    @staticmethod
    def _string_list(value: Any) -> list[str]:
        if not isinstance(value, list):
            return []
        result: list[str] = []
        seen: set[str] = set()
        for item in value:
            text = str(item).strip()
            key = text.lower()
            if text and key not in seen:
                seen.add(key)
                result.append(text)
        return result

    @staticmethod
    def _terms_from_prompt(prompt: str) -> list[str]:
        return [item.strip() for item in re.split(r"\s*\.\s*", prompt) if item.strip()]

    @staticmethod
    def _normalize_term(term: Any) -> str:
        return " ".join(
            re.sub(r"[^a-z0-9 -]+", " ", str(term).lower()).split()
        ).strip(" .")

    @staticmethod
    def _raw_task(parsed_task: Any) -> str:
        return str(getattr(parsed_task, "raw_task", "") or "").strip()

    @staticmethod
    def _primary_intent(parsed_task: Any) -> str:
        value = getattr(parsed_task, "primary_intent", "unknown")
        return str(getattr(value, "value", value) or "unknown")


_ACTION_TERMS = {
    "find",
    "go",
    "inspect",
    "search",
    "navigate",
    "look for",
    "move",
    "approach",
}
