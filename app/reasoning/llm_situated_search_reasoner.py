"""LLM-first situated search reasoning with a safe local fallback."""

from __future__ import annotations

from typing import Any

from openai import OpenAI

from app.config import Settings, get_settings
from app.prompts import build_situated_search_reasoning_prompt
from app.schemas import (
    Actionability,
    LLMReasoningRequest,
    LLMReasoningResult,
    LLMSearchHypothesis,
    NodeObservationStatus,
    QuadrupedActionPrimitive,
)
from app.utils.json_utils import extract_json_from_text


class LLMSituatedSearchReasoner:
    def __init__(
        self,
        client: Any | None = None,
        settings: Settings | None = None,
        auto_create_client: bool = True,
    ) -> None:
        self.settings = settings or get_settings()
        self.client = client
        if (
            self.client is None
            and auto_create_client
            and self.settings.siliconflow_api_key
        ):
            self.client = OpenAI(
                api_key=self.settings.siliconflow_api_key,
                base_url=self.settings.siliconflow_base_url,
                timeout=self.settings.siliconflow_reasoning_timeout_seconds,
            )

    def reason(self, request: LLMReasoningRequest) -> LLMReasoningResult:
        if request.observed_facts.target_observed:
            return _observed_target_result(request)
        if self.client is None:
            return self._fallback_result(request, RuntimeError("未配置推理模型 API"))
        try:
            messages = build_situated_search_reasoning_prompt(request)
            response = self.client.chat.completions.create(
                model=self.settings.reasoning_model,
                messages=messages,
                temperature=self.settings.llm_reasoning_temperature,
                max_tokens=self.settings.siliconflow_reasoning_max_tokens,
            )
            content = response.choices[0].message.content
            if not isinstance(content, str) or not content.strip():
                raise ValueError("LLM 情境推理返回空内容")
            result = self._parse_json_response(content)
            result = _sanitize_result(result, request)
            return result.model_copy(
                update={
                    "hypotheses": result.hypotheses[: request.max_hypotheses],
                    "reasoning_available": True,
                    "error_message": None,
                }
            )
        except Exception as exc:
            return self._fallback_result(request, exc)

    @staticmethod
    def _parse_json_response(text: str) -> LLMReasoningResult:
        payload = extract_json_from_text(text)
        return LLMReasoningResult.model_validate(payload)

    def _fallback_result(
        self,
        request: LLMReasoningRequest,
        error: Exception,
    ) -> LLMReasoningResult:
        anchors = sorted(
            request.observed_facts.visible_anchors,
            key=lambda item: (item.stable, item.confidence),
            reverse=True,
        )
        hypotheses: list[LLMSearchHypothesis] = []
        for index, anchor in enumerate(anchors[: request.max_hypotheses], start=1):
            region = anchor.image_region.split("/", 1)[0]
            strategy = _strategy_for_region(region)
            hypotheses.append(
                LLMSearchHypothesis(
                    hypothesis_id=f"fallback_hyp_{index:03d}",
                    target_name=request.target_text,
                    candidate_region_zh=f"{anchor.label_zh}所在的可见方向",
                    candidate_region_type="visible_anchor_viewpoint",
                    image_region_hint=anchor.image_region,
                    supporting_visible_anchor_ids=[anchor.object_id],
                    supporting_visible_anchor_names=[anchor.label_zh],
                    human_like_rationale_zh=(
                        "推理模型当前不可用；仅依据视觉锚点选择信息增益较高的新视角，"
                        "不对目标位置作常识性断言。"
                    ),
                    expected_visual_cues_zh=[f"目标本体或与{anchor.label_zh}相邻的可见线索"],
                    suggested_detector_prompts_en=[],
                    suggested_verification_question_zh=(
                        f"转向并重新观察{anchor.label_zh}方向后，是否能视觉确认目标？"
                    ),
                    confidence=max(0.2, min(0.55, anchor.confidence * 0.55)),
                    uncertainty_zh="这是无 LLM 时的视觉锚点降级建议。",
                    actionability=Actionability.NEEDS_REOBSERVATION,
                    quadruped_view_strategy=strategy,
                    safety_notes_zh=["只切换观察视角，不把该候选标记为已找到。"],
                    should_not_mark_found=True,
                    max_executable_distance_m=(
                        request.capability_contract.max_executable_distance_m
                    ),
                    execution_assumption=request.capability_contract.execution_assumption,
                )
            )
        if not hypotheses:
            hypotheses.append(
                LLMSearchHypothesis(
                    hypothesis_id="fallback_hyp_001",
                    target_name=request.target_text,
                    candidate_region_zh="当前视野及左右盲区",
                    candidate_region_type="global_scan",
                    human_like_rationale_zh="当前缺少可靠视觉锚点，先进行保守扫描。",
                    expected_visual_cues_zh=["目标本体或明确目标标识"],
                    suggested_detector_prompts_en=[],
                    suggested_verification_question_zh="扫描后是否出现可视觉确认的目标？",
                    confidence=0.25,
                    uncertainty_zh="缺少视觉锚点且推理模型不可用。",
                    actionability=Actionability.NEEDS_REOBSERVATION,
                    quadruped_view_strategy=[
                        QuadrupedActionPrimitive.SCAN_LEFT_TO_RIGHT,
                        QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                    ],
                    safety_notes_zh=["保持原地扫描。"],
                    max_executable_distance_m=(
                        request.capability_contract.max_executable_distance_m
                    ),
                    execution_assumption=request.capability_contract.execution_assumption,
                )
            )
        return LLMReasoningResult(
            scene_interpretation_zh=request.observed_facts.scene_summary_zh,
            target_search_logic_zh="情境推理不可用，已降级为基于视觉锚点的保守重观测。",
            hypotheses=hypotheses,
            global_uncertainty_zh="没有获得大模型动态常识推理结果。",
            recommended_next_observation_zh=(
                hypotheses[0].suggested_verification_question_zh
            ),
            no_target_found_policy_zh=(
                "目标未被视觉确认；所有候选保持 inferred，不能标记为 found。"
            ),
            recommended_motion_horizon_m=None,
            motion_horizon_reason_zh="LLM 不可用，由 Motion Horizon Planner 使用规则降级距离。",
            motion_profile_hint="platform_assisted_fallback",
            requires_stop_after_motion=True,
            reasoning_available=False,
            error_message=str(error),
        )


def _strategy_for_region(region: str) -> list[QuadrupedActionPrimitive]:
    if region == "left":
        return [
            QuadrupedActionPrimitive.TURN_LEFT,
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
        ]
    if region == "right":
        return [
            QuadrupedActionPrimitive.TURN_RIGHT,
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
        ]
    return [
        QuadrupedActionPrimitive.CENTER_VIEW_ON_REGION,
        QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
    ]


def _observed_target_result(request: LLMReasoningRequest) -> LLMReasoningResult:
    hypothesis = LLMSearchHypothesis(
        hypothesis_id="observed_target_001",
        target_name=request.target_text,
        status=NodeObservationStatus.OBSERVED,
        candidate_region_zh="当前视觉确认区域",
        candidate_region_type="observed_target",
        supporting_visible_anchor_ids=[],
        supporting_visible_anchor_names=[],
        human_like_rationale_zh="输入视觉事实已经确认目标。",
        expected_visual_cues_zh=request.observed_facts.target_evidence,
        suggested_detector_prompts_en=[],
        suggested_verification_question_zh="保持安全距离并停止重观测目标。",
        confidence=1.0,
        uncertainty_zh="仍需真实导航安全模块确认可达性。",
        actionability=Actionability.NEEDS_REOBSERVATION,
        quadruped_view_strategy=[
            QuadrupedActionPrimitive.STOP_AND_REOBSERVE
        ],
        safety_notes_zh=["不执行精细接近或操作。"],
        should_not_mark_found=False,
        max_executable_distance_m=request.capability_contract.max_executable_distance_m,
        execution_assumption=request.capability_contract.execution_assumption,
    )
    return LLMReasoningResult(
        scene_interpretation_zh=request.observed_facts.scene_summary_zh,
        target_search_logic_zh="目标已由视觉模块确认，不需要用推理替代视觉证据。",
        hypotheses=[hypothesis],
        global_uncertainty_zh="目标可见不等于目标可安全到达。",
        recommended_next_observation_zh="停止并重新观察目标区域。",
        no_target_found_policy_zh="不适用：目标已有视觉证据。",
        recommended_motion_horizon_m=0.5,
        motion_horizon_reason_zh="目标已视觉确认，仅保留短距离确认或停止重观测。",
        motion_profile_hint="target_candidate_confirm",
        requires_stop_after_motion=True,
    )


def _sanitize_result(
    result: LLMReasoningResult,
    request: LLMReasoningRequest,
) -> LLMReasoningResult:
    anchors = {
        item.object_id: item for item in request.observed_facts.visible_anchors
    }
    profile = request.target_profile or {}
    fallback_prompts = _dedupe_text(
        [
            *list(profile.get("primary_labels_en") or []),
            *list(profile.get("aliases_en") or []),
            *list(profile.get("en_terms") or []),
        ]
    )[:8]
    target_label = str(
        profile.get("canonical_name_zh") or request.target_text
    )
    hypotheses: list[LLMSearchHypothesis] = []
    for index, hypothesis in enumerate(result.hypotheses, start=1):
        anchor_ids = [
            item
            for item in hypothesis.supporting_visible_anchor_ids
            if item in anchors
        ]
        anchor_names = [anchors[item].label_zh for item in anchor_ids]
        prompts: list[str] = []
        seen: set[str] = set()
        for value in hypothesis.suggested_detector_prompts_en:
            normalized = " ".join(str(value).strip().lower().split())
            if normalized and normalized not in seen:
                seen.add(normalized)
                prompts.append(normalized)
        if not prompts:
            prompts = fallback_prompts
        expected_cues = list(hypothesis.expected_visual_cues_zh)
        if not expected_cues:
            expected_cues = [f"可视觉确认的{target_label}本体、轮廓或标识"]
        status = hypothesis.status
        should_not_mark_found = hypothesis.should_not_mark_found
        if not request.observed_facts.target_observed:
            status = (
                NodeObservationStatus.UNREACHABLE
                if hypothesis.actionability
                in {
                    Actionability.NEEDS_HUMAN,
                    Actionability.UNSAFE_OR_IMPOSSIBLE,
                }
                else NodeObservationStatus.INFERRED
            )
            should_not_mark_found = True
        hypotheses.append(
            hypothesis.model_copy(
                update={
                    "hypothesis_id": hypothesis.hypothesis_id
                    or f"hyp_{index:03d}",
                    "supporting_visible_anchor_ids": anchor_ids,
                    "supporting_visible_anchor_names": anchor_names,
                    "suggested_detector_prompts_en": prompts,
                    "expected_visual_cues_zh": expected_cues,
                    "candidate_region_type": (
                        hypothesis.candidate_region_type
                        if anchor_ids
                        else "unanchored_hypothesis"
                    ),
                    "status": status,
                    "should_not_mark_found": should_not_mark_found,
                    "max_executable_distance_m": (
                        hypothesis.max_executable_distance_m
                        or request.capability_contract.max_executable_distance_m
                    ),
                    "execution_assumption": (
                        hypothesis.execution_assumption
                        or request.capability_contract.execution_assumption
                    ),
                }
            )
        )
    return result.model_copy(update={"hypotheses": hypotheses})


def _dedupe_text(values: list[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        normalized = " ".join(str(value).strip().lower().split())
        if normalized and normalized not in seen:
            seen.add(normalized)
            result.append(normalized)
    return result
