"""Knowledge-aware orchestration on top of the existing SceneAnalyzer."""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from typing import Callable
from uuid import uuid4

from app.config import DEFAULT_OUTPUT_DIR, Settings, get_settings
from app.knowledge.kb_updater import update_knowledge_from_scene
from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.memory.llm_experience_memory import LLMExperienceMemory
from app.planning.actionability_gate import validate_hypotheses
from app.planning.motion_horizon import (
    estimate_motion_horizon,
    has_target_candidate,
    infer_scene_type_from_result,
)
from app.planning.quadruped_viewpoint_planner import (
    plan_quadruped_viewpoints,
    quadruped_plan_to_task_plan,
)
from app.planning.task_planner import plan_task
from app.reasoning.llm_situated_search_reasoner import LLMSituatedSearchReasoner
from app.reasoning.observed_facts_builder import build_observed_scene_facts
from app.reasoning.scene_reasoner import reason_about_scene
from app.reasoning.task_parser import parse_robot_task
from app.reasoning.visual_grounding_gate import (
    apply_visual_grounding_gate,
    collect_dynamic_detector_prompts,
    merge_visual_retry_scene,
)
from app.schemas import (
    KnowledgeAwareSceneResult,
    LLMReasoningRequest,
    SceneAnalysisResult,
    SceneHypothesis,
    SpatialExperienceMemory,
    default_quadruped_capability,
)
from app.services.predictive_scene_graph import build_reasoned_predictive_scene_graph
from app.services.psg_builder import build_predictive_scene_graph
from app.services.scene_analyzer import SceneAnalyzer


class KnowledgeAwareAnalyzer:
    def __init__(
        self,
        scene_analyzer: SceneAnalyzer | None = None,
        kb_dir: str | Path = "data/scene_kb",
        output_dir: str | Path = DEFAULT_OUTPUT_DIR,
        update_kb: bool = True,
        settings: Settings | None = None,
        enable_llm_reasoning: bool | None = None,
        enable_reasoning_memory: bool | None = None,
        quadruped_mode: bool = True,
        allow_remote_reasoning: bool = False,
        hide_unexecutable_actions: bool = False,
        visual_retry_callback: Callable[[list[str]], SceneAnalysisResult] | None = None,
    ) -> None:
        self.scene_analyzer = scene_analyzer
        self.kb_dir = Path(kb_dir)
        self.output_dir = Path(output_dir)
        self.update_kb = update_kb
        self.settings = settings or get_settings()
        self.enable_llm_reasoning = (
            self.settings.enable_llm_situated_reasoning
            if enable_llm_reasoning is None
            else enable_llm_reasoning
        )
        self.enable_reasoning_memory = (
            self.settings.enable_llm_reasoning_memory
            if enable_reasoning_memory is None
            else enable_reasoning_memory
        )
        self.quadruped_mode = quadruped_mode
        self.allow_remote_reasoning = allow_remote_reasoning
        self.hide_unexecutable_actions = hide_unexecutable_actions
        self.visual_retry_callback = visual_retry_callback

    def analyze(self, image_path: str, target_text: str) -> KnowledgeAwareSceneResult:
        if self.scene_analyzer is None:
            raise ValueError("KnowledgeAwareAnalyzer.analyze requires a SceneAnalyzer.")
        base_scene = self.scene_analyzer.analyze(image_path, target_text)
        return self.enrich_base_scene(base_scene, target_text)

    def enrich_base_scene(
        self,
        base_scene: SceneAnalysisResult,
        target_text: str,
    ) -> KnowledgeAwareSceneResult:
        parsed_task = parse_robot_task(target_text)
        static_kb_used = bool(self.settings.static_knowledge_base_enabled)
        retrieved_knowledge = (
            retrieve_relevant_knowledge(
                target_text=target_text,
                room_type=(
                    _infer_room_type_hint(base_scene)
                    if self.settings.handwritten_room_priors_enabled
                    else None
                ),
                location_hint=(
                    (parsed_task.target_location or parsed_task.scope)
                    if self.settings.handwritten_location_priors_enabled
                    else None
                ),
                kb_dir=self.kb_dir,
            )
            if static_kb_used
            else []
        )
        psg = build_predictive_scene_graph(base_scene, retrieved_knowledge, parsed_task)
        observed_facts = None
        llm_reasoning = None
        reasoned_psg = None
        quadruped_plan = None
        motion_horizon_decision = None
        actionability_notes: list[str] = []
        retrieved_experiences: list[dict] = []
        experience_writes: list[dict] = []
        visual_grounding_report: dict | None = None

        if self.enable_llm_reasoning and self.quadruped_mode:
            observed_facts = build_observed_scene_facts(base_scene, target_text)
            capability = default_quadruped_capability(
                max_forward_step_m=self.settings.quadruped_max_forward_step_m,
                max_executable_distance_m=(
                    self.settings.motion_absolute_max_step_m
                    if self.settings.platform_obstacle_avoidance_assumed
                    else self.settings.motion_strict_safe_max_step_m
                ),
                platform_obstacle_avoidance_assumed=(
                    self.settings.platform_obstacle_avoidance_assumed
                ),
                can_manipulate=self.settings.quadruped_can_manipulate,
                can_open_container=self.settings.quadruped_can_open_container,
                can_look_down=self.settings.quadruped_can_look_down,
            )
            memory = LLMExperienceMemory(self.settings.llm_experience_memory_path)
            if self.enable_reasoning_memory:
                retrieved_experiences = memory.retrieve(
                    target_text=target_text,
                    scene_type=observed_facts.room_type_guess,
                    visible_anchor_labels=[
                        item.label_zh for item in observed_facts.visible_anchors
                    ],
                    top_k=self.settings.llm_reasoning_max_hypotheses,
                )
            request = LLMReasoningRequest(
                target_text=target_text,
                target_profile=base_scene.target_profile,
                observed_facts=observed_facts,
                retrieved_episodes=retrieved_experiences,
                capability_contract=capability,
                max_hypotheses=self.settings.llm_reasoning_max_hypotheses,
            )
            llm_reasoning = LLMSituatedSearchReasoner(
                settings=self.settings,
                auto_create_client=self.allow_remote_reasoning,
            ).reason(request)
            if self.settings.llm_reasoning_require_actionability_gate:
                gated = validate_hypotheses(llm_reasoning.hypotheses, capability)
                actionability_notes = gated.notes_zh
                llm_reasoning = llm_reasoning.model_copy(
                    update={"hypotheses": gated.hypotheses}
                )
            dynamic_prompts = collect_dynamic_detector_prompts(
                llm_reasoning.hypotheses,
                self.settings.llm_dynamic_visual_retry_max_terms,
            )
            visual_grounding_report = {
                "attempted": False,
                "prompts": dynamic_prompts,
                "upgraded_target": False,
                "reason": (
                    "target_already_observed"
                    if observed_facts.target_observed
                    else "no_dynamic_prompts"
                    if not dynamic_prompts
                    else "retry_callback_unavailable"
                ),
            }
            if (
                self.settings.enable_llm_dynamic_visual_retry
                and not observed_facts.target_observed
                and dynamic_prompts
                and self.visual_retry_callback is not None
            ):
                try:
                    retry_scene = self.visual_retry_callback(dynamic_prompts)
                    base_scene, retry_report = merge_visual_retry_scene(
                        base_scene,
                        retry_scene,
                    )
                    visual_grounding_report = {
                        **retry_report,
                        "prompts": dynamic_prompts,
                        "reason": "visual_retry_completed",
                    }
                    observed_facts = build_observed_scene_facts(
                        base_scene,
                        target_text,
                    )
                    psg = build_predictive_scene_graph(
                        base_scene,
                        retrieved_knowledge,
                        parsed_task,
                    )
                except Exception as exc:
                    visual_grounding_report = {
                        "attempted": True,
                        "prompts": dynamic_prompts,
                        "upgraded_target": False,
                        "reason": "visual_retry_failed",
                        "error": f"{type(exc).__name__}: {exc}",
                    }
            if self.settings.llm_reasoning_require_visual_gate:
                llm_reasoning = llm_reasoning.model_copy(
                    update={
                        "hypotheses": apply_visual_grounding_gate(
                            llm_reasoning.hypotheses,
                            base_scene,
                        )
                    }
                )
            motion_horizon_decision = estimate_motion_horizon(
                requested_distance_m=llm_reasoning.recommended_motion_horizon_m,
                scene_type=_motion_scene_type(
                    llm_reasoning.motion_profile_hint,
                    observed_facts.room_type_guess,
                    base_scene,
                ),
                task_phase=(
                    "confirm_target"
                    if observed_facts.target_observed
                    else "approach_candidate"
                    if has_target_candidate(base_scene)
                    else "search"
                ),
                target_candidate_visible=(
                    observed_facts.target_observed or has_target_candidate(base_scene)
                ),
                target_confirming=observed_facts.target_observed,
                llm_recommended_horizon_m=(
                    llm_reasoning.recommended_motion_horizon_m
                    if llm_reasoning.reasoning_available
                    else None
                ),
                settings=self.settings,
            )
            quadruped_plan = plan_quadruped_viewpoints(
                target_text=target_text,
                target_found=observed_facts.target_observed,
                hypotheses=llm_reasoning.hypotheses,
                contract=capability,
                include_non_executable_steps=not self.hide_unexecutable_actions,
                motion_horizon_decision=motion_horizon_decision,
            )
            reasoned_psg = build_reasoned_predictive_scene_graph(
                base_scene,
                llm_reasoning,
                quadruped_plan,
                retrieved_experiences,
            )
            legacy_hypotheses = _legacy_hypotheses(llm_reasoning)
            reasoning_summary = _llm_reasoning_summary(base_scene, llm_reasoning)
            task_plan = quadruped_plan_to_task_plan(
                quadruped_plan,
                parsed_task,
            )
            if self.enable_reasoning_memory and self.update_kb:
                experience_writes = _write_experiences(
                    memory,
                    target_text,
                    observed_facts,
                    llm_reasoning,
                )
        else:
            reasoning = reason_about_scene(
                base_scene, parsed_task, retrieved_knowledge, psg
            )
            legacy_hypotheses = reasoning.hypotheses
            reasoning_summary = reasoning.reasoning_summary_zh
            task_plan = plan_task(base_scene, parsed_task, legacy_hypotheses, psg)
        knowledge_updates = (
            update_knowledge_from_scene(base_scene, parsed_task, kb_dir=self.kb_dir)
            if self.update_kb and self.settings.static_knowledge_base_enabled
            else []
        )

        return KnowledgeAwareSceneResult(
            base_scene=base_scene,
            deprecated_name=True,
            new_concept="llm_generated_commonsense_with_observation_memory",
            static_kb_used=static_kb_used,
            handcrafted_priors_used=(
                self.settings.handwritten_object_priors_enabled
                or self.settings.handwritten_location_priors_enabled
                or self.settings.handwritten_room_priors_enabled
                or self.settings.allow_handcrafted_search_rules
            ),
            parsed_task=parsed_task,
            retrieved_knowledge=retrieved_knowledge,
            predictive_scene_graph=psg,
            hypotheses=legacy_hypotheses,
            reasoning_summary_zh=reasoning_summary,
            task_plan=task_plan,
            knowledge_updates=knowledge_updates,
            final_answer_zh=_final_answer(reasoning_summary, task_plan.summary_zh),
            observed_facts=observed_facts,
            llm_reasoning=llm_reasoning,
            reasoned_predictive_scene_graph=reasoned_psg,
            quadruped_search_plan=quadruped_plan,
            actionability_notes_zh=actionability_notes,
            retrieved_experiences=retrieved_experiences,
            experience_writes=experience_writes,
            visual_grounding_report=visual_grounding_report,
            motion_horizon_decision=motion_horizon_decision,
        )


def _infer_room_type_hint(scene: SceneAnalysisResult) -> str | None:
    names = {obj.name.lower() for obj in scene.objects}
    if names & {"desk", "table", "monitor", "keyboard", "chair"}:
        return "office"
    return None


def _motion_scene_type(
    profile_hint: str | None,
    room_type_guess: str | None,
    scene: SceneAnalysisResult,
) -> str:
    hint = (profile_hint or "").strip().lower()
    if hint == "platform_assisted_open_area":
        return "open_area"
    if hint == "platform_assisted_indoor":
        return "indoor"
    return room_type_guess or infer_scene_type_from_result(scene)


def _final_answer(reasoning_summary_zh: str, plan_summary_zh: str) -> str:
    return f"{reasoning_summary_zh} 任务计划：{plan_summary_zh}"


def _legacy_hypotheses(llm_reasoning) -> list[SceneHypothesis]:
    return [
        SceneHypothesis(
            hypothesis_id=item.hypothesis_id,
            target=item.target_name,
            possible_location=item.candidate_region_zh,
            supporting_evidence=[
                item.human_like_rationale_zh,
                *[
                    f"可见锚点：{name}"
                    for name in item.supporting_visible_anchor_names
                ],
            ],
            contradicting_evidence=[item.uncertainty_zh],
            knowledge_sources=item.memory_sources,
            probability=item.confidence,
            verification_action=item.suggested_verification_question_zh,
            status=(
                "verified"
                if item.status.value == "observed"
                else "rejected"
                if item.status.value == "rejected"
                else "proposed"
            ),
        )
        for item in llm_reasoning.hypotheses
    ]


def _llm_reasoning_summary(base_scene, llm_reasoning) -> str:
    if base_scene.target_decision.is_present and base_scene.target_decision.matched_object_ids:
        return (
            f"目标已由视觉确认：{base_scene.target_decision.match_reason_zh}"
            "机械狗只执行停止和重观测，不把可见性等同于安全可达。"
        )
    availability = (
        "大模型已生成情境化搜索假设"
        if llm_reasoning.reasoning_available
        else "LLM 情境推理不可用，当前采用视觉锚点降级策略"
    )
    return (
        f"目标当前未被视觉确认。{availability}。"
        f"{llm_reasoning.recommended_next_observation_zh}"
        "所有未视觉确认候选均保持 inferred，不代表目标已经出现。"
    )


def _write_experiences(memory, target_text, facts, reasoning) -> list[dict]:
    written: list[dict] = []
    for hypothesis in reasoning.hypotheses:
        outcome = (
            "found"
            if hypothesis.status.value == "observed"
            else "human_needed"
            if hypothesis.status.value == "unreachable"
            else "not_verified"
        )
        record = SpatialExperienceMemory(
            memory_id=f"mem_{uuid4().hex[:12]}",
            created_at=datetime.now(UTC).isoformat(),
            target_text=target_text,
            target_normalized=target_text.strip().lower(),
            scene_type=facts.room_type_guess,
            visible_anchor_labels=[
                item.label_zh for item in facts.visible_anchors
            ],
            hypothesis_region_zh=hypothesis.candidate_region_zh,
            hypothesis_rationale_zh=hypothesis.human_like_rationale_zh,
            action_taken=[
                item.value for item in hypothesis.quadruped_view_strategy
            ],
            outcome=outcome,
            visual_evidence_summary_zh=(
                "；".join(facts.target_evidence)
                if facts.target_evidence
                else "当前无目标视觉证据。"
            ),
            negative_evidence_zh=facts.negative_evidence,
            confidence_after_outcome=hypothesis.confidence,
        )
        if memory.append_if_novel(record):
            written.append(record.model_dump(mode="json"))
    return written
