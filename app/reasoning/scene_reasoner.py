"""Explain scene reasoning with structured hypotheses."""

from __future__ import annotations

from dataclasses import dataclass

from app.reasoning.hypothesis_generator import generate_scene_hypotheses
from app.schemas import (
    KnowledgeItem,
    PredictiveSceneGraph,
    RobotTask,
    SceneAnalysisResult,
    SceneHypothesis,
)


@dataclass(frozen=True)
class SceneReasoningOutput:
    hypotheses: list[SceneHypothesis]
    reasoning_summary_zh: str
    recommended_action_zh: str


def reason_about_scene(
    scene: SceneAnalysisResult,
    task: RobotTask,
    knowledge: list[KnowledgeItem],
    psg: PredictiveSceneGraph,
) -> SceneReasoningOutput:
    hypotheses = generate_scene_hypotheses(scene, task, knowledge, psg)
    summary = _build_summary(scene, task, knowledge, hypotheses)
    recommended_action = (
        hypotheses[0].verification_action if hypotheses else "当前信息不足，建议重新观察当前场景。"
    )
    return SceneReasoningOutput(
        hypotheses=hypotheses,
        reasoning_summary_zh=summary,
        recommended_action_zh=recommended_action,
    )


def _build_summary(
    scene: SceneAnalysisResult,
    task: RobotTask,
    knowledge: list[KnowledgeItem],
    hypotheses: list[SceneHypothesis],
) -> str:
    if scene.target_decision.is_present and scene.target_decision.matched_object_ids:
        return (
            f"当前画面已经匹配到任务目标，匹配依据是：{scene.target_decision.match_reason_zh}"
            "建议靠近目标并进行二次确认。"
        )

    target = task.target_object or task.target_room or task.raw_text
    visible_objects = "、".join(obj.name_zh for obj in scene.objects[:5]) or "暂无明确物体"
    knowledge_note = "已检索到相关知识库先验" if knowledge else "未检索到强相关知识库先验"
    if hypotheses:
        best = hypotheses[0]
        return (
            f"当前画面没有直接确认{target}，可见物体包括：{visible_objects}。"
            f"{knowledge_note}，因此优先假设目标或待检查状态位于{best.possible_location}。"
            f"建议执行：{best.verification_action}"
        )
    return (
        f"当前画面没有直接确认{target}，可见物体包括：{visible_objects}。"
        "现有证据不足以生成可靠候选位置，建议先扩大视角重新观察。"
    )
