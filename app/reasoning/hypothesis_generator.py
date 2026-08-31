"""Generate structured scene hypotheses from PSG and retrieved knowledge."""

from __future__ import annotations

from app.reasoning.evidence_scorer import score_hypothesis_evidence
from app.reasoning.verification_planner import plan_verification_action
from app.schemas import (
    KnowledgeItem,
    PredictiveSceneGraph,
    RobotTask,
    SceneAnalysisResult,
    SceneHypothesis,
)


def generate_scene_hypotheses(
    scene: SceneAnalysisResult,
    task: RobotTask,
    knowledge: list[KnowledgeItem],
    psg: PredictiveSceneGraph,
) -> list[SceneHypothesis]:
    if scene.target_decision.is_present and scene.target_decision.matched_object_ids:
        return [_visible_target_hypothesis(scene, task)]

    candidate_locations = _candidate_locations(task, knowledge, psg)
    hypotheses: list[SceneHypothesis] = []
    for index, location in enumerate(candidate_locations, start=1):
        score = score_hypothesis_evidence(scene, knowledge, psg, location)
        hypotheses.append(
            SceneHypothesis(
                hypothesis_id=f"hyp_{index:03d}",
                target=task.target_object or task.target_room or task.raw_text,
                possible_location=location,
                supporting_evidence=_supporting_evidence(scene, knowledge, psg, location),
                contradicting_evidence=_contradicting_evidence(scene),
                knowledge_sources=[item.id for item in knowledge],
                probability=score.probability,
                verification_action=plan_verification_action(location, task),
                status="proposed",
            )
        )

    return sorted(hypotheses, key=lambda item: item.probability, reverse=True)


def _visible_target_hypothesis(
    scene: SceneAnalysisResult,
    task: RobotTask,
) -> SceneHypothesis:
    matched = "、".join(scene.target_decision.matched_object_ids)
    return SceneHypothesis(
        hypothesis_id="hyp_visible_001",
        target=task.target_object or task.raw_text,
        possible_location=f"当前可见目标：{matched}",
        supporting_evidence=[scene.target_decision.match_reason_zh],
        contradicting_evidence=[],
        knowledge_sources=[],
        probability=scene.target_decision.confidence,
        verification_action="靠近目标并进行二次观察确认。",
        status="verified",
    )


def _candidate_locations(
    task: RobotTask,
    knowledge: list[KnowledgeItem],
    psg: PredictiveSceneGraph,
) -> list[str]:
    candidates: list[str] = []
    candidates.extend(psg.recommended_verification_targets)

    for item in knowledge:
        if item.knowledge_type == "object_location_prior":
            if item.metadata.get("object_name") == "phone":
                candidates.extend(["桌面区域", "键盘旁", "显示器底座附近", "充电器附近"])
        elif item.knowledge_type == "environment_layout":
            if task.task_type in {"inspect_area", "check_door_state"}:
                candidates.append("楼层可见门和相邻门")
            elif task.task_type in {"find_room", "navigate_to_location"}:
                candidates.append(task.target_room or task.target_location or "相邻门牌")

    if task.target_location:
        candidates.insert(0, task.target_location)

    return _dedupe(candidates)[:4]


def _supporting_evidence(
    scene: SceneAnalysisResult,
    knowledge: list[KnowledgeItem],
    psg: PredictiveSceneGraph,
    location: str,
) -> list[str]:
    evidence = []
    visible = "、".join(obj.name_zh for obj in scene.objects[:5])
    if visible:
        evidence.append(f"当前可见物体包括：{visible}。")
    if knowledge:
        evidence.append("检索到与任务相关的场景知识或位置先验。")
    if location in psg.recommended_verification_targets:
        evidence.append("预测性场景图将该位置列为推荐验证目标。")
    return evidence or ["当前信息不足，但该位置仍是可验证候选。"]


def _contradicting_evidence(scene: SceneAnalysisResult) -> list[str]:
    if scene.target_decision.is_present:
        return []
    return ["当前画面没有直接检测到目标。"]


def _dedupe(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        if not value or value in seen:
            continue
        seen.add(value)
        result.append(value)
    return result
