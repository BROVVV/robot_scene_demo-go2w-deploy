"""Rule-based evidence scoring for scene hypotheses."""

from __future__ import annotations

from dataclasses import dataclass

from app.schemas import KnowledgeItem, PredictiveSceneGraph, SceneAnalysisResult


@dataclass(frozen=True)
class EvidenceScore:
    scene_type_match: float
    object_context_match: float
    spatial_plausibility: float
    visibility_gap: float
    knowledge_confidence: float
    navigation_cost: float
    risk_penalty: float

    @property
    def probability(self) -> float:
        score = (
            self.scene_type_match * 0.18
            + self.object_context_match * 0.24
            + self.spatial_plausibility * 0.18
            + self.visibility_gap * 0.14
            + self.knowledge_confidence * 0.18
            + (1.0 - self.navigation_cost) * 0.06
            - self.risk_penalty * 0.08
        )
        return max(0.0, min(1.0, round(score, 3)))


def score_hypothesis_evidence(
    scene: SceneAnalysisResult,
    knowledge: list[KnowledgeItem],
    psg: PredictiveSceneGraph,
    candidate_location: str,
) -> EvidenceScore:
    return EvidenceScore(
        scene_type_match=_scene_type_match(scene, knowledge),
        object_context_match=_object_context_match(scene, candidate_location),
        spatial_plausibility=_spatial_plausibility(psg, candidate_location),
        visibility_gap=0.72 if not scene.target_decision.is_present else 0.2,
        knowledge_confidence=_knowledge_confidence(knowledge),
        navigation_cost=_navigation_cost(candidate_location),
        risk_penalty=_risk_penalty(scene),
    )


def _scene_type_match(
    scene: SceneAnalysisResult,
    knowledge: list[KnowledgeItem],
) -> float:
    if any(item.knowledge_type == "room_type_prior" for item in knowledge):
        visible_names = {obj.name.lower() for obj in scene.objects}
        office_context = {"desk", "table", "monitor", "keyboard", "chair"}
        if visible_names & office_context:
            return 0.82
        return 0.62
    return 0.45


def _object_context_match(scene: SceneAnalysisResult, candidate_location: str) -> float:
    visible_text = " ".join(
        [obj.name.lower() for obj in scene.objects]
        + [obj.name_zh.lower() for obj in scene.objects]
        + [attribute.lower() for obj in scene.objects for attribute in obj.attributes]
    )
    location = candidate_location.lower()
    score = 0.35
    for keyword in ["desk", "table", "keyboard", "charger", "monitor", "桌", "键盘", "充电", "显示器"]:
        if keyword in visible_text and (keyword in location or candidate_location):
            score += 0.12
    return min(score, 0.9)


def _spatial_plausibility(psg: PredictiveSceneGraph, candidate_location: str) -> float:
    if candidate_location in psg.recommended_verification_targets:
        return 0.78
    if psg.inferred_node_ids:
        return 0.62
    return 0.42


def _knowledge_confidence(knowledge: list[KnowledgeItem]) -> float:
    if not knowledge:
        return 0.35
    return round(sum(item.confidence for item in knowledge) / len(knowledge), 3)


def _navigation_cost(candidate_location: str) -> float:
    if any(keyword in candidate_location for keyword in ["桌", "键盘", "显示器", "门"]):
        return 0.25
    if any(keyword in candidate_location for keyword in ["走廊尽头", "楼层", "相邻"]):
        return 0.55
    return 0.4


def _risk_penalty(scene: SceneAnalysisResult) -> float:
    risk_keywords = ["stairs", "step", "obstacle", "台阶", "楼梯", "障碍"]
    text = " ".join(
        [scene.scene_summary_zh]
        + [obj.name.lower() for obj in scene.objects]
        + [obj.name_zh for obj in scene.objects]
        + [attribute for obj in scene.objects for attribute in obj.attributes]
    )
    return 0.35 if any(keyword in text for keyword in risk_keywords) else 0.08
