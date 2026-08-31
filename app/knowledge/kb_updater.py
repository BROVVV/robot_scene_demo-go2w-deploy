"""Extract candidate facts and update the local scene knowledge base."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from app.knowledge import kb_store
from app.knowledge.kb_schema import ObservationLogRecord, RoomPrior, SceneKBData
from app.schemas import CandidateFact, KnowledgeUpdate, RobotTask, SceneAnalysisResult


STABLE_FACT_TYPES = {"environment_layout", "room_type_prior", "object_location_prior"}


def extract_candidate_facts(
    scene_result: SceneAnalysisResult,
    task: RobotTask,
) -> list[CandidateFact]:
    facts: list[CandidateFact] = []
    facts.extend(_room_type_facts(scene_result))
    facts.extend(_door_layout_facts(scene_result))
    facts.extend(_temporary_task_facts(scene_result, task))
    return facts


def merge_candidate_facts(
    kb: SceneKBData,
    facts: list[CandidateFact],
) -> tuple[SceneKBData, list[KnowledgeUpdate]]:
    updates: list[KnowledgeUpdate] = []
    for fact in facts:
        if fact.stable and fact.confidence >= 0.75 and fact.fact_type in STABLE_FACT_TYPES:
            updates.append(_merge_stable_fact(kb, fact))
        else:
            updates.append(
                KnowledgeUpdate(
                    update_id=f"update:{fact.fact_id}",
                    update_type="ignored",
                    knowledge_type=fact.fact_type,  # type: ignore[arg-type]
                    content_zh=f"{fact.content_zh}（未写入长期知识库）",
                    source_observation_id=None,
                    stable=fact.stable,
                    confidence=fact.confidence,
                )
            )
    return kb, updates


def update_knowledge_from_scene(
    scene_result: SceneAnalysisResult,
    task: RobotTask,
    kb_dir: str | Path = kb_store.DEFAULT_KB_DIR,
) -> list[KnowledgeUpdate]:
    kb = kb_store.load_kb(kb_dir)
    observation = _observation_from_scene(scene_result, task)
    kb_store.append_observation(observation, kb_dir=kb_dir)
    kb.observations.append(observation)
    facts = extract_candidate_facts(scene_result, task)
    updated_kb, updates = merge_candidate_facts(kb, facts)
    kb_store.save_kb(updated_kb, kb_dir=kb_dir)
    return [
        update.model_copy(update={"source_observation_id": observation.observation_id})
        for update in updates
    ]


def _room_type_facts(scene: SceneAnalysisResult) -> list[CandidateFact]:
    object_names = {obj.name.lower() for obj in scene.objects}
    office_context = {"desk", "table", "monitor", "keyboard", "chair"}
    if len(object_names & office_context) < 2:
        return []

    source_ids = [obj.id for obj in scene.objects if obj.name.lower() in office_context]
    confidence = min(0.92, 0.58 + len(source_ids) * 0.08)
    return [
        CandidateFact(
            fact_id="fact_room_type_office",
            fact_type="room_type_prior",
            content_zh="当前观察支持该区域具有办公室布局特征。",
            source_object_ids=source_ids,
            stable=True,
            confidence=round(confidence, 2),
            metadata={"room_type": "office"},
        )
    ]


def _door_layout_facts(scene: SceneAnalysisResult) -> list[CandidateFact]:
    facts: list[CandidateFact] = []
    for obj in scene.objects:
        text = " ".join([obj.name, obj.name_zh, *obj.attributes]).lower()
        if "door" not in text and "门" not in text:
            continue
        room_label = _extract_room_label(text)
        facts.append(
            CandidateFact(
                fact_id=f"fact_door_layout_{obj.id}",
                fact_type="environment_layout",
                content_zh=(
                    f"当前观察到门或门牌{room_label or ''}，可作为楼层布局候选事实。"
                ),
                source_object_ids=[obj.id],
                stable=True,
                confidence=obj.confidence,
                metadata={"room_label": room_label, "object_id": obj.id},
            )
        )
    return facts


def _temporary_task_facts(
    scene: SceneAnalysisResult,
    task: RobotTask,
) -> list[CandidateFact]:
    facts: list[CandidateFact] = []
    if task.task_type == "check_door_state" or task.parsed_slots.get("subtask") == "check_door_state":
        facts.append(
            CandidateFact(
                fact_id="fact_temporary_door_state",
                fact_type="temporary_state",
                content_zh="门当前是否打开属于临时状态，只记录为本次观察。",
                source_object_ids=scene.target_decision.matched_object_ids,
                stable=False,
                confidence=scene.target_decision.confidence,
                metadata={"state": task.parsed_slots.get("state")},
            )
        )

    if task.target_object == "phone":
        facts.append(
            CandidateFact(
                fact_id="fact_temporary_phone_location",
                fact_type="task_memory",
                content_zh="手机是否出现在当前画面只作为本次任务记忆。",
                source_object_ids=scene.target_decision.matched_object_ids,
                stable=False,
                confidence=scene.target_decision.confidence,
                metadata={"is_present": scene.target_decision.is_present},
            )
        )
    return facts


def _merge_stable_fact(kb: SceneKBData, fact: CandidateFact) -> KnowledgeUpdate:
    if fact.fact_type == "room_type_prior":
        room_type = str(fact.metadata.get("room_type") or "")
        for prior in kb.room_type_priors:
            if prior.room_type == room_type:
                prior.confidence = max(prior.confidence, fact.confidence)
                return _knowledge_update(fact, "confirmed")
        kb.room_type_priors.append(
            RoomPrior(
                room_type=room_type or "unknown",
                name_zh=None,
                common_objects=[],
                likely_layout=[],
                confidence=fact.confidence,
            )
        )
        return _knowledge_update(fact, "new")

    if fact.fact_type == "environment_layout":
        return _knowledge_update(fact, "confirmed")

    if fact.fact_type == "object_location_prior":
        return _knowledge_update(fact, "confirmed")

    return _knowledge_update(fact, "ignored")


def _knowledge_update(fact: CandidateFact, update_type: str) -> KnowledgeUpdate:
    return KnowledgeUpdate(
        update_id=f"update:{fact.fact_id}",
        update_type=update_type,  # type: ignore[arg-type]
        knowledge_type=fact.fact_type,  # type: ignore[arg-type]
        content_zh=fact.content_zh,
        source_observation_id=None,
        stable=fact.stable,
        confidence=fact.confidence,
    )


def _observation_from_scene(
    scene: SceneAnalysisResult,
    task: RobotTask,
) -> ObservationLogRecord:
    timestamp = datetime.now(timezone.utc).isoformat()
    return ObservationLogRecord(
        observation_id=f"obs_{timestamp.replace(':', '').replace('.', '')}",
        timestamp=timestamp,
        location_hint=task.scope,
        summary_zh=scene.scene_summary_zh,
        confidence=scene.target_decision.confidence,
    )


def _extract_room_label(text: str) -> str | None:
    import re

    match = re.search(r"\b\d{3,5}\b", text)
    return match.group(0) if match else None
