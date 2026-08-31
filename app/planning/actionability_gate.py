"""Validate and rewrite LLM hypotheses against quadruped capabilities."""

from __future__ import annotations

from dataclasses import dataclass

from app.schemas import (
    Actionability,
    LLMSearchHypothesis,
    NodeObservationStatus,
    QuadrupedActionPrimitive,
    RobotCapabilityContract,
)


HUMAN_REQUIRED_TERMS = {
    "打开",
    "翻找",
    "拿起",
    "拉开",
    "抽屉内部",
    "包内",
    "容器内",
    "口袋",
    "衣袋",
    "背包内部",
    "柜体内部",
    "封闭区域",
    "机械臂",
    "伸手",
    "移动物体",
}
VIEWPOINT_REWRITE_TERMS = {
    "低头",
    "俯视",
    "检查周边30-80厘米",
    "检查周边 30-80 厘米",
    "靠近桌面",
}


@dataclass(frozen=True)
class ActionabilityGateResult:
    hypotheses: list[LLMSearchHypothesis]
    notes_zh: list[str]


def validate_hypotheses(
    hypotheses: list[LLMSearchHypothesis],
    contract: RobotCapabilityContract,
) -> ActionabilityGateResult:
    validated: list[LLMSearchHypothesis] = []
    notes: list[str] = []
    allowed = set(contract.allowed_primitives)
    for hypothesis in hypotheses:
        combined = " ".join(
            [
                hypothesis.candidate_region_zh,
                hypothesis.human_like_rationale_zh,
                hypothesis.suggested_verification_question_zh,
                *hypothesis.safety_notes_zh,
            ]
        )
        forbidden_terms = set(contract.forbidden_action_phrases_zh)
        human_vocabulary = HUMAN_REQUIRED_TERMS | {
            term
            for term in forbidden_terms
            if not any(marker in term for marker in ["低头", "俯视", "30-80"])
        }
        viewpoint_vocabulary = VIEWPOINT_REWRITE_TERMS | {
            term
            for term in forbidden_terms
            if any(marker in term for marker in ["低头", "俯视", "30-80"])
        }
        human_terms = sorted(term for term in human_vocabulary if term in combined)
        rewrite_terms = sorted(term for term in VIEWPOINT_REWRITE_TERMS if term in combined)
        rewrite_terms = sorted(
            set(rewrite_terms)
            | {term for term in viewpoint_vocabulary if term in combined}
        )
        strategy = [item for item in hypothesis.quadruped_view_strategy if item in allowed]
        actionability = hypothesis.actionability
        status = hypothesis.status
        rationale = hypothesis.human_like_rationale_zh
        question = hypothesis.suggested_verification_question_zh
        safety = list(hypothesis.safety_notes_zh)

        if (
            not hypothesis.supporting_visible_anchor_ids
            and QuadrupedActionPrimitive.MOVE_FORWARD_SHORT in strategy
        ):
            strategy = []
            if QuadrupedActionPrimitive.SCAN_LEFT_TO_RIGHT in allowed:
                strategy.append(QuadrupedActionPrimitive.SCAN_LEFT_TO_RIGHT)
            if QuadrupedActionPrimitive.STOP_AND_REOBSERVE in allowed:
                strategy.append(QuadrupedActionPrimitive.STOP_AND_REOBSERVE)
            safety.append("缺少可见锚点，禁止向未知区域前进。")
            notes.append(
                f"{hypothesis.hypothesis_id}：缺少可见锚点，已移除 MOVE_FORWARD_SHORT。"
            )

        if human_terms and not contract.can_manipulate:
            actionability = Actionability.NEEDS_HUMAN
            status = NodeObservationStatus.UNREACHABLE
            strategy = [
                item
                for item in [
                    QuadrupedActionPrimitive.ASK_HUMAN_FOR_OCCLUDED_AREA,
                    QuadrupedActionPrimitive.MARK_UNREACHABLE,
                ]
                if item in allowed
            ]
            rationale = "该候选涉及封闭、遮挡或物体操作，机械狗只能记录并请求人工确认。"
            question = "请人工确认该遮挡或封闭区域；机械狗不进入内部验证。"
            safety.append("已拦截超出机械狗能力的操作建议。")
            notes.append(
                f"{hypothesis.hypothesis_id}：检测到需人工动作语义（{'、'.join(human_terms)}），"
                "已标记为 NEEDS_HUMAN。"
            )
        elif rewrite_terms and not contract.can_crouch_or_look_down:
            actionability = Actionability.ROBOT_VIEWPOINT_ONLY
            strategy = [
                item
                for item in [
                    QuadrupedActionPrimitive.CENTER_VIEW_ON_REGION,
                    QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                ]
                if item in allowed
            ]
            rationale = "机械狗不能低头或精细检查表面，已改为将候选方向置于视野中心后重观测。"
            question = "调整朝向并停止后，当前相机视角能否视觉确认目标？"
            safety.append("不执行低头、俯视或局部精查。")
            notes.append(
                f"{hypothesis.hypothesis_id}：不可执行的视角语义已改写为中心化重观测。"
            )

        if not strategy:
            strategy = [QuadrupedActionPrimitive.STOP_AND_REOBSERVE]
            actionability = Actionability.NEEDS_REOBSERVATION
            notes.append(
                f"{hypothesis.hypothesis_id}：无合法动作，已回退为 STOP_AND_REOBSERVE。"
            )

        if QuadrupedActionPrimitive.MOVE_FORWARD_SHORT in strategy:
            notes.append(
                f"{hypothesis.hypothesis_id}：move_forward 可执行；"
                f"高层运动视界上限由平台避障辅助策略裁剪，当前能力上限 "
                f"{contract.max_executable_distance_m:.2f} m。"
            )

        validated.append(
            hypothesis.model_copy(
                update={
                    "status": status,
                    "actionability": actionability,
                    "quadruped_view_strategy": strategy,
                    "human_like_rationale_zh": rationale,
                    "suggested_verification_question_zh": question,
                    "safety_notes_zh": safety,
                    "should_not_mark_found": (
                        status != NodeObservationStatus.OBSERVED
                    ),
                    "max_executable_distance_m": contract.max_executable_distance_m,
                    "execution_assumption": contract.execution_assumption,
                }
            )
        )
    return ActionabilityGateResult(hypotheses=validated, notes_zh=notes)
