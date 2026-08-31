from __future__ import annotations

import unittest

from app.planning.quadruped_viewpoint_planner import (
    MOTION_PRIMITIVES,
    plan_quadruped_viewpoints,
    quadruped_plan_to_task_plan,
)
from app.schemas import (
    Actionability,
    LLMSearchHypothesis,
    QuadrupedActionPrimitive,
    default_quadruped_capability,
    RobotTask,
)


class QuadrupedViewpointPlannerTest(unittest.TestCase):
    def test_converts_to_safe_legacy_task_plan(self) -> None:
        plan = plan_quadruped_viewpoints(
            target_text="手机",
            target_found=False,
            hypotheses=[],
            contract=default_quadruped_capability(),
        )
        task_plan = quadruped_plan_to_task_plan(
            plan,
            RobotTask(
                task_id="task_001",
                raw_text="找手机",
                task_type="find_object",
                target_object="phone",
                confidence=0.9,
            ),
        )
        self.assertEqual(task_plan.plan_type, "find_object")
        self.assertIn("视角", task_plan.summary_zh)
        self.assertNotIn("打开", task_plan.model_dump_json())

    def test_can_hide_non_executable_steps_but_keeps_note(self) -> None:
        contract = default_quadruped_capability()
        hypothesis = LLMSearchHypothesis(
            hypothesis_id="hyp_human",
            target_name="清洁用品",
            candidate_region_zh="封闭柜体内部",
            candidate_region_type="closed_container",
            supporting_visible_anchor_ids=["cabinet_1"],
            supporting_visible_anchor_names=["柜子"],
            human_like_rationale_zh="需要打开柜门。",
            expected_visual_cues_zh=[],
            suggested_detector_prompts_en=[],
            suggested_verification_question_zh="请求人工确认。",
            confidence=0.7,
            uncertainty_zh="机械狗无法验证内部。",
            actionability=Actionability.NEEDS_HUMAN,
            quadruped_view_strategy=[
                QuadrupedActionPrimitive.ASK_HUMAN_FOR_OCCLUDED_AREA,
                QuadrupedActionPrimitive.MARK_UNREACHABLE,
            ],
        )
        plan = plan_quadruped_viewpoints(
            target_text="清洁用品",
            target_found=False,
            hypotheses=[hypothesis],
            contract=contract,
            include_non_executable_steps=False,
        )
        self.assertEqual(
            [item.primitive for item in plan.steps],
            [QuadrupedActionPrimitive.STOP_AND_REOBSERVE],
        )
        self.assertTrue(plan.non_executable_notes_zh)

    def test_plan_uses_whitelist_and_stops_after_motion(self) -> None:
        contract = default_quadruped_capability()
        hypothesis = LLMSearchHypothesis(
            hypothesis_id="hyp_001",
            target_name="手机",
            candidate_region_zh="左侧门口",
            candidate_region_type="doorway",
            image_region_hint="left",
            supporting_visible_anchor_ids=["door_1"],
            supporting_visible_anchor_names=["门"],
            human_like_rationale_zh="门口方向值得换视角",
            expected_visual_cues_zh=["手机"],
            suggested_detector_prompts_en=["phone"],
            suggested_verification_question_zh="向左转后重观测",
            confidence=0.8,
            uncertainty_zh="未确认",
            actionability=Actionability.ROBOT_EXECUTABLE,
            quadruped_view_strategy=[
                QuadrupedActionPrimitive.TURN_LEFT,
                QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
            ],
        )
        plan = plan_quadruped_viewpoints(
            target_text="手机",
            target_found=False,
            hypotheses=[hypothesis],
            contract=contract,
        )
        primitives = [step.primitive for step in plan.steps]
        self.assertTrue(all(item in contract.allowed_primitives for item in primitives))
        for index, primitive in enumerate(primitives[:-1]):
            if primitive in MOTION_PRIMITIVES:
                self.assertEqual(
                    primitives[index + 1],
                    QuadrupedActionPrimitive.STOP_AND_REOBSERVE,
                )


if __name__ == "__main__":
    unittest.main()
