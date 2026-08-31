"""Task-specific planning strategies."""

from __future__ import annotations

from app.planning.exploration_planner import verification_targets
from app.planning.route_policy import (
    approach_description,
    fallback_observation_description,
)
from app.schemas import (
    CountState,
    PredictiveSceneGraph,
    RobotTask,
    SceneAnalysisResult,
    SceneHypothesis,
    TaskPlan,
    TaskPlanStep,
)


def plan_find_object(
    scene: SceneAnalysisResult,
    task: RobotTask,
    hypotheses: list[SceneHypothesis],
    psg: PredictiveSceneGraph | None = None,
) -> TaskPlan:
    if scene.target_decision.is_present and scene.target_decision.matched_object_ids:
        target = "、".join(scene.target_decision.matched_object_ids)
        return TaskPlan(
            plan_type="find_object",
            summary_zh="目标当前可见，优先靠近并二次确认。",
            steps=[
                TaskPlanStep(
                    step_id=1,
                    action_type="move",
                    target=target,
                    description_zh=approach_description("可见目标"),
                    expected_result="获得更清晰目标视角。",
                    confidence=scene.target_decision.confidence,
                ),
                TaskPlanStep(
                    step_id=2,
                    action_type="verify",
                    target=target,
                    description_zh="重新观察目标区域，确认目标类别和位置。",
                    expected_result="确认目标可达且与任务描述一致。",
                    depends_on=[1],
                    confidence=0.82,
                ),
            ],
            success_conditions=["目标被重新观察并确认。"],
        )

    targets = verification_targets(hypotheses, psg)
    steps = [
        TaskPlanStep(
            step_id=index,
            action_type="inspect" if index > 1 else "move",
            target=target,
            description_zh=approach_description(target),
            expected_result="确认候选位置是否存在目标。",
            confidence=max(0.45, 0.82 - index * 0.08),
        )
        for index, target in enumerate(targets[:3], start=1)
    ]
    return TaskPlan(
        plan_type="find_object",
        summary_zh="目标当前不可见，按候选位置的概率和验证成本逐一检查。",
        steps=steps,
        fallback_steps=[_fallback_step(len(steps) + 1)],
        success_conditions=["检测到目标并确认位置。", "高概率区域检查完毕后输出未发现结论。"],
        uncertainty_notes=["单张图片无法覆盖遮挡区域，需要通过新视角验证。"],
    )


def plan_count_objects(scene: SceneAnalysisResult, task: RobotTask) -> TaskPlan:
    target = task.target_object
    counted = [
        obj.id
        for obj in scene.objects
        if target is None or obj.name == target or obj.category == target
    ]
    uncertain_regions = _uncertain_regions(scene)
    count_state = CountState(
        counted_object_ids=counted,
        possible_duplicates=[],
        uncertain_regions=uncertain_regions,
        recommended_next_viewpoints=[
            f"重新观察{region}" for region in uncertain_regions
        ],
    )
    steps = [
        TaskPlanStep(
            step_id=1,
            action_type="count",
            target=target,
            description_zh="统计当前视野内已确认的目标物体，并记录物体 ID。",
            expected_result="得到当前视角的初始计数。",
            confidence=0.78,
        )
    ]
    if uncertain_regions:
        steps.append(
            TaskPlanStep(
                step_id=2,
                action_type="observe",
                target=uncertain_regions[0],
                description_zh=f"移动或转向以覆盖{uncertain_regions[0]}，避免漏数。",
                expected_result="补充盲区计数并与已计数 ID 去重。",
                depends_on=[1],
                confidence=0.66,
            )
        )
    return TaskPlan(
        plan_type="count_objects",
        summary_zh=f"当前视野确认 {len(counted)} 个候选目标，需检查盲区后合并计数。",
        steps=steps,
        success_conditions=["所有可见区域完成计数。", "多视角结果完成去重。"],
        uncertainty_notes=["遮挡和视野边缘可能导致漏检或重复计数。"],
        count_state=count_state,
    )


def plan_inspect_area(
    task: RobotTask,
    psg: PredictiveSceneGraph | None = None,
) -> TaskPlan:
    targets = psg.recommended_verification_targets if psg is not None else []
    if not targets:
        targets = ["当前可见区域", "相邻区域", "视野盲区"]
    steps = [
        TaskPlanStep(
            step_id=index,
            action_type="inspect",
            target=target,
            description_zh=f"按拓扑顺序检查{target}并记录状态。",
            expected_result="得到该检查点的状态。",
            confidence=max(0.5, 0.82 - index * 0.06),
        )
        for index, target in enumerate(targets[:4], start=1)
    ]
    return TaskPlan(
        plan_type="inspect_area",
        summary_zh="将区域拆成多个检查点，按可见拓扑顺序逐一巡查。",
        steps=steps,
        success_conditions=["所有检查点均有 open/closed/unknown 等结构化状态。"],
        uncertainty_notes=["状态不确定的检查点需要靠近或换角度观察。"],
    )


def plan_check_door_state(task: RobotTask) -> TaskPlan:
    target = task.target_room or task.target_location or task.target_object or "目标门"
    return TaskPlan(
        plan_type="check_door_state",
        summary_zh="优先靠近目标门，从门缝、门把手和门板角度判断开关状态。",
        steps=[
            TaskPlanStep(
                step_id=1,
                action_type="move",
                target=target,
                description_zh=approach_description(str(target)),
                expected_result="获得门的正面或侧向清晰视角。",
                confidence=0.76,
            ),
            TaskPlanStep(
                step_id=2,
                action_type="verify",
                target=target,
                description_zh="观察门缝、门把手和门板角度，输出 open/closed/unknown。",
                expected_result="得到门状态判断。",
                depends_on=[1],
                confidence=0.78,
            ),
        ],
        success_conditions=["门状态被判断为 open、closed 或 unknown。"],
        uncertainty_notes=["单张图片中门缝不可见时不能强行判断开关状态。"],
    )


def plan_navigate_to_location(task: RobotTask) -> TaskPlan:
    destination = task.target_room or task.target_location or task.raw_text
    return TaskPlan(
        plan_type="navigate_to_location",
        summary_zh="优先使用楼层知识和当前可见标识规划到目标地点的探索路线。",
        steps=[
            TaskPlanStep(
                step_id=1,
                action_type="observe",
                target="当前门牌和走廊方向",
                description_zh="确认当前门牌、走廊方向和可通行区域。",
                expected_result="建立当前位置到目标地点的相对线索。",
                confidence=0.72,
            ),
            TaskPlanStep(
                step_id=2,
                action_type="move",
                target=destination,
                description_zh=approach_description(str(destination)),
                expected_result="靠近目标地点或下一个可验证标识。",
                depends_on=[1],
                confidence=0.68,
            ),
        ],
        success_conditions=["确认到达目标地点或最近可验证标识。"],
        uncertainty_notes=["缺少楼层布局时需要边走边验证门牌和通行关系。"],
    )


def plan_general(task: RobotTask) -> TaskPlan:
    return TaskPlan(
        plan_type="general",
        summary_zh="先重新观察当前场景，再根据任务条件输出结构化结论。",
        steps=[
            TaskPlanStep(
                step_id=1,
                action_type="observe",
                target=task.raw_text,
                description_zh="重新观察与任务相关的区域或物体。",
                expected_result="获得足够证据回答任务。",
                confidence=0.62,
            )
        ],
        success_conditions=["收集到足够证据并输出结论。"],
    )


def _fallback_step(step_id: int) -> TaskPlanStep:
    return TaskPlanStep(
        step_id=step_id,
        action_type="observe",
        target="全局视角",
        description_zh=fallback_observation_description(),
        expected_result="获得新的候选位置或确认当前区域未发现目标。",
        confidence=0.52,
    )


def _uncertain_regions(scene: SceneAnalysisResult) -> list[str]:
    regions: list[str] = []
    if any(obj.bbox_2d.x1 < 0.08 for obj in scene.objects):
        regions.append("左侧视野边缘")
    if any(obj.bbox_2d.x2 > 0.92 for obj in scene.objects):
        regions.append("右侧视野边缘")
    if any("occluded" in attribute.lower() or "遮挡" in attribute for obj in scene.objects for attribute in obj.attributes):
        regions.append("遮挡区域")
    return regions or ["房间角落和遮挡区域"]
