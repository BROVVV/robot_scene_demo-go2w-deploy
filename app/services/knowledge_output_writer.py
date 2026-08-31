"""Persist knowledge-aware scene analysis outputs."""

from __future__ import annotations

import json
from pathlib import Path

from app.schemas import KnowledgeAwareSceneResult
from app.services.image_annotator import export_reasoned_annotated_image
from app.services.predictive_scene_graph import (
    export_reasoned_predictive_scene_graph,
)
from app.services.ros2_command_exporter import (
    export_quadruped_ros2_motion_plan,
)
from app.services.psg_builder import export_predictive_scene_graph_graphml


def write_knowledge_aware_outputs(
    result: KnowledgeAwareSceneResult,
    output_dir: str | Path,
    image_path: str | Path | None = None,
) -> dict[str, Path]:
    path = Path(output_dir)
    path.mkdir(parents=True, exist_ok=True)

    outputs: dict[str, Path] = {}
    outputs["knowledge_aware_result"] = _write_json(
        path / "knowledge_aware_result.json",
        result.model_dump(mode="json"),
    )
    outputs["parsed_task"] = _write_json(
        path / "parsed_task.json",
        result.parsed_task.model_dump(mode="json"),
    )
    outputs["retrieved_knowledge"] = _write_json(
        path / "retrieved_knowledge.json",
        [item.model_dump(mode="json") for item in result.retrieved_knowledge],
    )
    outputs["predictive_scene_graph_graphml"] = export_predictive_scene_graph_graphml(
        result.predictive_scene_graph,
        path / "predictive_scene_graph.graphml",
    )
    outputs["hypotheses"] = _write_json(
        path / "hypotheses.json",
        [item.model_dump(mode="json") for item in result.hypotheses],
    )
    outputs["knowledge_updates"] = _write_json(
        path / "knowledge_updates.json",
        [item.model_dump(mode="json") for item in result.knowledge_updates],
    )
    outputs["reasoning_report"] = _write_report(
        path / "reasoning_report.md",
        result,
    )
    if result.llm_reasoning is not None:
        outputs["llm_search_hypotheses"] = _write_json(
            path / "llm_search_hypotheses.json",
            result.llm_reasoning.model_dump(mode="json"),
        )
    if result.quadruped_search_plan is not None:
        outputs["quadruped_search_plan"] = _write_json(
            path / "quadruped_search_plan.json",
            result.quadruped_search_plan.model_dump(mode="json"),
        )
        if result.motion_horizon_decision is not None:
            outputs["motion_horizon_decision"] = _write_json(
                path / "motion_horizon_decision.json",
                result.motion_horizon_decision.model_dump(mode="json"),
            )
        outputs["quadruped_ros2_motion_plan"] = (
            export_quadruped_ros2_motion_plan(
                result.quadruped_search_plan,
                path / "quadruped_ros2_motion_plan.json",
            )
        )
    if result.reasoned_predictive_scene_graph is not None:
        json_path, graphml_path = export_reasoned_predictive_scene_graph(
            result.reasoned_predictive_scene_graph,
            json_path=path / "reasoned_predictive_scene_graph.json",
            graphml_path=path / "reasoned_predictive_scene_graph.graphml",
        )
        outputs["reasoned_predictive_scene_graph"] = json_path
        outputs["reasoned_predictive_scene_graph_graphml"] = graphml_path
    if result.llm_reasoning is not None:
        outputs["actionability_report"] = _write_actionability_report(
            path / "actionability_report.md",
            result,
        )
        if image_path is not None:
            outputs["reasoned_annotated_scene"] = (
                export_reasoned_annotated_image(
                    result.base_scene,
                    result.llm_reasoning,
                    image_path,
                    path / "reasoned_annotated_scene.png",
                )
            )
    if result.visual_grounding_report is not None:
        outputs["visual_grounding_report"] = _write_json(
            path / "visual_grounding_report.json",
            result.visual_grounding_report,
        )
    if result.experience_writes:
        outputs["llm_experience_writes"] = _write_json(
            path / "llm_experience_writes.json",
            result.experience_writes,
        )
    return outputs


def _write_json(path: Path, payload: object) -> Path:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def _write_report(path: Path, result: KnowledgeAwareSceneResult) -> Path:
    lines = [
        "# 场景推理报告",
        "",
        f"任务：{result.parsed_task.raw_text}",
        f"任务类型：{result.parsed_task.task_type}",
        "",
        "## 推理摘要",
        "",
        result.reasoning_summary_zh,
        "",
        "## 最终回答",
        "",
        result.final_answer_zh,
        "",
        "## 假设",
        "",
    ]
    for hypothesis in result.hypotheses:
        lines.append(
            f"- {hypothesis.hypothesis_id}: {hypothesis.possible_location} "
            f"(p={hypothesis.probability})"
        )
    if result.motion_horizon_decision is not None:
        decision = result.motion_horizon_decision
        lines.extend(
            [
                "",
                "## 运动视界决策",
                "",
                f"- 运动策略：{decision.profile}",
                f"- 平台避障假设：{decision.platform_obstacle_avoidance_assumed}",
                f"- 场景类型：{decision.scene_type}",
                f"- 任务阶段：{decision.task_phase}",
                f"- LLM 推荐距离：{decision.llm_recommended_horizon_m}",
                f"- 规则最大距离：{decision.max_allowed_distance_m}m",
                f"- 最终导出距离：{decision.recommended_distance_m}m",
                f"- 是否运动后重观测：{decision.requires_stop_after_motion}",
                f"- 说明：{decision.decision_reason_zh}",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_actionability_report(
    path: Path,
    result: KnowledgeAwareSceneResult,
) -> Path:
    lines = [
        "# 机械狗动作可执行性报告",
        "",
        "所有候选动作均已通过机械狗能力白名单校验。",
        "",
        "## 门控记录",
        "",
    ]
    if result.actionability_notes_zh:
        lines.extend(f"- {item}" for item in result.actionability_notes_zh)
    else:
        lines.append("- 未发现需要改写的越界动作。")
    lines.extend(["", "## 最终视角动作", ""])
    if result.quadruped_search_plan is not None:
        lines.extend(
            (
                f"- {step.step_id}: {step.primitive.value} — {step.reason_zh}"
                + (
                    f"（motion_horizon={step.motion_horizon_m:.2f}m, "
                    f"policy={step.motion_policy}）"
                    if step.motion_horizon_m is not None
                    else ""
                )
            )
            for step in result.quadruped_search_plan.steps
        )
        lines.extend(
            f"- 不可执行区域：{item}"
            for item in result.quadruped_search_plan.non_executable_notes_zh
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
