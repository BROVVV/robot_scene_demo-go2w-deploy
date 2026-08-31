"""Prompt templates for the vision scene understanding model."""

from __future__ import annotations

import json

from app.schemas import LLMReasoningRequest


SYSTEM_PROMPT = """你是机器狗的快速视觉场景理解模块。

目标是在单张图片中快速识别与目标定位、避障、导航相关的关键物体，并输出合法 JSON。

要求：
1. 优先速度和稳定性，不要输出冗长解释。
2. 输出 6 到 12 个关键独立物体；复杂场景最多 15 个。
3. 不要重复输出同一个物体。
4. 对“挂着黄衣服的椅子”这类目标，要把椅子和衣服分开作为 object，并在 relations 中写出关系。
5. 所有距离、角度都是估计值，必须保守。
6. 只输出 JSON 对象，不要 Markdown，不要解释文本。
7. 可以给出结构化任务和候选位置线索，但不要输出长篇自由文本推理；后续系统会用视觉证据门控、运行时 LLM 假设和观察记忆做最终评分与规划。
"""


def build_user_prompt(target_text: str, extra_instructions: str | None = None) -> str:
    prompt = f"""用户目标描述：{target_text}

请快速分析图片，返回下面结构的 JSON：

{{
  "scene_summary_zh": "一句话中文场景摘要",
  "objects": [
    {{
      "id": "obj_001",
      "name": "chair",
      "name_zh": "椅子",
      "category": "furniture",
      "color": "black",
      "attributes": ["key visible attributes"],
      "visible": true,
      "position": {{
        "horizontal": "left | center | right",
        "vertical": "front | middle | back",
        "relative_to_robot": "front-left",
        "estimated_distance_m": 1.2
      }},
      "bbox_2d": {{"x1": 0.0, "y1": 0.0, "x2": 1.0, "y2": 1.0}},
      "confidence": 0.8
    }}
  ],
  "relations": [
    {{
      "source_id": "obj_001",
      "target_id": "obj_002",
      "relation_type": "left_of | right_of | in_front_of | behind | on | under | above | below | in | near | far | contains | occluding",
      "description_zh": "中文关系",
      "estimated_distance_m": null,
      "confidence": 0.7
    }}
  ],
  "topology": {{"nodes": [], "edges": []}},
  "target_decision": {{
    "target_text": "{target_text}",
    "is_present": true,
    "matched_object_ids": ["obj_001"],
    "match_reason_zh": "中文原因",
    "confidence": 0.8
  }},
  "route_plan": {{
    "route_type": "approach_visible_target | explore_likely_location",
    "summary_zh": "中文路线摘要",
    "steps": [
      {{
        "step_id": 1,
        "action": "move_forward | move_backward | turn_left | turn_right | stop",
        "distance_m": 1.0,
        "turn_angle_deg": null,
        "description_zh": "中文动作"
      }}
    ],
    "safety_notes_zh": ["单张图片估计，仅用于 Demo"]
  }},
  "task_understanding": {{
    "task_type": "find_object | count_objects | inspect_area | check_door_state | find_room | navigate_to_location | verify_condition | summarize_scene | compare_states",
    "entities": ["phone"],
    "constraints": ["on desk"],
    "uncertainty": "中文不确定性说明"
  }},
  "scene_reasoning_hints": {{
    "scene_type": "office",
    "candidate_locations": ["desk surface", "beside keyboard"],
    "supporting_evidence": ["visible desk", "visible keyboard"],
    "recommended_next_observation": "靠近桌面右侧重新观察"
  }}
}}

约束：
- objects 只列关键物体，目标相关物体、障碍物、桌椅、人物、显示器、主机、箱子、鞋、篮子、线缆等优先。
- relations 优先输出强语义关系：承载、悬挂、遮挡、靠近、左右、前后。至少覆盖目标相关物体及明显相邻/承载物体。
- 不要为了关系数量枚举所有两两组合；系统会根据 bbox 自动补全稀疏空间关系以保证拓扑连通。
- 不要为了凑数量重复列出同一张桌子、同一台显示器、同一台主机。
- 目标是“挂着黄衣服的椅子”时，matched_object_ids 应包含相关椅子和黄色衣服。
- bbox_2d 坐标必须在 0 到 1。
- confidence 必须在 0 到 1。
- 不确定距离或角度用 null。
- task_understanding 和 scene_reasoning_hints 是可选辅助字段；如果输出，必须保持结构化 JSON，不要写完整思维链。
- 只输出 JSON。"""

    if extra_instructions:
        prompt += f"\n\n额外复查要求：\n{extra_instructions}"

    return prompt


def build_situated_search_reasoning_prompt(
    request: LLMReasoningRequest,
) -> list[dict[str, str]]:
    schema = {
        "scene_interpretation_zh": "场景解释",
        "target_search_logic_zh": "简洁搜索逻辑，不输出思维链",
        "hypotheses": [
            {
                "hypothesis_id": "hyp_001",
                "target_name": request.target_text,
                "status": "inferred",
                "candidate_region_zh": "候选区域",
                "candidate_region_type": "visible_anchor_area",
                "image_region_hint": "left/center/right",
                "supporting_visible_anchor_ids": ["obj_001"],
                "supporting_visible_anchor_names": ["可见锚点"],
                "human_like_rationale_zh": "简洁依据",
                "expected_visual_cues_zh": ["后续应看到的线索"],
                "suggested_detector_prompts_en": ["open vocabulary term"],
                "suggested_verification_question_zh": "下一帧验证问题",
                "confidence": 0.5,
                "uncertainty_zh": "不确定性",
                "actionability": "needs_reobservation",
                "quadruped_view_strategy": ["turn_left", "stop_and_reobserve"],
                "safety_notes_zh": ["安全说明"],
                "memory_sources": [],
                "should_not_mark_found": True,
                "max_executable_distance_m": 3.0,
                "execution_assumption": "platform_obstacle_avoidance_assumed",
            }
        ],
        "global_uncertainty_zh": "全局不确定性",
        "recommended_next_observation_zh": "下一观察建议",
        "no_target_found_policy_zh": "未找到策略",
        "recommended_motion_horizon_m": 3.0,
        "motion_horizon_reason_zh": "当前区域较开阔，目标尚未出现，建议移动到较远观察点扩大视野。",
        "motion_profile_hint": "platform_assisted_open_area",
        "requires_stop_after_motion": True,
        "reasoning_available": True,
        "error_message": None,
    }
    system = """你是机械狗第一视角目标搜索系统中的情境推理模块。
你正在为一只具备基础避障能力的机械狗规划下一观察动作。
你只能提出观察视角与高层语义移动距离建议，不能假设有机械臂、开合、翻找、拿取、低头俯视或近距离精查能力。
不要编写避障逻辑，不要假设自己能直接控制底层避障；真实安全由机械狗底层平台/SDK/ROS 安全层兜底。
硬性规则：
1. 除非 observed_facts.target_observed=true，否则目标状态只能是 inferred 或 unreachable，should_not_mark_found 必须为 true。
2. 禁止建议打开、翻找、拿取、移动物体、低头查看表面、检查容器内部。
3. 必须操作容器或遮挡物的假设标为 needs_human 或 unsafe_or_impossible。
4. quadruped_view_strategy 只能使用 capability_contract.allowed_primitives。
5. 每个假设引用可见锚点；没有锚点时明确写空列表。
6. 空间开阔且目标尚未出现时，recommended_motion_horizon_m 可建议 2 到 5 米；普通室内搜索建议 0.8 到 2 米；目标候选出现或确认阶段建议 0.3 到 0.8 米。
7. 不要固定每 0.5 米停一次；所有距离只是建议，最终会由 Motion Horizon Planner 根据配置和硬上限裁剪。
8. 输出严格 JSON，不输出 Markdown，不输出长篇思维链。"""
    payload = request.model_dump(mode="json")
    user = (
        "根据以下输入生成最多 "
        f"{request.max_hypotheses} 个搜索假设。\n输入："
        f"{json.dumps(payload, ensure_ascii=False)}\n输出结构："
        f"{json.dumps(schema, ensure_ascii=False)}"
    )
    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user},
    ]
