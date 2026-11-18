"""任务 Agent 与 WebUI Agent 面板的契约测试（计划书 §5.3 / §5.4 / §3.1）。"""

from __future__ import annotations

import json
import unittest

from app.live_robot.search_event import TARGET_MATCH_UPDATED, SearchEvent
from app.spatial.place_graph import PlaceGraph
from app.live_robot.search_state_store import SearchStateStore, _downstream_status
from app.reasoning.semantic_navigation.models import (
    GraphMatchResult,
    GraphMatchState,
    SearchReasoningContext,
)
from app.reasoning.semantic_navigation.search_reasoner import (
    COARSE_LR_DEG,
    HEADING_SOURCE_COARSE_LR,
    HEADING_SOURCE_MEASURED,
    SemanticNavigationReasoner,
)
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory
from app.reasoning.semantic_navigation.task_agent import (
    attach_trace,
    build_plan,
    enrich_context,
    summarize_topology,
)

LOCKED_REASON_ZH = "仅目标节点形成 PARTIAL，缺少可检查锚点，因此转入未观测方向探索"


def _context(**over) -> SearchReasoningContext:
    base = dict(
        graph_match=GraphMatchResult(
            state=GraphMatchState.PARTIAL, score=0.425,
            matched_scene_node_ids=["obj_bin_001"],
        ),
        scene_graph={"nodes": [{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                                "attributes": {"heading_delta_deg": -3.58}}], "edges": []},
        negative_memory=SemanticSearchMemory(),
    )
    base.update(over)
    return SearchReasoningContext(**base)


def _traced(**over) -> dict:
    context = _context(**over)
    directive = attach_trace(context, SemanticNavigationReasoner().propose(context))
    return directive.decision_trace


class TaskAgentTraceTests(unittest.TestCase):
    def test_trace_is_the_v1_schema_and_points_back_at_the_directive(self):
        context = _context()
        directive = attach_trace(context, SemanticNavigationReasoner().propose(context))
        trace = directive.decision_trace
        self.assertEqual(trace["schema_version"], "task_agent_trace_v1")
        self.assertEqual(trace["trace_id"], directive.directive_id)
        self.assertIs(trace["candidates"], directive.decision_trace["candidates"])

    def test_partial_without_anchor_uses_the_locked_chinese_reason(self):
        trace = _traced(task_context={"canonical_target": "白色垃圾桶"})
        self.assertEqual(
            trace["selected_preference"]["reason_code"], "partial_without_anchor"
        )
        self.assertIn(LOCKED_REASON_ZH, trace["selected_preference"]["reason_zh"])

    def test_attach_trace_returns_a_copy_and_leaves_the_original_alone(self):
        context = _context()
        original = SemanticNavigationReasoner().propose(context)
        self.assertIsNone(original.decision_trace.get("schema_version"))
        traced = attach_trace(context, original)
        self.assertIsNot(traced, original)
        self.assertEqual(traced.directive_id, original.directive_id)
        self.assertIsNone(original.decision_trace.get("schema_version"))

    def test_trace_never_claims_target_confirmation_or_motion_authority(self):
        for over in ({}, {"graph_match": GraphMatchResult(
            state=GraphMatchState.STRONG, score=0.9,
            matched_scene_node_ids=["obj_bin_001"])}):
            safety = _traced(**over)["safety"]
            self.assertIs(safety["can_confirm_target"], False)
            self.assertIs(safety["can_authorize_motion"], False)
            self.assertIs(safety["allow_forward"], False)

    def test_missing_sources_lists_what_is_absent_instead_of_zero(self):
        trace = _traced()
        self.assertIn("topology_summary", trace["inputs"]["missing_sources"])
        self.assertIn("task_context", trace["inputs"]["missing_sources"])
        self.assertIsNone(trace["inputs"]["topology"])
        codes = [item["code"] for item in trace["uncertainties"]]
        self.assertIn("missing_topology_summary", codes)

    def test_absent_knowledge_is_recorded_as_absent_not_as_a_negative(self):
        """计划书 §1.3 第 1 条：没有知识项时，trace 记 `knowledge=[]` 与明确的零检索状态。

        这一条守的是"空集不等于负证据"：没有知识**不能**被读成"查过了、没有"，
        否则下游会把"没检索到"当成"这些方向都不行"。
        """
        trace = _traced()  # 默认上下文不带 knowledge_items
        self.assertEqual(trace["knowledge"], [])
        codes = [item["code"] for item in trace["uncertainties"]]
        self.assertIn("no_knowledge_retrieved", codes)
        detail = next(item["detail"] for item in trace["uncertainties"]
                      if item["code"] == "no_knowledge_retrieved")
        self.assertIn("不构成", detail)  # 明说不是负证据，而不是留白让人猜

    def test_knowledge_is_context_only_until_it_lands_on_a_visible_entity(self):
        knowledge = [{"id": "object_prior:phone", "content_zh": "手机常在桌上",
                      "source": "data/scene_kb/object_location_priors.json",
                      "confidence": 0.86,
                      "metadata": {"related_objects": ["桌子"]}}]
        unused = _traced(knowledge_items=knowledge)
        self.assertEqual([item["status"] for item in unused["knowledge"]], ["context_only"])
        used = _traced(
            knowledge_items=knowledge,
            scene_graph={"nodes": [
                {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                 "attributes": {"heading_delta_deg": -3.58}},
                {"node_id": "obj_desk", "label_zh": "桌子",
                 "attributes": {"heading_delta_deg": 30.0}},
            ], "edges": []},
        )
        entry = used["knowledge"][0]
        self.assertEqual(entry["status"], "used")
        self.assertTrue(entry["supports"])

    def test_legacy_backend_gets_an_empty_but_explicit_candidate_audit(self):
        from app.reasoning.semantic_navigation.search_reasoner import LegacyReasoner
        from app.live_robot.step_planner import plan_scan_step

        class _Cfg:
            scan_turn_deg = 30.0
            scan_span = 3
            max_radius_m = 1.0
            forward_estimate_m = 0.15

        context = _context()
        context.legacy_scan_candidate = plan_scan_step(
            0, scan_turn_deg=30.0, scan_span=3, distance_m=0.0,
            max_radius_m=1.0, forward_estimate_m=0.15,
        )
        trace = attach_trace(context, LegacyReasoner().propose(context)).decision_trace
        self.assertEqual(trace["candidates"], [])
        self.assertEqual(
            trace["selected_preference"]["reason_code"], "legacy_scan_sequence"
        )
        self.assertIn(
            "no_candidate_ranking", [item["code"] for item in trace["uncertainties"]]
        )

    def test_whole_trace_is_strict_json(self):
        json.dumps(_traced(), allow_nan=False)


class EvidenceProvenanceTests(unittest.TestCase):
    """每条证据要带着"这个方向是怎么来的"一起进记录。

    光看角度分不出来历：写死的 ±25（`COARSE_LR_DEG`，仓库里没有任何测量支持
    这个数）和量出来的方位角在 trace 里长得一模一样 —— 同一个上下文里
    `heading_delta_deg: -25.0` 与 `stable_position_2d: right` 产出的整份 v1
    trace 曾经逐字节相同。真机 116 个录制、1036 个场景节点里
    `heading_delta_deg` 出现 0 次，也就是说面板上每一个"证据方位"都是那个常数：
    不标出来历，读的人会把它当成测量值。
    """

    MEASURED = {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                "attributes": {"heading_delta_deg": -25.0}}
    # 同一条几何，另一种写法。左右之外的未知位置不再兜底成某个角。
    COARSE = {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
              "attributes": {"stable_position_2d": "right"}}
    COARSE_LEFT = {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                   "attributes": {"stable_position_2d": "left"}}
    NO_DIRECTION = {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"position_2d": "center"}}

    def _evidence(self, node):
        trace = _traced(
            graph_match=GraphMatchResult(
                state=GraphMatchState.PARTIAL, score=0.425,
                matched_scene_node_ids=["obj_bin_001"],
            ),
            scene_graph={"nodes": [node], "edges": []},
        )
        return trace["selected_preference"]["evidence"]

    def test_the_angle_alone_cannot_tell_measured_from_guessed(self):
        measured = self._evidence(self.MEASURED)
        coarse = self._evidence(self.COARSE)

        self.assertEqual(len(measured), 1)
        self.assertEqual(len(coarse), 1)
        self.assertEqual(measured[0]["heading_delta_deg"], -25.0)
        self.assertEqual(coarse[0]["heading_delta_deg"], -25.0)

    def test_the_two_ways_of_getting_an_angle_are_told_apart(self):
        self.assertEqual(
            self._evidence(self.MEASURED)[0]["heading_source"],
            HEADING_SOURCE_MEASURED,
        )
        self.assertEqual(
            self._evidence(self.COARSE)[0]["heading_source"],
            HEADING_SOURCE_COARSE_LR,
        )
        # 粗推角就是那个常数本身，两个名字必须指着同一个数，否则面板上写的
        # "按左右粗推"和排序实际用的角会对不上。
        self.assertEqual(COARSE_LR_DEG, 25.0)

    def test_both_sides_of_the_coarse_guess_are_labelled_the_same_way(self):
        # 左右是两条各自 return 的分支，改其中一条碰不到另一条。只钉右边的话，
        # 左边被改成"量出来的"照样全绿 —— 这一条是校准发现的缺口。
        left = self._evidence(self.COARSE_LEFT)[0]
        right = self._evidence(self.COARSE)[0]

        self.assertEqual(left["heading_source"], HEADING_SOURCE_COARSE_LR)
        self.assertEqual(right["heading_source"], HEADING_SOURCE_COARSE_LR)
        self.assertEqual(left["heading_delta_deg"], COARSE_LR_DEG)
        self.assertEqual(right["heading_delta_deg"], -COARSE_LR_DEG)

    def test_the_anchor_path_carries_its_provenance_too(self):
        """±25 真正走到面板上的那条路，就是锚点分支。

        候选表里的方向是 ±30/±60/±90 这几个固定候选（真机 1026 份 trace 实测），
        而锚点分支的方位直接来自 `_entity_heading_delta` —— 真机上
        `heading_delta_deg` 一个都没有，所以锚点给出的那个角度**恒为** `coarse_lr`
        的 ±25。这条分支此前不带 decision_trace，于是信封里
        `raw.get("reason_code")` 取到空、回落成 "legacy_scan_sequence"：面板一边
        写着 `inspect_anchor`，一边说这决策是旧扫描后端做的，而那句方位没有任何来历。
        """
        from app.reasoning.semantic_navigation.search_reasoner import COARSE_LR_DEG

        context = SearchReasoningContext(
            graph_match=GraphMatchResult(
                state=GraphMatchState.PARTIAL, score=0.5,
                supporting_anchor_scene_node_ids=["anchor"],
            ),
            scene_graph={"nodes": [{"node_id": "anchor", "label_zh": "饮水机",
                                    "attributes": {"position_2d": "left"}}],
                         "edges": []},
        )
        directive = SemanticNavigationReasoner().propose(context)
        self.assertEqual(directive.kind.value, "inspect_anchor")
        self.assertEqual(directive.preferred_heading_delta_deg, COARSE_LR_DEG)

        preference = attach_trace(context, directive).decision_trace["selected_preference"]
        # 说成 legacy 就是把一个语义锚点决策归给了另一个后端。
        self.assertEqual(preference["reason_code"], "anchor_inspect")
        self.assertEqual(
            preference["evidence"],
            [{"source": "goal_graph", "ref": "anchor", "label": "饮水机",
              "heading_delta_deg": COARSE_LR_DEG,
              "heading_source": HEADING_SOURCE_COARSE_LR}],
        )

    def test_a_direction_that_cannot_be_established_never_becomes_evidence(self):
        # "center" 既不是左也不是右，也不是量出来的角。猜一个 0° 当证据比
        # 没有证据更坏，所以它整条不进 evidence（只在 uncertainties 里）。
        self.assertEqual(self._evidence(self.NO_DIRECTION), [])

    def test_each_evidence_keeps_the_ref_and_label_that_point_back_at_it(self):
        # 来历是加在原有字段上，不是替换：ref/label 仍要能指回场景图里的实体，
        # 否则面板只能说"有个方向"，说不出是谁的方向。
        entry = self._evidence(self.COARSE)[0]
        self.assertEqual(entry["ref"], "obj_bin_001")
        self.assertEqual(entry["label"], "垃圾桶")
        self.assertEqual(entry["source"], "goal_graph")

    def test_knowledge_evidence_is_labelled_as_knowledge_not_as_goal_graph(self):
        # 同一个实体可以同时被目标图和知识库指到。两条都要在，且各自带自己的
        # 来历 —— 面板上"目标图"和"知识库"是两回事。
        trace = _traced(
            graph_match=GraphMatchResult(
                state=GraphMatchState.PARTIAL, score=0.425,
                matched_scene_node_ids=["obj_bin_001"],
            ),
            scene_graph={"nodes": [self.COARSE], "edges": []},
            knowledge_items=[{
                "id": "object_prior:bin", "content_zh": "垃圾桶常在桌边",
                "source": "data/scene_kb/object_location_priors.json",
                "confidence": 0.8,
                "metadata": {"related_objects": ["垃圾桶"]},
            }],
        )
        evidence = trace["selected_preference"]["evidence"]
        by_source = {item["source"]: item for item in evidence}

        self.assertEqual(sorted(by_source), ["goal_graph", "knowledge"])
        for item in evidence:
            self.assertEqual(item["heading_source"], HEADING_SOURCE_COARSE_LR)
            self.assertEqual(item["heading_delta_deg"], -25.0)
        self.assertEqual(by_source["knowledge"]["ref"], "object_prior:bin")

    def test_the_envelope_rows_are_json_safe_and_never_carry_a_bare_angle(self):
        # 网页脚本按 heading_source 查表，查不到就写"来源未标注"。所以这里
        # 不能出现 None/缺失 —— 缺失会被前端读成"未标注"，而这一条其实是量出来的。
        entries = self._evidence(self.MEASURED) + self._evidence(self.COARSE)
        json.dumps(entries, allow_nan=False)
        for item in entries:
            self.assertIsInstance(item["heading_source"], str)
            self.assertTrue(item["heading_source"])


class TaskAgentPlanTests(unittest.TestCase):
    def test_default_plan_is_the_five_stage_plan_with_one_active_stage(self):
        plan = build_plan(None, active_id="search_or_explore")
        self.assertEqual(
            [stage["id"] for stage in plan],
            ["understand_task", "retrieve_context", "search_or_explore",
             "verify_target", "report_result"],
        )
        self.assertEqual(sum(1 for stage in plan if stage["status"] == "active"), 1)
        # §3.2：没有任务结束事件，就不许把结果阶段标成完成。
        self.assertEqual(plan[-1]["status"], "pending")

    def test_declared_subtasks_keep_their_order_and_are_classified_honestly(self):
        plan = build_plan({
            "canonical_target": "找到垃圾桶",
            "subtasks": [
                {"id": "s1", "object": "找到垃圾桶", "is_navigation_relevant": True},
                {"id": "s2", "object": "把它拿起来", "requires_manipulation": True},
                {"id": "s3", "object": "推到门外", "blocked_reason": "超出安全边界"},
            ]
        }, active_id="search_or_explore")
        self.assertEqual([stage["id"] for stage in plan], ["s1", "s2", "s3"])
        self.assertEqual(plan[0]["title"], "找到垃圾桶")
        self.assertEqual(plan[0]["status"], "active")
        self.assertEqual(plan[1]["status"], "unsupported_by_current_executor")
        self.assertEqual(plan[2]["status"], "blocked")
        self.assertEqual(plan[2]["detail"], "超出安全边界")

    def test_only_the_subtask_matching_the_current_target_is_active(self):
        """§3.2：active 是单数，且必须是**当前搜索目标**对应的那一个。

        改之前这里只有一个判据（`is_navigation_relevant`），于是两个导航子任务
        会同时被标成"正在进行中"——面板上看起来像机器人同时在干两件事。

        目标故意放在**第二位**：如果目标恰好排第一，"按目标挑"和"取第一个"
        会给出同一个结果，这条断言就分辨不出来了（注错校准 B2 抓到过这一点）。
        """
        plan = build_plan({
            "canonical_target": "蓝色箱子",
            "subtasks": [
                {"id": "s1", "object": "红色雨伞", "is_navigation_relevant": True},
                {"id": "s2", "object": "蓝色箱子", "is_navigation_relevant": True},
            ],
        }, active_id="search_or_explore")
        self.assertEqual([stage["status"] for stage in plan], ["pending", "active"])

    def test_no_subtask_is_active_when_the_current_target_is_unknown(self):
        """认不出当前目标是哪一个时，一个都不标 —— 不许"全都 active"兜底。

        第二个子任务**没有 `object`**：少了"当前目标必须非空"那道门，它的
        `object` 会取成空串，跟空目标"相等"，于是被误标成 active。没有这个
        样本，这条断言同样分辨不出来（注错校准 B3 抓到过）。
        """
        plan = build_plan({
            "subtasks": [
                {"id": "s1", "object": "红色雨伞", "is_navigation_relevant": True},
                {"id": "s2", "raw_text": "去看看门口", "is_navigation_relevant": True},
            ],
        }, active_id="search_or_explore")
        self.assertEqual([stage["status"] for stage in plan], ["pending", "pending"])

    def test_plan_never_marks_a_stage_completed_without_an_event(self):
        plan = build_plan({"subtasks": [{"id": "s1", "is_navigation_relevant": False}]},
                          active_id="search_or_explore")
        self.assertNotIn("completed", [stage["status"] for stage in plan])


class EnrichContextTests(unittest.TestCase):
    def test_enrich_sets_all_four_fields_and_keeps_none_meaning_missing(self):
        context = SearchReasoningContext()
        enrich_context(
            context,
            task_context={"canonical_target": "垃圾桶"},
            knowledge_items=[{"id": "k1"}],
            topology_summary=None,
            run_mode="offline_replay",
        )
        payload = context.to_dict()
        self.assertEqual(payload["run_mode"], "offline_replay")
        self.assertEqual(payload["knowledge_item_count"], 1)
        self.assertIsNone(payload["topology_summary"])
        self.assertEqual(payload["task_context"]["canonical_target"], "垃圾桶")

    def test_summarize_topology_gives_the_same_answer_for_the_live_graph_and_its_dict(self):
        """生产路径传活的 `PlaceGraph`，离线评估器传 `place_graph.json` 的 dict。

        两者必须给出**同一个**摘要，否则 §4.1 / §4.2 的"口径一致"就是空话。
        这也是这条改动的核心判据：改之前传活对象只会得到 `None`（`isinstance`
        那道门把它挡掉了），于是生产路径只能先 `to_dict()` 再传 —— 而
        `to_dict()` 会序列化整个会话的全部观测。
        """
        graph = PlaceGraph()
        graph.register_observation(observation_id="o1", heading_sector=0, objects=["桌子"])
        graph.register_observation(
            observation_id="o2", heading_sector=1, objects=["椅子"],
            observed_displacement_m=1.5,
        )
        live = summarize_topology(graph)
        self.assertIsNotNone(live)
        self.assertEqual(live, summarize_topology(graph.to_dict()))
        self.assertEqual(live["place_count"], 2)
        self.assertEqual(live["edge_count"], 1)
        self.assertEqual(live["current_place_id"], graph.current_place().place_id)

    def test_summarize_topology_does_not_serialize_the_live_graph(self):
        """§18.4：逐帧读，所以**不许**走 `to_dict()` —— 那是 O(整个会话的观测)。

        判据用"会抛异常的替身"，而不是看它跑多快：性能断言在 CI 里不可靠，
        而"有没有调用那个方法"是确定的。这条守的是 4.1-3 那个 O(n²) 陷阱。
        """
        graph = PlaceGraph()
        graph.register_observation(observation_id="o1", heading_sector=0, objects=["桌子"])

        def _boom() -> dict:
            raise AssertionError("summarize_topology 走了 to_dict()，逐帧调用就是 O(n²)")

        graph.to_dict = _boom  # type: ignore[method-assign]
        self.assertIsNotNone(summarize_topology(graph))

    def test_summarize_topology_returns_none_for_absent_or_unusable_input(self):
        self.assertIsNone(summarize_topology(None))
        self.assertIsNone(summarize_topology([]))
        self.assertIsNone(summarize_topology({"unrelated": 1}))
        summary = summarize_topology({
            "places": [{"place_id": "P1"}], "edges": [],
            "current_place_id": "P1",
        })
        self.assertEqual(summary["place_count"], 1)
        self.assertEqual(summary["current_place_id"], "P1")


class AgentSnapshotTests(unittest.TestCase):
    def _event(self, event_type: str, payload: dict, timestamp: float = 100.0) -> SearchEvent:
        return SearchEvent(event_id=1, session_id="search_test", timestamp=timestamp,
                           event_type=event_type, payload=payload)

    def _trace(self, trace_id: str = "directive_abc") -> dict:
        return {"schema_version": "task_agent_trace_v1", "trace_id": trace_id,
                "candidates": [], "selected_preference": {}}

    def test_snapshot_has_an_empty_agent_block_by_default(self):
        agent = SearchStateStore().snapshot()["agent"]
        self.assertIsNone(agent["trace"])
        self.assertEqual(agent["downstream_status"], "pending")

    def test_target_match_publishes_the_trace_into_the_snapshot(self):
        store = SearchStateStore()
        store.apply(self._event(TARGET_MATCH_UPDATED, {
            "directive": {"directive_id": "directive_abc",
                          "decision_trace": self._trace()},
        }, timestamp=123.0))
        agent = store.snapshot()["agent"]
        self.assertEqual(agent["trace"]["trace_id"], "directive_abc")
        self.assertEqual(agent["updated_at"], 123.0)

    def test_a_directive_without_a_trace_does_not_wipe_the_panel(self):
        store = SearchStateStore()
        store.apply(self._event(TARGET_MATCH_UPDATED, {
            "directive": {"directive_id": "d1", "decision_trace": self._trace("keep_me")},
        }))
        store.apply(self._event(TARGET_MATCH_UPDATED, {
            "directive": {"directive_id": "d2"},
        }, timestamp=200.0))
        self.assertEqual(store.snapshot()["agent"]["trace"]["trace_id"], "keep_me")

    def test_decision_recorded_links_the_real_decision_to_the_latest_trace(self):
        store = SearchStateStore()
        store.apply(self._event(TARGET_MATCH_UPDATED, {
            "directive": {"directive_id": "directive_abc",
                          "decision_trace": self._trace("directive_abc")},
        }))
        store.apply(self._event("DECISION_RECORDED", {
            "decision_id": "decision_1",
            "candidate_ranking": [{"goal": {"provenance": {
                "source": "semantic_navigation_directive",
                "directive_id": "directive_abc"}}}],
        }))
        agent = store.snapshot()["agent"]
        self.assertEqual(agent["decision_id"], "decision_1")
        self.assertEqual(agent["downstream_status"], "accepted")

    def test_restore_keeps_the_agent_panel_for_a_reconnect(self):
        store = SearchStateStore()
        store.apply(self._event(TARGET_MATCH_UPDATED, {
            "directive": {"directive_id": "directive_abc",
                          "decision_trace": self._trace("directive_abc")},
        }))
        restored = SearchStateStore()
        restored.restore(store.snapshot())
        self.assertEqual(restored.snapshot()["agent"]["trace"]["trace_id"], "directive_abc")


class DownstreamStatusTests(unittest.TestCase):
    def test_only_checkable_evidence_decides_accepted_or_rejected(self):
        trace = {"trace_id": "directive_abc"}
        self.assertEqual(_downstream_status(trace, {"candidate_ranking": [{"goal": {
            "provenance": {"source": "semantic_navigation_directive",
                           "directive_id": "directive_abc"}}}]}), "accepted")
        self.assertEqual(_downstream_status(trace, {"candidate_ranking": [{"goal": {
            "provenance": {"source": "local_goal_executor"}}}]}), "rejected")

    def test_uncheckable_or_absent_evidence_stays_pending_and_never_guesses(self):
        trace = {"trace_id": "directive_abc"}
        self.assertEqual(_downstream_status(None, {"candidate_ranking": []}), "pending")
        self.assertEqual(_downstream_status(trace, {}), "pending")
        # 来源是语义指令但 directive_id 对不上 —— 可能是别的轮次，不下结论。
        self.assertEqual(_downstream_status(trace, {"candidate_ranking": [{"goal": {
            "provenance": {"source": "semantic_navigation_directive",
                           "directive_id": "other"}}}]}), "pending")
        self.assertEqual(_downstream_status(trace, {"candidate_ranking": [{"goal": {}}]}), "pending")


if __name__ == "__main__":
    unittest.main()
