"""裸目标缺陷的回归测试：无锚点的 PARTIAL 不许再自称"已匹配到任务锚点"。

改造前 `SemanticNavigationReasoner.propose` 对 PARTIAL 无条件发
`inspect_anchor(anchor=None)` 并把 heading 硬编成 +30°，reason 里写着
"已匹配到任务锚点" —— 而 `supporting_anchor_scene_node_ids` 是空的。
真机 57 条录制里 4 条命中这条路（计划书 §5.3）。

这里同时钉住改造的**边界**：
  * 有方向、可追溯的锚点仍然走 INSPECT_ANCHOR，且用真实几何角度，
    不是 25/30 这类兜底常数；
  * 无证据时排序必须完全回落到原来的覆盖度/负记忆规则。
"""

from __future__ import annotations

import json
import unittest
from dataclasses import replace
from types import SimpleNamespace

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import scenario_no_target
from app.live_robot.search_directive_adapter import directive_to_step_plan
from app.navigation.backend_factory import MockBackend
from app.navigation.candidate_goal_generator import (
    _forward_fallback_block_reason,
    generate_live_exploration_candidates,
)
from app.navigation.exploration_config import (
    CandidateConfig,
    ExplorationBudget,
    load_exploration_policy,
)
from app.navigation.exploration_graph import ExplorationGraph, ObservationNode
from app.navigation.models import (
    GOAL_INSPECT_ANCHOR,
    GOAL_RELATIVE_MOVE,
    GOAL_ROTATE_VIEW,
    LiveObservation,
)
from app.navigation.robot_backend import RobotCapabilities
from app.navigation.safety_models import (
    STATE_CLEAR,
    STATE_OCCUPIED,
    LocalSafetySnapshot,
    PerceptionQuality,
)
from app.reasoning.semantic_navigation.models import (
    GraphMatchResult,
    GraphMatchState,
    SearchDirectiveKind,
    SearchReasoningContext,
)
from app.reasoning.semantic_navigation.search_reasoner import (
    HybridReasoner,
    SemanticNavigationReasoner,
)
from app.reasoning.semantic_navigation.semantic_memory import SemanticSearchMemory

ANCHOR_CLAIM_ZH = "已匹配到任务锚点"


def _context(state, score=0.0, *, anchors=None, matched=None, nodes=None,
             sectors=None, yaw=0.0, knowledge=None, memory=None, evidence=None):
    match = GraphMatchResult(
        state=state, score=score,
        matched_scene_node_ids=matched or [],
        supporting_anchor_scene_node_ids=anchors or [],
        evidence_refs=evidence or [],
    )
    return SearchReasoningContext(
        graph_match=match,
        scene_graph={"nodes": nodes or [], "edges": []},
        negative_memory=memory if memory is not None else SemanticSearchMemory(),
        observed_heading_sectors=sectors or [],
        robot_yaw_deg=yaw,
        knowledge_items=knowledge or [],
    )


def _ranking(directive) -> dict[float, dict]:
    return {
        row["relative_heading_deg"]: row
        for row in directive.decision_trace["candidate_ranking"]
    }


class BareTargetDefectTests(unittest.TestCase):
    """§5.3：目标短语只给出目标本身、给不出关系时，锚点必然为空。"""

    def _bare_target_context(self):
        # 目标自身被看见（matched），但目标图里没有 anchor/context 角色节点，
        # 于是 supporting_anchor_scene_node_ids 为空 —— 这就是真机那 4 条的形态。
        return _context(
            GraphMatchState.PARTIAL, 0.425,
            matched=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"position_2d": "center"}}],
        )

    def test_partial_without_anchor_does_not_claim_an_anchor(self):
        directive = SemanticNavigationReasoner().propose(self._bare_target_context())
        self.assertEqual(directive.kind, SearchDirectiveKind.EXPLORE_UNSEEN)
        self.assertNotIn(ANCHOR_CLAIM_ZH, directive.reason_zh)
        self.assertIsNone(directive.anchor_scene_node_id)
        self.assertIsNone(directive.anchor_label)
        self.assertEqual(
            directive.decision_trace["reason_code"], "partial_without_anchor"
        )

    def test_undirectionable_anchor_falls_back_and_records_the_gap(self):
        # 锚点候选存在，但没有任何可转成角度的空间信息 ⇒ 仍然不算有效锚点。
        context = _context(
            GraphMatchState.PARTIAL, 0.5, anchors=["anchor"],
            nodes=[{"node_id": "anchor", "label_zh": "饮水机",
                    "attributes": {"position_2d": "center"}}],
        )
        directive = SemanticNavigationReasoner().propose(context)
        self.assertNotEqual(directive.kind, SearchDirectiveKind.INSPECT_ANCHOR)
        self.assertIn(
            "anchor_direction_unknown", directive.decision_trace["uncertainties"]
        )

    def test_directional_anchor_still_inspects_with_real_geometry(self):
        # 改造的边界：锚点既然给得出方向，就必须继续 inspect，而且角度是
        # 场景图给的那一个 —— 不许退回 25/30 这类兜底常数。
        context = _context(
            GraphMatchState.PARTIAL, 0.5, anchors=["anchor"],
            nodes=[{"node_id": "anchor", "label_zh": "饮水机",
                    "attributes": {"heading_delta_deg": 42.0}}],
        )
        directive = SemanticNavigationReasoner().propose(context)
        self.assertEqual(directive.kind, SearchDirectiveKind.INSPECT_ANCHOR)
        self.assertEqual(directive.preferred_heading_delta_deg, 42.0)
        self.assertEqual(directive.anchor_label, "饮水机")


class EvidenceRankingTests(unittest.TestCase):
    """成对敏感性：同一段代码，喂相反的证据，必须给出相反的方向。"""

    def _with_scene_evidence(self, angle_deg: float):
        return _context(
            GraphMatchState.PARTIAL, 0.3485,
            matched=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"heading_delta_deg": angle_deg}}],
        )

    def test_scene_evidence_steers_candidate_choice(self):
        left = SemanticNavigationReasoner().propose(self._with_scene_evidence(-60.0))
        right = SemanticNavigationReasoner().propose(self._with_scene_evidence(60.0))
        self.assertEqual(left.preferred_heading_delta_deg, -60)
        self.assertEqual(right.preferred_heading_delta_deg, 60)

    def test_evidence_off_the_candidate_set_maps_to_the_nearest_one(self):
        # 证据在 -100°，候选里最近的离散方向是 -90°。
        directive = SemanticNavigationReasoner().propose(self._with_scene_evidence(-100.0))
        self.assertEqual(directive.preferred_heading_delta_deg, -90)

    def test_absent_evidence_is_null_in_the_trace_and_defers_to_coverage(self):
        # yaw=90 时 -30 落在 sector 2、+30 落在 sector 4。只把 sector 2 标成
        # 已观测：覆盖度项是唯一能让 +30 赢过 -30 的东西（两者 abs 都是 30，
        # 同分时按候选顺序本该取 -30）。所以这条断言同时钉住了两件事：
        # 没有证据时 trace 必须是 null，以及覆盖度项仍然在排序键里起作用。
        context = _context(GraphMatchState.ZERO, sectors=[2], yaw=90.0)
        directive = SemanticNavigationReasoner().propose(context)
        self.assertEqual(directive.preferred_heading_delta_deg, 30)
        rows = directive.decision_trace["candidate_ranking"]
        # 没有证据 ⇒ support_refs 全空、排序键第二项（语义距离）是 null 而不是 0。
        # 第一项现在是覆盖度/负记忆，见 `_rank_key`。
        self.assertTrue(all(row["semantic_support_refs"] == [] for row in rows))
        self.assertTrue(all(row["knowledge_support_refs"] == [] for row in rows))
        self.assertTrue(all(row["rank_key"][1] is None for row in rows))
        self.assertTrue(_ranking(directive)[-30]["observed"])
        self.assertFalse(_ranking(directive)[30]["observed"])

    def test_knowledge_evidence_requires_the_object_to_be_in_the_scene(self):
        knowledge = [{"item_id": "kb:phone",
                      "metadata": {"related_objects": ["桌子"]}}]
        absent = SemanticNavigationReasoner().propose(
            _context(GraphMatchState.ZERO, knowledge=knowledge)
        )
        self.assertTrue(all(
            row["knowledge_support_refs"] == []
            for row in absent.decision_trace["candidate_ranking"]
        ))
        present = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.ZERO, knowledge=knowledge,
            nodes=[{"node_id": "obj_desk", "label_zh": "桌子",
                    "attributes": {"heading_delta_deg": 60.0}}],
        ))
        self.assertEqual(present.preferred_heading_delta_deg, 60)
        self.assertTrue(any(
            item["source"] == "knowledge"
            for item in present.decision_trace["evidence"]
        ))

    def test_evidence_refs_are_deduplicated_before_they_reach_the_panel(self):
        # 图匹配自带的 ref 与按方向收上来的 goal_graph 证据指的是同一个节点。
        # 原样拼接会让面板把一处证据显示成两处 —— 那是拿一当二。
        directive = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.3485,
            matched=["obj_bin_001"], evidence=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"heading_delta_deg": -30.0}}],
        ))
        self.assertEqual(directive.evidence_refs, ["obj_bin_001"])

    def test_the_robots_own_observation_yaw_is_not_a_bearing(self):
        # 真机实测（116 个录制 / 1036 个场景节点）：`heading_delta_deg` 一个都没有，
        # `observed_heading_deg` 有 883 个 —— 后者是 semantic_observer.py:252 给每一帧
        # 每一个节点写下的机器人自己的偏航角，同处还标了
        # `position_status = "observation_pose_only"`（仓库自己说它没有方位）。
        # 拿它减当前 yaw 得到的是"机器人自己转了多少度"，与实体在哪无关；原样收作
        # 证据会让排序键首项退化成对称的 |heading| —— 看着像测量值，其实没有信息。
        context = _context(
            GraphMatchState.PARTIAL, 0.3485, matched=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"observed_heading_deg": 0.0,
                                   "position_status": "observation_pose_only"}}],
        )
        directive = SemanticNavigationReasoner().propose(context)
        self.assertEqual(directive.decision_trace["evidence"], [])
        self.assertEqual(directive.evidence_refs, [])
        self.assertTrue(all(
            row["semantic_support_refs"] == []
            for row in directive.decision_trace["candidate_ranking"]
        ))

    def test_coverage_outranks_a_nearby_entity_so_we_do_not_revisit(self):
        # 真机实测（57 会话 A/B，`ab_noobserve` → `ab_C` 恰好只有这 2 条会变）：
        # 证据项排在覆盖度前面时，`explore_go2w_20260914_125819` 会选中 −30、
        # `explore_go2w_20260916_200905` 会选中 +30 —— 两条都是 `obs=True` 的
        # **已观测扇区**，唯一理由是它们的语义距离最小（都是 5.0），而面板上
        # 那一行连一条拒绝理由都写不出来。C 把覆盖度挪到证据前面之后，两条各自
        # 改选"离证据最近的**未观测**方向"（语义距离 55.0）。
        # 这里把那个形态压成最小复现：目标在右边（→ −25°），而 −30 那个扇区刚看过。
        directive = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.3485,
            matched=["obj_bin_001"], sectors=[-1], yaw=0.0,
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"stable_position_2d": "right"}}],
        ))
        self.assertEqual(directive.preferred_heading_delta_deg, -60)
        rows = _ranking(directive)
        self.assertTrue(rows[-30]["observed"], "−30 必须是已观测扇区，否则这条测试没在测覆盖度")
        # 证据确实认出了右边，而且 −30 是离它最近的候选 —— 它只是输给了覆盖度。
        self.assertEqual(rows[-30]["semantic_support_refs"], ["obj_bin_001"])
        self.assertEqual(rows[-30]["rank_key"][1], 5.0)
        self.assertNotEqual(
            directive.preferred_heading_delta_deg, -30,
            "证据不许把候选推回已经看过的扇区",
        )

    def test_rejection_reasons_are_listed_in_the_rank_key_order(self):
        # 面板把 rejection_reasons 按序念出来，第一条读起来就是"它主要输在哪"。
        # 所以罗列顺序必须与 `_rank_key` 的元组顺序一致 —— 错开就会出现
        # "输在覆盖度、却说成离语义证据更远"的假解释，与 `_candidate_trace`
        # 那句"展示的分量、顺序必须就是代码真正用的那一套"是同一个要求。
        # 这里造一个**两项同时输**的候选：−30 既是被观测扇区（覆盖度 0.45 > 0），
        # 又离证据（+90°）120° 远（而胜者 +90 是 0°）。旧顺序会先说语义证据。
        directive = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.3485,
            matched=["obj_bin_001"], sectors=[-1], yaw=0.0,
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"heading_delta_deg": 90.0}}],
        ))
        self.assertEqual(directive.preferred_heading_delta_deg, 90)
        reasons = _ranking(directive)[-30]["rejection_reasons"]
        self.assertEqual(
            reasons[0], "sector_already_observed",
            "面板的第一条理由必须是排序键的第一项（覆盖度），不是语义证据",
        )
        self.assertIn("farther_from_semantic_evidence", reasons)

    def test_negative_memory_still_outranks_evidence_free_tie(self):
        # 没有证据时，负记忆的优先级不能被新键破坏（原行为）。
        memory = SemanticSearchMemory()
        memory.add_negative(target_key="target", heading_sector=-1,
                            reason="not seen", source_event_id="neg_left",
                            confidence=0.9)
        directive = SemanticNavigationReasoner().propose(
            _context(GraphMatchState.ZERO, memory=memory)
        )
        self.assertEqual(directive.preferred_heading_delta_deg, 30)


class NonFiniteEvidenceTests(unittest.TestCase):
    """inf / nan 方向必须等同"定不出方向"，不能当证据用。

    nan 一旦落到排序键首位就会吃掉后面每一个分量：元组比较先找"第一个不相等
    的位置"，而 `nan != nan` 恒真，于是第一位就判"不同"，覆盖度与负记忆再也
    不被读 —— 一个没有方向的候选会赢过有方向的候选。它还会炸掉评估器的
    `json.dumps(..., allow_nan=False)`。
    """

    def _with_poisoned_evidence(self, bad: float):
        # PARTIAL + matched 但无锚点：正好走排序器，且该节点会进证据集。
        # 毒值走 `heading_delta_deg` —— 删掉 `observed_heading_deg` 之后（见
        # `_entity_heading_delta`），它是唯一还能给出显式方位的字段。
        return _context(
            GraphMatchState.PARTIAL, 0.4, matched=["n1"], sectors=[2], yaw=90.0,
            nodes=[{"node_id": "n1", "label_zh": "垃圾桶",
                    "attributes": {"heading_delta_deg": bad}}],
        )

    def test_non_finite_direction_is_not_evidence(self):
        # 方向现在由覆盖度说了算（sector 2 已观测 ⇒ +30 赢），所以本用例真正钉住
        # 非有限值的是下面两条断言：只要 inf/nan 被当成证据，语义距离就不再是 null、
        # support_refs 也不再为空。方向断言保留作兜底。
        baseline = SemanticNavigationReasoner().propose(
            self._with_poisoned_evidence(90.0)  # 有限值 ⇒ 照旧是证据
        )
        for bad in (float("inf"), float("nan"), float("-inf")):
            directive = SemanticNavigationReasoner().propose(
                self._with_poisoned_evidence(bad)
            )
            rows = directive.decision_trace["candidate_ranking"]
            # ① 不许被当成"正对着证据"而翻转决策
            self.assertEqual(
                directive.preferred_heading_delta_deg, 30,
                f"{bad} 不应改变方向选择",
            )
            # ② trace 里一个非有限值都不许有
            self.assertTrue(all(row["rank_key"][1] is None for row in rows))
            self.assertTrue(all(row["semantic_support_refs"] == [] for row in rows))
            self.assertNotIn("Infinity", json.dumps(directive.to_dict(), allow_nan=False))
            self.assertNotIn("NaN", json.dumps(directive.to_dict(), allow_nan=False))
        # ③ 有限方向照旧是证据（守卫只挡非有限值，没有把整条证据通道关掉）
        self.assertEqual(
            [item["source"] for item in baseline.decision_trace["evidence"]],
            ["goal_graph"],
        )

    def test_poisoned_anchor_cannot_leak_a_non_finite_heading(self):
        # 锚点走的是同一个 `_entity_heading_delta`，所以它也必须被挡住：
        # 定不出方向的锚点等于没有锚点，不许发 inspect_anchor。
        directive = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.5, anchors=["anchor"],
            nodes=[{"node_id": "anchor", "label_zh": "饮水机",
                    "attributes": {"heading_delta_deg": float("nan")}}],
        ))
        self.assertNotEqual(directive.kind, SearchDirectiveKind.INSPECT_ANCHOR)
        self.assertIn(
            "anchor_direction_unknown", directive.decision_trace["uncertainties"]
        )
        json.dumps(directive.to_dict(), allow_nan=False)


class SafetyInvariantTests(unittest.TestCase):
    def test_no_branch_allows_forward_or_carries_a_motion_authorization(self):
        nodes = [{"node_id": "anchors", "label_zh": "饮水机",
                  "attributes": {"heading_delta_deg": 30.0}}]
        cases = [
            _context(GraphMatchState.ZERO),
            _context(GraphMatchState.PARTIAL, 0.4, matched=["o1"], nodes=[
                {"node_id": "o1", "label_zh": "垃圾桶", "attributes": {}}]),
            _context(GraphMatchState.PARTIAL, 0.5, anchors=["anchors"], nodes=nodes),
            _context(GraphMatchState.STRONG, 0.9),
            _context(GraphMatchState.STRONG, 0.9, anchors=["anchors"], nodes=nodes),
        ]
        for context in cases:
            for reasoner in (SemanticNavigationReasoner(), HybridReasoner()):
                directive = reasoner.propose(context)
                self.assertFalse(directive.allow_forward)
                self.assertFalse(hasattr(directive, "authorizes_motion"))


class DirectiveTraceContractTests(unittest.TestCase):
    def _cases(self):
        return [
            _context(GraphMatchState.ZERO),
            _context(GraphMatchState.PARTIAL, 0.425, matched=["obj_bin_001"], nodes=[
                {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                 "attributes": {"heading_delta_deg": 0.0}}]),
            _context(GraphMatchState.STRONG, 0.9, matched=["obj_bin_001"], nodes=[
                {"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                 "attributes": {"heading_delta_deg": 0.0}}]),
        ]

    def test_trace_is_strict_json_and_never_smuggles_infinity(self):
        # 无证据时内部排序键是 inf；它一旦漏进 trace，评估器的
        # json.dumps(..., allow_nan=False) 会直接炸。
        for context in self._cases():
            directive = SemanticNavigationReasoner().propose(context)
            payload = json.dumps(directive.to_dict(), allow_nan=False)
            self.assertNotIn("Infinity", payload)

    def test_trace_audits_every_candidate_and_marks_exactly_one_selected(self):
        for context in self._cases():
            directive = SemanticNavigationReasoner().propose(context)
            rows = directive.decision_trace["candidate_ranking"]
            self.assertEqual(len(rows), 6)
            self.assertEqual(sum(1 for row in rows if row["selected"]), 1)
            self.assertTrue(rows[0]["selected"])
            self.assertEqual(
                rows[0]["relative_heading_deg"],
                directive.preferred_heading_delta_deg,
            )
            # 拒绝原因必须只出现在落选的行上。
            self.assertEqual(rows[0]["rejection_reasons"], [])
            self.assertTrue(all(row["rejection_reasons"] for row in rows[1:]))


class ContextSerializationTests(unittest.TestCase):
    def test_context_to_dict_distinguishes_missing_from_empty(self):
        payload = SearchReasoningContext().to_dict()
        # 缺失 == None，空 == 0/[]；两者不能互相冒充。
        self.assertIsNone(payload["topology_summary"])
        self.assertIsNone(payload["task_context"])
        self.assertEqual(payload["knowledge_item_count"], 0)
        self.assertEqual(payload["observation_memory_ids"], [])
        self.assertEqual(payload["run_mode"], "live")

    def test_context_to_dict_skips_memory_items_without_an_id(self):
        payload = SearchReasoningContext(
            observation_memory=[{"memory_id": "m1"}, {"label": "no id"}],
            task_context={"task_id": "t1"},
            topology_summary={"place": "corridor", "frontier_count": 2},
            run_mode="offline_replay",
        ).to_dict()
        self.assertEqual(payload["observation_memory_ids"], ["m1"])
        self.assertEqual(payload["observation_memory_count"], 2)
        self.assertEqual(payload["topology_summary"]["frontier_count"], 2)
        self.assertEqual(payload["run_mode"], "offline_replay")


class FrozenConsumerContractTests(unittest.TestCase):
    """§8.1：新 directive 必须仍然能被冻结的导航消费者变成可执行候选。"""

    def _observation(self) -> LiveObservation:
        return LiveObservation(
            bundle_id="obs_1", timestamp=1.0, scene_objects=[],
            scene_graph={"nodes": [], "edges": []},
            target_match={"target_present": False},
            pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
            heading_sector=0, provenance={"source": "test"},
        )

    def _graph(self) -> ExplorationGraph:
        graph = ExplorationGraph(session_id="t")
        graph.add_observation(ObservationNode(
            node_id="node_a", timestamp=1.0, heading_sector=3, heading=90.0,
            objects=["water dispenser"], reachable_state="OBSERVED"))
        return graph

    def test_bare_target_directive_becomes_a_rotation_goal(self):
        directive = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.425, matched=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"heading_delta_deg": -3.58}}],
        ))
        goals = generate_live_exploration_candidates(
            observation=self._observation(), graph=self._graph(),
            directive=directive, current_yaw_deg=0.0,
            config=CandidateConfig(heading_sectors=12),
        )
        from_directive = [
            goal for goal in goals
            if goal.goal_type == GOAL_ROTATE_VIEW
            and goal.provenance.get("source") == "semantic_navigation_directive"
        ]
        self.assertTrue(from_directive)
        self.assertEqual(from_directive[0].relative_dyaw, -30.0)
        self.assertIn(
            directive.directive_id, from_directive[0].semantic_reason
        )

    ANCHOR_LABEL = "唯一锚点甲"

    def _anchored_directive(self):
        """一条**合法**的 inspect_anchor：锚点非空、方向来自真实几何。

        下面两条例共用这一个 fixture，第二条按 §1.5 的注错校准把它改成
        `anchor_label=None`，必须被识别为无效而不是照做一个目标出来。
        """
        return SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.425,
            matched=["obj_bin_001"], anchors=["obj_table_001"],
            nodes=[
                {"node_id": "obj_bin_001", "label_zh": "垃圾桶"},
                {"node_id": "obj_table_001", "label_zh": self.ANCHOR_LABEL,
                 "attributes": {"heading_delta_deg": 42.0}},
            ],
        ))

    def _inspect_goals(self, directive):
        return [
            goal for goal in generate_live_exploration_candidates(
                observation=self._observation(), graph=self._graph(),
                directive=directive, current_yaw_deg=0.0,
                config=CandidateConfig(heading_sectors=12),
            )
            if goal.goal_type == GOAL_INSPECT_ANCHOR
            and goal.provenance.get("source") == "semantic_navigation_directive"
        ]

    def test_legal_anchor_directive_does_not_vanish_in_the_frozen_consumer(self):
        directive = self._anchored_directive()
        self.assertEqual(directive.kind, SearchDirectiveKind.INSPECT_ANCHOR)
        goals = self._inspect_goals(directive)
        self.assertTrue(goals)
        # 锚点标签必须原样带过去：消费者就是拿它去查目标的。
        self.assertEqual(goals[0].semantic_anchor, self.ANCHOR_LABEL)
        self.assertIn(directive.directive_id, goals[0].semantic_reason)
        # 消费者会把转角**幅值**夹到 fallback_turn_deg（默认 30°），但不夹方向：
        # 42° 落在 +30°、扇区仍按未夹的 42° 归到第 1 格。证据方向是原样过去的，
        # 被改的只是执行器能转多少度。
        self.assertEqual(goals[0].relative_dyaw, 30.0)
        self.assertEqual(goals[0].heading_sector, 1)

    def test_illegal_inspect_directive_is_identified_not_silently_honoured(self):
        """§1.5 注错校准的落点：`inspect_anchor(anchor=None)` 必须被识别为无效。

        消费者对空锚点的处理是**一个候选都不生成**（`candidate_goal_generator`
        里 `kind_value in {"inspect_anchor"} and anchor_label` 那个守卫）。而
        "什么都不产出"本身不是解释，所以这条测试钉两件事：① 空锚点 directive
        不会变成一个合法目标；② 推理层根本不会再产出这种 directive —— 同一
        上下文里它给的是 `partial_without_anchor` 加 `no_anchor_candidate`，
        把"没有锚点可查"写进 uncertainties，而不是留一个静默的空结果。
        """
        illegal = replace(self._anchored_directive(), anchor_label=None)
        self.assertIsNone(illegal.anchor_label)
        self.assertEqual(self._inspect_goals(illegal), [])
        bare = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.425, matched=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶"}],
        ))
        self.assertNotEqual(bare.kind, SearchDirectiveKind.INSPECT_ANCHOR)
        self.assertIsNone(bare.anchor_label)
        self.assertEqual(
            bare.decision_trace["uncertainties"], ["no_anchor_candidate"]
        )

    # --- §1.5 的另一半：安全条件造成的拒绝 -------------------------------------
    #
    # 上面两条钉的是**地图条件**（锚点为空、锚点给不出方向）。§1.5 要求的是
    # "地图**或安全条件**"两种都要有可解释的拒绝结果，而安全这一半此前在
    # tests/ 下一个断言都没有 —— `FORWARD_NOT_SAFE` 检索零命中。
    #
    # 要钉的是这条路径：消费者在没有度量导航能力时会发一个"盲走一步换个视角"
    # 的兜底前进目标，但它必须先过 `_forward_fallback_block_reason` 这道闸。
    # 这段函数的返回值只被调用方当**布尔**用（`not forward_blocked_reason`），
    # 所以它算出来的那句原因在运行期是被丢掉的 —— 必须在这里钉住，否则
    # "为什么没动"在整条链路上没有任何一处留得下话。

    FORWARD_SOURCE = "fallback_forward"

    def _forward_fallbacks(self, *, directive=None, **kwargs):
        return [
            goal for goal in generate_live_exploration_candidates(
                observation=self._observation(), graph=self._graph(),
                directive=directive, current_yaw_deg=0.0,
                capabilities=RobotCapabilities(
                    supports_metric_navigation=False,
                    supports_relative_translation=True,
                ),
                config=CandidateConfig(heading_sectors=12),
                **kwargs,
            )
            if goal.provenance.get("source") == self.FORWARD_SOURCE
        ]

    def test_safety_condition_rejection_is_explainable_not_silent(self):
        directive = self._anchored_directive()

        # 对照臂：安全快照放行时，兜底前进必须真的发得出来。没有这一臂，
        # 下面"被拒时一个都没有"就可能只是这条路径压根没走到 —— 阴性断言
        # 必须先证明判据有分辨力。
        allowed = self._forward_fallbacks(
            directive=directive,
            local_safety=LocalSafetySnapshot(
                forward_safe=True, front_clearance_m=1.2, front_state=STATE_CLEAR),
        )
        self.assertEqual(len(allowed), 1)
        self.assertEqual(allowed[0].goal_type, GOAL_RELATIVE_MOVE)
        self.assertEqual(allowed[0].relative_dx, 0.20)

        # 前向不安全 ⇒ 一个目标都不生成。
        blocked_safety = LocalSafetySnapshot(
            forward_safe=False, front_clearance_m=0.18, front_state=STATE_OCCUPIED)
        self.assertEqual(
            self._forward_fallbacks(directive=directive, local_safety=blocked_safety),
            [],
        )
        # 而拒绝本身必须是可解释的：读得出是哪个条件、当时前向读数多少。
        self.assertEqual(
            _forward_fallback_block_reason(blocked_safety, None),
            "FORWARD_NOT_SAFE: front 0.18m state OCCUPIED",
        )

        # 相机被挡是另一条独立的理由，与前方几何无关，同样必须说出来。
        blocked_camera = PerceptionQuality(
            camera_blocked=True, reason="lens occluded by payload")
        self.assertEqual(
            self._forward_fallbacks(directive=directive,
                                    perception_quality=blocked_camera),
            [],
        )
        self.assertEqual(
            _forward_fallback_block_reason(None, blocked_camera),
            "CAMERA_BLOCKED: lens occluded by payload",
        )

        # 两条都不成立时才放行。
        self.assertEqual(
            _forward_fallback_block_reason(
                LocalSafetySnapshot(forward_safe=True, front_clearance_m=1.2,
                                    front_state=STATE_CLEAR),
                PerceptionQuality(camera_blocked=False),
            ),
            "",
        )

    def test_blocked_forward_fallback_reason_survives_into_the_run(self):
        """§1.5 的运行时那一半：那句原因必须真的留在链路上。

        上面那条钉的是 `_forward_fallback_block_reason` 的**返回值** ——
        之所以只能钉在那里，是因为运行期把它当布尔用完就丢了。这一条钉
        运行期：拒绝原因要出现在 `candidate_rejected_safety` 事件里，
        也就随之进了决策记录的 `hard_safety_rejections` 和 WebUI 快照。
        """
        blocked = LocalSafetySnapshot(
            forward_safe=False, front_clearance_m=0.18, front_state=STATE_OCCUPIED)

        def _reasons(snapshot):
            scene = scenario_no_target(empty_scenes=2)
            policy = load_exploration_policy()
            policy.budget = ExplorationBudget(max_motion_steps=0,
                                              max_planning_cycles=1)
            explorer = AutonomousExplorer(
                target="白色垃圾桶", observer=scene.observer(),
                matcher=scene.matcher(), verifier=scene.verifier(),
                backend=MockBackend(), graph=ExplorationGraph(session_id="t"),
                policy=policy, local_safety_provider=lambda obs: snapshot,
            )
            explorer.run()
            return [
                item.get("reason")
                for event in explorer.events
                if event.get("event") == "candidate_rejected_safety"
                for item in event.get("rejections") or []
            ]

        # 对照臂：前向放行时**不许**出现这条原因 —— 否则下面那个 assertIn
        # 可能只是"反正总会冒出一条拒绝"，证明不了是这道闸说的话。
        self.assertNotIn(
            "FORWARD_NOT_SAFE: front 0.18m state OCCUPIED",
            _reasons(LocalSafetySnapshot(
                forward_safe=True, front_clearance_m=1.2, front_state=STATE_CLEAR)),
        )

        # 前向不安全 ⇒ 兜底前进被拒，且原因原样留在事件流里（读数一起带上）。
        self.assertIn(
            "FORWARD_NOT_SAFE: front 0.18m state OCCUPIED",
            _reasons(blocked),
        )

    def test_live_adapter_turns_the_directive_into_a_semantic_scan_step(self):
        directive = SemanticNavigationReasoner().propose(_context(
            GraphMatchState.PARTIAL, 0.425, matched=["obj_bin_001"],
            nodes=[{"node_id": "obj_bin_001", "label_zh": "垃圾桶",
                    "attributes": {"heading_delta_deg": -3.58}}],
        ))
        plan = directive_to_step_plan(
            directive, scan_index=0, distance_m=0.0,
            step_config=SimpleNamespace(scan_turn_deg=30.0, scan_span=3,
                                        max_radius_m=0.0, forward_estimate_m=0.15),
        )
        # 落回 legacy 会带 phase != SEMANTIC_SEARCH；这里必须走语义选向。
        self.assertEqual(plan.phase, "SEMANTIC_SEARCH")
        self.assertEqual(plan.step, "r30")


if __name__ == "__main__":
    unittest.main()
