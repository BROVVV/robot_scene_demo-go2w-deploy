"""「搜绿色垃圾桶」确定性端到端（mock 后端，无需真机/ROS）。

场景：前两帧只有办公桌/办公椅/纸箱，最后一帧出现「绿色垃圾桶」（near 办公桌）。
验证：TARGET_FOUND，且 SemanticNavigationGraph 的 object_topology 里：
  * 有「绿色垃圾桶」持久节点（obj_xxx）；
  * 有 near 关系边（STRUCTURAL、端点都是 obj_xxx）；
  * 重复识别不重复建边（第三帧 near 又出现一次仍一条边）。
"""

from __future__ import annotations

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import scenario_green_bin
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph


def test_mock_search_green_bin_finds_target_and_builds_topology():
    scene = scenario_green_bin()
    explorer = AutonomousExplorer(
        target="绿色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=MockBackend(),
        policy=load_exploration_policy(),
        graph=ExplorationGraph(session_id="e2e_green_bin"),
        negative_target_key="绿色垃圾桶",
        finish_on_visual_confirmation=True,
    )
    result = explorer.run()
    assert result.result == "TARGET_FOUND", f"应找到绿色垃圾桶: {result.result}"

    graph = explorer.semantic_graph
    ot = graph.object_topology_snapshot()
    nodes = ot["nodes"]
    labels = {n["label"] for n in nodes}
    assert "绿色垃圾桶" in labels, f"拓扑应有绿色垃圾桶, got {labels}"
    assert "办公桌" in labels

    near = [e for e in ot["edges"] if e["relation"] == "near"]
    assert len(near) == 1, f"near 应合并为 1 条边: {ot['edges']}"
    edge = near[0]
    assert edge["from"].startswith("obj_") and edge["to"].startswith("obj_")
    assert edge["relation_scope"] == "STRUCTURAL"
    # 节点都必须是持久 obj id
    assert all(n["node_id"].startswith("obj_") for n in nodes)
    # 端点必须在节点里
    node_ids = {n["node_id"] for n in nodes}
    assert edge["from"] in node_ids and edge["to"] in node_ids
