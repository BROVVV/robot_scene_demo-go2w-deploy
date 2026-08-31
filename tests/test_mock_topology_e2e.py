"""Phase 37 mock E2E：确定性「语义拓扑」mock 场景两帧后，
persistent nodes = 3、persistent relation edges = 2、near 被证据融合。"""

from __future__ import annotations

from app.live_robot.mock_observation_scene import scenario_semantic_topology
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph


def test_mock_semantic_topology_two_frames():
    scene = scenario_semantic_topology()
    observer = scene.observer()
    graph = SemanticNavigationGraph(relation_confirm_min_observations=2)

    obs1 = observer()
    graph.update_observation(
        observation_id=obs1.bundle_id,
        heading_sector=0,
        scene_objects=obs1.scene_objects,
        scene_relations=obs1.scene_relations,
        pose=obs1.pose,
        timestamp=1.0,
    )
    topo1 = graph.object_topology_snapshot()
    assert len(topo1["nodes"]) == 3
    assert len(topo1["edges"]) == 2

    obs2 = observer()
    graph.update_observation(
        observation_id=obs2.bundle_id,
        heading_sector=1,
        scene_objects=obs2.scene_objects,
        scene_relations=obs2.scene_relations,
        pose=obs2.pose,
        timestamp=2.0,
    )
    topo2 = graph.object_topology_snapshot()
    assert len(topo2["nodes"]) == 3, "第二帧 frame id 全变，persistent 节点必须仍是 3"
    assert len(topo2["edges"]) == 2, "关系仍是 2 条（不重复建边）"
    near = [e for e in topo2["edges"] if e["relation"] == "near"]
    assert len(near) == 1
    assert near[0]["observation_count"] == 2
    assert near[0]["status"] == "CONFIRMED"
    left = [e for e in topo2["edges"] if e["relation"] == "left_of"]
    assert len(left) == 1
    assert left[0]["relation_scope"] == "VIEW_RELATIVE"
    assert left[0]["directed"] is True


def test_mock_scene_uses_frame_object_ids():
    """场景对象必须带 stable frame id（identity 链路的输入）。"""
    scene = scenario_semantic_topology()
    obs = scene.observer()()
    ids = [obj.get("id") for obj in obs.scene_objects]
    assert all(ids), "每个 mock object 都要有 frame id"
    assert len(set(ids)) == 3
    rel_ids = {
        r.get("subject_id") for r in obs.scene_relations
    } | {r.get("object_id") for r in obs.scene_relations}
    assert rel_ids <= set(ids), "relations 端点必须能被 frame id 解析"
