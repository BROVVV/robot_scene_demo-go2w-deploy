"""计划书 §17.10：frontier node_id 跨 Place 全局唯一（UI 短标签 Fxx 仅显示）。"""

from __future__ import annotations

from app.spatial.models import SpatialPose
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph


def test_frontier_ids_unique_across_places():
    graph = SemanticNavigationGraph(heading_sectors=12)
    # P1：只覆盖 sector 1 -> sector 0 生成 frontier:P1:00
    graph.update_observation(
        observation_id="obs_1",
        heading_sector=1,
        scene_objects=[],
        scene_relations=[],
        pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
        timestamp=1.0,
    )
    # P2：位移 1.5m 生成新 Place；也只覆盖 sector 1 -> sector 0 生成 frontier:P2:00
    graph.update_observation(
        observation_id="obs_2",
        heading_sector=1,
        scene_objects=[],
        scene_relations=[],
        pose={"x": 1.5, "y": 0.0, "yaw_rad": 0.0},
        timestamp=2.0,
    )
    assert len(graph.place_graph.places) == 2
    frontiers = list(graph.frontiers.values())
    # 每个 Place 对其未覆盖的每个 sector 生成一个 frontier：P1/P2 各 11 个。
    assert len(frontiers) == 22
    ids = {frontier["frontier_id"] for frontier in frontiers}
    # 关键断言：同一 bearing（sector 0）跨 Place 的 node_id 必须不同。
    f01 = [f for f in frontiers if f["label"] == "F01"]
    assert len(f01) == 2
    assert f01[0]["frontier_id"] != f01[1]["frontier_id"]
    assert {f["frontier_id"] for f in f01} == {"frontier:P1:00", "frontier:P2:00"}
    # UI 短标签按 bearing 编号（F01..F12），node_id 才是唯一键。
    nodes = graph.to_dict()["nodes"]
    frontier_nodes = [node for node in nodes if node["node_type"] == "FRONTIER"]
    assert len(frontier_nodes) == 22
    assert len({node["node_id"] for node in frontier_nodes}) == 22
    assert all(node["node_id"] != node["label"] for node in frontier_nodes)


def test_place_frontier_ids_list_uses_unique_ids():
    graph = SemanticNavigationGraph(heading_sectors=12)
    graph.update_observation(
        observation_id="obs_1",
        heading_sector=3,
        scene_objects=[],
        scene_relations=[],
        pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
        timestamp=1.0,
    )
    place = graph.place_graph.places["P1"]
    # 除 sector 3 外其余 11 个 sector 都是 frontier。
    assert len(place.frontier_ids) == 11
    assert all(str(fid).startswith("frontier:P1:") for fid in place.frontier_ids)
