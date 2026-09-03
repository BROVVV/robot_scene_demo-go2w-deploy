"""计划书 §11.2：WebUI 语义拓扑投影的契约测试。

前端契约用 headless node 跑 search_map.js 暴露的纯函数
``window.TopologyLayout.objectOnly``；后端契约直接检查
``SemanticNavigationGraph`` 的投影同时满足同一份约束。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from app.spatial.semantic_navigation_graph import SemanticNavigationGraph

NODE = shutil.which("node")
JS_TEST = Path(__file__).with_name("test_webui_object_topology_contract.js")
SEARCH_MAP = Path(__file__).resolve().parents[1] / "app/manual_web_demo/static/search_map.js"


@pytest.mark.skipif(NODE is None, reason="node.js not available")
def test_frontend_projection_contract():
    result = subprocess.run(
        [NODE, str(JS_TEST)], capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, (
        f"node topology contract test failed:\n{result.stdout}\n{result.stderr}"
    )
    assert "ALL JS TOPOLOGY CONTRACT TESTS PASSED" in result.stdout


def test_render_never_prefers_the_whole_navigation_graph():
    text = SEARCH_MAP.read_text(encoding="utf-8")
    assert "var topology = graph && graph.object_topology ? graph.object_topology : null;" in text
    # 旧的"先整张导航图，空了才退回 object_topology"逻辑必须彻底消失。
    assert "topology = (graph && graph.object_topology) || null;" not in text
    assert "var topology = graph || null;" not in text


def _graph_with_place_frontier_and_objects() -> SemanticNavigationGraph:
    graph = SemanticNavigationGraph(relation_confirm_min_observations=1)
    for index, observation in enumerate(("bundle_1", "bundle_2"), start=1):
        graph.update_observation(
            observation_id=observation,
            heading_sector=index - 1,
            scene_objects=[
                {"id": f"frame_{index}_1", "label": "办公桌",
                 "map_xyz": [1.0, 0.0, 0.0], "confidence": 0.9},
                {"id": f"frame_{index}_2", "label": "白色垃圾桶",
                 "map_xyz": [1.3, 0.5, 0.0], "confidence": 0.9},
            ],
            scene_relations=[{"subject_id": f"frame_{index}_1",
                              "object_id": f"frame_{index}_2",
                              "relation": "near", "confidence": 0.8}],
            pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0}, timestamp=float(index),
        )
    return graph


def _graph_with_p1_f01_and_two_objects() -> SemanticNavigationGraph:
    """计划书 §11.2 的完整导航图：P1 + F01 + obj_001 + obj_002。

    观测只覆盖 sector 1、2，所以 sector 0 留下的 frontier 短标签正好是 F01。
    """
    graph = SemanticNavigationGraph(relation_confirm_min_observations=1)
    for index, sector in enumerate((1, 2), start=1):
        graph.update_observation(
            observation_id=f"bundle_{index}", heading_sector=sector,
            scene_objects=[
                {"id": f"frame_{index}_1", "label": "办公桌",
                 "map_xyz": [1.0, 0.0, 0.0], "confidence": 0.9},
                {"id": f"frame_{index}_2", "label": "白色垃圾桶",
                 "map_xyz": [1.3, 0.5, 0.0], "confidence": 0.9},
            ],
            scene_relations=[{"subject_id": f"frame_{index}_1",
                              "object_id": f"frame_{index}_2",
                              "relation": "near", "confidence": 0.8}],
            pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0}, timestamp=float(index),
        )
    return graph


def test_projection_keeps_only_obj_001_and_obj_002():
    graph = _graph_with_p1_f01_and_two_objects()
    full = graph.to_dict()
    internal_ids = {node["node_id"] for node in full["nodes"]}
    internal_labels = {str(node.get("label")) for node in full["nodes"]}
    # 内部导航图必须同时含 P1 和短标签 F01 的 frontier，规划器要用，不许删。
    assert "P1" in internal_ids
    assert "F01" in internal_labels
    assert {"obj_001", "obj_002"} <= internal_ids

    topology = full["object_topology"]
    node_ids = {node["node_id"] for node in topology["nodes"]}
    labels = {str(node.get("label")) for node in topology["nodes"]}
    assert node_ids == {"obj_001", "obj_002"}
    assert "P1" not in node_ids and "P1" not in labels
    assert "F01" not in node_ids and "F01" not in labels
    for edge in topology["edges"]:
        assert edge["from"] in node_ids and edge["to"] in node_ids
    relations = {edge["relation"] for edge in topology["edges"]}
    assert "OBSERVED_FROM" not in relations
    assert "FRONTIER_TO" not in relations


def test_backend_projection_excludes_place_and_frontier():
    graph = _graph_with_place_frontier_and_objects()
    full = graph.to_dict()
    node_types = {node["node_type"] for node in full["nodes"]}
    # 内部导航图仍然保留 Place（以及产生的 Frontier），规划器要用。
    assert "PLACE" in node_types

    topology = full["object_topology"]
    assert topology["schema_version"] == "semantic_object_topology_v1"
    object_ids = {node["node_id"] for node in topology["nodes"]}
    assert object_ids, "expected persistent object nodes"
    assert all(node["node_type"] == "OBJECT" for node in topology["nodes"])
    assert not any(node_id.startswith(("P", "F")) for node_id in object_ids)
    for edge in topology["edges"]:
        assert edge["from"] in object_ids and edge["to"] in object_ids
        assert edge["relation"] not in {"OBSERVED_FROM", "FRONTIER_TO", "MOVED_TO"}
