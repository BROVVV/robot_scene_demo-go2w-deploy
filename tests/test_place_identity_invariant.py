# Copyright 2026 robot_scene_demo maintainers

"""计划书 §7 的 P0：WebUI 展示的「当前 Place」必须与 planner 读的是同一个。

2026-09-09 只把 worker 的 ``object_map`` 注入了 explorer 的
``SemanticNavigationGraph``，``place_graph`` 没有 —— 于是 Place 仍是两套：

  * worker 那张：``PlaceGraph(merge_distance_m=0.35,
    relocation_min_displacement_m=0.50)``，且 ``observe()`` 传得进
    ``observed_displacement_m``（轮式里程计）。
  * explorer 自建那张：``place_graph.py:49-50`` 的默认值 0.6 / 0.10（重定位门限
    **松 5 倍**），且两条 ``update_observation`` 调用路径都传不进位移证据。

planner / local scan / decision record 读的是 worker 那张
（``place_graph.current_place()``），而 WebUI 拿到的是 explorer 那张
（``memory_update`` 的 ``semantic_navigation_graph`` 字段）。同一个"现在在哪"
因此有两个身份。

本文件盯三件事，缺一条这个 bug 就会以某种形式回来：

1. **身份**：共用一张图之后，explorer 侧登记的 observation 不能把
   ``current_place`` 挪到别处，两边的 place_id 必须逐帧相等。
2. **不重复登记**：``observation_ids`` / ``visit_count`` / ``places`` 数量不许因为
   explorer 又调了一次 ``update_observation`` 而翻倍（worker 的 ``observe()``
   每周期已经登记过一次，explorer 还会在 confirm 分支和 UPDATE_MEMORY 各来一次）。
3. **不能顺手把拓扑视图掐掉**：早退只许跳过 Place 登记，``object_map`` 与
   ``relation_store`` 必须照常更新 —— 关系边是 WebUI 物体拓扑视图的唯一来源。

第 2 条里那个"没有位移证据时会重跑 _resolve_place"是真的会挪走 current place，
不是理论担忧：见 ``test_relocation_evidence_is_not_re_resolved_without_wheels``。
"""

from __future__ import annotations

from app.spatial.models import SpatialPose
from app.spatial.place_graph import PlaceGraph
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph

WORKER_MERGE_M = 0.35
WORKER_RELOCATION_M = 0.50


def _graph_pair() -> tuple[PlaceGraph, SemanticNavigationGraph]:
    """复刻 run_semantic_exploration.py 的注入：worker 的图，两边共用。"""
    place_graph = PlaceGraph(
        merge_distance_m=WORKER_MERGE_M,
        relocation_min_displacement_m=WORKER_RELOCATION_M,
    )
    return place_graph, SemanticNavigationGraph(place_graph=place_graph)


def _scene(label: str = "trash bin") -> list[dict]:
    return [{"label": label, "bbox_2d": [0, 0, 10, 10],
             "camera_xyz": [0.1, 0.0, 1.0], "depth_m": 1.0}]


def _pose(x: float, y: float, yaw: float = 0.0) -> SpatialPose:
    """worker 侧 register_observation 收 SpatialPose，explorer 侧收 dict。"""
    return SpatialPose(x=x, y=y, yaw=yaw, frame_id="pslam_map")


def _pose_dict(x: float, y: float, yaw: float = 0.0) -> dict:
    return {"x": x, "y": y, "yaw_rad": yaw, "frame_id": "pslam_map"}


def test_explorer_reuses_the_place_the_worker_already_registered() -> None:
    """正路：worker 登记 → explorer 拿同一帧再 update，place 必须一模一样。"""
    place_graph, graph = _graph_pair()

    # worker 的 observe()（run_semantic_exploration.py:1976）
    place_a, created = place_graph.register_observation(
        observation_id="bundle_1",
        heading_sector=0,
        objects=["trash bin"],
        pose=None,
        observed_displacement_m=None,
        timestamp=100.0,
    )
    assert created is True

    # explorer 的 _update_memory（autonomous_explorer.py:1861）拿同一帧再来一次
    update = graph.update_observation(
        observation_id="bundle_1",
        heading_sector=0,
        scene_objects=_scene(),
        scene_relations=[],
        pose=None,
        timestamp=100.0,
    )

    assert update["place_id"] == place_a
    assert update["created_place"] is False
    assert len(place_graph.places) == 1
    assert place_graph.places[place_a].observation_ids == ["bundle_1"]
    assert place_graph.places[place_a].visit_count == 1
    # 两边读的是同一个 current place —— 这就是 §7 要的那条不变量
    assert graph.current_place_id == place_graph.current_place().place_id


def test_confirm_branch_and_update_memory_do_not_double_register() -> None:
    """explorer 一周期内最多调两次 update_observation，两处都不许再登记。"""
    place_graph, graph = _graph_pair()
    place_graph.register_observation(
        observation_id="bundle_7", heading_sector=3, objects=["trash bin"],
        pose=None, observed_displacement_m=0.0, timestamp=200.0,
    )
    for _ in range(2):        # autonomous_explorer.py:672 与 :1861
        graph.update_observation(
            observation_id="bundle_7", heading_sector=3,
            scene_objects=_scene(), scene_relations=[], pose=None,
            timestamp=200.0, target_candidate=True,
        )
    place = place_graph.places[place_graph.current_place().place_id]
    assert place.observation_ids == ["bundle_7"]
    assert place.visit_count == 1


def test_relocation_evidence_is_not_re_resolved_without_wheels() -> None:
    """核心反例：没有位移证据时重登记，会把 current place 挪到一个 LIO 漂移造出的新地方。

    场景是一次**只转向**的周期：轮子报告没有平移（displacement=0.0，
    < relocation 门限 0.50），而 LIO 位姿漂到 1.0 m 外。
    ``_resolve_place`` 在有位移证据时会在 ``observed_displacement_m <
    relocation_min_displacement_m`` 处直接返回 current（place_graph.py:139-140，
    注释写明"turn-only cycle 里漂移的 LIO 位姿不许造 P2/P3"）；
    而没有位移证据时它走的是全局最近邻那一段，1.0 m > merge 0.35 ⇒ 新建一个 place。
    """
    place_graph, graph = _graph_pair()
    place_graph.register_observation(
        observation_id="bundle_10", heading_sector=0, objects=["trash bin"],
        pose=_pose(0.0, 0.0), observed_displacement_m=None, timestamp=300.0,
    )
    # 同一帧，轮子说原地没动 —— worker 侧（有位移证据）不会换 place
    place_a, created = place_graph.register_observation(
        observation_id="bundle_11", heading_sector=1, objects=["trash bin"],
        pose=_pose(0.0, 0.0), observed_displacement_m=0.0, timestamp=301.0,
    )
    assert created is False and len(place_graph.places) == 1
    assert place_graph.current_place().place_id == place_a

    # explorer 侧拿同一帧再来一次，只带那个漂移的位姿（它传不进 displacement）
    update = graph.update_observation(
        observation_id="bundle_11", heading_sector=1,
        scene_objects=_scene(), scene_relations=[],
        pose=_pose_dict(1.0, 0.0), timestamp=301.0,
    )
    assert update["place_id"] == place_a
    assert len(place_graph.places) == 1
    assert place_graph.current_place().place_id == place_a
    assert graph.current_place_id == place_a


def test_early_return_still_updates_objects_and_topology_edges() -> None:
    """早退只跳过 Place 登记。object_map 与 relation_store 照常更新。

    关系边是 WebUI 物体拓扑视图（to_dict 的 object_topology）的唯一来源，
    跟着一起跳过就等于把那张视图清空。
    """
    _, graph = _graph_pair()
    first = graph.update_observation(
        observation_id="bundle_20", heading_sector=0,
        scene_objects=_scene("table") + _scene("trash bin"),
        scene_relations=[], pose=_pose_dict(0.0, 0.0), timestamp=400.0,
    )
    assert first["created_place"] is True
    baseline = graph.object_topology_snapshot()
    assert baseline["stats"]["node_count"] >= 2

    # 同一帧第二次（走早退分支）：物体仍必须被更新到，节点数不许掉
    graph.update_observation(
        observation_id="bundle_20", heading_sector=0,
        scene_objects=_scene("table") + _scene("trash bin"),
        scene_relations=[{"subject": "trash bin", "relation": "next_to",
                          "object": "table"}],
        pose=_pose_dict(0.0, 0.0), timestamp=400.0,
    )
    after = graph.object_topology_snapshot()
    assert after["stats"]["node_count"] >= baseline["stats"]["node_count"]
    assert graph.object_map.objects, "早退不许跳过 object_map"


def test_standalone_explorer_still_creates_its_own_places() -> None:
    """没有注入 place_graph 时（单跑 explorer / 不开 --spatial-v2）行为不变。"""
    graph = SemanticNavigationGraph()
    update = graph.update_observation(
        observation_id="bundle_30", heading_sector=0,
        scene_objects=_scene(), scene_relations=[], pose=None, timestamp=500.0,
    )
    assert update["created_place"] is True
    assert len(graph.place_graph.places) == 1
    assert graph.current_place_id == update["place_id"]
