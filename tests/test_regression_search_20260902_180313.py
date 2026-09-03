"""搜索 search_20260902_180313_f3ab3bc5 的回归：目标确认落点与快照时机。

那次会话 cycle 8 复核确认了一个真实存在的白色网格垃圾桶（bundle_343131 图像
可证，VLM 复核语「图2中是白色带网格的垃圾桶……位于玻璃门旁」），但归档里：

- `target_found.object_id` 是 `obj_005`「纸箱」—— STALE、4.17 m、last_seen 比
  确认时刻早 770 s，与被确认的垃圾桶无关。旧的 `_target_object_id` 取的是
  object_map 里第一个 label 落在当前帧标签集合内的对象，「纸箱」恰好在那一帧。
- `webui_state.json` 的 semantic_graph 35 个对象、object_topology 35 节点里
  `is_target_confirmed` 全 false：唯一带语义图快照的 memory_update 事件在
  `mark_target_confirmed` 之前发出。
"""

from __future__ import annotations

import unittest

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import (
    MockObservationScene,
    MockSceneStep,
)
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.spatial.semantic_object_map import SemanticObjectEntry


def _explorer(target: str, scene: MockObservationScene) -> AutonomousExplorer:
    return AutonomousExplorer(
        target=target,
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=MockBackend(),
        policy=load_exploration_policy(),
        graph=ExplorationGraph(session_id="test"),
        negative_target_key=target,
    )


def _scene(objects: list[str]) -> MockObservationScene:
    return MockObservationScene(scenes=[
        MockSceneStep(objects=objects, target_present=True, target_score=0.95),
    ])


def _put(explorer: AutonomousExplorer, object_id: str, label: str) -> None:
    explorer.semantic_graph.object_map.objects[object_id] = SemanticObjectEntry(
        object_id=object_id, label=label, confidence=0.9, observation_count=1,
    )


class TestTargetConfirmationLandsOnTarget(unittest.TestCase):
    def test_unrelated_frame_label_is_not_stamped(self) -> None:
        """帧内标签命中不等于目标：object_map 里没有目标时不许标任何对象。"""
        explorer = _explorer("白色垃圾桶", _scene(["纸箱"]))
        _put(explorer, "obj_005", "纸箱")
        _put(explorer, "obj_024", "绿色垃圾桶")
        self.assertIsNone(explorer._target_object_id())

    def test_target_label_is_stamped_even_when_listed_later(self) -> None:
        """目标对象排在无关对象之后也要选中它，而不是插入顺序第一个。"""
        explorer = _explorer("白色垃圾桶", _scene(["纸箱"]))
        _put(explorer, "obj_005", "纸箱")
        _put(explorer, "obj_031", "白色垃圾桶")
        self.assertEqual(explorer._target_object_id(), "obj_031")

    def test_scene_label_variant_of_the_same_object_is_stamped(self) -> None:
        """场景 VLM 同一个桶写成「绿色垃圾桶」或「浅绿色垃圾桶」：都要命中。

        历史 search_worker.log 里同一现场的措辞在「蓝色和浅绿色的垃圾桶」「蓝色
        塑料筐和浅绿色塑料筐」之间来回变，纯相等匹配会让确认只落在 Place 上。
        """
        explorer = _explorer("绿色垃圾桶", _scene(["浅绿色垃圾桶"]))
        _put(explorer, "obj_005", "纸箱")
        _put(explorer, "obj_024", "浅绿色垃圾桶")
        self.assertEqual(explorer._target_object_id(), "obj_024")

    def test_exact_label_wins_over_a_containment_variant(self) -> None:
        """相等优先于包含：两个都在时不许落到前缀变体上。"""
        explorer = _explorer("绿色垃圾桶", _scene(["绿色垃圾桶"]))
        _put(explorer, "obj_024", "浅绿色垃圾桶")
        _put(explorer, "obj_031", "绿色垃圾桶")
        self.assertEqual(explorer._target_object_id(), "obj_031")

    def test_a_different_colour_bin_is_still_rejected(self) -> None:
        """放宽不等于放弃颜色：找白色垃圾桶时不许把确认打到绿色垃圾桶上。"""
        explorer = _explorer("白色垃圾桶", _scene(["绿色垃圾桶"]))
        _put(explorer, "obj_024", "绿色垃圾桶")
        _put(explorer, "obj_005", "纸箱")
        self.assertIsNone(explorer._target_object_id())

    def test_confirmed_flag_is_inside_the_webui_snapshot(self) -> None:
        """带语义图的 memory_update 必须已经含确认标记，否则 WebUI 永远看不到。"""
        explorer = _explorer("蓝色垃圾桶", _scene(["蓝色垃圾桶"]))
        result = explorer.run()
        self.assertEqual(result.result, "TARGET_FOUND")
        snapshots = [
            event["semantic_navigation_graph"] for event in explorer.events
            if event.get("event") == "memory_update"
            and event.get("semantic_navigation_graph")
        ]
        self.assertTrue(snapshots, "确认路径没有发出带语义图的 memory_update")
        places = snapshots[-1].get("places") or []
        self.assertTrue(
            any(place.get("target_confirmed") for place in places),
            f"快照里没有已确认的 Place: {[p.get('place_id') for p in places]}",
        )


    def test_target_found_event_carries_the_target_object_id(self) -> None:
        """§12.6 第 8 条上半句：`target_found.object_id` 必须指向目标对象。

        归档里那一条写的是 `obj_005`「纸箱」（STALE、4.17 m、last_seen 早 770 s）。
        事件里的 object_id 和 mark 用的是同一个 `_target_object_id()`。
        """
        explorer = _explorer("蓝色垃圾桶", _scene(["蓝色垃圾桶"]))
        self.assertEqual(explorer.run().result, "TARGET_FOUND")
        found = [e for e in explorer.events if e.get("event") == "target_found"]
        self.assertEqual(len(found), 1)
        object_id = found[0].get("object_id")
        self.assertTrue(object_id, "target_found 没带 object_id")
        entry = explorer.semantic_graph.object_map.objects[object_id]
        self.assertEqual(entry.label, "蓝色垃圾桶")

    def test_object_topology_in_the_snapshot_marks_the_target(self) -> None:
        """§12.6 第 8 条下半句：WebUI 那份 object_topology 要有 is_target_confirmed。

        `object_topology` 是 `to_dict()` 里现算的投影（`object_relation_store`
        读 `provenance["target_confirmed"]`），所以 mark 的时机决定它是 true 还是
        false —— 180313 归档 35 个对象节点全 false 就是这么来的。
        """
        explorer = _explorer("蓝色垃圾桶", _scene(["蓝色垃圾桶"]))
        self.assertEqual(explorer.run().result, "TARGET_FOUND")
        snapshots = [
            event["semantic_navigation_graph"] for event in explorer.events
            if event.get("event") == "memory_update"
            and event.get("semantic_navigation_graph")
        ]
        self.assertTrue(snapshots, "确认路径没有发出带语义图的 memory_update")
        nodes = (snapshots[-1].get("object_topology") or {}).get("nodes") or []
        confirmed = [node for node in nodes if node.get("is_target_confirmed")]
        self.assertTrue(
            confirmed,
            f"object_topology 里没有已确认对象: {[n.get('label') for n in nodes]}",
        )
        self.assertEqual(confirmed[0].get("label"), "蓝色垃圾桶")


if __name__ == "__main__":
    unittest.main()
