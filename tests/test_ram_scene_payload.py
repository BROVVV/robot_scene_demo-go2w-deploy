"""§17：Grounded-SAM + RAM++ 模式下，普通场景物体由检测结果提供，不再跑全场景 VLM。

这里验的是形状契约：`_ram_scene_payload` 交给语义观察的 payload，
必须能被 `LiveSemanticObserver` 原样吃下去（label / bbox_2d / mask 都不丢）。
"""

from __future__ import annotations

import unittest

from app.live_robot.semantic_observer import (
    SEMANTIC_STATUS_FRESH_QUICK,
    LiveSemanticObserver,
)


def _gsam_object(label: str, bbox, mask_area_ratio: float | None = None) -> dict:
    """`_gsam_to_dict` 的输出形状（run_autonomous_loop.py）。"""
    return {
        "label": label,
        "label_zh": label,
        "score": 0.8,
        "bbox_2d": list(bbox),
        "mask_rle": [[0, 4, 1], [10, 4, 1]],
        "mask_size": [720, 1280],
        "mask_area_ratio": mask_area_ratio,
    }


def _ram_scene_payload(image_path: str, objects: list[dict]) -> dict:
    """run_autonomous_loop._ram_scene_payload 的等价输出。"""
    frame_id = "bundle_137555"
    return {
        "scene_objects": objects,
        "scene_relations": [],
        "scene_summary_zh": (
            "场景物体：" + "、".join(
                str(item.get("label_zh") or item.get("label") or "object")
                for item in objects[:12]
            )
        ) if objects else "本帧检测未发现物体。",
        "image_path": image_path,
        "frame_id": frame_id,
        "source": "grounded_sam_ram_scene_objects",
        "semantic_status": SEMANTIC_STATUS_FRESH_QUICK,
        "semantic_quality": "grounded_sam_scene_objects",
        "semantic_source_frame_id": frame_id,
        "semantic_capture_timestamp": 1.0,
        "semantic_completed_timestamp": 1.0,
        "semantic_age_ms": 0.0,
    }


class RamScenePayloadTest(unittest.TestCase):
    def _observe(self, payload: dict):
        observer = LiveSemanticObserver(lambda _image, _profile: payload,
                                       now=lambda: 2.0)
        return observer.observe(
            target_profile=None,
            frame_or_bundle="/tmp/frame.jpg",
            robot_pose={"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        )

    def test_detections_become_scene_objects_with_geometry_and_masks(self) -> None:
        objects = [
            _gsam_object("office chair", [0.1, 0.2, 0.3, 0.9], 0.25),
            _gsam_object("backpack", [0.4, 0.3, 0.5, 0.6], 0.05),
        ]

        observation = self._observe(_ram_scene_payload("/tmp/frame.jpg", objects))

        labels = [item["label"] for item in observation.objects]
        self.assertEqual(labels, ["office chair", "backpack"])
        # §17 的意义就是这些普通物体照常进地图：bbox 与 SAM2 mask 都不能丢。
        first = observation.objects[0]
        self.assertEqual(first["bbox"], [0.1, 0.2, 0.3, 0.9])
        self.assertEqual(first["mask_rle"], [[0, 4, 1], [10, 4, 1]])
        self.assertEqual(first["mask_size"], (720, 1280))
        self.assertEqual(observation.semantic_status, SEMANTIC_STATUS_FRESH_QUICK)

    def test_scene_graph_is_built_from_detections(self) -> None:
        payload = _ram_scene_payload(
            "/tmp/frame.jpg", [_gsam_object("chair", [0.1, 0.1, 0.4, 0.6])]
        )

        observation = self._observe(payload)

        self.assertIsNotNone(observation.scene_graph)
        self.assertTrue(observation.scene_graph.nodes)

    def test_empty_detections_do_not_fake_a_scene(self) -> None:
        # §64：检测结果为空就是空场景 —— 那也是检测器的结论，不能再回去问 VLM
        # 一遍（full-scene VLM enumeration 必须 = 0），也绝不能凭空造物体。
        observation = self._observe(_ram_scene_payload("/tmp/frame.jpg", []))

        self.assertEqual(observation.objects, [])


if __name__ == "__main__":
    unittest.main()


# --------------------------------------------------------------------------- #
# §7.2 四段链路契约：GroundedSAM -> 观察 payload -> 三维定位 -> 持久实体
#
# 链路是 `DetectedObject` -> `_gsam_to_dict` 的 dict -> `ObjectSpatialObservation`
# -> `SemanticObjectEntry` -> 快照读的那几个键。整改在各段之间加了
# `label_zh`/`category`，这一段就是钉住"四段之间字段不丢"。
#
# `_gsam_to_dict` 住在 `scripts/go2w/run_autonomous_loop.py` 里，那份要 rclpy 才
# import 得动，所以这里按它的**输出形状**复刻（与文件上半部分同一套办法），
# 但只对真实产出的 `DetectedObject` 做断言 —— 复刻错了这里就会红。
# --------------------------------------------------------------------------- #
def _gsam_shape(obj) -> dict:
    """`run_autonomous_loop._gsam_to_dict` 的输出形状。"""
    return {
        "label": obj.label,
        "label_zh": obj.label_zh,
        "category": obj.category,
        "source_prompt_term": obj.source_prompt_term,
        "score": obj.score,
        "bbox_2d": list(obj.bbox_2d),
        "mask_rle": obj.mask_rle,
        "mask_size": list(obj.mask_size) if obj.mask_size else None,
        "mask_area_ratio": obj.mask_area_ratio,
    }


class SemanticLabelChainTest(unittest.TestCase):
    def test_detected_object_carries_a_chinese_name_and_a_category(self) -> None:
        from app.detectors.grounded_sam_subprocess import _to_detected_object

        obj = _to_detected_object(
            {"label": "speaker", "score": 0.8, "bbox_2d": [0.1, 0.2, 0.3, 0.4]}
        )
        self.assertEqual(obj.label, "speaker")      # 关联键/取证字段
        self.assertEqual(obj.label_zh, "音箱")       # 显示字段
        self.assertEqual(obj.category, "electronics")

    def test_ram_chinese_pair_beats_the_static_dictionary(self) -> None:
        """RAM++ 同一次调用给的中文优先 —— 它已经算好了，不能丢（§2.2）。"""
        from app.detectors.grounded_sam_subprocess import _to_detected_object

        obj = _to_detected_object(
            {"label": "bin", "score": 0.8, "bbox_2d": [0.1, 0.2, 0.3, 0.4]},
            runtime_zh="容器",
        )
        self.assertEqual(obj.label_zh, "容器")

    def test_gsam_shape_keeps_all_four_fields(self) -> None:
        from app.detectors.grounded_sam_subprocess import _to_detected_object

        payload = _gsam_shape(
            _to_detected_object(
                {"label": "barrel", "score": 0.7, "bbox_2d": [0.1, 0.2, 0.3, 0.4],
                 "mask_area_ratio": 0.12}
            )
        )
        self.assertEqual(payload["label"], "barrel")
        self.assertEqual(payload["label_zh"], "桶")
        self.assertEqual(payload["category"], "container")
        self.assertEqual(payload["source_prompt_term"], "barrel")
        self.assertEqual(payload["mask_area_ratio"], 0.12)

    def test_fields_survive_localization_and_persistence(self) -> None:
        """四段走通：检测 -> 定位 -> 持久实体 -> 快照读得到的键。"""
        from app.detectors.grounded_sam_subprocess import _to_detected_object
        from app.perception.depth_object_localizer import (
            ObjectSpatialObservation,
            object_labels,
        )
        from app.spatial.semantic_object_map import SemanticObjectMap

        detected = _gsam_shape(
            _to_detected_object(
                {"label": "speaker", "score": 0.8,
                 "bbox_2d": [0.1, 0.2, 0.3, 0.4]}
            )
        )
        # 第二段：localizer 的取值函数（真实实现，不是复刻）。
        label, label_zh = object_labels(detected)
        observation = ObjectSpatialObservation(
            object_id="semantic_obj_001",
            label=label,
            label_zh=label_zh,
            category=str(detected.get("category") or ""),
            map_xyz=(1.0, 2.0, 0.5),
            confidence=float(detected["score"]),
        )
        # 第三段：持久实体。
        semantic_map = SemanticObjectMap(confirm_min_observations=1)
        result = semantic_map.update_with_associations(
            [observation], now=1.0)
        entry = semantic_map.objects[result.created_ids[0]]
        # 第四段：`semantic_object_localized` 事件里 object 字段就是 entry.to_dict()。
        event_object = entry.to_dict()
        self.assertEqual(event_object["label_zh"], "音箱")
        self.assertEqual(event_object["category"], "electronics")
        self.assertEqual(event_object["confidence"], 0.8)
        self.assertEqual(event_object["observation_count"], 1)

        # 快照写端从这条事件里取的就是这四个键。
        from app.spatial.semantic_object_gate import normalize_snapshot_object

        record = normalize_snapshot_object(event_object)
        self.assertEqual(record["label_zh"], "音箱")
        self.assertEqual(record["raw_label"], "音箱")
        self.assertEqual(record["confidence"], 0.8)


if __name__ == "__main__":
    unittest.main()
