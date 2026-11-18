"""WebUI 三维物体框的显示门（计划书 §7.3 的表逐行落成用例）。

被测的是 ``app/spatial/semantic_object_gate.py`` —— 判据刻意住在那里而不是快照
写端：写端要 rclpy 才起得来，判据必须能被普通 pytest 直接跑。

改这些用例之前请先读计划书 §5.1/§5.2：门限不是随手定的数，`CONFIRMED` 要
`observation_count >= 2`、`TENTATIVE` 要更高的分数，是因为"见过一次"和
"多帧反复见过"的可信度不一样；两档门限的差（0.45 vs 0.65）就是那个差。
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.spatial.semantic_object_gate import (
    DEFAULT_CONFIRMED_MIN_CONFIDENCE,
    DEFAULT_TENTATIVE_MIN_CONFIDENCE,
    audit_rows,
    display_decision,
    filter_persistent_observations,
    load_object_records,
    normalize_snapshot_object,
    select_display_objects,
    valid_box_size,
)


def record(**overrides) -> dict:
    """一条已经过 normalize 的记录；覆盖字段用来摆出各个用例。"""
    base = {
        "object_id": "obj_001",
        "raw_label": "chair",
        "label_zh": "办公椅",
        "category": "furniture",
        "confidence": 0.60,
        "confidence_max": 0.60,
        "observation_count": 3,
        "status": "CONFIRMED",
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "size": None,
        "target_verified": False,
    }
    base.update(overrides)
    return base


class DisplayThresholdTest(unittest.TestCase):
    """§7.3 表的前四行。"""

    def test_confirmed_with_enough_observations_and_score(self) -> None:
        self.assertEqual(
            display_decision(record()), (True, "DISPLAY")
        )

    def test_confirmed_but_below_the_confidence_floor(self) -> None:
        """0.44 < 0.45：确认过也不画。这就是 speaker=0.395/5 次的情形。"""
        shown, reason = display_decision(record(confidence=0.44))
        self.assertFalse(shown)
        self.assertEqual(reason, "below_display_threshold")

    def test_confirmed_but_only_seen_once(self) -> None:
        """count=1 不满足 A 条，落到 TENTATIVE 档又只有 0.60 < 0.65。"""
        shown, reason = display_decision(
            record(observation_count=1, confidence=0.60)
        )
        self.assertFalse(shown)
        self.assertEqual(reason, "below_display_threshold")

    def test_tentative_with_high_score_is_shown(self) -> None:
        shown, reason = display_decision(
            record(status="TENTATIVE", observation_count=1, confidence=0.70)
        )
        self.assertTrue(shown)
        self.assertEqual(reason, "DISPLAY")

    def test_tentative_below_its_higher_floor(self) -> None:
        shown, _ = display_decision(
            record(status="TENTATIVE", observation_count=1, confidence=0.60)
        )
        self.assertFalse(shown)

    def test_thresholds_are_inclusive_at_the_boundary(self) -> None:
        """门限是 >=，边界值必须显示 —— 否则现场会看到"刚好 0.45 却不出来"。"""
        self.assertTrue(
            display_decision(record(confidence=DEFAULT_CONFIRMED_MIN_CONFIDENCE))[0]
        )
        self.assertTrue(
            display_decision(
                record(status="TENTATIVE", observation_count=1,
                       confidence=DEFAULT_TENTATIVE_MIN_CONFIDENCE)
            )[0]
        )

    def test_observation_count_alone_is_not_enough(self) -> None:
        """§10 明令：只看 observation_count 就无条件显示是禁止改法。

        见过 20 次、每次 0.2 分 —— 平均下来就是不可信，不能因为"重复出现"上屏。
        """
        shown, _ = display_decision(
            record(observation_count=20, confidence=0.20)
        )
        self.assertFalse(shown)


class InvalidValueTest(unittest.TestCase):
    """§7.3：缺失、NaN、Inf、负数置信度一律过滤。"""

    def test_non_finite_and_out_of_range_scores(self) -> None:
        for value in (float("nan"), float("inf"), float("-inf"), -0.2, 1.5, None, "x"):
            shown, reason = display_decision(record(confidence=value))
            self.assertFalse(shown, value)
            self.assertEqual(reason, "invalid_score", value)

    def test_invalid_position_cannot_be_drawn(self) -> None:
        for overrides in (
            {"x": float("nan")},
            {"y": float("inf")},
            {"z": None},
            {"x": "左"},
            {"y": None, "z": None},
        ):
            payload = record(**overrides)
            payload.pop("z", None) if overrides.get("z", 0) is None else None
            shown, reason = display_decision(payload)
            self.assertFalse(shown, overrides)
            self.assertEqual(reason, "invalid_position", overrides)

    def test_position_is_checked_before_the_target_exception(self) -> None:
        """复核过的目标也画不到没有坐标的地方。"""
        shown, reason = display_decision(
            record(x=float("nan"), target_verified=True)
        )
        self.assertFalse(shown)
        self.assertEqual(reason, "invalid_position")


class NonObjectLabelTest(unittest.TestCase):
    """§7.3：purple/floor/corner 即使高分也过滤。"""

    def test_high_score_does_not_rescue_a_colour_or_a_region(self) -> None:
        for raw, zh in (
            ("purple", "紫色"),
            ("floor", "地面"),
            ("corner", "角落"),
            ("sparkle", "闪光"),
            ("wall", "墙"),
        ):
            shown, reason = display_decision(
                record(raw_label=raw, label_zh=zh, confidence=0.99)
            )
            self.assertFalse(shown, raw)
            self.assertEqual(reason, "non_object_label", raw)

    def test_chinese_only_label_is_caught_too(self) -> None:
        """R7 路线上游就是中文，只判英文拦不住。"""
        shown, reason = display_decision(record(raw_label="地面", label_zh="地面"))
        self.assertFalse(shown)
        self.assertEqual(reason, "non_object_label")


class HistoricalCompatibilityTest(unittest.TestCase):
    """§7.3：历史事件只有英文 `speaker` —— 中文化后再判定。"""

    def test_english_only_event_gets_a_chinese_display_name(self) -> None:
        item = normalize_snapshot_object(
            {"object_id": "obj_009", "label": "speaker", "map_xyz": [1.0, 2.0, 0.5],
             "confidence": 0.395, "observation_count": 5}
        )
        self.assertEqual(item["label_zh"], "音箱")
        self.assertEqual(item["raw_label"], "speaker")
        self.assertEqual(item["confidence_max"], 0.395)
        # count=5 ⇒ 推导成 CONFIRMED，但 0.395 过不了 0.45。
        self.assertEqual(item["status"], "CONFIRMED")
        self.assertFalse(display_decision(item)[0])

    def test_unknown_english_uses_chinese_fallback(self) -> None:
        item = normalize_snapshot_object(
            {"object_id": "obj_010", "label": "gizmo", "map_xyz": [1.0, 2.0, 0.5],
             "confidence": 0.9, "observation_count": 3}
        )
        self.assertEqual(item["label_zh"], "未分类物体")
        self.assertEqual(item["raw_label"], "gizmo")

    def test_status_derived_from_observation_count(self) -> None:
        self.assertEqual(
            normalize_snapshot_object({"label": "chair", "observation_count": 1})[
                "status"
            ],
            "TENTATIVE",
        )
        # 门槛 2026-09-18 由 2 提到 3，边界跟着挪一格。
        self.assertEqual(
            normalize_snapshot_object({"label": "chair", "observation_count": 2})[
                "status"
            ],
            "TENTATIVE",
        )
        self.assertEqual(
            normalize_snapshot_object({"label": "chair", "observation_count": 3})[
                "status"
            ],
            "CONFIRMED",
        )

    def test_existing_status_and_label_zh_win(self) -> None:
        """新事件带了的字段不许被"兼容推导"覆盖掉。"""
        item = normalize_snapshot_object(
            {"label": "bin", "label_zh": "容器", "status": "TENTATIVE",
             "observation_count": 9, "confidence": 0.8}
        )
        self.assertEqual(item["label_zh"], "容器")
        self.assertEqual(item["status"], "TENTATIVE")

    def test_english_display_label_is_translated(self) -> None:
        """英文显示值进入快照层时仍统一翻译成中文。"""
        item = normalize_snapshot_object(
            {"label": "barrel", "label_zh": "barrel", "confidence": 0.8,
             "observation_count": 2}
        )
        self.assertEqual(item["label_zh"], "桶")

    def test_map_xyz_is_accepted_as_the_position(self) -> None:
        item = normalize_snapshot_object(
            {"label": "chair", "map_xyz": [3.5, -1.25, 0.4], "confidence": 0.9,
             "observation_count": 2, "box_size": [0.4, 0.45, 0.9]}
        )
        self.assertEqual((item["x"], item["y"], item["z"]), (3.5, -1.25, 0.4))
        self.assertEqual(item["size"], [0.4, 0.45, 0.9])


class BoxSizeTest(unittest.TestCase):
    """§7.3：合法框尺寸画 AABB；非法/零/负尺寸不画伪框。"""

    def test_valid_sizes(self) -> None:
        self.assertEqual(valid_box_size([0.3, 0.2, 0.4]), (0.3, 0.2, 0.4))
        self.assertEqual(valid_box_size(("1", "2", "3")), (1.0, 2.0, 3.0))

    def test_invalid_sizes(self) -> None:
        for size in (
            None,
            [0.3, 0.2],
            [0.0, 0.2, 0.4],
            [-0.3, 0.2, 0.4],
            [0.3, float("nan"), 0.4],
            [0.3, float("inf"), 0.4],
            [0.3, 0.2, 40.0],
            "big",
        ):
            self.assertIsNone(valid_box_size(size), size)

    def test_invalid_size_degrades_to_a_dot_not_a_drop(self) -> None:
        """尺寸坏了是几何信息缺失，不是"它不存在" —— 该显示，只是画成点。"""
        visible, _stats = select_display_objects(
            [record(size=[0.0, 0.0, 0.0]), record(size=[0.4, 0.4, 0.9])]
        )
        self.assertEqual(len(visible), 2)
        self.assertIsNone(visible[0]["size"])
        self.assertFalse(visible[0]["box_valid"])
        self.assertTrue(visible[1]["box_valid"])

    def test_oversized_box_is_not_drawn(self) -> None:
        """40 m 的框会罩住整个房间 —— 宁可画点。"""
        visible, _ = select_display_objects([record(size=[40.0, 40.0, 40.0])])
        self.assertIsNone(visible[0]["size"])


class TargetVerifiedTest(unittest.TestCase):
    """§7.3 最后一行：复核目标走例外规则。"""

    def test_verified_target_is_shown_despite_a_low_score(self) -> None:
        shown, reason = display_decision(
            record(raw_label="chair", label_zh="办公椅", confidence=0.2,
                   observation_count=1, status="TENTATIVE", target_verified=True)
        )
        self.assertTrue(shown)
        self.assertEqual(reason, "TARGET_VERIFIED")

    def test_raw_detections_get_no_exception(self) -> None:
        """原始候选不能自动获得例外 —— 例外只认显式字段。"""
        for overrides in ({"target_verified": False}, {}):
            payload = record(confidence=0.2, **overrides)
            payload.pop("target_verified", None) if not overrides else None
            self.assertFalse(display_decision(payload)[0])

    def test_verified_flag_survives_normalisation(self) -> None:
        self.assertTrue(
            normalize_snapshot_object({"label": "chair", "target_verified": True})[
                "target_verified"
            ]
        )
        self.assertFalse(
            normalize_snapshot_object({"label": "chair"})["target_verified"]
        )


class StatsAndAuditTest(unittest.TestCase):
    def test_stats_count_every_drop_by_reason(self) -> None:
        records = [
            record(object_id="keep"),
            record(object_id="low", confidence=0.10),
            record(object_id="colour", raw_label="purple", label_zh="紫色",
                   confidence=0.99),
            record(object_id="nan", confidence=float("nan")),
            record(object_id="nowhere", x=float("nan")),
        ]
        visible, stats = select_display_objects(records)
        self.assertEqual([item["object_id"] for item in visible], ["keep"])
        self.assertEqual(stats["input"], 5)
        self.assertEqual(stats["displayed"], 1)
        self.assertEqual(stats["filtered"], 4)
        self.assertEqual(
            stats["dropped"],
            {"below_display_threshold": 1, "non_object_label": 1,
             "invalid_score": 1, "invalid_position": 1},
        )

    def test_visible_records_carry_their_decision(self) -> None:
        visible, _ = select_display_objects(
            [record(target_verified=True), record(object_id="plain")]
        )
        self.assertEqual(visible[0]["decision"], "TARGET_VERIFIED")
        self.assertEqual(visible[1]["decision"], "DISPLAY")

    def test_audit_rows_keep_shown_and_dropped(self) -> None:
        """审计表必须两类都留：只记被丢的，事后答不了"为什么没上屏"。"""
        rows = audit_rows(
            [record(object_id="keep"), record(object_id="drop", confidence=0.1)]
        )
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["decision"], "DISPLAY")
        self.assertIsNone(rows[0]["drop_reason"])
        self.assertEqual(rows[1]["decision"], "DROP")
        self.assertEqual(rows[1]["drop_reason"], "below_display_threshold")
        for row in rows:
            self.assertEqual(
                set(row),
                {"object_id", "raw_label", "label_zh", "confidence",
                 "confidence_max", "observation_count", "status", "decision",
                 "drop_reason", "box_valid"},
            )


class _Observation:
    """`ObjectSpatialObservation` 的最小替身：入口门只读这三个字段。"""

    def __init__(self, label: str, confidence: float, label_zh: str = "") -> None:
        self.label = label
        self.label_zh = label_zh
        self.confidence = confidence


class LoadObjectRecordsTest(unittest.TestCase):
    """读 events.jsonl 这一步（§7.3 表里"同一 object_id 多次事件"那一行）。

    写端和离线回放共用这一个函数 —— 它是"回放出来的结论对写端有约束力"的
    前提，所以必须自己也有回归用例，不能只靠回放时人工看一眼。
    """

    def _write(self, events: list) -> str:
        handle = tempfile.NamedTemporaryFile(
            "w", suffix=".jsonl", encoding="utf-8", delete=False)
        with handle:
            for event in events:
                if isinstance(event, str):
                    handle.write(event + "\n")   # 允许塞坏行
                else:
                    handle.write(json.dumps(event, ensure_ascii=False) + "\n")
        self.addCleanup(Path(handle.name).unlink)
        return handle.name

    @staticmethod
    def _localized(object_id: str, confidence: float, count: int, **extra) -> dict:
        obj = {
            "object_id": object_id,
            "label": "speaker",
            "map_xyz": [1.0, 2.0, 0.5],
            "confidence": confidence,
            "observation_count": count,
        }
        obj.update(extra)
        return {"event": "semantic_object_localized", "object": obj}

    def test_latest_event_wins_for_the_same_object(self) -> None:
        """实体每次合并都会重写事件，所以"最新一条"就是"当前状态"。"""
        path = self._write([
            self._localized("obj_001", 0.9, 1),
            self._localized("obj_001", 0.2, 2),
            self._localized("obj_001", 0.375, 4),
        ])
        records = load_object_records(path)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["observation_count"], 4)
        self.assertEqual(records[0]["confidence"], 0.375)

    def test_events_without_a_position_are_skipped(self) -> None:
        """没有 map_xyz 就画不出东西，也不参与去重键 —— 直接跳过。"""
        path = self._write([
            {"event": "semantic_object_localized",
             "object": {"object_id": "obj_001", "label": "speaker"}},
            self._localized("obj_002", 0.7, 2),
        ])
        self.assertEqual([r["object_id"] for r in load_object_records(path)],
                         ["obj_002"])

    def test_other_events_and_broken_lines_are_ignored(self) -> None:
        path = self._write([
            '{"event": "motion_step", "detail": "not json-safe: \u0000"}',
            "{ this is not json }",
            # 写到一半被截断的行：现场断电/进程被杀时真的会出现，不能整份文件作废。
            '{"event": "semantic_object_localized", "object": {"object_id": "obj_x"',
            json.dumps({"event": "semantic_frame_objects_localized",
                        "objects": [{"label": "chair"}]}, ensure_ascii=False),
            self._localized("obj_003", 0.8, 3),
        ])
        self.assertEqual([r["object_id"] for r in load_object_records(path)],
                         ["obj_003"])

    def test_objects_without_an_id_fall_back_to_label_and_position(self) -> None:
        """老事件可能没写 object_id；退回 (label, 位置) 键。

        关键是**两个同标签但不同位置的物体不能被并成一个** —— 用 label 单独
        做键就会并，那正是计划书 §3 禁止的按标签合并。
        """
        path = self._write([
            {"event": "semantic_object_localized",
             "object": {"label": "chair", "map_xyz": [1.0, 2.0, 0.5],
                        "confidence": 0.7, "observation_count": 2}},
            {"event": "semantic_object_localized",
             "object": {"label": "chair", "map_xyz": [4.0, 5.0, 0.5],
                        "confidence": 0.8, "observation_count": 2}},
        ])
        records = load_object_records(path)
        self.assertEqual(len(records), 2)
        self.assertEqual([r["object_id"] for r in records], ["", ""])


class IngestGateTest(unittest.TestCase):
    """持久地图入口门（计划书 §5.1）。"""

    def test_low_score_observations_are_dropped(self) -> None:
        kept, dropped = filter_persistent_observations(
            [_Observation("垃圾桶", 0.12), _Observation("垃圾桶", 0.35),
             _Observation("办公椅", 0.82)],
            min_confidence=0.35,
        )
        self.assertEqual([item.confidence for item in kept], [0.35, 0.82])
        self.assertEqual(dropped["low_confidence"], 1)

    def test_non_object_labels_are_dropped_even_at_a_high_score(self) -> None:
        kept, dropped = filter_persistent_observations(
            [_Observation("purple", 0.95), _Observation("地面", 0.9),
             _Observation("角落", 0.88), _Observation("垃圾桶", 0.7)]
        )
        self.assertEqual(len(kept), 1)
        self.assertEqual(dropped["non_object_label"], 3)

    def test_invalid_scores_are_dropped(self) -> None:
        _kept, dropped = filter_persistent_observations(
            [_Observation("垃圾桶", float("nan")),
             _Observation("垃圾桶", -1.0)]
        )
        self.assertEqual(dropped["invalid_score"], 2)

    def test_label_is_checked_before_score(self) -> None:
        """`purple` 拿了 0.95 也该记成 non_object_label，不是 low_confidence。"""
        _kept, dropped = filter_persistent_observations([_Observation("purple", 0.95)])
        self.assertEqual(dropped["non_object_label"], 1)
        self.assertEqual(dropped["low_confidence"], 0)

    def test_order_and_identity_are_preserved(self) -> None:
        """返回的必须是同一批对象、原顺序 —— 下标映射直接建在它上面。"""
        items = [_Observation("垃圾桶", 0.5), _Observation("purple", 0.9),
                 _Observation("办公椅", 0.8)]
        kept, _ = filter_persistent_observations(items)
        self.assertEqual([id(item) for item in kept], [id(items[0]), id(items[2])])

    def test_counter_agrees_with_the_dropped_list(self) -> None:
        """统计口径：input = kept + 各原因之和，不许有"消失的观测"。"""
        items = [_Observation("垃圾桶", 0.5), _Observation("purple", 0.9),
                 _Observation("桶", 0.1), _Observation("椅", float("inf"))]
        kept, dropped = filter_persistent_observations(items)
        self.assertEqual(len(kept) + sum(dropped.values()), len(items))

    def test_gate_does_not_touch_the_raw_observation_log(self) -> None:
        """§8.1 验收："原始事件数量不因过滤而减少"。

        逐帧原始事件写的是 `observation.scene_objects`，入口门吃的是
        `localized`，两者是**两张不同的表**，门既不原地改元素、也不改入参
        列表本身。这条一旦松掉，"低分候选仍可在日志里取证"就没了。
        """
        items = [_Observation("purple", 0.9), _Observation("垃圾桶", 0.1),
                 _Observation("办公椅", 0.7)]
        snapshot = [(item.label, item.confidence) for item in items]
        length = len(items)
        filter_persistent_observations(items, min_confidence=0.35)
        self.assertEqual(len(items), length)
        self.assertEqual([(item.label, item.confidence) for item in items], snapshot)


if __name__ == "__main__":
    unittest.main()
