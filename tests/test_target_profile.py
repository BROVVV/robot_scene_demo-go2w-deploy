from __future__ import annotations

import unittest
from types import SimpleNamespace

from app.config import Settings
from app.video.target_profile import TargetProfileResolver
from app.detectors.vocabulary import build_detection_prompts
from app.detectors.grounded_sam_subprocess import _to_detected_object
from app.video.target_profile import TargetProfile


class _FakeCompletions:
    def create(self, **kwargs):
        return SimpleNamespace(
            choices=[
                SimpleNamespace(
                    message=SimpleNamespace(
                        content="""{
                          "canonical_name_zh": "A3打印机",
                          "target_type": "attribute_relation",
                          "primary_labels_en": ["printer", "office printer"],
                          "aliases_zh": ["打印机", "打印设备"],
                          "aliases_en": ["laser printer"],
                          "attributes": ["supports A3 paper"],
                          "relation_constraints": [],
                          "context_labels_en": ["paper tray", "office desk"],
                          "context_labels_zh": ["纸盒", "办公桌"],
                          "likely_regions_zh": ["打印区", "办公设备区"],
                          "search_hint_zh": "优先检查办公设备区的打印机。"
                        }"""
                    )
                )
            ]
        )


class _FakeClient:
    def __init__(self):
        self.chat = SimpleNamespace(completions=_FakeCompletions())


class TargetProfileTest(unittest.TestCase):
    def test_fire_extinguisher_has_deterministic_fallback_terms(self) -> None:
        profile = TargetProfileResolver(
            settings=Settings(siliconflow_api_key="")
        ).resolve(
            "找到红色灭火器",
            use_llm=False,
        )
        self.assertIn("fire extinguisher", profile.detector_terms())

    def test_llm_resolves_unlisted_natural_language_target(self) -> None:
        profile = TargetProfileResolver(
            settings=Settings(siliconflow_api_key="test"),
            client=_FakeClient(),
        ).resolve("帮我找一台能打印 A3 纸的设备")

        self.assertEqual(profile.canonical_name_zh, "A3打印机")
        self.assertIn("printer", profile.detector_terms())
        self.assertIn("supports A3 paper", profile.attributes)
        self.assertIn("paper tray", profile.context_terms())
        self.assertEqual(profile.resolver_source, "llm")
        prompts = build_detection_prompts(
            profile.raw_query,
            dynamic_terms=profile.detector_terms(),
            context_terms=profile.context_labels_en,
        )
        self.assertIn("printer", prompts[0])
        self.assertTrue(any("paper tray" in prompt for prompt in prompts))

    def test_relation_anchor_is_always_a_context_term(self) -> None:
        # §15：关系锚点（「挂着书包的椅子」的书包）必须能当 context 传给裁剪复核。
        # 上游 LLM 实测会把锚点塞进 primary/aliases，而不是通用的 context_labels，
        # 单独立一个字段就是为了不让它丢 —— 这条测试锁住那个契约。
        profile = TargetProfile(
            raw_query="找挂着书包的椅子",
            canonical_name_zh="挂着书包的椅子",
            primary_labels_en=["chair"],
            aliases_en=["chair with backpack"],
            relation_constraints=["backpack hanging on chair"],
            relation_anchor_labels_en=["backpack"],
            relation_anchor_labels_zh=["书包"],
        )

        self.assertIn("backpack", profile.context_terms())
        self.assertIn("书包", profile.context_terms())
        # 只填了通用 context、漏填锚点时也必须能对上（防止字段填一半）。
        loose = TargetProfile(
            raw_query="找挂着书包的椅子",
            canonical_name_zh="挂着书包的椅子",
            primary_labels_en=["chair"],
            relation_constraints=["backpack hanging on chair"],
            context_labels_en=["backpack"],
        )
        self.assertIn("backpack", loose.context_terms())

    def test_fallback_strips_task_verb_without_api(self) -> None:
        profile = TargetProfileResolver(
            settings=Settings(siliconflow_api_key=""),
        ).resolve("请帮我找红色把手的白色柜门")

        self.assertEqual(profile.canonical_name_zh, "红色把手的白色柜门")
        self.assertEqual(profile.resolver_source, "fallback")

    def test_dynamic_grounded_label_uses_resolved_target_name(self) -> None:
        profile = TargetProfile(
            raw_query="寻找挂在墙上的红色消防器材",
            canonical_name_zh="红色消防器材",
            primary_labels_en=["fire extinguisher"],
            aliases_en=["wall-mounted extinguisher"],
        )
        detected = _to_detected_object(
            {
                "label": "wall-mounted extinguisher",
                "score": 0.8,
                "bbox_2d": [0.1, 0.2, 0.3, 0.6],
            },
            profile,
        )

        self.assertEqual(detected.label_zh, "红色消防器材")
        self.assertEqual(detected.category, "object")


if __name__ == "__main__":
    unittest.main()
