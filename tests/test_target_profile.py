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
