"""§12.1：RAM++ 返回再多 tag，也不能把用户目标词挤出 GroundingDINO prompt。"""

from __future__ import annotations

import unittest
from unittest import mock

from app.config import Settings
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector
from app.task_understanding.schemas import GroundingPromptPlan
from app.video.target_profile import TargetProfile


def _yellow_chair_profile() -> TargetProfile:
    return TargetProfile(
        raw_query="找黄色椅子",
        canonical_name_zh="黄色椅子",
        primary_labels_en=["chair", "yellow chair"],
        aliases_en=["yellow seat"],
        colors=["yellow"],
    )


def _flood_of_ram_terms() -> list[str]:
    # 远超槽位上限的一批 RAM tag，用来逼出「目标词被挤掉」的回归。
    return [f"scene object {index}" for index in range(30)]


class RamTargetPriorityTest(unittest.TestCase):
    def _detector(self, **overrides) -> GroundedSAMSubprocessDetector:
        settings = Settings(
            siliconflow_api_key="",
            grounding_prompt_llm_expansion_enabled=False,
            static_object_prompts_enabled=True,
            **overrides,
        )
        return GroundedSAMSubprocessDetector(
            settings=settings, target_profile=_yellow_chair_profile()
        )

    def test_target_terms_survive_ram_flood(self) -> None:
        detector = self._detector()

        prompts = detector._build_prompts("找黄色椅子", [], _flood_of_ram_terms())

        self.assertTrue(prompts)
        self.assertIn("chair", prompts[0])
        self.assertIn("yellow chair", prompts[0])
        # RAM 词进自己的槽位，不参与目标词那一槽的 16 个名额竞争。
        self.assertNotIn("scene object 0", prompts[0])
        self.assertTrue(any("scene object 0" in prompt for prompt in prompts[1:]))

    def test_ram_terms_are_capped_not_dropped_entirely(self) -> None:
        detector = self._detector(ram_scene_max_terms=5)

        prompts = detector._build_prompts("找黄色椅子", [], _flood_of_ram_terms())

        ram_prompts = [prompt for prompt in prompts if "scene object" in prompt]
        self.assertEqual(len(ram_prompts), 1)
        self.assertEqual(ram_prompts[0].count("scene object"), 5)

    def test_a_hundred_ram_terms_cannot_evict_any_target_term(self) -> None:
        """§44.4：RAM 灌 100 个 term，截断之后目标词必须**一个不少**。

        既有那条 cap 测试用的是 30 个词、只断言 RAM 被截成 5 个，**从没断言过
        目标词还在**。这两件事必须分开测：把 RAM 截断写对了、同时把目标词一起
        截掉，是完全可能的（比如把两个槽位合成一个列表再切 `[:max_terms]`），
        而那正是 §44.4 要防的回归。
        """
        detector = self._detector()

        prompts = detector._build_prompts(
            "找黄色椅子", [], [f"scene object {index}" for index in range(100)]
        )

        # `detector_terms()` 的全量：primary + 颜色组合 + aliases，一个都不许掉。
        for term in ("chair", "yellow chair", "yellow seat"):
            self.assertIn(term, prompts[0])
        ram_prompts = [prompt for prompt in prompts if "scene object" in prompt]
        self.assertEqual(len(ram_prompts), 1)
        self.assertEqual(
            ram_prompts[0].count("scene object"),
            detector.settings.ram_scene_max_terms,
        )

    def test_the_tightest_ram_cap_does_not_shrink_the_target_slot(self) -> None:
        """把 RAM 槽位压到只剩 1 个名额，目标槽位也不许跟着缩。

        从另一头验同一件事：上一条问「输入再多」，这条问「cap 再小」。而
        `ram_scene_max_terms` 若被当成了整条 prompt 的全局上限（而不是只管 RAM
        那一槽），目标词就会被 `[:1]` 切到只剩 `chair` —— `yellow chair` 和
        alias `yellow seat` 静默消失，搜索退化成「找椅子」，而所有日志都正常。
        """
        detector = self._detector(ram_scene_max_terms=1)

        prompts = detector._build_prompts(
            "找黄色椅子", [], ["stool", "plant", "monitor"]
        )

        for term in ("chair", "yellow chair", "yellow seat"):
            self.assertIn(term, prompts[0])
        # cap 必须真的生效，否则这条在「完全忽略 cap」的实现下也会假绿。
        ram_prompts = [prompt for prompt in prompts if "stool" in prompt]
        self.assertEqual(len(ram_prompts), 1)
        self.assertEqual(ram_prompts[0], "stool.")

    def test_ram_terms_do_not_enter_the_target_slot(self) -> None:
        detector = self._detector()

        prompts = detector._build_prompts("找黄色椅子", [], ["stool", "plant", "monitor"])

        self.assertNotIn("stool", prompts[0])
        self.assertTrue(any("stool" in prompt for prompt in prompts[1:]))

    def test_base_terms_still_present_without_a_target_profile(self) -> None:
        detector = GroundedSAMSubprocessDetector(
            settings=Settings(
                siliconflow_api_key="",
                grounding_prompt_llm_expansion_enabled=False,
                static_object_prompts_enabled=True,
            )
        )

        prompts = detector._build_prompts("", [], ["plant"])

        joined = " ".join(prompts)
        self.assertIn("plant", joined)
        # RAM 扩词之后不能把静态基线词表整个丢掉（否则又退回封闭词表）。
        self.assertIn("chair", joined)

    def test_prompts_keep_period_separators(self) -> None:
        # worker 是按 "." 把 prompt 拆成单 term 逐个推理的；一旦分隔符被
        # 归一化掉，整串会当成一个 term 喂进去，检出的 label 就变成整条 prompt。
        detector = self._detector()

        prompts = detector._build_prompts("找黄色椅子", [], ["stool", "plant"])

        ram_prompt = [prompt for prompt in prompts if "stool" in prompt][0]
        worker_terms = [t.strip() for t in ram_prompt.split(".") if t.strip()]
        self.assertEqual(worker_terms, ["stool", "plant"])

    def test_llm_expansion_branch_appends_ram_after_target_terms(self) -> None:
        plan = GroundingPromptPlan(
            grounding_prompt="chair. yellow chair. table.",
            is_valid_for_grounding_dino=True,
        )
        detector = GroundedSAMSubprocessDetector(
            settings=Settings(
                siliconflow_api_key="", grounding_prompt_llm_expansion_enabled=True
            ),
            target_profile=_yellow_chair_profile(),
            parsed_task=object(),
            llm_client=object(),
        )

        with mock.patch(
            "app.detectors.grounded_sam_subprocess.GroundingPromptPlanner"
        ) as planner_cls:
            planner_cls.return_value.build.return_value = plan
            prompts = detector._build_prompts("找黄色椅子", [], ["stool", "plant"])

        self.assertEqual(len(prompts), 1)
        prompt = prompts[0]
        # 目标词来自 LLM planner，排在 RAM 词前面，永远不会被 RAM 截断。
        self.assertLess(prompt.index("yellow chair"), prompt.index("stool"))
        self.assertIn("plant", prompt)


if __name__ == "__main__":
    unittest.main()
