from __future__ import annotations

import unittest

from app.detectors.vocabulary import (
    BASE_TERMS,
    RAM_SCENE_STOPLIST,
    build_detection_prompts,
    filter_ram_scene_terms,
)


class FilterRamSceneTermsTest(unittest.TestCase):
    def test_drops_scene_action_colour_and_material_words(self) -> None:
        tags = [
            "chair",
            "gray",
            "green",
            "room",
            "sit",
            "wood floor",
            "lamp",
            "indoor",
            "metal",
        ]

        self.assertEqual(filter_ram_scene_terms(tags), ["chair", "wood floor", "lamp"])

    def test_keeps_open_vocabulary_words_outside_known_vocabulary(self) -> None:
        # RAM++ 的意义就是扩大开放物体覆盖：不在词表里的多词短语必须保留，
        # 否则又退回封闭词表。
        tags = ["picture frame", "wall lamp", "fire cabinet", "network switch", "tool box"]

        self.assertEqual(filter_ram_scene_terms(tags), tags)

    def test_real_ram_tags_extend_coverage_beyond_the_static_vocabulary(self) -> None:
        # §64：普通 scene object coverage 相比固定词表明显增加。
        # 这里用的是 A7 离线验证里真实跑出来的 RAM++ tags（帧 bundle_137555.jpg），
        # 不是编的：12 个里 4 个落在 BASE_TERMS 里（backpack/chair/machine/office chair），
        # 另外 8 个是固定词表根本没有的类别。
        ram_tags = [
            "backpack", "barrel", "bin", "chair", "container", "equipment",
            "floor", "glass door", "glass wall", "machine", "office chair", "stool",
        ]
        base = {term.lower() for term in BASE_TERMS}

        kept = filter_ram_scene_terms(ram_tags)
        beyond_base = [term for term in kept if term not in base]

        self.assertEqual(
            beyond_base,
            [
                "barrel", "bin", "container", "equipment",
                "floor", "glass door", "glass wall", "stool",
            ],
        )

    def test_normalizes_and_dedupes(self) -> None:
        tags = ["  Chair ", "chair", "CARD  board   box", "chair."]

        self.assertEqual(filter_ram_scene_terms(tags), ["chair", "card board box"])

    def test_rejects_phrases_longer_than_four_words(self) -> None:
        tags = ["a b c d e", "a b c d"]

        self.assertEqual(filter_ram_scene_terms(tags), ["a b c d"])

    def test_caps_term_count(self) -> None:
        tags = [f"object{i}" for i in range(30)]

        self.assertEqual(len(filter_ram_scene_terms(tags, max_terms=5)), 5)

    def test_stoplist_has_no_multi_word_entries(self) -> None:
        # 过滤是按「整条 tag 命中」判断的，stoplist 里出现多词条目会静默失效。
        self.assertTrue(all(" " not in term for term in RAM_SCENE_STOPLIST))

    def test_ram_terms_get_their_own_prompt_slot(self) -> None:
        # 目标词那一槽位只有 16 个名额：RAM 词必须独占一槽，
        # 否则 RAM 返回的 tag 一多就会把用户目标词挤掉。
        target_terms = ["chair", "yellow chair"]
        ram_terms = [f"scene object {index}" for index in range(12)]

        prompts = build_detection_prompts(
            "黄色椅子",
            dynamic_terms=target_terms,
            include_base_terms=False,
        )
        prompts.append(". ".join(ram_terms) + ".")

        self.assertIn("chair", prompts[0])
        self.assertIn("yellow chair", prompts[0])
        self.assertNotIn("scene object 0", prompts[0])
        self.assertTrue(any("scene object 0" in prompt for prompt in prompts[1:]))


if __name__ == "__main__":
    unittest.main()
