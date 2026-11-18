"""中文显示名解析器与 RAM++ en/zh 配对（计划书 §7.1）。

整改前 WebUI 的三维物体框直接画 `label`，而 `label` 是 GroundingDINO 的 prompt
词/实体关联键 —— 于是 `bin`/`barrel`/`speaker` 原样上屏。这里测的就是"唯一的中文
显示名入口" `resolve_label_zh` 在四条来源上的优先级，以及它**永远不会**把未知英文
当结果漏出去。

关联键 `association_label` 是另一条路，本文件一并钉住它和显示名不相等这件事：
未知英文的显示名兜底成「未分类物体」，而关联键保留原文 —— 两者若混用，两个互不
相干的未知类别会被并成同一个实体（计划书 §3 明令禁止）。
"""

from __future__ import annotations

import unittest

from app.detectors.vocabulary import (
    LABEL_ZH_FALLBACK,
    association_label,
    filter_ram_scene_term_pairs,
    filter_ram_scene_terms,
    is_non_object_label,
    normalize_scene_tag,
    resolve_display_label,
    resolve_label_zh,
)


class ResolveLabelZhTest(unittest.TestCase):
    def test_static_dictionary(self) -> None:
        self.assertEqual(resolve_label_zh("speaker"), "音箱")
        self.assertEqual(resolve_label_zh("stool"), "凳子")
        self.assertEqual(resolve_label_zh("curtain"), "窗帘")
        # 本次整改新补的四个：整改前它们全都没有中文名，直接漏英文上屏。
        self.assertEqual(resolve_label_zh("bin"), "垃圾桶")
        self.assertEqual(resolve_label_zh("barrel"), "桶")

    def test_case_and_underscore_normalised(self) -> None:
        self.assertEqual(resolve_label_zh("Office_Chair"), "办公椅")

    def test_runtime_pair_beats_static_dictionary(self) -> None:
        """RAM++ 同一次调用给的中文优先于我们写死的词典。

        RAM 看着同一个框说「容器」，我们就不能写死成「垃圾桶」——它可能真是
        别的桶。这条也是"RAM 已经算好的中文不能再丢"（计划书 §2.2）的落点。
        """
        self.assertEqual(resolve_label_zh("bin", runtime_zh="容器"), "容器")
        self.assertEqual(resolve_label_zh("speaker", runtime_zh="扬声器"), "扬声器")

    def test_target_name_beats_everything(self) -> None:
        self.assertEqual(
            resolve_label_zh("chair", runtime_zh="椅子", target_zh="办公椅"), "办公椅"
        )

    def test_runtime_pair_must_actually_be_chinese(self) -> None:
        """英文/空串的"配对"不是配对，不能顶掉词典。

        真机跑过一版 tags_zh 与 tags_en 下标错位的 payload，表现就是拿英文来配。
        """
        self.assertEqual(resolve_label_zh("speaker", runtime_zh="speaker"), "音箱")
        self.assertEqual(resolve_label_zh("speaker", runtime_zh="  "), "音箱")

    def test_fallback_string_is_not_a_valid_runtime_pair(self) -> None:
        """「未分类物体」是兜底串不是知识，喂回来也不能算命中。

        这条挡的是一条具体的错路：调用方把已经解析过的 label_zh 再当 runtime_zh
        传回来。那种写法下「未分类物体」本身含中文，会被当成有效配对一路固化。
        """
        self.assertEqual(
            resolve_label_zh("gizmo", runtime_zh=LABEL_ZH_FALLBACK), LABEL_ZH_FALLBACK
        )
        self.assertEqual(
            resolve_label_zh("speaker", runtime_zh=LABEL_ZH_FALLBACK), "音箱"
        )

    def test_unknown_english_gets_chinese_fallback(self) -> None:
        value = resolve_label_zh("gizmo")
        self.assertEqual(value, LABEL_ZH_FALLBACK)
        self.assertEqual(value, "未分类物体")

    def test_already_chinese_is_preserved(self) -> None:
        """上游已经是中文时不许动它，也不许再"翻"一次。"""
        for label in ("垃圾桶", "蓝色垃圾桶", "办公椅"):
            self.assertEqual(resolve_label_zh(label), label)

    def test_never_returns_ascii_only_text(self) -> None:
        """本文件的核心断言：显示名里不许出现纯英文回退。"""
        for raw in ("gizmo", "unknown thing", "x", "", None, "   "):
            value = resolve_label_zh(raw)
            self.assertTrue(
                any("一" <= char <= "鿿" for char in value),
                f"{raw!r} 解析出 {value!r}，里面一个汉字都没有",
            )

    def test_association_key_keeps_raw_english(self) -> None:
        """关联键保留原文 —— 两个未知类别不能共用「未分类物体」当键。"""
        self.assertEqual(association_label("gizmo"), "gizmo")
        self.assertEqual(association_label("widget"), "widget")
        self.assertNotEqual(
            association_label("gizmo"), association_label("widget")
        )
        # 知道中文时关联键也用中文，与既有实体（中文 label）能对上。
        self.assertEqual(association_label("speaker"), "音箱")

    def test_display_name_is_always_chinese(self) -> None:
        self.assertEqual(resolve_display_label("gizmo"), "未分类物体")
        self.assertEqual(resolve_display_label("movie poster"), "电影海报")
        self.assertEqual(resolve_display_label("speaker"), "音箱")
        self.assertEqual(
            resolve_display_label("bin", runtime_zh="容器"), "容器"
        )
        self.assertEqual(resolve_display_label("chair", runtime_zh="chair"), "椅子")

    def test_prompt_free_common_entities_have_chinese_names(self) -> None:
        expected = {
            "luggage": "行李", "handbag": "手提包", "locker": "储物柜",
            "safe": "保险柜", "camera": "相机", "glasses": "眼镜",
            "wood-burning stove": "燃木炉", "arcade machine": "街机",
            "pillow": "枕头", "cushion": "靠垫",
        }
        for raw, zh in expected.items():
            self.assertEqual(resolve_display_label(raw), zh)


class RamSceneTermPairTest(unittest.TestCase):
    def test_pairs_by_original_index(self) -> None:
        pairs = filter_ram_scene_term_pairs(
            ["bin", "chair", "purple"], ["垃圾桶", "椅子", "紫色"]
        )
        self.assertEqual(pairs, [("bin", "垃圾桶"), ("chair", "椅子")])

    def test_missing_translation_is_empty_not_guessed(self) -> None:
        """没有中文配对就留空，不许拿英文顶或者猜一个。"""
        pairs = filter_ram_scene_term_pairs(["bin", "chair"], ["垃圾桶"])
        self.assertEqual(pairs, [("bin", "垃圾桶"), ("chair", "")])

    def test_terms_only_view_matches_the_pairs(self) -> None:
        """两个公开函数必须同一套过滤，否则配对表查不到词。"""
        tags_en = ["chair", "purple", "corner", "sparkle", "bin", "barrel"]
        tags_zh = ["椅子", "紫色", "角落", "闪光", "垃圾桶", "桶"]
        self.assertEqual(
            filter_ram_scene_terms(tags_en),
            [term for term, _zh in filter_ram_scene_term_pairs(tags_en, tags_zh)],
        )
        self.assertEqual(
            filter_ram_scene_terms(tags_en), ["chair", "bin", "barrel"]
        )

    def test_pairing_survives_a_dropped_term(self) -> None:
        """丢掉 `purple` 之后，后面的词不能整体错位一格。"""
        pairs = filter_ram_scene_term_pairs(
            ["purple", "bin", "sparkle", "barrel"],
            ["紫色", "垃圾桶", "闪光", "桶"],
        )
        self.assertEqual(pairs, [("bin", "垃圾桶"), ("barrel", "桶")])

    def test_duplicate_english_keeps_the_first_pairing(self) -> None:
        """重复英文只留第一次，且中文侧仍按**原下标**对回来。

        去重必须发生在配对之前、且不能压缩下标 —— 否则第二个 `bin` 会把
        后面的词整体前移一格，`barrel` 拿到 `桶`、`stool` 拿到 `chair` 的中文。
        """
        pairs = filter_ram_scene_term_pairs(
            ["bin", "barrel", "bin", "stool"],
            ["垃圾桶", "桶", "容器", "凳子"],
        )
        self.assertEqual(pairs, [("bin", "垃圾桶"), ("barrel", "桶"),
                                 ("stool", "凳子")])

    def test_normalize_scene_tag_is_the_shared_key(self) -> None:
        """配对表的键就是它 —— 检测回包查表用的也必须是同一个函数。"""
        self.assertEqual(normalize_scene_tag("Office Chair"), "office chair")
        self.assertEqual(normalize_scene_tag(" bin "), "bin")


class NonObjectLabelTest(unittest.TestCase):
    def test_colour_and_region_words_are_not_objects(self) -> None:
        for label in ("purple", "sparkle", "corner", "floor", "wall", "ceiling"):
            self.assertTrue(is_non_object_label(label), label)
        for label in ("紫色", "闪光", "角落", "地面", "墙", "天花板"):
            self.assertTrue(is_non_object_label(label), label)

    def test_object_words_are_kept(self) -> None:
        """「物体」是目标搜索的合法兜底词，拦掉会伤目标链路。"""
        for label in ("物体", "object", "item", "垃圾桶", "音箱"):
            self.assertFalse(is_non_object_label(label), label)

    def test_prompt_free_scene_media_and_parts_are_not_map_objects(self) -> None:
        for label in (
            "video", "elevator lobby", "airplane window", "bank vault",
            "hinge", "metal hinge", "door handle", "camera lens", "zoom lens",
            "sew", "sit", "walk", "electric chair", "bus window",
            "tv sitcom", "television show", "movie scene",
            "film premiere", "american shorthair", "glass bead", "soap bubble",
            "christmas ball", "patient", "newborn", "newborn baby",
        ):
            self.assertTrue(is_non_object_label(label), label)

    def test_prompt_free_standalone_objects_are_kept(self) -> None:
        for label in ("art print", "glasses", "weight", "chair", "fire extinguisher"):
            self.assertFalse(is_non_object_label(label), label)

    def test_either_language_triggers(self) -> None:
        """英文关联键 + 中文显示名一起判：只看一个必然漏。"""
        self.assertTrue(is_non_object_label("purple", "紫色"))
        self.assertTrue(is_non_object_label("purple", "未分类物体"))
        self.assertTrue(is_non_object_label("gizmo", "地面"))
        self.assertFalse(is_non_object_label("chair", "椅子"))


if __name__ == "__main__":
    unittest.main()
