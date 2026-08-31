import unittest

from streamlit.testing.v1 import AppTest


class StreamlitVideoModeTest(unittest.TestCase):
    def test_quadruped_reasoning_controls_and_tabs_render(self) -> None:
        app = AppTest.from_file("streamlit_app.py", default_timeout=90).run()
        self.assertEqual(len(app.exception), 0)
        toggle_labels = {item.label for item in app.toggle}
        self.assertTrue(
            {
                "启用大模型情境推理",
                "使用长期经验记忆辅助推理",
                "显示 PSG 节点来源",
                "只显示机械狗可执行建议",
                "隐藏不可验证假设",
            }
            <= toggle_labels
        )

        app.button[0].click().run(timeout=90)

        self.assertEqual(len(app.exception), 0)
        tab_labels = {item.label for item in app.tabs}
        self.assertTrue(
            {
                "视觉观察结果",
                "大模型情境推理",
                "PSG 来源可视化",
                "机械狗下一视角计划",
                "不可验证/需人工区域",
                "长期经验写入记录",
            }
            <= tab_labels
        )

    def test_video_mode_renders_video_controls(self) -> None:
        app = AppTest.from_file("streamlit_app.py", default_timeout=90).run()
        self.assertEqual(len(app.exception), 0)
        self.assertIn("视频目标搜索", app.radio[0].options)
        self.assertNotIn("视频全场景建图", app.radio[0].options)

        app.radio[0].set_value("视频目标搜索").run()

        self.assertEqual(len(app.exception), 0)
        self.assertIn(
            "上传机器狗第一视角视频",
            [item.label for item in app.get("file_uploader")],
        )
        self.assertIn("视频检测器", [item.label for item in app.selectbox])
        detector = [
            item for item in app.selectbox if item.label == "视频检测器"
        ][0]
        self.assertEqual(detector.value, "llm")
        self.assertTrue(
            {"关键帧采样 FPS", "最大分析帧数"}
            <= {item.label for item in app.slider}
        )
        self.assertIn(
            "启用视频记忆",
            [item.label for item in app.toggle],
        )
        self.assertTrue(
            {
                "启用视频 PSG",
                "启用全场景建图辅助",
                "生成导航拓扑图",
                "使用拓扑图辅助目标搜索排序",
            }
            <= {item.label for item in app.toggle}
        )


if __name__ == "__main__":
    unittest.main()
