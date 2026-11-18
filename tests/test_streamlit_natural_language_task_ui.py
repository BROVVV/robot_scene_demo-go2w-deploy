from __future__ import annotations

import unittest
from pathlib import Path
from unittest import mock

from streamlit.testing.v1 import AppTest

from app.task_understanding import llm_task_interpreter as _llm_interpreter


class StreamlitNaturalLanguageTaskUiTest(unittest.TestCase):
    def setUp(self) -> None:
        # §14-10 离线桩。`streamlit_app.py` 的「自然语言任务」输入框有非空默认值
        # （`:84` `"帮我找到手机"`），AppTest 跑一次顶层渲染就会走到任务理解链路，
        # 而 `llm_task_interpreter.py:161` 是 `client = llm_client or
        # _default_openai_client()` ⇒ 每次都在打真实 SiliconFlow（实测 49 秒）。
        # 本测试断言的是 **UI 元素**（有没有那个输入框、预设下拉在不在），
        # 与 LLM 返回什么无关，所以把出口钉死在「不可用」不影响它的分辨力。
        patcher = mock.patch.object(
            _llm_interpreter,
            "_default_openai_client",
            side_effect=RuntimeError("offline stub: real LLM disabled in tests"),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_sidebar_uses_natural_language_task_input_without_presets(self) -> None:
        app = AppTest.from_file(
            str(Path(__file__).resolve().parents[1] / "streamlit_app.py"),
            default_timeout=30,
        ).run()

        self.assertEqual(len(app.exception), 0)
        self.assertIn("自然语言任务", [item.label for item in app.text_area])
        self.assertNotIn("任务模板", [item.label for item in app.selectbox])
        self.assertNotIn("目标描述", [item.label for item in app.text_area])


if __name__ == "__main__":
    unittest.main()
