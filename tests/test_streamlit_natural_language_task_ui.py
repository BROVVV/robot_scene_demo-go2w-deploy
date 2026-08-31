from __future__ import annotations

import unittest

from streamlit.testing.v1 import AppTest


class StreamlitNaturalLanguageTaskUiTest(unittest.TestCase):
    def test_sidebar_uses_natural_language_task_input_without_presets(self) -> None:
        app = AppTest.from_file("streamlit_app.py", default_timeout=30).run()

        self.assertEqual(len(app.exception), 0)
        self.assertIn("自然语言任务", [item.label for item in app.text_area])
        self.assertNotIn("任务模板", [item.label for item in app.selectbox])
        self.assertNotIn("目标描述", [item.label for item in app.text_area])


if __name__ == "__main__":
    unittest.main()
