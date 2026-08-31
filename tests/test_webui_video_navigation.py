import unittest


class WebuiVideoNavigationTest(unittest.TestCase):
    def test_render_helper_imports(self):
        from streamlit_app import _render_video_navigation_result

        self.assertTrue(callable(_render_video_navigation_result))


if __name__ == "__main__":
    unittest.main()
