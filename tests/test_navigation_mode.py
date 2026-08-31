import unittest

from app.navigation.navigation_mode import VideoNavigationMode, normalize_video_navigation_mode


class NavigationModeTest(unittest.TestCase):
    def test_visual_preview_normalization(self):
        self.assertEqual(normalize_video_navigation_mode(None), VideoNavigationMode.VISUAL_PREVIEW)
        self.assertEqual(normalize_video_navigation_mode("offline_preview"), VideoNavigationMode.VISUAL_PREVIEW)


if __name__ == "__main__":
    unittest.main()
