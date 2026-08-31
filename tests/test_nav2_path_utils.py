import unittest
from app.navigation.nav2_path_utils import *
class PathUtilsTest(unittest.TestCase):
    def test_length_and_cumulative(self):
        p=[{"x":0,"y":0},{"x":3,"y":4}]
        self.assertEqual(compute_path_length(p),5); self.assertEqual(compute_cumulative_distances(p),[0,5])
    def test_rdp(self):
        p=[{"x":0,"y":0},{"x":1,"y":.01},{"x":2,"y":0}]
        self.assertEqual(len(simplify_path_rdp(p,.1)),2)
    def test_progress_clamped(self): self.assertEqual(compute_progress_ratio(10,20),0)
    def test_pixel_flip(self): self.assertEqual(map_to_pixel(1,2,0,0,1,10),(1,7))
