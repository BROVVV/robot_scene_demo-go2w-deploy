import unittest
from app.navigation.nav2_instruction_preview import build_instruction_preview
class InstructionsTest(unittest.TestCase):
    def test_turn_and_warning(self):
        p=[{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}]
        result=build_instruction_preview(p,epsilon=0)
        self.assertIn("不是底层速度",result["warning"])
        self.assertIn("rotate_left",[s["action"] for s in result["steps"]])
