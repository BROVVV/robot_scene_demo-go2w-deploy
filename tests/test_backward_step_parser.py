from __future__ import annotations

import unittest

from app.live_robot.step_parser import (
    backward_step_distance,
    forward_step_distance,
    translation_step_distance,
)


class TestBackwardStepParser(unittest.TestCase):
    def test_forward_parser(self) -> None:
        self.assertEqual(forward_step_distance("f", 0.1), 0.1)
        self.assertEqual(forward_step_distance("f0.05", 0.1), 0.05)

    def test_backward_parser(self) -> None:
        self.assertEqual(backward_step_distance("b", 0.08), 0.08)
        self.assertEqual(backward_step_distance("b0.05", 0.1), 0.05)
        self.assertEqual(backward_step_distance("b0.10", 0.1), 0.10)
        self.assertEqual(backward_step_distance("b0.12", 0.1), 0.12)

    def test_translation_parser_returns_direction(self) -> None:
        self.assertEqual(
            translation_step_distance(
                "b0.10", default_forward_m=0.2, default_backward_m=0.08
            ),
            ("backward", 0.10),
        )
        self.assertEqual(
            translation_step_distance(
                "f0.10", default_forward_m=0.2, default_backward_m=0.08
            ),
            ("forward", 0.10),
        )

    def test_rejects_invalid_backward(self) -> None:
        for bad in ("b0", "b-0.1", "babc", "b1.5"):
            with self.assertRaises(ValueError):
                backward_step_distance(bad, 0.1)


if __name__ == "__main__":
    unittest.main()