from __future__ import annotations

import unittest
from pathlib import Path

from scripts.evaluate_task_examples import evaluate_task_examples


ROOT = Path(__file__).resolve().parents[1]


class TaskExamplesEvaluatorTest(unittest.TestCase):
    def test_all_task_examples_pass_minimum_checks(self) -> None:
        reports = evaluate_task_examples(ROOT / "examples" / "tasks")

        self.assertGreaterEqual(len(reports), 5)
        self.assertTrue(all(report["passed"] for report in reports), reports)
        self.assertTrue(
            any(report["has_scene_fixture"] for report in reports),
            "At least one example should exercise scene reasoning.",
        )


if __name__ == "__main__":
    unittest.main()
