from __future__ import annotations

import unittest
from pathlib import Path
from unittest import mock

from app.task_understanding import llm_task_interpreter as _llm_interpreter
from scripts.evaluate_task_examples import evaluate_task_examples


ROOT = Path(__file__).resolve().parents[1]


class TaskExamplesEvaluatorTest(unittest.TestCase):
    def setUp(self) -> None:
        # §14-10 离线桩。`evaluate_task_examples` 的 `llm_client` 默认 None，
        # 而 `scripts/evaluate_task_examples.py:64` 是
        # `enable_verifier=llm_client is None` ⇒ 每条样例都走真实 LLM
        # （实测整条 84 秒，是这三条里最慢的）。
        # 那个脚本自己的注释就写着「This evaluator is also the repository's
        # offline fixture smoke test; keep it runnable without an API」，
        # 桩正是把它钉回这个设计意图：本用例断言的是 **examples/tasks 下的
        # fixture 能不能跑通检查项**，不是 LLM 解析质量。
        patcher = mock.patch.object(
            _llm_interpreter,
            "_default_openai_client",
            side_effect=RuntimeError("offline stub: real LLM disabled in tests"),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

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
