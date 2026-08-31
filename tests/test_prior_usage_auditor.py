from __future__ import annotations

import unittest

from app.config import Settings
from app.reasoning.prior_usage_auditor import build_prior_usage_report


class PriorUsageAuditorTest(unittest.TestCase):
    def test_report_fails_when_static_kb_used(self) -> None:
        report = build_prior_usage_report(
            settings=Settings(siliconflow_api_key=""),
            static_kb_used=True,
        )

        self.assertFalse(report["passed"])
        self.assertEqual(report["violations"][0]["type"], "static_kb")


if __name__ == "__main__":
    unittest.main()
