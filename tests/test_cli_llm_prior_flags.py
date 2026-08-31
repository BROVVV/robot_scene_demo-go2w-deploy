from __future__ import annotations

import unittest

from run_demo import parse_args
from run_video_demo import parse_args as parse_video_args


class CliLLMPriorFlagsTest(unittest.TestCase):
    def test_single_image_flags_parse(self) -> None:
        args = parse_args(
            [
                "--image",
                "x.jpg",
                "--target",
                "手机",
                "--enable-llm-prior",
                "--enable-observation-memory",
                "--enable-evidence-gating",
                "--disable-handwritten-priors",
                "--disable-static-kb",
                "--prior-audit",
            ]
        )

        self.assertTrue(args.enable_llm_prior)
        self.assertTrue(args.enable_observation_memory)
        self.assertTrue(args.enable_evidence_gating)
        self.assertTrue(args.disable_handwritten_priors)
        self.assertTrue(args.disable_static_kb)
        self.assertTrue(args.prior_audit)

    def test_video_flags_parse(self) -> None:
        args = parse_video_args(
            [
                "--video",
                "x.mp4",
                "--target",
                "手机",
                "--disable-llm-prior",
                "--disable-observation-memory",
                "--disable-evidence-gating",
            ]
        )

        self.assertFalse(args.enable_llm_prior)
        self.assertFalse(args.enable_observation_memory)
        self.assertFalse(args.enable_evidence_gating)


if __name__ == "__main__":
    unittest.main()
