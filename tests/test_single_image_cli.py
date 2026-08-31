"""Single-image command-line compatibility checks."""

from __future__ import annotations

import contextlib
import io
import json
import unittest
from pathlib import Path

from pydantic import ValidationError

from app.schemas import SceneAnalysisResult
from run_demo import main, parse_args


class SingleImageCliTest(unittest.TestCase):
    def test_florence2_detector_is_not_a_cli_option(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--detector", "florence2"])

    def test_video_input_uses_video_pipeline_and_reports_missing_file(self) -> None:
        stderr = io.StringIO()

        with contextlib.redirect_stderr(stderr):
            exit_code = main(["--video", "input.mp4", "--target", "找到手机"])

        self.assertEqual(exit_code, 1)
        self.assertIn("Video file not found", stderr.getvalue())

    def test_scene_schema_rejects_readme13_geometry_fields(self) -> None:
        payload = json.loads(
            Path("examples/mock_scene_result.json").read_text(encoding="utf-8")
        )
        payload["geometry"] = {}

        with self.assertRaises(ValidationError):
            SceneAnalysisResult.model_validate(payload)


if __name__ == "__main__":
    unittest.main()
