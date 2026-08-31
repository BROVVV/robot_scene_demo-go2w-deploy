from __future__ import annotations

import subprocess
import tempfile
import unittest
import json
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

from app.config import Settings
from app.detectors.grounded_sam_subprocess import (
    DetectorRuntimeError,
    GroundedSAMSubprocessDetector,
)
from app.video.target_profile import TargetProfile


class GroundedSAMRuntimeTest(unittest.TestCase):
    def test_dynamic_retry_uses_focused_prompt_and_parses_result(self) -> None:
        detector = GroundedSAMSubprocessDetector(
            Settings(
                siliconflow_api_key="",
                grounded_sam_root="/tmp",
                grounded_sam_python="python",
                enable_sam2=False,
            ),
            target_profile=TargetProfile(
                raw_query="找灭火器",
                canonical_name_zh="灭火器",
                primary_labels_en=["fire extinguisher"],
                context_labels_en=["wall"],
            ),
        )
        commands: list[list[str]] = []

        def fake_run(command, **kwargs):
            commands.append(command)
            output = Path(command[command.index("--output") + 1])
            output.write_text(
                json.dumps(
                    {
                        "objects": [
                            {
                                "label": "red fire extinguisher",
                                "score": 0.91,
                                "bbox_2d": [0.1, 0.2, 0.2, 0.6],
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        with tempfile.TemporaryDirectory() as tmpdir:
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"mock")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.run",
                side_effect=fake_run,
            ):
                objects = detector.detect_with_dynamic_terms(
                    str(image),
                    "找灭火器",
                    ["portable extinguisher"],
                )

        prompt = commands[0][commands[0].index("--text-prompt") + 1]
        self.assertIn("fire extinguisher", prompt)
        self.assertIn("portable extinguisher", prompt)
        self.assertNotIn("office chair", prompt)
        self.assertEqual(objects[0].label_zh, "灭火器")

    def test_timeout_is_wrapped_as_detector_runtime_error(self) -> None:
        detector = GroundedSAMSubprocessDetector(
            Settings(
                siliconflow_api_key="",
                grounded_sam_root="/tmp",
                grounded_sam_python="python",
                detector_timeout_seconds=1.0,
            ),
            target_profile=TargetProfile(
                raw_query="找灭火器",
                canonical_name_zh="灭火器",
                primary_labels_en=["fire extinguisher"],
            ),
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            image = Path(tmpdir) / "image.jpg"
            image.write_bytes(b"not-an-image-but-worker-is-mocked")
            with patch(
                "app.detectors.grounded_sam_subprocess.subprocess.run",
                side_effect=subprocess.TimeoutExpired("worker", 1.0),
            ):
                with self.assertRaisesRegex(
                    DetectorRuntimeError,
                    "timed out",
                ):
                    detector.detect(str(image), "找灭火器")


if __name__ == "__main__":
    unittest.main()
