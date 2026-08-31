from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from PIL import Image

from app.reasoning.llm_situated_search_reasoner import LLMSituatedSearchReasoner
from app.reasoning.observed_facts_builder import build_observed_scene_facts
from app.schemas import LLMReasoningRequest, SceneAnalysisResult, default_quadruped_capability
from app.services.image_annotator import export_reasoned_annotated_image


ROOT = Path(__file__).resolve().parents[1]


class ReasonedAnnotatorTest(unittest.TestCase):
    def test_draws_reasoned_annotation(self) -> None:
        scene = SceneAnalysisResult.model_validate(
            json.loads(
                (ROOT / "examples/mock_knowledge_aware_result.json").read_text(
                    encoding="utf-8"
                )
            )["base_scene"]
        )
        reasoning = LLMSituatedSearchReasoner(
            auto_create_client=False
        ).reason(
            LLMReasoningRequest(
                target_text="找到手机",
                observed_facts=build_observed_scene_facts(scene, "找到手机"),
                capability_contract=default_quadruped_capability(),
            )
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            source = Path(tmpdir) / "source.png"
            output = Path(tmpdir) / "reasoned.png"
            Image.new("RGB", (640, 480), "white").save(source)
            export_reasoned_annotated_image(scene, reasoning, source, output)
            with Image.open(output) as image:
                self.assertEqual(image.size, (640, 480))
            self.assertGreater(output.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
