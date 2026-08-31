from __future__ import annotations

from pathlib import Path
import unittest

JS = Path(__file__).resolve().parents[1] / "app/manual_web_demo/static/search_map.js"


class TestWebUITopologyOnlyContract(unittest.TestCase):
    def test_mode_is_topology_only(self) -> None:
        text = JS.read_text(encoding="utf-8")
        self.assertIn('this.mode = "semantic_topology"', text)
        # The mode switch must not permit spatial_map.
        self.assertNotIn('mode === "spatial_map"', text)
        self.assertNotIn('setMode("spatial_map")', text)

    def test_render_uses_semantic_graph(self) -> None:
        text = JS.read_text(encoding="utf-8")
        self.assertIn("var graph = this.spatial.semantic_graph || null;", text)
        self.assertIn("renderObjectTopology", text)

    def test_no_metric_occupancy_in_render(self) -> None:
        text = JS.read_text(encoding="utf-8")
        # The main render dispatch should never draw spatial overlays.
        self.assertNotIn("this.renderSpatialMap(this.data, this.spatial);", text)


if __name__ == "__main__":
    unittest.main()