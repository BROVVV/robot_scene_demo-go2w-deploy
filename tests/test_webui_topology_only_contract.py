from __future__ import annotations

from pathlib import Path
import unittest

JS = Path(__file__).resolve().parents[1] / "app/manual_web_demo/static/search_map.js"
UI_JS = Path(__file__).resolve().parents[1] / "app/manual_web_demo/static/search_ui.js"
INDEX_HTML = Path(__file__).resolve().parents[1] / "app/manual_web_demo/templates/index.html"


class TestWebUITopologyOnlyContract(unittest.TestCase):
    def test_spatial_map_mode_is_entirely_gone(self) -> None:
        """计量"空间地图"视图连入口一起移除（2026-09-15）。

        它的按钮早已是假的——`setMode()` 把 mode 硬写成 "semantic_topology"，
        点"空间地图"只换个高亮、画面纹丝不动。三维建图上线后它更没有存在意义。
        这里断言的是"彻底没有"，而不是"没被调用"。
        """
        js = JS.read_text(encoding="utf-8")
        ui_js = UI_JS.read_text(encoding="utf-8")
        html = INDEX_HTML.read_text(encoding="utf-8")
        for text in (js, ui_js):
            self.assertNotIn('"spatial_map"', text)
            self.assertNotIn("setMode", text)
            self.assertNotIn("getMode", text)
            self.assertNotIn("this.mode =", text)
        # 入口按钮与它专属的图例也必须从模板里消失。
        self.assertNotIn('data-mode="spatial_map"', html)
        self.assertNotIn("map-view-toggle", html)
        self.assertNotIn("map-legend", html)
        self.assertNotIn("mapLegend", ui_js)

    def test_render_uses_single_graph_argument(self) -> None:
        """计划书 §2.1：图只有一个来源 —— `state.graph`。

        渲染器不再从 `spatial.semantic_graph` 里翻第二张图，调用方也不再传
        一份 "map" 进来；`MAP_UPDATED` 只带 revision。
        """
        text = JS.read_text(encoding="utf-8")
        # Projection selection is executed in the headless render tests;
        # both object and integrated views still use the same state.graph.
        self.assertIn("renderObjectTopology", text)
        for stale in ("this.spatial.semantic_graph", "this.data", "semantic_graph"):
            self.assertNotIn(stale, text)
        ui = UI_JS.read_text(encoding="utf-8")
        self.assertNotIn("appState.map", ui)
        self.assertIn("appState.graph = state.graph || null;", ui)
        self.assertIn("pullGraphState();", ui)

    def test_no_metric_occupancy_in_render(self) -> None:
        text = JS.read_text(encoding="utf-8")
        # The main render dispatch should never draw spatial overlays.
        self.assertNotIn("this.renderSpatialMap(this.data, this.spatial);", text)


if __name__ == "__main__":
    unittest.main()
