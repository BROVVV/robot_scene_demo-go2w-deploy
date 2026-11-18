"""任务 Agent 解释层（WebUI Agent 面板）的展示契约测试。

前端契约用 headless node 跑 ``agent_trace.js`` 暴露的纯渲染层
``window.AgentTrace``：展示规则（角度正负约定、"未提供"而不是 0、used /
context_only、真实排序分量、offline 标签、图标+文字状态、文本转义、
下游不猜拒绝原因）全部落在那一层，所以可以脱离浏览器钉住。

见《GO2W 任务Agent与寻路决策链_WebUI离线改造实施计划》§5.2 / §5.3 / §5.4。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

NODE = shutil.which("node")
JS_TEST = Path(__file__).with_name("test_webui_agent_trace_contract.js")
ROOT = Path(__file__).resolve().parents[1]
AGENT_TRACE = ROOT / "app/manual_web_demo/static/agent_trace.js"
SEARCH_UI = ROOT / "app/manual_web_demo/static/search_ui.js"
INDEX = ROOT / "app/manual_web_demo/templates/index.html"


@pytest.mark.skipif(NODE is None, reason="node.js not available")
def test_frontend_agent_panel_contract():
    result = subprocess.run(
        [NODE, str(JS_TEST)], capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, (
        f"node agent trace contract test failed:\n{result.stdout}\n{result.stderr}"
    )
    assert "ALL JS AGENT TRACE CONTRACT TESTS PASSED" in result.stdout


def test_panel_is_wired_into_the_decision_cell_without_replacing_it():
    html = INDEX.read_text(encoding="utf-8")
    # §5.1 的布局顺序：Agent 解释层排在"下一步决策"之前，后者原封不动。
    for element in ("agent-mode", "agent-safety", "agent-plan", "agent-choice",
                    "agent-downstream", "agent-candidates", "agent-knowledge",
                    "agent-inputs", "agent-uncertainties"):
        assert f'id="{element}"' in html, element
    assert html.index('id="agent-uncertainties"') < html.index("下一步决策")
    assert html.index("下一步决策") < html.index('id="dec-candidates"')
    # 它必须是解释层：候选、地图、安全状态一个都没被替换掉。
    for kept in ("dec-candidates", "dec-history", "search-map",
                 "st-embodiment", "search-timeline"):
        assert f'id="{kept}"' in html, kept


def test_agent_trace_module_loads_before_search_ui():
    html = INDEX.read_text(encoding="utf-8")
    assert html.index("agent_trace.js") < html.index("search_ui.js")
    ui = SEARCH_UI.read_text(encoding="utf-8")
    # 展示规则只有一份实现：search_ui.js 只挂 DOM，不许自己再判一次。
    assert "window.AgentTrace.render(" in ui
    assert "window.AgentTrace.renderHistoryLine(" in ui
    for duplicated in ("function headingText", "function num(", "PLAN_STATUS =",
                       "DOWNSTREAM_STATUS =", "ANGLE_CONVENTION ="):
        assert duplicated not in ui, f"{duplicated} 不该在 search_ui.js 里重实现"


def test_polling_fallback_also_carries_the_agent_trace():
    """WS 掉线时兜底轮询必须也能刷新面板，否则它会停在旧结论上。"""
    ui = SEARCH_UI.read_text(encoding="utf-8")
    assert "if (state.agent) appState.search.agent = state.agent;" in ui
