"""§3-C 的请求边界：**请求只表达任务、预算、检测模式、干跑/运动授权**。

三条性质，都要能区分"改对了"和"看起来改对了"：

1. ``reasoner`` 不再是请求选项。旧实现里请求体可以选 ``legacy/hybrid``，
   ``from_dict`` 还会把历史取值归一成 ``semantic``；现在整条链只有
   ``run_semantic_exploration.py --reasoner`` 的 CLI 名一个取值。
2. ``rtabmap`` 从生产请求模型里消失。它以前是一条真的能走通的链路
   （``{"rtabmap": true}`` → ``--rtabmap`` → RTAB-Map SpatialProvider），
   所以光删字段不够 —— 还得钉住 ``build_argv`` 不再认这个参数。
3. RGB-D / 空间探索档位不再由请求体选择。请求体写 ``spatial_v2: false``
   之类不再是"关掉空间探索"，而是**没有任何效果**；生效值只在配置里，
   而配置读坏了必须显式抛错，不能静默退回另一套默认值。

第 3 条里"忽略请求体"和"旧实现被删了"在行为上恰好同形（旧默认也是这些值），
所以这里断言的是**判别性**的那一半：显式写反的值不生效，且写坏的配置会抛错。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from app.manual_web_demo import search_models
from app.manual_web_demo.search_models import (
    SEARCH_REASONER,
    SearchStartRequest,
    load_search_defaults,
)
from scripts.go2w.autonomous_search_worker import build_argv

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _harden(argv: list[str], flag: str) -> bool:
    return flag in argv


# --------------------------------------------------------------------------- #
# 1. reasoner
# --------------------------------------------------------------------------- #
def test_reasoner_is_not_a_request_field() -> None:
    request = SearchStartRequest.from_dict({"task_text": "找垃圾桶"})
    assert not hasattr(request, "reasoner"), (
        "请求模型不再提供 reasoner —— 它由 Web 侧固定传 CLI 名"
    )
    # 请求体里带旧值不再有任何效果，也不该报错（老前端仍会发这个键）。
    assert SearchStartRequest.from_dict(
        {"task_text": "找垃圾桶", "reasoner": "legacy"}
    ) == request


def test_worker_command_always_carries_the_cli_reasoner() -> None:
    for params in (
        {"target": "垃圾桶"},
        {"target": "垃圾桶", "reasoner": "legacy"},
        {"target": "垃圾桶", "reasoner": "hybrid"},
    ):
        argv = build_argv(params)
        assert argv[argv.index("--reasoner") + 1] == SEARCH_REASONER, params


def test_the_reasoner_constant_is_a_real_cli_choice() -> None:
    """常量必须与 ``run_semantic_exploration.py --reasoner`` 的 choices 同源。

    build_argv 里那个字面量是硬编码的，跟这里各写一份；argv 一旦对不上
    argparse 的 choices，worker 会在 parse_args 阶段炸掉。
    """
    source = (PROJECT_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py").read_text(
        encoding="utf-8"
    )
    assert 'choices=("legacy", "semantic", "semantic_navigation", "hybrid")' in source
    assert SEARCH_REASONER == "semantic_navigation"
    # 旧内部值不再出现在 Web 侧的 worker 命令里。
    argv = build_argv({"target": "垃圾桶"})
    assert argv[argv.index("--reasoner") + 1] != "semantic"


# --------------------------------------------------------------------------- #
# 2. rtabmap
# --------------------------------------------------------------------------- #
def test_rtabmap_is_gone_from_the_request_model() -> None:
    request = SearchStartRequest.from_dict({"task_text": "找垃圾桶"})
    assert not hasattr(request, "rtabmap")
    # 旧对象字面量路径（直接构造）也要被 validate 挡住，而不是跑成另一条链路。
    rejected = SearchStartRequest(
        task_text="找垃圾桶", spatial_provider="rtabmap"
    ).validate()
    assert rejected is not None and "unsupported spatial_provider" in rejected, rejected


def test_worker_never_emits_the_rtabmap_flag() -> None:
    assert not _harden(build_argv({"target": "垃圾桶", "rtabmap": True}), "--rtabmap")


# --------------------------------------------------------------------------- #
# 3. 生产固定的空间/RGB-D 档位
# --------------------------------------------------------------------------- #
def test_spatial_profile_is_not_request_selectable() -> None:
    hostile = SearchStartRequest.from_dict({
        "task_text": "找垃圾桶",
        "spatial_v2": False,
        "spatial_provider": "camera",
        "rgbd_source": False,
        "rgbd_base_url": "http://127.0.0.1:1",
    })
    defaults = load_search_defaults()
    assert hostile.spatial_v2 == defaults["spatial_v2"] is True
    assert hostile.spatial_provider == defaults["spatial_provider"] == "plain_slam"
    assert hostile.rgbd_source == defaults["rgbd_source"] is True
    assert hostile.rgbd_base_url == defaults["rgbd_base_url"]
    # 生产默认必须自洽：plain_slam 是合法档位，直接构造也不会被 validate 拦下。
    assert hostile.validate() is None


def test_missing_config_falls_back_to_builtin_defaults(tmp_path, monkeypatch) -> None:
    """"没配" 不是错误：文件不在就用内置默认值（backend 仍是 nav2）。"""
    monkeypatch.setattr(search_models, "_PROJECT_ROOT", tmp_path)
    defaults = load_search_defaults()
    assert defaults["backend"] == "nav2"
    assert defaults["spatial_provider"] == "plain_slam"


def test_a_broken_config_raises_instead_of_switching_defaults(
    tmp_path, monkeypatch
) -> None:
    """写坏的 yaml 必须显式炸掉。

    旧实现把异常吞了，WebUI 会静默退回内置默认值 —— detector 从 grounded_sam
    变成 llm，后端/档位一起换掉，而界面上看不出任何区别。这里钉住"抛错"，
    并且钉住它在请求边界上也是抛错（from_dict）而不是悄悄放行。
    """
    config_dir = tmp_path / "configs" / "go2w"
    config_dir.mkdir(parents=True)
    (config_dir / "autonomous_search_web.yaml").write_text(
        "search: [unclosed\n", encoding="utf-8"
    )
    monkeypatch.setattr(search_models, "_PROJECT_ROOT", tmp_path)
    with pytest.raises(RuntimeError):
        load_search_defaults()
    with pytest.raises(RuntimeError):
        SearchStartRequest.from_dict({"task_text": "找垃圾桶"})


def test_a_non_mapping_config_raises(tmp_path, monkeypatch) -> None:
    """合法 YAML 但形状不对（顶层是列表）同样是显式错误，不是静默默认值。"""
    config_dir = tmp_path / "configs" / "go2w"
    config_dir.mkdir(parents=True)
    (config_dir / "autonomous_search_web.yaml").write_text(
        "- nav2\n- nav2\n", encoding="utf-8"
    )
    monkeypatch.setattr(search_models, "_PROJECT_ROOT", tmp_path)
    with pytest.raises(RuntimeError):
        load_search_defaults()


# --------------------------------------------------------------------------- #
# 4. 配置里不再留死键
# --------------------------------------------------------------------------- #
def test_the_web_config_no_longer_advertises_dead_knobs() -> None:
    """删掉的死键别又被人加回来 —— 加回来就是又一个"改了没反应"的旋钮。

    白名单来自 ``load_search_defaults`` 真正读的那几个键。
    """
    import yaml

    path = PROJECT_ROOT / "configs" / "go2w" / "autonomous_search_web.yaml"
    search = (yaml.safe_load(path.read_text(encoding="utf-8")) or {})["search"]
    consumed = {
        "backend", "detector", "finish_on_visual_confirmation",
        "enable_autonomous_motion", "operator_supervised_experiment",
        "rgbd_source", "rgbd_base_url", "spatial_v2", "spatial_provider",
    }
    assert set(search) == consumed, set(search) ^ consumed
