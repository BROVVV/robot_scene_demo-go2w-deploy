"""物体识别后端（``--detector``）从 WebUI 到 worker 的整条链路。

背景（2026-09-17）：WebUI 起搜索时地图上扫到的物体一个 box 框都没有，只有红点。
真因是这一路走的是云端 VLM（``--detector llm``）——VLM 产不出 mask，``box_size``
恒为 None（``run_semantic_exploration.py:982`` 自己写着这句话）。而 grounded_sam
那条路（本地 GroundingDINO+SAM2，带真实 mask）的开关当时**没有任何配置入口**：
``build_argv`` 早就支持 ``detector → --detector``，但 yaml 里没有这个键、
``load_search_defaults`` 的白名单里没有它、``SearchStartRequest`` 也没有这个字段 ——
于是 argparse 的默认值 ``llm`` 恒定生效。

本文件锁的就是这段"每一环都存在、但中间断了三环"的链路。断言的是**接线**，不是
某个具体取值：值可以改（yaml 里一行），接错线不行。

配套的真机事实：本机 grounded_sam 栈可跑（conda python + CUDA 正常，首次调用
24.7 s 载模型，常驻后单帧 0.4 s，返回的物体带 mask_rle/mask_size）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "configs" / "go2w" / "autonomous_search_web.yaml"
sys.path.insert(0, str(PROJECT_ROOT))

from app.manual_web_demo.search_models import (  # noqa: E402
    SearchStartRequest,
    load_search_defaults,
)
from scripts.go2w.autonomous_search_worker import build_argv  # noqa: E402

# argparse 侧允许的取值，逐字取自 run_semantic_exploration.py 的 choices。
ALLOWED = {"llm", "grounded_sam", "yoloe_seg"}


def test_yaml_key_is_not_dead() -> None:
    """yaml 里写的 detector 必须真的被 load_search_defaults 读出来。

    这个仓里"改了 yaml 却什么都没变"是有前科的（同一个文件里就躺着 reasoner /
    max_search_seconds 等几个死键，注释里写着）。这条断言挡的就是再添一个死键。
    """
    written = (yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {})["search"]["detector"]
    assert written in ALLOWED
    assert load_search_defaults()["detector"] == written


def test_deployed_detector_is_grounded_sam() -> None:
    """当前部署取的是 grounded_sam：初始物体识别在本地做（带 mask ⇒ 有 3D box）。"""
    assert load_search_defaults()["detector"] == "grounded_sam"


def test_request_can_still_override() -> None:
    """请求体优先级最高，一行就能切回云端 VLM。"""
    assert SearchStartRequest.from_dict({"detector": "llm"}).detector == "llm"
    assert SearchStartRequest.from_dict({}).detector == "grounded_sam"


def test_unknown_detector_is_rejected_at_the_boundary() -> None:
    """非法值在请求边界就拦下，别等 worker 起来才炸成 argparse 错误。"""
    request = SearchStartRequest.from_dict({"target": "垃圾桶", "detector": "dino"})
    assert request.validate() == "unsupported detector: dino"


def test_worker_argv_carries_the_flag() -> None:
    """最后一环：build_argv 真的把它变成 ``--detector``。"""
    argv = build_argv({"target": "垃圾桶", "detector": "grounded_sam"})
    assert argv[argv.index("--detector") + 1] == "grounded_sam"
    # 缺省不能凭空塞一个：build_argv 对空值就是不传，由 argparse 默认值兜底。
    assert "--detector" not in build_argv({"target": "垃圾桶"})


@pytest.mark.parametrize("value", sorted(ALLOWED))
def test_choices_match_the_parser(value: str) -> None:
    """取值集合与 ``run_semantic_exploration.py --detector`` 的 choices 同源。"""
    source = (PROJECT_ROOT / "scripts" / "go2w" / "run_semantic_exploration.py").read_text(
        encoding="utf-8")
    assert 'choices=("llm", "grounded_sam", "yoloe_seg")' in source
    assert value in ALLOWED
