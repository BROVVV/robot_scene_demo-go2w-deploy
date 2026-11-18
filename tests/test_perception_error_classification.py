"""计划书 §8.2 结构化感知错误码 —— 分类器的可达性与归段正确性。

审计报告 `reports/go2w_plan_completion_audit_20260915.md:166` 的原话：
``RGB_IMAGE_ERROR`` / ``FRAME_MISMATCH`` 只存在于 docstring，无任何 ``raise`` 使用
→ 相机/深度抓取失败会落到 ``UNKNOWN_PERCEPTION_ERROR``，**现场定位走弯路**。

**真机数据决定了两件事，本文件把它们钉成断言：**

1. **窄关键词**。分类器一度用裸的 ``image`` 触发"相机段"，结果真机上出现过的那条
   ``UnboundLocalError: local variable 'image_stamp'``（一个代码 bug）被误判成
   ``RGB_IMAGE_ERROR`` —— 现场会去查相机。宽版本是错的，窄版本才对。
2. **归段优先于归因**。旧写法把 ``timeout`` 判在最前，于是
   ``RGB-D capture failed: ... timed out`` 记成 ``QUICK_VLM_TIMEOUT``。两个码都
   合法、都 recoverable，但指向完全不同的排查方向。

**校准状态（不许含糊）**：真机上到 2026-09-16 只出现过两类感知失败
（``SiliconFlow vision API timed out`` 22 次、``UnboundLocalError: image_stamp``
1 次），两者都在下面用**原始文本**复现。``RGB_IMAGE_ERROR`` 与 ``DEPTH_ERROR``
**从未在真机上出现过**，对应的用例是**合成的**，证明的是"这两个码产生得出来"，
不是"分得准"。不要把它们当已校准。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts" / "go2w") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts" / "go2w"))

from run_semantic_exploration import _classify_perception_error  # noqa: E402


def classify(text: str) -> str:
    return _classify_perception_error(RuntimeError(text))


# --- 真机原始文本（这两条有校准，别改成合成的） ---------------------------

def test_real_siliconflow_timeout_stays_a_quick_vlm_timeout() -> None:
    """真机 22 次。改分类器时**不能**把它挪走 —— 这是唯一有量的真样本。"""
    assert classify("SiliconFlow vision API timed out") == "QUICK_VLM_TIMEOUT"


def test_real_unbound_local_error_is_not_blamed_on_the_camera() -> None:
    """真机 1 次。这是把宽关键词否掉的那条证据。

    ``image_stamp`` 只是变量名。若分类器用裸的 ``image`` 判"相机段"，这条会变成
    ``RGB_IMAGE_ERROR``，而它实际是个 ``UnboundLocalError`` 代码 bug —— 现场会带着
    错误的结论去查相机链路。
    """
    assert classify(
        "UnboundLocalError: local variable 'image_stamp' referenced before assignment"
    ) == "UNKNOWN_PERCEPTION_ERROR"


# --- 归段：先认哪一段，再认什么毛病 --------------------------------------

def test_rgbd_timeout_is_not_reported_as_a_quick_vlm_timeout() -> None:
    """**注入校准的意义所在**：把 ``timeout`` 判回最前面，这条先红。

    取帧超时与 VLM 超时都 recoverable，重试行为相同，所以这个错误**不会以任何
    功能故障的形式暴露出来** —— 它只在人翻日志时伤人。这正是需要断言的原因。
    """
    assert classify("RGB-D capture failed: TimeoutError: timed out") == "RGBD_TIMEOUT"


def test_rgb_image_error_is_reachable() -> None:
    """审计报告点名的那一条：非超时的取帧失败此前落到 UNKNOWN。"""
    assert classify(
        "RGB-D capture failed: RuntimeError: cannot decode jpeg"
    ) == "RGB_IMAGE_ERROR"


def test_depth_failure_inside_the_capture_path_is_a_depth_error() -> None:
    """取帧失败的文本一律以 ``RGB-D capture failed: `` 开头，所以"图像还是深度"
    必须在同一个分支**内部**再判一次 —— 放到外面判永远够不着 ``DEPTH_ERROR``。"""
    assert classify(
        "RGB-D capture failed: RuntimeError: depth stream returned no data"
    ) == "DEPTH_ERROR"
    assert classify("depth_localizer: no depth for frame 12") == "DEPTH_ERROR"


def test_depth_takes_a_back_seat_to_a_timeout() -> None:
    """深度 + 超时 = 取帧超时。两者都出现时，"超时"是更可操作的那条信息。"""
    assert classify(
        "RGB-D capture failed: RuntimeError: depth stream timed out"
    ) == "RGBD_TIMEOUT"


# --- 既有码不许被这次改动碰掉 --------------------------------------------

@pytest.mark.parametrize("text,expected", [
    ("TF lookup failed for pslam_odom", "TF_UNAVAILABLE"),
    ("transform unavailable for base_link", "TF_UNAVAILABLE"),
    ("camera frame stale after motion", "RGBD_TIMEOUT"),
    ("rgbd source unavailable", "RGBD_TIMEOUT"),
])
def test_existing_mappings_survive(text: str, expected: str) -> None:
    assert classify(text) == expected


def test_explicit_code_wins_over_text_sniffing() -> None:
    """显式带码的异常必须原样透传，否则调用点传的 ``code=`` 会被文本猜测覆盖。"""
    class WithCode(RuntimeError):
        code = "FRAME_STALE"

    assert _classify_perception_error(WithCode("RGB-D capture failed")) == "FRAME_STALE"


def test_frame_mismatch_is_no_longer_advertised_as_a_failure_code() -> None:
    """§5.2 说帧对不上是**降级**（SEMANTIC_2D_ONLY + 事件），不是 PerceptionFailure。

    留一个永远 raise 不出来的码在 docstring 里，会让人以为"帧不匹配会自动重试"。
    """
    doc = (ROOT / "app" / "live_robot" / "autonomous_explorer.py").read_text(
        encoding="utf-8")
    head = doc[:doc.index("def __init__")]  # 只查 PerceptionFailure 的 docstring
    # 码表是逗号分隔的，所以查 "FRAME_MISMATCH,"：这样解释"它为什么被删掉"的那句
    # 散文（用反引号包着、后面没有逗号）不会把断言顶红。
    assert "FRAME_MISMATCH," not in head, "FRAME_MISMATCH 又回到了错误码表里"
    assert "FRAME_STALE," in head, "把 FRAME_STALE 也一起删掉了 —— 它是真实使用的码"
