"""Pandar 计划 §21 / §3.2：归档必须自证"这份地图由哪份代码、哪种时间戳模式生成"。

**为什么需要这些断言。** 2026-09-16 实测：

1. `configs/go2w/plain_slam_lock.yaml` 的 `commit`（`0c4b610f…`）与
   `ros2_ws/src/plain_slam_ros2` 的实际 HEAD（`65a135f2…`）不一致；
2. `provenance.json` 里**根本没有 commit 字段**（`save_plain_slam_map.sh` 只补
   `pandar_model` / `imu_source` / `saved_at` / `mapping_assist_only` / `map_3d_saved`）。

合起来就是：任何一份存档地图都无法回答"它是哪份代码跑出来的"。这不是文档瑕疵，
是归档自证失效 —— 正是这个仓库最怕的那类假绿。

**为什么 commit 不许等于 built_head。** `repository` 指向上游，而 `built_head`
是本地 Eigen 补丁提交，上游 clone 不到。所以这里断言的是"commit 是 built_head
的祖先"，不是"两者相等"。把 commit 改成 built_head 会让 lock 无法用于复现，
那是把一种坏换成了另一种坏。
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

LOCK = ROOT / "configs" / "go2w" / "plain_slam_lock.yaml"
VENDOR = ROOT / "scripts" / "go2w" / "vendor_plain_slam_ros2.sh"
SAVE = ROOT / "scripts" / "go2w" / "save_plain_slam_map.sh"
SLAM_SRC = ROOT / "ros2_ws" / "src" / "plain_slam_ros2"

needs_checkout = pytest.mark.skipif(
    not (SLAM_SRC / ".git").exists(),
    reason=(
        "ros2_ws/src/plain_slam_ros2 未 vendoring（嵌套 git 仓库，不随主仓分发）。"
        "本组断言比的是 lock 与**实际检出**，没有检出就无从比 —— 跳过而不是"
        "假装通过。vendoring 后重跑即可。"
    ),
)


def _rev_parse(rev: str) -> str:
    return subprocess.run(
        ["git", "-C", str(SLAM_SRC), "rev-parse", rev],
        capture_output=True, text=True, check=True,
    ).stdout.strip()


@needs_checkout
def test_lock_built_head_matches_the_actual_checkout() -> None:
    """`built_head` 必须等于实际 HEAD —— 这是它存在的全部意义。

    2026-09-16 之前这个字段不存在，lock 只有一个记着上游基线的 `commit`，
    于是"记录的是实际检出的 SHA"这句话是错的。
    """
    lock = yaml.safe_load(LOCK.read_text(encoding="utf-8"))
    recorded = lock["plain_slam_ros2"]["built_head"]
    actual = _rev_parse("HEAD")
    assert recorded == actual, (
        f"lock 的 built_head={recorded} 与实际检出 {actual} 不一致。"
        "跑 `bash scripts/go2w/vendor_plain_slam_ros2.sh` 刷新，"
        "或如果这是有意的检出变更，把新值一并提交。"
    )


@needs_checkout
def test_lock_commit_is_an_ancestor_of_built_head() -> None:
    """编译出来的树必须**派生自** lock 钉的那个基线。

    ⚠ 这条断言**分辨力有限**，别指望它拦"commit 被换成上游没有的提交"——
    注入校准（把 commit 换成 built_head）时它照样通过，因为一个提交是它自己的
    祖先，"是祖先"平凡成立。真正拦那件事的是下面那条远端可达性断言。
    两条都在，各拦一半：这条拦"钉了一个与构建无关的提交"。
    """
    lock = yaml.safe_load(LOCK.read_text(encoding="utf-8"))
    base = lock["plain_slam_ros2"]["commit"]
    head = lock["plain_slam_ros2"]["built_head"]
    result = subprocess.run(
        ["git", "-C", str(SLAM_SRC), "merge-base", "--is-ancestor", base, head],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, (
        f"lock 的 commit={base} 不是 built_head={head} 的祖先 —— "
        "说明 commit 已经和这棵树没有派生关系，复现语义失效。"
    )


@needs_checkout
def test_lock_commit_is_reachable_from_the_declared_upstream() -> None:
    """**这条才拦得住"commit 被写成上游 clone 不到的本地提交"。**

    lock 里 `repository:` 指向上游，所以 `commit` 必须能从上游取到；否则换一台
    机器按 lock 复现时会直接 checkout 失败。`built_head` 恰恰**不**满足这一点
    （它是本地 Eigen 补丁提交），这就是两个字段必须分开的原因。

    写法说明：最初写的是"commit 是 built_head 的祖先"。做注入校准时把 commit
    换成 built_head，它**照样通过**（自己是自己的祖先），分辨力为零 —— 所以
    改成查远端分支包含关系。改回去之前请先重跑那次注入。
    """
    lock = yaml.safe_load(LOCK.read_text(encoding="utf-8"))
    base = lock["plain_slam_ros2"]["commit"]
    result = subprocess.run(
        ["git", "-C", str(SLAM_SRC), "branch", "-r", "--contains", base],
        capture_output=True, text=True,
    )
    assert result.stdout.strip(), (
        f"lock 的 commit={base} 不在任何远端分支上 —— 它多半是本地补丁提交。"
        "把它当上游基线会让新机器 clone 不到，lock 也就无法用于复现。"
        f"git 输出：{result.stdout.strip()!r} {result.stderr.strip()!r}"
    )


def test_lock_documents_both_commits() -> None:
    """两个字段都必须在，且头部注释要讲清区别（否则下一个人还会当成一个）。"""
    lock = yaml.safe_load(LOCK.read_text(encoding="utf-8"))
    entry = lock["plain_slam_ros2"]
    assert "commit" in entry and "built_head" in entry
    header = LOCK.read_text(encoding="utf-8").split("schema_version")[0]
    assert "built_head" in header, "lock 头部没有解释 built_head 是什么"
    assert "上游" in header, "lock 头部没有说明 commit 是上游可达的基线"


def test_save_script_records_the_two_missing_provenance_fields() -> None:
    """§21 点名要求的 `plain_slam commit` 与 `point timestamp mode`。

    只断言"字面量在文件里"是不够的 —— 那正是仓库里"接了个永远不发火的事件"
    那类假绿的形状。所以同时断言它们取自运行时（rev-parse / 话题），不是常量。
    """
    src = SAVE.read_text(encoding="utf-8")
    assert '"plain_slam_commit"' in src, "provenance 没有 plain_slam_commit 字段"
    assert '"point_timestamp_mode"' in src, "provenance 没有 point_timestamp_mode 字段"
    assert '"rev-parse", "HEAD"' in src, (
        "plain_slam_commit 不是现取的实际检出 HEAD —— 写死或取错基准都会让"
        "这份 provenance 又不自证"
    )
    assert "/go2w/slam/point_status" in src, (
        "point_timestamp_mode 没有从 pandar_slam_adapter 发布的 point_status 取"
    )


def test_built_head_refresh_is_not_inside_the_placeholder_guard() -> None:
    """**本条是这次修复的病根。**

    `vendor_plain_slam_ros2.sh` 原来只在 `grep -q PENDING_VENDOR_FILL_ME` 成立
    时才写 lock。占位符一被填过就永远不再更新 —— 一个"跑过了、但再也不生效"
    的写入点，和 `place_graph.py` 那两个从未生效的默认值是同一种病。

    所以断言刷新 built_head 的那行在 `fi` **之后**。放回 `if` 里面就红。
    """
    lines = VENDOR.read_text(encoding="utf-8").splitlines()
    guard = next(
        i for i, line in enumerate(lines)
        if "PENDING_VENDOR_FILL_ME" in line and line.strip().startswith("if ")
    )
    close = next(i for i in range(guard, len(lines)) if lines[i].strip() == "fi")
    refresh = next(
        (i for i, line in enumerate(lines) if "built_head:.*" in line and "sed -i" in line),
        None,
    )
    assert refresh is not None, "vendor 脚本不再刷新 built_head"
    assert refresh > close, (
        "刷新 built_head 的 sed 又回到占位符判断里面了 —— 那样它只会生效一次，"
        "之后 lock 会重新变陈旧，正是本次要修的问题"
    )
