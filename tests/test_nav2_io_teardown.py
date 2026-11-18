"""计划书 §P3-1：nav2 IO 收尾必须自己吞掉失败，且必须幂等。

这一段代码只在 ``_run_go2w_explorer`` 的 ``finally`` 里跑，那一层要 rclpy，``.venv``
里进不去 —— 所以收尾被提成模块级函数 ``_teardown_nav2_io(node)``，node 按鸭子类型
用。这里测的就是它。
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import tokenize

from scripts.go2w.run_semantic_exploration import _teardown_nav2_io

RUNNER = Path(__file__).resolve().parents[1] / "scripts/go2w/run_semantic_exploration.py"


class _Recorder:
    def __init__(self, log: list[str], name: str, *, raises: bool = False) -> None:
        self._log, self._name, self._raises = log, name, raises

    def _note(self) -> None:
        self._log.append(self._name)
        if self._raises:
            raise RuntimeError(self._name + " exploded")


class _Executor(_Recorder):
    def shutdown(self, timeout_sec: float = 0.0) -> None:
        self._note()


class _Thread(_Recorder):
    def join(self, timeout: float = 0.0) -> None:
        self._note()


class _Client(_Recorder):
    def destroy(self) -> None:
        self._note()


class _Node(_Recorder):
    def destroy_node(self) -> None:
        self._note()


class _Port:
    def __init__(self, client: _Client) -> None:
        self._client = client


def _node(log: list[str], *, shutdown_raises: bool = False, with_thread: bool = True,
          with_backend: bool = True, destroy_raises: bool = False) -> SimpleNamespace:
    node = SimpleNamespace(
        _nav2_io_executor=_Executor(log, "executor.shutdown", raises=shutdown_raises),
        _nav2_io_node=_Node(log, "io_node.destroy_node", raises=destroy_raises),
    )
    if with_thread:
        node._nav2_io_thread = _Thread(log, "thread.join")
    if with_backend:
        inner = SimpleNamespace(
            _path_client=_Port(_Client(log, "path_client.destroy")),
            _navigate_client=_Port(_Client(log, "navigate_client.destroy")),
            _spin_client=_Port(_Client(log, "spin_client.destroy")),
        )
        node._search_nav2_backend = SimpleNamespace(_inner=inner)
    return node


# --------------------------------------------------------------- 正常路径


def test_every_resource_is_released_in_dependency_order():
    log: list[str] = []
    node = _node(log)
    assert _teardown_nav2_io(node) == []
    # 顺序是有理由的：client 长在 io_node 上，必须先于它销毁（rclpy 的
    # destroy_node 不接管所有 ActionClient 实体，晚到的句柄会碰失效的 context）。
    assert log == ["executor.shutdown", "thread.join", "path_client.destroy",
                   "navigate_client.destroy", "spin_client.destroy",
                   "io_node.destroy_node"]


def test_missing_backend_still_releases_the_executor_and_node():
    """后端还没建出来（例如第 4047 行之后就抛了）也要能收。"""
    log: list[str] = []
    node = _node(log, with_backend=False)
    assert _teardown_nav2_io(node) == []
    assert log == ["executor.shutdown", "thread.join", "io_node.destroy_node"]


def test_partial_setup_without_thread_does_not_raise():
    """**这是原实现的真缺陷**：executor 挂上了、thread 还没建。

    ``run_go2w`` 里那四行是依次挂的（backend → io_node → executor → thread），
    中间任何一行失败都会留下这个半成品状态。原收尾写的是裸的
    ``node._nav2_io_thread.join(...)``，用的是"executor 在 ⇒ 后面几样都在"这个
    **没人保证**的不变量 —— 不成立时抛 AttributeError，而它正好发生在最该安静的
    那一段里。
    """
    log: list[str] = []
    node = _node(log, with_thread=False)
    assert _teardown_nav2_io(node) == []
    assert "executor.shutdown" in log and "io_node.destroy_node" in log


def test_nothing_set_up_is_a_no_op():
    node = SimpleNamespace()
    assert _teardown_nav2_io(node) == []


# --------------------------------------------------------------- 失败路径


def test_shutdown_failure_does_not_skip_the_remaining_cleanup():
    log: list[str] = []
    node = _node(log, shutdown_raises=True)
    assert _teardown_nav2_io(node) == ["io_executor.shutdown"]
    # 后面的样样做到：一处失败不该让别的资源泄漏。
    assert log == ["executor.shutdown", "thread.join", "path_client.destroy",
                   "navigate_client.destroy", "spin_client.destroy",
                   "io_node.destroy_node"]


def test_destroy_node_failure_is_reported_not_raised():
    log: list[str] = []
    node = _node(log, destroy_raises=True)
    assert _teardown_nav2_io(node) == ["io_node.destroy_node"]


def test_it_never_raises_even_when_everything_fails():
    log: list[str] = []
    node = _node(log, shutdown_raises=True, destroy_raises=True)
    # 不抛就够了 —— 抛出去会顶掉 _run_go2w_explorer 真正的返回码，并跳过
    # provider / manager / output / video / destroy_node 全部清理。
    assert _teardown_nav2_io(node) == ["io_executor.shutdown", "io_node.destroy_node"]


# --------------------------------------------------------------- 幂等


def test_second_call_is_a_no_op():
    log: list[str] = []
    node = _node(log)
    _teardown_nav2_io(node)
    before = list(log)
    assert _teardown_nav2_io(node) == []
    assert log == before, "第二次不该对着已拆掉的 executor / node 再拆一次"


def test_second_call_after_a_failed_first_call_is_still_a_no_op():
    """失败的那一次也把句柄摘掉了 —— 否则重入会拿半死的对象再拆一遍。"""
    log: list[str] = []
    node = _node(log, shutdown_raises=True)
    _teardown_nav2_io(node)
    before = list(log)
    assert _teardown_nav2_io(node) == []
    assert log == before


# --------------------------------------------------------------- 接线


def _has_attribute_chain(path: Path, first: str, second: str) -> bool:
    """源码里是否真的写着 ``<first>.<second>`` 这条属性链。

    按 token 判而不是按全文子串判：``_teardown_nav2_io`` 的注释里**故意**引用了
    旧写法来说明为什么要改，全文子串会把它当成"还没改"。
    """
    skip = (tokenize.COMMENT, tokenize.STRING, tokenize.NL, tokenize.NEWLINE,
            tokenize.INDENT, tokenize.DEDENT)
    with path.open("rb") as handle:
        tokens = [t for t in tokenize.tokenize(handle.readline) if t.type not in skip]
    return any(a.string == first and b.string == "." and c.string == second
               for a, b, c in zip(tokens, tokens[1:], tokens[2:]))


def test_the_finally_block_actually_calls_it():
    """收尾函数存在但没人调 = 没修。同时钉住裸属性链已经不在代码里。"""
    src = RUNNER.read_text(encoding="utf-8")
    assert "_teardown_nav2_io(node)" in src
    assert not _has_attribute_chain(RUNNER, "_search_nav2_backend", "_inner")
    assert not _has_attribute_chain(RUNNER, "_nav2_io_thread", "join")


def test_cli_sigint_keeps_ros_alive_until_disarm_cleanup() -> None:
    src = RUNNER.read_text(encoding="utf-8")
    # The cross-distro helper keeps Humble's NO option and Foxy's native
    # context-alive SIGINT behavior; its branches are exercised separately.
    assert "init_preserving_python_signals()" in src
    assert "except KeyboardInterrupt:" in src
    interrupt_block = src.split("except KeyboardInterrupt:", 1)[1].split(
        "except Exception as exc:", 1
    )[0]
    assert "node._arm(False)" in interrupt_block
    assert "active_backend.stop()" in interrupt_block
