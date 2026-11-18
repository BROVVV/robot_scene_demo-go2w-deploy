"""生产入口 `run_go2w` 的接线作用域核查。

背景（2026-09-17 对抗性复核 F6-5）：把 task / 知识 / 只读拓扑摘要送进推理上下文时，
`run_go2w` 里被写成了 `task_context=task_context`，而 `task_context` 既不是该函数的
形参、也不是模块级名字。Python 在求值实参时就抛 `NameError`，被紧邻的
`except Exception` 吞掉、只往 ROS 日志里写一行，最终以 `return 4` 收场。

后果不是"接错了线"，而是**生产入口每次调用都失败**：
WebUI → `autonomous_search_worker.py` → `rse.run_go2w(args, hook)` 是原样透传的，
所以在改动之后、任何一次真机搜索都进不到 `_run_go2w_explorer`。
它不报错、不崩溃、只返回一个和"传感器/API 故障"长得一样的退出码。

这条测试钉的是**故障类别**，不是某一行的拼写：函数体里凡是被读取、却既不是局部绑定、
也不是模块级名字的标识符，都算未定义名。计划书 §1「先建立有分辨力的测试」。
"""

import ast
import builtins
import pathlib

import pytest

SOURCE = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "go2w" / "run_semantic_exploration.py"


def _parse() -> ast.Module:
    return ast.parse(SOURCE.read_text(encoding="utf-8"))


def _module_level_names(tree: ast.Module) -> set[str]:
    """模块层绑定过的名字：import / def / class / 赋值 / 带星号 import 之外的 ImportFrom。"""
    names: set[str] = set()
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                names.add(alias.asname or alias.name.split(".")[0])
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                for sub in ast.walk(target):
                    if isinstance(sub, ast.Name):
                        names.add(sub.id)
    return names


def _bound_inside(func: ast.AST) -> set[str]:
    """函数体内任何层级绑定过的名字（含嵌套 def / 推导式 / with / except）。

    这是**过近似**：嵌套作用域里的绑定也被算进来，因此可能漏报真正的作用域错误，
    但不会误报。`task_context` 这个类别的故障在这个过近似下依然会被抓到。
    """
    bound: set[str] = set()
    for node in ast.walk(func):
        if isinstance(node, ast.arg):
            bound.add(node.arg)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bound.add(node.name)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
            bound.add(node.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                bound.add(alias.asname or alias.name.split(".")[0])
        elif isinstance(node, ast.ExceptHandler) and node.name:
            bound.add(node.name)
    return bound


def _undefined_names(func_name: str) -> list[str]:
    tree = _parse()
    func = next(
        (n for n in ast.walk(tree)
         if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == func_name),
        None,
    )
    assert func is not None, f"{SOURCE.name} 里找不到函数 {func_name}"
    known = _module_level_names(tree) | _bound_inside(func) | set(dir(builtins))
    loaded = {
        n.id for n in ast.walk(func)
        if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
    }
    return sorted(loaded - known)


def test_run_go2w_reads_no_name_that_is_not_in_scope():
    """`run_go2w` 不得读取任何未定义的名字。

    这正是 2026-09-17 那次故障的判据：`task_context=task_context` 会在求值实参时
    raise NameError，而它被 `except Exception` 吞成 return 4，看起来像传感器故障。
    """
    undefined = _undefined_names("run_go2w")
    assert undefined == [], (
        f"run_go2w 读取了未定义的名字 {undefined} —— 生产入口每次调用都会抛 NameError，"
        "被 except 吞掉后以 return 4 收场（表现为一条普通的启动失败，没有 traceback 到调用方）"
    )


def test_live_explorer_closures_read_no_unbound_names():
    # This is a separate function scope: a run_go2w local is not visible to
    # spatial_candidate_generator inside _run_go2w_explorer.
    assert _undefined_names('_run_go2w_explorer') == []


def test_run_go2w_passes_a_task_context_into_the_explorer():
    """`run_go2w` 必须把 task 上下文交给 `_run_go2w_explorer`（计划书 §4.1）。

    反方向也钉住：如果为了绕开上面那条 NameError 而把实参整个删掉，
    `_run_go2w_explorer` 的形参退回默认 `None`，§4.1 的装配会静默失效 ——
    比抛异常更糟，因为它不留痕迹。
    """
    tree = _parse()
    func = next(
        n for n in ast.walk(tree)
        if isinstance(n, ast.FunctionDef) and n.name == "run_go2w"
    )
    calls = [
        n for n in ast.walk(func)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "_run_go2w_explorer"
    ]
    assert len(calls) == 1, f"run_go2w 里对 _run_go2w_explorer 的调用有 {len(calls)} 处，预期 1 处"
    keywords = {kw.arg for kw in calls[0].keywords}
    assert "task_context" in keywords, (
        "run_go2w 没把 task_context 传给 _run_go2w_explorer —— "
        "§4.1 的任务/知识/拓扑装配会在生产路径上静默失效"
    )
    value = next(kw.value for kw in calls[0].keywords if kw.arg == "task_context")
    assert not (isinstance(value, ast.Name) and value.id == "task_context"), (
        "又写回了 `task_context=task_context` —— 这个名字在 run_go2w 作用域里不存在"
    )


def _find_func(tree: ast.Module, name: str, *, within: str | None = None) -> ast.AST:
    """按名字取函数定义。`within` 给定时只在该函数的子树里找。

    这个文件里有**两个** `matcher`（`run_replay` 里一个、`_run_go2w_explorer` 里
    一个），所以必须限定作用域 —— 否则会钉到离线回放那个身上，看着是绿的实际
    什么也没管。
    """
    scope: ast.AST = _find_func(tree, within) if within else tree
    func = next(
        (n for n in ast.walk(scope)
         if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == name),
        None,
    )
    assert func is not None, f"{SOURCE.name} 里找不到函数 {name}" + (
        f"（在 {within} 内）" if within else ""
    )
    return func


def test_topology_summary_is_recomputed_per_frame_from_the_live_graph():
    """§18.4：拓扑摘要必须在 `matcher` **内**逐帧算，且读的是活的 `place_graph`。

    运行时没有症状，所以只能靠结构钉：`place_graph` 要到 `explorer.run()` 之后才
    开始装观测，在闭包外只算一次就会把一张刚 new 出来的空图冻结成整个会话的
    "当前拓扑"（面板上恒显示"地点 0"，而离线评估器读的是收尾的终局图）；
    而 `place_graph.to_dict()` 会把整个会话的观测序列化一遍，逐帧调用是 O(n²)。
    """
    tree = _parse()
    matcher = _find_func(tree, "matcher", within="_run_go2w_explorer")
    calls = [
        n for n in ast.walk(matcher)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name) and n.func.id == "summarize_topology"
    ]
    assert calls, (
        "matcher 里没有调用 summarize_topology —— 拓扑摘要在闭包外只算了一次，"
        "会把刚 new 出来的空图冻结成整个会话的当前拓扑"
    )
    for call in calls:
        args = list(call.args) + [kw.value for kw in call.keywords]
        rendered = [ast.unparse(arg) for arg in args]
        assert "place_graph" in rendered, (
            f"matcher 里的 summarize_topology 读的不是 place_graph，而是 {rendered}"
        )
        for arg in args:
            assert not isinstance(arg, ast.Call), (
                f"matcher 里的 summarize_topology 传的是 {ast.unparse(arg)} —— "
                "live 分支要走 O(1) 的 places/edges，不许 to_dict()（逐帧就是 O(n²)）"
            )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
