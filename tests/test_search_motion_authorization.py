"""计划书 §P5-4：只读服务必须拒绝"请求 JSON 擅自 enable motion"。

P5 第 4 条原文：

    注入"请求 JSON 擅自 enable motion"必须在只读服务中拒绝；无启动权限时不能
    触发 arm/NavigateToPose，更不能触急停或误停别人的控制。

实现落在 `SearchSessionService.start_search` 的一处三条件判断上：

    if (request.backend == "nav2" and not request.dry_run_motion
            and not self._executor_factory_is_mock
            and os.getenv("GO2W_NAV2_EXECUTION_AUTHORIZED") != "1"):
        return {"ok": False, "error": "Nav2 运动尚未由启动流程授权…", "forbidden": True}

**这一条原来没有任何用例**：别的搜索用例要么用带 `is_mock_factory` 的工厂（于是这道门
被 `not self._executor_factory_is_mock` 跳过），要么像
`test_search_session_recovery.py` 那样显式 `setenv(..., "1")` 绕开它。也就是说
**把这道门整个删掉，现有用例照样全绿** —— 这正是"阴性结论必须先做注入校准"说的
那种状态。本文件就是给它配的正反两刀。

三件互相独立、都不能省的事：

1. **前提**：`configs/go2w/autonomous_search_web.yaml` 里
   `enable_autonomous_motion: true`，所以**光看配置文件，一个什么都不带的 WebUI
   请求就是一个运动请求**。只读档位靠的是启动脚本导出的
   `AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION=0`（见
   `start_autonomous_search_web.sh:463`），不是配置文件。
2. **请求体自己说了不算**：即便 body 里写 `enable_autonomous_motion: true`、
   `dry_run_motion: false`，甚至塞一个同名的环境变量键，也不能把门打开 ——
   门读的是**进程环境**，而那个环境由启动脚本给。
3. **正控制**：同一个运动请求在 `GO2W_NAV2_EXECUTION_AUTHORIZED=1` 下**不再**
   被拒绝。没有这一条，"拒绝"也可能只是"什么都拒绝"（比如 `validate()` 先一步
   把请求毙了），那就证明不了门在工作。

不碰真机、不碰网络：executor 是桩，任务理解走本地 `mock_fallback`。
"""
from __future__ import annotations

import pytest

from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_models import SearchStartRequest
from app.manual_web_demo.search_session_service import SearchSessionService
from app.task_understanding.search_task_context import SearchTaskContext

READ_ONLY_REFUSAL = "只读验证"


class _IdleExecutor:
    """什么都不做的 executor：本文件测的是**授权门**，不是执行。"""

    def set_on_message(self, cb) -> None: ...
    def start(self, params: dict) -> None: ...
    def pause(self) -> None: ...
    def resume(self) -> None: ...
    def stop(self) -> None: ...
    def estop(self) -> None: ...
    def status(self) -> dict: return {"state": "stopped", "session_id": None}
    def alive(self) -> bool: return True
    def shutdown(self) -> None: ...


def _service(session_dir) -> SearchSessionService:
    """一个**没被打上 mock 标记**的桩工厂。

    这是本文件的关键手法：`_executor_factory_is_mock` 只认工厂对象上的
    `is_mock_factory` 属性，所以这里刻意不加 —— 那道授权门于是**真的被求值**，
    而产出的仍是不会起子进程的桩。用例里再显式断言这个标记是 False，
    免得哪天实现改成"按 backend 判"，用例就悄悄退化成"门被跳过"的假绿。
    """
    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=lambda: _IdleExecutor(),
        session_dir=str(session_dir),
        task_understanding_runner=lambda text: SearchTaskContext.mock_fallback(text),
    )
    assert service._executor_factory_is_mock is False, "桩工厂被打上了 mock 标记，这道门会被跳过"
    return service


def _read_only_launcher(monkeypatch):
    """启动脚本在**不给** `--enable-autonomous-motion` 时导出的那几个变量。"""
    monkeypatch.setenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", "0")
    monkeypatch.setenv("AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED", "0")
    monkeypatch.setenv("GO2W_NAV2_EXECUTION_AUTHORIZED", "0")


@pytest.mark.parametrize("body", [
    # 明确要求运动。
    {"task_text": "找到垃圾桶", "backend": "nav2",
     "enable_autonomous_motion": True, "dry_run_motion": False},
    # 只给 operator_supervised_experiment 也够把 dry_run 推成 False（见 from_dict）。
    {"task_text": "找到垃圾桶", "backend": "nav2",
     "operator_supervised_experiment": True, "dry_run_motion": False},
    # 两个都写。
    {"task_text": "找到垃圾桶", "backend": "nav2",
     "enable_autonomous_motion": True, "operator_supervised_experiment": True},
])
def test_read_only_launcher_refuses_a_motion_request(monkeypatch, tmp_path, body):
    _read_only_launcher(monkeypatch)
    request = SearchStartRequest.from_dict(body)
    assert request.dry_run_motion is False, "这条请求本身就不是 dry-run，下面的拒绝才有意义"
    result = _service(tmp_path).start_search(request)
    assert result["ok"] is False and result["forbidden"] is True, result
    assert READ_ONLY_REFUSAL in result["error"], result


def test_a_motion_default_coming_from_the_config_is_refused_too(monkeypatch, tmp_path):
    """启动脚本那两条 export 万一没了，门还在。

    这时候请求是 WebUI 真正发的那两个字段（`search_ui.js` 只发 task_text/target），
    运动档位全部来自配置文件 —— 一个字节的"擅自"都谈不上，可它就是个运动请求。
    受理它等于把 WebUI 变成运动入口，所以这里必须仍然是拒绝。
    """
    monkeypatch.delenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", raising=False)
    monkeypatch.delenv("AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED", raising=False)
    monkeypatch.setenv("GO2W_NAV2_EXECUTION_AUTHORIZED", "0")
    request = SearchStartRequest.from_dict(
        {"task_text": "找到垃圾桶", "target": "垃圾桶", "backend": "nav2"})
    assert request.dry_run_motion is False
    result = _service(tmp_path).start_search(request)
    assert result["ok"] is False and result["forbidden"] is True, result


def test_the_config_file_alone_is_not_a_read_only_profile(monkeypatch, tmp_path):
    """前提用例：只读**不是**配置文件的属性，而是启动脚本的属性。

    `configs/go2w/autonomous_search_web.yaml` 写着 `enable_autonomous_motion: true`。
    把一个什么都不带的请求按"没有环境覆盖"来解析，它就是个运动请求。所以
    "WebUI 只读"这句话成立的前提，是启动脚本导出了
    `AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION=0` —— 一旦哪天有人把那个 export
    删掉（或改了变量名），这份配置会立刻把 WebUI 变成运动入口，而配置文件本身
    看起来毫无变化。这条用例就是那个删改的报警器。
    """
    monkeypatch.delenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", raising=False)
    monkeypatch.delenv("AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED", raising=False)
    bare = SearchStartRequest.from_dict({"task_text": "找到垃圾桶", "backend": "nav2"})
    assert bare.dry_run_motion is False
    # 启动脚本一说话，同一个请求体就变成只读的 —— 请求体一个字节都没变。
    _read_only_launcher(monkeypatch)
    assert SearchStartRequest.from_dict(
        {"task_text": "找到垃圾桶", "backend": "nav2"}).dry_run_motion is True
    assert _service(tmp_path).start_search(
        SearchStartRequest.from_dict({"task_text": "找到垃圾桶", "backend": "nav2"})
    )["ok"] is True


def test_the_request_body_cannot_open_the_gate_by_itself(monkeypatch, tmp_path):
    """请求体里能写的全写上，包括一个同名的环境变量键 —— 门归然不动。"""
    _read_only_launcher(monkeypatch)
    service = _service(tmp_path)
    result = service.start_search(SearchStartRequest.from_dict({
        "task_text": "找到垃圾桶", "backend": "nav2",
        "enable_autonomous_motion": True, "operator_supervised_experiment": True,
        "dry_run_motion": False, "allow_degraded": True,
        # 请求体里塞进程环境变量名。它只会被 from_dict 当作未知字段丢掉；
        # 这条断言防的是"哪天有人图省事，改成从 body 里读这个键"。
        "GO2W_NAV2_EXECUTION_AUTHORIZED": "1",
    }))
    assert result["ok"] is False and result["forbidden"] is True, result


def test_launcher_opt_in_is_what_opens_the_gate(monkeypatch, tmp_path):
    """正控制：同一个运动请求，只把**启动脚本那一路**的开关打开，就不再被拒。

    没有这条，"拒绝"可能只是"这个请求怎么都被拒"（比如 `validate()` 先毙了），
    那就证明不了这道授权门在做任何区分。
    """
    _read_only_launcher(monkeypatch)
    monkeypatch.setenv("AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION", "1")
    monkeypatch.setenv("AUTONOMOUS_SEARCH_OPERATOR_SUPERVISED", "1")
    monkeypatch.setenv("GO2W_NAV2_EXECUTION_AUTHORIZED", "1")
    request = SearchStartRequest.from_dict({
        "task_text": "找到垃圾桶", "backend": "nav2",
        "enable_autonomous_motion": True,
    })
    assert request.dry_run_motion is False
    result = _service(tmp_path).start_search(request)
    assert result.get("forbidden") is not True, result
    assert result["ok"] is True, result
