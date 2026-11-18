"""§44.2 / §21：GroundingDINO+SAM2(+RAM++) 检测器必须**全程只开一个 worker 进程**。

计划书 §44.2 原文要求：

    fake worker 连续：tag / detect / tag / detect
    要求：Popen 仍然只有 1 个

**为什么这条值得单独立文件。** 模型加载是这个栈里最贵的一步（`grounded_sam_worker.py`
开头自陈："so a whole search session loads each model once"）。一旦
`_ensure_worker`（`app/detectors/grounded_sam_subprocess.py:144`）每次请求都重开，
表现**不是**报错、也不是变慢一点 —— 是每一帧都重新加载三个模型，语义循环从秒级
掉到分钟级，而所有日志都正常、`pub=1`、进程活着。这正是本仓库反复出现的那类
"静默失效"。

**这个文件守的是一对命题，缺一半就没有分辨力：**

1. worker **活着**时连发 4 次 → `Popen` 恰好 1 次；
2. worker **死了**之后再来一次 → `Popen` 变 2 次。

只测第 1 条的话，"`_ensure_worker` 无条件提前 return"（例如把 `poll()` 判定写反、
或干脆 `return` 不检查）也能让计数停在 1 —— 那是一个永远复用一个**死**进程的
实现，比每次重开更糟。第 2 条是第 1 条的对照。

**诚实边界**：`subprocess.Popen` 被替换成假进程，所以这里证明的是**客户端的
复用逻辑**，不证明真 worker 的模型真的只加载了一次（后者要在实机上看加载日志）。
假进程同时充当"单一 worker"的证人：4 条请求全部落在**同一个**假进程的 stdin 上，
这一条比 Popen 计数更强 —— 它排除了"计数是 1 但请求发给了两个进程"。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from app.config import Settings
from app.detectors import grounded_sam_subprocess as gss

IMAGE = Path(__file__).resolve().parents[1] / "docs" / "go2w" / "plain_slam_pandarxt16.md"


# --------------------------------------------------------------------------
# 假 worker：一个能被伪装成 Popen、收得下请求、答得出响应的对象
# --------------------------------------------------------------------------

class _FakeStdin:
    def __init__(self, requests: list[dict]) -> None:
        self._requests = requests
        self.closed = False

    def write(self, text: str) -> int:
        for line in text.splitlines():
            if line.strip():
                self._requests.append(json.loads(line))
        return len(text)

    def flush(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class _FakeStdout:
    """按预设顺序吐出响应行；给完了就返回 `""`（EOF）。"""

    def __init__(self, responses: list[dict]) -> None:
        self._lines = [json.dumps(r, ensure_ascii=False) + "\n" for r in responses]
        self._index = 0

    def readline(self) -> str:
        if self._index >= len(self._lines):
            return ""
        line = self._lines[self._index]
        self._index += 1
        return line


class _FakeWorker:
    """一个跑着的 worker。`poll()` 返回 `None` 表示活着。"""

    def __init__(self, responses: list[dict], requests: list[dict]) -> None:
        self.stdin = _FakeStdin(requests)
        self.stdout = _FakeStdout(responses)
        self.stderr = None
        self.terminate_calls = 0
        self._alive = True

    def poll(self):
        return None if self._alive else 0

    def die(self) -> None:
        """模拟 worker 崩了（模型 OOM、被 OOM killer 杀掉）。"""
        self._alive = False

    def terminate(self) -> None:
        self.terminate_calls += 1
        self._alive = False

    def kill(self) -> None:
        self._alive = False

    def wait(self, timeout: float | None = None) -> int:
        return 0


class _WorkerFarm:
    """替换 `subprocess.Popen`：记下每次调用，产出新的假 worker。"""

    def __init__(self, responses: list[dict]) -> None:
        self.responses = responses
        self.requests: list[dict] = []          # 全部请求，跨所有 worker
        self.workers: list[_FakeWorker] = []
        self.commands: list[list[str]] = []
        self.kwargs: list[dict] = []

    def __call__(self, command, **kwargs):
        self.commands.append(list(command))
        self.kwargs.append(kwargs)
        worker = _FakeWorker(list(self.responses), self.requests)
        self.workers.append(worker)
        return worker


# --------------------------------------------------------------------------
# 一条能跑通的最小请求：tag 返回 tags_en，detect 返回 objects
# --------------------------------------------------------------------------

TAG_RESPONSE = {"tags_en": ["chair", "table"]}
# 必须**非空** objects：`detect_with_dynamic_terms` 在空结果时会走 prompt 扩写与
# 高召回重试（各再发一次请求），那样序列就不是 tag/detect/tag/detect 了。
DETECT_RESPONSE = {
    "objects": [
        {"label": "chair", "box": [1.0, 2.0, 30.0, 40.0], "score": 0.8,
         "mask_area_ratio": 0.1, "mask_runs": []},
    ]
}


@pytest.fixture()
def farm(monkeypatch: pytest.MonkeyPatch) -> _WorkerFarm:
    farm = _WorkerFarm([TAG_RESPONSE, DETECT_RESPONSE] * 2)
    monkeypatch.setattr(gss.subprocess, "Popen", farm)
    return farm


def _detector() -> gss.GroundedSAMSubprocessDetector:
    return gss.GroundedSAMSubprocessDetector(Settings(siliconflow_api_key=""))


def _detect(detector) -> None:
    """一次 detect 操作。

    走 `detect_with_dynamic_terms` 而不是 `detect`：后者只是前者 `dynamic_terms=[]`
    的转发，而没有 target_profile / parsed_task 时 `_build_prompts` 会拼出空
    prompt，触发 `grounding_prompt_require_non_empty` 抛错（这是**另一条**配置
    契约，不是本文件要测的东西）。请求里的 `operation` 字段两者都是 `detect`，
    §44.2 要的序列不受影响。
    """
    detector.detect_with_dynamic_terms(str(IMAGE), "chair", ["chair"])


def _run_tag_detect_tag_detect(detector) -> None:
    """计划书 §44.2 给的那一串操作，一字不改。"""
    detector.discover_scene_terms(str(IMAGE))
    _detect(detector)
    detector.discover_scene_terms(str(IMAGE))
    _detect(detector)


def test_tag_detect_tag_detect_keeps_exactly_one_worker(farm: _WorkerFarm) -> None:
    """§44.2 的正面要求：4 次操作，`Popen` 恰好 1 次。"""
    _run_tag_detect_tag_detect(_detector())

    assert len(farm.workers) == 1, (
        f"开了 {len(farm.workers)} 个 worker —— 每帧都会重新加载三个模型"
    )


def test_all_four_requests_land_on_that_one_worker(farm: _WorkerFarm) -> None:
    """比计数更强的一条：4 条请求必须**都落在同一个进程的 stdin 上**。

    计数为 1 但请求发给了两个不同对象（比如 `_worker` 被重新赋值而旧进程没关）
    是可能的，而且后果与重开一样糟 —— 旧进程会变成孤儿继续占显存。
    """
    _run_tag_detect_tag_detect(_detector())

    assert [r["operation"] for r in farm.requests] == ["tag", "detect", "tag", "detect"]
    assert farm.workers[0].stdin.closed is False, "全程不许把在用的 worker 关掉"


def test_the_second_tag_reuses_the_process_started_by_the_first(farm: _WorkerFarm) -> None:
    """两次 tag 之间没有新进程 —— 单独拎出来，因为 RAM++ 是三个模型里最重的一个。"""
    detector = _detector()
    detector.discover_scene_terms(str(IMAGE))
    detector.discover_scene_terms(str(IMAGE))

    assert len(farm.workers) == 1
    assert [r["operation"] for r in farm.requests] == ["tag", "tag"]


# --------------------------------------------------------------------------
# 对照：worker 死了必须重开（否则上面的"1"没有分辨力）
# --------------------------------------------------------------------------

def test_a_dead_worker_is_replaced_instead_of_reused(farm: _WorkerFarm) -> None:
    """worker 崩了之后，下一次请求必须**重开**一个。

    这是上一条的对照面。没有它，"永远复用第一个进程"（哪怕它已经死了）这种实现
    也能让计数停在 1 —— 而那种实现的症状是：worker 一崩，之后每一次请求都写到
    一个已死的 stdin 上，`readline()` 立刻拿到 `""`，于是每帧都抛
    `DetectorRuntimeError`，整场搜索报废。
    """
    detector = _detector()
    detector.discover_scene_terms(str(IMAGE))
    assert len(farm.workers) == 1

    farm.workers[0].die()

    detector.discover_scene_terms(str(IMAGE))
    assert len(farm.workers) == 2, "worker 死了必须重开，不许复用一个死进程"
    assert [r["operation"] for r in farm.requests] == ["tag", "tag"]


def test_close_releases_the_worker_so_the_next_request_spawns_a_new_one(
    farm: _WorkerFarm,
) -> None:
    """`close()` 之后再来一次请求 ⇒ 新开一个；且 `close()` 本身必须幂等。

    **幂等那半条才是这条的可观测面。** `close()` 先 `self._worker = None` 再关，
    所以第二次调用直接 `return`。若去掉那句置空，第二次 `close()` 会拿着**同一个
    已关对象**再 `terminate()` 一次 —— 而 `poll()` 早已非 `None`，所以"下次请求
    会重开"这件事**照样成立**，光看进程计数分不出这两种实现。分得出的是
    `terminate` 被调了几次。

    在真机上这一条对应的是：WebUI 停止搜索时连调两次 `close()`（页面关闭 +
    worker 退出各一次），第二次去 `close()` 一个已经 `stdin.close()` 过的管道、
    再 `terminate()` 一个已回收的 pid —— 后者在 pid 被系统复用时**会杀错进程**。
    """
    detector = _detector()
    detector.discover_scene_terms(str(IMAGE))
    worker = farm.workers[0]

    detector.close()
    assert worker.terminate_calls == 1, "close() 必须真的终止子进程"
    assert worker.stdin.closed is True

    detector.close()  # 幂等：worker 已是 None，必须直接返回
    assert worker.terminate_calls == 1, "close() 被调了两次，第二次不该再动子进程"

    detector.discover_scene_terms(str(IMAGE))
    assert len(farm.workers) == 2, "close() 之后必须能重新拉起来"


# --------------------------------------------------------------------------
# 命令行契约：只开一个进程的前提是这条命令拼得对
# --------------------------------------------------------------------------

def test_the_worker_command_targets_the_serve_entrypoint_not_a_one_shot(
    farm: _WorkerFarm,
) -> None:
    """命令里必须是 `--serve`，且必须带上三个模型各自的路径。

    少了 `--serve`，worker 会按"跑一次就退出"的模式启动 —— 进程立刻结束、
    `poll()` 不再返回 `None`，于是**每一次**请求都重开一个进程。这就是
    "Popen 只有 1 个"在真机上退化成"每帧重开"的路径，而计数断言在假进程下
    看不出来，所以在这里把命令行本身钉住。
    """
    _detector().discover_scene_terms(str(IMAGE))

    (command,) = farm.commands
    assert command[0], "python 解释器路径不许为空"
    assert command[1].endswith("grounded_sam_worker.py")
    assert "--serve" in command
    # 三个模型各要一对 (config, checkpoint)/root，缺一个就是那个模型不加载。
    for flag in ("--grounding-config", "--grounding-checkpoint",
                 "--sam2-config", "--sam2-checkpoint", "--root"):
        assert flag in command, f"命令行缺 {flag}"
        assert command[command.index(flag) + 1].strip(), f"{flag} 后面是空值"


def test_the_settings_pythonpath_is_prepended_not_replaced(
    farm: _WorkerFarm, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """`grounded_sam_pythonpath` 必须**前置**到 PYTHONPATH，不许覆盖。

    覆盖会把调用方（WebUI worker）自己需要的路径挤掉 —— 那正是 memory 里
    `go2w-webui-worker-pythonnousersite-breaks-search` 那一类事故的形状。
    这条在真机上的表现是"搜索起来 0.5 秒就挂"，与 worker 复用毫无关系，
    却发生在同一个函数里。
    """
    monkeypatch.setenv("PYTHONPATH", "/existing/path")
    settings = Settings(siliconflow_api_key="", grounded_sam_pythonpath="/gs/path")
    gss.GroundedSAMSubprocessDetector(settings).discover_scene_terms(str(IMAGE))

    (env,) = (kwargs["env"] for kwargs in farm.kwargs)
    assert env["PYTHONPATH"] == "/gs/path:/existing/path", "必须前置而不是覆盖"
    assert env["PYTHONPATH"].endswith("/existing/path"), "调用方原有的路径被挤掉了"


def test_without_a_configured_pythonpath_the_caller_env_is_untouched(
    farm: _WorkerFarm, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """没配 `grounded_sam_pythonpath` 时，`PYTHONPATH` 必须原样传递。

    「前置」与「什么都不做」是两条不同的路径，只测前者的话，一个无条件写
    `env["PYTHONPATH"] = "" + ":" + existing` 的实现会留下一个**前导冒号** ——
    空字符串在 `PYTHONPATH` 里表示"当前目录"，于是 worker 的 import 会先搜
    cwd。这是能静默改变 import 结果的形状。
    """
    monkeypatch.setenv("PYTHONPATH", "/existing/path")
    gss.GroundedSAMSubprocessDetector(
        Settings(siliconflow_api_key="", grounded_sam_pythonpath="")
    ).discover_scene_terms(str(IMAGE))

    (env,) = (kwargs["env"] for kwargs in farm.kwargs)
    assert env["PYTHONPATH"] == "/existing/path"
