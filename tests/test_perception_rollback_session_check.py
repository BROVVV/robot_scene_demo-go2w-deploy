"""`scripts/go2w/check_perception_rollback_session.sh` 的会话身份判据。

这个脚本是感知提速阶段一报告 §6.3 第二层交给操作员的核验步骤。它已经栽过两次，
两次都是**"工具跑通了，但回答的不是你要问的问题"**：

1. 第二次复审 F2：当前会话的事件流还没落盘时**默默退回历史会话**，于是拿旧实例
   的诊断当本次回退生效的证据，还以 0 退出。
2. 第三次复审：只看 `session_id` 不看 `status`。搜索结束后 WebUI 仍保留
   `session_id` 与终态 `status`（`current_session()` 只要 `_info` 不为 None 就
   一直返回），日志也还在 —— 于是一个 `OPERATOR_STOP` 的旧会话被当成"正在跑的
   实例"，报告里"核验跑着的实例"这句话就不成立。

所以这里每条判据都断言两件事：**退出码**，以及**有没有把别的会话/已结束的会话
的证据当成本次证据**。全程在 `tmp_path` 里跑：`curl` 用替身（不出网）、
会话目录现造（不读也不写真实会话）。
"""

from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CHECK = PROJECT_ROOT / "scripts" / "go2w" / "check_perception_rollback_session.sh"

# 跑着的状态（脚本的白名单）与终态，两边都从**代码里的真来源**取名字，
# 免得脚本改了白名单而测试还按旧名字编。
LIVE = ["STARTING", "RUNNING", "PAUSED"]
# app/manual_web_demo/search_session_archive.py: TERMINAL_STATUSES
TERMINAL = [
    "TARGET_FOUND",
    "SEARCH_EXHAUSTED",
    "FAILED",
    "OPERATOR_STOP",
    "FINISHED",
]


def _diag_event(frame_id: str, *, cache: bool = True, rle: str = "vector",
                backbone: int = 1, terms: int = 19) -> str:
    """一条真形状的 detector_diagnostics 事件（两次尝试，其中一条没回诊断）。"""
    return json.dumps(
        {
            "event": "detector_diagnostics",
            "frame_id": frame_id,
            "diagnostics_available": True,
            "diagnostics": {"frame_feature_cache": cache, "mask_rle_encoder": rle},
            "attempts": [
                {
                    "request_id": "r1",
                    "frame_id": frame_id,
                    "diagnostics": {
                        "frame_feature_cache": cache,
                        "mask_rle_encoder": rle,
                        "mask_rle_encoder_requested": rle,
                        "backbone_count": backbone,
                        "term_count": terms,
                    },
                },
                {"request_id": "r2", "frame_id": frame_id, "diagnostics": None},
            ],
        }
    )


def _session(root: Path, name: str, lines: list[str], mtime: float | None = None) -> Path:
    d = root / "outputs" / "live_runs" / name
    d.mkdir(parents=True, exist_ok=True)
    path = d / "events.jsonl"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    if mtime is not None:
        os.utime(path, (mtime, mtime))
    return path


def _session_without_log(root: Path, name: str) -> None:
    (root / "outputs" / "live_runs" / name).mkdir(parents=True, exist_ok=True)


@pytest.fixture
def fake_state(tmp_path: Path):
    """装一个只回 `/api/search/state` 内容的 curl 替身，不出网。"""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()

    def install(payload: dict | None) -> Path:
        script = bin_dir / "curl"
        if payload is None:
            script.write_text("#!/usr/bin/env bash\nexit 7\n", encoding="utf-8")
        else:
            body = json.dumps(payload)
            script.write_text(
                "#!/usr/bin/env bash\nprintf '%s\\n' " + json.dumps(body) + "\n",
                encoding="utf-8",
            )
        script.chmod(0o755)
        return bin_dir

    install.bin_dir = bin_dir  # type: ignore[attr-defined]
    return install


def _run(root: Path, bin_dir: Path, *args: str) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env["PERCEPTION_CHECK_ROOT"] = str(root)
    env["PATH"] = f"{bin_dir}{os.pathsep}{env.get('PATH', '')}"
    return subprocess.run(
        ["bash", str(CHECK), *args],
        capture_output=True,
        text=True,
        env=env,
        cwd=str(root),
    )


# --------------------------------------------------------------------------- #
# 1) 核验通过：会话在跑 + 有诊断
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("status", LIVE)
def test_a_live_session_with_diagnostics_passes(tmp_path, fake_state, status) -> None:
    bin_dir = fake_state({"session_id": "live_session", "status": status})
    _session(tmp_path, "live_session", [_diag_event("live_frame", cache=True)])

    result = _run(tmp_path, bin_dir)

    assert result.returncode == 0, result.stderr
    assert "live_session" in result.stdout
    assert status in result.stdout, "状态要显示出来，操作员才知道核验的是哪一轮"
    assert '"frame_id":"live_frame"' in result.stdout
    assert '"cache":true' in result.stdout and '"backbone":1' in result.stdout
    # diagnostics 为 null 的那次如实给 null，不拿上一条冒充
    assert '"r2"' in result.stdout


# --------------------------------------------------------------------------- #
# 2) 第三次复审点名的负向测试：会话已结束
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("status", TERMINAL)
def test_a_finished_session_is_not_mistaken_for_the_running_one(
    tmp_path, fake_state, status
) -> None:
    """**这条就是第三次复审复现出来的那一条。**

    搜索结束后 WebUI 仍留着 `session_id` 和终态 `status`，日志也还在。只看
    `session_id` 就会 `rc=0` 并把已结束会话的诊断当成"跑着的实例"的证据 ——
    那份诊断可能是回退**之前**写的，于是操作员以为回退生效了。
    """
    bin_dir = fake_state({"session_id": "done_session", "status": status})
    _session(tmp_path, "done_session", [_diag_event("stale_frame", cache=False, rle="python")])

    result = _run(tmp_path, bin_dir)

    assert result.returncode != 0, (
        f"status={status} 是终态，不该判为'正在跑的实例'（rc=0）\n{result.stdout}"
    )
    assert "stale_frame" not in result.stdout, "终态会话的旧诊断被当成本次证据了"
    assert "尚无法验证" in result.stderr
    assert status in result.stderr, "要说清是哪个状态让它不算数"


# --------------------------------------------------------------------------- #
# 3) 第二次复审点名的那条：当前会话日志未落盘，不得退回历史
# --------------------------------------------------------------------------- #
def test_a_session_without_a_log_yet_does_not_fall_back_to_history(
    tmp_path, fake_state
) -> None:
    bin_dir = fake_state({"session_id": "just_started", "status": "STARTING"})
    _session_without_log(tmp_path, "just_started")
    # 别的会话有诊断，且是"最近"的那个 —— 旧版脚本退回去正好读到它
    _session(tmp_path, "newest_other", [_diag_event("newest_frame")], mtime=2_000_000_000)
    _session(tmp_path, "older_other", [_diag_event("older_frame")], mtime=1_000_000_000)

    result = _run(tmp_path, bin_dir)

    assert result.returncode != 0, "日志还没落盘就说'核验通过'"
    for leaked in ("newest_frame", "older_frame"):
        assert leaked not in result.stdout, f"退回了历史会话，泄露了 {leaked}"
    assert "尚无法验证" in result.stderr
    assert "不会" in result.stderr and "历史" in result.stderr, "要说清它没有退回历史"


def test_a_live_session_without_any_diagnostic_event_is_not_a_pass(
    tmp_path, fake_state
) -> None:
    """有日志、但一次检测都没发生 —— `jq` 零行输出不能算通过。"""
    bin_dir = fake_state({"session_id": "quiet_session", "status": "RUNNING"})
    _session(tmp_path, "quiet_session", [json.dumps({"event": "search_step", "step": "f"})])

    result = _run(tmp_path, bin_dir)

    assert result.returncode != 0
    assert "尚无法验证" in result.stderr


def test_no_session_at_all_is_not_a_pass(tmp_path, fake_state) -> None:
    bin_dir = fake_state({"session_id": "", "status": "IDLE"})

    result = _run(tmp_path, bin_dir)

    assert result.returncode != 0
    assert "--history" in result.stderr


def test_an_unreachable_webui_is_not_a_pass(tmp_path, fake_state) -> None:
    """`curl` 失败（WebUI 没起）时不能静默当成"没有诊断"。"""
    bin_dir = fake_state(None)

    result = _run(tmp_path, bin_dir)

    assert result.returncode != 0
    assert "取不到" in result.stderr


# --------------------------------------------------------------------------- #
# 4) 历史模式：只在显式要求时才用，且要有真诊断
# --------------------------------------------------------------------------- #
def test_history_mode_is_explicit_and_needs_a_real_diagnostic(tmp_path, fake_state) -> None:
    bin_dir = fake_state({"session_id": "", "status": "IDLE"})
    _session(tmp_path, "live_one", [_diag_event("live_frame")], mtime=2_000_000_000)
    _session(tmp_path, "quiet_one", [json.dumps({"event": "search_step"})],
             mtime=3_000_000_000)  # 最近的那个反而没有诊断

    newest = _run(tmp_path, bin_dir, "--history", "1")
    assert newest.returncode != 0, "历史模式也要真读到诊断"

    second = _run(tmp_path, bin_dir, "--history", "2")
    assert second.returncode == 0, second.stderr
    assert "明确要求浏览历史" in second.stdout
    assert '"frame_id":"live_frame"' in second.stdout


# --------------------------------------------------------------------------- #
# 5) jq 过滤器对得上代码**真正**写出的事件形状
# --------------------------------------------------------------------------- #
def _jq_filter() -> str:
    """从被测脚本里抽出那段 jq（不重抄一份，否则两边会漂移）。

    按行抽：从 `JQ_FILTER='` 那一行到**行尾是 `'`** 的那一行。不能按
    "\n'" 找 —— 结束引号挂在最后一行末尾，不在行首（先前就是这么把脚本
    后半段一起吞进过滤器的）。
    """
    lines = CHECK.read_text(encoding="utf-8").splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith("JQ_FILTER='"))
    end = next(i for i in range(start, len(lines)) if lines[i].endswith("'"))
    body = "\n".join(lines[start : end + 1])
    body = body[len("JQ_FILTER='") : -1]
    assert "exit 0" not in body and "curl" not in body, (
        "抽出来的不是那段 jq，是脚本的后半段 —— 抽法要跟着改"
    )
    return body


def test_the_filter_matches_the_real_event_shape() -> None:
    """事件由真函数产出，不是手抄的 JSON。

    手写事件只能证明"过滤器不空转"，证明不了"事件真的是这个形状"。
    """
    from app.detectors.grounded_sam_subprocess import diagnostics_event_payload

    class Stub:
        def __init__(self, attempts, available, diagnostics):
            self.detect_attempts = attempts
            self.last_diagnostics_available = available
            self.last_diagnostics = diagnostics

    full = {
        "frame_feature_cache": True,
        "mask_rle_encoder": "vector",
        "mask_rle_encoder_requested": "vector",
        "backbone_count": 1,
        "term_count": 19,
    }
    with_retry = Stub(
        [
            {"request_id": "r1", "operation": "detect", "frame_id": "5465",
             "source_image": "x.jpg", "diagnostics": full},
            {"request_id": "r2", "operation": "detect", "frame_id": "5465",
             "source_image": "x.jpg", "diagnostics": None},
        ],
        True,
        full,
    )
    nothing = Stub([], False, None)          # 真函数此时返回 None → 整条事件都不写
    no_attempts = Stub([], True, dict(full, prompt_count=2, term_count=38))

    events = []
    for frame, stub in (("5465", with_retry), ("5466", nothing), ("5467", no_attempts)):
        payload = diagnostics_event_payload(stub, frame_id=frame)
        if payload is not None:
            events.append(json.dumps(payload))

    jq = subprocess.run(
        ["jq", "-c", "-R", _jq_filter()],
        input="\n".join(events) + "\n",
        capture_output=True,
        text=True,
        check=True,
    )
    out = jq.stdout

    assert '"frame_id":"5465"' in out
    assert '"cache":true' in out and '"rle":"vector"' in out
    assert '"requested":"vector"' in out
    assert '"backbone":1' in out and '"terms":19' in out
    assert '"r2"' in out, "重试那一次要单独一行"
    assert '"frame_id":"5467","available":true,"attempts":[]' in out
    assert '"frame_id":"5466"' not in out, (
        "真函数对'既没有 attempts 也没有诊断'的帧返回 None、整条不写；"
        "投影里出现 5466 说明形状理解错了"
    )
