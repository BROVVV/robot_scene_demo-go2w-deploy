"""本地（机器狗）VLM 复核 provider 的接线测试。

守住的是一个**旁路开关**：`GO2W_LOCAL_VLM_VERIFY` 打开后，verify 只打机器狗上的
llama-server，云端客户端与 `SILICONFLOW_BASE_URL` 一概不碰；关掉时云端行为
**逐字不变**。

这里只测**接线**（发了几张图、发的哪张、出错怎么走），不测本地 3B 判得准不准
—— 后者要真模型，见 `reports/lfm2_5_vl_3b_robot_handoff_20261012.md` 的 bench。

两条最容易写错、写错了还全绿的性质，单独钉：

1. **本地出错不许重试、不许吞**。云端路径有「多图被拒就退回单张整帧」的重试，
   那是为云端供应商写的；本地 3B 超时若照抄这条，会把一次超时变成两次调用，
   而交接书明确要求「不能把超时当作未发现目标」。所以断言的是 `create` 恰好被
   调一次，且异常原样抛出。
2. **关掉开关时云端行为不变**。少这条的话，一个「无论如何都走本地」的实现
   会让上面几条照常全绿。
"""

from __future__ import annotations

import json
from typing import Any

import pytest
from PIL import Image

from app.config import Settings
from app.detectors import siliconflow_vision_worker as worker

CROP_SENTINEL = "data:image/jpeg;base64,Q1JPUC1TRU5USU5FTA=="
TARGET_REPLY = json.dumps(
    {
        "object_name_zh": "绿色垃圾桶",
        "is_target": True,
        "confidence": 0.91,
        "reason_zh": "框内是绿色桶身，颜色相符",
    },
    ensure_ascii=False,
)


class _FakeResponse:
    def __init__(self, content: str) -> None:
        message = type("_Msg", (), {"content": content})()
        choice = type("_Choice", (), {"message": message})()
        self.choices = [choice]


class _FakeClient:
    """记录每次 `create` 的入参；可按调用次序逐次抛异常。"""

    def __init__(self, contents: list[str] | None = None,
                 errors: list[Exception | None] | None = None) -> None:
        self.calls: list[dict[str, Any]] = []
        self._contents = contents or [TARGET_REPLY]
        self._errors = errors or []
        self.chat = type("_Chat", (), {"completions": self})()

    def create(self, **kwargs: Any) -> _FakeResponse:
        index = len(self.calls)
        self.calls.append(kwargs)
        error = self._errors[index] if index < len(self._errors) else None
        if error is not None:
            raise error
        content = self._contents[min(index, len(self._contents) - 1)]
        return _FakeResponse(content)

    def image_parts(self, call_index: int = 0) -> list[dict]:
        parts = self.calls[call_index]["messages"][0]["content"]
        return [part for part in parts if part.get("type") == "image_url"]


def _settings(**overrides: Any) -> Settings:
    base: dict[str, Any] = {"siliconflow_api_key": "cloud-key"}
    base.update(overrides)
    return Settings(**base)


def _image(tmp_path) -> str:
    path = tmp_path / "frame.jpg"
    Image.new("RGB", (400, 300), (200, 200, 200)).save(path)
    return str(path)


@pytest.fixture(autouse=True)
def _stub_crop(monkeypatch: pytest.MonkeyPatch) -> None:
    """把裁剪图钉成哨兵串，好分辨发出去的是整帧还是裁剪图。

    裁剪**几何**不是这条测试的对象（`_make_crop_data_url` 另有覆盖）。
    """
    monkeypatch.setattr(worker, "_make_crop_data_url", lambda *a, **k: CROP_SENTINEL)


def _patch_openai(monkeypatch: pytest.MonkeyPatch, client: _FakeClient) -> list[dict]:
    """替换 `openai.OpenAI`，并记下每次构造用的参数。"""
    built: list[dict] = []

    def _factory(**kwargs: Any) -> _FakeClient:
        built.append(kwargs)
        return client

    monkeypatch.setattr("openai.OpenAI", _factory)
    return built


# --------------------------------------------------------------------------
# 开关本身
# --------------------------------------------------------------------------


def test_disabled_by_default_builds_no_local_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """默认关闭：连本地 client 都不许构造，且 provider 返回 None。

    断言"没构造"而不是只断言返回值：返回值对了但顺手建了个指向
    `127.0.0.1:18081` 的 client，在没有端口转发的工作站上会变成一次
    连接尝试 —— 关着的开关不该有任何副作用。
    """
    built = _patch_openai(monkeypatch, _FakeClient())

    assert worker._local_verify_provider(_settings()) is None
    assert built == []


def test_enabled_provider_points_at_the_robot_endpoint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """开启后用的是机器人地址/模型/超时，而不是 `SILICONFLOW_*`。"""
    built = _patch_openai(monkeypatch, _FakeClient())
    settings = _settings(
        siliconflow_base_url="https://api.siliconflow.cn/v1",
        local_vlm_verify_enabled=True,
        local_vlm_base_url="http://127.0.0.1:18081/v1",
        local_vlm_model="lfm-local",
        local_vlm_api_key="sk-local",
        local_vlm_timeout_seconds=123.0,
    )

    provider = worker._local_verify_provider(settings)

    assert provider is not None
    assert built == [
        {
            "api_key": "sk-local",
            "base_url": "http://127.0.0.1:18081/v1",
            "timeout": 123.0,
        }
    ], "本地 provider 必须自带 base_url，绝不能复用云端那个"
    assert provider[1] == "lfm-local"
    assert provider[2] is True and provider[3] is True


@pytest.mark.parametrize(
    ("mode", "expect_full", "expect_crop"),
    [
        ("full+crop", True, True),
        ("crop", False, True),
        # 拼错/大写/空串都退回与云端对齐的默认，不静默变成 crop-only
        ("CROP", False, True),
        ("bogus", True, True),
        ("", True, True),
    ],
)
def test_image_mode_parsing(
    monkeypatch: pytest.MonkeyPatch, mode: str, expect_full: bool, expect_crop: bool
) -> None:
    _patch_openai(monkeypatch, _FakeClient())
    settings = _settings(local_vlm_verify_enabled=True, local_vlm_verify_images=mode)

    provider = worker._local_verify_provider(settings)

    assert provider is not None
    assert (provider[2], provider[3]) == (expect_full, expect_crop)


# --------------------------------------------------------------------------
# 实际发出的图片
# --------------------------------------------------------------------------


def test_full_plus_crop_sends_two_images(monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
    client = _FakeClient()
    _patch_openai(monkeypatch, client)
    settings = _settings(
        local_vlm_verify_enabled=True, local_vlm_verify_images="full+crop"
    )

    worker._verify_detect(settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5")

    parts = client.image_parts()
    assert len(parts) == 2, f"full+crop 必须发两张图，实际 {len(parts)}"
    assert [part["image_url"]["url"] for part in parts][1] == CROP_SENTINEL
    assert client.calls[0]["model"] == "lfm-local"


def test_verify_prompt_carries_current_profile_without_relation_example_bias(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    client = _FakeClient()
    _patch_openai(monkeypatch, client)
    settings = _settings(local_vlm_verify_enabled=True)

    worker._verify_detect(
        settings,
        _image(tmp_path),
        "黑椅子",
        "0.1,0.1,0.3,0.5",
        extra_instructions='目标画像={"colors":["black"],"relation_constraints":[]}',
    )

    prompt = client.calls[0]["messages"][0]["content"][0]["text"]
    assert '"colors":["black"]' in prompt
    assert "没有关系约束时不得自行添加关系条件" in prompt
    assert "饮水机旁边" not in prompt


def test_crop_only_sends_exactly_the_crop(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """crop-only：**只有**裁剪图，一张都不许多。

    多带一张整帧就退化成 full+crop，而两种模式的耗时/准确率对比会失去意义。
    """
    client = _FakeClient()
    _patch_openai(monkeypatch, client)
    settings = _settings(local_vlm_verify_enabled=True, local_vlm_verify_images="crop")

    worker._verify_detect(settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5")

    parts = client.image_parts()
    assert len(parts) == 1, f"crop-only 只该发一张图，实际 {len(parts)}"
    assert parts[0]["image_url"]["url"] == CROP_SENTINEL, "发出去的必须是裁剪图"


def test_crop_only_falls_back_to_full_frame_when_crop_fails(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """裁剪图取不到时退回整帧 —— 但**不是**发出一个纯文本请求。"""
    monkeypatch.setattr(worker, "_make_crop_data_url", lambda *a, **k: None)
    client = _FakeClient()
    _patch_openai(monkeypatch, client)
    settings = _settings(local_vlm_verify_enabled=True, local_vlm_verify_images="crop")

    worker._verify_detect(settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5")

    assert len(client.image_parts()) == 1, "退化成纯文本请求的话模型无从判断"


# --------------------------------------------------------------------------
# 错误语义
# --------------------------------------------------------------------------


def test_local_timeout_propagates_and_is_never_retried(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """本地一次超时 = 一次调用 + 异常上抛，绝不能变成重试或"没目标"。

    这是整个开关的安全边界：上层 `_verify_target` 把异常翻成
    `PerceptionFailure(recoverable=True)`。若这里吞掉异常返回
    `is_target=False`，搜索会把"复核没做成"读成"这一帧没有目标"，
    等于**静默漏检**。
    """
    timeout = TimeoutError("local llama-server timed out")
    client = _FakeClient(errors=[timeout])
    _patch_openai(monkeypatch, client)
    settings = _settings(local_vlm_verify_enabled=True)

    with pytest.raises(TimeoutError):
        worker._verify_detect(settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5")

    assert len(client.calls) == 1, (
        f"云端的单帧重试不该套用到本地 provider，实际调用 {len(client.calls)} 次"
    )


def test_local_success_keeps_the_shared_return_shape(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """返回结构必须与云端逐字一致（下游只认这四个键）。"""
    client = _FakeClient()
    _patch_openai(monkeypatch, client)
    settings = _settings(local_vlm_verify_enabled=True)

    result = worker._verify_detect(
        settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5"
    )

    assert result == {
        "object_name_zh": "绿色垃圾桶",
        "is_target": True,
        "confidence": 0.91,
        "reason_zh": "框内是绿色桶身，颜色相符",
        # 第五个键是元数据（判定来源），跟着 payload 透传到
        # `target_verification` 事件，供混判追溯。下游读的是 .get()，
        # 多这一个键不影响它们。
        "_provider": "local",
    }


def test_cloud_path_still_retries_a_single_full_frame(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """**对照**：开关关着时，云端「多图被拒退回整帧」的重试照旧。

    没有这条，上面那条"本地不重试"可以由一个"谁都不重试"的实现满足 ——
    而那会悄悄改掉云端既有行为（供应商不支持多图时会直接失败而不是退化）。
    同时也钉住 `model_override` 仍然生效、`client` 参数仍被复用。
    """
    passed_client = _FakeClient(errors=[RuntimeError("multi-image rejected")])
    built = _patch_openai(monkeypatch, _FakeClient())  # 只用于断言"没被构造"
    settings = _settings()

    result = worker._verify_detect(
        settings,
        _image(tmp_path),
        "绿色垃圾桶",
        "0.1,0.1,0.3,0.5",
        model_override="Qwen/Qwen3-VL-8B-Instruct",
        client=passed_client,
    )

    assert built == [], "云端路径不该构造本地 client"
    assert len(passed_client.calls) == 2, "云端必须重试一次"
    assert len(passed_client.image_parts(0)) == 2, "第一次是两图"
    assert len(passed_client.image_parts(1)) == 1, "重试必须退回单张整帧"
    assert passed_client.calls[0]["model"] == "Qwen/Qwen3-VL-8B-Instruct"
    assert result["is_target"] is True


# --------------------------------------------------------------------------
# 云端备用路径（本地不可用时回落）
# --------------------------------------------------------------------------


def _httpx2_request():
    import httpx2

    return httpx2.Request("POST", "http://127.0.0.1:18081/v1/chat/completions")


def _connection_error():
    from openai import APIConnectionError

    return APIConnectionError(request=_httpx2_request())


def _timeout_error():
    from openai import APITimeoutError

    return APITimeoutError(request=_httpx2_request())


def _server_error():
    from openai import APIStatusError
    import httpx2

    request = _httpx2_request()
    return APIStatusError(
        "503 Service Unavailable",
        response=httpx2.Response(503, request=request),
        body=None,
    )


def test_local_connection_error_falls_back_to_cloud(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """本地端点没了（连接失败）→ 用云端那把备用钥匙再打一次，且能出结果。

    备用路径必须复用调用方传进来的客户端。断言这一点，是因为"临时新建一个
    指向 `settings.siliconflow_*` 的客户端"也能让这条测试通过，但那会在
    调用方已经持有客户端时白白多建一个连接池。
    """
    local = _FakeClient(errors=[_connection_error()])
    cloud = _FakeClient()
    _patch_openai(monkeypatch, local)
    settings = _settings(local_vlm_verify_enabled=True)

    result = worker._verify_detect(
        settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud
    )

    assert len(local.calls) == 1, "本地只该被试一次"
    assert len(cloud.calls) == 1, "备用路径该恰好被用一次"
    assert cloud.calls[0]["model"] == settings.vision_model, "备用走的是云端模型"
    assert result["is_target"] is True


def test_local_server_error_also_falls_back(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """本地 5xx 同样算"不可用"，也要走备用。"""
    local = _FakeClient(errors=[_server_error()])
    cloud = _FakeClient()
    _patch_openai(monkeypatch, local)
    settings = _settings(local_vlm_verify_enabled=True)

    result = worker._verify_detect(
        settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud
    )

    assert len(cloud.calls) == 1
    assert result["is_target"] is True


def test_local_timeout_never_falls_back(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """**超时不回落** —— 这条钉的是一个真实的类层次陷阱。

    openai 里 `APITimeoutError` 是 `APIConnectionError` 的**子类**，所以
    `except (APIConnectionError, ...)` 会把超时一起捕获。用真类（而不是
    `test_local_timeout_propagates_and_is_never_retried` 里那个裸
    `TimeoutError`）才能暴露这个顺序错误：本地被单槽排队拖过 45 s 闸门时，
    再打一次云端只会更晚，等于白等。
    """
    local = _FakeClient(errors=[_timeout_error()])
    cloud = _FakeClient()
    _patch_openai(monkeypatch, local)
    settings = _settings(local_vlm_verify_enabled=True)

    with pytest.raises(Exception) as excinfo:
        worker._verify_detect(
            settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud
        )

    assert type(excinfo.value).__name__ == "APITimeoutError", "超时必须原样上抛"
    assert len(local.calls) == 1
    assert cloud.calls == [], "超时绝不能触发云端备用"


def test_failure_after_fallback_still_propagates(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """本地不可用 + 云端也失败 ⇒ 异常上抛，绝不变成 `is_target=False`。

    这是启用备用路径之后新的安全边界：两条腿都断了的时候，上层必须收到
    `PerceptionFailure(recoverable=True)`，而不是"这一帧没有目标"。
    """
    local = _FakeClient(errors=[_connection_error()])
    cloud = _FakeClient(errors=[RuntimeError("cloud down"), RuntimeError("cloud down")])
    _patch_openai(monkeypatch, local)
    settings = _settings(local_vlm_verify_enabled=True)

    with pytest.raises(RuntimeError, match="cloud down"):
        worker._verify_detect(
            settings, _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud
        )

    assert len(cloud.calls) == 2, "云端仍该走它自己的单帧重试"


def test_provider_tag_marks_who_answered(
    monkeypatch: pytest.MonkeyPatch, tmp_path
) -> None:
    """三个分支各留下自己的来源标记 —— 这是"混判可追溯"的全部依据。

    没有这个标记，一次 run 里哪些帧由本地 3B 判、哪些由云端判就无从分辨，
    而两者的精度并不相同。
    """
    local = _FakeClient()
    cloud = _FakeClient()
    _patch_openai(monkeypatch, local)

    # 1) 本地判
    got = worker._verify_detect(
        _settings(local_vlm_verify_enabled=True),
        _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud,
    )
    assert got["_provider"] == "local"

    # 2) 关着开关 = 纯云端
    got = worker._verify_detect(
        _settings(), _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud,
    )
    assert got["_provider"] == "cloud"

    # 3) 本地挂了、云端接手 —— 这个标记最要紧：它说明这一帧的判定
    #    不是本地模型给的，精度不能按本地那套来读。
    local_down = _FakeClient(errors=[_connection_error()])
    _patch_openai(monkeypatch, local_down)
    got = worker._verify_detect(
        _settings(local_vlm_verify_enabled=True),
        _image(tmp_path), "绿色垃圾桶", "0.1,0.1,0.3,0.5", client=cloud,
    )
    assert got["_provider"] == "cloud-after-local-failure"
