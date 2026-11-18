"""感知提速计划（2026-10-13）阶段一：**不换模型**的两项等价优化。

计划书 §4 的两件必做项：

1. **同帧 DINO 图像特征复用** —— `predict()` 每次都是
   `model(image[None], captions=[caption])`，而 GroundingDINO 的 forward
   `unset_image_tensor` 默认 True，每个词跑完就把 `self.features` 删掉；本仓库
   逐词推理，19 个词就是 19 次 backbone。实测约 0.894 s
   （`reports/perception_profile_results_20261013.md`）。
2. **等价向量化 RLE** —— `_encode_mask_runs()` 的逐像素 while 循环换成 NumPy
   边界提取；实测 384 ms → 32 ms。

**这个文件守的判据是"等价"，不是"更快"。** 提速本身就存在于实现里，跑一遍
真实回放就能看见；真正会悄悄坏掉的是输出：错位的 mask、少一个词的框、变了形的
RLE。所以这里用**真的生产 `_detect()`**（torch 用最小替身注入，见 `_FakeTorch`）
跑出两套对象，逐字段、逐字节比较。

**诚实边界**：这里证明的是**装配逻辑**的等价 —— 同一个 `_detect`、同一份词表、
同一组框与掩码下，开/关两个开关产出完全相同的对象 JSON。它**不**证明真实
GroundingDINO/SAM2 权重下也等价（那是 `artifacts/perception_optimization/` 里
真实模型回放的活）。替身模型的 `forward` 刻意照抄了
`external/Grounded-SAM-2/.../groundingdino.py:303` 与 `:362-364` 的
`features` 存在性判断与 `unset_image_tensor` 分支，否则测的就不是同一件事。
"""

from __future__ import annotations

import contextlib
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from app.config import (
    DEFAULT_DETECTION_PROFILE_ENABLED,
    DEFAULT_ENABLE_DINO_FRAME_FEATURE_CACHE,
    DEFAULT_MASK_RLE_ENCODER,
    Settings,
    get_settings,
)
from app.detectors import grounded_sam_worker as worker

# ---------------------------------------------------------------------------
# torch 的最小替身
# ---------------------------------------------------------------------------
# `_detect()` 用到的 torch 面很窄：张量乘法/切片/比较、nonzero/argsort、
# empty/cat/tensor、inference_mode。.venv 里没有 torch（侧车跑在另一个解释器），
# 所以这里注入一个替身，让**生产装配逻辑本身**能在任何环境里被测到 ——
# 而不是把整段逻辑改成"跑不了就 skip"。


class _T:
    """`_detect` 触到的 tensor 表面。`.detach().cpu().numpy()` 链路必须真能走通，
    因为生产代码就是这么取值的。"""

    def __init__(self, value) -> None:
        self.v = np.asarray(value)

    def __array__(self, dtype=None):
        return self.v.astype(dtype) if dtype is not None else self.v

    def _raw(self, other):
        return other.v if isinstance(other, _T) else other

    def __len__(self) -> int:
        return len(self.v)

    def __getitem__(self, key):
        return _T(self.v[self._raw(key)])

    def __mul__(self, other):
        return _T(self.v * self._raw(other))

    __rmul__ = __mul__

    def __sub__(self, other):
        return _T(self.v - self._raw(other))

    def __truediv__(self, other):
        return _T(self.v / self._raw(other))

    def __lt__(self, other):
        return _T(self.v < self._raw(other))

    def __gt__(self, other):
        return _T(self.v > self._raw(other))

    def __and__(self, other):
        return _T(np.logical_and(self.v, self._raw(other)))

    def flatten(self):
        return _T(self.v.flatten())

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.v

    def tolist(self):
        return self.v.tolist()


class _FakeTorch:
    class cuda:
        @staticmethod
        def synchronize() -> None:
            pass

    @staticmethod
    def empty(shape):
        return _T(np.zeros(shape))

    @staticmethod
    def tensor(values):
        return _T(np.asarray(values, dtype=float))

    @staticmethod
    def cat(items, dim=0):
        return _T(np.concatenate([item.v for item in items], axis=dim))

    @staticmethod
    def nonzero(mask):
        return _T(np.nonzero(mask.v)[0])

    @staticmethod
    def argsort(values, descending=False):
        order = np.argsort(values.v)
        return _T(order[::-1] if descending else order)

    @staticmethod
    @contextlib.contextmanager
    def inference_mode():
        yield


# ---------------------------------------------------------------------------
# 替身模型：照抄真 GroundingDINO 的 features / unset_image_tensor 分支
# ---------------------------------------------------------------------------

class _FakeBackbone:
    def __init__(self) -> None:
        self.calls = 0

    def __call__(self, samples):
        return self.forward(samples)

    def forward(self, samples):
        self.calls += 1
        return ["levels"]


class _FakeGroundingDINO:
    """只保留本阶段关心的行为：features 存在就不重跑 backbone。"""

    def __init__(self) -> None:
        self.backbone = _FakeBackbone()
        self.unset_flags: list[bool] = []

    def __call__(self, samples, **kw):
        # nn.Module.__call__ 就是这样解析 `self.forward` 的 —— 实例字典里
        # 打了补丁，`model(...)` 也必须走到补丁那一层，否则 `_DinoFrameContext`
        # 在真模型上生效、在替身上不生效，测出来的东西就不是同一件事。
        return self.forward(samples, **kw)

    def forward(self, samples, targets=None, **kw):
        unset = kw.get("unset_image_tensor", True)
        self.unset_flags.append(bool(unset))
        if not hasattr(self, "features"):
            # 真模型 `set_image_tensor()` 干的就是这一句（groundingdino.py:209）。
            self.backbone(samples)
            self.features = ["levels"]
        if unset:
            self.unset_image_tensor()
        return {"pred_logits": None}

    def unset_image_tensor(self) -> None:
        self.__dict__.pop("features", None)


# ---------------------------------------------------------------------------
# 一份确定性的假检测结果：术语 → 框/分数
# ---------------------------------------------------------------------------

_HEIGHT, _WIDTH = 40, 60
_TERM_BOXES = {
    "trash can": [(0.30, 0.50, 0.20, 0.40, 0.91)],
    "green trash bin": [(0.32, 0.52, 0.18, 0.36, 0.88)],
    "chair": [(0.70, 0.60, 0.30, 0.50, 0.83), (0.20, 0.30, 0.10, 0.10, 0.21)],
    "table": [(0.50, 0.80, 0.40, 0.20, 0.77)],
}


def _make_predict(model):
    def predict(*, model, image, caption, box_threshold, text_threshold, device):
        # 真 `predict()` 就是这一句：模型自己决定要不要重算 backbone。
        model(image, captions=[caption])
        rows = _TERM_BOXES.get(caption.lower(), [])
        boxes = _T(np.array([[r[0], r[1], r[2], r[3]] for r in rows], dtype=float)
                   if rows else np.zeros((0, 4)))
        scores = _T(np.array([r[4] for r in rows], dtype=float)
                    if rows else np.zeros((0,)))
        return boxes, scores, []

    return predict


def _fake_box_convert(*, boxes, in_fmt, out_fmt):
    assert (in_fmt, out_fmt) == ("cxcywh", "xyxy")
    cx, cy, w, h = boxes.v[:, 0], boxes.v[:, 1], boxes.v[:, 2], boxes.v[:, 3]
    return _T(np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1))


def _fake_nms(boxes, scores, iou_threshold=0.5):
    del iou_threshold
    return _T(np.arange(len(boxes.v)))


class _FakeSAM2:
    """给每个框一个确定的掩码，形状取奇数尺寸以便踩到行尾边界。"""

    def __init__(self) -> None:
        self.set_image_calls = 0

    def set_image(self, image_source):
        self.set_image_calls += 1

    def predict(self, *, point_coords, point_labels, box, multimask_output):
        masks = []
        for index in range(len(np.asarray(box))):
            mask = np.zeros((_HEIGHT, _WIDTH), dtype=bool)
            # 每个框的掩码形状不同：整行 / 单像素 / 断成两段 / 空
            if index % 4 == 0:
                mask[5] = True
            elif index % 4 == 1:
                mask[7, 9] = True
            elif index % 4 == 2:
                mask[11, 2:6] = True
                mask[11, 20:23] = True
            masks.append(mask)
        return np.asarray(masks), None, None


def _detect(keep_features: bool, rle_encoder: str, terms: list[str],
            *, profile: bool = False):
    """跑一次真的 `_detect()`，返回 (objects, stats, model, sam)。"""
    model = _FakeGroundingDINO()
    sam = _FakeSAM2()
    stats: dict = {}
    objects = worker._detect(
        SimpleNamespace(),
        "frame.jpg",
        " || ".join(terms),
        0.12,
        0.10,
        100,
        0.35,
        model,
        sam,
        "cpu",
        _FakeTorch,
        np,
        lambda path: (np.zeros((_HEIGHT, _WIDTH, 3), dtype=np.uint8), _T(np.zeros((3, _HEIGHT, _WIDTH)))),
        _make_predict(model),
        _fake_box_convert,
        _fake_nms,
        options={
            "frame_feature_cache": keep_features,
            "mask_rle_encoder": rle_encoder,
            "profile": profile,
            "request_id": "test-1",
        },
        stats=stats,
    )
    return objects, stats, model, sam


# ===========================================================================
# 1. RLE：逐元素、逐类型等价
# ===========================================================================

def _rle_cases() -> list[np.ndarray]:
    rng = np.random.default_rng(20261013)
    cases = [
        np.zeros((5, 9), dtype=bool),          # 全空
        np.ones((5, 9), dtype=bool),           # 全满
        np.zeros((0, 0), dtype=bool),          # 退化尺寸
        np.zeros((0, 4), dtype=bool),
        np.zeros((4, 0), dtype=bool),
        np.ones((1, 1), dtype=bool),           # 单像素
    ]
    single = np.zeros((3, 7), dtype=bool)
    single[1, 3] = True
    cases.append(single)
    edges = np.zeros((2, 6), dtype=bool)       # 首列 + 末列 + 整行
    edges[0, 0] = True
    edges[1, 5] = True
    edges[1, 0:6] = True
    cases.append(edges)
    gaps = np.zeros((3, 10), dtype=bool)       # 行内多段（非连续）
    gaps[1, 0:2] = True
    gaps[1, 4:5] = True
    gaps[1, 8:10] = True
    cases.append(gaps)
    multi = np.zeros((9, 13), dtype=bool)      # 多连通区域
    multi[1:4, 1:4] = True
    multi[1:4, 8:11] = True
    multi[6:9, 4:7] = True
    cases.append(multi)
    for _ in range(30):
        cases.append(rng.random((rng.integers(1, 25), rng.integers(1, 30))) > 0.6)
    return cases


def test_vector_rle_matches_the_python_loop_element_for_element() -> None:
    """固定输入上新旧实现必须逐元素相同 —— 这是"等价"的字面意思。"""
    for mask in _rle_cases():
        assert worker._encode_mask_runs_vector(mask) == worker._encode_mask_runs_python(mask), (
            f"mask shape {mask.shape} 上新旧 RLE 不一致"
        )


def test_rle_output_is_json_serializable_python_ints() -> None:
    """JSON 契约：`[[row, start, length], ...]`，且必须是 Python `int`。

    `np.int64` 在 `json.dumps` 下会抛 `TypeError: Object of type int64 is not
    JSON serializable` —— 而这条链是把结果写进 JSONL 给下游用的。`.tolist()`
    正是为此，这条断言把它钉住。
    """
    mask = np.zeros((6, 8), dtype=bool)
    mask[0, 0:3] = True
    mask[5, 7] = True
    for encoder in ("python", "vector"):
        runs = worker._encode_mask_runs(mask, encoder=encoder)
        assert runs == [[0, 0, 3], [5, 7, 1]]
        assert all(type(value) is int for run in runs for value in run), (
            f"{encoder} 编出来的 run 里混进了非 Python int"
        )
        assert json.loads(json.dumps(runs)) == runs


def test_rle_encoder_switch_selects_the_implementation() -> None:
    """关掉开关必须真的走回旧实现，否则"可回退"是句空话。"""
    mask = np.zeros((4, 5), dtype=bool)
    mask[2, 1:4] = True
    assert worker._encode_mask_runs(mask, encoder="python") == worker._encode_mask_runs_python(mask)
    assert worker._encode_mask_runs(mask, encoder="vector") == worker._encode_mask_runs_vector(mask)
    assert worker._encode_mask_runs(mask) == worker._encode_mask_runs_vector(mask), (
        "默认实现必须是向量化那条"
    )


def test_non_binary_masks_follow_the_old_truthiness_semantics() -> None:
    """非 0/1 输入必须按**真值**处理，而不是按 `== 1`。

    旧实现判的是真值（`while row[x]`），所以 2 也算前景。向量化版本若直接
    `astype(np.int8)`，`changes == ±1` 之外的跳变会凭空消失 —— **不报错**，
    只是把掩码少画一块。这条把两种语义钉死在一起。
    """
    for raw in (
        np.array([[0, 2, 2, 0]], dtype=np.int8),
        np.array([[0, 1, 2, 0]], dtype=np.int8),
        np.array([[0.0, 0.5, 0.5, 0.0]]),
        np.array([[False, True, True, False]]),
    ):
        assert worker._encode_mask_runs_vector(raw) == worker._encode_mask_runs_python(raw), (
            f"{raw.tolist()} 上新旧语义分叉了"
        )


def test_a_three_dimensional_mask_is_rejected_instead_of_silently_wrong() -> None:
    """三维输入在旧实现里判的是"一行是不是数组"，结果不可预期；这里直接拒绝。"""
    with pytest.raises(ValueError, match="2-D"):
        worker._encode_mask_runs_vector(np.zeros((2, 3, 4), dtype=bool))


# ===========================================================================
# 2. 同帧特征复用：backbone 的次数才是判据
# ===========================================================================

def test_one_backbone_encode_per_frame_regardless_of_term_count() -> None:
    """开着复用：一帧 4 个词，backbone 恰好 1 次。"""
    terms = ["trash can", "green trash bin", "chair", "table"]
    _, stats, model, _ = _detect(True, "vector", terms)

    assert model.backbone.calls == 1, (
        f"{len(terms)} 个词跑了 {model.backbone.calls} 次 backbone —— 复用没生效"
    )
    assert stats["backbone_count"] == 1
    assert stats["term_count"] == 4
    assert stats["dino_forward_count"] == 4, "复用的是图像特征，不是文本前向"
    assert stats["frame_feature_cache"] is True


def test_without_the_cache_every_term_recomputes_the_backbone() -> None:
    """关着复用：同样的 4 个词 ⇒ 4 次 backbone（旧行为）。

    这是上一条的对照面。只测"1 次"的话，一个**从来不跑 backbone** 的实现
    （例如 forward 被换成了空函数）也能让计数停在 1，而那样一个框都出不来。
    """
    terms = ["trash can", "green trash bin", "chair", "table"]
    objects, stats, model, _ = _detect(False, "vector", terms)

    assert model.backbone.calls == 4, "关掉缓存后每个词都该重算一次 backbone"
    assert stats["backbone_count"] == 4
    assert objects, "关掉缓存也必须照样出框"
    # 复用与否，forward 收到的 unset 语义必须不同 —— 这是两条路径真的分开了的证据
    assert set(model.unset_flags) == {True}


class _SizedFakeSAM2:
    """按**这一帧自己的尺寸**给空掩码。

    尺寸跟着帧走，是为了让断言只盯一件事：跨帧的 bbox 是不是真的来自这一帧的
    图像。若掩码固定成某个尺寸，`refined_box` 会把 bbox 挪到掩码的坐标系里，
    那又是另一件事了。
    """

    def __init__(self) -> None:
        self.set_image_calls = 0
        self.shapes: list[tuple[int, int]] = []

    def set_image(self, image_source) -> None:
        self.set_image_calls += 1
        self.shapes.append(tuple(np.asarray(image_source).shape[:2]))

    def predict(self, *, point_coords, point_labels, box, multimask_output):
        height, width = self.shapes[-1]
        return np.zeros((len(np.asarray(box)), height, width), dtype=bool), None, None


def _detect_frame(model, sam, term: str, *, marker: float,
                  shape: tuple[int, int], explode: bool = False):
    """用**给定的模型实例**跑一帧真的 `_detect()`。

    与 `_detect()` 的区别只有两点，都是为了 R4：模型由调用方传进来（这样才能
    验证同一个常驻模型跨帧的行为），以及**预测结果由图像内容决定** —— 假
    `predict` 读一遍图像张量再据此给框，于是"这一帧的框"和"这一帧的图"绑在
    一起：谁把上一帧的特征或图像漏给了下一帧，框就会跟着错。
    """
    height, width = shape
    stats: dict = {}
    sensor = np.zeros((height, width, 3), dtype=np.uint8)

    def load_image(_path):
        return sensor, _T(np.full((3, height, width), float(marker)))

    def predict(*, model, image, caption, box_threshold, text_threshold, device):
        model(image, captions=[caption])
        if explode:
            raise RuntimeError("dino blew up mid-frame")
        value = float(np.asarray(image).mean())
        return (_T(np.array([[0.20 + 0.01 * value, 0.5, 0.3, 0.3]])),
                _T(np.array([0.9])), [])

    objects = worker._detect(
        SimpleNamespace(), "frame.jpg", term, 0.12, 0.10, 100, 0.35,
        model, sam, "cpu", _FakeTorch, np,
        load_image, predict, _fake_box_convert, _fake_nms,
        options={"frame_feature_cache": True, "mask_rle_encoder": "vector"},
        stats=stats,
    )
    return objects, stats


def test_a_new_frame_is_encoded_again_with_the_same_resident_model() -> None:
    """跨帧必须重新编码：复用旧特征＝把这一帧的框画到上一张图上。

    独立审查 R4：原来这条跑两次 `_detect()`，而那个辅助函数**每次都新建一个
    `_FakeGroundingDINO`** —— 两个模型各编码一次，恰好都等于 1，看着像通过，
    其实证不了生产中**同一个常驻模型**从图 A 切到图 B 时有没有清掉旧特征。

    改成：一个模型实例，连续跑三帧**尺寸与内容都不同**的图。两个断言各堵一条
    路 —— `backbone.calls` 堵"没重新编码"，bbox 堵"输出没跟着图像走"。
    """
    model = _FakeGroundingDINO()
    sam = _SizedFakeSAM2()
    frames = [
        (1.0, (40, 60)),   # 图 A
        (5.0, (64, 48)),   # 图 B：内容不同、尺寸不同
        (9.0, (31, 17)),   # 图 C：再来一张更小的
    ]

    boxes = []
    for marker, shape in frames:
        objects, stats = _detect_frame(model, sam, "trash can",
                                       marker=marker, shape=shape)
        assert stats["backbone_count"] == 1, "同一帧内只该编码一次"
        assert len(objects) == 1
        boxes.append(tuple(objects[0]["bbox_2d"]))

    assert model.backbone.calls == len(frames), (
        f"三帧只编码了 {model.backbone.calls} 次 —— 特征被跨帧留下了"
    )
    assert len(set(boxes)) == len(frames), (
        f"三帧的框一模一样 {boxes} —— 输出没跟着各自那一帧的图像走"
    )
    assert sam.set_image_calls == len(frames), "每帧都该重新 set_image"


def test_an_exception_mid_frame_leaves_the_model_clean_for_the_next_frame() -> None:
    """一帧推理中途抛错后：补丁完整还原、特征清干净、下一个正常帧照跑。

    R4 点名的第二个缺口。异常路径的清理写错时**不会报错** —— 错的是下一帧：
    它要么复用上一帧的特征，要么跑在被我们替换过的 `forward` 上。
    """
    model = _FakeGroundingDINO()
    sam = _SizedFakeSAM2()

    with pytest.raises(RuntimeError):
        _detect_frame(model, sam, "trash can", marker=1.0, shape=(40, 60),
                      explode=True)

    # 我们只在实例字典上打过补丁；退出上下文必须原样清掉（不是留个绑定方法）。
    assert "forward" not in model.__dict__, "model.forward 补丁没还原"
    assert "forward" not in model.backbone.__dict__, "backbone.forward 补丁没还原"
    assert not hasattr(model, "features"), "异常后特征没清，下一帧会拿旧特征配新图"

    before = model.backbone.calls
    objects, stats = _detect_frame(model, sam, "trash can", marker=3.0,
                                   shape=(40, 60))
    assert model.backbone.calls == before + 1, "异常之后的正常帧没有重新编码"
    assert stats["backbone_count"] == 1
    assert len(objects) == 1
    # cx = 0.20 + 0.01*3 = 0.23，w = 0.30 ⇒ bbox 的 x 是 [0.08, 0.38]
    assert objects[0]["bbox_2d"] == pytest.approx([0.08, 0.35, 0.38, 0.65])


def test_features_are_dropped_when_the_frame_ends_even_on_error() -> None:
    """正常结束、异常、空结果三条路都要把特征丢掉。"""
    model = _FakeGroundingDINO()
    with worker._DinoFrameContext(model, keep_features=True):
        model("img", captions=["x"])
        assert hasattr(model, "features")
    assert not hasattr(model, "features"), "正常结束没清特征"

    model = _FakeGroundingDINO()
    with pytest.raises(RuntimeError):
        with worker._DinoFrameContext(model, keep_features=True):
            model("img", captions=["x"])
            assert hasattr(model, "features")
            raise RuntimeError("boom")
    assert not hasattr(model, "features"), "异常路径没清特征"

    model = _FakeGroundingDINO()
    with worker._DinoFrameContext(model, keep_features=True):
        pass  # 一个词都没跑（空 prompt）
    assert not hasattr(model, "features")


def test_the_context_restores_whatever_forward_it_found() -> None:
    """实例上原本就有 `forward` 时必须还原成那一个，而不是删掉。

    有些路径会往实例字典里放 `forward`（torch.compile 的 optimize 包装、
    或本仓库既有的 monkeypatch）。直接 `del` 会把那一层悄悄拆掉。
    """
    model = _FakeGroundingDINO()
    sentinel = lambda *a, **kw: "sentinel"  # noqa: E731
    model.__dict__["forward"] = sentinel
    with worker._DinoFrameContext(model, keep_features=True):
        assert model.forward is not sentinel
    assert model.forward is sentinel, "还原错了：原有的实例 forward 被丢掉了"

    plain = _FakeGroundingDINO()
    assert "forward" not in plain.__dict__
    with worker._DinoFrameContext(plain, keep_features=True):
        pass
    assert "forward" not in plain.__dict__, "不该给没打补丁的模型留下实例属性"


# ===========================================================================
# 3. 端到端装配等价：新旧两条路的对象 JSON 逐字节相同
# ===========================================================================

def test_baseline_and_optimized_produce_byte_identical_objects() -> None:
    """同一个 `_detect`、同一份词表、同一组框与掩码 ⇒ 对象 JSON 完全相同。

    这是阶段一验收标准里"新旧路径在固定输入上对象顺序、标签、分数、bbox、
    mask RLE 一致"那一条。用 SHA-256 比整份 JSON，而不是只比字段个数。
    """
    terms = ["trash can", "green trash bin", "chair", "table", "bottle"]
    baseline, _, _, _ = _detect(False, "python", terms)
    optimized, _, _, _ = _detect(True, "vector", terms)

    dump = lambda objs: json.dumps({"objects": objs}, ensure_ascii=False, sort_keys=False)  # noqa: E731
    assert baseline, "基线没出任何对象 —— 这条断言没有分辨力"
    assert dump(baseline) == dump(optimized)
    assert [o["label"] for o in baseline] == [o["label"] for o in optimized]
    assert [o["mask_rle"] for o in baseline] == [o["mask_rle"] for o in optimized]
    assert any(o["mask_rle"] for o in baseline), "一个掩码都没有，RLE 那条没被覆盖到"


def test_masks_cover_the_shapes_that_break_naive_encoders() -> None:
    """掩码必须真的走到那几种边界形状，否则上面的等价断言是空的。"""
    terms = ["chair"]  # 两个框：一个 0.83（过 0.35 门槛），一个 0.21（不过）
    objects, stats, _, sam = _detect(True, "vector", terms)

    assert sam.set_image_calls == 1, "同一帧的 SAM2 图像编码必须只有一次"
    assert stats["mask_count"] == 1, "只有过门槛的那个框该有掩码"
    assert objects[1]["mask_rle"] is None, "没过门槛的框不该有掩码，但必须保留 2D 框"
    assert objects[1]["bbox_2d"] != objects[0]["bbox_2d"]

    runs = objects[0]["mask_rle"]
    assert runs == [[5, 0, _WIDTH]], f"整行掩码编错了：{runs}"


# ===========================================================================
# 4. 配置与接线：开关的**生效值**可查
# ===========================================================================

def test_the_optimizations_default_to_on() -> None:
    settings = Settings(siliconflow_api_key="")
    assert settings.enable_dino_frame_feature_cache is DEFAULT_ENABLE_DINO_FRAME_FEATURE_CACHE is True
    assert settings.mask_rle_encoder == DEFAULT_MASK_RLE_ENCODER == "vector"
    assert settings.detection_profile_enabled is DEFAULT_DETECTION_PROFILE_ENABLED is False, (
        "分项计时默认必须是关的：它会给每个阶段插 CUDA 同步，改变生产耗时结构"
    )


def test_env_overrides_reach_the_settings(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ENABLE_DINO_FRAME_FEATURE_CACHE", "false")
    monkeypatch.setenv("MASK_RLE_ENCODER", "python")
    monkeypatch.setenv("DETECTION_PROFILE_ENABLED", "true")
    get_settings.cache_clear()
    try:
        settings = get_settings()
        assert settings.enable_dino_frame_feature_cache is False
        assert settings.mask_rle_encoder == "python"
        assert settings.detection_profile_enabled is True
    finally:
        get_settings.cache_clear()


def test_the_request_carries_the_switches_and_the_worker_echoes_the_effective_values() -> None:
    """`settings → 请求体 → worker` 三段都要通，回填的生效值才算数。

    只断言 settings 里那个值是不够的 —— 本仓库的老毛病就是"配了、名字对、
    也不报错，只是从没生效"。这里用真 `Settings` 的默认值走一遍请求。
    """
    from app.config import Settings
    from app.detectors import grounded_sam_subprocess as gss

    settings = Settings(
        siliconflow_api_key="",
        grounded_sam_root="/tmp",
        grounded_sam_python="python",
        mask_rle_encoder="python",
        enable_dino_frame_feature_cache=False,
        detection_profile_enabled=True,
    )
    detector = gss.GroundedSAMSubprocessDetector(settings)
    options = worker._detect_options(
        SimpleNamespace(
            dino_frame_feature_cache=settings.enable_dino_frame_feature_cache,
            mask_rle_encoder=settings.mask_rle_encoder,
            profile_detection=settings.detection_profile_enabled,
        ),
        {
            "dino_frame_feature_cache": settings.enable_dino_frame_feature_cache,
            "mask_rle_encoder": settings.mask_rle_encoder,
            "profile_detection": settings.detection_profile_enabled,
            "request_id": "abc-7",
        },
    )
    assert options == {
        "frame_feature_cache": False,
        "mask_rle_encoder": "python",
        "mask_rle_encoder_requested": "python",
        "profile": True,
        "request_id": "abc-7",
        "frame_id": "",  # 这次请求没带源帧号（R2：由调用点显式传，worker 不猜）
        # §5.1.6 的多词分组开关。这条请求没带该键、args 上也没有 ⇒ 生效值
        # 是默认 `per_term`，请求值记成空串（"没人要求过"与"要求了 per_term"
        # 必须分得开，见 `test_an_unknown_prompt_mode_falls_back_and_says_so`）。
        "prompt_mode": "per_term",
        "prompt_mode_requested": "",
    }
    assert detector.last_diagnostics == {}, "还没有跑过检测，不该有生效值"


def test_an_unknown_rle_encoder_falls_back_and_says_so() -> None:
    """未知的编码器名字不能"跑的是 A、日志记的是 B"（审查的低优先级项）。

    `--mask-rle-encoder` 命令行有 choices 挡着，但请求体/env 没有：手打一个
    `MASK_RLE_ENCODER=numpy`，实现会静默走向量版，而 diagnostics 里仍然写着
    `numpy`。现在解析一次、两个字段都回填 —— 对不上就是配置写错了。
    """
    options = worker._detect_options(
        SimpleNamespace(dino_frame_feature_cache=True, mask_rle_encoder="vector",
                        profile_detection=False),
        {"mask_rle_encoder": "numpy", "frame_id": "42"},
    )
    assert options["mask_rle_encoder"] == "vector", "未知取值必须收敛到实际跑的实现"
    assert options["mask_rle_encoder_requested"] == "numpy", "原始请求值不能被抹掉"
    assert options["frame_id"] == "42"

    # 走一遍 `_detect` 的装配：生效值要落进 stats，且真的用它编码。
    _, stats, _, _ = _detect(True, "numpy", ["trash can"])
    assert stats["mask_rle_encoder"] == "vector"
    assert stats["mask_rle_encoder_requested"] == "numpy"
    assert json.loads(json.dumps(stats)) == stats


def test_request_values_win_over_the_command_line_defaults() -> None:
    """`--serve` 路径上请求体优先。命令行那份是够不着的死路径。"""
    args = SimpleNamespace(
        dino_frame_feature_cache=True, mask_rle_encoder="vector", profile_detection=False
    )
    options = worker._detect_options(
        args, {"dino_frame_feature_cache": False, "mask_rle_encoder": "python"}
    )
    assert options["frame_feature_cache"] is False
    assert options["mask_rle_encoder"] == "python"
    assert options["profile"] is False


def test_a_string_false_in_the_request_does_not_turn_the_cache_back_on() -> None:
    """请求体是 JSON，但键是调用方手写的：`"false"` 在 `if value:` 下是真值。

    把它当真值，症状就是"我明明关了、它还在复用" —— 而且不报错。
    """
    args = SimpleNamespace(
        dino_frame_feature_cache=True, mask_rle_encoder="vector", profile_detection=False
    )
    for raw in ("false", "False", "0", "no", "off", ""):
        assert worker._detect_options(args, {"dino_frame_feature_cache": raw})[
            "frame_feature_cache"
        ] is False, f"{raw!r} 没有被当成 False"
    assert worker._detect_options(args, {"dino_frame_feature_cache": "true"})[
        "frame_feature_cache"
    ] is True


def test_the_subprocess_sends_the_switches_in_every_request() -> None:
    """客户端源码契约：开关必须出现在请求体里（而不是命令行）。"""
    from pathlib import Path

    src = (
        Path(__file__).resolve().parents[1]
        / "app" / "detectors" / "grounded_sam_subprocess.py"
    ).read_text(encoding="utf-8")
    assert '"dino_frame_feature_cache": self.settings.enable_dino_frame_feature_cache,' in src
    assert '"mask_rle_encoder": self.settings.mask_rle_encoder,' in src
    assert '"dino_prompt_mode": self.settings.go2w_dino_prompt_mode,' in src, (
        "§5.1.6 的分组开关没进请求体 —— 那就只能在命令行上，"
        "而 `--serve` 路径够不着命令行那个值"
    )
    assert '"profile_detection": self.settings.detection_profile_enabled,' in src
    assert '"request_id"' in src, "没有请求 ID，同一帧的多条记录分不出谁是谁"


def test_the_production_assembly_wires_both_switches() -> None:
    """`_detect` 的装配点：帧上下文包住逐词推理，RLE 走开关选实现。"""
    from pathlib import Path

    src = (
        Path(__file__).resolve().parents[1]
        / "app" / "detectors" / "grounded_sam_worker.py"
    ).read_text(encoding="utf-8")
    body = src[src.index("def _detect("):src.index("def _finish_stats(")]
    assert "_DinoFrameContext(" in body, "逐词推理没被帧级特征上下文包住"
    assert "keep_features=frame_feature_cache" in body
    assert "_encode_mask_runs(mask, encoder=mask_rle_encoder)" in body, (
        "RLE 没有走开关 —— 关掉开关不会回到旧实现"
    )
    # 上下文必须包住 `_predict_all_prompts`，否则等于没包
    assert body.index("with _DinoFrameContext(") < body.index("_predict_all_prompts(")


def test_diagnostics_counts_are_internally_consistent() -> None:
    """词数 == 前向次数；backbone 次数 ≤ 前向次数。三者不等就说明调度被改过。"""
    terms = ["trash can", "chair", "table"]
    for keep in (True, False):
        _, stats, _, _ = _detect(keep, "vector", terms)
        assert stats["term_count"] == stats["dino_forward_count"] == len(terms)
        assert 1 <= stats["backbone_count"] <= stats["dino_forward_count"]
        # 测试替身用 " || " 拼词表（见 `_detect` helper），所以每个词一个槽位。
        assert stats["prompt_count"] == len(terms)
        assert stats["detector_backend"] == "grounding_dino_sam2"
        assert stats["object_count"] == len(_detect(keep, "vector", terms)[0])
        if keep:
            assert stats["backbone_count"] == 1
        else:
            assert stats["backbone_count"] == len(terms)


def test_prompt_and_term_counts_follow_the_real_two_slot_prompt() -> None:
    """真实 prompt 是 `"目标词. 目标词. ... || RAM词. RAM词. ..."`。

    "词数"和"槽位数"是两个不同的量（阶段二要按槽位分频），计数必须分开且都对。
    """
    model = _FakeGroundingDINO()
    stats: dict = {}
    worker._detect(
        SimpleNamespace(),
        "frame.jpg",
        "trash can. green trash bin || chair. table. bottle",
        0.12, 0.10, 100, 0.35,
        model, _FakeSAM2(), "cpu", _FakeTorch, np,
        lambda path: (np.zeros((_HEIGHT, _WIDTH, 3), dtype=np.uint8), _T(np.zeros((3, _HEIGHT, _WIDTH)))),
        _make_predict(model), _fake_box_convert, _fake_nms,
        options={"frame_feature_cache": True, "mask_rle_encoder": "vector"},
        stats=stats,
    )
    assert stats["prompt_count"] == 2
    assert stats["term_count"] == 5
    assert stats["dino_forward_count"] == 5
    assert stats["backbone_count"] == 1


def test_profiling_is_opt_in_and_only_then_are_stage_timings_recorded() -> None:
    """分项耗时只在 profiling 下产生 —— 生产路径不能被测量本身改变。"""
    terms = ["trash can", "chair"]
    _, off, _, _ = _detect(True, "vector", terms, profile=False)
    assert "stage_ms" not in off, "没开 profiling 却记了分项耗时"
    assert off["profile"] is False

    _, on, _, _ = _detect(True, "vector", terms, profile=True)
    assert on["profile"] is True
    for stage in ("preprocess", "dino", "sam2", "mask_rle", "total"):
        assert stage in on["stage_ms"], f"profiling 下缺少 {stage} 分项"
        assert on["stage_ms"][stage] >= 0


def test_diagnostics_survive_json_round_trip() -> None:
    """诊断块要能进 JSONL：不能让 np 标量溜进去。"""
    _, stats, _, _ = _detect(True, "vector", ["trash can", "chair"], profile=True)
    assert json.loads(json.dumps(stats)) == stats


def _module_source(relative: str) -> str:
    from pathlib import Path

    return (Path(__file__).resolve().parents[1] / relative).read_text(encoding="utf-8")


def test_the_local_detector_path_cannot_touch_cloud_api_counters() -> None:
    """§4.1.5 的**结构**面：本地检测链路里不该出现任何云端计数。

    跑本地 GroundingDINO 的是 `run_autonomous_loop.py`，云端计数桶长在
    `run_semantic_exploration.py` 上；这条守住"两者不串门"。
    """
    for relative in ("app/detectors/grounded_sam_worker.py",
                     "app/detectors/grounded_sam_subprocess.py",
                     "scripts/go2w/run_autonomous_loop.py"):
        src = _module_source(relative)
        for forbidden in ("quick_api_calls", "semantic_api_calls", "LatencyProfiler"):
            assert forbidden not in src, f"{relative} 里出现了云端计数 {forbidden}"


def test_a_local_detection_is_not_counted_as_a_cloud_api_call() -> None:
    """§4.1.5 / 独立审查 R1：按后端分流是**行为**，不是源码里的先后位置。

    这条测试以前断言的是"`incr("quick_api_calls")` 出现在
    `profiler.record("quick_vlm_ms", ...)` 之后"。那只说明两行代码谁在前，
    完全说明不了本地检测会不会被算成云端调用 —— 而无条件 +1 恰好满足那个
    断言，于是 R1 的错误被测试**固化**了。

    现在跑真的 `LatencyProfiler`，本地/云端两种后端的替身各来一次，直接看
    桶里的数字。
    """
    import importlib.util
    from pathlib import Path

    from app.live_robot.latency_profiler import LatencyProfiler

    spec = importlib.util.spec_from_file_location(
        "_rse_for_counters",
        Path(__file__).resolve().parents[1] / "scripts" / "go2w"
        / "run_semantic_exploration.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    local = SimpleNamespace(_detector="grounded_sam")
    cloud = SimpleNamespace(_detector="llm")

    profiler = LatencyProfiler()
    assert module.count_cloud_api_call(profiler, local, "quick_api_calls") is False
    assert module.count_cloud_api_call(profiler, local, "semantic_api_calls") is False
    assert profiler.profile.api_calls == {}, (
        f"网页默认的本地检测被算成了云端调用：{profiler.profile.api_calls}"
    )

    assert module.count_cloud_api_call(profiler, cloud, "quick_api_calls") is True
    assert module.count_cloud_api_call(profiler, cloud, "semantic_api_calls") is True
    assert profiler.profile.api_calls == {
        "quick_api_calls": 1, "semantic_api_calls": 1
    }, "云端那一路反而没记上"
    # 拿不到 `_detector` 时按云端算：漏记比多记更难发现（默认值就是 llm）。
    assert module.count_cloud_api_call(
        profiler, SimpleNamespace(), "quick_api_calls"
    ) is True
    assert profiler.profile.api_calls["quick_api_calls"] == 2


def test_the_diagnostics_event_writes_no_api_or_http_call() -> None:
    """新加的事件只是写一条 JSONL —— 不请求、不计数。"""
    src = _module_source("scripts/go2w/run_autonomous_loop.py")
    body = src[src.index("def _write_detector_diagnostics("):]
    body = body[:body.index("\n    def ", 1)] if "\n    def " in body[1:] else body
    for forbidden in ("_daemon_vlm_request", "requests.", "httpx", "openai",
                      "incr(", "urlopen"):
        assert forbidden not in body, f"诊断事件里出现了 {forbidden}"


def test_an_older_worker_without_diagnostics_keeps_the_previous_record() -> None:
    """兼容性：worker 是**另一个解释器**，可能还是旧版本。

    旧响应没有 `diagnostics` 键。客户端必须既不当成异常，也不把上一次的生效值
    用空字典抹掉 —— 否则日志里"生效值"会看起来像"没生效"。
    """
    from app.config import get_settings
    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

    detector = GroundedSAMSubprocessDetector(get_settings())
    detector.last_diagnostics = {"frame_feature_cache": True, "backbone_count": 1}

    class _FakeWorker:
        stdin = SimpleNamespace(write=lambda _text: None, flush=lambda: None)
        stdout = SimpleNamespace(readline=lambda: json.dumps({"objects": []}) + "\n")
        poll = staticmethod(lambda: None)  # 活着：`_ensure_worker` 只在这时才复用

    detector._worker = _FakeWorker()  # type: ignore[assignment]
    payload = detector._request(image=__import__("pathlib").Path("frame.jpg"))

    assert payload == {"objects": []}
    assert detector.last_diagnostics == {"frame_feature_cache": True, "backbone_count": 1}
    # 保留历史值是为了兼容旧日志读取方，但**必须**同时说清"这一次没拿到诊断"
    # —— 否则日志里那条记录会被当成这一次请求的生效值（R2）。
    assert detector.last_diagnostics_available is False
    assert detector.detect_attempts[0]["diagnostics"] is None
    assert "no diagnostics" in detector.detect_attempts[0]["diagnostics_unavailable"]


def test_the_diagnostics_event_binds_frame_and_keeps_every_attempt() -> None:
    """诊断事件的负载：帧号是传进来的那个、每次请求都在、缺诊断不冒充（R2）。

    跑的是生产里的 `diagnostics_event_payload()` 本体。之所以能直接跑，是因为
    它被从 `run_autonomous_loop.py`（依赖 rclpy，宿主 .venv 里 import 不进来）
    挪到了 detector 模块 —— 判据留在那边就只能靠读源码来"验证"。
    """
    from app.config import get_settings
    from app.detectors.grounded_sam_subprocess import (
        GroundedSAMSubprocessDetector,
        diagnostics_event_payload,
    )

    detector = GroundedSAMSubprocessDetector(get_settings())

    # 一帧两次请求（首检 + 高召回重试），各自带自己的诊断。
    detector.detect_attempts = [
        {"request_id": "sess-7", "frame_id": "1177",
         "diagnostics": {"term_count": 19, "backbone_count": 1}},
        {"request_id": "sess-8", "frame_id": "1177",
         "diagnostics": {"term_count": 19, "backbone_count": 1}},
    ]
    detector.last_diagnostics = {"term_count": 19, "backbone_count": 1}
    detector.last_diagnostics_available = True

    payload = diagnostics_event_payload(detector, frame_id=1177)
    assert payload["event"] == "detector_diagnostics"
    # 显式传入的帧号，不是某个"最近一帧"的全局状态。
    assert payload["frame_id"] == "1177"
    assert [item["request_id"] for item in payload["attempts"]] == ["sess-7", "sess-8"]
    assert payload["diagnostics_available"] is True

    # 换一帧、且这一次响应没带诊断：历史值不得冒充当前值。
    detector.detect_attempts = [
        {"request_id": "sess-9", "frame_id": "2001", "diagnostics": None,
         "diagnostics_unavailable": "worker response carried no diagnostics"},
    ]
    detector.last_diagnostics_available = False
    stale = diagnostics_event_payload(detector, frame_id=2001)
    assert stale["frame_id"] == "2001"
    assert stale["diagnostics"] is None, "把上一条的历史诊断当成了这一次的生效值"
    assert stale["diagnostics_available"] is False
    assert stale["attempts"][0]["diagnostics"] is None

    # 什么诊断都没有（例如旧 worker + 没有 attempts）时干脆不写这条事件。
    empty = GroundedSAMSubprocessDetector(get_settings())
    assert diagnostics_event_payload(empty, frame_id=1) is None


def test_the_high_recall_retry_is_a_separate_request_and_re_encodes_once(
    tmp_path,
) -> None:
    """高召回重试**不复用**首次的骨干特征 —— 这一点必须显式写下来并测住。

    重试走的是 `_maybe_retry_high_recall_thresholds` → 又一次 `_request` → worker
    里又一次 `_detect` → 新的帧上下文。所以同一帧最多两次 backbone 编码（各约
    47 ms，见 `reports/perception_profile_results_20261013.md`），不是"两次检测
    共用一次编码"。代价小，但**必须是已知的**：如果哪天有人把帧上下文提到
    请求之间，这条测试会红，那时得先论证跨请求复用特征不会串帧。
    """
    from dataclasses import replace

    from app.config import get_settings
    from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector

    settings = replace(
        get_settings(),
        enable_gdino_high_recall=True,
        grounding_dino_box_threshold=0.12,
        grounding_dino_high_recall_box_threshold=0.08,
        grounding_dino_text_threshold=0.10,
        grounding_dino_high_recall_text_threshold=0.05,
    )
    detector = GroundedSAMSubprocessDetector(settings)
    sent: list[dict] = []

    class _FakeWorker:
        stdin = SimpleNamespace(
            write=lambda text: sent.append(json.loads(text)),
            flush=lambda: None,
        )
        # 第一次空 → 触发重试；第二次给一个物体让流程收口。
        stdout = SimpleNamespace(readline=lambda: next(replies))
        poll = staticmethod(lambda: None)

    replies = iter([
        json.dumps({"objects": [], "diagnostics": {"backbone_count": 1}}) + "\n",
        json.dumps({"objects": [{"label": "chair", "bbox_2d": [0, 0, 1, 1]}],
                    "diagnostics": {"backbone_count": 1}}) + "\n",
    ])
    detector._worker = _FakeWorker()  # type: ignore[assignment]

    # 假 worker 不会真读图，客户端只查 `is_file()`，所以这里只要是个文件。
    frame = tmp_path / "frame.jpg"
    frame.write_bytes(b"\xff\xd8\xff")
    objects = detector.detect_with_dynamic_terms(
        str(frame), "chair", ["chair"], frame_id="5465"
    )

    assert len(sent) == 2, "重试没有发出第二次请求"
    assert sent[0]["box_threshold"] == 0.12 and sent[1]["box_threshold"] == 0.08
    # 开关必须跟着重试一起走，否则重试那一轮会悄悄退回旧路径。
    for request in sent:
        assert request["dino_frame_feature_cache"] is True
        assert request["mask_rle_encoder"] == "vector"
        # 源帧号也跟着走：诊断要能绑到帧上，重试那一轮同样不能丢（R2）。
        assert request["frame_id"] == "5465"
    assert [item.label for item in objects] == ["chair"]

    # 两次请求都留在 attempts 里。只留最后一条的话，首次请求的词数/前向次数
    # 就从日志里消失了 —— 而"这一帧到底跑了多少活"正需要它们（R2）。
    attempts = detector.detect_attempts
    assert len(attempts) == 2, f"只记下了 {len(attempts)} 次请求"
    assert [item["frame_id"] for item in attempts] == ["5465", "5465"]
    assert [item["diagnostics"]["backbone_count"] for item in attempts] == [1, 1]
    assert len({item["request_id"] for item in attempts}) == 2, "两次请求共用了一个 id"
    assert all(item["source_image"].endswith("frame.jpg") for item in attempts)
