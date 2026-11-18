"""Worker script run by a Python environment that has Grounding DINO/SAM2 deps.

Two modes:

* one-shot (legacy): ``--image`` + ``--output``, run once and write a JSON file.
* ``--serve``: load GroundingDINO/SAM2 exactly once, then read one JSON request
  per line from stdin and write one JSON response per line to stdout.  This is
  what the persistent detector uses so a whole search session loads each model
  a single time instead of once per frame.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serve", action="store_true")
    parser.add_argument("--image", required=False)
    parser.add_argument("--output", required=False)
    parser.add_argument("--root", required=True)
    parser.add_argument("--text-prompt", required=False)
    parser.add_argument("--grounding-config", required=True)
    parser.add_argument("--grounding-checkpoint", required=True)
    parser.add_argument("--box-threshold", type=float, required=False)
    parser.add_argument("--text-threshold", type=float, required=False)
    parser.add_argument("--sam2-config", required=True)
    parser.add_argument("--sam2-checkpoint", required=True)
    parser.add_argument("--max-objects", type=int, required=False)
    # 轻量化计划 §20。「只有通过 GroundingDINO 基础门槛的 bbox 才跑 SAM2」——
    # 用的就是这里的 score（confidences），不是另算一个量。
    #
    # 这里给 **default 而不是像上面几个阈值那样靠调用点 `or 0.35` 兜底**：
    # 0.0 是合法且有意义的值（关掉这道门 = 所有框都跑 SAM2，即旧行为），
    # `or` 会把它顶成 0.35，于是"关掉"变成"开到 0.35"。字面量与
    # ``app/config.py: DEFAULT_SAM2_MIN_BOX_SCORE`` 同值 —— worker 跑在另一个
    # 解释器里、import 不到 app，这是本文件既有的约定（0.35/0.25/20 同理）。
    parser.add_argument("--sam2-min-box-score", type=float, default=0.35)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--disable-sam2", action="store_true")
    parser.add_argument("--ram-root", default="")
    parser.add_argument("--ram-checkpoint", default="")
    parser.add_argument("--disable-ram-plus", action="store_true")
    # 计划书 §4.1（阶段一）的两个等价优化开关 + 计时开关。
    #
    # 和 box/text 阈值一样，这几个参数在 `--serve` 路径上**以请求体为准**
    # （见 `_detect_options`），命令行这份只对 `--image` 一次性路径生效。
    # 默认值与 `app/config.py` 的 DEFAULT_* 同值 —— worker 跑在另一个解释器里、
    # import 不到 app，这是本文件既有的约定（0.35/0.25/20 同理）。
    parser.add_argument(
        "--no-dino-frame-feature-cache",
        dest="dino_frame_feature_cache",
        action="store_false",
        default=True,
        help="关闭同帧 DINO 图像特征复用（回到每个词都重算 backbone 的旧行为）",
    )
    parser.add_argument(
        "--mask-rle-encoder",
        choices=("python", "vector"),
        default="vector",
        help="mask 游程编码实现；python = 旧的逐像素循环",
    )
    parser.add_argument(
        "--profile-detection",
        action="store_true",
        help="打开分项计时；会给每个阶段加 CUDA 同步，只用于测量，不进生产",
    )
    # 计划书 §5.1.6 的多词分组前向**实验**开关。与上面几个一样：`--serve` 路径
    # 以请求体为准，命令行这份只对 `--image` 一次性路径生效。默认逐词 ——
    # 分组是待评估的候选，不是新默认（默认值字面量与
    # ``app/config.py: DEFAULT_GO2W_DINO_PROMPT_MODE`` 同值）。
    parser.add_argument(
        "--dino-prompt-mode",
        choices=PROMPT_MODES,
        default=DEFAULT_PROMPT_MODE,
        help="per_term = 每个词一次 predict（现行）；"
             "grouped = 多词拼一条 caption 一次 predict，再用 token 跨度把框关联回词",
    )
    return parser.parse_args()


RAM_IMAGE_SIZE = 384


def _as_bool(value) -> bool:
    """把请求体里可能是 `true` / `"true"` / `1` 的值收敛成 bool。

    JSON 请求体里这几个键是**调用方**写的，一个手写的 `"false"` 字符串在
    `if value:` 下是真值 —— 那会让"关掉缓存"变成"开着缓存"，且不报错。
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in {"", "0", "false", "no", "off"}
    return bool(value)


MASK_RLE_ENCODERS = ("python", "vector")
DEFAULT_MASK_RLE_ENCODER = "vector"


def _resolve_mask_rle_encoder(value) -> tuple[str, str]:
    """把编码器名字收敛成 ``(生效值, 请求值)``。

    未知取值（例如 env 里手打的 `MASK_RLE_ENCODER=numpy`）以前会**静默**退回
    向量实现，而 diagnostics 仍然回填那个未知字符串 —— 于是"生效值"记录不实：
    日志说跑的是 `numpy`，实际跑的是 `vector`。这里统一解析一次，两个字段都
    回填：`mask_rle_encoder` 是真正跑的那一个，`mask_rle_encoder_requested`
    是要求跑的那一个；对不上就是配置写错了，肉眼可查。
    """
    requested = str(value or "").strip().lower()
    effective = requested if requested in MASK_RLE_ENCODERS else DEFAULT_MASK_RLE_ENCODER
    return effective, requested


PROMPT_MODES = ("per_term", "grouped")
DEFAULT_PROMPT_MODE = "per_term"


def _resolve_prompt_mode(value) -> tuple[str, str]:
    """把 prompt 模式收敛成 ``(生效值, 请求值)``，规矩同 ``_resolve_mask_rle_encoder``。

    分两组回填是**必须**的：分组前向是实验项，若请求写了个不认识的取值
    （如 `group` 少个 ed）而 worker 静默跑 per_term、diagnostics 却回填
    `group`，事后就看不出"这一批数据其实全是逐词跑的"。
    """
    requested = str(value or "").strip().lower()
    effective = requested if requested in PROMPT_MODES else DEFAULT_PROMPT_MODE
    return effective, requested


_WORD_SPLIT = " \t\n\r"


def _normalize_term(text) -> str:
    """term / phrase 的归一化：小写、去点、折叠空白。

    `.` 要去掉：``predict`` 回传的 phrase 已经 `.replace('.','')`，而词表里的词
    本身也可能带点（`"coffee. table"` 这种写法在静态词表里有）。
    """
    return " ".join(str(text or "").lower().replace(".", " ").split())


def associate_phrase_to_term(phrase: str, terms: list[str]) -> str | None:
    """把 ``predict()`` 回传的 phrase 关联回**一个**词；关联不上返回 ``None``。

    只认这三步，顺序即优先级：

    1. 归一化后**逐字相等** —— 绝大多数情况走这条。
    2. 否则"包含"关系，且候选**唯一**：phrase 是某个词的子串（
       ``"trash"`` → ``"trash bin"``），或某个词是 phrase 的子串。
    3. 候选不唯一（``"chair"`` 同时命中 ``"chair"``/``"office chair"`` —
       不过第 1 步已经先接住它了）或一个都没命中 ⇒ ``None``。

    为什么要"唯一"这一条：把 ``"office chair"`` 的框塞给 ``"chair"`` 正是阶段一
    删掉 phrase 模式的那个串词病。宁可丢掉一个框并**计数**，也不猜。调用方
    （``_predict_grouped``）把丢掉的框数写进 diagnostics，所以"丢了多少"是
    可查的，不是静默的。
    """
    target = _normalize_term(phrase)
    if not target:
        return None
    normalized = [(_normalize_term(term), term) for term in terms]
    for norm, original in normalized:
        if norm and norm == target:
            return original
    contained = [
        original for norm, original in normalized
        if norm and (norm in target or target in norm)
    ]
    if len(contained) == 1:
        return contained[0]
    return None


def _detect_options(args: argparse.Namespace, request: dict | None = None) -> dict:
    """本次检测的生效选项。请求体优先于命令行，与阈值同一套约定。"""
    request = request or {}
    encoder, encoder_requested = _resolve_mask_rle_encoder(
        request.get("mask_rle_encoder", args.mask_rle_encoder)
    )
    prompt_mode, prompt_mode_requested = _resolve_prompt_mode(
        request.get("dino_prompt_mode", getattr(args, "dino_prompt_mode", None))
    )
    return {
        "frame_feature_cache": _as_bool(
            request.get("dino_frame_feature_cache", args.dino_frame_feature_cache)
        ),
        "mask_rle_encoder": encoder,
        "mask_rle_encoder_requested": encoder_requested,
        "prompt_mode": prompt_mode,
        "prompt_mode_requested": prompt_mode_requested,
        "profile": _as_bool(
            request.get("profile_detection", getattr(args, "profile_detection", False))
        ),
        "request_id": str(request.get("request_id") or ""),
        # 调用方显式带过来的源帧号。**不由 worker 猜**：worker 只是把请求里
        # 这一格原样回填，好让"这次耗时/计数属于哪一帧"能对上，而不是靠
        # 客户端记住"最近一帧是谁"。
        "frame_id": str(request.get("frame_id") or ""),
    }


class _DinoFrameContext:
    """一帧之内复用 GroundingDINO 的 backbone 特征，并统计 backbone 调用次数。

    `predict()`（`grounding_dino/util/inference.py`）每次都是
    `model(image[None], captions=[caption])`，而模型 forward 的
    `unset_image_tensor` 默认 True —— 每个 term 结束就把 `self.features` 删掉，
    下一个 term 重新跑一遍 backbone。本仓库当前是**逐词**推理（19 个词就是 19
    次 backbone），这是实测 3.865 s 里约 0.894 s 的来源。

    进这个上下文后，forward 被强制 `unset_image_tensor=False`：第一个词编码图像，
    其余词复用。作用域是**一次 `_detect` 调用**（= 同一个 `load_image` 出来的
    张量），退出时无条件清干净 —— 下一帧必然重新编码，不会拿旧特征配新图。

    只在实例 `__dict__` 上打补丁并原样还原（`forward` 是不是原本就在实例字典里
    都要正确还原），避免把 nn.Module 的类方法换成绑定方法留在实例上。
    """

    def __init__(self, model, *, keep_features: bool) -> None:
        self._model = model
        self._keep = bool(keep_features)
        self.backbone_calls = 0
        self._restore: list[tuple[str, object]] = []

    def _patch(self, target, name: str, replacement) -> None:
        had = name in target.__dict__
        self._restore.append((name, target, had, target.__dict__.get(name)))
        target.__dict__[name] = replacement

    def __enter__(self) -> "_DinoFrameContext":
        model = self._model
        backbone = getattr(model, "backbone", None)
        if backbone is not None:
            original = backbone.forward

            def counting_backbone_forward(*call_args, **call_kwargs):
                self.backbone_calls += 1
                return original(*call_args, **call_kwargs)

            self._patch(backbone, "forward", counting_backbone_forward)
        if self._keep:
            original_forward = model.forward

            def keeping_forward(samples, targets=None, **kwargs):
                # 覆盖调用方可能传进来的 True；`predict()` 不传，默认就是 True。
                kwargs["unset_image_tensor"] = False
                return original_forward(samples, targets=targets, **kwargs)

            self._patch(model, "forward", keeping_forward)
        return self

    def __exit__(self, *_exc_info) -> bool:
        for name, target, had, original in reversed(self._restore):
            if had:
                target.__dict__[name] = original
            else:
                target.__dict__.pop(name, None)
        self._restore.clear()
        # 无论走的是哪条路（正常结束、异常、空结果提前返回）都把特征丢掉。
        # 忘了这一步的后果不是报错，而是**下一帧复用上一帧的图像特征** ——
        # 检测照常出框，只是框的位置来自上一张图。
        try:
            self._model.unset_image_tensor()
        except Exception:
            pass
        return False


@contextlib.contextmanager
def _stage(stats: dict | None, name: str, *, gpu: bool = False,
           device: str = "", torch=None):
    """分项计时。

    **只在 profiling 模式记录**：非 profiling 下 GPU 阶段只能测到 kernel 入队，
    数字没有意义；而加同步又会改变生产路径的耗时结构。所以默认既不记录、
    也不同步，生产路径的性能不被测量本身污染（计划书 §4.1.5）。
    """
    if not stats or not stats.get("profile"):
        yield
        return
    if gpu:
        _cuda_sync(torch, device)
    started = time.perf_counter()
    try:
        yield
    finally:
        if gpu:
            _cuda_sync(torch, device)
        stages = stats.setdefault("stage_ms", {})
        stages[name] = stages.get(name, 0.0) + (time.perf_counter() - started) * 1000.0


def _cuda_sync(torch, device: str) -> None:
    if torch is None:
        return
    try:
        if str(device).startswith("cuda"):
            torch.cuda.synchronize()
    except Exception:
        pass


def main() -> int:
    args = parse_args()
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root))
    os.chdir(root)

    try:
        import numpy as np
        import torch
        from torchvision.ops import box_convert, nms
        from grounding_dino.groundingdino.util.inference import (
            load_image,
            load_model,
            predict,
        )
    except Exception as exc:
        print(
            "Missing Grounding DINO runtime dependencies. "
            "Install torch, torchvision, numpy and the local Grounded-SAM-2 package "
            f"for this Python interpreter. Original error: {type(exc).__name__}: {exc}",
            file=sys.stderr,
        )
        return 2

    device = _select_device(args.device, torch)
    grounding_model = load_model(
        model_config_path=str(root / args.grounding_config),
        model_checkpoint_path=str(root / args.grounding_checkpoint),
        device=device,
    )
    sam2_predictor = _build_sam2_predictor(root, args, device, torch)
    ram_model, ram_transform = _build_ram_plus(args, device, torch)

    if args.serve:
        _serve(
            args, grounding_model, sam2_predictor, ram_model, ram_transform,
            device, torch, np, load_image, predict, box_convert, nms,
        )
        return 0

    if not args.text_prompt or not args.text_prompt.strip():
        raise ValueError(
            "text_prompt is empty. GroundingDINO requires a non-empty "
            "open-vocabulary prompt."
        )
    stats: dict = {}
    objects = _detect(
        args, args.image, args.text_prompt,
        args.box_threshold, args.text_threshold, args.max_objects,
        args.sam2_min_box_score,
        grounding_model, sam2_predictor, device, torch, np,
        load_image, predict, box_convert, nms,
        options=_detect_options(args), stats=stats,
    )
    _write_json(args.output, {"objects": objects, "diagnostics": stats})
    return 0


def _serve(args, grounding_model, sam2_predictor, ram_model, ram_transform,
           device, torch, np, load_image, predict, box_convert, nms) -> None:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
        except json.JSONDecodeError as exc:
            _write_stdout({"error": f"bad request json: {exc}"})
            continue
        if request.get("operation", "detect") == "tag":
            try:
                _write_stdout(_tag(ram_model, ram_transform, request["image"], device, torch))
            except Exception as exc:  # 一帧坏了不能弄死整个会话
                _write_stdout({"error": f"{type(exc).__name__}: {exc}"})
            continue
        if not request.get("text_prompt") or not str(request["text_prompt"]).strip():
            _write_stdout({"error": "empty text_prompt"})
            continue
        try:
            stats: dict = {}
            objects = _detect(
                args, request["image"], str(request["text_prompt"]),
                float(request.get("box_threshold", args.box_threshold or 0.35)),
                float(request.get("text_threshold", args.text_threshold or 0.25)),
                int(request.get("max_objects", args.max_objects or 20)),
                # 不用 `or`：--sam2-min-box-score 有 default(0.35)，args 永不为
                # None，而 0.0 是有意义的取值（关掉这道门），`or` 会顶掉它。
                float(request.get("sam2_min_box_score", args.sam2_min_box_score)),
                grounding_model, sam2_predictor, device, torch, np,
                load_image, predict, box_convert, nms,
                options=_detect_options(args, request), stats=stats,
            )
            # `diagnostics` 是**新增字段**，下游读取方（`_read_response` /
            # `detect_with_dynamic_terms`）都只取 `objects`，多一个键不影响它们。
            _write_stdout({"objects": objects, "diagnostics": stats})
        except Exception as exc:  # a bad frame must not kill the session
            _write_stdout({"error": f"{type(exc).__name__}: {exc}"})


def _detect(args, image_path, text_prompt, box_threshold, text_threshold,
            max_objects, sam2_min_box_score,
            grounding_model, sam2_predictor, device, torch, np,
            load_image, predict, box_convert, nms,
            *, options: dict | None = None, stats: dict | None = None) -> list[dict]:
    """跑完一帧：加载图像 → 逐词 GroundingDINO → 过滤/NMS → SAM2 → RLE。

    ``options`` / ``stats`` 是本阶段新加的**可选**入参：

    * ``options``：``frame_feature_cache`` / ``mask_rle_encoder`` / ``profile`` /
      ``request_id``。缺省就是"新路径全开"（阶段一验收通过后的默认值）。
    * ``stats``：可选输出字典，回来时带着生效配置与计数（profiling 时还有分项耗时）。
      不传就不收集 —— 老调用点（如 `artifacts/perception_profile_20261013/`）
      少传两个参数照样能跑。
    """
    options = options or {}
    frame_feature_cache = _as_bool(options.get("frame_feature_cache", True))
    # 与 `_detect_options` 走同一个解析器：两层各写一份的话，`_serve` 路径
    # 和直接调用路径会对同一个未知取值给出不同的"生效值"。
    mask_rle_encoder, mask_rle_encoder_requested = _resolve_mask_rle_encoder(
        options.get("mask_rle_encoder", DEFAULT_MASK_RLE_ENCODER)
    )
    prompt_mode, prompt_mode_requested = _resolve_prompt_mode(
        options.get("prompt_mode", DEFAULT_PROMPT_MODE)
    )
    profile = _as_bool(options.get("profile", False))
    if stats is not None:
        stats.update({
            "detector_backend": "grounding_dino_sam2",
            "request_id": str(options.get("request_id") or ""),
            "frame_id": str(options.get("frame_id") or ""),
            "frame_feature_cache": frame_feature_cache,
            "mask_rle_encoder": mask_rle_encoder,
            "mask_rle_encoder_requested": mask_rle_encoder_requested,
            # §3.1.2：记**生效值**。requested 与 effective 对不上就是配置写错了。
            "prompt_mode": prompt_mode,
            "prompt_mode_requested": prompt_mode_requested,
            "profile": profile,
            "prompt_count": 0,
            "term_count": 0,
            "dino_forward_count": 0,
            "backbone_count": 0,
            "object_count": 0,
            "mask_count": 0,
        })
    started = time.perf_counter()

    with _stage(stats, "preprocess", device=device, torch=torch):
        image_source, image = load_image(image_path)
    height, width, _ = image_source.shape

    with _DinoFrameContext(
        grounding_model, keep_features=frame_feature_cache
    ) as frame_ctx, _stage(stats, "dino", gpu=True, device=device, torch=torch):
        boxes, confidences, labels = _predict_all_prompts(
            grounding_model=grounding_model,
            image=image,
            text_prompt=text_prompt,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=device,
            torch=torch,
            predict=predict,
            stats=stats,
            prompt_mode=prompt_mode,
        )
    if stats is not None:
        stats["backbone_count"] = int(frame_ctx.backbone_calls)
    if len(boxes) == 0:
        _finish_stats(stats, started)
        return []

    pixel_boxes = boxes * torch.tensor([width, height, width, height])
    xyxy = box_convert(boxes=pixel_boxes, in_fmt="cxcywh", out_fmt="xyxy")
    keep = _filter_boxes(xyxy, confidences, width, height, torch)
    xyxy = xyxy[keep]
    confidences = confidences[keep]
    labels = [labels[index] for index in keep.detach().cpu().numpy().tolist()]
    if len(xyxy) == 0:
        _finish_stats(stats, started)
        return []

    keep = nms(xyxy, confidences, iou_threshold=0.5)
    keep_indexes = _keep_high_dynamic_candidates(
        keep.detach().cpu().numpy().tolist(),
        confidences.detach().cpu().numpy().tolist(),
        labels,
        max_objects=max_objects,
    )
    xyxy = xyxy[keep_indexes].detach().cpu().numpy()
    confidences_list = confidences[keep_indexes].detach().cpu().numpy().tolist()
    labels_list = [labels[index] for index in keep_indexes]

    # 轻量化计划 §20：**只给过了分的框跑 SAM2**。此前是 NMS 之后每个框都跑，
    # 而 SAM2 正是这条链上最贵的一段。
    #
    # 保住下标对齐：下面的组装循环是 `masks[index]`，所以先铺一个和 xyxy 等长的
    # 槽位表，只往过关的槽里填。直接 `_predict_masks(..., xyxy[passing], ...)` 会
    # 让第 k 个 mask 落到第 k 个**框**上 —— 低分框一多，掩码就整体错位，而错位的
    # 掩码不会报错，只会把 bbox 悄悄精修到别人身上。
    #
    # 没过关的框**不被丢弃**：mask 留 None，组装循环对 None 已有分支（refined_box
    # 用原框、mask_rle/mask_area_ratio 留空），它们仍是带 2D bbox 的物体。
    #
    # 门槛取值 0.35 来自 §20 原文，与真机数据校准过：`data/memory/
    # observational_memory.jsonl` 等产出里 39 个非空 detector_score 中有 3 个低于
    # 0.35（0.136 / 0.141 / 0.248），而过关的最低分是 0.800 —— 门槛坐在 3.2 倍的
    # 空隙里，所以选值不敏感，不是拿精度换速度。
    mask_slots: list = [None] * len(xyxy)
    scored = [index for index, score in enumerate(confidences_list)
              if float(score) >= float(sam2_min_box_score)]
    if scored:
        with _stage(stats, "sam2", gpu=True, device=device, torch=torch):
            predicted = _predict_masks(sam2_predictor, image_source, xyxy[scored],
                                       device, torch, np)
        if predicted is not None:
            for slot, mask in zip(scored, predicted):
                mask_slots[slot] = mask
    masks = mask_slots

    objects = []
    for index, (label, score, box) in enumerate(zip(labels_list, confidences_list, xyxy.tolist())):
        mask = masks[index] if masks is not None else None
        mask_rle = None
        mask_size = None
        refined_box = box
        mask_area_ratio = None
        if mask is not None:
            mask_size = [int(height), int(width)]
            with _stage(stats, "mask_rle", device=device, torch=torch):
                mask_rle = _encode_mask_runs(mask, encoder=mask_rle_encoder)
            mask_area_ratio = float(mask.sum() / (width * height))
            ys, xs = np.nonzero(mask)
            if xs.size > 0:
                refined_box = [float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())]
        x1, y1, x2, y2 = refined_box
        objects.append(
            {
                "label": str(label).lower().strip(),
                "score": float(score),
                "text_score": float(score),
                "source_prompt_term": str(label).lower().strip(),
                "bbox_2d": [
                    _clamp(x1 / width),
                    _clamp(y1 / height),
                    _clamp(x2 / width),
                    _clamp(y2 / height),
                ],
                "mask_area_ratio": mask_area_ratio,
                "mask_size": mask_size,
                "mask_rle": mask_rle,
                "attributes": [],
            }
        )

    if stats is not None:
        stats["mask_count"] = sum(item["mask_rle"] is not None for item in objects)
    _finish_stats(stats, started, object_count=len(objects))
    return objects


def _finish_stats(stats: dict | None, started: float, *, object_count: int | None = None) -> None:
    if stats is None:
        return
    if object_count is not None:
        stats["object_count"] = int(object_count)
    if stats.get("profile"):
        stats.setdefault("stage_ms", {})["total"] = (time.perf_counter() - started) * 1000.0


def _build_ram_plus(args: argparse.Namespace, device: str, torch):
    """载入 RAM++（swin_l, 384）。返回 (model, transform)，未启用时 (None, None)。

    只在 worker 启动时调用一次 —— RAM++ 的职责是「这帧大致有哪些类别」，
    不是 tracking，绝不能每帧重新载入。
    """
    if args.disable_ram_plus or not args.ram_root or not args.ram_checkpoint:
        return None, None
    ram_root = Path(args.ram_root).resolve()
    sys.path.insert(0, str(ram_root))
    try:
        from ram import get_transform
        from ram.models import ram_plus
    except Exception as exc:
        print(f"RAM++ import failed, continuing without tags: {exc}", file=sys.stderr)
        return None, None
    checkpoint = Path(args.ram_checkpoint)
    if not checkpoint.is_absolute():
        checkpoint = ram_root / checkpoint
    try:
        model = ram_plus(
            pretrained=str(checkpoint),
            image_size=RAM_IMAGE_SIZE,
            vit="swin_l",
        )
        model.eval()
        return model.to(device), get_transform(image_size=RAM_IMAGE_SIZE)
    except Exception as exc:
        print(f"RAM++ build failed, continuing without tags: {exc}", file=sys.stderr)
        return None, None


def _tag(ram_model, ram_transform, image_path: str, device: str, torch) -> dict:
    if ram_model is None or ram_transform is None:
        raise RuntimeError(
            "RAM++ is not loaded in this worker (missing --ram-root/--ram-checkpoint "
            "or --disable-ram-plus was passed)."
        )
    from PIL import Image
    from ram import inference_ram

    image = ram_transform(Image.open(image_path).convert("RGB")).unsqueeze(0).to(device)
    tags_en, tags_zh = inference_ram(image, ram_model)
    return {
        "tags_en": _split_tags(tags_en),
        "tags_zh": _split_tags(tags_zh),
    }


def _split_tags(raw) -> list[str]:
    """RAM++ 返回的是 `|` 分隔的一整串（实测，不是逗号），拆成列表。"""
    return [
        part.strip()
        for part in re.split(r"[|,]", str(raw))
        if part.strip()
    ]


def _build_sam2_predictor(root: Path, args: argparse.Namespace, device: str, torch):
    if args.disable_sam2:
        return None
    try:
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
    except Exception as exc:
        print(f"SAM2 import failed, continuing without masks: {exc}", file=sys.stderr)
        return None
    try:
        sam2_model = build_sam2(
            args.sam2_config,
            str(root / args.sam2_checkpoint),
            device=device,
        )
        return SAM2ImagePredictor(sam2_model)
    except Exception as exc:
        print(f"SAM2 build failed, continuing without masks: {exc}", file=sys.stderr)
        return None


def _predict_masks(predictor, image_source, boxes, device: str, torch, np):
    """Return one bool mask (HxW) per box, or None for a box that failed."""
    if predictor is None or len(boxes) == 0:
        return None
    try:
        predictor.set_image(image_source)
        with torch.inference_mode():
            masks, _, _ = predictor.predict(
                point_coords=None,
                point_labels=None,
                box=boxes,
                multimask_output=False,
            )
        if masks.ndim == 4:
            masks = masks.squeeze(1)
        return [np.asarray(mask, dtype=bool) for mask in masks]
    except Exception as exc:
        print(f"SAM2 prediction failed, continuing without masks: {exc}", file=sys.stderr)
        return None


def _encode_mask_runs(mask, encoder: str = "vector") -> list[list[int]]:
    """Row-run encode a bool mask as [row, start_x, length] runs.

    ``encoder="python"`` 保留旧的逐像素实现（对照与回退用）；默认走 NumPy
    向量化实现。两者在固定输入上逐元素相同，见
    ``tests/test_grounded_sam_equivalent_optimization.py``。

    调用方应传 `_resolve_mask_rle_encoder()` 解析过的**生效值**；这里的兜底
    分支只是保证任何取值都能跑，不再承担"把未知值纠正过来"的职责 ——
    那个职责在解析器里，且结果会回填进 diagnostics。
    """
    if str(encoder).strip().lower() == "python":
        return _encode_mask_runs_python(mask)
    return _encode_mask_runs_vector(mask)


def _encode_mask_runs_python(mask) -> list[list[int]]:
    """旧实现：对每个 H×W mask 逐行逐像素扫描。

    实测这条在 6 个 mask 上花 384 ms（`reports/perception_profile_results_20261013.md`），
    是纯 CPU 的 Python 循环。保留原样作为对照与回退路径。
    """
    runs: list[list[int]] = []
    height, width = mask.shape
    for y in range(height):
        row = mask[y]
        x = 0
        while x < width:
            if not row[x]:
                x += 1
                continue
            start = x
            while x < width and row[x]:
                x += 1
            runs.append([y, start, x - start])
    return runs


def _encode_mask_runs_vector(mask) -> list[list[int]]:
    """NumPy 实现：一次求出行内 0↔1 的边界，替代逐像素 while 循环。

    输出契约与旧实现完全一致 —— ``[row, start_x, length]``、行优先顺序、
    零基坐标、Python ``int``（不是 ``np.int64``，否则 JSON 里会变成
    ``np.int64(...)`` 无法序列化）。
    """
    # 局部 import：模块顶层故意只依赖标准库 —— `main()` 把 numpy/torch 的缺失
    # 包在 try 里换成一句人能看懂的报错，顶层 import 会让那句报错永远到不了。
    import numpy as np

    array = np.asarray(mask)
    if array.ndim != 2:
        raise ValueError(f"mask must be 2-D, got shape {array.shape}")
    # 先按**真值语义**收敛成 bool，再 view 成 int8 —— 差分要求整数 dtype
    # （bool 不支持 subtract，`np.diff` 会直接抛错）。
    #
    # 必须走 `astype(bool)` 而不是 `astype(np.int8)`：旧实现判的是真值
    # （`while row[x]`），所以 2 也算前景。直接转 int8 会让 `changes == ±1`
    # 之外的跳变凭空消失 —— 不报错，只是把掩码少画一块。
    binary = array.astype(bool)
    padded = np.pad(binary.view(np.int8), ((0, 0), (1, 1)))
    changes = np.diff(padded, axis=1)
    starts_y, starts_x = np.nonzero(changes == 1)
    ends_y, ends_x = np.nonzero(changes == -1)
    if not np.array_equal(starts_y, ends_y):
        # 每行左右各补一个 0 之后，0↔1 的跳变在该行内必然 +1/-1 交替、且以
        # 0→1 起头，所以第 k 个起点配第 k 个终点，**行号**必须一一对上
        # （列号天然相差 run 的长度，不参与比较）。不成对说明 numpy 行为变了，
        # 宁可直接失败，也不要把错位的 run 交给下游 —— 错位的 RLE 不抛异常，
        # 只会把掩码画歪，下游看到的是一个"有掩码的物体"。
        raise ValueError("mask runs are unpaired; cannot encode mask safely")
    return np.column_stack((starts_y, starts_x, ends_x - starts_x)).tolist()


def _select_device(requested: str, torch) -> str:
    if requested != "auto":
        return requested
    has_cuda_runtime = getattr(torch.version, "cuda", None) is not None
    return "cuda" if has_cuda_runtime and torch.cuda.is_available() else "cpu"


def _split_terms(text_prompt: str) -> tuple[list[str], list[str]]:
    """把 prompt 文本拆成 ``(prompt 组, 词)``。

    两层结构不能压成一层：``"||"`` 分组是**槽位**（目标词一槽、RAM 场景词一槽），
    组内 ``"."`` 才是词。`prompt_count` 数的是前者、`term_count` 数的是后者。
    """
    prompts = [item.strip() for item in text_prompt.split("||") if item.strip()]
    terms = [part.strip() for prompt in prompts for part in prompt.split(".") if part.strip()]
    return prompts, terms


def _predict_per_term(grounding_model, image, terms, box_threshold, text_threshold,
                      device, torch, predict, stats):
    """现行路径：每个词一次 ``predict()``。

    **不要**改回"多词一条 caption 直接问"：text_threshold 很低时 phrase 会把
    所有超过门限的 token 连成一串（"trash can trash bin waste bin"），下游查不到
    词表就兜底成"物体"，所有物体标签雷同、被错误合并。
    """
    boxes_list = []
    confidences_list = []
    labels: list[str] = []
    for term in terms:
        # 每个 term 恰好一次 `predict()` —— 这就是"词数"和"前向次数"两个计数在
        # `per_term`（默认模式）下必然相等的原因。它们不等时说明跑的是
        # `grouped`：那是本阶段要评估的实验分支，见 `_predict_grouped`。
        if stats is not None:
            stats["term_count"] = int(stats.get("term_count", 0)) + 1
            stats["dino_forward_count"] = int(stats.get("dino_forward_count", 0)) + 1
        boxes, confidences, _ = predict(
            model=grounding_model,
            image=image,
            caption=term.lower(),
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=device,
        )
        if len(boxes) == 0:
            continue
        boxes_list.append(boxes)
        confidences_list.append(confidences)
        # 不用 predict 回传的 phrase：text_threshold=0.10 很低，query 的激活常落在
        # [CLS]/[SEP] 上，解出来是空串或 "[unk] [unk] [unk]"。这里每次只问一个
        # term，检出来的就是这个 term，直接用它当标签最稳。
        labels.extend([term] * len(boxes))

    if not boxes_list:
        return torch.empty((0, 4)), torch.empty((0,)), []
    return torch.cat(boxes_list, dim=0), torch.cat(confidences_list, dim=0), labels


def _predict_grouped(grounding_model, image, terms, box_threshold, text_threshold,
                     device, torch, predict, stats):
    """实验分支：多词拼一条 caption，**一次** ``predict()``，再把框关联回词。

    与旧的 phrase 模式（阶段一删掉的那次）**不是**一回事，区别全在
    ``remove_combined=True``：``grounding_dino/util/inference.py`` 的
    `predict` 在这个开关下先用 token id ``[CLS]/[SEP]/.`` 算出分隔符位置
    ``sep_idx``，再用 `get_phrases_from_posmap(logit > text_threshold, tokenized,
    tokenizer, left_idx, right_idx)` 把每个 query 的激活**限制在相邻两个分隔符
    之间** —— 也就是"这个词自己的 token 跨度"。所以 phrase 不会再跨词拼串，
    串词的根因被 token 跨度挡住，而不是靠阈值调参。

    框仍然可能关联不回任何一个词（phrase 为空、或跨度解出来是别的文本）：
    那些框**丢掉并计数**（``grouped_unmatched_boxes`` / ``grouped_empty_boxes``），
    不猜、不兜底赋类。丢掉的框在 `_build_quality` 里是"漏检"而不是"没错"。
    """
    caption = " . ".join(terms) + " ."
    if stats is not None:
        stats["term_count"] = len(terms)
        stats["dino_forward_count"] = 1
        stats["grouped_caption"] = caption.lower()
    try:
        boxes, confidences, phrases = predict(
            model=grounding_model,
            image=image,
            caption=caption.lower(),
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=device,
            remove_combined=True,
        )
    except IndexError as exc:
        # `predict` 的 `bisect_left(sep_idx, max_idx)` 在后面那格越界时才会这样：
        # 某个 query 的最大激活落在了最后一个分隔符之后（padding 区）。这是上游
        # 实现的性质，不是本分支能修的；把模式写进异常，免得看着像别的故障。
        raise RuntimeError(
            f"grouped prompt mode failed inside predict()（caption={caption!r}）: {exc}"
        ) from exc

    kept_indexes: list[int] = []
    labels: list[str] = []
    unmatched = 0
    empty = 0
    for index, phrase in enumerate(phrases):
        term = associate_phrase_to_term(phrase, terms)
        if term is None:
            if _normalize_term(phrase):
                unmatched += 1
            else:
                empty += 1
            continue
        kept_indexes.append(index)
        labels.append(term)
    if stats is not None:
        stats["grouped_box_count"] = int(len(boxes))
        stats["grouped_kept_boxes"] = len(kept_indexes)
        stats["grouped_unmatched_boxes"] = unmatched
        stats["grouped_empty_boxes"] = empty
    if not kept_indexes:
        return torch.empty((0, 4)), torch.empty((0,)), []
    index_tensor = torch.tensor(kept_indexes, dtype=torch.long)
    return boxes[index_tensor], confidences[index_tensor], labels


def _predict_all_prompts(
    grounding_model,
    image,
    text_prompt: str,
    box_threshold: float,
    text_threshold: float,
    device: str,
    torch,
    predict,
    stats: dict | None = None,
    prompt_mode: str = DEFAULT_PROMPT_MODE,
):
    prompts, terms = _split_terms(text_prompt)
    if stats is not None:
        stats["prompt_count"] = len(prompts)
    if not terms:
        return torch.empty((0, 4)), torch.empty((0,)), []
    if prompt_mode == "grouped":
        return _predict_grouped(
            grounding_model, image, terms, box_threshold, text_threshold,
            device, torch, predict, stats,
        )
    return _predict_per_term(
        grounding_model, image, terms, box_threshold, text_threshold,
        device, torch, predict, stats,
    )



def _filter_boxes(xyxy, confidences, width: int, height: int, torch):
    box_widths = xyxy[:, 2] - xyxy[:, 0]
    box_heights = xyxy[:, 3] - xyxy[:, 1]
    area_ratio = (box_widths * box_heights) / float(width * height)
    valid = (area_ratio < 0.85) & (box_widths > 2) & (box_heights > 2)
    indexes = torch.nonzero(valid).flatten()
    if len(indexes) > 0:
        return indexes
    return torch.argsort(confidences, descending=True)[:1]


_HIGH_DYNAMIC_NMS_GROUPS = {
    "person": "person", "human": "person", "man": "person",
    "woman": "person", "child": "person", "pedestrian": "person",
    "dog": "dog", "puppy": "dog", "cat": "cat", "kitten": "cat",
    "animal": "animal", "pet": "animal", "bird": "bird",
    "running robot": "robot", "mobile robot": "robot",
}


def _keep_high_dynamic_candidates(
    global_keep: list[int],
    scores: list[float],
    labels: list[str],
    *,
    max_objects: int,
) -> list[int]:
    """Preserve one person/animal candidate across category-global NMS.

    The detector normally uses global NMS to collapse synonym prompts.  In a
    real robot frame a seated person substantially overlaps the office chair;
    global NMS kept the higher scoring chair and silently removed the person.
    Keep the best candidate for every high dynamic semantic group, then fill
    the remaining budget from the normal global-NMS result.  This preserves
    the existing duplicate suppression for ordinary scene objects.
    """
    mandatory: set[int] = set()
    best_by_group: dict[str, int] = {}
    for index, label in enumerate(labels):
        group = _HIGH_DYNAMIC_NMS_GROUPS.get(str(label).strip().casefold())
        if group is None:
            continue
        previous = best_by_group.get(group)
        if previous is None or float(scores[index]) > float(scores[previous]):
            best_by_group[group] = index
    mandatory.update(best_by_group.values())

    limit = max(0, int(max_objects))
    ranked_mandatory = sorted(mandatory, key=lambda index: float(scores[index]), reverse=True)
    selected = ranked_mandatory[:limit]
    for index in global_keep:
        index = int(index)
        if len(selected) >= limit:
            break
        if index not in mandatory and index not in selected:
            selected.append(index)
    return sorted(selected, key=lambda index: float(scores[index]), reverse=True)


def _write_json(path: str, payload: dict) -> None:
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _write_stdout(payload: dict) -> None:
    sys.stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


if __name__ == "__main__":
    raise SystemExit(main())
