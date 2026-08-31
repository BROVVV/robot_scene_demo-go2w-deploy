"""Worker script run by a Python environment that has Grounding DINO/SAM2 deps."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--root", required=True)
    parser.add_argument("--text-prompt", required=True)
    parser.add_argument("--grounding-config", required=True)
    parser.add_argument("--grounding-checkpoint", required=True)
    parser.add_argument("--box-threshold", type=float, required=True)
    parser.add_argument("--text-threshold", type=float, required=True)
    parser.add_argument("--sam2-config", required=True)
    parser.add_argument("--sam2-checkpoint", required=True)
    parser.add_argument("--max-objects", type=int, required=True)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--disable-sam2", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.text_prompt or not args.text_prompt.strip():
        raise ValueError(
            "text_prompt is empty. GroundingDINO requires a non-empty "
            "open-vocabulary prompt."
        )
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
    image_source, image = load_image(args.image)
    height, width, _ = image_source.shape

    grounding_model = load_model(
        model_config_path=str(root / args.grounding_config),
        model_checkpoint_path=str(root / args.grounding_checkpoint),
        device=device,
    )
    boxes, confidences, labels = _predict_all_prompts(
        grounding_model=grounding_model,
        image=image,
        text_prompt=args.text_prompt,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        device=device,
        torch=torch,
        predict=predict,
    )
    if len(boxes) == 0:
        _write_json(args.output, {"objects": []})
        return 0

    pixel_boxes = boxes * torch.tensor([width, height, width, height])
    xyxy = box_convert(boxes=pixel_boxes, in_fmt="cxcywh", out_fmt="xyxy")
    keep = _filter_boxes(xyxy, confidences, width, height, torch)
    xyxy = xyxy[keep]
    confidences = confidences[keep]
    labels = [labels[index] for index in keep.detach().cpu().numpy().tolist()]
    if len(xyxy) == 0:
        _write_json(args.output, {"objects": []})
        return 0

    keep = nms(xyxy, confidences, iou_threshold=0.5)[: args.max_objects]
    xyxy = xyxy[keep].detach().cpu().numpy()
    confidences_list = confidences[keep].detach().cpu().numpy().tolist()
    labels_list = [labels[index] for index in keep.detach().cpu().numpy().tolist()]

    mask_area_ratios: list[float | None] = [None] * len(labels_list)
    if not args.disable_sam2 and len(labels_list) > 0:
        mask_area_ratios = _predict_mask_areas(
            root=root,
            args=args,
            image_source=image_source,
            boxes=xyxy,
            device=device,
            torch=torch,
            np=np,
            image_area=width * height,
        )

    objects = []
    for label, score, box, mask_area_ratio in zip(
        labels_list,
        confidences_list,
        xyxy.tolist(),
        mask_area_ratios,
    ):
        x1, y1, x2, y2 = box
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
                "attributes": [],
            }
        )

    _write_json(args.output, {"objects": objects})
    return 0


def _predict_mask_areas(
    root: Path,
    args: argparse.Namespace,
    image_source,
    boxes,
    device: str,
    torch,
    np,
    image_area: int,
) -> list[float | None]:
    try:
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
    except Exception as exc:
        print(f"SAM2 import failed, continuing without masks: {exc}", file=sys.stderr)
        return [None] * len(boxes)

    try:
        sam2_model = build_sam2(
            args.sam2_config,
            str(root / args.sam2_checkpoint),
            device=device,
        )
        predictor = SAM2ImagePredictor(sam2_model)
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
        return [
            float(np.asarray(mask, dtype=bool).sum() / image_area)
            for mask in masks
        ]
    except Exception as exc:
        print(f"SAM2 prediction failed, continuing without masks: {exc}", file=sys.stderr)
        return [None] * len(boxes)


def _select_device(requested: str, torch) -> str:
    if requested != "auto":
        return requested
    has_cuda_runtime = getattr(torch.version, "cuda", None) is not None
    return "cuda" if has_cuda_runtime and torch.cuda.is_available() else "cpu"


def _predict_all_prompts(
    grounding_model,
    image,
    text_prompt: str,
    box_threshold: float,
    text_threshold: float,
    device: str,
    torch,
    predict,
):
    boxes_list = []
    confidences_list = []
    labels: list[str] = []
    for prompt in [item.strip() for item in text_prompt.split("||") if item.strip()]:
        boxes, confidences, prompt_labels = predict(
            model=grounding_model,
            image=image,
            caption=prompt.lower(),
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=device,
        )
        if len(boxes) == 0:
            continue
        boxes_list.append(boxes)
        confidences_list.append(confidences)
        labels.extend(prompt_labels)

    if not boxes_list:
        return torch.empty((0, 4)), torch.empty((0,)), []
    return torch.cat(boxes_list, dim=0), torch.cat(confidences_list, dim=0), labels


def _filter_boxes(xyxy, confidences, width: int, height: int, torch):
    box_widths = xyxy[:, 2] - xyxy[:, 0]
    box_heights = xyxy[:, 3] - xyxy[:, 1]
    area_ratio = (box_widths * box_heights) / float(width * height)
    valid = (area_ratio < 0.85) & (box_widths > 2) & (box_heights > 2)
    indexes = torch.nonzero(valid).flatten()
    if len(indexes) > 0:
        return indexes
    return torch.argsort(confidences, descending=True)[:1]


def _write_json(path: str, payload: dict) -> None:
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


if __name__ == "__main__":
    raise SystemExit(main())
