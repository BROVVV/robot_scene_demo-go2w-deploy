#!/usr/bin/env python3
"""Evaluate target detection JSON using IoU, precision and recall."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data/eval/annotations")
    parser.add_argument("--predictions-dir")
    parser.add_argument("--detector", default="grounded_sam")
    parser.add_argument("--enable-crop-verify", action="store_true")
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--output-dir", default="outputs/eval")
    return parser.parse_args()


def evaluate(
    dataset: str | Path,
    predictions_dir: str | Path | None = None,
    iou_threshold: float = 0.5,
) -> dict[str, Any]:
    annotation_paths = sorted(Path(dataset).glob("*.json"))
    if not annotation_paths:
        raise FileNotFoundError(f"No JSON annotations found in {dataset}")
    true_positive = false_positive = false_negative = 0
    ious: list[float] = []
    before_tp = before_fp = 0
    error_cases: list[dict[str, Any]] = []

    for annotation_path in annotation_paths:
        annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
        prediction = _load_prediction(
            annotation, annotation_path, predictions_dir
        )
        gt = [item for item in annotation.get("objects", []) if item.get("is_target")]
        raw_predictions = prediction.get("candidate_objects") or prediction.get("objects") or []
        final_predictions = [
            item
            for item in raw_predictions
            if item.get("decision") in {None, "confirmed"}
        ]
        before_matches, before_unmatched = _match(gt, raw_predictions, iou_threshold)
        matches, unmatched = _match(gt, final_predictions, iou_threshold)
        true_positive += len(matches)
        false_negative += len(gt) - len(matches)
        false_positive += unmatched
        before_tp += len(before_matches)
        before_fp += before_unmatched
        ious.extend(score for _, _, score in matches)
        if len(matches) < len(gt) or unmatched:
            error_cases.append(
                {
                    "annotation": str(annotation_path),
                    "missed": len(gt) - len(matches),
                    "false_positive": unmatched,
                }
            )

    precision = _ratio(true_positive, true_positive + false_positive)
    recall = _ratio(true_positive, true_positive + false_negative)
    before_precision = _ratio(before_tp, before_tp + before_fp)
    before_recall = _ratio(before_tp, before_tp + false_negative)
    return {
        "num_images": len(annotation_paths),
        "target_recall": recall,
        "target_precision": precision,
        "mean_iou": round(mean(ious), 4) if ious else 0.0,
        "false_positive_per_image": round(false_positive / len(annotation_paths), 4),
        "before_crop_verify": {
            "target_recall": before_recall,
            "target_precision": before_precision,
        },
        "after_crop_verify": {
            "target_recall": recall,
            "target_precision": precision,
        },
        "error_cases": error_cases,
    }


def write_report(result: dict[str, Any], output_dir: str | Path) -> tuple[Path, Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "detection_eval_result.json"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    md_path = output / "detection_eval_report.md"
    md_path.write_text(
        "\n".join(
            [
                "# Detection accuracy evaluation",
                "",
                f"- images: {result['num_images']}",
                f"- target recall: {result['target_recall']:.4f}",
                f"- target precision: {result['target_precision']:.4f}",
                f"- mean IoU: {result['mean_iou']:.4f}",
                f"- false positives/image: {result['false_positive_per_image']:.4f}",
                "",
                "## Before/after crop verification",
                "",
                f"- before: {result['before_crop_verify']}",
                f"- after: {result['after_crop_verify']}",
                "",
                "## Error cases",
                "",
                "```json",
                json.dumps(result["error_cases"], ensure_ascii=False, indent=2),
                "```",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return json_path, md_path


def _load_prediction(
    annotation: dict[str, Any],
    annotation_path: Path,
    predictions_dir: str | Path | None,
) -> dict[str, Any]:
    if isinstance(annotation.get("prediction"), dict):
        return annotation["prediction"]
    if predictions_dir:
        path = Path(predictions_dir) / annotation_path.name
        if path.is_file():
            return json.loads(path.read_text(encoding="utf-8"))
    raise FileNotFoundError(
        f"No embedded prediction or matching prediction file for {annotation_path}"
    )


def _match(
    ground_truth: list[dict[str, Any]],
    predictions: list[dict[str, Any]],
    threshold: float,
) -> tuple[list[tuple[int, int, float]], int]:
    matches: list[tuple[int, int, float]] = []
    used_predictions: set[int] = set()
    for gt_index, gt in enumerate(ground_truth):
        best_index = None
        best_iou = 0.0
        for pred_index, prediction in enumerate(predictions):
            if pred_index in used_predictions:
                continue
            score = bbox_iou(gt.get("bbox", []), _prediction_bbox(prediction))
            if score > best_iou:
                best_iou = score
                best_index = pred_index
        if best_index is not None and best_iou >= threshold:
            used_predictions.add(best_index)
            matches.append((gt_index, best_index, best_iou))
    return matches, len(predictions) - len(used_predictions)


def bbox_iou(left: list[float], right: list[float]) -> float:
    if len(left) != 4 or len(right) != 4:
        return 0.0
    x1 = max(float(left[0]), float(right[0]))
    y1 = max(float(left[1]), float(right[1]))
    x2 = min(float(left[2]), float(right[2]))
    y2 = min(float(left[3]), float(right[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    left_area = max(0.0, float(left[2]) - float(left[0])) * max(
        0.0, float(left[3]) - float(left[1])
    )
    right_area = max(0.0, float(right[2]) - float(right[0])) * max(
        0.0, float(right[3]) - float(right[1])
    )
    union = left_area + right_area - intersection
    return intersection / union if union > 0 else 0.0


def _prediction_bbox(prediction: dict[str, Any]) -> list[float]:
    bbox = prediction.get("bbox") or prediction.get("bbox_2d") or []
    if isinstance(bbox, dict):
        return [bbox.get("x1", 0), bbox.get("y1", 0), bbox.get("x2", 0), bbox.get("y2", 0)]
    return list(bbox)


def _ratio(numerator: int, denominator: int) -> float:
    return round(numerator / denominator, 4) if denominator else 0.0


def main() -> int:
    args = parse_args()
    result = evaluate(args.dataset, args.predictions_dir, args.iou_threshold)
    for path in write_report(result, args.output_dir):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
