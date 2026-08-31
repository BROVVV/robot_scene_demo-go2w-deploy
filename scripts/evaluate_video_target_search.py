#!/usr/bin/env python3
"""Evaluate event-level video target search and track stability."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="data/eval/video_annotations")
    parser.add_argument("--predictions-dir")
    parser.add_argument("--detector", default="grounded_sam")
    parser.add_argument("--enable-tracking", action="store_true")
    parser.add_argument("--enable-crop-verify", action="store_true")
    parser.add_argument("--output-dir", default="outputs/eval")
    return parser.parse_args()


def evaluate(
    dataset: str | Path,
    predictions_dir: str | Path | None = None,
) -> dict[str, Any]:
    paths = sorted(Path(dataset).glob("*.json"))
    if not paths:
        raise FileNotFoundError(f"No JSON annotations found in {dataset}")
    tp = fp = fn = 0
    time_errors: list[float] = []
    stability: list[float] = []
    for path in paths:
        annotation = json.loads(path.read_text(encoding="utf-8"))
        prediction = annotation.get("prediction")
        if prediction is None and predictions_dir:
            prediction_path = Path(predictions_dir) / path.name
            prediction = json.loads(prediction_path.read_text(encoding="utf-8"))
        if not isinstance(prediction, dict):
            raise FileNotFoundError(f"No prediction available for {path}")
        events = [item for item in annotation.get("events", []) if item.get("is_target")]
        tracks = [
            item
            for item in prediction.get("tracks", prediction.get("video_object_tracks", []))
            if item.get("decision") == "confirmed"
        ]
        used: set[int] = set()
        for event in events:
            best = None
            best_overlap = 0.0
            for index, track in enumerate(tracks):
                if index in used:
                    continue
                overlap = _temporal_iou(event, track)
                if overlap > best_overlap:
                    best_overlap = overlap
                    best = index
            if best is not None and best_overlap > 0:
                used.add(best)
                tp += 1
                track = tracks[best]
                time_errors.append(
                    abs(float(track.get("first_seen_sec", 0.0)) - float(event["start_sec"]))
                )
                stability.append(
                    min(1.0, float(track.get("frame_count", 0)) / max(1, len(event.get("bboxes", {}))))
                )
            else:
                fn += 1
        fp += len(tracks) - len(used)
    return {
        "num_videos": len(paths),
        "event_recall": _ratio(tp, tp + fn),
        "event_precision": _ratio(tp, tp + fp),
        "mean_time_error_sec": round(mean(time_errors), 4) if time_errors else 0.0,
        "track_stability": round(mean(stability), 4) if stability else 0.0,
        "false_tracks_per_video": round(fp / len(paths), 4),
    }


def write_report(result: dict[str, Any], output_dir: str | Path) -> tuple[Path, Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "video_eval_result.json"
    json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    md_path = output / "video_eval_report.md"
    md_path.write_text(
        "# Video target-search evaluation\n\n"
        + "\n".join(f"- {key}: {value}" for key, value in result.items())
        + "\n",
        encoding="utf-8",
    )
    return json_path, md_path


def _temporal_iou(event: dict[str, Any], track: dict[str, Any]) -> float:
    start = max(float(event["start_sec"]), float(track.get("first_seen_sec", 0.0)))
    end = min(float(event["end_sec"]), float(track.get("last_seen_sec", 0.0)))
    intersection = max(0.0, end - start)
    union = max(float(event["end_sec"]), float(track.get("last_seen_sec", 0.0))) - min(
        float(event["start_sec"]), float(track.get("first_seen_sec", 0.0))
    )
    return intersection / union if union > 0 else 0.0


def _ratio(numerator: int, denominator: int) -> float:
    return round(numerator / denominator, 4) if denominator else 0.0


def main() -> int:
    args = parse_args()
    result = evaluate(args.dataset, args.predictions_dir)
    for path in write_report(result, args.output_dir):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
