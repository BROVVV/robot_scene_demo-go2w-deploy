"""Draw detected object boxes on source images."""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from app.schemas import (
    LLMReasoningResult,
    NodeObservationStatus,
    SceneAnalysisResult,
    SceneObject,
)


BOX_COLORS = [
    "#ff4d4f",
    "#1677ff",
    "#52c41a",
    "#faad14",
    "#722ed1",
    "#13c2c2",
    "#eb2f96",
    "#fa541c",
]


def export_annotated_image(
    result: SceneAnalysisResult,
    image_path: str | Path,
    output_path: str | Path,
) -> Path:
    source_path = Path(image_path)
    if not source_path.is_file():
        raise FileNotFoundError(f"Image file not found: {source_path}")

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    with Image.open(source_path) as image:
        canvas = image.convert("RGB")
    draw = ImageDraw.Draw(canvas)
    font = _load_font(max(14, canvas.width // 90))

    for index, obj in enumerate(result.objects):
        if not _has_usable_bbox(obj):
            continue
        color = {
            "confirmed": "#00b84a",
            "candidate": "#ff9800",
            "rejected": "#9ca3af",
        }.get(obj.decision or "", BOX_COLORS[index % len(BOX_COLORS)])
        if obj.decision == "rejected":
            continue
        _draw_object(draw, canvas.size, obj, color, font)

    canvas.save(output)
    return output


def export_reasoned_annotated_image(
    result: SceneAnalysisResult,
    reasoning: LLMReasoningResult,
    image_path: str | Path,
    output_path: str | Path,
) -> Path:
    source_path = Path(image_path)
    if not source_path.is_file():
        raise FileNotFoundError(f"Image file not found: {source_path}")
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    with Image.open(source_path) as image:
        canvas = image.convert("RGB")
    draw = ImageDraw.Draw(canvas, "RGBA")
    font = _load_font(max(14, canvas.width // 90))
    for obj in result.objects:
        if _has_usable_bbox(obj):
            _draw_object(draw, canvas.size, obj, "#1677ff", font)

    rendered_regions: set[tuple[tuple[int, int, int, int], str]] = set()
    for hypothesis in sorted(
        reasoning.hypotheses,
        key=lambda item: item.confidence,
        reverse=True,
    ):
        region = _coarse_region_box(
            hypothesis.image_region_hint,
            canvas.size,
        )
        if region is None:
            continue
        if hypothesis.status == NodeObservationStatus.OBSERVED:
            color = "#00b84a"
            status = "OBSERVED"
        elif hypothesis.status == NodeObservationStatus.UNREACHABLE:
            color = "#6b7280"
            status = "NEEDS_HUMAN"
        else:
            color = "#f59e0b"
            status = "INFERRED"
        region_key = (region, status)
        if region_key in rendered_regions or len(rendered_regions) >= 3:
            continue
        rendered_regions.add(region_key)
        _draw_dashed_rectangle(draw, region, color, max(3, canvas.width // 500))
        draw.rectangle(region, fill=_rgba(color, 28))
        label = f"{status}: {hypothesis.candidate_region_zh}"
        _draw_region_label(draw, region, label, color, font)

    legend = "实线=视觉观察  虚线=推断区域  灰色=需人工"
    text_box = draw.textbbox((0, 0), legend, font=font)
    legend_width = text_box[2] - text_box[0] + 16
    legend_height = text_box[3] - text_box[1] + 12
    draw.rectangle(
        (8, 8, 8 + legend_width, 8 + legend_height),
        fill=(17, 24, 39, 210),
    )
    draw.text((16, 14), legend, fill="white", font=font)
    canvas.save(output)
    return output


def _draw_object(
    draw: ImageDraw.ImageDraw,
    image_size: tuple[int, int],
    obj: SceneObject,
    color: str,
    font: ImageFont.ImageFont,
) -> None:
    width, height = image_size
    bbox = obj.bbox_2d
    x1 = int(bbox.x1 * width)
    y1 = int(bbox.y1 * height)
    x2 = int(bbox.x2 * width)
    y2 = int(bbox.y2 * height)
    score = obj.final_score if obj.final_score is not None else obj.confidence
    decision = f" {obj.decision}" if obj.decision else ""
    label = f"{obj.name_zh} {score:.2f}{decision}"

    line_width = max(4 if obj.decision == "confirmed" else 2, width // 500)
    draw.rectangle((x1, y1, x2, y2), outline=color, width=line_width)

    text_box = draw.textbbox((0, 0), label, font=font)
    text_width = text_box[2] - text_box[0]
    text_height = text_box[3] - text_box[1]
    label_y1 = max(0, y1 - text_height - 6)
    label_y2 = label_y1 + text_height + 6
    label_x2 = min(width, x1 + text_width + 8)
    draw.rectangle((x1, label_y1, label_x2, label_y2), fill=color)
    draw.text((x1 + 4, label_y1 + 3), label, fill="white", font=font)


def _load_font(size: int) -> ImageFont.ImageFont:
    for path in [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    ]:
        if Path(path).is_file():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def _has_usable_bbox(obj: SceneObject) -> bool:
    bbox = obj.bbox_2d
    width = bbox.x2 - bbox.x1
    height = bbox.y2 - bbox.y1
    if width <= 0 or height <= 0:
        return False
    if width >= 0.98 and height >= 0.98:
        return False
    return True


def _coarse_region_box(
    hint: str | None,
    image_size: tuple[int, int],
) -> tuple[int, int, int, int] | None:
    if not hint:
        return None
    width, height = image_size
    value = hint.lower()
    if "left" in value:
        x1, x2 = 0, width // 3
    elif "right" in value:
        x1, x2 = width * 2 // 3, width
    else:
        x1, x2 = width // 3, width * 2 // 3
    if "upper" in value:
        y1, y2 = 0, height // 2
    elif "lower" in value or "foreground" in value:
        y1, y2 = height // 2, height
    elif "background" in value:
        y1, y2 = 0, height * 2 // 3
    else:
        y1, y2 = 0, height
    return int(x1), int(y1), int(x2), int(y2)


def _draw_dashed_rectangle(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    color: str,
    width: int,
) -> None:
    x1, y1, x2, y2 = box
    dash = max(10, width * 4)
    for start in range(x1, x2, dash * 2):
        draw.line((start, y1, min(start + dash, x2), y1), fill=color, width=width)
        draw.line((start, y2, min(start + dash, x2), y2), fill=color, width=width)
    for start in range(y1, y2, dash * 2):
        draw.line((x1, start, x1, min(start + dash, y2)), fill=color, width=width)
        draw.line((x2, start, x2, min(start + dash, y2)), fill=color, width=width)


def _draw_region_label(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    label: str,
    color: str,
    font: ImageFont.ImageFont,
) -> None:
    x1, y1, x2, _ = box
    text_box = draw.textbbox((0, 0), label, font=font)
    text_width = min(x2 - x1, text_box[2] - text_box[0] + 8)
    text_height = text_box[3] - text_box[1] + 6
    draw.rectangle(
        (x1, y1, x1 + text_width, y1 + text_height),
        fill=_rgba(color, 220),
    )
    draw.text((x1 + 4, y1 + 3), label, fill="white", font=font)


def _rgba(color: str, alpha: int) -> tuple[int, int, int, int]:
    value = color.lstrip("#")
    return (
        int(value[0:2], 16),
        int(value[2:4], 16),
        int(value[4:6], 16),
        alpha,
    )
