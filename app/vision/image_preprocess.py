"""Non-destructive image resize used by optional preprocessing workflows."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageFilter


@dataclass(frozen=True)
class PreparedImage:
    path: Path
    original_width: int
    original_height: int
    width: int
    height: int
    scale: float


def load_and_prepare_image(
    path: str | Path,
    output_path: str | Path,
    max_side: int = 1280,
    sharpen: bool = False,
    denoise: bool = False,
) -> PreparedImage:
    source = Path(path)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(source) as opened:
        image = opened.convert("RGB")
        original_width, original_height = image.size
        scale = min(1.0, max_side / max(image.size))
        if scale < 1.0:
            image = image.resize(
                (
                    max(1, round(original_width * scale)),
                    max(1, round(original_height * scale)),
                ),
                Image.Resampling.LANCZOS,
            )
        if denoise:
            image = image.filter(ImageFilter.MedianFilter(size=3))
        if sharpen:
            image = image.filter(ImageFilter.SHARPEN)
        image.save(output, format="JPEG", quality=92)
    return PreparedImage(
        path=output,
        original_width=original_width,
        original_height=original_height,
        width=image.width,
        height=image.height,
        scale=scale,
    )
