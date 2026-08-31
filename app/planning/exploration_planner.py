"""Small helpers for ordering exploration targets."""

from __future__ import annotations

from app.schemas import PredictiveSceneGraph, SceneHypothesis


def verification_targets(
    hypotheses: list[SceneHypothesis],
    psg: PredictiveSceneGraph | None = None,
) -> list[str]:
    targets = [hypothesis.possible_location for hypothesis in hypotheses]
    if psg is not None:
        targets.extend(psg.recommended_verification_targets)
    return _dedupe(targets)


def _dedupe(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        if not value or value in seen:
            continue
        seen.add(value)
        result.append(value)
    return result
