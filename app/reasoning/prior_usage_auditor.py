"""Runtime audit report for handcrafted-prior-free search."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from app.config import Settings, get_settings


def build_prior_usage_report(
    *,
    settings: Settings | None = None,
    llm_prior: dict[str, Any] | None = None,
    dynamic_prompts: dict[str, Any] | None = None,
    evidence_report: dict[str, Any] | None = None,
    observation_memory_used: bool = False,
    static_kb_used: bool = False,
    handcrafted_priors_used: bool = False,
    target_confirmed_by: str | None = None,
) -> dict[str, Any]:
    config = settings or get_settings()
    violations = []
    if handcrafted_priors_used:
        violations.append(
            {
                "type": "handcrafted_prior",
                "message": "Runtime used handcrafted search prior.",
            }
        )
    if static_kb_used:
        violations.append(
            {"type": "static_kb", "message": "Runtime used static scene KB."}
        )
    if dynamic_prompts and dynamic_prompts.get("handwritten_prompt_used"):
        violations.append(
            {
                "type": "static_object_prompt",
                "message": "Runtime used static detector prompt table.",
            }
        )
    if llm_prior and llm_prior.get("can_confirm_target") is not False:
        violations.append(
            {
                "type": "llm_prior_can_confirm_target",
                "message": "LLM prior result attempted to confirm target.",
            }
        )
    return {
        "handcrafted_priors_used": handcrafted_priors_used,
        "static_kb_used": static_kb_used,
        "static_object_prompts_used": bool(
            dynamic_prompts and dynamic_prompts.get("handwritten_prompt_used")
        ),
        "llm_runtime_commonsense_used": bool(
            llm_prior and llm_prior.get("enabled") and llm_prior.get("available")
        ),
        "observation_memory_used": observation_memory_used,
        "visual_evidence_required_for_confirmation": (
            config.target_confirmation_require_visual_evidence
        ),
        "llm_prior_can_confirm_target": False,
        "target_confirmed_by": target_confirmed_by
        or (
            "visual_evidence_gate"
            if evidence_report and evidence_report.get("target_found")
            else "not_confirmed"
        ),
        "violations": violations,
        "passed": not violations,
    }


def write_prior_usage_report(payload: dict[str, Any], path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output
