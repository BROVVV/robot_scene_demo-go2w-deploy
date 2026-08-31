"""Natural-language task understanding for the robot scene demo.

The package is imported by the ROS/system-Python search worker even when the
worker receives a task context that has already been parsed by the WebUI.
Keep the LLM parser imports lazy: the worker uses the ROS Python for rclpy and
the vision subprocess uses the Conda Python for OpenAI, so importing the
package must not require OpenAI in the ROS interpreter just to access the
lightweight :class:`SearchTaskContext` dataclass.
"""

from typing import Any

from app.task_understanding.capability_gate import (
    evaluate_actionability,
    evaluate_capability_and_safety,
)
from app.task_understanding.navigation_router import build_navigation_task
from app.task_understanding.search_task_context import SearchTaskContext


def parse_natural_language_task(*args: Any, **kwargs: Any) -> Any:
    from app.task_understanding.intent_parser import parse_natural_language_task as parse

    return parse(*args, **kwargs)


def run_task_understanding_pipeline(*args: Any, **kwargs: Any) -> Any:
    from app.task_understanding.task_pipeline import run_task_understanding_pipeline as run

    return run(*args, **kwargs)


def prepare_navigation_task_from_text(*args: Any, **kwargs: Any) -> Any:
    from app.task_understanding.task_pipeline import prepare_navigation_task_from_text as prepare

    return prepare(*args, **kwargs)


def write_task_understanding_outputs(*args: Any, **kwargs: Any) -> Any:
    from app.task_understanding.task_pipeline import write_task_understanding_outputs as write

    return write(*args, **kwargs)

__all__ = [
    "build_navigation_task",
    "evaluate_actionability",
    "evaluate_capability_and_safety",
    "parse_natural_language_task",
    "prepare_navigation_task_from_text",
    "run_task_understanding_pipeline",
    "SearchTaskContext",
    "write_task_understanding_outputs",
]
