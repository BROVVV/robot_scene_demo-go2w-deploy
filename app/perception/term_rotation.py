"""Fair prompt rotation for the remote semantic-mapping worker."""
from __future__ import annotations

from typing import Any, Iterable


class RotatingTermScheduler:
    """Keep safety terms resident while giving every scene group a turn."""

    def __init__(self, config: dict[str, Any] | None, fallback: Iterable[str] = ()) -> None:
        settings = config or {}
        self.max_terms = max(1, int(settings.get("max_terms_per_round", 16)))
        self.rotate_every = max(1, int(settings.get("rotate_every_rounds", 1)))
        self.priority = _unique(settings.get("priority_terms", []))
        self.profiles = settings.get("profiles", {}) if isinstance(settings.get("profiles", {}), dict) else {}
        self.fallback = _unique(fallback)
        self._round = 0
        self._offsets: dict[str, int] = {}
        self.last_terms: list[str] = []
        self.last_group_index: int | None = None

    def next_terms(self, profile: str, extra_priority: Iterable[str] = ()) -> list[str]:
        priorities = _unique([*extra_priority, *self.priority])
        budget = max(0, self.max_terms - len(priorities))
        configured = self.profiles.get(profile, {})
        raw_groups = configured.get("groups", []) if isinstance(configured, dict) else []
        groups = [_unique(group) for group in raw_groups if isinstance(group, (list, tuple))]
        if not groups:
            groups = [self.fallback]

        # Rotate over the flattened grouped vocabulary so the tail of a group
        # also gets scheduled when that group alone is larger than the budget.
        ordinary = _unique(term for group in groups for term in group if term not in priorities)
        start = self._offsets.get(profile, 0) % max(1, len(ordinary))
        selected = [ordinary[(start + index) % len(ordinary)]
                    for index in range(min(budget, len(ordinary)))] if ordinary else []
        consumed = 0
        self.last_group_index = 0
        for index, group in enumerate(groups):
            next_consumed = consumed + len([term for term in group if term not in priorities])
            if consumed <= start < next_consumed:
                self.last_group_index = index
                break
            consumed = next_consumed
        self.last_terms = _unique([*priorities, *selected])[: self.max_terms]
        self._round += 1
        if self._round % self.rotate_every == 0:
            self._offsets[profile] = (start + max(1, budget)) % max(1, len(ordinary))
        return list(self.last_terms)


def _unique(values: Iterable[Any]) -> list[str]:
    out: list[str] = []
    for value in values:
        term = str(value).strip()
        if term and term not in out:
            out.append(term)
    return out
