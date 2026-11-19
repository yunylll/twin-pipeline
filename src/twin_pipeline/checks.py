"""Small validators shared by the spec core and the stage sections.

Every function returns a list of messages; an empty list means the value is fine. Messages name the
field, what was expected and what was found, e.g. ``joints.elbow.lower: expected a finite number, got 'a'``.
"""

from __future__ import annotations

import math
from typing import Any, Iterable


def show(value: Any) -> str:
    text = repr(value)
    return text if len(text) <= 60 else text[:57] + "..."


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def mapping(where: str, value: Any) -> list[str]:
    return [] if isinstance(value, dict) else [f"{where}: expected a mapping, got {show(value)}"]


def unknown_keys(where: str, value: dict, allowed: Iterable[str]) -> list[str]:
    allowed = set(allowed)
    extra = sorted(str(key) for key in set(value) - allowed)
    if not extra:
        return []
    return [f"{where}: unknown key(s) {', '.join(map(repr, extra))}; allowed: {', '.join(sorted(allowed))}"]


def number(where: str, value: Any, *, low: float | None = None, high: float | None = None, low_open: bool = False) -> list[str]:
    if not is_number(value):
        return [f"{where}: expected a finite number, got {show(value)}"]
    if low is not None and (value <= low if low_open else value < low):
        return [f"{where}: expected a number {'>' if low_open else '>='} {low}, got {value}"]
    if high is not None and value > high:
        return [f"{where}: expected a number <= {high}, got {value}"]
    return []


def integer(where: str, value: Any, *, low: int | None = None) -> list[str]:
    if not isinstance(value, int) or isinstance(value, bool):
        return [f"{where}: expected an integer, got {show(value)}"]
    if low is not None and value < low:
        return [f"{where}: expected an integer >= {low}, got {value}"]
    return []


def boolean(where: str, value: Any) -> list[str]:
    return [] if isinstance(value, bool) else [f"{where}: expected true or false, got {show(value)}"]


def string(where: str, value: Any) -> list[str]:
    return [] if isinstance(value, str) and value.strip() else [f"{where}: expected a non-empty string, got {show(value)}"]


def choice(where: str, value: Any, options: Iterable[str]) -> list[str]:
    options = tuple(options)
    return [] if value in options else [f"{where}: expected one of {', '.join(options)}, got {show(value)}"]
