"""The stage result contract: every stage writes ``<run>/<stage>/result.json``. See docs/stage-contract.md."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

RESULT_SCHEMA = 1
STATUSES = ("succeeded", "failed", "skipped", "needs_review", "error")


@dataclass
class StageResult:
    """What one stage did.

    ``passed`` is set by the runner, never by a stage: True for succeeded, and for needs_review once
    the user approved it on a resumed run. A stage reports problems through ``status`` and ``errors``.
    """

    stage: str
    status: str
    passed: bool = False
    reason: str | None = None
    applicable: bool | None = None
    outputs: dict[str, str] = field(default_factory=dict)
    metrics: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    errors: list[dict] = field(default_factory=list)
    review: list[dict] = field(default_factory=list)
    approved: bool = False
    approved_at: str | None = None
    started_at: str | None = None
    finished_at: str | None = None
    duration_seconds: float | None = None
    schema_version: int = RESULT_SCHEMA

    @classmethod
    def succeeded(cls, stage, *, outputs=None, metrics=None, warnings=None, reason=None):
        return cls(stage, "succeeded", reason=reason, outputs=dict(outputs or {}), metrics=dict(metrics or {}), warnings=list(warnings or []))

    @classmethod
    def failed(cls, stage, code, message, *, outputs=None, metrics=None, warnings=None, errors=None):
        rows = [{"code": code, "message": message}] + list(errors or [])
        return cls(stage, "failed", reason=message, outputs=dict(outputs or {}), metrics=dict(metrics or {}), warnings=list(warnings or []), errors=rows)

    @classmethod
    def skipped(cls, stage, reason, *, applicable=True, warnings=None):
        """applicable=False: the stage has nothing to do for this twin (e.g. no cad section).
        applicable=True: it should have run but could not (missing input, not implemented yet)."""
        return cls(stage, "skipped", reason=reason, applicable=applicable, warnings=list(warnings or []))

    @classmethod
    def needs_review(cls, stage, items, *, outputs=None, metrics=None, warnings=None, reason=None):
        """items: [{"item": what to look at, "reason": why a person has to decide}]."""
        return cls(stage, "needs_review", reason=reason or f"{len(items)} item(s) need review", outputs=dict(outputs or {}),
                   metrics=dict(metrics or {}), warnings=list(warnings or []), review=list(items))

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "StageResult":
        known = {key: value for key, value in data.items() if key in cls.__dataclass_fields__}
        return cls(**known)


def write_result(path: Path, result: StageResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(result.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_result(path: Path) -> StageResult:
    return StageResult.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
