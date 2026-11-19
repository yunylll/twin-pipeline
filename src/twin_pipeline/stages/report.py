"""report stage: one HTML page per run (stage results, metrics, assumptions, changes since the last run).

Owner: session B (docs/ROADMAP.md). Placeholder: validates its section and reports skipped.
Section owned: report.
Outputs when implemented: html (the report page); see docs/stage-contract.md.
"""

from __future__ import annotations

from .. import checks
from ..result import StageResult
from ..stage import Stage, StageContext


def validate_report(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, ("title",))
    if "title" in value:
        errors += checks.string(f"{where}.title", value["title"])
    return errors


def run(context: StageContext) -> StageResult:
    return StageResult.skipped("report", "report is not implemented yet (session B, see docs/ROADMAP.md)")


STAGES = [Stage("report", run, "Write an HTML report of the run")]
SECTIONS = {"report": validate_report}
URDF_CHECKS = []
