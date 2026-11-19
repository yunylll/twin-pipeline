"""report stage: one self-contained HTML page per run (stage results, metrics, provenance, changes since the last run).

Owner: session B (docs/ROADMAP.md). Section owned: report. Output: ``html`` (report/report.html).

The page covers the stages that ran before this one. The stage is marked ``always``, so it also runs
after a failed or unapproved stage stopped the run and shows the run as stopped there. To redraw the
page of any run from disk later, use ``python -m twin_pipeline.report <run folder>`` (see twin_pipeline/report).
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
    from ..report import write_for_context

    path, data = write_for_context(context)
    assumed = sum(1 for row in data.overrides if row["source"] in ("assumed", "unspecified"))
    assumptions = sum(1 for row in data.assumptions if row["source"] in ("assumed", "unspecified"))
    metrics = {"stages_reported": len(data.stages) - 1, "assumed_values": assumed, "assumptions_assumed": assumptions,
               "urdf_values_without_provenance": len(data.urdf_values), "previous_run": data.previous["run"] if data.previous else None}
    return StageResult.succeeded("report", outputs={"html": context.rel(path)}, metrics=metrics, warnings=list(data.notes))


STAGES = [Stage("report", run, "Write an HTML report of the run", always=True)]
SECTIONS = {"report": validate_report}
URDF_CHECKS = []
