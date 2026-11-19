"""CAD stages: detect (did the CAD change?), export (solidworks-urdf-tool), ingest (archive the package).

Owner: phase 2 (integration). Placeholders for now: they validate the cad section and report
skipped, so a twin with a cad section shows as incomplete rather than passing silently.
Section owned: cad.
"""

from __future__ import annotations

from .. import checks
from ..result import StageResult
from ..stage import Stage, StageContext

CAD_KEYS = ("model", "config", "package", "tool", "archive", "timeout_seconds", "stall_timeout_seconds")


def validate_cad(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, CAD_KEYS)
    errors += checks.string(f"{where}.model", value.get("model"))
    for key in ("config", "package", "tool", "archive"):
        if key in value:
            errors += checks.string(f"{where}.{key}", value[key])
    for key in ("timeout_seconds", "stall_timeout_seconds"):
        if key in value:
            errors += checks.integer(f"{where}.{key}", value[key], low=1)
    return errors


def _placeholder(name):
    def run(context: StageContext) -> StageResult:
        if "cad" not in context.spec.data:
            return StageResult.skipped(name, "no cad section: the URDF is given directly", applicable=False)
        return StageResult.skipped(name, f"{name} is not implemented yet (phase 2, see docs/ROADMAP.md)")
    return run


STAGES = [
    Stage("detect", _placeholder("detect"), "Compare the CAD model with the version the export config was verified on"),
    Stage("export", _placeholder("export"), "Check and export the URDF with solidworks-urdf-tool"),
    Stage("ingest", _placeholder("ingest"), "Archive the exported package next to the twin, keeping earlier exports"),
]
SECTIONS = {"cad": validate_cad}
URDF_CHECKS = []
