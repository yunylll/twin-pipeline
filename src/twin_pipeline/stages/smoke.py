"""smoke stage: headless checks on the MuJoCo model (load, home pose, joint sweep, stability).

Owner: session B (docs/ROADMAP.md). Placeholder: validates its section and reports skipped.
Section owned: smoke.
Outputs when implemented: smoke (the metrics JSON); see docs/stage-contract.md.
"""

from __future__ import annotations

from .. import checks
from ..result import StageResult
from ..stage import Stage, StageContext

SMOKE_KEYS = ("sweep_fraction", "settle_seconds", "timestep")


def validate_smoke(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, SMOKE_KEYS)
    if "sweep_fraction" in value:
        errors += checks.number(f"{where}.sweep_fraction", value["sweep_fraction"], low=0, low_open=True, high=1)
    for key in ("settle_seconds", "timestep"):
        if key in value:
            errors += checks.number(f"{where}.{key}", value[key], low=0, low_open=True)
    return errors


def run(context: StageContext) -> StageResult:
    if context.output("mjcf", "mjcf") is None:
        return StageResult.skipped("smoke", "no MJCF: the mjcf stage did not pass in this run")
    return StageResult.skipped("smoke", "smoke is not implemented yet (session B, see docs/ROADMAP.md)")


STAGES = [Stage("smoke", run, "Load the MuJoCo model headless and check home pose, joint sweep and stability")]
SECTIONS = {"smoke": validate_smoke}
URDF_CHECKS = []
