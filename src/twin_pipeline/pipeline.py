"""Run the stages in order, write one result.json per stage and run.json for the whole run.

Rules (docs/stage-contract.md):
  * failed / error stops the run; needs_review stops it until the user approves on a resumed run;
  * skipped does not stop it (later stages see the missing outputs and decide for themselves);
  * on --resume, stages that passed are kept as long as nothing before them was re-run.
"""

from __future__ import annotations

import json
import platform
import shutil
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

from . import __version__
from .result import STATUSES, StageResult, read_result, write_result
from .spec import TwinSpec
from .stage import Stage, StageContext

RUN_SCHEMA = 1
EXIT_CODES = {"succeeded": 0, "partial": 0, "failed": 1, "needs_review": 2, "incomplete": 3}


def now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _new_run_dir(spec: TwinSpec, runs_dir: Path | None) -> Path:
    root = Path(runs_dir) if runs_dir else spec.dir / "runs"
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    candidate, index = root / spec.robot / stamp, 1
    while candidate.exists():
        index += 1
        candidate = root / spec.robot / f"{stamp}-{index}"
    return candidate


def _execute(stage: Stage, context: StageContext) -> StageResult:
    if context.stage_dir.exists():
        shutil.rmtree(context.stage_dir)
    context.stage_dir.mkdir(parents=True)
    started, clock = now(), time.monotonic()
    try:
        result = stage.run(context)
        if not isinstance(result, StageResult) or result.stage != stage.name or result.status not in STATUSES:
            raise TypeError(f"stage {stage.name} returned {result!r}; expected a StageResult for this stage with a status in {STATUSES}")
    except Exception as error:  # a crashing stage is reported, not propagated
        (context.stage_dir / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
        result = StageResult(stage.name, "error", reason=f"{type(error).__name__}: {error}",
                             errors=[{"code": "STAGE_CRASHED", "message": f"{type(error).__name__}: {error}"}],
                             outputs={"traceback": context.rel(context.stage_dir / "traceback.txt")})
    result.passed = result.status == "succeeded"
    result.started_at, result.finished_at = started, now()
    result.duration_seconds = round(time.monotonic() - clock, 3)
    return result


def _entry(result: StageResult | None, name: str, *, reused: bool = False) -> dict:
    if result is None:
        return {"name": name, "status": "not_run", "passed": False}
    row = {"name": name, "status": result.status, "passed": result.passed, "result": f"{name}/result.json"}
    if result.reason:
        row["reason"] = result.reason
    if result.status == "skipped":
        row["applicable"] = bool(result.applicable)
    if result.approved:
        row["approved"] = True
    if reused:
        row["reused"] = True
    return row


def _overall(entries: list[dict], subset: bool) -> str:
    statuses = [row["status"] for row in entries]
    if "failed" in statuses or "error" in statuses:
        return "failed"
    if any(row["status"] == "needs_review" and not row["passed"] for row in entries):
        return "needs_review"
    if any(row["status"] == "skipped" and row.get("applicable") for row in entries):
        return "incomplete"
    if "not_run" in statuses:
        return "partial" if subset else "incomplete"
    return "succeeded"


def run_pipeline(spec: TwinSpec, *, runs_dir: Path | None = None, only: list[str] | None = None, resume: Path | None = None,
                 approve: list[str] | tuple = (), options: dict | None = None, stages: list[Stage] | None = None) -> dict:
    """Run the pipeline for one twin and return the run summary (also written to run.json)."""
    if stages is None:
        from .stages import all_stages
        stages = all_stages()
    names = [stage.name for stage in stages]
    for label, requested in (("--stages", only or []), ("--approve", approve)):
        unknown = [name for name in requested if name not in names]
        if unknown:
            raise ValueError(f"{label}: unknown stage(s) {', '.join(unknown)}; stages are {', '.join(names)}")
    if approve and resume is None:
        raise ValueError("--approve needs --resume: approve a stage after reading the review items of an existing run")

    previous: dict[str, StageResult] = {}
    if resume is not None:
        run_dir = Path(resume).resolve()
        summary_path = run_dir / "run.json"
        if not summary_path.exists():
            raise ValueError(f"--resume: {run_dir} has no run.json")
        old = json.loads(summary_path.read_text(encoding="utf-8"))
        if old.get("spec_sha256") != spec.sha256:
            raise ValueError(f"--resume: twin.yaml changed since run {run_dir.name}; start a new run instead")
        for name in names:
            path = run_dir / name / "result.json"
            if path.exists():
                previous[name] = read_result(path)
        started = old.get("started_at") or now()
    else:
        run_dir = _new_run_dir(spec, runs_dir).resolve()
        run_dir.mkdir(parents=True)
        shutil.copy2(spec.path, run_dir / "twin.yaml")
        started = now()

    results: dict[str, StageResult] = {}
    entries: list[dict] = []
    stopped_at, rerun = None, False
    for stage in stages:
        if stopped_at is not None:
            entries.append(_entry(None, stage.name))
            continue
        prior = previous.get(stage.name)
        if only and stage.name not in only:
            if prior is not None and prior.passed and not rerun:
                results[stage.name] = prior
                entries.append(_entry(prior, stage.name, reused=True))
            else:
                entries.append(_entry(None, stage.name))
            continue
        if prior is not None and not rerun:
            if prior.passed:
                results[stage.name] = prior
                entries.append(_entry(prior, stage.name, reused=True))
                continue
            if prior.status == "needs_review" and stage.name in approve:
                prior.passed, prior.approved, prior.approved_at = True, True, now()
                write_result(run_dir / stage.name / "result.json", prior)
                results[stage.name] = prior
                entries.append(_entry(prior, stage.name, reused=True))
                continue
        rerun = True
        context = StageContext(spec, run_dir, run_dir / stage.name, results, dict(options or {}))
        result = _execute(stage, context)
        write_result(run_dir / stage.name / "result.json", result)
        results[stage.name] = result
        entries.append(_entry(result, stage.name))
        if result.status in ("failed", "error") or (result.status == "needs_review" and not result.passed):
            stopped_at = stage.name

    status = _overall(entries, subset=bool(only))
    summary = {
        "schema_version": RUN_SCHEMA,
        "robot": spec.robot,
        "spec_path": str(spec.path),
        "spec_sha256": spec.sha256,
        "run_dir": str(run_dir),
        "status": status,
        "passed": status == "succeeded",
        "stopped_at": stopped_at,
        "started_at": started,
        "finished_at": now(),
        "stages": entries,
        "environment": {"twin_pipeline": __version__, "python": sys.version.split()[0], "platform": platform.platform()},
    }
    (run_dir / "run.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary
