"""CAD stages, through solidworks-urdf-tool (https://github.com/yunylll/solidworks-urdf-tool).

Section owned: cad (see docs/twin-yaml.md).

detect  Reads files only, never starts SolidWorks. Checks the export configuration against the model
        (``tool_cli.py validate-config --model``) and decides whether a new export is needed: the model
        file, every CAD file the archived export read, and the configuration are compared with the
        archived export. Unchanged -> the archived URDF is reused and no SolidWorks job runs.
export  Only when detect says so (or ``--option force_export=true``): a check-only job first (minutes,
        catches configuration and CAD-version problems before the long export), then the export job,
        published by the tool into the archive (<archive>/urdf/<package>/, <archive>/export/, earlier
        files moved to <archive>/export/history/). Waits for both jobs.
ingest  Makes the archived package the URDF of the run. After a new export it stops with
        needs_review so a person looks at the tool's preview renders before the twin uses the model
        (``twin run --resume <run> --approve ingest``); a reused export passes straight through.

The archive is the one place a stage writes outside its own folder: that is the twin's own
directory, and the tool, not this stage, writes there.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from .. import checks
from ..result import StageResult
from ..spec import check_against_urdf
from ..stage import Stage, StageContext
from ..urdf import read_urdf

CAD_KEYS = ("model", "config", "package", "tool", "archive", "check_first", "use_saved_files", "timeout_seconds", "stall_timeout_seconds")
TOOL_ENV = "TWIN_SW_URDF_TOOL"
TERMINAL = {"succeeded", "failed", "timed_out", "interrupted", "rejected", "cancelled"}
CAD_SUFFIXES = {".sldasm", ".sldprt", ".slddrw"}
DEFAULT_TIMEOUT = 14400
WAIT_STEP = 300  # seconds per `tool_cli.py job --wait` call


def validate_cad(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, CAD_KEYS)
    errors += checks.string(f"{where}.model", value.get("model"))
    for key in ("config", "package", "tool", "archive"):
        if key in value:
            errors += checks.string(f"{where}.{key}", value[key])
    for key in ("check_first", "use_saved_files"):
        if key in value:
            errors += checks.boolean(f"{where}.{key}", value[key])
    for key in ("timeout_seconds", "stall_timeout_seconds"):
        if key in value:
            errors += checks.integer(f"{where}.{key}", value[key], low=1)
    return errors


class ToolError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code, self.message = code, message


# ---- twin.yaml helpers ----

def _cad(spec) -> dict:
    return spec.section("cad")


def package_name(spec) -> str:
    return _cad(spec).get("package") or f"{spec.robot}_description"


def archive_dir(spec) -> Path:
    value = _cad(spec).get("archive")
    return spec.resolve(value) if value else spec.dir


def _flag(options: dict, key: str) -> bool:
    return str(options.get(key, "")).strip().lower() in ("1", "true", "yes", "on")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        return None


def _without_source(config):
    """A configuration as it decides the export: the tool's source block only records where it was verified."""
    if not isinstance(config, dict):
        return config
    return {key: value for key, value in config.items() if key != "source"}


# ---- running the tool ----

def tool_dir(context: StageContext) -> Path:
    value = context.options.get("tool") or _cad(context.spec).get("tool") or os.environ.get(TOOL_ENV)
    if not value:
        raise ToolError("TOOL_NOT_CONFIGURED", f"where solidworks-urdf-tool is: set cad.tool in twin.yaml, the environment variable {TOOL_ENV}, or --option tool=<folder>")
    path = context.spec.resolve(value)
    if not (path / "scripts" / "tool_cli.py").is_file():
        raise ToolError("TOOL_NOT_FOUND", f"expected solidworks-urdf-tool at {path} (with scripts/tool_cli.py); found no such file")
    return path


def tool_python(tool: Path) -> str:
    for candidate in (tool / ".venv" / "Scripts" / "python.exe", tool / ".venv" / "bin" / "python"):
        if candidate.is_file():
            return str(candidate)
    return sys.executable


def run_tool(context: StageContext, tool: Path, args: list[str], *, log: str, timeout: float) -> dict:
    """One tool_cli.py call; its JSON answer is kept as <stage>/<log>.json. Exit codes are not used: the JSON says."""
    command = [tool_python(tool), str(tool / "scripts" / "tool_cli.py"), *args]
    try:
        process = subprocess.run(command, cwd=tool, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout)
    except subprocess.TimeoutExpired:
        raise ToolError("TOOL_TIMEOUT", f"tool_cli.py {args[0]} did not answer within {timeout:.0f} s") from None
    (context.stage_dir / f"{log}.json").write_text(process.stdout, encoding="utf-8")
    if process.stderr.strip():
        (context.stage_dir / f"{log}.stderr.log").write_text(process.stderr, encoding="utf-8")
    try:
        return json.loads(process.stdout)
    except ValueError:
        tail = (process.stderr.strip() or process.stdout.strip())[-600:]
        raise ToolError("TOOL_OUTPUT_INVALID", f"tool_cli.py {args[0]} exited {process.returncode} without a JSON answer: {tail}") from None


def run_job(context: StageContext, tool: Path, start_args: list[str], *, label: str, timeout: float) -> dict:
    """Start a tool job, then wait for it in WAIT_STEP slices until it is terminal; returns its final record."""
    started = run_tool(context, tool, start_args, log=f"{label}-start", timeout=300)
    job_id = started.get("job_id")
    if not job_id:
        error = started.get("error") or {}
        raise ToolError(error.get("code") or "TOOL_REJECTED", error.get("message") or f"the tool did not start the {label} job")
    (context.stage_dir / f"{label}-job-id.txt").write_text(job_id + "\n", encoding="utf-8")
    step = int(context.options.get("wait_step", WAIT_STEP))
    deadline = time.monotonic() + timeout + 600
    while True:
        state = run_tool(context, tool, ["job", job_id, "--wait", str(step)], log=f"{label}-job", timeout=step + 300)
        if state.get("status") in TERMINAL:
            return state
        if time.monotonic() > deadline:
            raise ToolError("TOOL_TIMEOUT", f"{label} job {job_id} is still {state.get('status')} after {timeout:.0f} s; it keeps running in the tool (tool_cli.py job {job_id})")


def _job_failure(stage: str, label: str, state: dict, outputs: dict) -> StageResult:
    error = state.get("error") or {}
    code = error.get("code") or f"{label.upper()}_{str(state.get('status')).upper()}"
    message = error.get("message") or f"the {label} job ended {state.get('status')} with passed={state.get('passed')}"
    return StageResult.failed(stage, code, f"{label} job {state.get('job_id')}: {message}", outputs=outputs,
                              warnings=list(state.get("warnings") or []), metrics={"job_id": state.get("job_id"), "status": state.get("status")})


# ---- the archived export ----

def archived_export(spec) -> dict:
    """What the archive holds: the export record, its configuration and the URDF (each None if absent)."""
    archive = archive_dir(spec)
    export = archive / "export"
    package = archive / "urdf" / package_name(spec)
    urdfs = sorted((package / "urdf").glob("*.urdf")) if (package / "urdf").is_dir() else []
    return {"result": _read_json(export / "export-result.json"), "config": _read_json(export / "export-config.json"),
            "package": package if urdfs else None, "urdf": urdfs[0] if urdfs else None}


def _current_digests(folder: Path) -> set[str]:
    return {sha256(path) for path in folder.rglob("*") if path.is_file() and path.suffix.lower() in CAD_SUFFIXES}


def export_reasons(spec, model: Path, config_path: Path | None, archived: dict) -> list[str]:
    """Why a new export is needed; an empty list means the archived export still matches the CAD and configuration."""
    record = archived["result"]
    if archived["urdf"] is None:
        return [f"no archived URDF in {archive_dir(spec) / 'urdf' / package_name(spec)}"]
    if not isinstance(record, dict) or record.get("status") != "succeeded" or record.get("passed") is not True:
        return ["the archive has no passed export record (export/export-result.json)"]
    reasons = []
    # cad_version_sha256 is the original model's hash when the export ran on a prepared copy
    if (record.get("cad_version_sha256") or record.get("source_model_sha256")) != sha256(model):
        reasons.append("the model file changed since the archived export")
    before = record.get("source_hashes_before") or {}
    if before:
        present = _current_digests(model.parent)
        missing = sum(1 for digest in before.values() if digest not in present)
        if missing:
            reasons.append(f"{missing} of {len(before)} CAD files the archived export read are not under {model.parent} with the same content (edited, moved or outside the model folder)")
    else:
        reasons.append("the archived export did not record the hashes of the CAD files it read")
    if config_path is not None:
        if _without_source(_read_json(config_path)) != _without_source(archived["config"]):
            reasons.append("the export configuration differs from the archived one")
    elif archived["config"] is not None:
        reasons.append("twin.yaml has no cad.config but the archived export used a JSON configuration")
    return reasons


# ---- stages ----

def _not_applicable(name):
    return StageResult.skipped(name, "no cad section: the URDF is given directly", applicable=False)


def detect(context: StageContext) -> StageResult:
    spec = context.spec
    if "cad" not in spec.data:
        return _not_applicable("detect")
    cad = _cad(spec)
    model = spec.resolve(cad["model"])
    if not model.is_file():
        return StageResult.failed("detect", "MODEL_NOT_FOUND", f"cad.model: expected a SolidWorks file at {model}; found none")
    config_path = spec.resolve(cad["config"]) if cad.get("config") else None
    if config_path is not None and not config_path.is_file():
        return StageResult.failed("detect", "CONFIG_NOT_FOUND", f"cad.config: expected the export configuration at {config_path}; found none")
    metrics = {"model": str(model), "model_sha256": sha256(model), "archive": str(archive_dir(spec)), "package": package_name(spec)}
    outputs, warnings = {}, []
    if config_path is not None:
        try:
            checked = run_tool(context, tool_dir(context), ["validate-config", str(config_path), "--model", str(model)], log="validate-config", timeout=600)
        except ToolError as error:
            return StageResult.failed("detect", error.code, error.message)
        outputs["validation"] = context.rel(context.stage_dir / "validate-config.json")
        if not checked.get("passed"):
            problems = [e if isinstance(e, str) else (e.get("message") or json.dumps(e, ensure_ascii=False)) for e in checked.get("errors") or []]
            message = "; ".join(problems) or (checked.get("error") or {}).get("message") or "the tool rejected the configuration"
            return StageResult.failed("detect", "CONFIG_INVALID", f"{config_path.name}: {message}", outputs=outputs, metrics=metrics)
        status = checked.get("source_status") or {}
        metrics["config_stale"] = bool(status.get("stale"))
        warnings += [str(w) for w in checked.get("warnings") or []]
    archived = archived_export(spec)
    reasons = export_reasons(spec, model, config_path, archived)
    if archived["result"]:
        metrics["archived_job"] = archived["result"].get("job_id")
    metrics["export_needed"] = bool(reasons)
    metrics["export_reasons"] = reasons
    reason = ("export needed: " + "; ".join(reasons)) if reasons else f"CAD and configuration match the archived export (job {metrics.get('archived_job')}); it is reused"
    return StageResult.succeeded("detect", outputs=outputs, metrics=metrics, warnings=warnings, reason=reason)


def export(context: StageContext) -> StageResult:
    spec = context.spec
    if "cad" not in spec.data:
        return _not_applicable("export")
    detected = context.results.get("detect")
    if detected is None or not detected.passed:
        return StageResult.skipped("export", "detect did not pass in this run")
    force = _flag(context.options, "force_export")
    if not detected.metrics.get("export_needed") and not force:
        return StageResult.succeeded("export", metrics={"reused": True, "archived_job": detected.metrics.get("archived_job")},
                                     reason="not needed: the archived export matches the CAD and configuration (--option force_export=true exports anyway)")
    cad = _cad(spec)
    try:
        tool = tool_dir(context)
    except ToolError as error:
        return StageResult.failed("export", error.code, error.message)
    model = str(spec.resolve(cad["model"]))
    package = package_name(spec)
    timeout = int(cad.get("timeout_seconds", DEFAULT_TIMEOUT))
    common = ["--timeout", str(timeout)]
    if "stall_timeout_seconds" in cad:
        common += ["--stall-timeout", str(cad["stall_timeout_seconds"])]
    if cad.get("use_saved_files"):
        common.append("--use-saved-files")
    config = ["--config", str(spec.resolve(cad["config"]))] if cad.get("config") else []
    outputs, warnings, metrics = {}, [], {"reused": False, "forced": force and not detected.metrics.get("export_needed")}
    try:
        if config and cad.get("check_first", True):
            check = run_job(context, tool, ["start-export", model, "--package", package, "--check-only", *config, *common], label="check", timeout=timeout)
            outputs["check"] = context.rel(context.stage_dir / "check-job.json")
            metrics["check_job"] = check.get("job_id")
            if check.get("status") != "succeeded" or check.get("passed") is not True:
                return _job_failure("export", "check", check, outputs)
            warnings += [str(w) for w in check.get("warnings") or []]
            reusable = check.get("reusable_model")
            if isinstance(reusable, dict):
                reusable = reusable.get("path") or reusable.get("model_path")
            if isinstance(reusable, str) and Path(reusable).exists():
                model = reusable  # the check's prepared CAD copy: skips opening the original and Pack and Go again
                metrics["reused_prepared_copy"] = True
        state = run_job(context, tool, ["start-export", model, "--package", package, "--publish-to", str(archive_dir(spec)), *config, *common],
                        label="export", timeout=timeout)
    except ToolError as error:
        return StageResult.failed("export", error.code, error.message, outputs=outputs, metrics=metrics, warnings=warnings)
    outputs["job_result"] = context.rel(context.stage_dir / "export-job.json")
    metrics["job_id"] = state.get("job_id")
    if state.get("status") != "succeeded" or state.get("passed") is not True:
        return _job_failure("export", "export", state, outputs)
    warnings += [str(w) for w in state.get("warnings") or []]
    publish = state.get("publish") or {}
    if publish.get("passed") is not True:
        message = (publish.get("error") or {}).get("message") or publish.get("message") or "the tool did not publish the export"
        return StageResult.failed("export", "PUBLISH_FAILED", f"export job {state.get('job_id')} passed but publishing to {archive_dir(spec)} failed: {message}",
                                  outputs=outputs, metrics=metrics, warnings=warnings)
    package_path = Path(publish["package"]["path"])
    urdfs = sorted((package_path / "urdf").glob("*.urdf"))
    if not urdfs:
        return StageResult.failed("export", "PUBLISHED_URDF_MISSING", f"expected a URDF in {package_path / 'urdf'}; found none", outputs=outputs, metrics=metrics)
    outputs.update(package=str(package_path), urdf=str(urdfs[0]))
    if publish.get("history"):
        metrics["previous_export_moved_to"] = publish["history"]
    return StageResult.succeeded("export", outputs=outputs, metrics=metrics, warnings=warnings,
                                 reason=f"exported by job {state.get('job_id')} and published to {package_path}")


def ingest(context: StageContext) -> StageResult:
    spec = context.spec
    if "cad" not in spec.data:
        return _not_applicable("ingest")
    exported = context.results.get("export")
    if exported is None or not exported.passed:
        return StageResult.skipped("ingest", "export did not pass in this run")
    archived = archived_export(spec)
    if archived["urdf"] is None:
        return StageResult.failed("ingest", "ARCHIVE_MISSING", f"expected the URDF package in {archive_dir(spec) / 'urdf' / package_name(spec)}; found no URDF")
    record = archived["result"] or {}
    package, urdf = archived["package"], archived["urdf"]
    metrics = {"job_id": record.get("job_id"), "source_model_sha256": record.get("source_model_sha256"), "new_export": not exported.metrics.get("reused")}
    validation = record.get("validation") or {}
    for key in ("links", "joints", "total_mass_kg"):
        if key in validation:
            metrics[key] = validation[key]
    outputs = {"package": str(package), "urdf": str(urdf)}
    try:
        problems = check_against_urdf(spec, read_urdf(urdf))
    except (OSError, ValueError) as error:
        return StageResult.failed("ingest", "URDF_UNREADABLE", f"{urdf}: {error}", outputs=outputs, metrics=metrics)
    if problems:  # twin.yaml could not be cross-checked at load time: the URDF did not exist yet
        return StageResult.failed("ingest", "SPEC_URDF_MISMATCH", f"twin.yaml does not fit {urdf.name}: " + "; ".join(problems),
                                  outputs=outputs, metrics=metrics, errors=[{"code": "SPEC_URDF_MISMATCH", "message": p} for p in problems])
    previews = sorted((package / "previews").glob("*.png")) if (package / "previews").is_dir() else []
    if previews:
        outputs.update({f"preview_{path.stem}": str(path) for path in previews})
    if exported.metrics.get("reused"):
        return StageResult.succeeded("ingest", outputs=outputs, metrics=metrics, reason=f"reusing the archived export of job {record.get('job_id')}")
    items = [{"item": f"previews of job {record.get('job_id')}",
              "reason": ("a new export: look at " + ", ".join(p.name for p in previews) + f" in {package / 'previews'} (one colour per Link) before the twin uses it"
                         if previews else f"a new export without preview renders: open {urdf} in a URDF viewer before the twin uses it")}]
    items += [{"item": "export warning", "reason": warning} for warning in exported.warnings]
    return StageResult.needs_review("ingest", items, outputs=outputs, metrics=metrics,
                                    reason=f"new export of job {record.get('job_id')}: approve with twin run --resume <run> --approve ingest")


STAGES = [
    Stage("detect", detect, "Check the export configuration against the CAD and decide whether a new export is needed (no SolidWorks)"),
    Stage("export", export, "Check and export the URDF with solidworks-urdf-tool and publish it to the archive"),
    Stage("ingest", ingest, "Use the archived URDF package; after a new export, ask for a look at its previews"),
]
SECTIONS = {"cad": validate_cad}
URDF_CHECKS = []
