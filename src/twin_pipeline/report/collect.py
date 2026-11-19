"""Gather everything the report shows from a run: stage results, smoke numbers, provenance, the previous run.

Two ways in:
  * ``from_context`` while the report stage runs: the results of the stages that ran before it;
  * ``from_run_dir`` afterwards, from run.json and the result.json files, which also covers runs that
    stopped before the report stage (the runner stops at a failed stage).
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from ..result import StageResult, read_result
from ..spec import JOINT_VALUES, TwinSpec
from ..urdf import UrdfModel, read_urdf

URDF_ONLY_VALUES = ("effort", "velocity")  # actuator limits CAD does not know; flagged when twin.yaml does not set them
RUN_NAME = re.compile(r"^(\d{8}-\d{6})(?:-(\d+))?$")


@dataclass
class StageRow:
    name: str
    status: str  # a result status, "not_run", or "running" for the report stage itself
    result: StageResult | None = None
    reused: bool = False
    smoke: dict | None = None  # the parsed smoke.json, for the smoke stage


@dataclass
class ReportData:
    run_dir: Path
    report_dir: Path
    robot: str
    title: str
    status: str
    provisional: bool  # True while the report stage computes the status before the run has finished
    started_at: str | None
    finished_at: str | None
    spec_sha256: str | None
    environment: dict
    stages: list[StageRow]
    overrides: list[dict] = field(default_factory=list)
    urdf_values: list[dict] = field(default_factory=list)
    assumptions: list[dict] = field(default_factory=list)
    home: dict = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)
    previous: dict | None = None


def provenance(spec: TwinSpec, urdf: UrdfModel | None) -> tuple[list[dict], list[dict], list[dict]]:
    """(overrides, urdf_values, assumptions) from twin.yaml, with URDF values for comparison."""
    overrides = []
    for joint, override in (spec.data.get("joints") or {}).items():
        item = urdf.joints.get(joint) if urdf else None
        for name in JOINT_VALUES:
            if name not in override:
                continue
            overrides.append({"joint": joint, "field": name, "value": override[name],
                              "urdf": getattr(item, name, None) if item is not None else None,
                              "source": spec.source_of(joint, name) or "unspecified", "note": override.get("note")})
    urdf_values = []
    if urdf is not None:
        for joint, item in urdf.joints.items():
            if not item.movable:
                continue
            for name in URDF_ONLY_VALUES:
                if spec.source_of(joint, name) is None:
                    urdf_values.append({"joint": joint, "field": name, "value": getattr(item, name)})
    assumptions = [{"id": row.get("id"), "text": row.get("text"), "value": row.get("value"),
                    "source": row.get("source", "unspecified"), "note": row.get("note")} for row in (spec.data.get("assumptions") or [])]
    return overrides, urdf_values, assumptions


def _read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _resolve(run_dir: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else run_dir / path


def _smoke(run_dir: Path, result: StageResult | None) -> dict | None:
    if result is None or "smoke" not in result.outputs:
        return None
    return _read_json(_resolve(run_dir, result.outputs["smoke"]))


def _run_key(path: Path):
    match = RUN_NAME.match(path.name)
    return (match.group(1), int(match.group(2) or 1)) if match else (path.name, 0)


def previous_run(run_dir: Path) -> Path | None:
    """The latest earlier run of the same robot under the same runs root (by run folder name)."""
    here = _run_key(run_dir)
    earlier = [path for path in run_dir.parent.iterdir()
               if path.is_dir() and path != run_dir and RUN_NAME.match(path.name) and _run_key(path) < here and (path / "run.json").exists()]
    return max(earlier, key=_run_key) if earlier else None


def _flat_provenance(overrides, assumptions) -> dict:
    rows = {f"joints.{row['joint']}.{row['field']}": (row["value"], row["source"]) for row in overrides}
    rows.update({f"assumptions.{row['id']}": (row.get("value"), row["source"]) for row in assumptions})
    return rows


def compare(data: ReportData, previous_dir: Path) -> dict:
    """Differences between this run and an earlier one: stage statuses, metrics, twin.yaml values."""
    old_run = _read_json(previous_dir / "run.json") or {}
    old_rows = {row["name"]: row for row in old_run.get("stages", [])}
    stages, metrics = [], []
    for row in data.stages:
        old = old_rows.get(row.name)
        old_status = old["status"] if old else "absent"
        if row.status != "running" and old_status != row.status:
            stages.append({"stage": row.name, "before": old_status, "now": row.status})
        old_result_path = previous_dir / row.name / "result.json"
        if row.result is None or not old_result_path.exists():
            continue
        try:
            old_metrics = read_result(old_result_path).metrics
        except (OSError, ValueError, TypeError):
            continue
        for key in sorted(set(old_metrics) | set(row.result.metrics)):
            before, now = old_metrics.get(key), row.result.metrics.get(key)
            if before != now:
                metrics.append({"stage": row.name, "metric": key, "before": before, "now": now})
    values = []
    old_spec = previous_dir / "twin.yaml"
    if old_spec.exists():
        try:
            old_data = yaml.safe_load(old_spec.read_text(encoding="utf-8")) or {}
            old_overrides, _, old_assumptions = provenance(TwinSpec(old_spec, old_data), None)
        except (OSError, yaml.YAMLError, AttributeError, TypeError):
            old_overrides, old_assumptions = [], []
        before = _flat_provenance(old_overrides, old_assumptions)
        now = _flat_provenance([{**row, "urdf": None} for row in data.overrides], data.assumptions)
        for key in sorted(set(before) | set(now)):
            if before.get(key) != now.get(key):
                values.append({"key": key, "before": before.get(key), "now": now.get(key)})
    return {"run": previous_dir.name, "status": old_run.get("status"), "same_spec": old_run.get("spec_sha256") == data.spec_sha256,
            "stages": stages, "metrics": metrics, "values": values}


def _fill(data: ReportData, spec: TwinSpec | None, urdf_path: Path | None):
    if spec is None:
        data.notes.append("twin.yaml could not be read: no provenance table")
        return
    urdf = None
    if urdf_path is not None and urdf_path.exists():
        try:
            urdf = read_urdf(urdf_path)
        except (OSError, ValueError) as error:
            data.notes.append(f"the URDF could not be read ({error}): URDF values are not shown")
    else:
        data.notes.append("no URDF found: URDF values are not shown")
    data.overrides, data.urdf_values, data.assumptions = provenance(spec, urdf)
    data.home = dict(spec.data.get("home") or {})
    earlier = previous_run(data.run_dir)
    if earlier is not None:
        data.previous = compare(data, earlier)


def from_context(context, title: str | None = None) -> ReportData:
    """The report stage's view: the results of every stage that ran before it in this run."""
    from ..pipeline import _entry, _overall

    rows = [StageRow(name, result.status, result, smoke=_smoke(context.run_dir, result) if name == "smoke" else None)
            for name, result in context.results.items() if name != "report"]
    entries = [_entry(row.result, row.name) for row in rows] + [{"name": "report", "status": "succeeded", "passed": True}]
    rows.append(StageRow("report", "running"))
    spec = context.spec
    data = ReportData(run_dir=context.run_dir, report_dir=context.stage_dir, robot=spec.robot,
                      title=title or spec.section("report").get("title") or spec.robot,
                      status=_overall(entries, subset=False), provisional=True,
                      started_at=min((r.result.started_at for r in rows if r.result and r.result.started_at), default=None),
                      finished_at=None, spec_sha256=spec.sha256, environment=_environment(), stages=rows)
    _fill(data, spec, context.urdf())
    return data


def from_run_dir(run_dir: Path, report_dir: Path | None = None, title: str | None = None) -> ReportData:
    """A finished (or stopped) run read back from disk: run.json plus every stage's result.json."""
    run_dir = Path(run_dir).resolve()
    summary = _read_json(run_dir / "run.json")
    if summary is None:
        raise ValueError(f"{run_dir} has no readable run.json")
    rows = []
    for entry in summary.get("stages", []):
        name = entry["name"]
        path = run_dir / name / "result.json"
        result = read_result(path) if entry["status"] != "not_run" and path.exists() else None
        rows.append(StageRow(name, entry["status"], result, reused=bool(entry.get("reused")),
                             smoke=_smoke(run_dir, result) if name == "smoke" else None))
    spec = None
    copy = run_dir / "twin.yaml"
    if copy.exists():
        try:
            spec_data = yaml.safe_load(copy.read_text(encoding="utf-8")) or {}
            # relative paths in twin.yaml are relative to the original file, not to the copy in the run
            original = Path(summary.get("spec_path") or copy)
            spec = TwinSpec(original if original.exists() else copy, spec_data)
        except (OSError, yaml.YAMLError):
            spec = None
    urdf_path = None
    ingest = next((row.result for row in rows if row.name == "ingest" and row.result is not None and row.result.passed), None)
    if ingest is not None and "urdf" in ingest.outputs:
        urdf_path = _resolve(run_dir, ingest.outputs["urdf"])
    elif spec is not None:
        urdf_path = spec.urdf_path
    robot = summary.get("robot") or (spec.robot if spec else run_dir.parent.name)
    data = ReportData(run_dir=run_dir, report_dir=Path(report_dir) if report_dir else run_dir / "report", robot=robot,
                      title=title or (spec.section("report").get("title") if spec else None) or robot,
                      status=summary.get("status", "unknown"), provisional=False,
                      started_at=summary.get("started_at"), finished_at=summary.get("finished_at"),
                      spec_sha256=summary.get("spec_sha256"), environment=dict(summary.get("environment") or {}), stages=rows)
    _fill(data, spec, urdf_path)
    return data


def _environment() -> dict:
    import platform
    import sys

    from .. import __version__

    return {"twin_pipeline": __version__, "python": sys.version.split()[0], "platform": platform.platform()}
