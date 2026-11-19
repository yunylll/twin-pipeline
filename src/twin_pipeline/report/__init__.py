"""The run report: one self-contained HTML page per run (no scripts, no network requests).

``write_for_context(context)`` is what the report stage calls. ``write_for_run(run_dir)`` writes the
page for a run read back from disk, e.g. one that stopped at a failed stage before the report stage
ran: ``python -m twin_pipeline.report <run folder>``.
"""

from __future__ import annotations

from pathlib import Path

from .collect import ReportData, from_context, from_run_dir
from .html import render

PAGE = "report.html"


def write_for_context(context, title: str | None = None) -> tuple[Path, ReportData]:
    data = from_context(context, title)
    path = context.stage_dir / PAGE
    path.write_text(render(data), encoding="utf-8")
    return path, data


def write_for_run(run_dir: Path | str, out: Path | str | None = None, title: str | None = None) -> Path:
    run_dir = Path(run_dir).resolve()
    path = Path(out).resolve() if out else run_dir / "report" / PAGE
    data = from_run_dir(run_dir, report_dir=path.parent, title=title)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render(data), encoding="utf-8")
    return path


__all__ = ["PAGE", "ReportData", "from_context", "from_run_dir", "render", "write_for_context", "write_for_run"]
