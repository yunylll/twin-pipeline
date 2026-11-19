"""Render ReportData as one self-contained HTML page: inline CSS, no scripts, no network requests."""

from __future__ import annotations

import json
import os
from html import escape
from pathlib import Path
from urllib.parse import quote

from .collect import ReportData, StageRow

STATUS_LABELS = {"succeeded": "succeeded", "failed": "failed", "error": "error", "needs_review": "needs review",
                 "skipped": "skipped", "not_run": "not run", "running": "this page", "incomplete": "incomplete",
                 "partial": "partial", "unknown": "unknown"}
STATUS_TONES = {"succeeded": "good", "partial": "good", "failed": "bad", "error": "bad", "needs_review": "warn",
                "incomplete": "warn", "skipped": "muted", "not_run": "muted", "running": "info", "unknown": "muted"}

CSS = """
:root{--bg:#f7f7f5;--panel:#fff;--ink:#1d1d1b;--muted:#6b6b66;--line:#e3e2dc;--code:#f1f0ec;
--good:#1f7a4d;--good-bg:#e4f4ea;--bad:#b3261e;--bad-bg:#fbe7e5;--warn:#8a5a00;--warn-bg:#fff2cc;
--info:#1d5fa8;--info-bg:#e3eefb;--muted-bg:#eeede8;--assumed:#fff4d1;--assumed-line:#e8c55a}
@media (prefers-color-scheme:dark){:root{--bg:#161615;--panel:#1f1f1d;--ink:#ecebe6;--muted:#a3a29b;--line:#34332f;--code:#2a2926;
--good:#6fd39b;--good-bg:#173726;--bad:#ff8a80;--bad-bg:#401b18;--warn:#f2c14e;--warn-bg:#3a2e10;--info:#8cb8f0;--info-bg:#18293f;
--muted-bg:#2a2926;--assumed:#3a2f10;--assumed-line:#8a6d1c}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
main{max-width:1080px;margin:0 auto;padding:28px 16px 64px}
h1{font-size:26px;margin:0 0 4px}h2{font-size:19px;margin:36px 0 10px}h3{font-size:16px;margin:18px 0 8px}
p{margin:6px 0}a{color:var(--info)}code,.mono{font-family:ui-monospace,"Cascadia Mono",Consolas,monospace;font-size:13px}
code{background:var(--code);padding:1px 5px;border-radius:4px}
.sub{color:var(--muted)}
.badge{display:inline-block;padding:2px 9px;border-radius:999px;font-size:12.5px;font-weight:600;white-space:nowrap}
.good{color:var(--good);background:var(--good-bg)}.bad{color:var(--bad);background:var(--bad-bg)}
.warn{color:var(--warn);background:var(--warn-bg)}.info{color:var(--info);background:var(--info-bg)}.muted{color:var(--muted);background:var(--muted-bg)}
.head{display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between}
.head .badge{font-size:15px;padding:4px 14px}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;margin:18px 0}
.card{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.card b{display:block;font-size:24px;line-height:1.2}.card span{color:var(--muted);font-size:13px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:4px 16px 12px;margin:10px 0}
.scroll{overflow-x:auto}
table{border-collapse:collapse;width:100%;font-size:14px}
th,td{text-align:left;padding:7px 10px;border-bottom:1px solid var(--line);vertical-align:top}
th{color:var(--muted);font-weight:600;font-size:12.5px;text-transform:uppercase;letter-spacing:.03em}
tr:last-child td{border-bottom:none}
td.num{font-variant-numeric:tabular-nums;white-space:nowrap}
tr.assumed td,tr.flag td{background:var(--assumed)}tr.assumed td:first-child,tr.flag td:first-child{box-shadow:inset 3px 0 0 var(--assumed-line)}
details{border-top:1px solid var(--line)}details:first-of-type{border-top:none}
summary{cursor:pointer;padding:10px 0;display:flex;gap:10px;align-items:baseline;flex-wrap:wrap;list-style:none}
summary::-webkit-details-marker{display:none}summary::before{content:"\\25B8";color:var(--muted)}details[open] summary::before{content:"\\25BE"}
summary .name{font-weight:600;min-width:70px}summary .reason{color:var(--muted)}
.body{padding:0 0 12px 22px}
ul.items{margin:6px 0;padding-left:20px}ul.items li{margin:3px 0}
.attention li{margin:5px 0}
.kv{display:grid;grid-template-columns:max-content 1fr;gap:2px 14px;font-size:14px}.kv dt{color:var(--muted)}.kv dd{margin:0}
footer{margin-top:40px;color:var(--muted);font-size:13px}
"""


def _e(value) -> str:
    return escape("" if value is None else str(value))


def _badge(status: str, label: str | None = None) -> str:
    return f'<span class="badge {STATUS_TONES.get(status, "muted")}">{_e(label or STATUS_LABELS.get(status, status))}</span>'


def _value(value) -> str:
    if value is None:
        return '<span class="sub">—</span>'
    if isinstance(value, float):
        return _e(f"{value:.6g}")
    if isinstance(value, (list, dict)):
        return f'<span class="mono">{_e(json.dumps(value, ensure_ascii=False))}</span>'
    return _e(value)


def _table(headers, rows, classes=None) -> str:
    head = "".join(f"<th>{_e(h)}</th>" for h in headers)
    body = []
    for index, row in enumerate(rows):
        css = f' class="{classes[index]}"' if classes and classes[index] else ""
        body.append(f"<tr{css}>" + "".join(f"<td>{cell}</td>" for cell in row) + "</tr>")
    return f'<div class="scroll"><table><thead><tr>{head}</tr></thead><tbody>{"".join(body)}</tbody></table></div>'


def _link(data: ReportData, value: str) -> str:
    """A link to a stage output, relative to the report page; outside the run it is shown as text."""
    path = Path(value)
    target = path if path.is_absolute() else data.run_dir / path
    try:
        target.resolve().relative_to(data.run_dir.resolve())
    except ValueError:
        return f'<span class="mono">{_e(value)}</span> <span class="sub">(outside the run)</span>'
    href = Path(os.path.relpath(target, data.report_dir)).as_posix()
    return f'<a class="mono" href="{_e(quote(href))}">{_e(value)}</a>'


def _stage_details(data: ReportData, row: StageRow) -> str:
    result = row.result
    flags = []
    if row.reused:
        flags.append('<span class="badge muted">kept from an earlier attempt</span>')
    if result is not None and result.approved:
        flags.append(f'<span class="badge info">approved {_e(result.approved_at)}</span>')
    if row.status == "skipped" and result is not None:
        flags.append('<span class="badge muted">nothing to do for this twin</span>' if result.applicable is False
                     else '<span class="badge warn">should have run</span>')
    reason = result.reason if result is not None else ("the run stopped before this stage" if row.status == "not_run" else "")
    if row.status == "running":
        reason = "writing this page"
    parts = [f'<summary><span class="name">{_e(row.name)}</span>{_badge(row.status)}{"".join(flags)}<span class="reason">{_e(reason)}</span></summary>']
    if result is None:
        return f"<details>{parts[0]}</details>"
    body = []
    if result.errors:
        body.append("<h3>Errors</h3><ul class=\"items\">" + "".join(
            f"<li><code>{_e(item.get('code'))}</code> {_e(item.get('message'))}</li>" for item in result.errors) + "</ul>")
    if result.review:
        body.append("<h3>For review</h3><ul class=\"items\">" + "".join(
            f"<li><b>{_e(item.get('item'))}</b> — {_e(item.get('reason'))}</li>" for item in result.review) + "</ul>")
    if result.warnings:
        body.append("<h3>Warnings</h3><ul class=\"items\">" + "".join(f"<li>{_e(w)}</li>" for w in result.warnings) + "</ul>")
    if result.metrics:
        body.append("<h3>Metrics</h3><dl class=\"kv\">" + "".join(
            f"<dt>{_e(k)}</dt><dd class=\"mono\">{_value(v)}</dd>" for k, v in result.metrics.items()) + "</dl>")
    if result.outputs:
        body.append("<h3>Outputs</h3><dl class=\"kv\">" + "".join(
            f"<dt>{_e(k)}</dt><dd>{_link(data, v)}</dd>" for k, v in result.outputs.items()) + "</dl>")
    timing = f"{result.duration_seconds:g} s" if result.duration_seconds is not None else ""
    body.append(f'<p class="sub">result: {_link(data, f"{row.name}/result.json")} {_e(timing)}</p>')
    is_open = " open" if row.status in ("failed", "error", "needs_review") else ""
    return f'<details{is_open}>{parts[0]}<div class="body">{"".join(body)}</div></details>'


def _attention(data: ReportData) -> str:
    items = []
    for row in data.stages:
        result = row.result
        if result is None:
            continue
        for error in result.errors:
            items.append(f'<li>{_badge("failed")} <b>{_e(row.name)}</b> <code>{_e(error.get("code"))}</code> {_e(error.get("message"))}</li>')
        if not result.passed or row.status == "needs_review":
            for item in result.review:
                items.append(f'<li>{_badge("needs_review")} <b>{_e(row.name)}</b> {_e(item.get("item"))} — {_e(item.get("reason"))}</li>')
        if row.status == "skipped" and result.applicable:
            items.append(f'<li>{_badge("skipped")} <b>{_e(row.name)}</b> should have run: {_e(result.reason)}</li>')
    stopped = next((row.name for row in data.stages if row.status in ("failed", "error") or (row.status == "needs_review" and not (row.result and row.result.passed))), None)
    not_run = [row.name for row in data.stages if row.status == "not_run"]
    if stopped and not_run:
        items.append(f'<li>{_badge("not_run")} the run stopped at <b>{_e(stopped)}</b>; not run: {_e(", ".join(not_run))}</li>')
    assumed = sum(1 for row in data.overrides if row["source"] in ("assumed", "unspecified"))
    assumed += sum(1 for row in data.assumptions if row["source"] in ("assumed", "unspecified"))
    if assumed:
        items.append(f'<li><span class="badge warn">assumed</span> {assumed} value(s) and assumption(s) are still assumed: see <a href="#provenance">provenance</a></li>')
    if not items:
        return ""
    return f'<h2>Needs attention</h2><div class="panel"><ul class="items attention">{"".join(items)}</ul></div>'


def _smoke(data: ReportData) -> str:
    row = next((r for r in data.stages if r.name == "smoke" and r.smoke), None)
    if row is None:
        return ""
    smoke = row.smoke
    checks = smoke.get("checks") or {}
    tone = {"passed": "succeeded", "failed": "failed", "review": "needs_review", "not run": "not_run"}
    check_line = " ".join(f'<span class="sub">{_e(name)}</span> {_badge(tone.get(state, "unknown"), state)}' for name, state in checks.items())
    model = smoke.get("model") or {}
    gravity = smoke.get("gravity") or {}
    info = (f'<p class="sub">MuJoCo {_e(smoke.get("mujoco"))} · timestep {_value(model.get("timestep"))} s ({_e(model.get("timestep_from"))}) · '
            f'gravity {_value(gravity.get("vector"))} for up axis {_e(gravity.get("up_axis"))} · settings {_value(smoke.get("settings"))}</p>')
    home = (smoke.get("home") or {}).get("joints") or {}
    settle = (smoke.get("settle") or {}).get("joints") or {}
    rows, classes = [], []
    for name, joint in (smoke.get("joints") or {}).items():
        sweep = joint.get("sweep") or []
        unit = "rad" if joint.get("type") == "hinge" else "m"
        cells = []
        bad = False
        for entry in sweep:
            if "error" in entry:
                ok = entry.get("within_tolerance")
                bad |= not ok
                cells.append(f'{_value(entry["target"])} → {_value(entry["reached"])} '
                             f'<span class="{"sub" if ok else "badge warn"}">err {_value(entry["error"])}</span>')
            elif "unstable" in entry:
                bad = True
                cells.append(f'{_value(entry["target"])} <span class="badge bad">unstable</span>')
        home_row = home.get(name) or {}
        settle_row = settle.get(name) or {}
        rest = "" if not settle_row else (_badge("succeeded", "at rest") if settle_row.get("at_rest") else '<span class="badge warn">moving</span>')
        rows.append([f"<b>{_e(name)}</b><br><span class=\"sub\">{_e(joint.get('type'))}, {_e(unit)}</span>",
                     _value(joint.get("effective_limits")) + f'<br><span class="sub">{_e(joint.get("limits_from"))}</span>',
                     _e(joint.get("mode")),
                     f'{_value(home_row.get("used", home_row.get("value")))}<br><span class="sub">{_e(home_row.get("from"))}</span>' if home_row else "",
                     "<br>".join(cells) or '<span class="sub">not swept</span>',
                     f'{rest} <span class="sub">drift {_value(settle_row.get("drift"))}</span>' if settle_row else ""])
        classes.append("flag" if bad else "")
    table = _table(["joint", "effective limits", "driven by", "home", "sweep: target → reached", "settle"], rows, classes)
    contacts = (smoke.get("home") or {}).get("contacts") or []
    contact_text = f'<p class="sub">{len(contacts)} deep contact(s) at home.</p>' if contacts else ""
    return f'<h2>Smoke test</h2><div class="panel"><p>{check_line}</p>{info}{table}{contact_text}</div>'


def _provenance(data: ReportData) -> str:
    parts = ['<h2 id="provenance">Provenance</h2>',
             '<p class="sub">Every value twin.yaml adds to the CAD, with where it comes from. '
             '<span class="badge warn">assumed</span> rows are working values nobody has checked yet.</p>']
    if data.overrides:
        rows, classes = [], []
        for row in data.overrides:
            source = row["source"]
            pill = f'<span class="badge {"warn" if source in ("assumed", "unspecified") else "good" if source in ("measured", "calibrated") else "muted"}">{_e(source)}</span>'
            rows.append([f"<b>{_e(row['joint'])}</b>", _e(row["field"]), f'<span class="num">{_value(row["value"])}</span>',
                         f'<span class="num">{_value(row["urdf"])}</span>', pill, _e(row.get("note"))])
            classes.append("assumed" if source in ("assumed", "unspecified") else "")
        parts.append("<h3>Joint values set in twin.yaml</h3><div class=\"panel\">" +
                     _table(["joint", "field", "twin.yaml", "URDF", "source", "note"], rows, classes) + "</div>")
    else:
        parts.append('<p>twin.yaml overrides no joint values.</p>')
    if data.assumptions:
        rows, classes = [], []
        for row in data.assumptions:
            source = row["source"]
            pill = f'<span class="badge {"warn" if source in ("assumed", "unspecified") else "good" if source in ("measured", "calibrated") else "muted"}">{_e(source)}</span>'
            rows.append([f"<code>{_e(row['id'])}</code>", _e(row["text"]), _value(row.get("value")), pill, _e(row.get("note"))])
            classes.append("assumed" if source in ("assumed", "unspecified") else "")
        parts.append("<h3>Assumptions</h3><div class=\"panel\">" + _table(["id", "assumption", "value", "source", "note"], rows, classes) + "</div>")
    if data.urdf_values:
        rows = [[f"<b>{_e(row['joint'])}</b>", _e(row["field"]), f'<span class="num">{_value(row["value"])}</span>'] for row in data.urdf_values]
        parts.append("<h3>Actuator limits taken from the URDF</h3>"
                     '<p class="sub">CAD does not know effort or velocity limits; these come from the URDF with no provenance in twin.yaml. '
                     'Set them under <code>joints</code> with a source once known.</p><div class="panel">'
                     + _table(["joint", "field", "URDF value"], rows, ["assumed"] * len(rows)) + "</div>")
    if data.home:
        rows = [[f"<b>{_e(k)}</b>", f'<span class="num">{_value(v)}</span>'] for k, v in data.home.items()]
        parts.append('<h3>Home pose</h3><div class="panel">' + _table(["joint", "home"], rows) + "</div>")
    return "".join(parts)


def _previous(data: ReportData) -> str:
    previous = data.previous
    if not previous:
        return '<h2>Since the previous run</h2><p class="sub">No earlier run of this robot under the same runs folder.</p>'
    old_dir = data.run_dir.parent / previous["run"]
    page = old_dir / "report" / "report.html"
    href = Path(os.path.relpath(page if page.exists() else old_dir / "run.json", data.report_dir)).as_posix()
    parts = [f'<h2>Since the previous run</h2><div class="panel"><p>Compared with <a class="mono" href="{_e(quote(href))}">{_e(previous["run"])}</a> '
             f'({_badge(previous.get("status") or "unknown")}); twin.yaml {"unchanged" if previous["same_spec"] else "<b>changed</b>"}.</p>']
    if previous["values"]:
        rows = []
        for row in previous["values"]:
            before = "—" if row["before"] is None else f"{_value(row['before'][0])} ({_e(row['before'][1])})"
            now = "—" if row["now"] is None else f"{_value(row['now'][0])} ({_e(row['now'][1])})"
            rows.append([f"<code>{_e(row['key'])}</code>", before, now])
        parts.append("<h3>twin.yaml values</h3>" + _table(["value", "before (source)", "now (source)"], rows))
    if previous["stages"]:
        parts.append("<h3>Stage status</h3>" + _table(["stage", "before", "now"], [
            [_e(r["stage"]), _badge(r["before"]) if r["before"] != "absent" else "—", _badge(r["now"])] for r in previous["stages"]]))
    if previous["metrics"]:
        parts.append("<h3>Metrics</h3>" + _table(["stage", "metric", "before", "now"], [
            [_e(r["stage"]), f"<code>{_e(r['metric'])}</code>", _value(r["before"]), _value(r["now"])] for r in previous["metrics"]]))
    if not (previous["values"] or previous["stages"] or previous["metrics"]):
        parts.append('<p class="sub">No differences in stage status, metrics or twin.yaml values.</p>')
    return "".join(parts) + "</div>"


def render(data: ReportData) -> str:
    stages = [row for row in data.stages]
    ran = [row for row in stages if row.status not in ("not_run", "running")
           and not (row.status == "skipped" and row.result is not None and row.result.applicable is False)]
    passed = sum(1 for row in ran if row.result is not None and row.result.passed)
    review = sum(len(row.result.review) for row in ran if row.result is not None and not row.result.passed)
    assumed = sum(1 for row in data.overrides if row["source"] in ("assumed", "unspecified"))
    assumptions = sum(1 for row in data.assumptions if row["source"] in ("assumed", "unspecified"))
    status_note = ("<p class=\"sub\">Status as of the report stage; run.json has the final one.</p>" if data.provisional else "")
    env = " · ".join(f"{_e(k)} {_e(v)}" for k, v in data.environment.items())
    when = _e(data.started_at) + (f" → {_e(data.finished_at)}" if data.finished_at else "")
    sha = _e((data.spec_sha256 or "")[:12])
    cards = "".join(f'<div class="card"><b>{value}</b><span>{_e(label)}</span></div>' for value, label in (
        (f"{passed}/{len(ran)}", "applicable stages passed"), (review, "items for review"),
        (assumed, "joint values still assumed"), (assumptions, "assumptions still assumed")))
    notes = "".join(f'<p class="sub">{_e(n)}</p>' for n in data.notes)
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src 'unsafe-inline'; img-src data:">
<title>{_e(data.title)} — {_e(data.run_dir.name)}</title><style>{CSS}</style></head>
<body><main>
<div class="head"><div><h1>{_e(data.title)}</h1>
<p class="sub">robot <code>{_e(data.robot)}</code> · run <code>{_e(data.run_dir.name)}</code> · {when}</p></div>{_badge(data.status)}</div>
{status_note}
<div class="cards">{cards}</div>
{_attention(data)}
<h2>Stages</h2><div class="panel">{"".join(_stage_details(data, row) for row in stages)}</div>
{_smoke(data)}
{_provenance(data)}
{_previous(data)}
{notes}
<footer>twin.yaml of this run: {_link(data, "twin.yaml")} (sha256 {sha}…) · run summary: {_link(data, "run.json") if (data.run_dir / "run.json").exists() else "run.json is written when the run ends"}<br>
{env}<br>
The report stage covers the stages that ran before it; the runner stops at a failed stage, so for such a run write the page afterwards with
<code>python -m twin_pipeline.report &lt;run folder&gt;</code>.</footer>
</main></body></html>
"""
