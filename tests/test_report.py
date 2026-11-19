"""report stage and ``python -m twin_pipeline.report``: the page renders for passed, skipped, failed,
crashed and needs_review runs, is self-contained, and shows provenance and the previous run."""

import re
from pathlib import Path

import pytest

from twin_pipeline.pipeline import run_pipeline
from twin_pipeline.report import write_for_run
from twin_pipeline.report.__main__ import main as report_main
from twin_pipeline.result import read_result
from twin_pipeline.spec import load_spec
from twin_pipeline.stage import Stage
from twin_pipeline.stages import report as report_stage
from twin_pipeline.stages import smoke

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "arm_demo" / "twin.yaml"
REPORT = report_stage.STAGES[0]


def page_of(run_dir: Path) -> str:
    return (Path(run_dir) / "report" / "report.html").read_text(encoding="utf-8")


def assert_self_contained(html: str):
    assert "<script" not in html.lower()
    assert "<link" not in html.lower()
    assert not re.search(r"""(src|href)\s*=\s*["']?(https?:)?//""", html), "no network references"
    assert "@import" not in html and "url(" not in html


def test_report_on_the_default_pipeline_with_skipped_stages(tmp_path):
    """The mjcf placeholder skips, smoke skips for lack of an MJCF; the report still runs and says so."""
    summary = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path)
    run_dir = Path(summary["run_dir"])
    result = read_result(run_dir / "report" / "result.json")
    assert result.status == "succeeded" and result.outputs == {"html": "report/report.html"}
    assert result.metrics["assumed_values"] == 2 and result.metrics["assumptions_assumed"] == 1
    html = page_of(run_dir)
    assert_self_contained(html)
    assert "Demo arm" in html and "should have run" in html and "nothing to do for this twin" in html
    assert "incomplete" in html


def test_provenance_table_highlights_assumed_values(tmp_path):
    summary = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path)
    html = page_of(summary["run_dir"])
    rows = re.findall(r'<tr class="assumed">(.*?)</tr>', html, re.S)
    assumed = " ".join(rows)
    assert "elbow_joint" in assumed and "lower" in assumed and "upper" in assumed and "zero_pose" in assumed
    measured = re.search(r"<tr>(?:(?!</tr>).)*finger_joint(?:(?!</tr>).)*measured(?:(?!</tr>).)*</tr>", html, re.S)
    assert measured, "the measured finger effort is listed but not highlighted"
    # effort/velocity the URDF gives without a provenance in twin.yaml are flagged; the measured one is not
    urdf_section = html.split("Actuator limits taken from the URDF", 1)[1].split("</table>", 1)[0]
    assert "pitch_joint" in urdf_section and "velocity" in urdf_section
    assert not re.search(r"finger_joint</b></td><td>effort", urdf_section)


def test_report_after_smoke_links_outputs_relatively(tmp_path):
    from test_smoke import STIFF, demo_spec, fake_mjcf

    summary = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path, stages=[fake_mjcf(spec=demo_spec(STIFF)), smoke.STAGES[0], REPORT])
    assert summary["status"] == "succeeded"
    html = page_of(summary["run_dir"])
    assert_self_contained(html)
    assert 'href="../smoke/smoke.json"' in html and 'href="../mjcf/model.xml"' in html and 'href="../twin.yaml"' in html
    assert "Smoke test" in html and "position (elbow_act)" in html
    assert "Status as of the report stage" in html


def test_failed_run_is_reported_afterwards(tmp_path, capsys):
    """The runner stops at a failed stage, so the report stage does not run; the module writes the page from disk."""
    from test_smoke import fake_mjcf

    summary = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path, stages=[fake_mjcf(status="failed"), smoke.STAGES[0], REPORT])
    run_dir = Path(summary["run_dir"])
    assert summary["status"] == "failed" and not (run_dir / "report").exists()
    assert report_main([str(run_dir)]) == 0
    assert capsys.readouterr().out.strip().endswith("report.html")
    html = page_of(run_dir)
    assert_self_contained(html)
    assert "MESH_TOO_LARGE" in html and "the run stopped at <b>mjcf</b>" in html
    assert html.count("not run") >= 2 and "Status as of" not in html
    assert "elbow_joint" in html  # provenance still shown


def test_crash_and_review_render(tmp_path):
    def crash(context):
        raise RuntimeError("boom <b>")

    summary = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path, stages=[Stage("mjcf", crash), REPORT])
    page = write_for_run(summary["run_dir"])
    html = page.read_text(encoding="utf-8")
    assert "STAGE_CRASHED" in html and "boom &lt;b&gt;" in html and "boom <b>" not in html
    assert 'href="../mjcf/traceback.txt"' in html

    from test_smoke import SOFT, demo_spec, fake_mjcf

    summary = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path, stages=[fake_mjcf(spec=demo_spec(SOFT)), smoke.STAGES[0], REPORT])
    assert summary["status"] == "needs_review"
    html = write_for_run(summary["run_dir"]).read_text(encoding="utf-8")
    assert "For review" in html and "pitch_joint sweep" in html and "needs review" in html


def test_comparison_with_the_previous_run(tmp_path):
    first = run_pipeline(load_spec(EXAMPLE), runs_dir=tmp_path)
    assert "No earlier run" in page_of(first["run_dir"])
    twin = tmp_path / "twin.yaml"
    text = EXAMPLE.read_text(encoding="utf-8").replace("urdf: urdf/arm_demo.urdf", f"urdf: {(EXAMPLE.parent / 'urdf' / 'arm_demo.urdf').as_posix()}")
    twin.write_text(text.replace("    source: assumed\n    note: Soft limits", "    source: measured\n    note: Soft limits"), encoding="utf-8")
    second = run_pipeline(load_spec(twin), runs_dir=tmp_path)
    assert Path(second["run_dir"]).parent == Path(first["run_dir"]).parent
    html = page_of(second["run_dir"])
    section = html.split("Since the previous run", 1)[1]
    assert Path(first["run_dir"]).name in section and "<b>changed</b>" in section
    assert "joints.elbow_joint.lower" in section and "(assumed)" in section and "(measured)" in section
    result = read_result(Path(second["run_dir"]) / "report" / "result.json")
    assert result.metrics["previous_run"] == Path(first["run_dir"]).name and result.metrics["assumed_values"] == 0


def test_report_escapes_twin_yaml_text(tmp_path):
    twin = tmp_path / "twin.yaml"
    text = EXAMPLE.read_text(encoding="utf-8").replace("urdf: urdf/arm_demo.urdf", f"urdf: {(EXAMPLE.parent / 'urdf' / 'arm_demo.urdf').as_posix()}")
    twin.write_text(text.replace("Joint zeros are the CAD zero pose", "<script>alert(1)</script> zeros"), encoding="utf-8")
    summary = run_pipeline(load_spec(twin), runs_dir=tmp_path / "runs")
    html = page_of(summary["run_dir"])
    assert "&lt;script&gt;alert(1)&lt;/script&gt;" in html
    assert_self_contained(html)


def test_module_rejects_a_folder_without_run_json(tmp_path, capsys):
    assert report_main([str(tmp_path)]) == 1
    assert "run.json" in capsys.readouterr().err


def test_validator():
    assert report_stage.validate_report("report", {"title": "x"}) == []
    assert report_stage.validate_report("report", {"titel": "x"})
    assert report_stage.validate_report("report", {"title": ""})


@pytest.fixture(autouse=True)
def _tests_on_path(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parent))
