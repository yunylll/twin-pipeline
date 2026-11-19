import json
from pathlib import Path

import pytest

from twin_pipeline.cli import main
from twin_pipeline.pipeline import run_pipeline
from twin_pipeline.result import StageResult, read_result
from twin_pipeline.spec import load_spec
from twin_pipeline.stage import Stage

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "arm_demo" / "twin.yaml"


@pytest.fixture
def spec():
    return load_spec(EXAMPLE)


def test_default_pipeline_on_example(spec, tmp_path):
    summary = run_pipeline(spec, runs_dir=tmp_path)
    run_dir = Path(summary["run_dir"])
    assert (run_dir / "run.json").exists() and (run_dir / "twin.yaml").exists()
    rows = {row["name"]: row for row in summary["stages"]}
    assert [row["name"] for row in summary["stages"]] == ["detect", "export", "ingest", "mjcf", "smoke", "report"]
    for name in ("detect", "export", "ingest"):
        assert rows[name]["status"] == "skipped" and rows[name]["applicable"] is False
    assert rows["mjcf"]["status"] == "skipped" and rows["mjcf"]["applicable"] is True
    assert summary["status"] == "incomplete" and summary["passed"] is False
    assert read_result(run_dir / "mjcf" / "result.json").stage == "mjcf"


def fake(name, status, log, **kwargs):
    def run(context):
        log.append(name)
        if status == "crash":
            raise RuntimeError("boom")
        if status == "succeeded":
            path = context.stage_dir / "out.txt"
            path.write_text(name)
            return StageResult.succeeded(name, outputs={"out": context.rel(path)})
        if status == "failed":
            return StageResult.failed(name, "BAD", "it went wrong")
        if status == "needs_review":
            return StageResult.needs_review(name, [{"item": "previews", "reason": "look at them"}])
        return StageResult.skipped(name, "nothing", **kwargs)
    return Stage(name, run)


def test_failure_stops_the_run(spec, tmp_path):
    log = []
    stages = [fake("a", "succeeded", log), fake("b", "failed", log), fake("c", "succeeded", log)]
    summary = run_pipeline(spec, runs_dir=tmp_path, stages=stages)
    assert log == ["a", "b"]
    assert summary["status"] == "failed" and summary["stopped_at"] == "b"
    assert summary["stages"][2]["status"] == "not_run"


def test_crash_is_reported(spec, tmp_path):
    summary = run_pipeline(spec, runs_dir=tmp_path, stages=[fake("a", "crash", [])])
    result = read_result(Path(summary["run_dir"]) / "a" / "result.json")
    assert result.status == "error" and result.errors[0]["code"] == "STAGE_CRASHED"
    assert (Path(summary["run_dir"]) / "a" / "traceback.txt").exists()
    assert summary["status"] == "failed"


def test_outputs_flow_to_later_stages(spec, tmp_path):
    seen = {}

    def consumer(context):
        seen["path"] = context.output("a", "out")
        seen["missing"] = context.output("a", "nope")
        return StageResult.succeeded("b")

    summary = run_pipeline(spec, runs_dir=tmp_path, stages=[fake("a", "succeeded", []), Stage("b", consumer)])
    assert summary["status"] == "succeeded" and summary["passed"] is True
    assert seen["path"].read_text() == "a" and seen["missing"] is None


def test_review_then_approve_on_resume(spec, tmp_path):
    log = []
    stages = [fake("a", "succeeded", log), fake("b", "needs_review", log), fake("c", "succeeded", log)]
    first = run_pipeline(spec, runs_dir=tmp_path, stages=stages)
    assert first["status"] == "needs_review" and log == ["a", "b"]
    with pytest.raises(ValueError, match="--approve needs --resume"):
        run_pipeline(spec, runs_dir=tmp_path, stages=stages, approve=["b"])
    second = run_pipeline(spec, resume=Path(first["run_dir"]), stages=stages, approve=["b"])
    assert log == ["a", "b", "c"]
    assert second["status"] == "succeeded"
    rows = {row["name"]: row for row in second["stages"]}
    assert rows["a"].get("reused") and rows["b"]["approved"]


def test_resume_reruns_after_a_failure(spec, tmp_path):
    log = []
    first = run_pipeline(spec, runs_dir=tmp_path, stages=[fake("a", "succeeded", log), fake("b", "failed", log)])
    fixed = [fake("a", "succeeded", log), fake("b", "succeeded", log)]
    second = run_pipeline(spec, resume=Path(first["run_dir"]), stages=fixed)
    assert log == ["a", "b", "b"] and second["status"] == "succeeded"


def test_applicable_skip_is_incomplete(spec, tmp_path):
    stages = [fake("a", "skipped", [], applicable=False), fake("b", "skipped", [], applicable=True)]
    assert run_pipeline(spec, runs_dir=tmp_path, stages=stages)["status"] == "incomplete"
    stages = [fake("a", "skipped", [], applicable=False), fake("b", "succeeded", [])]
    assert run_pipeline(spec, runs_dir=tmp_path, stages=stages)["status"] == "succeeded"


def test_subset_run_is_partial(spec, tmp_path):
    stages = [fake("a", "succeeded", []), fake("b", "succeeded", [])]
    assert run_pipeline(spec, runs_dir=tmp_path, stages=stages, only=["a"])["status"] == "partial"
    with pytest.raises(ValueError, match="unknown stage"):
        run_pipeline(spec, runs_dir=tmp_path, stages=stages, only=["z"])


def test_cli(tmp_path, capsys):
    assert main(["validate", str(EXAMPLE)]) == 0
    assert "OK: arm_demo" in capsys.readouterr().out
    assert main(["run", str(EXAMPLE), "--runs-dir", str(tmp_path)]) == 3
    out = json.loads(capsys.readouterr().out)
    assert out["status"] == "incomplete"
    assert main(["stages"]) == 0
