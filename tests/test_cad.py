"""CAD stages against a fake solidworks-urdf-tool (tests/fake_sw_tool): no SolidWorks needed."""

import json
import shutil
from pathlib import Path

import pytest
import yaml

from twin_pipeline.pipeline import run_pipeline
from twin_pipeline.result import read_result
from twin_pipeline.spec import load_spec, validate_data
from twin_pipeline.stages import all_stages

pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "examples" / "arm_demo"
FAKE_TOOL = Path(__file__).resolve().parent / "fake_sw_tool"
CAD_STAGES = [stage for stage in all_stages() if stage.name in ("detect", "export", "ingest")]


@pytest.fixture
def twin(tmp_path):
    """A twin folder with a fake CAD model and export config, and a fake tool next to it."""
    tool = tmp_path / "tool"
    shutil.copytree(FAKE_TOOL, tool)
    folder = tmp_path / "twin"
    (folder / "source").mkdir(parents=True)
    (folder / "source" / "robot.SLDASM").write_bytes(b"assembly v1")
    (folder / "source" / "part.SLDPRT").write_bytes(b"part v1")
    (folder / "export-config.json").write_text(json.dumps({"schema_version": 1, "robot_name": "arm_demo", "links": []}), encoding="utf-8")
    data = yaml.safe_load((DEMO / "twin.yaml").read_text(encoding="utf-8"))
    del data["urdf"]
    data["cad"] = {"model": "source/robot.SLDASM", "config": "export-config.json", "package": "arm_demo_description", "tool": str(tool)}
    (folder / "twin.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    scenario(tool, {})
    return folder, tool


def scenario(tool, data):
    data = dict(data)
    data.setdefault("urdf", str(DEMO / "urdf" / "arm_demo.urdf"))
    (tool / "scenario.json").write_text(json.dumps(data), encoding="utf-8")


def calls(tool):
    path = tool / "calls.jsonl"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()] if path.exists() else []


def starts(tool):
    return [c for c in calls(tool) if c[0] == "start-export"]


def run(folder, tmp_path, **kwargs):
    return run_pipeline(load_spec(folder / "twin.yaml"), runs_dir=tmp_path / "runs", **kwargs)


def rows(summary):
    return {row["name"]: row for row in summary["stages"]}


def result(summary, stage):
    return read_result(Path(summary["run_dir"]) / stage / "result.json")


def test_cad_section_validation():
    base = {"schema_version": 1, "robot": "r"}
    assert validate_data(dict(base, cad={"model": "m.SLDASM", "check_first": False, "timeout_seconds": 60})) == []
    errors = validate_data(dict(base, cad={"model": "", "check_first": "yes", "timeout_seconds": 0, "archve": "x"}))
    assert any("cad.model" in e for e in errors) and any("cad.check_first" in e for e in errors)
    assert any("cad.timeout_seconds" in e for e in errors) and any("'archve'" in e for e in errors)


def test_first_export_needs_review_then_runs_to_the_end(twin, tmp_path):
    folder, tool = twin
    first = run(folder, tmp_path)
    assert first["status"] == "needs_review" and first["stopped_at"] == "ingest"
    assert [rows(first)[n]["status"] for n in ("detect", "export", "ingest", "mjcf", "report")] == ["succeeded", "succeeded", "needs_review", "not_run", "succeeded"]
    detect = result(first, "detect")
    assert detect.metrics["export_needed"] and "no archived URDF" in detect.metrics["export_reasons"][0]
    kinds = ["check" if "--check-only" in c else "export" for c in starts(tool)]
    assert kinds == ["check", "export"]
    export_call = starts(tool)[1]
    assert export_call[export_call.index("--publish-to") + 1] == str(folder)
    published = folder / "urdf" / "arm_demo_description" / "urdf" / "arm_demo_description.urdf"
    assert published.exists() and (folder / "export" / "export-result.json").exists()
    ingest = result(first, "ingest")
    assert Path(ingest.outputs["urdf"]) == published and "front.png" in ingest.review[0]["reason"]

    second = run_pipeline(load_spec(folder / "twin.yaml"), resume=Path(first["run_dir"]), approve=["ingest"])
    assert second["status"] == "succeeded", second
    assert rows(second)["ingest"]["approved"] and rows(second)["smoke"]["status"] == "succeeded"
    assert len(starts(tool)) == 2  # approving re-ran nothing in the tool


def test_unchanged_cad_reuses_the_archive_without_solidworks(twin, tmp_path):
    folder, tool = twin
    run(folder, tmp_path)
    before = len(starts(tool))
    again = run(folder, tmp_path)
    assert again["status"] == "succeeded"
    assert result(again, "detect").metrics["export_needed"] is False
    assert result(again, "export").metrics["reused"] is True and result(again, "ingest").status == "succeeded"
    assert len(starts(tool)) == before
    # the export ran on the check's prepared copy, whose assembly differs from the original: the originals count
    fingerprint = json.loads((folder / "export" / "twin-pipeline-export.json").read_text(encoding="utf-8"))
    assert str(folder / "source" / "robot.SLDASM") in fingerprint["source_hashes"]
    (folder / "source" / "part.SLDPRT").write_bytes(b"part v2")
    reasons = result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect").metrics["export_reasons"]
    assert reasons == ["1 of 2 CAD files the archived export read changed or are missing: part.SLDPRT"]


def test_what_makes_an_export_needed(twin, tmp_path):
    folder, tool = twin
    run(folder, tmp_path)
    config = folder / "export-config.json"
    data = json.loads(config.read_text(encoding="utf-8"))
    data["source"] = {"model_sha256": "recorded by the tool"}
    config.write_text(json.dumps(data), encoding="utf-8")
    only_source = run(folder, tmp_path, stages=CAD_STAGES)
    assert result(only_source, "detect").metrics["export_needed"] is False  # the source block alone changes nothing

    data["links"] = [{"name": "base_link"}]
    config.write_text(json.dumps(data), encoding="utf-8")
    changed = result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect")
    assert changed.metrics["export_reasons"] == ["the export configuration changed since the archived export"]

    data["links"] = []
    config.write_text(json.dumps(data), encoding="utf-8")
    (folder / "source" / "robot.SLDASM").write_bytes(b"assembly v2")
    reasons = result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect").metrics["export_reasons"]
    assert any("model file changed" in r for r in reasons) and any("CAD files the archived export read" in r for r in reasons)


def test_config_edited_in_the_archive_is_noticed(twin, tmp_path):
    """cad.config pointing at the archived export/export-config.json, edited in place for the next export."""
    folder, tool = twin
    data = yaml.safe_load((folder / "twin.yaml").read_text(encoding="utf-8"))
    data["cad"]["config"] = "export/export-config.json"
    (folder / "twin.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    (folder / "export").mkdir()
    shutil.move(str(folder / "export-config.json"), str(folder / "export" / "export-config.json"))
    run(folder, tmp_path, stages=CAD_STAGES)
    assert json.loads((folder / "export" / "twin-pipeline-export.json").read_text(encoding="utf-8"))["config_fingerprint"]
    assert result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect").metrics["export_needed"] is False

    config = folder / "export" / "export-config.json"
    edited = json.loads(config.read_text(encoding="utf-8"))
    edited["exclude_hidden_bodies"] = ["bracket.SLDPRT"]
    config.write_text(json.dumps(edited), encoding="utf-8")
    reasons = result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect").metrics["export_reasons"]
    assert reasons == ["the export configuration changed since the archived export"]

    (folder / "export" / "twin-pipeline-export.json").unlink()  # an archive from before twin-pipeline
    reasons = result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect").metrics["export_reasons"]
    assert any("cannot be ruled out" in r for r in reasons)


def test_force_export_option(twin, tmp_path):
    folder, tool = twin
    run(folder, tmp_path)
    forced = run(folder, tmp_path, stages=CAD_STAGES, options={"force_export": "true"})
    assert result(forced, "export").metrics["forced"] is True
    assert rows(forced)["ingest"]["status"] == "needs_review"
    assert (folder / "export" / "history").is_dir()  # the earlier export was moved aside, not overwritten


def test_failed_check_stops_before_the_export(twin, tmp_path):
    folder, tool = twin
    scenario(tool, {"check": {"status": "failed", "passed": False, "error": {"code": "CONFIG_OUTDATED", "message": "base_link moved 3 mm"}}})
    summary = run(folder, tmp_path)
    export = result(summary, "export")
    assert export.status == "failed" and export.errors[0]["code"] == "CONFIG_OUTDATED" and "base_link moved" in export.reason
    assert ["--check-only" in c for c in starts(tool)] == [True]
    assert rows(summary)["report"]["status"] == "succeeded"


def test_invalid_configuration_fails_detect(twin, tmp_path):
    folder, tool = twin
    scenario(tool, {"validate": {"passed": False, "errors": ["links[1].components: 'arm-1' is not in the model"]}})
    detect = result(run(folder, tmp_path), "detect")
    assert detect.status == "failed" and detect.errors[0]["code"] == "CONFIG_INVALID" and "arm-1" in detect.reason
    assert starts(tool) == []


def test_publish_failure_fails_export(twin, tmp_path):
    folder, tool = twin
    scenario(tool, {"publish": False})
    export = result(run(folder, tmp_path), "export")
    assert export.status == "failed" and export.errors[0]["code"] == "PUBLISH_FAILED"


def test_missing_tool_and_model(twin, tmp_path, monkeypatch):
    folder, tool = twin
    monkeypatch.delenv("TWIN_SW_URDF_TOOL", raising=False)
    data = yaml.safe_load((folder / "twin.yaml").read_text(encoding="utf-8"))
    del data["cad"]["tool"]
    (folder / "twin.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    assert result(run(folder, tmp_path), "detect").errors[0]["code"] == "TOOL_NOT_CONFIGURED"
    monkeypatch.setenv("TWIN_SW_URDF_TOOL", str(tool))
    assert result(run(folder, tmp_path, stages=CAD_STAGES[:1]), "detect").status == "succeeded"
    (folder / "source" / "robot.SLDASM").unlink()
    assert result(run(folder, tmp_path), "detect").errors[0]["code"] == "MODEL_NOT_FOUND"


def test_twin_yaml_is_checked_against_the_exported_urdf(twin, tmp_path):
    folder, tool = twin
    data = yaml.safe_load((folder / "twin.yaml").read_text(encoding="utf-8"))
    data["home"]["wrist_joint"] = 0.1
    (folder / "twin.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    ingest = result(run(folder, tmp_path), "ingest")
    assert ingest.status == "failed" and ingest.errors[0]["code"] == "SPEC_URDF_MISMATCH" and "wrist_joint" in ingest.reason
