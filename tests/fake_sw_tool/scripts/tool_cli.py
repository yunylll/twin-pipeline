"""Stand-in for solidworks-urdf-tool's scripts/tool_cli.py in the tests: same commands and JSON shapes, no SolidWorks.

Behaviour comes from <tool>/scenario.json:
  validate   the answer of validate-config (default: passed)
  check      fields of the check job's final record (default: succeeded, passed)
  export     fields of the export job's final record (default: succeeded, passed)
  publish    false: the export passes but publishing fails
  urdf       the URDF file a passed export publishes
Every call is appended to <tool>/calls.jsonl.
"""

import hashlib
import json
import shutil
import sys
import uuid
from pathlib import Path

TOOL = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def option(args, name, default=None):
    return args[args.index(name) + 1] if name in args else default


def main():
    args = sys.argv[1:]
    scenario = json.loads((TOOL / "scenario.json").read_text(encoding="utf-8")) if (TOOL / "scenario.json").exists() else {}
    with open(TOOL / "calls.jsonl", "a", encoding="utf-8") as log:
        log.write(json.dumps(args) + "\n")
    jobs = TOOL / "jobs"
    jobs.mkdir(exist_ok=True)
    command = args[0]
    if command == "validate-config":
        answer = {"passed": True, "errors": [], "warnings": [], "source_status": {"stale": False}}
        answer.update(scenario.get("validate", {}))
    elif command == "start-export":
        kind = "check" if "--check-only" in args else "export"
        job_id = f"{kind}-{uuid.uuid4().hex[:8]}"
        (jobs / f"{job_id}.json").write_text(json.dumps({"kind": kind, "args": args}), encoding="utf-8")
        answer = {"job_id": job_id, "job": str(jobs / job_id), "status": "queued"}
    elif command == "job":
        job_id = args[1]
        job = json.loads((jobs / f"{job_id}.json").read_text(encoding="utf-8"))
        kind, start = job["kind"], job["args"]
        answer = {"job_id": job_id, "status": "succeeded", "passed": True, "operation": kind, "warnings": []}
        answer.update(scenario.get(kind, {}))
        if kind == "export" and answer["status"] == "succeeded" and answer["passed"] and "--publish-to" in start:
            answer["publish"] = publish(start, job_id, scenario, answer)
    else:
        answer = {"passed": False, "status": "rejected", "error": {"code": "INVALID_INPUT", "message": f"fake tool: no {command}"}}
    print(json.dumps(answer, indent=2))
    return 0 if answer.get("passed") else 1


def publish(start, job_id, scenario, answer):
    if scenario.get("publish") is False:
        return {"passed": False, "error": {"code": "PUBLISH_FAILED", "message": "fake publish failure"}}
    model, target, package = Path(start[1]), Path(option(start, "--publish-to")), option(start, "--package")
    config = option(start, "--config")
    export, destination = target / "export", target / "urdf" / package
    moved = []
    if destination.exists() or (export / "export-result.json").exists():
        history = export / "history" / f"job-{job_id}"
        history.mkdir(parents=True)
        if destination.exists():
            shutil.move(str(destination), str(history / "urdf"))
        for name in ("export-config.json", "export-result.json"):
            if (export / name).exists():
                shutil.move(str(export / name), str(history / name))
        moved.append(str(history))
    (destination / "urdf").mkdir(parents=True)
    shutil.copy2(scenario["urdf"], destination / "urdf" / f"{package}.urdf")
    (destination / "previews").mkdir()
    (destination / "previews" / "front.png").write_bytes(b"\x89PNG fake")
    export.mkdir(parents=True, exist_ok=True)
    if config:
        shutil.copy2(config, export / "export-config.json")
    record = dict(answer, source_model_sha256=sha(model), source_hashes_before={str(model): sha(model)},
                  validation={"passed": True, "links": 6, "joints": 5})
    (export / "export-result.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    block = {"passed": True, "package": {"path": str(destination)}}
    if moved:
        block["history"] = moved[0]
    return block


if __name__ == "__main__":
    sys.exit(main())
