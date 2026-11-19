# twin-pipeline

> **Status: pre-alpha.** The pipeline skeleton, the `twin.yaml` format and the stage contract are in place. The MuJoCo build, smoke checks, report and CAD stages are being written (see [docs/ROADMAP.md](docs/ROADMAP.md)).

A config-driven pipeline that turns a validated URDF into a tested MuJoCo digital twin:

```
CAD ──▶ detect ──▶ export ──▶ ingest ──▶ mjcf ──▶ smoke ──▶ report
        (CAD changed?) (URDF)   (archive)  (MuJoCo)  (checks)  (HTML)
```

- **One file per robot.** `twin.yaml` names the URDF (or the CAD it comes from) and adds what CAD does not know: actuators, collision policy, home pose, calibrated joint values and assumptions. Every value that is not from CAD says where it comes from (`cad`, `assumed`, `measured`, `calibrated`), and the report lists what is still assumed.
- **Every stage leaves evidence.** Each stage writes `runs/<robot>/<run>/<stage>/result.json` with a status, outputs, metrics, warnings and errors. A failed stage stops the run; a stage that needs a person to decide stops it until the run is resumed with `--approve`.
- **CAD is optional.** The CAD stages use [solidworks-urdf-tool](https://github.com/yunylll/solidworks-urdf-tool) (SolidWorks 2026, Windows, needs your own SolidWorks licence). Without a `cad` section the pipeline starts from the URDF, on any OS.

## Quick start

```bash
python -m venv .venv
.venv/bin/python -m pip install -e ".[dev,sim]"     # Windows: .venv\Scripts\python.exe
.venv/bin/twin validate examples/arm_demo/twin.yaml
.venv/bin/twin run examples/arm_demo/twin.yaml
.venv/bin/python -m pytest
```

`twin run` exits 0 when every stage succeeded, 1 on a failure, 2 when a stage needs review and 3 when the run is incomplete (a stage that should have run was skipped).

## Documentation

- [docs/twin-yaml.md](docs/twin-yaml.md) — the `twin.yaml` format
- [docs/stage-contract.md](docs/stage-contract.md) — what a stage gets, what it must write, how runs stop and resume
- [docs/ROADMAP.md](docs/ROADMAP.md) — what is being built and who owns which files

## License

MIT, see [LICENSE](LICENSE).
