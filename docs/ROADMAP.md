# Roadmap and work split

## Phase 0 — skeleton (done)

`twin.yaml` format and validation, stage contract, runner with stop/resume/approve, CLI, placeholder stages, the primitive-shape demo arm, tests.

## Phase 1 — parallel work (done, merged 2026-11-19)

Integration fixes at the merge: `TwinSpec.limits` applies `zero_offset` for every stage (it was only applied in the mjcf builder, so smoke reported `LIMITS_MISMATCH`); stages can be `always=True` and the report is, so a stopped run still gets its page; the builder uses MuJoCo's `implicitfast` integrator; the demo arm's gains are high enough to hold its pose under gravity, so `twin run` passes end to end on both examples.

Open from phase 1: actuator gains (`kp`, `kv`) have no provenance yet, so the report cannot list them as assumed. URDF `<mimic>` joints: done (joint equality constraints, checked by smoke).


Each piece owns its files. Do not edit another piece's files; if a shared file must change, keep the edit small and additive and say so in the commit message.

| Piece | Owns | Done when |
|---|---|---|
| **A. MuJoCo build** (`mjcf` stage) | `src/twin_pipeline/stages/mjcf.py`, `src/twin_pipeline/mjcf/` (new), `tests/test_mjcf*.py`, `examples/*/` mesh fixtures it adds | `twin run examples/arm_demo/twin.yaml` gets `mjcf: succeeded` with a saved MJCF; handles meshes over `meshes.max_faces` (split losslessly), `up_axis` (gravity), `collision` modes, `actuators` per joint within effective limits, `joints` overrides (limits, damping, friction, effort), fixed links MuJoCo would merge; a mesh-based example (a generated STL, no lab CAD) is tested too |
| **B. Smoke checks and report** (`smoke`, `report` stages) | `src/twin_pipeline/stages/smoke.py`, `src/twin_pipeline/stages/report.py`, `src/twin_pipeline/report/` (new), `tests/test_smoke*.py`, `tests/test_report*.py` | smoke: headless load, home pose inside limits, joint sweep at `sweep_fraction` of the effective range reaches its targets, no NaN or blow-up while settling; `needs_review` where a person has to judge. report: one self-contained HTML per run with stage results, metrics, warnings, the provenance table (every `assumed` value and assumption) and links to outputs |
| **C. solidworks-urdf-tool release prep** | the `solidworks-urdf-tool` repository only | pending change committed with tests, public README pointing at twin-pipeline, no machine paths or personal data in tracked files, CI for the offline tests |

B does not need A's code: until A lands, B's tests build an MJCF from the demo URDF with `mujoco.MjModel.from_xml_path(urdf)` and `mujoco.mj_saveLastXML`, and feed it in as a fake `mjcf` result.

Shared files already set up so A and B do not need to touch them: `pyproject.toml` (the `sim` extra has `mujoco` and `numpy`), CI, `stages/__init__.py` (fixed order), `result.py`, `pipeline.py`, `stage.py`.

## Phase 2 — integration (in progress)

Done: `stages/cad.py` (detect / export / ingest) on solidworks-urdf-tool's CLI, with its `--publish-to` doing the archiving; offline tests against a fake tool (`tests/fake_sw_tool`).


- `stages/cad.py`: `detect` (solidworks-urdf-tool `validate-config --model`, no SolidWorks start), `export` (`check` then export through the tool's Python API, reusing the prepared copy), `ingest` (copy the package into `cad.archive`, earlier exports to `history/`).
- Run on real twins from a private repository that keeps their `twin.yaml`, CAD and calibration; nothing of them comes into this repository.
- Then: ROS 2 package stage, system identification writing `calibrated` values back.
