# Stage contract

## Layout of a run

```
runs/<robot>/<YYYYmmdd-HHMMSS>/
  twin.yaml            copy of the spec the run used
  run.json             summary: status, stages, environment
  <stage>/result.json  one per stage that ran
  <stage>/...          files the stage produced (only inside its own folder)
```

## A stage

A stage is a `Stage(name, run, description)` in `src/twin_pipeline/stages/<module>.py`. `run(context)` returns a `StageResult` and does not raise for expected problems; an exception is caught and recorded as `error` (`STAGE_CRASHED`, with `traceback.txt`).

`StageContext` gives:

- `spec` — the loaded `TwinSpec` (`spec.section(name)`, `spec.resolve(path)`, `spec.limits(joint, urdf)`, `spec.source_of(joint, value)`)
- `stage_dir` — the stage's own folder, emptied before it runs; write only here
- `output(stage, key)` — a file an earlier stage of this run produced, or `None` if that stage did not pass
- `urdf()` — the URDF to use: `ingest`'s output, else `twin.yaml`'s `urdf`
- `rel(path)` — how to store a path in `outputs` (relative to the run when inside it)
- `options` — `key=value` pairs from `twin run --option`

## result.json

```json
{
  "schema_version": 1,
  "stage": "mjcf",
  "status": "succeeded",
  "passed": true,
  "reason": null,
  "applicable": null,
  "outputs": {"mjcf": "mjcf/model.xml"},
  "metrics": {"nbody": 6},
  "warnings": [],
  "errors": [],
  "review": [],
  "approved": false,
  "approved_at": null,
  "started_at": "...", "finished_at": "...", "duration_seconds": 0.4
}
```

| status | meaning | stops the run |
|---|---|---|
| `succeeded` | did its job; outputs are usable | no |
| `failed` | found a problem; `errors[]` = `{code, message}` saying what, expected and found | yes |
| `error` | crashed | yes |
| `needs_review` | did its job but a person must decide; `review[]` = `{item, reason}` | yes, until approved |
| `skipped` | did not run; `applicable: false` = nothing to do for this twin, `true` = should have run but could not | no |

`passed` is set by the runner only: true for `succeeded`, and for `needs_review` after `--approve`.

Use the factories: `StageResult.succeeded/failed/skipped/needs_review(...)`.

## Outputs each stage provides

Downstream stages depend on these keys; changing one is a contract change (update this table and the consumers in the same commit).

| Stage | Outputs | Notes |
|---|---|---|
| `detect` | — | metrics: `config_stale`, `model_sha256` |
| `export` | `package`, `urdf` | metrics: `job_id` of solidworks-urdf-tool |
| `ingest` | `package`, `urdf` | the archived copy; `context.urdf()` prefers it |
| `mjcf` | `mjcf`, `assets` | `mjcf/model.xml`, loads with `MjModel.from_xml_path` and can be moved together with `assets/` (`meshdir="assets"`); `assets` is that folder, always present. Actuators and the `home` keyframe are named after joints, fixed frames are bodies and sites named after their links. metrics: `nbody`, `njnt`, `nu`, `ngeom`, `nmesh`, `nmeshface`, `nkey`, `nsite`, `visual_geoms`, `collision_geoms`, `mesh_files`, `meshes_split`, `mesh_parts`, `meshes_converted`, `max_faces`, `gravity`, `collision_links` (links per mode), `actuators` ({joint: type}), `fixed_frames` |
| `smoke` | `smoke` | JSON of the checks and their numbers |
| `report` | `html` | self-contained page |

## Run status

`run.json` `status`: `succeeded` (all ran and passed, or were not applicable), `failed`, `needs_review`, `incomplete` (an applicable stage was skipped or not run), `partial` (a `--stages` subset ran and passed). `twin run` exits 0 / 1 / 2 / 3 for succeeded or partial / failed / needs_review / incomplete.

## Resume and approve

`twin run twin.yaml --resume <run_dir> [--approve <stage>]` keeps every stage that passed until the first one that did not, approves the named `needs_review` stages, and re-runs from there; once a stage re-runs, every later stage re-runs too. Resume is refused if `twin.yaml` changed since the run started. `--approve` is only accepted with `--resume`, so a stage can only be approved after its review items were written.
