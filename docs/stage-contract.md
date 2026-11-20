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

A stage is a `Stage(name, run, description, always=False)` in `src/twin_pipeline/stages/<module>.py`. A stage with `always=True` (the report) also runs after an earlier stage stopped the run; it sees the results so far and does not change `stopped_at` or the run status. `run(context)` returns a `StageResult` and does not raise for expected problems; an exception is caught and recorded as `error` (`STAGE_CRASHED`, with `traceback.txt`).

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
| `detect` | `validation` (the tool's validate-config answer, when `cad.config` is set) | reads files only. metrics: `model_sha256`, `config_stale`, `export_needed`, `export_reasons` (why a new export is needed: model file, any CAD file the archived export read, or the configuration without its `source` block changed, or nothing is archived), `archived_job` |
| `export` | `package`, `urdf`, `check`, `job_result` | only when detect says so or `--option force_export=true`; else succeeded with `reused: true` and no outputs. A check-only job first (`cad.check_first`, default true with a config), then the export job with `--publish-to <archive>`; the tool's error code becomes the stage's (e.g. `CONFIG_OUTDATED`, `UNSAVED_CHANGES`), `PUBLISH_FAILED` if publishing failed. metrics: `job_id`, `check_job`, `reused`, `forced` |
| `ingest` | `package`, `urdf`, `preview_*` | the archived package; `context.urdf()` prefers it. twin.yaml is cross-checked against this URDF (`SPEC_URDF_MISMATCH`). After a new export: `needs_review` with the preview renders, approve with `--resume <run> --approve ingest`; a reused export passes. metrics: `job_id`, `new_export`, `links`, `joints`, `total_mass_kg` |
| `mjcf` | `mjcf`, `assets` | `mjcf/model.xml`, loads with `MjModel.from_xml_path` and can be moved together with `assets/` (`meshdir="assets"`); `assets` is that folder, always present. Actuators and the `home` keyframe are named after joints, fixed frames are bodies and sites named after their links. metrics: `nbody`, `njnt`, `nu`, `ngeom`, `nmesh`, `nmeshface`, `nkey`, `nsite`, `visual_geoms`, `collision_geoms`, `mesh_files`, `meshes_split`, `mesh_parts`, `meshes_converted`, `max_faces`, `gravity`, `collision_links` (links per mode), `actuators` ({joint: type}), `fixed_frames`, `neq`, `mimic` ({follower: {leader, multiplier, offset}}) |
| `smoke` | `smoke` | `smoke/smoke.json`: settings, model sizes, per joint the effective limits, how it is driven, home value, each sweep target with the position reached, error and actuator force, the settle numbers, and `checks` = `{load, model, home, sweep, settle}` each `passed` / `failed` / `review` / `not run`. Written for every status except skipped. metrics: `sweep_targets`, `sweep_reached`, `sweep_max_error_rad`, `sweep_max_error_m`, `joints_without_actuator`, `home_contacts`, `settle_max_angular_speed`, `mimic_checks`, `mimic_followed`, `mimic_max_error_rad`, `mimic_max_error_m`, ... |
| `report` | `html` | `report/report.html`: one self-contained page (inline CSS, no scripts, no network requests). metrics: `assumed_values`, `assumptions_assumed`, `urdf_values_without_provenance`, `previous_run` |

### smoke

Reads `mjcf`; skipped (applicable) when there is none or MuJoCo is not installed. Checks in order, stopping after the first that fails:

1. **load** — `MjModel.from_xml_path`. Failure: `MJCF_LOAD_FAILED`.
2. **model** — every movable URDF joint is in the MJCF (`MISSING_JOINT`) with the matching type (`JOINT_TYPE_MISMATCH`) and exactly the effective limits, `spec.limits` with `context.urdf()`; continuous joints unlimited (`LIMITS_MISMATCH`); gravity points against `up_axis` (`GRAVITY_MISMATCH`). Joints only in the MJCF are checked against their own range, with a warning; free and ball joints are held fixed.
3. **home** — twin.yaml `home`, else 0, inside the effective limits (`HOME_OUTSIDE_LIMITS` for a given value; review when the default 0 is outside, and the smoke test uses the nearest limit); contacts deeper than 1 mm at home are a review item.
4. **sweep** — each hinge/slide joint, one at a time from home, to both sweep targets for `sweep_seconds` with the model's own actuators: position actuators get the target as ctrl, velocity and motor actuators are driven by a P / PD-plus-gravity loop the smoke test closes (2 Hz bandwidth), always within ctrlrange. A joint with no actuator is set through qpos directly (said in warnings and in `smoke.json` `mode`), and while another joint is swept it is held at home. A target not reached within the tolerance is a review item saying how far off it is and whether the actuator was at its force limit or in contact. A joint that follows another through an equality constraint (a URDF mimic) is not swept itself: its home comes from its leader's, and at every sweep target of the leader it must sit where the constraint puts it (`followers` in each sweep entry); off by more than the tolerance, or put outside its own limits, is a review item.
5. **settle** — hold home under gravity for `settle_seconds`. NaN/Inf, MuJoCo's bad-qpos/qvel/qacc warnings, or a joint faster than 100 rad/s or 10 m/s in any phase is `SIMULATION_UNSTABLE`; a joint still moving faster than 0.05 rad/s or 5 mm/s at the end is a review item.

`failed` is for defects; `needs_review` is for what a person judges (a target not reached might be blocked by design). A failed smoke result still lists the review items found before it stopped.

### report

The page shows the run status, each stage's status, reason, warnings, errors, review items, metrics and outputs (relative links), the smoke numbers per joint, the provenance tables (every joint value set in twin.yaml with its source, every assumption, effort/velocity the URDF gives without a provenance; `assumed` rows highlighted) and the differences from the previous run of the same robot in the same runs folder (stage statuses, metrics, twin.yaml values and their sources).

The report stage is declared `always=True`: it runs even after a failed or unapproved stage stopped the run, and sees the results of the stages that ran. Its run status is marked as provisional because run.json is written after it. To redraw the page of any run from disk, use `python -m twin_pipeline.report <run folder> [--out page.html]` (default `<run>/report/report.html`; no result.json is written).

## Run status

`run.json` `status`: `succeeded` (all ran and passed, or were not applicable), `failed`, `needs_review`, `incomplete` (an applicable stage was skipped or not run), `partial` (a `--stages` subset ran and passed). `twin run` exits 0 / 1 / 2 / 3 for succeeded or partial / failed / needs_review / incomplete.

## Resume and approve

`twin run twin.yaml --resume <run_dir> [--approve <stage>]` keeps every stage that passed until the first one that did not, approves the named `needs_review` stages, and re-runs from there; once a stage re-runs, every later stage re-runs too. Resume is refused if `twin.yaml` changed since the run started. `--approve` is only accepted with `--resume`, so a stage can only be approved after its review items were written.
