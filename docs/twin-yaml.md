# twin.yaml (schema_version 1)

One file per robot. Relative paths are relative to the file. `twin validate <file>` checks it, and cross-checks it against the URDF when the URDF exists. Every error names the field, what was expected and what was found. Unknown keys are errors, so a typo never passes silently.

Example: [examples/arm_demo/twin.yaml](../examples/arm_demo/twin.yaml).

## Core (owned by `spec.py`)

| Key | Required | Meaning |
|---|---|---|
| `schema_version` | yes | `1` |
| `robot` | yes | Name: letters, digits, `_`, `-`; starts with a letter. Runs go to `runs/<robot>/`. |
| `description` | no | Free text. |
| `urdf` | unless `cad` is given | The URDF downstream stages use. With a `cad` section, `ingest` provides it instead. |
| `up_axis` | no | Which model axis points up: `+x -x +y -y +z -z`, default `+z`. The mjcf stage sets gravity from it. |
| `joints` | no | Per-joint values CAD does not know, each with a provenance (below). |
| `home` | no | `{joint: value}` — the rest pose (rad or m). Must be inside the effective limits. |
| `assumptions` | no | List of `{id, text, source, value?, note?}` — things the twin relies on that are not in any value above. |

### joints

```yaml
joints:
  elbow_joint:
    lower: -2.0          # rad (revolute) or m (prismatic)
    upper: 2.0
    effort: 10.0         # > 0
    velocity: 2.0        # > 0
    damping: 0.1         # >= 0
    friction: 0.0        # >= 0
    zero_offset: 0.0     # added to the URDF zero
    source: assumed      # cad | assumed | measured | calibrated — applies to every value given
    sources: {effort: measured}   # optional per-value provenance, overrides source
    note: Soft limits until the real stops are measured.
```

Every value given needs a provenance, through `source` or `sources.<value>`. An override replaces the URDF value; values not given keep the URDF's. Against the URDF: the joint must exist and be movable; `lower`/`upper` are not allowed on a continuous joint; the merged lower must not exceed the merged upper.

Provenance meanings: `cad` taken from the model; `assumed` a working value nobody has checked; `measured` read off the real machine or its datasheet; `calibrated` fitted from recorded data. The report lists every `assumed` value and assumption.

## Stage sections

Each section belongs to one stage module, which validates it (`SECTIONS`) and cross-checks it against the URDF (`URDF_CHECKS`). The owner may extend its sections; the table is updated with the code.

| Section | Owner | Keys today |
|---|---|---|
| `cad` | `stages/cad.py` | `model` (required), `config`, `package`, `tool`, `archive`, `timeout_seconds`, `stall_timeout_seconds` |
| `meshes` | `stages/mjcf.py` | `max_faces` (int ≥ 1; MuJoCo's STL decoder limit is 200 000 faces per file) |
| `collision` | `stages/mjcf.py` | `default`, `links: {link: mode}`; mode is `none`, `mesh`, `convex_hull` or `convex_decomposition` |
| `actuators` | `stages/mjcf.py` | `default`, `joints: {joint: {...}}`; each `{type: position\|velocity\|motor\|none, kp, kv}` |
| `smoke` | `stages/smoke.py` | `sweep_fraction` (0–1], `settle_seconds` > 0, `timestep` > 0 |
| `report` | `stages/report.py` | `title` |
