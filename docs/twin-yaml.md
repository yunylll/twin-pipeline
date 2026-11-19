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
| `meshes` | `stages/mjcf.py` | `max_faces` (int ≥ 1, default 200 000 = MuJoCo's STL decoder limit; STL files with more faces are split losslessly), `packages: {name: folder}` (where `package://name/` points; default: the nearest folder called `name` above the URDF) |
| `collision` | `stages/mjcf.py` | `default` (default `mesh`), `links: {link: mode}`; mode is `none`, `mesh`, `convex_hull` or `convex_decomposition` (needs `pip install coacd`, else the stage fails) |
| `actuators` | `stages/mjcf.py` | `default`, `joints: {joint: {...}}`; each `{type: position\|velocity\|motor\|none, kp, kv}`; `type` is required, `position` needs `kp` (`kv` optional), `velocity` needs `kv`, `motor`/`none` take neither |
| `smoke` | `stages/smoke.py` | `sweep_fraction` (0–1], `settle_seconds` > 0, `timestep` > 0 |
| `report` | `stages/report.py` | `title` |

### mjcf stage: what the sections do

The `mjcf` stage builds `mjcf/model.xml` and `mjcf/assets/` (details in `src/twin_pipeline/mjcf/build.py`):

- **Kinematics and mass** come from the URDF through MuJoCo's own importer. Mass and inertia are the URDF inertials only (`inertiafromgeom="false"`), so meshes never add mass and splitting or replacing collision geometry cannot change the dynamics. A moving link without an inertial makes MuJoCo refuse the model (`MJCF_COMPILE_FAILED`); nothing is guessed for it.
- **Fixed links stay.** A link on a fixed joint (e.g. `tool_frame`) is kept as its own body instead of being fused into its parent, and gets a site of the same name.
- **Meshes** (`meshes`): STL, OBJ and MSH are accepted, anything else fails with `MESH_FORMAT_UNSUPPORTED`. ASCII STL is rewritten as binary STL (MuJoCo reads binary only). An STL over `max_faces` is split into consecutive runs of whole faces, one visual geom per part; the triangles are unchanged. `package://`, `file://`, absolute and URDF-relative paths are resolved; every missing file is listed (`MESH_NOT_FOUND`).
- **Gravity** is 9.81 m/s² against `up_axis` (`+y` gives `0 -9.81 0`).
- **Geoms**: visual geoms are group 2 and never collide. Collision geoms are group 3 and follow `collision`, per link:
  - `none` — no collision geoms (e.g. a base shell whose hull would swallow the arm);
  - `mesh` — the URDF `<collision>` elements as they are; MuJoCo collides each mesh geom through its convex hull, so a split mesh collides as the hulls of its parts (the stage warns);
  - `convex_hull` — primitives as they are, all collision meshes of the link replaced by one convex hull (computed by MuJoCo, saved as `<link>_hull.stl`);
  - `convex_decomposition` — primitives as they are, the link's collision meshes decomposed with CoACD into convex parts (`<link>_decomp<k>.stl`); fails with `CONVEX_DECOMPOSITION_UNAVAILABLE` when `coacd` is not installed.
- **Joint overrides** (`joints`): `lower`/`upper` set the range, `damping` and `friction` (MuJoCo `frictionloss`) replace the URDF `<dynamics>`, `effort` becomes the joint's actuator force range `±effort`. `velocity` only bounds a velocity actuator (MuJoCo has no joint speed limit). `zero_offset: o` moves the joint zero to URDF position `o`: the twin's joint reads `q = q_urdf − o` (MuJoCo `ref = −o`), URDF limits are shifted by `−o`, limits given in twin.yaml are taken as already in the twin's coordinate.
- **Actuators** (`actuators`): one per joint that `actuators.joints.<joint>` or `actuators.default` asks for; a joint entry replaces the default entirely, and a joint with neither gets none. The actuator has the joint's name. `ctrlrange` is the effective range for `position`, `±velocity` for `velocity`, `±effort` for `motor`, and open when that value is unknown (e.g. a position actuator on a continuous joint; the stage warns).
- **Home** becomes the keyframe `home`: listed joints at their value, the others at their zero, position actuators' `ctrl` at the pose. No `home`, no keyframe.
