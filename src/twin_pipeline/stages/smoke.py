"""smoke stage: headless checks on the MuJoCo model the mjcf stage built.

Owner: session B (docs/ROADMAP.md). Section owned: smoke. Output: ``smoke`` (smoke/smoke.json).

Checks, in order (each writes its numbers to smoke.json):

1. load    the MJCF loads headless with ``MjModel.from_xml_path``
2. model   every movable URDF joint exists with the right type and the effective limits
           (twin.yaml ``joints`` merged with the URDF); gravity points against ``up_axis``
3. home    the home pose (twin.yaml ``home``, else zeros) is inside the effective limits and has no
           deep contacts
4. sweep   each hinge/slide joint is driven to ``sweep_fraction`` of its effective range on both sides
           by the model's own actuators and must reach the target within the tolerance; a joint with
           no actuator is set through qpos directly and only checked for range and stability
5. settle  the model holds home for ``settle_seconds`` under gravity: no NaN, no blow-up, at rest

``failed`` is for clear defects (does not load, missing joint, wrong limits or gravity, NaN, blow-up).
``needs_review`` is for what a person has to judge: a target not reached (weak actuator, or blocked
by design?), a joint still moving after settling, contacts at home, an actuator that cannot command
part of the range.
"""

from __future__ import annotations

import json
import math

from .. import checks
from ..result import StageResult
from ..stage import Stage, StageContext
from ..urdf import read_urdf

DEFAULTS = {"sweep_fraction": 0.9, "settle_seconds": 1.0, "sweep_seconds": 2.0, "angle_tolerance": 0.05, "distance_tolerance": 0.002}
SMOKE_KEYS = ("sweep_fraction", "settle_seconds", "timestep", "sweep_seconds", "angle_tolerance", "distance_tolerance")
MAX_ANGULAR_SPEED = 100.0  # rad/s; above this in a smoke test the simulation has blown up
MAX_LINEAR_SPEED = 10.0  # m/s
REST_ANGULAR_SPEED = 0.05  # rad/s; below this at the end of settling a joint is at rest
REST_LINEAR_SPEED = 0.005  # m/s
HOME_PENETRATION = 0.001  # m; deeper contacts at the home pose need a look
LIMIT_TOLERANCE = 1e-5  # rad or m, for comparing the MJCF range with the effective limits
CONTROL_BANDWIDTH = 2 * math.pi * 2.0  # rad/s, for the loops the smoke test closes itself (motor, velocity)


def validate_smoke(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, SMOKE_KEYS)
    if "sweep_fraction" in value:
        errors += checks.number(f"{where}.sweep_fraction", value["sweep_fraction"], low=0, low_open=True, high=1)
    for key in ("settle_seconds", "timestep", "sweep_seconds", "angle_tolerance", "distance_tolerance"):
        if key in value:
            errors += checks.number(f"{where}.{key}", value[key], low=0, low_open=True)
    return errors


class _Findings:
    def __init__(self):
        self.errors: list[dict] = []
        self.review: list[dict] = []
        self.warnings: list[str] = []

    def fail(self, code, message):
        self.errors.append({"code": code, "message": message})

    def ask(self, item, reason):
        self.review.append({"item": item, "reason": reason})


def _round(value, digits=6):
    return None if value is None else round(float(value), digits)


def _actuator_kind(mujoco, model, index) -> str:
    gain, bias = model.actuator_gaintype[index], model.actuator_biastype[index]
    gp, bp = model.actuator_gainprm[index], model.actuator_biasprm[index]
    if model.actuator_dyntype[index] not in (mujoco.mjtDyn.mjDYN_NONE, mujoco.mjtDyn.mjDYN_FILTER, mujoco.mjtDyn.mjDYN_FILTEREXACT):
        return "unsupported"
    if gain != mujoco.mjtGain.mjGAIN_FIXED:
        return "unsupported"
    if bias == mujoco.mjtBias.mjBIAS_NONE:
        return "motor"
    if bias == mujoco.mjtBias.mjBIAS_AFFINE and gp[0] > 0:
        if bp[0] == 0 and bp[1] < 0 and math.isclose(-bp[1], gp[0], rel_tol=1e-9):
            return "position"
        if bp[0] == 0 and bp[1] == 0 and bp[2] < 0 and math.isclose(-bp[2], gp[0], rel_tol=1e-9):
            return "velocity"
    return "unsupported"


class _Sim:
    """The model plus the controller the smoke test runs on it."""

    def __init__(self, mujoco, model, joints, home):
        self.mujoco, self.model, self.data = mujoco, model, mujoco.MjData(model)
        self.joints = joints  # name -> info dict (id, qadr, dadr, kind, actuators)
        self.home = home  # full qpos vector
        self.held = []  # (qadr, nq, dadr, nv) joints held kinematically at home
        self.max_speed = {"angular": 0.0, "linear": 0.0}
        self.bad_state = None

    def reset(self):
        self.mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.home
        self.data.qvel[:] = 0
        self.mujoco.mj_forward(self.model, self.data)
        self.max_speed = {"angular": 0.0, "linear": 0.0}

    def control(self, targets):
        """Set ctrl so each actuated joint goes to targets[name]."""
        model, data = self.model, self.data
        for name, info in self.joints.items():
            if name not in targets:
                continue
            target = targets[name]
            q, qd = data.qpos[info["qadr"]], data.qvel[info["dadr"]]
            for act in info["actuators"]:
                index, kind, gear = act["id"], act["kind"], act["gear"]
                if kind == "position":
                    ctrl = gear * target
                elif kind == "velocity":
                    ctrl = gear * CONTROL_BANDWIDTH * (target - q)
                elif kind == "motor":
                    inertia = max(info["inertia"], 1e-9)
                    torque = data.qfrc_bias[info["dadr"]] + inertia * (CONTROL_BANDWIDTH ** 2 * (target - q) - 2 * CONTROL_BANDWIDTH * qd)
                    torque /= len([a for a in info["actuators"] if a["kind"] == "motor"])
                    ctrl = torque / (gear * model.actuator_gainprm[index][0])
                else:
                    ctrl = 0.0
                if model.actuator_ctrllimited[index]:
                    low, high = model.actuator_ctrlrange[index]
                    ctrl = min(max(ctrl, low), high)
                data.ctrl[index] = ctrl

    def step(self, targets, seconds, *, set_qpos=None, watch=None):
        """Simulate; set_qpos = {name: value} joints placed directly; watch(t) is called after each step."""
        model, data, mujoco = self.model, self.data, self.mujoco
        steps = max(1, int(round(seconds / model.opt.timestep)))
        for index in range(steps):
            self.control(targets)
            mujoco.mj_step(model, data)
            for qadr, nq, dadr, nv in self.held:
                data.qpos[qadr:qadr + nq] = self.home[qadr:qadr + nq]
                data.qvel[dadr:dadr + nv] = 0
            for name, value in (set_qpos or {}).items():
                info = self.joints[name]
                data.qpos[info["qadr"]] = value
                data.qvel[info["dadr"]] = 0
            if not self._state_ok():
                return False
            if watch is not None:
                watch((index + 1) * model.opt.timestep)
        return True

    def _state_ok(self):
        data = self.data
        for label, values in (("qpos", data.qpos), ("qvel", data.qvel), ("qacc", data.qacc)):
            if not all(math.isfinite(v) for v in values):
                self.bad_state = f"{label} is not finite at t={data.time:.3f} s"
                return False
        for warning in ("mjWARN_BADQPOS", "mjWARN_BADQVEL", "mjWARN_BADQACC"):
            if data.warning[getattr(self.mujoco.mjtWarning, warning)].number:
                self.bad_state = f"MuJoCo reported {warning} at t={data.time:.3f} s (it resets the state when this happens)"
                return False
        for name, info in self.joints.items():
            speed = abs(data.qvel[info["dadr"]])
            unit = "angular" if info["kind"] == "hinge" else "linear"
            self.max_speed[unit] = max(self.max_speed[unit], speed)
            bound = MAX_ANGULAR_SPEED if unit == "angular" else MAX_LINEAR_SPEED
            if speed > bound:
                self.bad_state = f"{name} moves at {speed:.4g} {'rad/s' if unit == 'angular' else 'm/s'} at t={data.time:.3f} s (bound {bound})"
                return False
        return True

    def saturation(self, info):
        """Largest |joint force from actuators| as a fraction of the joint's actuator force range (None if unlimited)."""
        model, data = self.model, self.data
        if not model.jnt_actfrclimited[info["id"]]:
            fractions = []
            for act in info["actuators"]:
                if model.actuator_forcelimited[act["id"]]:
                    low, high = model.actuator_forcerange[act["id"]]
                    bound = max(abs(low), abs(high))
                    if bound > 0:
                        fractions.append(abs(data.actuator_force[act["id"]]) / bound)
            return max(fractions) if fractions else None
        low, high = model.jnt_actfrcrange[info["id"]]
        bound = max(abs(low), abs(high))
        return abs(data.qfrc_actuator[info["dadr"]]) / bound if bound > 0 else None


def _check_model(mujoco, model, context, urdf_model, found: _Findings, report: dict):
    """Map MJCF joints, compare types and limits with the effective limits; return {name: info}."""
    names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j): j for j in range(model.njnt)}
    joints, rows = {}, {}
    kinds = {mujoco.mjtJoint.mjJNT_HINGE: "hinge", mujoco.mjtJoint.mjJNT_SLIDE: "slide",
             mujoco.mjtJoint.mjJNT_BALL: "ball", mujoco.mjtJoint.mjJNT_FREE: "free"}
    for name, j in names.items():
        kind = kinds[int(model.jnt_type[j])]
        label = name or f"joint #{j}"
        if kind in ("ball", "free"):
            found.warnings.append(f"{label} is a {kind} joint: held fixed at its initial pose during the smoke test, not swept")
            rows[label] = {"type": kind, "mode": "held"}
            continue
        limited = bool(model.jnt_limited[j])
        mj_range = [float(v) for v in model.jnt_range[j]] if limited else None
        info = {"id": j, "qadr": int(model.jnt_qposadr[j]), "dadr": int(model.jnt_dofadr[j]), "kind": kind, "actuators": [],
                "inertia": float(model.dof_M0[model.jnt_dofadr[j]]) if hasattr(model, "dof_M0") else 1.0}
        row = {"type": kind, "mjcf_range": mj_range, "effective_limits": mj_range, "limits_from": "mjcf"}
        urdf_joint = urdf_model.joints.get(name) if urdf_model else None
        if urdf_model is not None and urdf_joint is None:
            found.warnings.append(f"{label} is in the MJCF but not in the URDF: checked against its MJCF range only")
        if urdf_joint is not None:
            expected_kind = "slide" if urdf_joint.type == "prismatic" else "hinge"
            if urdf_joint.type not in ("revolute", "continuous", "prismatic"):
                found.fail("JOINT_TYPE_MISMATCH", f"{name}: the URDF joint is {urdf_joint.type} but the MJCF has a movable {kind} joint")
            elif kind != expected_kind:
                found.fail("JOINT_TYPE_MISMATCH", f"{name}: expected a {expected_kind} joint for the URDF {urdf_joint.type} joint, found {kind}")
            lower, upper = context.spec.limits(name, urdf_model) if urdf_joint.bounded else (None, None)
            effective = None if lower is None or upper is None else [float(lower), float(upper)]
            row["effective_limits"], row["limits_from"] = effective, "twin.yaml + URDF"
            if effective is None and mj_range is not None:
                found.fail("LIMITS_MISMATCH", f"{name}: expected an unlimited joint (URDF {urdf_joint.type}), found MJCF range {mj_range}")
            elif effective is not None and mj_range is None:
                found.fail("LIMITS_MISMATCH", f"{name}: expected range {effective} (effective limits), found an unlimited MJCF joint")
            elif effective is not None and any(abs(a - b) > LIMIT_TOLERANCE for a, b in zip(effective, mj_range)):
                found.fail("LIMITS_MISMATCH", f"{name}: expected range {effective} (effective limits), found MJCF range {mj_range}")
        joints[name] = info
        rows[name] = row
    if urdf_model is not None:
        for name, item in urdf_model.joints.items():
            if item.movable and name not in names:
                found.fail("MISSING_JOINT", f"{name}: movable {item.type} joint in the URDF, not found in the MJCF")

    # actuators: which drive which joint, and how the smoke test commands them
    for index in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index) or f"actuator #{index}"
        joint_id = int(model.actuator_trnid[index][0])
        if model.actuator_trntype[index] != mujoco.mjtTrn.mjTRN_JOINT or joint_id < 0:
            found.warnings.append(f"{act_name} does not act on a joint directly: left at ctrl 0")
            continue
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if joint_name not in joints:
            found.warnings.append(f"{act_name} acts on {joint_name}, which is not swept: left at ctrl 0")
            continue
        kind = _actuator_kind(mujoco, model, index)
        gear = float(model.actuator_gear[index][0])
        if kind == "unsupported" or gear == 0:
            found.warnings.append(f"{act_name} on {joint_name}: actuator type not supported by the smoke test (position, velocity, motor are): left at ctrl 0")
            continue
        joints[joint_name]["actuators"].append({"id": index, "name": act_name, "kind": kind, "gear": gear})

    # joints driven by an equality constraint (e.g. a URDF mimic) follow another joint
    followers = set()
    for e in range(model.neq):
        if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT and model.eq_active0[e] and model.eq_obj2id[e] >= 0:
            followers.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.eq_obj1id[e])))
    for name, info in joints.items():
        info["follower"] = name in followers
        acts = info["actuators"]
        if info["follower"]:
            mode = "follows an equality constraint"
        elif not acts:
            mode = "qpos (no actuator)"
            found.warnings.append(f"{name} has no actuator: swept by setting qpos directly (checks range and stability, not actuation); passive while settling")
        else:
            mode = " + ".join(f"{a['kind']} ({a['name']})" for a in acts)
        rows[name]["mode"] = mode
        rows[name]["actuators"] = [a["name"] for a in acts]
    report["joints"] = rows
    return joints


def _check_gravity(model, up_axis, found: _Findings, report: dict):
    gravity = [float(v) for v in model.opt.gravity]
    norm = math.sqrt(sum(v * v for v in gravity))
    axis = {"x": 0, "y": 1, "z": 2}[up_axis[1]]
    up = [0.0, 0.0, 0.0]
    up[axis] = 1.0 if up_axis[0] == "+" else -1.0
    report["gravity"] = {"vector": gravity, "up_axis": up_axis}
    if norm < 1e-9:
        found.fail("GRAVITY_MISMATCH", f"expected gravity along -({up_axis}), found no gravity {gravity}")
        return
    alignment = -sum(g * u for g, u in zip(gravity, up)) / norm
    report["gravity"]["alignment"] = _round(alignment)
    if alignment < 0.999:
        found.fail("GRAVITY_MISMATCH", f"expected gravity pointing against up_axis {up_axis}, found {gravity}")


def _sweep_targets(row, fraction):
    limits = row["effective_limits"]
    if limits is None:
        if row["type"] == "hinge":
            return [-fraction * math.pi, fraction * math.pi]
        return None
    middle, half = (limits[0] + limits[1]) / 2, (limits[1] - limits[0]) / 2
    return [middle - fraction * half, middle + fraction * half]


def run(context: StageContext) -> StageResult:
    path = context.output("mjcf", "mjcf")
    if path is None:
        return StageResult.skipped("smoke", "no MJCF: the mjcf stage did not pass in this run")
    try:
        import mujoco
    except ImportError:
        return StageResult.skipped("smoke", "mujoco is not installed: pip install twin-pipeline[sim]")

    settings = dict(DEFAULTS)
    settings.update(context.spec.section("smoke"))
    found, report = _Findings(), {"schema_version": 1, "mjcf": context.rel(path), "mujoco": mujoco.__version__, "settings": settings, "checks": {}}
    output = context.stage_dir / "smoke.json"

    def finish(metrics):
        report["checks"] = {name: report["checks"].get(name, "not run") for name in ("load", "model", "home", "sweep", "settle")}
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        outputs = {"smoke": context.rel(output)}
        if found.errors:
            first = found.errors[0]
            result = StageResult.failed("smoke", first["code"], first["message"], outputs=outputs, metrics=metrics,
                                        warnings=found.warnings, errors=found.errors[1:])
            result.review = list(found.review)
            return result
        if found.review:
            return StageResult.needs_review("smoke", found.review, outputs=outputs, metrics=metrics, warnings=found.warnings)
        return StageResult.succeeded("smoke", outputs=outputs, metrics=metrics, warnings=found.warnings)

    # 1. load
    try:
        model = mujoco.MjModel.from_xml_path(str(path))
    except Exception as error:  # MuJoCo raises ValueError with the compiler message
        found.fail("MJCF_LOAD_FAILED", f"{context.rel(path)} does not load in MuJoCo {mujoco.__version__}: {error}")
        report["checks"]["load"] = "failed"
        return finish({})
    report["checks"]["load"] = "passed"
    if "timestep" in context.spec.section("smoke"):
        model.opt.timestep = float(settings["timestep"])
    report["model"] = {"nq": model.nq, "nv": model.nv, "nu": model.nu, "nbody": model.nbody, "njnt": model.njnt,
                       "timestep": float(model.opt.timestep), "timestep_from": "twin.yaml smoke.timestep" if "timestep" in context.spec.section("smoke") else "mjcf"}

    # 2. model: joints, limits, gravity
    urdf_path = context.urdf()
    urdf_model = None
    if urdf_path is not None and urdf_path.exists():
        urdf_model = read_urdf(urdf_path)
    else:
        found.warnings.append(f"no URDF ({urdf_path}): limits checked against the MJCF only, twin.yaml joint overrides not compared")
    errors_before = len(found.errors)
    joints = _check_model(mujoco, model, context, urdf_model, found, report)
    _check_gravity(model, context.spec.up_axis, found, report)
    report["checks"]["model"] = "failed" if len(found.errors) > errors_before else "passed"
    metrics = {"nq": model.nq, "nu": model.nu, "timestep": float(model.opt.timestep), "joints": len(joints),
               "joints_without_actuator": sum(1 for info in joints.values() if not info["actuators"] and not info["follower"])}
    if found.errors:
        return finish(metrics)

    # 3. home pose
    home_spec = context.spec.data.get("home") or {}
    home = [float(v) for v in model.qpos0]
    home_rows, home_ok = {}, True
    for name, info in joints.items():
        row = report["joints"][name]
        value = float(home_spec.get(name, 0.0))
        source = "twin.yaml home" if name in home_spec else "zero (no home given)"
        limits = row["effective_limits"]
        inside = limits is None or limits[0] - LIMIT_TOLERANCE <= value <= limits[1] + LIMIT_TOLERANCE
        home_rows[name] = {"value": value, "from": source, "inside_limits": inside}
        if not inside:
            home_ok = False
            if name in home_spec:
                found.fail("HOME_OUTSIDE_LIMITS", f"home.{name}: expected inside the effective limits {limits}, found {value}")
            else:
                clamped = min(max(value, limits[0]), limits[1])
                found.ask(f"home pose of {name}", f"no home value given and 0 is outside the effective limits {limits}; the smoke test used {clamped:.4g}. Add home.{name} to twin.yaml")
                value = clamped
                home_rows[name]["used"] = value
        home[info["qadr"]] = value
    sim = _Sim(mujoco, model, joints, home)
    for j in range(model.njnt):
        if model.jnt_type[j] in (mujoco.mjtJoint.mjJNT_FREE, mujoco.mjtJoint.mjJNT_BALL):
            nq = 7 if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE else 4
            sim.held.append((int(model.jnt_qposadr[j]), nq, int(model.jnt_dofadr[j]), nq - 1))
    sim.reset()
    contacts = []
    for c in range(sim.data.ncon):
        contact = sim.data.contact[c]
        if contact.dist < -HOME_PENETRATION:
            pair = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or f"geom #{int(g)}" for g in contact.geom]
            bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) for g in contact.geom]
            contacts.append({"geoms": pair, "bodies": bodies, "depth": _round(-contact.dist)})
    if contacts:
        listed = "; ".join(f"{' / '.join(map(str, c['bodies']))} {c['depth'] * 1000:.1f} mm" for c in contacts[:5])
        found.ask("contacts at the home pose", f"{len(contacts)} contact(s) deeper than {HOME_PENETRATION * 1000:g} mm at home ({listed}): overlapping collision geometry, or touching by design?")
    report["home"] = {"joints": home_rows, "contacts": contacts}
    report["checks"]["home"] = "failed" if not home_ok and any(e["code"] == "HOME_OUTSIDE_LIMITS" for e in found.errors) else ("review" if contacts or not home_ok else "passed")
    metrics["home_contacts"] = len(contacts)
    if found.errors:
        return finish(metrics)

    # 4. joint sweep
    home_targets = {name: sim.home[info["qadr"]] for name, info in joints.items()}
    reached = total = 0
    worst = {"hinge": 0.0, "slide": 0.0}
    sweep_failed = False
    for name, info in joints.items():
        row = report["joints"][name]
        targets = _sweep_targets(row, settings["sweep_fraction"])
        if info["follower"]:
            row["sweep"] = []
            continue
        if targets is None:
            found.warnings.append(f"{name}: unlimited slide joint, no range to sweep")
            row["sweep"] = []
            continue
        tolerance = settings["angle_tolerance"] if info["kind"] == "hinge" else settings["distance_tolerance"]
        unit = "rad" if info["kind"] == "hinge" else "m"
        row["tolerance"], row["sweep"] = tolerance, []
        # unactuated joints other than the swept one are held at home so each sweep is about one joint
        others = [(other["qadr"], 1, other["dadr"], 1) for other_name, other in joints.items()
                  if other_name != name and not other["actuators"] and not other["follower"]]
        sim.held.extend(others)
        for target in targets:
            total += 1
            sim.reset()
            goal = dict(home_targets, **{name: target})
            entry = {"target": _round(target)}
            for act in info["actuators"]:
                if model.actuator_ctrllimited[act["id"]] and act["kind"] == "position":
                    low, high = model.actuator_ctrlrange[act["id"]]
                    command = act["gear"] * target
                    if not low - 1e-9 <= command <= high + 1e-9:
                        entry["ctrl_clipped"] = True
                        found.ask(f"{name} actuator range", f"{act['name']} has ctrlrange [{low:.4g}, {high:.4g}] but the sweep target {target:.4g} {unit} is inside the effective limits: the actuator cannot command that part of the range")
            first_inside = [None]
            peak = [0.0]

            def watch(t, name=name, target=target, tolerance=tolerance, info=info):
                if first_inside[0] is None and abs(sim.data.qpos[info["qadr"]] - target) <= tolerance:
                    first_inside[0] = t
                fraction = sim.saturation(info)
                if fraction is not None:
                    peak[0] = max(peak[0], fraction)

            direct = {name: target} if not info["actuators"] else None
            ok = sim.step(goal, settings["sweep_seconds"], set_qpos=direct, watch=watch)
            if not ok:
                found.fail("SIMULATION_UNSTABLE", f"sweep of {name} to {target:.4g} {unit}: {sim.bad_state}")
                entry["unstable"] = sim.bad_state
                row["sweep"].append(entry)
                sweep_failed = True
                break
            position = float(sim.data.qpos[info["qadr"]])
            error = abs(position - target)
            final_force = sim.saturation(info) if info["actuators"] else None
            entry.update({"reached": _round(position), "error": _round(error), "within_tolerance": error <= tolerance,
                          "time_to_tolerance": _round(first_inside[0], 4),
                          "peak_force_fraction": _round(peak[0], 3) if info["actuators"] else None,
                          "final_force_fraction": _round(final_force, 3)})
            worst[info["kind"]] = max(worst[info["kind"]], error)
            row["sweep"].append(entry)
            if error <= tolerance:
                reached += 1
            else:
                why = [f"reached {position:.4g} {unit}, {error:.3g} {unit} from the target (tolerance {tolerance:g}) after {settings['sweep_seconds']:g} s"]
                if final_force is not None and final_force >= 0.99:
                    why.append("the actuator force is at its limit: too weak for the load, or pushing against something")
                elif info["actuators"]:
                    why.append("the actuator is not at its force limit, so this looks like a steady-state error (gain too low for the load)")
                if sim.data.ncon:
                    why.append(f"{sim.data.ncon} contact(s) at the end: may be blocked by geometry")
                why.append("is it blocked by design, or does the actuator need more gain or force?")
                found.ask(f"{name} sweep to {target:.4g} {unit}", "; ".join(why))
        del sim.held[len(sim.held) - len(others):]
        if sweep_failed:
            break
    report["checks"]["sweep"] = "failed" if sweep_failed else ("review" if reached < total else "passed")
    metrics.update({"sweep_targets": total, "sweep_reached": reached, "sweep_max_error_rad": _round(worst["hinge"]),
                    "sweep_max_error_m": _round(worst["slide"])})
    if found.errors:
        return finish(metrics)

    # 5. settle at home under gravity
    sim.reset()
    energy_flag = mujoco.mjtEnableBit.mjENBL_ENERGY
    model.opt.enableflags |= energy_flag
    kinetic = [0.0]

    def watch_energy(t):
        kinetic[0] = max(kinetic[0], float(sim.data.energy[1]))

    ok = sim.step(home_targets, settings["settle_seconds"], watch=watch_energy)
    settle = {"seconds": settings["settle_seconds"], "max_angular_speed": _round(sim.max_speed["angular"]),
              "max_linear_speed": _round(sim.max_speed["linear"]), "max_kinetic_energy": _round(kinetic[0])}
    if not ok:
        found.fail("SIMULATION_UNSTABLE", f"settling at home: {sim.bad_state}")
        settle["unstable"] = sim.bad_state
        report["checks"]["settle"] = "failed"
    else:
        moving, final = [], {}
        for name, info in joints.items():
            speed = abs(float(sim.data.qvel[info["dadr"]]))
            position = float(sim.data.qpos[info["qadr"]])
            rest = REST_ANGULAR_SPEED if info["kind"] == "hinge" else REST_LINEAR_SPEED
            final[name] = {"position": _round(position), "drift": _round(abs(position - home_targets[name])), "speed": _round(speed),
                           "at_rest": speed <= rest, "passive": not info["actuators"]}
            if speed > rest:
                moving.append(name)
        settle["joints"] = final
        if moving:
            found.ask("settling", f"{', '.join(moving)} still moving after {settings['settle_seconds']:g} s at home under gravity "
                                  f"(rest bound {REST_ANGULAR_SPEED} rad/s, {REST_LINEAR_SPEED} m/s): passive joint without damping, or an oscillating controller?")
        report["checks"]["settle"] = "review" if moving else "passed"
        metrics["settle_moving_joints"] = len(moving)
    report["settle"] = settle
    metrics.update({"settle_max_angular_speed": settle["max_angular_speed"], "settle_max_linear_speed": settle["max_linear_speed"]})
    return finish(metrics)


STAGES = [Stage("smoke", run, "Load the MuJoCo model headless and check home pose, joint sweep and stability")]
SECTIONS = {"smoke": validate_smoke}
URDF_CHECKS = []
