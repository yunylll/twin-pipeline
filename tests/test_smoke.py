"""smoke stage. Until the mjcf stage lands, the MJCF comes from the demo URDF through MuJoCo's own URDF
import (MjSpec), with the twin.yaml limit override and actuators added, given to smoke as a fake mjcf stage."""

import json
from pathlib import Path

import pytest
import yaml

from twin_pipeline.pipeline import run_pipeline
from twin_pipeline.result import StageResult
from twin_pipeline.spec import load_spec
from twin_pipeline.stage import Stage, StageContext
from twin_pipeline.stages import report as report_stage
from twin_pipeline.stages import smoke

mujoco = pytest.importorskip("mujoco")

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "arm_demo" / "twin.yaml"
URDF = EXAMPLE.parent / "urdf" / "arm_demo.urdf"
SOFT = {"pitch_joint": (50, 2), "elbow_joint": (50, 2), "finger_joint": (200, 5)}  # the demo twin.yaml's actuators
STIFF = {"yaw_joint": (400, 20), "pitch_joint": (400, 20), "elbow_joint": (400, 20), "finger_joint": (2000, 40)}


def demo_spec(actuators=SOFT, *, elbow=(-2.0, 2.0), edit=None):
    """A MuJoCo model spec of the demo arm, as the mjcf stage would build it."""
    spec = mujoco.MjSpec.from_file(str(URDF))
    if elbow is not None:
        spec.joint("elbow_joint").range = list(elbow)
    for joint, (kp, kv) in actuators.items():
        actuator = spec.add_actuator()
        actuator.name, actuator.target, actuator.trntype = joint.replace("_joint", "_act"), joint, mujoco.mjtTrn.mjTRN_JOINT
        actuator.set_to_position(kp=kp, kv=kv)
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # actuator damping stays stable at high kv
    if edit is not None:
        edit(spec)
    return spec


def fake_mjcf(xml: str | None = None, *, spec=None, status="succeeded"):
    """A stand-in mjcf stage that writes the given MJCF and passes it on as output 'mjcf'."""
    def run(context):
        if status == "failed":
            return StageResult.failed("mjcf", "MESH_TOO_LARGE", "expected at most 200000 faces, found 250000 in base.stl")
        path = context.stage_dir / "model.xml"
        path.write_text(xml if xml is not None else (spec or demo_spec()).to_xml(), encoding="utf-8")
        return StageResult.succeeded("mjcf", outputs={"mjcf": context.rel(path)}, metrics={"nbody": 6})
    return Stage("mjcf", run)


def write_twin(tmp_path, change=None) -> Path:
    """A copy of the demo twin.yaml in tmp_path (URDF path made absolute), optionally edited."""
    data = yaml.safe_load(EXAMPLE.read_text(encoding="utf-8"))
    data["urdf"] = str(URDF)
    if change is not None:
        change(data)
    path = tmp_path / "twin" / "twin.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def run_smoke(tmp_path, twin=EXAMPLE, **mjcf):
    spec = load_spec(twin)
    summary = run_pipeline(spec, runs_dir=tmp_path / "runs", stages=[fake_mjcf(**mjcf), smoke.STAGES[0]])
    run_dir = Path(summary["run_dir"])
    result = json.loads((run_dir / "smoke" / "result.json").read_text(encoding="utf-8"))
    data = json.loads((run_dir / "smoke" / "smoke.json").read_text(encoding="utf-8")) if (run_dir / "smoke" / "smoke.json").exists() else None
    return summary, result, data


def codes(result):
    return [error["code"] for error in result["errors"]]


def test_stiff_demo_passes_every_check(tmp_path):
    summary, result, data = run_smoke(tmp_path, spec=demo_spec(STIFF))
    assert result["status"] == "succeeded", result
    assert data["checks"] == {"load": "passed", "model": "passed", "home": "passed", "sweep": "passed", "settle": "passed"}
    assert result["metrics"]["sweep_reached"] == result["metrics"]["sweep_targets"] == 8
    assert result["outputs"] == {"smoke": "smoke/smoke.json"}
    elbow = data["joints"]["elbow_joint"]
    assert elbow["effective_limits"] == [-2.0, 2.0] and elbow["mode"] == "position (elbow_act)"
    assert [entry["target"] for entry in elbow["sweep"]] == [-1.8, 1.8]
    assert data["home"]["joints"]["pitch_joint"] == {"value": 0.3, "from": "twin.yaml home", "inside_limits": True}
    assert data["joints"]["yaw_joint"]["effective_limits"] is None  # continuous: swept to +-0.9 pi
    assert data["joints"]["yaw_joint"]["sweep"][1]["target"] == pytest.approx(0.9 * 3.141593, abs=1e-5)
    assert all(joint["at_rest"] for joint in data["settle"]["joints"].values())
    assert summary["status"] == "succeeded"


def test_soft_actuator_under_gravity_needs_review_and_unactuated_joint_is_named(tmp_path):
    summary, result, data = run_smoke(tmp_path, spec=demo_spec(SOFT))
    assert result["status"] == "needs_review" and summary["status"] == "needs_review"
    items = [row["item"] for row in result["review"]]
    assert any(item.startswith("pitch_joint sweep") for item in items), items
    reason = next(row["reason"] for row in result["review"] if row["item"].startswith("pitch_joint"))
    assert "tolerance 0.05" in reason and "blocked by design" in reason
    assert any("yaw_joint has no actuator: swept by setting qpos directly" in w for w in result["warnings"])
    assert data["joints"]["yaw_joint"]["mode"] == "qpos (no actuator)"


def test_mjcf_without_the_limit_override_fails(tmp_path):
    _, result, data = run_smoke(tmp_path, spec=demo_spec(STIFF, elbow=None))
    assert result["status"] == "failed" and codes(result) == ["LIMITS_MISMATCH"]
    assert "elbow_joint: expected range [-2.0, 2.0]" in result["errors"][0]["message"]
    assert data["checks"]["model"] == "failed" and data["checks"]["sweep"] == "not run"


def test_missing_joint_and_wrong_gravity_fail(tmp_path):
    def edit(spec):
        spec.joint("finger_joint").name = "gripper"
        spec.option.gravity = [0, 0, 9.81]
    actuators = {k: v for k, v in STIFF.items() if k != "finger_joint"}
    _, result, _ = run_smoke(tmp_path, spec=demo_spec(actuators, edit=edit))
    assert set(codes(result)) == {"MISSING_JOINT", "GRAVITY_MISMATCH"}
    assert any("gripper is in the MJCF but not in the URDF" in w for w in result["warnings"])


def test_gravity_follows_up_axis(tmp_path):
    def edit(spec):
        spec.option.gravity = [0, -9.81, 0]
    twin = write_twin(tmp_path, lambda data: data.update(up_axis="+y"))
    _, result, data = run_smoke(tmp_path, twin=twin, spec=demo_spec(STIFF, edit=edit))
    assert "GRAVITY_MISMATCH" not in codes(result)
    assert data["gravity"]["up_axis"] == "+y" and data["gravity"]["alignment"] == 1.0


def test_mjcf_that_does_not_load_fails(tmp_path):
    _, result, data = run_smoke(tmp_path, xml="<mujoco><worldbody><body><joint type='nope'/></body></worldbody></mujoco>")
    assert result["status"] == "failed" and codes(result) == ["MJCF_LOAD_FAILED"]
    assert data["checks"]["load"] == "failed"


def test_blow_up_is_a_failure(tmp_path):
    twin = write_twin(tmp_path, lambda data: data["smoke"].update(timestep=0.05))
    huge = {joint: (1e7, 0) for joint in STIFF}
    _, result, data = run_smoke(tmp_path, twin=twin, spec=demo_spec(huge))
    assert result["status"] == "failed" and codes(result)[0] == "SIMULATION_UNSTABLE", result
    assert data["model"]["timestep"] == 0.05 and data["model"]["timestep_from"] == "twin.yaml smoke.timestep"


def test_zero_home_outside_limits_and_contacts_at_home_need_review(tmp_path):
    def change(data):
        data["joints"]["finger_joint"].update(lower=0.01, upper=0.03)

    def edit(spec):
        spec.joint("finger_joint").range = [0.01, 0.03]
        obstacle = spec.worldbody.add_geom()
        obstacle.name, obstacle.type, obstacle.size, obstacle.pos = "obstacle", mujoco.mjtGeom.mjGEOM_BOX, [0.05, 0.05, 0.05], [0, 0, 0.3]

    twin = write_twin(tmp_path, change)
    _, result, data = run_smoke(tmp_path, twin=twin, spec=demo_spec(STIFF, edit=edit))
    assert result["status"] == "needs_review"
    items = {row["item"]: row["reason"] for row in result["review"]}
    assert "add home.finger_joint" in items["home pose of finger_joint"].lower()
    assert "contacts at the home pose" in items
    assert data["home"]["joints"]["finger_joint"]["used"] == 0.01
    assert data["home"]["contacts"]


def test_home_given_outside_mjcf_limits_fails(tmp_path):
    twin = write_twin(tmp_path, lambda data: data["home"].update(finger_joint=0.0))

    def edit(spec):
        spec.joint("finger_joint").range = [0.01, 0.03]
    # twin.yaml keeps the URDF range [0, 0.03], so the MJCF range disagrees first
    _, result, _ = run_smoke(tmp_path, twin=twin, spec=demo_spec(STIFF, edit=edit))
    assert codes(result) == ["LIMITS_MISMATCH"]


def test_skipped_without_mjcf(tmp_path):
    spec = load_spec(EXAMPLE)
    context = StageContext(spec, tmp_path, tmp_path / "smoke", {})
    result = smoke.run(context)
    assert result.status == "skipped" and result.applicable is True and "no MJCF" in result.reason


def test_works_without_a_urdf(tmp_path):
    """Any MJCF: with no URDF the MJCF ranges are the limits and the joints are still swept."""
    spec = load_spec(EXAMPLE)
    run_dir = tmp_path / "run"
    (run_dir / "mjcf").mkdir(parents=True)
    (run_dir / "mjcf" / "model.xml").write_text(demo_spec(STIFF).to_xml(), encoding="utf-8")
    upstream = StageResult.succeeded("mjcf", outputs={"mjcf": "mjcf/model.xml"})
    upstream.passed = True
    context = StageContext(spec, run_dir, run_dir / "smoke", {"mjcf": upstream})
    context.stage_dir.mkdir()
    object.__setattr__(spec, "data", {**spec.data, "urdf": "missing.urdf"})
    result = smoke.run(context)
    assert result.status == "succeeded", result
    assert any("no URDF" in w for w in result.warnings)


def test_validator():
    assert smoke.validate_smoke("smoke", {"sweep_fraction": 0.5, "sweep_seconds": 1, "angle_tolerance": 0.1, "distance_tolerance": 0.01}) == []
    errors = smoke.validate_smoke("smoke", {"sweep_fraction": 1.5, "angle_tolerance": 0, "sweeps": 3})
    assert any("sweeps" in e for e in errors) and any("sweep_fraction" in e for e in errors) and any("angle_tolerance" in e for e in errors)


def test_full_pipeline_with_fake_mjcf_writes_a_report(tmp_path):
    spec = load_spec(EXAMPLE)
    stages = [fake_mjcf(spec=demo_spec(STIFF)), smoke.STAGES[0], report_stage.STAGES[0]]
    summary = run_pipeline(spec, runs_dir=tmp_path, stages=stages)
    assert summary["status"] == "succeeded", summary
    page = Path(summary["run_dir"]) / "report" / "report.html"
    assert page.exists() and "elbow_joint" in page.read_text(encoding="utf-8")


def test_oscillating_controller_needs_review(tmp_path):
    """High kv on light links is unstable with explicit Euler: the joints chatter instead of settling."""
    def edit(spec):
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_EULER
    _, result, data = run_smoke(tmp_path, spec=demo_spec(STIFF, edit=edit))
    assert result["status"] == "needs_review"
    assert any(row["item"] == "settling" for row in result["review"])
    assert data["checks"]["settle"] == "review"


@pytest.mark.parametrize("kind", ["motor", "velocity"])
def test_motor_and_velocity_actuators_are_driven(tmp_path, kind):
    def edit(spec):
        for joint in STIFF:
            actuator = spec.add_actuator()
            actuator.name, actuator.target, actuator.trntype = f"{joint}_{kind}", joint, mujoco.mjtTrn.mjTRN_JOINT
            actuator.set_to_motor() if kind == "motor" else actuator.set_to_velocity(kv=200)
    _, result, data = run_smoke(tmp_path, spec=demo_spec({}, edit=edit))
    assert data["joints"]["elbow_joint"]["mode"] == f"{kind} (elbow_joint_{kind})"
    assert result["metrics"]["sweep_reached"] == result["metrics"]["sweep_targets"], result["review"]
