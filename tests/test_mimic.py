"""URDF <mimic> joints: parsing, twin.yaml checks, the MuJoCo equality constraint and the smoke checks."""

import json
from pathlib import Path

import pytest
import yaml

from twin_pipeline.pipeline import run_pipeline
from twin_pipeline.result import read_result
from twin_pipeline.spec import SpecError, load_spec
from twin_pipeline.stages import all_stages
from twin_pipeline.urdf import mimic_problems, read_urdf

mujoco = pytest.importorskip("mujoco")

DEMO = Path(__file__).resolve().parents[1] / "examples" / "arm_demo"
SIM_STAGES = [stage for stage in all_stages() if stage.name in ("mjcf", "smoke")]

SECOND_FINGER = """
  <joint name="finger2_joint" type="prismatic">
    <parent link="forearm_link"/>
    <child link="finger2_link"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 -1 0"/>
    <limit lower="{lower}" upper="{upper}" effort="5" velocity="0.1"/>
    <mimic joint="{leader}" multiplier="{multiplier}" offset="{offset}"/>
  </joint>
  <link name="finger2_link">
    <inertial>
      <origin xyz="0 0 0.025"/>
      <mass value="0.05"/>
      <inertia ixx="0.00001083" ixy="0" ixz="0" iyy="0.00001208" iyz="0" izz="0.00000208"/>
    </inertial>
    <visual><origin xyz="0 0 0.025"/><geometry><box size="0.02 0.01 0.05"/></geometry></visual>
  </link>
</robot>"""


def make_twin(tmp_path, *, lower=0.0, upper=0.03, leader="finger_joint", multiplier=1.0, offset=0.0, edit=None):
    """The demo arm with a second finger that mimics the first (the usual symmetric gripper)."""
    folder = tmp_path / "twin"
    (folder / "urdf").mkdir(parents=True)
    urdf = (DEMO / "urdf" / "arm_demo.urdf").read_text(encoding="utf-8").replace(
        "</robot>", SECOND_FINGER.format(lower=lower, upper=upper, leader=leader, multiplier=multiplier, offset=offset))
    (folder / "urdf" / "arm_demo.urdf").write_text(urdf, encoding="utf-8")
    data = yaml.safe_load((DEMO / "twin.yaml").read_text(encoding="utf-8"))
    data["home"]["finger_joint"] = 0.02
    if edit:
        edit(data)
    (folder / "twin.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return folder / "twin.yaml"


def run(path, tmp_path):
    summary = run_pipeline(load_spec(path), runs_dir=tmp_path / "runs", stages=SIM_STAGES)
    return summary, {name: read_result(Path(summary["run_dir"]) / name / "result.json") for name in ("mjcf", "smoke")}


def test_parse_and_problems(tmp_path):
    urdf = read_urdf(make_twin(tmp_path, multiplier=2.0, offset=0.001).parent / "urdf" / "arm_demo.urdf")
    mimic = urdf.joints["finger2_joint"].mimic
    assert (mimic.leader, mimic.multiplier, mimic.offset) == ("finger_joint", 2.0, 0.001)
    assert urdf.joints["finger_joint"].mimic is None and mimic_problems(urdf) == []
    for leader, expected in (("nope_joint", "no joint 'nope_joint'"), ("tool_joint", "the leader is a fixed joint"),
                             ("finger2_joint", "cannot follow itself")):
        bad = read_urdf(make_twin(tmp_path / leader, leader=leader).parent / "urdf" / "arm_demo.urdf")
        assert any(expected in p for p in mimic_problems(bad)), (leader, mimic_problems(bad))


def test_twin_yaml_may_not_pose_or_actuate_a_follower(tmp_path):
    def edit(data):
        data["home"]["finger2_joint"] = 0.01
        data["actuators"]["joints"]["finger2_joint"] = {"type": "position", "kp": 10.0}
    with pytest.raises(SpecError) as caught:
        load_spec(make_twin(tmp_path, edit=edit))
    errors = " | ".join(caught.value.errors)
    assert "home.finger2_joint: the joint follows finger_joint" in errors and "set home.finger_joint instead" in errors
    assert "actuators.joints.finger2_joint: the joint follows finger_joint" in errors


def test_equality_constraint_home_and_no_actuator(tmp_path):
    summary, results = run(make_twin(tmp_path, multiplier=1.0, offset=0.0), tmp_path)
    mjcf = results["mjcf"]
    assert mjcf.status == "succeeded", mjcf.errors
    assert mjcf.metrics["neq"] == 1 and mjcf.metrics["mimic"] == {"finger2_joint": {"leader": "finger_joint", "multiplier": 1.0, "offset": 0.0}}
    assert "finger2_joint" not in mjcf.metrics["actuators"] and "finger_joint" in mjcf.metrics["actuators"]
    assert any("no actuator for the mimic joints finger2_joint" in w for w in mjcf.warnings)
    model = mujoco.MjModel.from_xml_path(str(Path(summary["run_dir"]) / mjcf.outputs["mjcf"]))
    assert model.eq(0).name == "mimic_finger2_joint" and list(model.eq_data[0][:2]) == [0.0, 1.0]
    key = model.key("home")
    assert key.qpos[model.joint("finger2_joint").qposadr[0]] == pytest.approx(0.02)  # follows the leader's home
    smoke = results["smoke"]
    assert smoke.status == "succeeded", (smoke.errors, smoke.review)
    assert smoke.metrics["mimic_checks"] == 2 and smoke.metrics["mimic_followed"] == 2
    data = json.loads((Path(summary["run_dir"]) / smoke.outputs["smoke"]).read_text(encoding="utf-8"))
    assert data["joints"]["finger2_joint"]["mode"].startswith("follows finger_joint")
    assert data["home"]["joints"]["finger2_joint"]["from"] == "follows finger_joint"
    assert all(entry["followers"]["finger2_joint"]["within_tolerance"] for entry in data["joints"]["finger_joint"]["sweep"])


def test_multiplier_offset_and_zero_offsets(tmp_path):
    """follower = offset + multiplier * leader in URDF coordinates, whatever zero_offset does to either joint."""
    def edit(data):
        data["joints"]["finger_joint"]["zero_offset"] = 0.005
        data["joints"]["finger2_joint"] = {"zero_offset": -0.002, "source": "assumed"}
        data["home"]["finger_joint"] = 0.01  # twin coordinate: URDF 0.015
    summary, results = run(make_twin(tmp_path, upper=0.07, multiplier=2.0, offset=0.004, edit=edit), tmp_path)
    assert results["mjcf"].status == "succeeded", results["mjcf"].errors
    model = mujoco.MjModel.from_xml_path(str(Path(summary["run_dir"]) / results["mjcf"].outputs["mjcf"]))
    home = model.key("home").qpos
    follower = home[model.joint("finger2_joint").qposadr[0]]
    assert follower == pytest.approx(0.004 + 2.0 * (0.01 + 0.005) - (-0.002))  # twin coordinate of URDF 0.034
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    assert abs(data.efc_pos[data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY]).max() < 1e-9  # the keyframe satisfies the constraint
    assert results["smoke"].status == "succeeded", (results["smoke"].errors, results["smoke"].review)
    assert results["smoke"].metrics["mimic_followed"] == results["smoke"].metrics["mimic_checks"] == 2


def test_follower_limits_narrower_than_the_leader_needs(tmp_path):
    summary, results = run(make_twin(tmp_path, upper=0.02), tmp_path)
    assert any("mimic finger2_joint: over finger_joint's range" in w for w in results["mjcf"].warnings)
    smoke = results["smoke"]
    assert smoke.status == "needs_review"
    assert any(item["item"].startswith("finger2_joint follows finger_joint") and "outside its effective limits" in item["reason"] for item in smoke.review)


def test_invalid_mimic_is_refused_with_the_spec(tmp_path):
    # the cross-check against the URDF refuses it before any stage runs (the builder raises MIMIC_INVALID as a backstop)
    with pytest.raises(SpecError, match="the leader is a fixed joint"):
        load_spec(make_twin(tmp_path, leader="tool_joint"))
