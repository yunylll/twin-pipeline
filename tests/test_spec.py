import copy
from pathlib import Path

import pytest
import yaml

from twin_pipeline.spec import SpecError, check_against_urdf, load_spec, validate_data
from twin_pipeline.urdf import read_urdf

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "arm_demo" / "twin.yaml"


@pytest.fixture
def data():
    return yaml.safe_load(EXAMPLE.read_text(encoding="utf-8"))


def write(tmp_path, data):
    folder = tmp_path / "twin"
    (folder / "urdf").mkdir(parents=True)
    (folder / "urdf" / "arm_demo.urdf").write_bytes((EXAMPLE.parent / "urdf" / "arm_demo.urdf").read_bytes())
    path = folder / "twin.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def errors_of(tmp_path, data):
    with pytest.raises(SpecError) as caught:
        load_spec(write(tmp_path, data))
    return caught.value.errors


def test_example_is_valid():
    spec = load_spec(EXAMPLE)
    assert spec.robot == "arm_demo"
    assert spec.urdf_path.exists()
    assert spec.source_of("elbow_joint", "lower") == "assumed"
    assert spec.source_of("elbow_joint", "effort") is None
    assert spec.limits("elbow_joint", read_urdf(spec.urdf_path)) == (-2.0, 2.0)
    assert spec.limits("pitch_joint", read_urdf(spec.urdf_path)) == (-1.57, 1.57)


def test_unknown_top_level_key(data):
    data["colision"] = {}
    assert any("unknown key(s) 'colision'" in e for e in validate_data(data))


def test_schema_version_and_robot_name(data):
    data["schema_version"] = 2
    data["robot"] = "2 arms"
    errors = validate_data(data)
    assert any(e.startswith("schema_version: expected 1") for e in errors)
    assert any(e.startswith("robot:") for e in errors)


def test_urdf_or_cad_required(data):
    del data["urdf"]
    assert any(e.startswith("urdf: required") for e in validate_data(data))
    data["cad"] = {"model": "C:/robot.SLDASM"}
    assert validate_data(data) == []


def test_joint_values_need_provenance(data):
    del data["joints"]["elbow_joint"]["source"]
    errors = validate_data(data)
    assert any("joints.elbow_joint: lower, upper need a provenance" in e for e in errors)
    data["joints"]["elbow_joint"]["sources"] = {"lower": "measured", "upper": "cad"}
    assert validate_data(data) == []


def test_bad_joint_values(data):
    data["joints"]["elbow_joint"].update(lower=1.0, upper=0.0, source="guessed")
    data["joints"]["finger_joint"]["effort"] = -1
    errors = validate_data(data)
    assert any("lower 1.0 is above upper 0.0" in e for e in errors)
    assert any(e.startswith("joints.elbow_joint.source: expected one of cad") for e in errors)
    assert any(e.startswith("joints.finger_joint.effort: expected a number > 0") for e in errors)


def test_stage_sections_are_validated(data):
    data["meshes"]["max_faces"] = 0
    data["collision"]["default"] = "hull"
    data["smoke"]["sweep_fraction"] = 1.5
    errors = validate_data(data)
    assert any(e.startswith("meshes.max_faces") for e in errors)
    assert any(e.startswith("collision.default") for e in errors)
    assert any(e.startswith("smoke.sweep_fraction") for e in errors)


def test_urdf_cross_check(tmp_path, data):
    data["joints"]["elbo_joint"] = {"upper": 1.0, "source": "assumed"}
    data["joints"]["yaw_joint"] = {"lower": -1.0, "source": "assumed"}
    data["home"]["elbow_joint"] = 2.2
    data["home"]["tool_joint"] = 0.0
    data["actuators"]["joints"]["tool_joint"] = {"type": "position", "kp": 1.0}
    errors = errors_of(tmp_path, data)
    assert any("joints.elbo_joint: no joint 'elbo_joint' in the URDF; did you mean elbow_joint" in e for e in errors)
    assert any("joints.yaw_joint: the URDF joint is continuous" in e for e in errors)
    assert any("home.elbow_joint: 2.2 is outside the effective limits [-2.0, 2.0]" in e for e in errors)
    assert any("home.tool_joint: the URDF joint is fixed" in e for e in errors)
    assert any("actuators.joints.tool_joint" in e for e in errors)


def test_missing_urdf_is_not_cross_checked_yet(tmp_path, data):
    data["urdf"] = "urdf/not_exported_yet.urdf"
    spec = load_spec(write(tmp_path, data))
    assert not spec.urdf_path.exists()


def test_zero_offset_shifts_urdf_limits_only(data):
    from twin_pipeline.spec import TwinSpec
    data["joints"]["pitch_joint"] = {"zero_offset": 0.1, "source": "assumed"}
    data["joints"]["elbow_joint"]["zero_offset"] = 0.5
    spec = TwinSpec(EXAMPLE, data)
    urdf = read_urdf(EXAMPLE.parent / "urdf" / "arm_demo.urdf")
    lower, upper = spec.limits("pitch_joint", urdf)  # URDF [-1.57, 1.57] in the twin's coordinate
    assert lower == pytest.approx(-1.67) and upper == pytest.approx(1.47)
    assert spec.limits("elbow_joint", urdf) == (-2.0, 2.0)  # twin.yaml limits are already in the twin's coordinate
    data["home"]["pitch_joint"] = 1.5
    assert any("home.pitch_joint: 1.5 is outside the effective limits" in e for e in check_against_urdf(spec, urdf))


def test_override_merged_with_urdf_limits(data):
    spec_data = copy.deepcopy(data)
    spec_data["joints"]["pitch_joint"] = {"lower": 2.0, "source": "assumed"}
    from twin_pipeline.spec import TwinSpec
    spec = TwinSpec(EXAMPLE, spec_data)
    errors = check_against_urdf(spec, read_urdf(EXAMPLE.parent / "urdf" / "arm_demo.urdf"))
    assert any("effective lower 2.0 is above effective upper 1.57" in e for e in errors)
