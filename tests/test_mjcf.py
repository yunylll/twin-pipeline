import copy
import math
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip("mujoco")

from twin_pipeline.mjcf import BuildError  # noqa: E402
from twin_pipeline.mjcf.build import build_mjcf  # noqa: E402
from twin_pipeline.mjcf import collision as hulls  # noqa: E402
from twin_pipeline.mjcf import stl  # noqa: E402
from twin_pipeline.mjcf.geometry import rpy_to_quat  # noqa: E402
from twin_pipeline.pipeline import run_pipeline  # noqa: E402
from twin_pipeline.result import read_result  # noqa: E402
from twin_pipeline.spec import load_spec, validate_data  # noqa: E402
from twin_pipeline.stages import mjcf as mjcf_stage  # noqa: E402

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
ARM_DEMO = EXAMPLES / "arm_demo" / "twin.yaml"
MESH_ARM = EXAMPLES / "mesh_arm" / "twin.yaml"
MESHES = EXAMPLES / "mesh_arm" / "mesh_arm_description" / "meshes"


def id_of(model, kind, name):
    index = mujoco.mj_name2id(model, kind, name)
    assert index >= 0, f"no {kind} named {name}"
    return index


def joint(model, name):
    return model.joint(id_of(model, mujoco.mjtObj.mjOBJ_JOINT, name))


def actuator(model, name):
    return model.actuator(id_of(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))


def geoms_of(model, body):
    index = id_of(model, mujoco.mjtObj.mjOBJ_BODY, body)
    return [g for g in range(model.ngeom) if model.geom_bodyid[g] == index]


def copy_example(tmp_path, example=MESH_ARM, edit=None, folder="twin"):
    """A copy of an example twin in tmp_path, with twin.yaml changed by edit(data)."""
    target = tmp_path / folder
    shutil.copytree(example.parent, target, ignore=shutil.ignore_patterns("runs"))
    data = yaml.safe_load(example.read_text(encoding="utf-8"))
    if edit:
        edit(data)
    path = target / "twin.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def build(tmp_path, spec_path):
    spec = load_spec(spec_path)
    return build_mjcf(spec, spec.urdf_path, tmp_path / "out")


def failure(tmp_path, spec_path) -> BuildError:
    with pytest.raises(BuildError) as caught:
        build(tmp_path, spec_path)
    return caught.value


# --- the primitive demo arm through the pipeline -------------------------------------------------

def test_arm_demo_stage_succeeds(tmp_path):
    summary = run_pipeline(load_spec(ARM_DEMO), runs_dir=tmp_path, stages=mjcf_stage.STAGES)
    run_dir = Path(summary["run_dir"])
    result = read_result(run_dir / "mjcf" / "result.json")
    assert result.status == "succeeded", result.errors
    assert result.outputs == {"mjcf": "mjcf/model.xml", "assets": "mjcf/assets"}
    model = mujoco.MjModel.from_xml_path(str(run_dir / result.outputs["mjcf"]))
    assert (model.njnt, model.nu, model.nkey) == (4, 4, 1)
    assert result.metrics["nbody"] == model.nbody == 7  # world + 6 links: the fixed tool_frame is kept
    assert result.metrics["fixed_frames"] == ["tool_frame"]
    assert id_of(model, mujoco.mjtObj.mjOBJ_SITE, "tool_frame") >= 0
    tool = id_of(model, mujoco.mjtObj.mjOBJ_BODY, "tool_frame")
    assert model.body_parentid[tool] == id_of(model, mujoco.mjtObj.mjOBJ_BODY, "forearm_link")
    assert np.allclose(model.body_pos[tool], [0, 0, 0.3])
    assert np.allclose(model.opt.gravity, [0, 0, -9.81])


def test_arm_demo_joints_actuators_and_home(tmp_path):
    model = mujoco.MjModel.from_xml_path(str(build(tmp_path, ARM_DEMO).mjcf))
    assert np.allclose(joint(model, "elbow_joint").range, [-2.0, 2.0])     # twin.yaml override
    assert np.allclose(joint(model, "pitch_joint").range, [-1.57, 1.57])   # URDF
    assert not model.jnt_limited[id_of(model, mujoco.mjtObj.mjOBJ_JOINT, "yaw_joint")]
    assert np.allclose(model.jnt_actfrcrange[id_of(model, mujoco.mjtObj.mjOBJ_JOINT, "finger_joint")], [-3, 3])
    assert np.allclose(model.jnt_actfrcrange[id_of(model, mujoco.mjtObj.mjOBJ_JOINT, "pitch_joint")], [-15, 15])
    finger, pitch = actuator(model, "finger_joint"), actuator(model, "pitch_joint")
    assert finger.gainprm[0] == 200.0 and finger.biasprm[2] == -5.0
    assert pitch.gainprm[0] == 50.0 and pitch.biasprm[2] == -2.0
    assert np.allclose(actuator(model, "elbow_joint").ctrlrange, [-2.0, 2.0])
    assert not model.actuator_ctrllimited[id_of(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "yaw_joint")]
    key = model.key("home")
    assert np.allclose(key.qpos, [0, 0.3, -0.6, 0]) and np.allclose(key.ctrl, [0, 0.3, -0.6, 0])
    # collision default none: every geom is visual and never collides
    assert model.ngeom == 5 and not model.geom_contype.any() and not model.geom_conaffinity.any()
    assert set(model.geom_group) == {2}


def test_saved_model_moves_with_its_assets(tmp_path):
    result = build(tmp_path, MESH_ARM)
    moved = tmp_path / "elsewhere"
    shutil.copytree(result.mjcf.parent, moved)
    shutil.rmtree(result.mjcf.parent)
    model = mujoco.MjModel.from_xml_path(str(moved / "model.xml"))
    assert model.nmesh == 6


# --- the mesh example ----------------------------------------------------------------------------

@pytest.fixture(scope="module")
def mesh_arm(tmp_path_factory):
    out = tmp_path_factory.mktemp("mesh_arm")
    result = build(out, MESH_ARM)
    return result, mujoco.MjModel.from_xml_path(str(result.mjcf))


def test_mesh_arm_meshes(mesh_arm):
    result, model = mesh_arm
    assets = result.assets
    assert sorted(p.name for p in assets.iterdir()) == [
        "base_part0.stl", "base_part1.stl", "forearm.obj", "forearm_link_hull.stl", "upper_arm.stl", "upper_arm_link_hull.stl"]
    metrics = result.metrics
    assert (metrics["mesh_files"], metrics["meshes_split"], metrics["mesh_parts"], metrics["meshes_converted"]) == (3, 1, 2, 1)
    # the split is lossless: the parts' faces, in order, are the original faces byte for byte
    original, _ = stl.read_stl(MESHES / "base.stl")
    parts = [stl.read_stl(assets / f"base_part{k}.stl") for k in range(2)]
    assert [kind for _, kind in parts] == ["binary", "binary"] and [len(p) for p, _ in parts] == [500, 172]
    assert np.concatenate([p for p, _ in parts]).tobytes() == original.tobytes()
    # the ASCII STL became a binary STL with the same triangles
    ascii_faces, kind = stl.read_stl(MESHES / "upper_arm.stl")
    converted, converted_kind = stl.read_stl(assets / "upper_arm.stl")
    assert (kind, converted_kind) == ("ascii", "binary")
    assert np.array_equal(ascii_faces["vertices"], converted["vertices"])
    # the relative-path OBJ is copied as it is
    assert (assets / "forearm.obj").read_bytes() == (MESHES / "forearm.obj").read_bytes()
    # both base parts are visual geoms of base_link
    base = [model.geom(g).name for g in geoms_of(model, "base_link")]
    assert base == ["base_link_visual0_part0", "base_link_visual0_part1"]
    assert metrics["nmeshface"] == model.nmeshface


def test_mesh_arm_gravity_collision_and_frames(mesh_arm):
    result, model = mesh_arm
    assert np.allclose(model.opt.gravity, [0, -9.81, 0])
    assert result.metrics["collision_links"] == {"none": 1, "convex_hull": 3, "mesh": 1}
    collides = lambda body: [model.geom(g).name for g in geoms_of(model, body) if model.geom_contype[g]]
    assert collides("base_link") == []
    assert collides("upper_arm_link") == ["upper_arm_link_hull"]
    assert collides("hand_link") == ["hand_link_collision0"]
    assert model.geom_type[id_of(model, mujoco.mjtObj.mjOBJ_GEOM, "hand_link_collision0")] == mujoco.mjtGeom.mjGEOM_SPHERE
    # convex_hull keeps the URDF primitive of finger_link and has no mesh to hull there
    assert collides("finger_link") == ["finger_link_collision0"]
    for g in range(model.ngeom):
        assert model.geom_group[g] == (3 if model.geom_contype[g] else 2)
    assert id_of(model, mujoco.mjtObj.mjOBJ_SITE, "tool_frame") >= 0


def test_convex_hull_contains_the_mesh(mesh_arm):
    result, _ = mesh_arm
    hull, _ = stl.read_stl(result.assets / "upper_arm_link_hull.stl")
    source, _ = stl.read_stl(MESHES / "upper_arm.stl")
    points = source["vertices"].reshape(-1, 3)
    corners = hull["vertices"].astype(np.float64)
    normal = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    distance = np.einsum("fj,pj->pf", normal, points) - np.einsum("fj,fj->f", normal, corners[:, 0])
    assert distance.max() < 1e-6  # every source vertex is inside or on every outward hull face
    assert np.allclose(corners.reshape(-1, 3).min(0), points.min(0), atol=1e-6)
    assert np.allclose(corners.reshape(-1, 3).max(0), points.max(0), atol=1e-6)


def test_mesh_arm_joint_overrides_and_actuators(mesh_arm):
    _, model = mesh_arm
    shoulder = joint(model, "shoulder_joint")
    assert np.isclose(model.qpos0[shoulder.qposadr[0]], -0.1)       # zero_offset 0.1: CAD pose reads -0.1
    assert np.allclose(shoulder.range, [-2.1, 1.9])                 # URDF limits shifted into the twin's coordinate
    assert np.allclose(actuator(model, "shoulder_joint").ctrlrange, [-2.1, 1.9])
    elbow = id_of(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow_joint")
    assert np.isclose(model.dof_damping[model.jnt_dofadr[elbow]], 0.05)
    assert np.isclose(model.dof_frictionloss[model.jnt_dofadr[elbow]], 0.01)
    wrist = actuator(model, "wrist_joint")
    assert model.actuator_biastype[wrist.id] == mujoco.mjtBias.mjBIAS_AFFINE and wrist.biasprm[2] == -0.5
    assert np.allclose(wrist.ctrlrange, [-3.0, 3.0])               # twin.yaml velocity, not the URDF's 4.0
    gripper = actuator(model, "gripper_joint")
    assert model.actuator_biastype[gripper.id] == mujoco.mjtBias.mjBIAS_NONE
    assert np.allclose(gripper.ctrlrange, [-2.0, 2.0])             # twin.yaml effort
    key = model.key("home")
    assert np.allclose(key.qpos, [0.2, 0.5, 0, 0]) and np.allclose(key.ctrl, [0.2, 0.5, 0, 0])


def test_geometry_frames_match_the_importer():
    """Visual geoms are placed with rpy_to_quat; it must agree with how MuJoCo's URDF importer reads rpy."""
    urdf = """<robot name="r"><link name="a"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
    <link name="b"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
    <joint name="j" type="fixed"><parent link="a"/><child link="b"/><origin rpy="0.3 -0.7 1.1"/></joint></robot>"""
    spec = mujoco.MjSpec.from_string(urdf.replace('<robot name="r">', '<robot name="r"><mujoco><compiler fusestatic="false"/></mujoco>'))
    model = spec.compile()
    expected = model.body_quat[id_of(model, mujoco.mjtObj.mjOBJ_BODY, "b")]
    ours = np.array(rpy_to_quat(0.3, -0.7, 1.1))
    assert np.allclose(ours, expected, atol=1e-6) or np.allclose(ours, -expected, atol=1e-6)


# --- other collision modes, actuator choices and package paths ---------------------------------

def test_collision_mesh_mode_on_a_split_mesh_warns(tmp_path):
    def edit(data):
        data["collision"] = {"default": "mesh"}
    result = build(tmp_path, copy_example(tmp_path, edit=edit))
    model = mujoco.MjModel.from_xml_path(str(result.mjcf))
    names = [model.geom(g).name for g in geoms_of(model, "base_link") if model.geom_contype[g]]
    assert names == ["base_link_collision0_part0", "base_link_collision0_part1"]
    assert any("split" in warning and "base_link" in warning for warning in result.warnings)


def test_actuator_none_and_missing_entries_add_nothing(tmp_path):
    def edit(data):
        data["actuators"] = {"joints": {"elbow_joint": {"type": "position", "kp": 5.0}, "wrist_joint": {"type": "none"}}}
    result = build(tmp_path, copy_example(tmp_path, edit=edit))
    model = mujoco.MjModel.from_xml_path(str(result.mjcf))
    assert model.nu == 1 and result.metrics["actuators"] == {"elbow_joint": "position"}
    assert actuator(model, "elbow_joint").biasprm[2] == 0.0  # no kv given: no damping term invented


def test_no_home_means_no_keyframe(tmp_path):
    result = build(tmp_path, copy_example(tmp_path, edit=lambda data: data.pop("home")))
    assert mujoco.MjModel.from_xml_path(str(result.mjcf)).nkey == 0


def test_package_folder_from_twin_yaml(tmp_path):
    path = copy_example(tmp_path, edit=lambda data: data["meshes"].update(packages={"mesh_arm_description": "renamed_pkg"}))
    (path.parent / "mesh_arm_description").rename(path.parent / "renamed_pkg")
    data = yaml.safe_load(path.read_text())
    data["urdf"] = "renamed_pkg/urdf/mesh_arm.urdf"
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    assert build(tmp_path, path).metrics["mesh_files"] == 3


def test_stl_over_mujocos_limit_is_split(tmp_path):
    """A generated surface just over MuJoCo's 200 000-face STL limit, with the default max_faces."""
    n = 318
    x, z = np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, n))
    vertices = np.stack([x.ravel(), 0.05 * np.sin(6 * x.ravel()) * np.cos(6 * z.ravel()), z.ravel()], axis=1)
    index = np.arange(n * n).reshape(n, n)
    a, b, c, d = index[:-1, :-1].ravel(), index[:-1, 1:].ravel(), index[1:, :-1].ravel(), index[1:, 1:].ravel()
    faces = np.concatenate([np.stack([a, c, b], 1), np.stack([b, c, d], 1)])
    assert len(faces) > 200_000

    def edit(data):
        data["meshes"] = {}
        data["collision"] = {"default": "none"}
    path = copy_example(tmp_path, edit=edit)
    stl.write_stl(path.parent / "mesh_arm_description" / "meshes" / "base.stl", stl.triangles(vertices, faces))
    result = build(tmp_path, path)
    assert result.metrics["max_faces"] == 200_000 and result.metrics["mesh_parts"] == 2
    model = mujoco.MjModel.from_xml_path(str(result.mjcf))
    assert model.nmeshface >= len(faces)


@pytest.mark.skipif(not hulls.decomposition_available(), reason="coacd is not installed")
def test_convex_decomposition_with_coacd(tmp_path):
    def edit(data):
        data["collision"] = {"default": "none", "links": {"upper_arm_link": "convex_decomposition"}}
    result = build(tmp_path, copy_example(tmp_path, edit=edit))
    model = mujoco.MjModel.from_xml_path(str(result.mjcf))
    names = [model.geom(g).name for g in geoms_of(model, "upper_arm_link") if model.geom_contype[g]]
    assert names and all(name.startswith("upper_arm_link_decomp") for name in names)


# --- failures: a clear code and what / expected / found -----------------------------------------

def test_convex_decomposition_without_a_library_fails(tmp_path, monkeypatch):
    monkeypatch.setattr(hulls, "decomposition_available", lambda: False)
    path = copy_example(tmp_path, edit=lambda data: data["collision"].update(default="convex_decomposition"))
    error = failure(tmp_path, path)
    assert error.code == "CONVEX_DECOMPOSITION_UNAVAILABLE" and "coacd" in error.message and "upper_arm_link" in error.message


def test_missing_meshes_are_all_listed(tmp_path):
    path = copy_example(tmp_path)
    (path.parent / "mesh_arm_description" / "meshes" / "base.stl").unlink()
    (path.parent / "mesh_arm_description" / "meshes" / "forearm.obj").unlink()
    error = failure(tmp_path, path)
    assert error.code == "MESH_NOT_FOUND"
    messages = [error.message] + [row["message"] for row in error.rows()]
    assert messages[0].startswith("4 geometry problems")  # base and forearm, visual and collision each
    assert len(messages) == 4 and all(row["code"] == "MESH_NOT_FOUND" for row in error.rows())
    assert sum("base.stl" in m for m in messages) == 2 and sum("forearm.obj" in m for m in messages) == 2


def test_unresolved_package_fails(tmp_path):
    path = copy_example(tmp_path)
    urdf = path.parent / "mesh_arm_description" / "urdf" / "mesh_arm.urdf"
    urdf.write_text(urdf.read_text().replace("package://mesh_arm_description/", "package://other_pkg/"))
    error = failure(tmp_path, path)
    assert error.code == "MESH_PACKAGE_UNRESOLVED" and "other_pkg" in error.message and "meshes.packages" in error.message


def test_unsupported_mesh_format_fails(tmp_path):
    path = copy_example(tmp_path)
    urdf = path.parent / "mesh_arm_description" / "urdf" / "mesh_arm.urdf"
    shutil.copyfile(MESHES / "forearm.obj", path.parent / "mesh_arm_description" / "meshes" / "forearm.dae")
    urdf.write_text(urdf.read_text().replace("forearm.obj", "forearm.dae"))
    error = failure(tmp_path, path)
    assert error.code == "MESH_FORMAT_UNSUPPORTED" and ".dae" in error.message


def test_truncated_stl_fails(tmp_path):
    path = copy_example(tmp_path)
    mesh = path.parent / "mesh_arm_description" / "meshes" / "base.stl"
    mesh.write_bytes(mesh.read_bytes()[:1000])
    error = failure(tmp_path, path)
    assert error.code == "MESH_UNREADABLE" and "truncated" in error.message


def test_unreadable_urdf_fails_through_the_stage(tmp_path):
    path = copy_example(tmp_path, example=ARM_DEMO)
    spec = load_spec(path)
    (path.parent / "urdf" / "arm_demo.urdf").write_text("<robot name='x'><link name='a'>")
    summary = run_pipeline(spec, runs_dir=tmp_path / "runs", stages=mjcf_stage.STAGES)
    result = read_result(Path(summary["run_dir"]) / "mjcf" / "result.json")
    assert result.status == "failed" and result.errors[0]["code"] == "URDF_UNREADABLE"
    assert "expected" in result.errors[0]["message"]


def test_missing_urdf_fails_through_the_stage(tmp_path):
    path = copy_example(tmp_path, example=ARM_DEMO)
    spec = load_spec(path)
    (path.parent / "urdf" / "arm_demo.urdf").unlink()
    summary = run_pipeline(spec, runs_dir=tmp_path / "runs", stages=mjcf_stage.STAGES)
    result = read_result(Path(summary["run_dir"]) / "mjcf" / "result.json")
    assert result.status == "failed" and result.errors[0]["code"] == "MISSING_URDF"


# --- section validation --------------------------------------------------------------------------

@pytest.fixture
def demo_data():
    return yaml.safe_load(ARM_DEMO.read_text(encoding="utf-8"))


@pytest.mark.parametrize("actuator_entry, expected", [
    ({"kp": 1.0}, "actuators.default.type: required"),
    ({"type": "position"}, "actuators.default.kp: a position actuator needs kp"),
    ({"type": "velocity"}, "actuators.default.kv: a velocity actuator needs kv"),
    ({"type": "velocity", "kv": 1.0, "kp": 2.0}, "actuators.default.kp: a velocity actuator does not use kp"),
    ({"type": "motor", "kv": 1.0}, "actuators.default.kv: a motor actuator does not use kv"),
    ({"type": "servo"}, "actuators.default.type: expected one of"),
])
def test_actuator_validation(demo_data, actuator_entry, expected):
    data = copy.deepcopy(demo_data)
    data["actuators"]["default"] = actuator_entry
    errors = validate_data(data)
    assert any(error.startswith(expected) for error in errors), errors


def test_meshes_validation(demo_data):
    data = copy.deepcopy(demo_data)
    data["meshes"] = {"max_faces": 0, "packages": {"pkg": ""}, "extra": 1}
    errors = validate_data(data)
    assert any("meshes.max_faces: expected an integer >= 1" in e for e in errors)
    assert any("meshes.packages.pkg: expected a non-empty string" in e for e in errors)
    assert any("meshes: unknown key(s) 'extra'" in e for e in errors)


# --- STL helpers ---------------------------------------------------------------------------------

def test_stl_roundtrip_and_chunks(tmp_path):
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    faces = np.array([[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])
    triangles = stl.triangles(vertices, faces)
    stl.write_stl(tmp_path / "t.stl", triangles)
    read, kind = stl.read_stl(tmp_path / "t.stl")
    assert kind == "binary" and read.tobytes() == triangles.tobytes()
    assert [len(c) for c in stl.chunks(read, 3)] == [3, 1]
    assert np.allclose(np.linalg.norm(read["normal"], axis=1), 1)
    assert math.isclose(float(read["normal"][0][2]), -1.0)
