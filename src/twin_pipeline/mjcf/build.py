"""Build a MuJoCo model (MJCF + mesh assets) from a URDF and the twin.yaml sections the mjcf stage owns.

How the pieces fit (each choice is in docs/twin-yaml.md as well):

* MuJoCo's own URDF importer reads the kinematic tree and the inertials, with every <visual> and
  <collision> removed first; geometry is added afterwards so this module decides what each geom is.
* ``fusestatic`` is off, so a link on a fixed joint (a tool frame) stays its own body, and it also
  gets a site of the same name for sensors and lookups.
* Mass and inertia come from the URDF inertials only (``inertiafromgeom="false"``); meshes never add
  mass, so splitting or replacing collision geometry cannot change the dynamics.
* Visual geoms are group 2 and never collide; collision geoms are group 3 and follow ``collision``.
* Joint overrides from twin.yaml replace the URDF values; actuators are added only where twin.yaml
  asks for one, with the gains it gives and ranges from the effective joint values.
* ``home`` becomes the keyframe ``home``.
* A URDF ``<mimic>`` becomes a joint equality constraint ``mimic_<joint>`` (follower = offset +
  multiplier * leader, in URDF coordinates). A follower gets no actuator, and its keyframe position
  is computed from its leader's.
"""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

import mujoco as mj
import numpy as np

from ..spec import TwinSpec
from ..urdf import UrdfModel, mimic_problems, read_urdf
from . import collision as hulls
from . import stl
from .errors import BuildError
from .geometry import Geometry, MeshResolver, read_geometry
from .meshes import MeshStore, read_triangles

GRAVITY = 9.81
STL_FACE_LIMIT = 200_000  # MuJoCo's STL decoder refuses larger files
DEFAULT_COLLISION = "mesh"
VISUAL_GROUP, COLLISION_GROUP = 2, 3
MODEL_FILE, ASSETS_DIR = "model.xml", "assets"


@dataclass
class Build:
    mjcf: Path
    assets: Path
    metrics: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)


@dataclass
class JointValues:
    """A movable joint's effective values: twin.yaml override, else URDF; limits in the twin's joint coordinate."""
    name: str
    type: str
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None
    offset: float

    @property
    def bounded(self) -> bool:
        return self.lower is not None and self.upper is not None


def gravity(up_axis: str) -> list[float]:
    sign, axis = (1.0 if up_axis[0] == "+" else -1.0), "xyz".index(up_axis[1])
    vector = [0.0, 0.0, 0.0]
    vector[axis] = -sign * GRAVITY
    return vector


def joint_values(spec: TwinSpec, urdf: UrdfModel, name: str) -> JointValues:
    """Limits in the twin's coordinate (spec.limits applies zero_offset: twin q = URDF q - zero_offset)."""
    item, override = urdf.joints[name], spec.joint_override(name)
    offset = spec.zero_offset(name)
    lower, upper = spec.limits(name, urdf) if item.bounded else (None, None)
    return JointValues(name, item.type, lower, upper, override.get("effort", item.effort),
                       override.get("velocity", item.velocity), offset)


def actuator_config(spec: TwinSpec, joint: str) -> dict | None:
    """The actuator twin.yaml asks for: actuators.joints.<joint> replaces actuators.default entirely."""
    section = spec.section("actuators")
    config = (section.get("joints") or {}).get(joint, section.get("default"))
    return None if not config or config.get("type") == "none" else config


def collision_mode(spec: TwinSpec, link: str) -> str:
    section = spec.section("collision")
    return (section.get("links") or {}).get(link, section.get("default", DEFAULT_COLLISION))


def _read_root(urdf_path: Path) -> ET.Element:
    try:
        root = ET.parse(urdf_path).getroot()
    except (ET.ParseError, OSError) as error:
        raise BuildError("URDF_UNREADABLE", f"{urdf_path.name}: expected a readable URDF (XML); {error}") from None
    if root.tag != "robot":
        raise BuildError("URDF_UNREADABLE", f"{urdf_path.name}: expected a <robot> root element, found <{root.tag}>")
    return root


def _kinematic_urdf(root: ET.Element, warnings: list[str]) -> str:
    """The URDF without geometry, with the compiler settings the builder relies on."""
    root = copy.deepcopy(root)
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    for block in root.findall("mujoco"):
        warnings.append("the URDF's own <mujoco> block was replaced by the mjcf stage's compiler settings")
        root.remove(block)
    for joint in root.findall("joint"):  # the builder adds mimic constraints itself, named and checked
        for mimic in joint.findall("mimic"):
            joint.remove(mimic)
    root.insert(0, ET.fromstring('<mujoco><compiler fusestatic="false" discardvisual="false"/></mujoco>'))
    return ET.tostring(root, encoding="unicode")


class _Geoms:
    """Adds geoms and mesh assets to the MjSpec, keeping counts for the metrics."""

    def __init__(self, mj, spec, store: MeshStore):
        self.mj, self.spec, self.store = mj, spec, store
        self.mesh_names: dict[tuple[str, tuple], str] = {}
        self.taken: set[str] = set()
        self.count = Counter()

    def mesh(self, file: str, scale=(1.0, 1.0, 1.0)) -> str:
        key = (file, tuple(scale))
        if key not in self.mesh_names:
            name, index = Path(file).stem, 1
            while name in self.taken:
                index += 1
                name = f"{Path(file).stem}_{index}"
            self.taken.add(name)
            self.spec.add_mesh(name=name, file=file, scale=list(scale))
            self.mesh_names[key] = name
        return self.mesh_names[key]

    def add(self, body, name: str, item: Geometry | None, *, collides: bool, mesh: str | None = None, rgba=None):
        mj = self.mj
        attributes = {"name": name, "group": COLLISION_GROUP if collides else VISUAL_GROUP}
        if not collides:
            attributes.update(contype=0, conaffinity=0)
        if item is not None:
            attributes.update(pos=list(item.pos), quat=list(item.quat))
        if mesh is not None:
            attributes.update(type=mj.mjtGeom.mjGEOM_MESH, meshname=mesh)
        else:
            kind = {"box": mj.mjtGeom.mjGEOM_BOX, "cylinder": mj.mjtGeom.mjGEOM_CYLINDER, "sphere": mj.mjtGeom.mjGEOM_SPHERE}[item.kind]
            attributes.update(type=kind, size=list(item.size) + [0.0] * (3 - len(item.size)))
        if rgba is not None:
            attributes["rgba"] = list(rgba)
        body.add_geom(**attributes)
        self.count["collision_geoms" if collides else "visual_geoms"] += 1

    def element(self, body, item: Geometry, *, collides: bool) -> list[str]:
        """One URDF element as geom(s); returns the asset files it used."""
        label = f"{item.link}_{item.role}{item.index}"
        if item.kind != "mesh":
            self.add(body, label, item, collides=collides, rgba=item.rgba)
            return []
        prepared = self.store.prepare(item.path)
        for index, file in enumerate(prepared.files):
            name = label if len(prepared.files) == 1 else f"{label}_part{index}"
            self.add(body, name, item, collides=collides, mesh=self.mesh(file, item.scale), rgba=item.rgba)
        return prepared.files


def _link_triangles(items: list[Geometry]) -> tuple[np.ndarray, np.ndarray]:
    """All collision meshes of one link in the link frame, as one vertex/triangle set."""
    vertices, faces, offset = [], [], 0
    for item in items:
        try:
            v, f = read_triangles(item.path)
        except (stl.StlError, ValueError, IndexError) as error:
            raise BuildError("MESH_UNREADABLE", f"link {item.link} collision[{item.index}] {item.path.name}: {error}") from None
        vertices.append(hulls.to_link_frame(v, item.scale, item.pos, item.quat))
        faces.append(f + offset)
        offset += len(v)
    return np.vstack(vertices), np.vstack(faces)


def build_mjcf(spec: TwinSpec, urdf_path: Path, out_dir: Path) -> Build:
    """Write <out_dir>/model.xml and <out_dir>/assets/. Raises BuildError for every expected problem."""
    warnings: list[str] = []
    root = _read_root(urdf_path)
    try:
        urdf = read_urdf(urdf_path)
    except (ValueError, AttributeError, TypeError) as error:
        raise BuildError("URDF_UNREADABLE", f"{urdf_path.name}: {error}") from None

    meshes = spec.section("meshes")
    packages = {name: spec.resolve(path) for name, path in (meshes.get("packages") or {}).items()}
    links, problems = read_geometry(root, MeshResolver(urdf_path, packages))
    for name, geometry in links.items():
        for item in geometry.visual + geometry.collision:
            if item.kind == "mesh" and not item.path.is_file():
                problems.append(BuildError("MESH_NOT_FOUND", f"link {name} {item.role}[{item.index}]: expected the mesh "
                                                             f"{item.uri!r} at {item.path}; no such file"))
    if problems:
        first = problems[0]
        raise BuildError(first.code, first.message if len(problems) == 1 else
                         f"{len(problems)} geometry problems; first: {first.message}", more=problems[1:])

    modes = {name: collision_mode(spec, name) for name in links}
    wants_decomposition = [name for name in links if modes[name] == "convex_decomposition"
                           and any(item.kind == "mesh" for item in links[name].collision)]
    if wants_decomposition and not hulls.decomposition_available():
        raise BuildError("CONVEX_DECOMPOSITION_UNAVAILABLE",
                         f"collision mode convex_decomposition (links: {', '.join(wants_decomposition)}) needs the coacd "
                         f"package; it is not installed. Install it (pip install coacd) or choose mesh / convex_hull / none")

    problems = mimic_problems(urdf)
    if problems:
        raise BuildError("MIMIC_INVALID", "; ".join(problems))
    textured = sorted({link.get("name") for link in root.findall("link") for texture in link.iter("texture")})
    if textured:
        warnings.append(f"URDF textures are not carried over (links: {', '.join(textured)}); colours are")

    try:
        mjspec = mj.MjSpec.from_string(_kinematic_urdf(root, warnings))
    except ValueError as error:
        raise BuildError("URDF_IMPORT_FAILED", f"MuJoCo's URDF importer refused {urdf_path.name}: {error}") from None
    mjspec.compiler.inertiafromgeom = mj.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
    mjspec.option.gravity = gravity(spec.up_axis)
    # implicitfast integrates actuator and joint damping implicitly: the default Euler integrator makes
    # high-kv position actuators on light links chatter (found by the smoke stage on the demo arm)
    mjspec.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST

    assets = out_dir / ASSETS_DIR
    max_faces = meshes.get("max_faces", STL_FACE_LIMIT)
    store = MeshStore(assets, max_faces)
    geoms = _Geoms(mj, mjspec, store)
    collision_links = Counter()
    chunked_collision = []

    try:
        for name, geometry in links.items():
            body = mjspec.body(name)
            if body is None:  # the importer keeps every link; guard against URDFs it rewrites
                warnings.append(f"link {name} has no body in the MuJoCo model; its geometry is left out")
                continue
            for item in geometry.visual:
                geoms.element(body, item, collides=False)
            mode = modes[name]
            if geometry.collision:
                collision_links[mode] += 1
            if mode == "none":
                continue
            primitives = [item for item in geometry.collision if item.kind != "mesh"]
            mesh_items = [item for item in geometry.collision if item.kind == "mesh"]
            for item in (geometry.collision if mode == "mesh" else primitives):
                files = geoms.element(body, item, collides=True)
                if len(files) > 1:
                    chunked_collision.append(f"{name} ({item.path.name}: {len(files)} parts)")
            if mode == "convex_hull" and mesh_items:
                vertices, _ = _link_triangles(mesh_items)
                hull_vertices, hull_faces = hulls.convex_hull(vertices)
                file = store.add(f"{name}_hull", stl.triangles(hull_vertices, hull_faces), f"convex hull of {name} collision")
                geoms.add(body, f"{name}_hull", None, collides=True, mesh=geoms.mesh(file))
            elif mode == "convex_decomposition" and mesh_items:
                vertices, faces = _link_triangles(mesh_items)
                for index, (part_vertices, part_faces) in enumerate(hulls.convex_decomposition(vertices, faces)):
                    file = store.add(f"{name}_decomp{index}", stl.triangles(part_vertices, part_faces),
                                     f"convex part {index} of {name} collision")
                    geoms.add(body, f"{name}_decomp{index}", None, collides=True, mesh=geoms.mesh(file))
    except OSError as error:
        raise BuildError("MESH_UNREADABLE", f"reading or writing a mesh failed: {error}") from None
    if chunked_collision:
        warnings.append("collision meshes over meshes.max_faces were split, and MuJoCo collides each part through its own "
                        f"convex hull (use convex_hull for one hull per link): {', '.join(chunked_collision)}")

    fixed_frames = [joint.child for joint in urdf.joints.values() if joint.type == "fixed"]
    for frame in fixed_frames:
        body = mjspec.body(frame)
        if body is not None:
            body.add_site(name=frame)

    values = {name: joint_values(spec, urdf, name) for name, joint in urdf.joints.items() if joint.movable}
    for name in spec.data.get("joints") or {}:
        override, joint = spec.joint_override(name), mjspec.joint(name)
        if joint is None or name not in values:
            continue
        item = values[name]
        if item.bounded:
            joint.range = [item.lower, item.upper]
            joint.limited = mj.mjtLimited.mjLIMITED_TRUE
        if "damping" in override:
            joint.damping = [float(override["damping"]), 0.0, 0.0]
        if "friction" in override:
            joint.frictionloss = float(override["friction"])
        if "effort" in override:
            joint.actfrcrange = [-float(override["effort"]), float(override["effort"])]
            joint.actfrclimited = mj.mjtLimited.mjLIMITED_TRUE
        if item.offset:
            joint.ref = -item.offset

    # URDF <mimic>: a joint equality constraint. MuJoCo's form is y - y0 = a0 + a1 (x - x0) with y0, x0 the
    # joints' qpos0 (= ref = -zero_offset), so in URDF coordinates it is exactly follower = offset + multiplier * leader.
    mimics = {}
    for name, joint in urdf.joints.items():
        if joint.mimic is None:
            continue
        equality = mjspec.add_equality()
        equality.type, equality.objtype = mj.mjtEq.mjEQ_JOINT, mj.mjtObj.mjOBJ_JOINT
        equality.name, equality.name1, equality.name2 = f"mimic_{name}", name, joint.mimic.leader
        data = list(equality.data)
        data[:5] = [joint.mimic.offset, joint.mimic.multiplier, 0.0, 0.0, 0.0]
        equality.data = data
        mimics[name] = {"leader": joint.mimic.leader, "multiplier": joint.mimic.multiplier, "offset": joint.mimic.offset}
        follower, leader = values[name], values[joint.mimic.leader]
        if follower.bounded:
            if leader.bounded:
                reach = sorted(spec.follower_position(name, urdf, value) for value in (leader.lower, leader.upper))
                if reach[0] < follower.lower - 1e-9 or reach[1] > follower.upper + 1e-9:
                    warnings.append(f"mimic {name}: over {leader.name}'s range [{leader.lower:.6g}, {leader.upper:.6g}] it would move to "
                                    f"[{reach[0]:.6g}, {reach[1]:.6g}], outside its own limits [{follower.lower:.6g}, {follower.upper:.6g}]; "
                                    f"at the ends the joint limit and the mimic constraint fight")
            elif joint.mimic.multiplier != 0:
                warnings.append(f"mimic {name}: the leader {leader.name} is unlimited but {name} has limits [{follower.lower:.6g}, {follower.upper:.6g}]; "
                                f"they fight once the leader turns far enough")
    if mimics and not spec.data.get("home") and any(m["offset"] for m in mimics.values()):
        warnings.append("mimic joints with an offset are not at their constraint at the URDF zero pose and settle at the start; "
                        "give a home pose so the keyframe starts consistent")

    actuated = {}
    skipped_followers = []
    for name, item in values.items():
        config = actuator_config(spec, name)
        if config is None or mjspec.joint(name) is None:
            continue
        if name in mimics:  # it moves with its leader; an actuator would fight the constraint
            skipped_followers.append(name)
            continue
        actuator = mjspec.add_actuator(name=name, target=name, trntype=mj.mjtTrn.mjTRN_JOINT)
        kind = config["type"]
        if kind == "position":
            actuator.set_to_position(kp=float(config["kp"]), kv=float(config.get("kv", 0.0)))
            limits = (item.lower, item.upper) if item.bounded else None
            missing = f"the joint is {item.type} with no limits"
        elif kind == "velocity":
            actuator.set_to_velocity(kv=float(config["kv"]))
            limits = (-item.velocity, item.velocity) if item.velocity else None
            missing = "no velocity limit in the URDF or joints.<joint>.velocity"
        else:
            actuator.set_to_motor()
            limits = (-item.effort, item.effort) if item.effort else None
            missing = "no effort limit in the URDF or joints.<joint>.effort"
        if limits is None:
            warnings.append(f"{kind} actuator {name}: {missing}, so its ctrlrange is open")
        else:
            actuator.ctrlrange = list(limits)
            actuator.ctrllimited = mj.mjtLimited.mjLIMITED_TRUE
        actuated[name] = kind
    if skipped_followers:
        warnings.append(f"no actuator for the mimic joints {', '.join(skipped_followers)}: they move with their leaders")

    def compile_spec():
        try:
            return mjspec.compile()
        except ValueError as error:
            raise BuildError("MJCF_COMPILE_FAILED", f"MuJoCo could not compile the model: {error}") from None

    assets.mkdir(parents=True, exist_ok=True)
    # meshdir stays relative so the saved file can move with its assets; modelfiledir anchors it while building
    mjspec.modelfiledir = str(out_dir.resolve())
    mjspec.meshdir = ASSETS_DIR
    model = compile_spec()
    home = spec.data.get("home") or {}
    if home:
        qpos = model.qpos0.copy()
        for name, value in home.items():
            item = values.get(name)
            if item is not None and item.bounded and not item.lower <= value <= item.upper:
                raise BuildError("HOME_OUTSIDE_LIMITS", f"home.{name}: expected a value inside the effective range "
                                                        f"[{item.lower}, {item.upper}] (after zero_offset); found {value}")
            qpos[model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)]] = value
        for name, mimic in mimics.items():  # followers start where their leader puts them
            leader_value = float(qpos[model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, mimic["leader"])]])
            value = spec.follower_position(name, urdf, leader_value)
            item = values[name]
            if item.bounded and not item.lower - 1e-9 <= value <= item.upper + 1e-9:
                raise BuildError("HOME_OUTSIDE_LIMITS", f"home: {mimic['leader']} at {leader_value:.6g} puts the mimic joint {name} at "
                                                        f"{value:.6g}, outside its range [{item.lower:.6g}, {item.upper:.6g}]")
            qpos[model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)]] = value
        ctrl = []
        for index in range(model.nu):
            joint = model.actuator_trnid[index, 0]
            ctrl.append(float(qpos[model.jnt_qposadr[joint]]) if actuated[model.actuator(index).name] == "position" else 0.0)
        mjspec.add_key(name="home", qpos=qpos.tolist(), ctrl=ctrl)
        model = compile_spec()

    path = out_dir / MODEL_FILE
    try:
        text = mjspec.to_xml()
    except ValueError as error:
        raise BuildError("MJCF_SAVE_FAILED", f"MuJoCo could not write the model as MJCF: {error}") from None
    path.write_text(text, encoding="utf-8")
    try:
        saved = mj.MjModel.from_xml_path(str(path))
    except ValueError as error:
        raise BuildError("MJCF_RELOAD_FAILED", f"the saved {MODEL_FILE} does not load with MjModel.from_xml_path: {error}") from None

    prepared = list(store.files.values())
    metrics = {
        "nbody": saved.nbody, "njnt": saved.njnt, "nu": saved.nu, "ngeom": saved.ngeom,
        "nmesh": saved.nmesh, "nmeshface": int(saved.nmeshface), "nkey": saved.nkey, "nsite": saved.nsite,
        "visual_geoms": geoms.count["visual_geoms"], "collision_geoms": geoms.count["collision_geoms"],
        "mesh_files": len(prepared), "meshes_split": sum(item.split for item in prepared),
        "mesh_parts": sum(len(item.files) for item in prepared if item.split),
        "meshes_converted": sum(item.format == "ascii-stl" for item in prepared),
        "max_faces": max_faces, "gravity": gravity(spec.up_axis),
        "collision_links": dict(collision_links), "actuators": actuated, "fixed_frames": fixed_frames,
        "neq": saved.neq, "mimic": mimics,
    }
    return Build(mjcf=path, assets=assets, metrics=metrics, warnings=warnings)
