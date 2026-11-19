"""mjcf stage: build a MuJoCo model (MJCF + assets) from the URDF and twin.yaml.

Owner: session A (docs/ROADMAP.md). Sections owned: meshes, collision, actuators.
Outputs: mjcf (mjcf/model.xml, loads with mujoco.MjModel.from_xml_path), assets (mjcf/assets/, its meshes).
The building is in twin_pipeline.mjcf; this module validates the sections and reports the result.
"""

from __future__ import annotations

from .. import checks
from ..mjcf import BuildError
from ..result import StageResult
from ..stage import Stage, StageContext

COLLISION_MODES = ("none", "mesh", "convex_hull", "convex_decomposition")
ACTUATOR_TYPES = ("position", "velocity", "motor", "none")
ACTUATOR_KEYS = ("type", "kp", "kv")
# per type: gains that must be given, gains that must not be (kv on a position actuator is optional damping)
ACTUATOR_GAINS = {"position": (("kp",), ()), "velocity": (("kv",), ("kp",)), "motor": ((), ("kp", "kv")), "none": ((), ("kp", "kv"))}


def validate_meshes(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, ("max_faces", "packages"))
    if "max_faces" in value:
        errors += checks.integer(f"{where}.max_faces", value["max_faces"], low=1)
    if "packages" in value:
        packages = value["packages"]
        errors += checks.mapping(f"{where}.packages", packages)
        if isinstance(packages, dict):
            for name, path in packages.items():
                errors += checks.string(f"{where}.packages.{name}", path)
    return errors


def validate_collision(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, ("default", "links"))
    if "default" in value:
        errors += checks.choice(f"{where}.default", value["default"], COLLISION_MODES)
    links = value.get("links", {})
    errors += checks.mapping(f"{where}.links", links)
    if isinstance(links, dict):
        for name, mode in links.items():
            errors += checks.choice(f"{where}.links.{name}", mode, COLLISION_MODES)
    return errors


def _validate_actuator(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, ACTUATOR_KEYS)
    if "type" not in value:
        return errors + [f"{where}.type: required; expected one of {', '.join(ACTUATOR_TYPES)}"]
    errors += checks.choice(f"{where}.type", value["type"], ACTUATOR_TYPES)
    for key in ("kp", "kv"):
        if key in value:
            errors += checks.number(f"{where}.{key}", value[key], low=0)
    needed, unused = ACTUATOR_GAINS.get(value["type"], ((), ()))
    for key in needed:
        if key not in value:
            errors.append(f"{where}.{key}: a {value['type']} actuator needs {key}; none given (it is never guessed)")
    for key in unused:
        if key in value:
            errors.append(f"{where}.{key}: a {value['type']} actuator does not use {key}; remove it")
    return errors


def validate_actuators(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, ("default", "joints"))
    if "default" in value:
        errors += _validate_actuator(f"{where}.default", value["default"])
    joints = value.get("joints", {})
    errors += checks.mapping(f"{where}.joints", joints)
    if isinstance(joints, dict):
        for name, actuator in joints.items():
            errors += _validate_actuator(f"{where}.joints.{name}", actuator)
    return errors


def check_names(spec, urdf) -> list[str]:
    errors = []
    for name in (spec.section("collision").get("links") or {}):
        if name not in urdf.links:
            errors.append(f"collision.links.{name}: no link {name!r} in the URDF")
    for name in (spec.section("actuators").get("joints") or {}):
        if name not in urdf.joints:
            errors.append(f"actuators.joints.{name}: no joint {name!r} in the URDF")
        elif not urdf.joints[name].movable:
            errors.append(f"actuators.joints.{name}: the URDF joint is {urdf.joints[name].type} and cannot be actuated")
    return errors


def run(context: StageContext) -> StageResult:
    urdf = context.urdf()
    if urdf is None or not urdf.exists():
        return StageResult.failed("mjcf", "MISSING_URDF", f"expected a URDF to build from; found none (looked for {urdf})")
    try:
        from ..mjcf.build import build_mjcf
    except ImportError as error:
        return StageResult.failed("mjcf", "SIM_EXTRA_MISSING", f"the mjcf stage needs numpy and mujoco; install the sim extra "
                                                               f"(pip install -e \".[sim]\"); found: {error}")
    try:
        build = build_mjcf(context.spec, urdf, context.stage_dir)
    except BuildError as error:
        return StageResult.failed("mjcf", error.code, error.message, errors=error.rows())
    return StageResult.succeeded("mjcf", outputs={"mjcf": context.rel(build.mjcf), "assets": context.rel(build.assets)},
                                 metrics=build.metrics, warnings=build.warnings)


STAGES = [Stage("mjcf", run, "Build the MuJoCo model from the URDF and twin.yaml")]
SECTIONS = {"meshes": validate_meshes, "collision": validate_collision, "actuators": validate_actuators}
URDF_CHECKS = [check_names]
