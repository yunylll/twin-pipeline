"""mjcf stage: build a MuJoCo model (MJCF + assets) from the URDF and twin.yaml.

Owner: session A (docs/ROADMAP.md). Placeholder: validates its sections and reports skipped.
Sections owned: meshes, collision, actuators.
Outputs when implemented: mjcf (the model XML), assets (its mesh folder); see docs/stage-contract.md.
"""

from __future__ import annotations

from .. import checks
from ..result import StageResult
from ..stage import Stage, StageContext

COLLISION_MODES = ("none", "mesh", "convex_hull", "convex_decomposition")
ACTUATOR_TYPES = ("position", "velocity", "motor", "none")
ACTUATOR_KEYS = ("type", "kp", "kv")


def validate_meshes(where, value) -> list[str]:
    errors = checks.mapping(where, value)
    if errors:
        return errors
    errors += checks.unknown_keys(where, value, ("max_faces",))
    if "max_faces" in value:
        errors += checks.integer(f"{where}.max_faces", value["max_faces"], low=1)
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
    if "type" in value:
        errors += checks.choice(f"{where}.type", value["type"], ACTUATOR_TYPES)
    for key in ("kp", "kv"):
        if key in value:
            errors += checks.number(f"{where}.{key}", value[key], low=0)
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
        return StageResult.failed("mjcf", "MISSING_URDF", f"no URDF to build from (looked for {urdf})")
    return StageResult.skipped("mjcf", "mjcf is not implemented yet (session A, see docs/ROADMAP.md)")


STAGES = [Stage("mjcf", run, "Build the MuJoCo model from the URDF and twin.yaml")]
SECTIONS = {"meshes": validate_meshes, "collision": validate_collision, "actuators": validate_actuators}
URDF_CHECKS = [check_names]
