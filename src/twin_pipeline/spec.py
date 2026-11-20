"""Load and validate ``twin.yaml`` (schema_version 1). See docs/twin-yaml.md.

The core owns the top-level identity and the robot facts that are not tied to one stage: ``urdf``,
``up_axis``, ``joints`` (calibration overrides with provenance), ``home`` and ``assumptions``.
Every other top-level section belongs to a stage module, which validates it (``SECTIONS`` in
``twin_pipeline.stages.<module>``) and may cross-check it against the URDF (``URDF_CHECKS``).
"""

from __future__ import annotations

import difflib
import hashlib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from . import checks
from .urdf import UrdfModel, mimic_problems, read_urdf

SCHEMA_VERSION = 1
PROVENANCE = ("cad", "assumed", "measured", "calibrated")
UP_AXES = ("+x", "-x", "+y", "-y", "+z", "-z")
CORE_KEYS = ("schema_version", "robot", "description", "urdf", "up_axis", "joints", "home", "assumptions")
JOINT_VALUES = ("lower", "upper", "effort", "velocity", "damping", "friction", "zero_offset")
JOINT_KEYS = JOINT_VALUES + ("source", "sources", "note")
ASSUMPTION_KEYS = ("id", "text", "source", "value", "note")
ROBOT_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_\-]*$")


class SpecError(ValueError):
    def __init__(self, path: Path | str, errors: list[str]):
        self.path = Path(path)
        self.errors = errors
        super().__init__(f"{path}: {len(errors)} problem(s)\n" + "\n".join(f"  - {e}" for e in errors))


@dataclass(frozen=True)
class TwinSpec:
    path: Path
    data: dict

    @property
    def dir(self) -> Path:
        return self.path.parent

    @property
    def robot(self) -> str:
        return self.data["robot"]

    @property
    def up_axis(self) -> str:
        return self.data.get("up_axis", "+z")

    @property
    def urdf_path(self) -> Path | None:
        value = self.data.get("urdf")
        return None if value is None else self.resolve(value)

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.path.read_bytes()).hexdigest()

    def resolve(self, value: str | Path) -> Path:
        path = Path(value)
        return path if path.is_absolute() else (self.dir / path).resolve()

    def section(self, name: str) -> dict:
        return self.data.get(name) or {}

    def joint_override(self, name: str) -> dict:
        return (self.data.get("joints") or {}).get(name) or {}

    def source_of(self, joint: str, field: str) -> str | None:
        """Provenance of one overridden joint value, or None when twin.yaml does not set it."""
        override = self.joint_override(joint)
        if field not in override:
            return None
        return (override.get("sources") or {}).get(field, override.get("source"))

    def zero_offset(self, joint) -> float:
        return float(self.joint_override(joint).get("zero_offset", 0.0))

    def limits(self, joint, urdf: UrdfModel) -> tuple[float | None, float | None]:
        """Effective (lower, upper) in the twin's joint coordinate, the one every stage uses.

        zero_offset o puts the twin's zero at URDF position o, so twin q = URDF q - o: a limit given in
        twin.yaml is already in the twin's coordinate and is used as it is; a URDF limit is shifted by -o.
        """
        override, item, offset = self.joint_override(joint), urdf.joints[joint], self.zero_offset(joint)

        def pick(key, urdf_value):
            if key in override:
                return override[key]
            return None if urdf_value is None else urdf_value - offset

        return pick("lower", item.lower), pick("upper", item.upper)

    def follower_position(self, joint, urdf: UrdfModel, leader_position: float) -> float:
        """Where a URDF mimic joint sits, in the twin's coordinate, when its leader is at leader_position (twin's coordinate).

        URDF: follower = offset + multiplier * leader, both in URDF coordinates; twin q = URDF q - zero_offset.
        """
        mimic = urdf.joints[joint].mimic
        leader_urdf = leader_position + self.zero_offset(mimic.leader)
        return mimic.offset + mimic.multiplier * leader_urdf - self.zero_offset(joint)


def _validate_joints(joints: Any) -> list[str]:
    errors = checks.mapping("joints", joints)
    if errors:
        return errors
    for name, override in joints.items():
        where = f"joints.{name}"
        if checks.mapping(where, override):
            errors += checks.mapping(where, override)
            continue
        errors += checks.unknown_keys(where, override, JOINT_KEYS)
        for field in JOINT_VALUES:
            if field not in override:
                continue
            if field in ("effort", "velocity"):
                errors += checks.number(f"{where}.{field}", override[field], low=0, low_open=True)
            elif field in ("damping", "friction"):
                errors += checks.number(f"{where}.{field}", override[field], low=0)
            else:
                errors += checks.number(f"{where}.{field}", override[field])
        lower, upper = override.get("lower"), override.get("upper")
        if checks.is_number(lower) and checks.is_number(upper) and lower > upper:
            errors.append(f"{where}: lower {lower} is above upper {upper}")
        given = [field for field in JOINT_VALUES if field in override]
        sources = override.get("sources") or {}
        if checks.mapping(f"{where}.sources", sources):
            errors += checks.mapping(f"{where}.sources", sources)
            sources = {}
        for field, source in sources.items():
            if field not in given:
                errors.append(f"{where}.sources.{field}: names a value this joint does not set (it sets: {', '.join(given) or 'nothing'})")
            errors += checks.choice(f"{where}.sources.{field}", source, PROVENANCE)
        if "source" in override:
            errors += checks.choice(f"{where}.source", override["source"], PROVENANCE)
        missing = [field for field in given if field not in sources and "source" not in override]
        if missing:
            errors.append(f"{where}: {', '.join(missing)} need a provenance: add source ({' | '.join(PROVENANCE)}) or sources.<field>")
    return errors


def _validate_assumptions(rows: Any) -> list[str]:
    if not isinstance(rows, list):
        return [f"assumptions: expected a list, got {checks.show(rows)}"]
    errors, seen = [], set()
    for index, row in enumerate(rows):
        where = f"assumptions[{index}]"
        if checks.mapping(where, row):
            errors += checks.mapping(where, row)
            continue
        errors += checks.unknown_keys(where, row, ASSUMPTION_KEYS)
        errors += checks.string(f"{where}.id", row.get("id"))
        errors += checks.string(f"{where}.text", row.get("text"))
        if "source" in row:
            errors += checks.choice(f"{where}.source", row["source"], PROVENANCE)
        if row.get("id") in seen:
            errors.append(f"{where}.id: {row.get('id')!r} is used twice")
        seen.add(row.get("id"))
    return errors


def validate_data(data: Any) -> list[str]:
    """Check the parsed YAML without touching the file system."""
    from .stages import section_validators

    if not isinstance(data, dict):
        return [f"top level: expected a mapping, got {checks.show(data)}"]
    validators = section_validators()
    errors = checks.unknown_keys("top level", data, set(CORE_KEYS) | set(validators))
    if data.get("schema_version") != SCHEMA_VERSION:
        errors.append(f"schema_version: expected {SCHEMA_VERSION}, got {checks.show(data.get('schema_version'))}")
    robot = data.get("robot")
    if not isinstance(robot, str) or not ROBOT_NAME.match(robot):
        errors.append(f"robot: expected a name of letters, digits, '_' or '-' starting with a letter or digit, got {checks.show(robot)}")
    if "description" in data:
        errors += checks.string("description", data["description"])
    if "urdf" in data:
        errors += checks.string("urdf", data["urdf"])
    elif "cad" not in data:
        errors.append("urdf: required unless a cad section exports the URDF")
    if "up_axis" in data:
        errors += checks.choice("up_axis", data["up_axis"], UP_AXES)
    if "joints" in data:
        errors += _validate_joints(data["joints"])
    if "home" in data:
        home = data["home"]
        errors += checks.mapping("home", home)
        if isinstance(home, dict):
            for name, value in home.items():
                errors += checks.number(f"home.{name}", value)
    if "assumptions" in data:
        errors += _validate_assumptions(data["assumptions"])
    for name, validator in validators.items():
        if name in data:
            errors += validator(name, data[name])
    return errors


def _unknown_joint(where: str, name: str, urdf: UrdfModel) -> str:
    close = difflib.get_close_matches(name, list(urdf.joints), n=3)
    hint = f"; did you mean {', '.join(close)}?" if close else ""
    return f"{where}: no joint {name!r} in the URDF{hint}"


def check_against_urdf(spec: TwinSpec, urdf: UrdfModel) -> list[str]:
    """Cross-check joint names, limits and home values (and every stage section) against the URDF."""
    from .stages import urdf_checks

    errors = list(mimic_problems(urdf))
    for name, override in (spec.data.get("joints") or {}).items():
        if name not in urdf.joints:
            errors.append(_unknown_joint(f"joints.{name}", name, urdf))
            continue
        item = urdf.joints[name]
        if not item.movable:
            errors.append(f"joints.{name}: the URDF joint is {item.type}; only movable joints take overrides")
        elif item.type == "continuous" and ({"lower", "upper"} & set(override)):
            errors.append(f"joints.{name}: the URDF joint is continuous; lower/upper would need a revolute joint")
        else:
            lower, upper = spec.limits(name, urdf)
            if lower is not None and upper is not None and lower > upper:
                errors.append(f"joints.{name}: effective lower {lower} is above effective upper {upper} (override merged with URDF)")
    for name, value in (spec.data.get("home") or {}).items():
        if name not in urdf.joints:
            errors.append(_unknown_joint(f"home.{name}", name, urdf))
            continue
        item = urdf.joints[name]
        if not item.movable:
            errors.append(f"home.{name}: the URDF joint is {item.type} and cannot be posed")
        elif item.mimic is not None:
            errors.append(f"home.{name}: the joint follows {item.mimic.leader} through a URDF <mimic> "
                          f"(offset {item.mimic.offset:g} + {item.mimic.multiplier:g} x {item.mimic.leader}); set home.{item.mimic.leader} instead")
        elif item.bounded:
            lower, upper = spec.limits(name, urdf)
            if lower is not None and upper is not None and not lower <= value <= upper:
                errors.append(f"home.{name}: {value} is outside the effective limits [{lower}, {upper}]")
    for check in urdf_checks():
        errors += check(spec, urdf)
    return errors


def load_spec(path: str | Path, *, check_urdf: bool = True) -> TwinSpec:
    """Read and validate twin.yaml. Raises SpecError listing every problem found.

    With check_urdf, the spec is also cross-checked against its URDF when that file exists already
    (a URDF that a cad section will export later is not required yet).
    """
    path = Path(path).resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise SpecError(path, [f"not valid YAML: {error}"]) from error
    errors = validate_data(data)
    if errors:
        raise SpecError(path, errors)
    spec = TwinSpec(path, data)
    if check_urdf and spec.urdf_path is not None and spec.urdf_path.exists():
        errors = check_against_urdf(spec, read_urdf(spec.urdf_path))
        if errors:
            raise SpecError(path, errors)
    return spec
