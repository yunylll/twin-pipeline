"""Minimal URDF reader: link names and joints with their limits. No dependencies beyond the stdlib."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

JOINT_TYPES = ("fixed", "continuous", "revolute", "prismatic", "floating", "planar")


@dataclass(frozen=True)
class Mimic:
    """<mimic joint= multiplier= offset=>: this joint's URDF position = offset + multiplier * the leader's."""
    leader: str
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    type: str
    parent: str
    child: str
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    mimic: Mimic | None = None

    @property
    def movable(self) -> bool:
        return self.type in ("continuous", "revolute", "prismatic")

    @property
    def bounded(self) -> bool:
        return self.type in ("revolute", "prismatic")


@dataclass(frozen=True)
class UrdfModel:
    name: str
    links: tuple[str, ...]
    joints: dict[str, UrdfJoint]


def _float(element, attribute):
    if element is None or element.get(attribute) is None:
        return None
    return float(element.get(attribute))


def read_urdf(path: str | Path) -> UrdfModel:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"{path}: root element is <{root.tag}>, expected <robot>")
    links = tuple(link.get("name") for link in root.findall("link"))
    joints = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        if mimic is not None:
            mimic = Mimic(mimic.get("joint"), float(mimic.get("multiplier", 1.0)), float(mimic.get("offset", 0.0)))
        item = UrdfJoint(
            name=joint.get("name"),
            type=joint.get("type"),
            parent=joint.find("parent").get("link"),
            child=joint.find("child").get("link"),
            lower=_float(limit, "lower"),
            upper=_float(limit, "upper"),
            effort=_float(limit, "effort"),
            velocity=_float(limit, "velocity"),
            mimic=mimic,
        )
        joints[item.name] = item
    return UrdfModel(name=root.get("name"), links=links, joints=joints)


def mimic_problems(model: UrdfModel) -> list[str]:
    """Mimic declarations that cannot be modelled: unknown or fixed leader, a fixed follower, chains."""
    problems = []
    for name, joint in model.joints.items():
        mimic = joint.mimic
        if mimic is None:
            continue
        leader = model.joints.get(mimic.leader)
        where = f"URDF joint {name} <mimic joint={mimic.leader!r}>"
        if not joint.movable:
            problems.append(f"{where}: a {joint.type} joint cannot follow another joint")
        elif mimic.leader == name:
            problems.append(f"{where}: a joint cannot follow itself")
        elif leader is None:
            problems.append(f"{where}: no joint {mimic.leader!r} in the URDF")
        elif not leader.movable:
            problems.append(f"{where}: the leader is a {leader.type} joint; expected a movable one")
        elif leader.mimic is not None:
            problems.append(f"{where}: the leader itself follows {leader.mimic.leader!r}; point the mimic at {leader.mimic.leader!r} with the combined multiplier and offset")
    return problems
