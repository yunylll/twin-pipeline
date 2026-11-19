"""Minimal URDF reader: link names and joints with their limits. No dependencies beyond the stdlib."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

JOINT_TYPES = ("fixed", "continuous", "revolute", "prismatic", "floating", "planar")


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
        item = UrdfJoint(
            name=joint.get("name"),
            type=joint.get("type"),
            parent=joint.find("parent").get("link"),
            child=joint.find("child").get("link"),
            lower=_float(limit, "lower"),
            upper=_float(limit, "upper"),
            effort=_float(limit, "effort"),
            velocity=_float(limit, "velocity"),
        )
        joints[item.name] = item
    return UrdfModel(name=root.get("name"), links=links, joints=joints)
