"""Read the visual and collision geometry of every URDF link, and find the mesh files it names."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import unquote, urlparse

from .errors import BuildError

PRIMITIVES = ("box", "cylinder", "sphere")


@dataclass
class Geometry:
    link: str
    role: str                      # "visual" or "collision"
    index: int                     # position among the link's elements of that role
    kind: str                      # box, cylinder, sphere or mesh
    size: tuple[float, ...] = ()   # MuJoCo size (half extents, radius, half length)
    uri: str | None = None         # the mesh filename as the URDF writes it
    path: Path | None = None       # resolved mesh file
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    rgba: tuple[float, float, float, float] | None = None


@dataclass
class LinkGeometry:
    visual: list[Geometry] = field(default_factory=list)
    collision: list[Geometry] = field(default_factory=list)


def numbers(text: str | None, count: int, default: tuple[float, ...], where: str) -> tuple[float, ...]:
    if text is None:
        if default is None:
            raise BuildError("URDF_INVALID", f"{where}: expected {count} number(s), found the attribute missing")
        return default
    try:
        values = tuple(float(part) for part in text.split())
    except ValueError:
        values = ()
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise BuildError("URDF_INVALID", f"{where}: expected {count} numbers, found {text!r}")
    return values


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """URDF rpy (fixed axes X, then Y, then Z: R = Rz(yaw) Ry(pitch) Rx(roll)) as a MuJoCo quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def origin_of(element: ET.Element, where: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    origin = element.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    pos = numbers(origin.get("xyz"), 3, (0.0, 0.0, 0.0), f"{where} origin xyz")
    rpy = numbers(origin.get("rpy"), 3, (0.0, 0.0, 0.0), f"{where} origin rpy")
    return pos, rpy_to_quat(*rpy)


class MeshResolver:
    """Turns a URDF mesh filename into a file.

    package://<name>/<rest>: <name> from meshes.packages in twin.yaml, else the nearest folder above
    the URDF that is called <name> (the usual <name>/urdf/<robot>.urdf layout of an exported package).
    file://<path> and absolute paths are used as they are; other paths are relative to the URDF.
    """

    def __init__(self, urdf_path: Path, packages: dict[str, Path] | None = None):
        self.urdf_dir = urdf_path.resolve().parent
        self.packages = dict(packages or {})

    def package_root(self, name: str) -> Path | None:
        if name in self.packages:
            return self.packages[name]
        for folder in (self.urdf_dir, *self.urdf_dir.parents):
            if folder.name == name:
                return folder
        return None

    def resolve(self, uri: str, where: str) -> Path:
        if uri.startswith("package://"):
            rest = uri[len("package://"):]
            name, _, inner = rest.partition("/")
            root = self.package_root(name)
            if root is None:
                raise BuildError("MESH_PACKAGE_UNRESOLVED",
                                 f"{where}: {uri!r} names package {name!r}; expected meshes.packages.{name} in twin.yaml "
                                 f"or a folder called {name!r} above the URDF ({self.urdf_dir}); found neither")
            return (root / unquote(inner)).resolve()
        if uri.startswith("file://"):
            parsed = urlparse(uri)
            text = unquote(parsed.path)
            if re.match(r"^/[A-Za-z]:", text):  # file:///C:/... on Windows
                text = text[1:]
            return Path(parsed.netloc + text if parsed.netloc else text).resolve()
        path = Path(uri)
        return path if path.is_absolute() else (self.urdf_dir / path).resolve()


def _material_colors(root: ET.Element) -> dict[str, tuple[float, ...]]:
    colors = {}
    for material in root.findall("material"):
        color = material.find("color")
        if material.get("name") and color is not None and color.get("rgba"):
            colors[material.get("name")] = numbers(color.get("rgba"), 4, (), f"material {material.get('name')} rgba")
    return colors


def _rgba(element: ET.Element, colors: dict, where: str) -> tuple[float, ...] | None:
    material = element.find("material")
    if material is None:
        return None
    color = material.find("color")
    if color is not None and color.get("rgba"):
        return numbers(color.get("rgba"), 4, (), f"{where} material rgba")
    return colors.get(material.get("name"))


def _geometry(link: str, role: str, index: int, element: ET.Element, colors: dict, resolver: MeshResolver) -> Geometry:
    where = f"link {link} {role}[{index}]"
    shape = element.find("geometry")
    shapes = list(shape) if shape is not None else []
    if len(shapes) != 1:
        raise BuildError("URDF_INVALID", f"{where}: expected one shape inside <geometry>, found {len(shapes)}")
    shape = shapes[0]
    pos, quat = origin_of(element, where)
    item = Geometry(link=link, role=role, index=index, kind=shape.tag, pos=pos, quat=quat,
                    rgba=_rgba(element, colors, where) if role == "visual" else None)
    if shape.tag == "box":
        item.size = tuple(value / 2 for value in numbers(shape.get("size"), 3, None, f"{where} box size"))
    elif shape.tag == "cylinder":
        radius = numbers(shape.get("radius"), 1, None, f"{where} cylinder radius")[0]
        length = numbers(shape.get("length"), 1, None, f"{where} cylinder length")[0]
        item.size = (radius, length / 2)
    elif shape.tag == "sphere":
        item.size = numbers(shape.get("radius"), 1, None, f"{where} sphere radius")
    elif shape.tag == "mesh":
        item.uri = shape.get("filename")
        if not item.uri:
            raise BuildError("URDF_INVALID", f"{where}: <mesh> needs a filename attribute; found none")
        item.path = resolver.resolve(item.uri, where)
        item.scale = numbers(shape.get("scale"), 3, (1.0, 1.0, 1.0), f"{where} mesh scale")
    else:
        raise BuildError("URDF_UNSUPPORTED_GEOMETRY",
                         f"{where}: expected one of box, cylinder, sphere, mesh; found <{shape.tag}>")
    return item


def read_geometry(root: ET.Element, resolver: MeshResolver) -> tuple[dict[str, LinkGeometry], list[BuildError]]:
    """Every link's geometry, and the problems found (all of them, so one run lists every missing mesh)."""
    colors = _material_colors(root)
    links, problems = {}, []
    for link in root.findall("link"):
        name = link.get("name")
        geometry = links.setdefault(name, LinkGeometry())
        for role in ("visual", "collision"):
            for index, element in enumerate(link.findall(role)):
                try:
                    getattr(geometry, role).append(_geometry(name, role, index, element, colors, resolver))
                except BuildError as error:
                    problems.append(error)
    return links, problems
