"""Copy the URDF's mesh files into the stage's assets folder in a form MuJoCo accepts.

* binary STL up to max_faces: copied byte for byte;
* ASCII STL: rewritten as binary STL (MuJoCo reads binary STL only), same triangles;
* STL over max_faces: split into consecutive chunks of whole faces (lossless, see stl.py);
* OBJ and MSH: copied; max_faces does not apply (the 200 000-face limit is MuJoCo's STL decoder's).
Anything else (e.g. DAE) is refused with MESH_FORMAT_UNSUPPORTED.
"""

from __future__ import annotations

import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import stl
from .errors import BuildError

FORMATS = (".stl", ".obj", ".msh")


@dataclass
class MeshFile:
    source: Path
    format: str                                   # stl, ascii-stl, obj, msh
    files: list[str] = field(default_factory=list)  # names inside the assets folder, in order
    faces: int | None = None

    @property
    def split(self) -> bool:
        return len(self.files) > 1


class MeshStore:
    def __init__(self, assets_dir: Path, max_faces: int):
        self.dir = assets_dir
        self.max_faces = max_faces
        self.files: dict[Path, MeshFile] = {}
        self._taken: set[str] = set()

    def _name(self, stem: str, suffix: str) -> str:
        stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", stem).strip("._") or "mesh"
        name, index = f"{stem}{suffix}", 1
        while name.lower() in self._taken:  # case-insensitive: assets may land on Windows
            index += 1
            name = f"{stem}_{index}{suffix}"
        self._taken.add(name.lower())
        return name

    def prepare(self, source: Path) -> MeshFile:
        source = source.resolve()
        if source in self.files:
            return self.files[source]
        suffix = source.suffix.lower()
        if suffix not in FORMATS:
            raise BuildError("MESH_FORMAT_UNSUPPORTED",
                             f"{source.name}: MuJoCo reads {', '.join(FORMATS)} meshes; found {source.suffix or 'no extension'} "
                             f"(convert it, e.g. export STL from the CAD tool)")
        self.dir.mkdir(parents=True, exist_ok=True)
        if suffix != ".stl":
            item = MeshFile(source, suffix[1:], [self._name(source.stem, suffix)])
            shutil.copyfile(source, self.dir / item.files[0])
        else:
            try:
                count = stl.binary_face_count(source)
                if count is not None and count <= self.max_faces:
                    item = MeshFile(source, "stl", [self._name(source.stem, ".stl")], count)
                    shutil.copyfile(source, self.dir / item.files[0])
                else:
                    faces, kind = stl.read_stl(source)
                    item = MeshFile(source, "stl" if kind == "binary" else "ascii-stl", faces=len(faces))
                    parts = stl.chunks(faces, self.max_faces)
                    for index, part in enumerate(parts):
                        stem = source.stem if len(parts) == 1 else f"{source.stem}_part{index}"
                        name = self._name(stem, ".stl")
                        note = f"{source.name}" if len(parts) == 1 else f"{source.name} faces {index * self.max_faces}+{len(part)}"
                        stl.write_stl(self.dir / name, part, header=f"twin-pipeline mjcf: {note}")
                        item.files.append(name)
            except stl.StlError as error:
                raise BuildError("MESH_UNREADABLE", str(error)) from None
        self.files[source] = item
        return item

    def add(self, stem: str, faces: np.ndarray, note: str) -> str:
        """Write a mesh the builder made (a hull, a decomposition part) as binary STL; returns its asset file name."""
        self.dir.mkdir(parents=True, exist_ok=True)
        name = self._name(stem, ".stl")
        stl.write_stl(self.dir / name, faces, header=f"twin-pipeline mjcf: {note}")
        return name


def read_triangles(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Vertices (n, 3) and triangle indices (m, 3) of an STL, OBJ or MSH file (for hulls and decomposition)."""
    suffix = path.suffix.lower()
    if suffix == ".stl":
        faces, _ = stl.read_stl(path)
        corners = faces["vertices"].reshape(-1, 3).astype(np.float64)
        return corners, np.arange(len(corners)).reshape(-1, 3)
    if suffix == ".obj":
        return _read_obj(path)
    if suffix == ".msh":
        return _read_msh(path)
    raise BuildError("MESH_FORMAT_UNSUPPORTED", f"{path.name}: cannot read {suffix} meshes")


def _read_obj(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertices, faces = [], []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            vertices.append([float(value) for value in parts[1:4]])
        elif parts[0] == "f":
            corners = []
            for part in parts[1:]:
                index = int(part.split("/")[0])
                corners.append(index - 1 if index > 0 else len(vertices) + index)
            faces += [[corners[0], corners[k], corners[k + 1]] for k in range(1, len(corners) - 1)]
    if not vertices:
        raise BuildError("MESH_UNREADABLE", f"{path.name}: expected 'v' lines in the OBJ file; found none")
    return np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64).reshape(-1, 3)


def _read_msh(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = path.read_bytes()
    nvert, nnormal, ntex, nface = (int(value) for value in np.frombuffer(raw[:16], "<i4"))
    at = 16
    vertices = np.frombuffer(raw, "<f4", 3 * nvert, at).reshape(-1, 3).astype(np.float64)
    at += 4 * (3 * nvert + 3 * nnormal + 2 * ntex)
    faces = np.frombuffer(raw, "<i4", 3 * nface, at).reshape(-1, 3).astype(np.int64)
    return vertices, faces
