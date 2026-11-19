"""STL reading, writing and lossless splitting (numpy only).

MuJoCo reads binary STL only and refuses files with more than 200 000 faces. A binary STL is an
80-byte header, a uint32 face count and 50 bytes per face (normal, three vertices, attribute), so
splitting it into chunks of whole faces keeps every triangle bit for bit.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

FACE = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
assert FACE.itemsize == 50


class StlError(ValueError):
    """The file is not a readable STL; the message says what was expected and found."""


def binary_face_count(path: Path) -> int | None:
    """The face count of a binary STL, or None when the file is not one (e.g. ASCII)."""
    size = path.stat().st_size
    if size < 84:
        return None
    with open(path, "rb") as handle:
        handle.seek(80)
        count = int(np.frombuffer(handle.read(4), "<u4")[0])
    return count if size == 84 + 50 * count else None


def read_stl(path: str | Path) -> tuple[np.ndarray, str]:
    """All faces of an STL as a FACE array, and the format it was in ('binary' or 'ascii')."""
    path = Path(path)
    count = binary_face_count(path)
    if count is not None:
        return np.fromfile(path, dtype=FACE, offset=84, count=count), "binary"
    raw = path.read_bytes()
    if raw.lstrip()[:5].lower() == b"solid":
        return _read_ascii(path, raw), "ascii"
    if len(raw) >= 84:
        count = int(np.frombuffer(raw[80:84], "<u4")[0])
        raise StlError(f"{path.name}: binary STL header says {count} faces, so the file should be "
                       f"{84 + 50 * count} bytes; it is {len(raw)} bytes (truncated or not an STL)")
    raise StlError(f"{path.name}: expected a binary or ASCII STL, found {len(raw)} bytes that are neither")


def _read_ascii(path: Path, raw: bytes) -> np.ndarray:
    tokens = raw.decode("ascii", errors="replace").split()
    lowered = [token.lower() for token in tokens]
    try:
        vertices = [[float(tokens[i + 1]), float(tokens[i + 2]), float(tokens[i + 3])]
                    for i, token in enumerate(lowered) if token == "vertex"]
        normals = [[float(tokens[i + 1]), float(tokens[i + 2]), float(tokens[i + 3])]
                   for i, token in enumerate(lowered) if token == "normal"]
    except (IndexError, ValueError) as error:
        raise StlError(f"{path.name}: ASCII STL with a vertex or normal that is not three numbers ({error})") from None
    if not vertices or len(vertices) != 3 * len(normals):
        raise StlError(f"{path.name}: ASCII STL should have 3 vertices per facet; found {len(vertices)} "
                       f"vertices for {len(normals)} facets")
    faces = np.zeros(len(normals), dtype=FACE)
    faces["normal"] = np.asarray(normals, dtype="<f4")
    faces["vertices"] = np.asarray(vertices, dtype="<f4").reshape(-1, 3, 3)
    return faces


def write_stl(path: str | Path, faces: np.ndarray, header: str = "") -> None:
    data = header.encode("ascii", errors="replace")[:80].ljust(80, b" ")
    with open(path, "wb") as handle:
        handle.write(data)
        handle.write(np.uint32(len(faces)).tobytes())
        np.ascontiguousarray(faces, dtype=FACE).tofile(handle)


def triangles(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """A FACE array from vertex positions (n, 3) and triangle indices (m, 3), with unit normals."""
    corners = np.asarray(vertices, dtype=np.float64)[np.asarray(faces)]
    normal = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    length = np.linalg.norm(normal, axis=1, keepdims=True)
    out = np.zeros(len(corners), dtype=FACE)
    out["normal"] = np.divide(normal, length, out=np.zeros_like(normal), where=length > 0)
    out["vertices"] = corners
    return out


def chunks(faces: np.ndarray, max_faces: int) -> list[np.ndarray]:
    """Consecutive runs of at most max_faces faces; together they are exactly the input."""
    return [faces[start:start + max_faces] for start in range(0, max(len(faces), 1), max_faces)]
