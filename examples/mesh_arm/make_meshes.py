"""Regenerate the meshes of the mesh_arm example (numpy only; output is deterministic).

    python examples/mesh_arm/make_meshes.py

* base.stl       binary STL, a dome of 672 faces: over the example's meshes.max_faces (500), so it is split
* upper_arm.stl  ASCII STL, a box: MuJoCo reads binary STL only, so the mjcf stage converts it
* forearm.obj    OBJ, a hexagonal prism, referenced by a relative path instead of package://
All shapes are made up for the example; they are not a real robot.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MESHES = HERE / "mesh_arm_description" / "meshes"


def dome(radius=0.12, segments=24, rings=16):
    """A UV sphere squashed to a dome sitting on y = 0 (Y is up in this example)."""
    vertices = [(0.0, radius * 0.6 * 2, 0.0)]  # top pole
    for ring in range(1, rings - 1):
        polar = math.pi * ring / (rings - 1)
        for segment in range(segments):
            azimuth = 2 * math.pi * segment / segments
            vertices.append((radius * math.sin(polar) * math.cos(azimuth),
                             radius * 0.6 * (1 + math.cos(polar)),
                             radius * math.sin(polar) * math.sin(azimuth)))
    vertices.append((0.0, 0.0, 0.0))  # bottom pole
    bottom = len(vertices) - 1
    faces = []
    for segment in range(segments):
        nxt = (segment + 1) % segments
        faces.append((0, 1 + nxt, 1 + segment))
        for ring in range(rings - 3):
            a, b = 1 + ring * segments, 1 + (ring + 1) * segments
            faces.append((a + segment, a + nxt, b + segment))
            faces.append((a + nxt, b + nxt, b + segment))
        last = 1 + (rings - 3) * segments
        faces.append((last + segment, last + nxt, bottom))
    return np.array(vertices), np.array(faces)


def box(sx, sy, sz, y0=0.0):
    x, y, z = sx / 2, sy, sz / 2
    v = np.array([[-x, y0, -z], [x, y0, -z], [x, y0, z], [-x, y0, z],
                  [-x, y0 + y, -z], [x, y0 + y, -z], [x, y0 + y, z], [-x, y0 + y, z]])
    f = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6], [0, 4, 5], [0, 5, 1],
                  [1, 5, 6], [1, 6, 2], [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]])
    return v, f


def normals(vertices, faces):
    corners = vertices[faces]
    n = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
    return n / np.linalg.norm(n, axis=1, keepdims=True)


def write_binary_stl(path, vertices, faces):
    record = np.zeros(len(faces), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    record["n"], record["v"] = normals(vertices, faces), vertices[faces]
    with open(path, "wb") as handle:
        handle.write(b"mesh_arm example: base dome".ljust(80, b" "))
        handle.write(np.uint32(len(faces)).tobytes())
        record.tofile(handle)


def write_ascii_stl(path, vertices, faces):
    lines = ["solid upper_arm"]
    for normal, face in zip(normals(vertices, faces), faces):
        lines.append(f"  facet normal {normal[0]:.6f} {normal[1]:.6f} {normal[2]:.6f}")
        lines.append("    outer loop")
        lines += [f"      vertex {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices[face]]
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid upper_arm")
    path.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")


def write_obj(path, radius=0.03, length=0.25, sides=6):
    ring = [(radius * math.cos(2 * math.pi * k / sides), radius * math.sin(2 * math.pi * k / sides)) for k in range(sides)]
    lines = ["# mesh_arm example: forearm, a hexagonal prism along +y"]
    lines += [f"v {x:.6f} 0.000000 {z:.6f}" for x, z in ring]
    lines += [f"v {x:.6f} {length:.6f} {z:.6f}" for x, z in ring]
    for k in range(sides):
        n = (k + 1) % sides
        lines.append(f"f {k + 1} {sides + k + 1} {sides + n + 1} {n + 1}")
    lines.append("f " + " ".join(str(k + 1) for k in range(sides)))
    lines.append("f " + " ".join(str(sides + k + 1) for k in reversed(range(sides))))
    path.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")


def main():
    MESHES.mkdir(parents=True, exist_ok=True)
    write_binary_stl(MESHES / "base.stl", *dome())
    write_ascii_stl(MESHES / "upper_arm.stl", *box(0.06, 0.3, 0.06))
    write_obj(MESHES / "forearm.obj")


if __name__ == "__main__":
    main()
