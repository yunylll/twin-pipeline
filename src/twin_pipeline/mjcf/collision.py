"""Convex hulls (computed by MuJoCo itself) and convex decomposition (CoACD, optional) of a link's collision meshes."""

from __future__ import annotations

import numpy as np

from .errors import BuildError


def quat_to_matrix(quat) -> np.ndarray:
    w, x, y, z = quat
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_link_frame(vertices: np.ndarray, scale, pos, quat) -> np.ndarray:
    """Mesh vertices placed as the URDF element places them: scaled, rotated, then moved."""
    return (vertices * np.asarray(scale)) @ quat_to_matrix(quat).T + np.asarray(pos)


def convex_hull(vertices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Hull vertices and outward triangles, using MuJoCo's own hull (qhull) so contacts match what MuJoCo builds."""
    import mujoco

    points = np.unique(np.asarray(vertices, dtype=np.float32), axis=0).astype(np.float64)
    if len(points) < 4:
        raise BuildError("COLLISION_HULL_FAILED", f"a convex hull needs at least 4 distinct vertices; found {len(points)}")
    spec = mujoco.MjSpec()
    spec.add_mesh(name="hull", uservert=points.ravel().tolist())
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="hull")
    try:
        model = spec.compile()
    except ValueError as error:
        raise BuildError("COLLISION_HULL_FAILED", f"MuJoCo could not build the convex hull: {error}") from None
    stored = model.mesh_vert[model.mesh_vertadr[0]: model.mesh_vertadr[0] + model.mesh_vertnum[0]].astype(np.float64)
    faces = model.mesh_face[model.mesh_faceadr[0]: model.mesh_faceadr[0] + model.mesh_facenum[0]].astype(np.int64)
    # MuJoCo stores mesh vertices in the mesh's inertial frame; mesh_pos / mesh_quat map them back.
    rotation = quat_to_matrix(model.mesh_quat[0])
    original = stored @ rotation.T + model.mesh_pos[0]
    used = np.unique(faces)
    remap = np.full(len(original), -1)
    remap[used] = np.arange(len(used))
    return original[used], remap[faces]


def decomposition_available() -> bool:
    try:
        import coacd  # noqa: F401
    except ImportError:
        return False
    return True


def convex_decomposition(vertices: np.ndarray, faces: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Approximate convex decomposition with CoACD's defaults (seed fixed so runs repeat)."""
    import coacd

    coacd.set_log_level("error")
    parts = coacd.run_coacd(coacd.Mesh(np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64)), seed=0)
    return [(np.asarray(v, dtype=np.float64), np.asarray(f, dtype=np.int64)) for v, f in parts]
