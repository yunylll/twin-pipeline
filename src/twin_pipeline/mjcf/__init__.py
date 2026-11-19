"""URDF + twin.yaml to a MuJoCo model: see build.build_mjcf and docs/twin-yaml.md (meshes, collision, actuators).

Importing this package needs only the standard library; build, stl, meshes and collision need the sim
extra (numpy, mujoco), so a core install can still load every stage and validate twin.yaml.
"""

from .errors import BuildError

__all__ = ["BuildError"]
