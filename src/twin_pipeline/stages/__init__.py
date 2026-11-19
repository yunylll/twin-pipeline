"""The stage registry. The order here is the pipeline order; it is fixed so parallel work never edits this file.

Each module exports:
  STAGES       list[Stage] it implements, in order
  SECTIONS     {top-level twin.yaml key: validator(where, value) -> list[str]} for the sections it owns
  URDF_CHECKS  [check(spec, urdf_model) -> list[str]] cross-checks of its sections against the URDF
"""

from __future__ import annotations

from . import cad, mjcf, report, smoke

MODULES = (cad, mjcf, smoke, report)


def all_stages():
    return [stage for module in MODULES for stage in module.STAGES]


def section_validators() -> dict:
    validators = {}
    for module in MODULES:
        for name, validator in module.SECTIONS.items():
            if name in validators:
                raise RuntimeError(f"twin.yaml section {name!r} is claimed by two stage modules")
            validators[name] = validator
    return validators


def urdf_checks() -> list:
    return [check for module in MODULES for check in getattr(module, "URDF_CHECKS", [])]
