# Notes for coding agents

- Setup: `python -m venv .venv`, then `.venv\Scripts\python.exe -m pip install -e ".[dev,sim]"`. Tests: `.venv\Scripts\python.exe -m pytest -q`. Run the demo: `.venv\Scripts\twin.exe run examples\arm_demo\twin.yaml`.
- This repository is public. Never add lab CAD, meshes or URDFs of real lab robots, calibration values, absolute paths from a lab machine, people's names or lab journal text. Examples use primitive shapes, generated meshes or openly licensed models only.
- Read docs/stage-contract.md before writing a stage and docs/ROADMAP.md for which files each piece of work owns. Do not edit files owned by another piece.
- A stage reports problems in its StageResult (status, errors with what / expected / found), never by raising. It writes only inside its own stage folder.
- Every value in twin.yaml that is not from CAD carries a provenance; never fill in actuator or calibration values that nobody gave you.
