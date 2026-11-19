"""What a stage is and what it gets to work with."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from .result import StageResult
from .spec import TwinSpec


@dataclass
class StageContext:
    spec: TwinSpec
    run_dir: Path
    stage_dir: Path
    results: dict[str, StageResult] = field(default_factory=dict)
    options: dict = field(default_factory=dict)

    def output(self, stage: str, key: str) -> Path | None:
        """A file an earlier stage of this run produced, or None if that stage did not pass."""
        result = self.results.get(stage)
        if result is None or not result.passed or key not in result.outputs:
            return None
        path = Path(result.outputs[key])
        return path if path.is_absolute() else (self.run_dir / path).resolve()

    def urdf(self) -> Path | None:
        """The URDF downstream stages use: the one ingest archived, else twin.yaml's urdf."""
        return self.output("ingest", "urdf") or self.spec.urdf_path

    def rel(self, path: Path | str) -> str:
        """How to store a path in result.json: relative to the run when inside it, else absolute."""
        path = Path(path).resolve()
        try:
            return path.relative_to(self.run_dir.resolve()).as_posix()
        except ValueError:
            return str(path)


@dataclass(frozen=True)
class Stage:
    name: str
    run: Callable[[StageContext], StageResult]
    description: str = ""
