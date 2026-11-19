"""Command line: ``twin validate``, ``twin run``, ``twin stages``."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .pipeline import EXIT_CODES, run_pipeline
from .spec import SpecError, load_spec


def _option(text: str):
    key, sep, value = text.partition("=")
    if not sep or not key:
        raise argparse.ArgumentTypeError(f"expected key=value, got {text!r}")
    return key, value


def _names(text: str) -> list[str]:
    return [name.strip() for name in text.split(",") if name.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="twin", description="Digital twin pipeline: URDF -> MuJoCo model -> checks -> report.")
    commands = parser.add_subparsers(dest="command", required=True)

    validate = commands.add_parser("validate", help="check twin.yaml (and its URDF when present) without running anything")
    validate.add_argument("spec", type=Path)

    run = commands.add_parser("run", help="run the pipeline; exit 0 succeeded, 1 failed, 2 needs review, 3 incomplete")
    run.add_argument("spec", type=Path)
    run.add_argument("--stages", type=_names, help="comma-separated subset of stages to run")
    run.add_argument("--runs-dir", type=Path, help="where runs go (default: runs/ next to twin.yaml)")
    run.add_argument("--resume", type=Path, help="continue an existing run directory")
    run.add_argument("--approve", action="append", default=[], help="approve a needs_review stage of the resumed run (repeatable)")
    run.add_argument("--option", action="append", type=_option, default=[], help="key=value passed to the stages (repeatable)")

    commands.add_parser("stages", help="list the stages in pipeline order")

    args = parser.parse_args(argv)
    if args.command == "stages":
        from .stages import all_stages
        for stage in all_stages():
            print(f"{stage.name:8}  {stage.description}")
        return 0

    try:
        spec = load_spec(args.spec)
    except SpecError as error:
        print(error, file=sys.stderr)
        return 1
    if args.command == "validate":
        note = "" if spec.urdf_path is None or spec.urdf_path.exists() else f" (URDF {spec.urdf_path} does not exist yet; not cross-checked)"
        print(f"OK: {spec.robot}{note}")
        return 0

    try:
        summary = run_pipeline(spec, runs_dir=args.runs_dir, only=args.stages, resume=args.resume,
                               approve=args.approve, options=dict(args.option))
    except ValueError as error:
        print(f"twin run: {error}", file=sys.stderr)
        return 1
    print(json.dumps({key: summary[key] for key in ("robot", "status", "stopped_at", "run_dir", "stages")}, indent=2, ensure_ascii=False))
    return EXIT_CODES[summary["status"]]


if __name__ == "__main__":
    sys.exit(main())
